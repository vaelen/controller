# SD Card Support for BeagleBone Black on RTEMS 7

This document describes the modifications needed to enable SD card support on the BeagleBone Black running RTEMS 7 with rtems-libbsd.

## Overview

The BeagleBone Black has two MMC controllers:
- **MMC0** (`sdhci_ti0`): External SD card slot
- **MMC1** (`sdhci_ti1`): Internal eMMC storage

The SD card driver stack consists of:
1. **sdhci_ti** - TI SDHCI controller driver
2. **mmc** - MMC/SD bus layer
3. **mmcsd** - Block device driver for MMC/SD cards
4. **DOSFS** - FAT filesystem support

## Required libbsd Patches

Two patches were required to make SD card support work on RTEMS. These patches work around the fact that RTEMS libbsd uses a flat driver hierarchy (drivers attach directly to simplebus) rather than the full FreeBSD device tree with clock providers.

### Patch 1: ti_sysc.c - Clock List Initialization

**File:** `freebsd/sys/arm/ti/ti_sysc.c`

**Problem:** The `ti_sysc_clock_enable()` function iterates over `sc->clk_list`, but this list was never initialized for RTEMS builds. This caused a crash when the sdhci_ti driver attempted to enable clocks.

**Solution:** Initialize the clock list to empty before the `#ifndef __rtems__` block that skips clock attachment:

```c
/* Around line 539, before #ifndef __rtems__ */

/*
 * Initialize clk_list to empty.
 * This is needed for RTEMS even when we don't attach clocks,
 * because ti_sysc_clock_enable() iterates over this list.
 */
sc->num_clocks = 0;
TAILQ_INIT(&sc->clk_list);
```

Also, in `ti_sysc_clock_enable()`, initialize `err = 0` so it returns success when the clock list is empty:

```c
static int
ti_sysc_clock_enable(device_t dev)
{
    struct ti_sysc_softc *sc = device_get_softc(dev);
    struct clk_list *clkp;
    int err = 0;  /* Initialize to success */

    TAILQ_FOREACH(clkp, &sc->clk_list, next) {
        /* ... */
    }
    return (err);
}
```

### Patch 2: ti_sdhci.c - Skip Clock Operations on RTEMS

**File:** `freebsd/sys/arm/ti/ti_sdhci.c`

**Problem:** The `ti_sdhci_hw_init()` function calls `ti_sysc_clock_enable()` on its parent device, expecting it to be a ti_sysc device. On RTEMS, sdhci_ti attaches directly to simplebus, so the parent device type is wrong and clock operations fail with "Can not find mmc_clk".

**Solution:** Skip clock operations on RTEMS and use the default 96MHz clock rate for AM335x:

```c
static int
ti_sdhci_hw_init(device_t dev)
{
    struct ti_sdhci_softc *sc = device_get_softc(dev);

#ifdef __rtems__
    /*
     * On RTEMS, the driver hierarchy is flat - sdhci_ti attaches
     * directly to simplebus, not to ti_sysc. Skip clock operations
     * as they require the proper device tree hierarchy and clock
     * providers which are not fully supported.
     * Use the default 96MHz clock rate for AM335x.
     */
    sc->baseclk_hz = 96000000;
#else
    /* Enable the controller and interface/functional clocks */
    if (ti_sysc_clock_enable(device_get_parent(dev)) != 0) {
        device_printf(dev, "Error: failed to enable MMC clock\n");
        return (ENXIO);
    }

    /* Get the base clock frequency */
    if (ti_prcm_clk_get_source_freq(MMC1_CLK + sc->mmchs_device_id,
        &sc->baseclk_hz) != 0) {
        device_printf(dev, "Error: failed to get source clock\n");
        return (ENXIO);
    }
#endif /* __rtems__ */

    /* ... rest of function ... */
}
```

## Application Configuration

### init.c - Driver References

The application must declare driver references for the SD card stack:

```c
#include <machine/rtems-bsd-config.h>

RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
SYSINIT_DRIVER_REFERENCE(ti_scm, simplebus);
SYSINIT_DRIVER_REFERENCE(sdhci_ti, simplebus);
SYSINIT_DRIVER_REFERENCE(mmcsd, mmc);
```

### init.c - RTEMS Configuration

Enable filesystem and block device support:

```c
#define CONFIGURE_FILESYSTEM_DOSFS
#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 64
```

### controller.c - Initialization Sequence

The initialization sequence is critical:

```c
#include <rtems/bdbuf.h>
#include <rtems/media.h>
#include <rtems/rtemsmmcsd.h>
#include <rtems/bdpart.h>

static bool libbsd_init(void) {
    rtems_status_code sc;

    /* 1. Initialize block device buffer */
    sc = rtems_bdbuf_init();
    if (sc != RTEMS_SUCCESSFUL) return false;

    /* 2. Initialize media manager */
    sc = rtems_media_initialize();
    if (sc != RTEMS_SUCCESSFUL) return false;

    /* 3. Start media server (handles async attach events) */
    sc = rtems_media_server_initialize(
        200,              /* Priority */
        32 * 1024,        /* Stack size */
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES
    );

    /* 4. Configure mmcsd to use media server attach mode */
    rtems_mmcsd_use_media_server();

    /* 5. Initialize libbsd (this probes and attaches drivers) */
    sc = rtems_bsd_initialize();
    if (sc != RTEMS_SUCCESSFUL) return false;

    /* 6. Wait for raw device to appear */
    /* Device will be /dev/mmcsd-0 (media server naming) */

    /* 7. Register partitions manually */
    rtems_bdpart_register_from_disk("/dev/mmcsd-0");

    /* Partition device will be /dev/mmcsd-01 */
    return true;
}
```

## Device Naming

The device naming depends on the attach mode:

| Attach Mode   | Raw Device     | Partition 1     |
|---------------|----------------|-----------------|
| Static        | /dev/mmcsd0    | /dev/mmcsd0s1   |
| Media Server  | /dev/mmcsd-0   | /dev/mmcsd-01   |

When using `rtems_mmcsd_use_media_server()`, devices use the media server naming convention with a dash separator.

## Mounting the Filesystem

After partition registration, mount the FAT filesystem:

```c
#include <rtems/dosfs.h>

int rv = mount(
    "/dev/mmcsd-01",              /* Partition device */
    "/mnt/sd",                     /* Mount point */
    RTEMS_FILESYSTEM_TYPE_DOSFS,   /* FAT filesystem */
    RTEMS_FILESYSTEM_READ_WRITE,   /* Read-write access */
    NULL                           /* No options */
);
```

## Rebuilding libbsd

After applying the patches, rebuild and reinstall libbsd:

```bash
cd ~/rtems/src/rtems-libbsd

# Clean and rebuild
./waf distclean
./waf configure --prefix=$HOME/rtems/7 \
    --rtems-bsps=arm/beagleboneblack \
    --buildset=buildset/default.ini
./waf build
./waf install
```

## Troubleshooting

### "Can not find mmc_clk" Error

This indicates the ti_sdhci.c patch is not applied. The driver is trying to get clock information from the parent device but failing.

### Data Abort in clk.c

This indicates the ti_sysc.c patch is not applied. The clock list is uninitialized and iterating over it causes a crash.

### Device Not Found

- Check that driver references are declared in init.c
- Verify libbsd initialization order
- Ensure `rtems_bdpart_register_from_disk()` is called after the raw device appears

### Mount Fails with "Invalid argument"

- The partition device may not exist - check /dev for the correct device name
- Ensure the SD card has a valid FAT filesystem
- Try mounting the raw device if the card uses superfloppy format (no partition table)

## Summary

Getting SD card support working on RTEMS requires:

1. Patching ti_sysc.c to initialize the clock list
2. Patching ti_sdhci.c to skip clock operations and use default clock rate
3. Proper driver references in the application
4. Correct initialization order: bdbuf → media → media server → mmcsd hook → libbsd → bdpart
5. Understanding the device naming convention for media server attach mode
