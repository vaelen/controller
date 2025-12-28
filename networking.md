# Networking on BeagleBone Black with RTEMS libbsd

This document describes the changes required to enable Ethernet networking on the BeagleBone Black with RTEMS 7 and the libbsd TCP/IP stack.

## Overview

The BeagleBone Black uses the TI AM335x SoC which includes a CPSW (Common Platform Switch) Ethernet controller. The RTEMS libbsd port includes a driver for this controller, but as of late 2024/early 2025, several files were missing from the build configuration that prevented it from working with Linux-derived device trees.

## Problem Description

The cpsw Ethernet driver requires the **syscon** (system controller) framework to read the MAC address from the AM335x control module. The driver expects to find a syscon device registered for the `scm_conf` node in the device tree.

Two issues prevented this from working:

1. **Missing source files**: The `ti_scm_syscon.c` and `syscon_generic.c` files were not included in the libbsd build, so no syscon driver was available.

2. **Driver hierarchy mismatch**: The `ti_scm_syscon` driver was only registered to attach to `simplebus` parent devices. However, in Linux device trees, `scm_conf` is a child of `ti_scm`, not `simplebus`.

## Changes to libbsd

All changes are relative to the rtems-libbsd source directory (e.g., `~/rtems/src/rtems-libbsd`).

### 1. Add source files to libbsd.py

Edit `libbsd.py` and add these files to the `dev_net` module's `addKernelSpaceSourceFiles` list (around line 1495):

```python
'sys/arm/ti/ti_scm_syscon.c',
'sys/dev/extres/syscon/syscon_generic.c',
```

Add to the base module's `addRTEMSKernelSourceFiles` list (around line 179):

```python
'local/clkdev_if.c',
```

### 2. Create ti_scm_syscon.c

Copy the file from `freebsd-org/` to `freebsd/` with RTEMS modifications:

```bash
cp freebsd-org/sys/arm/ti/ti_scm_syscon.c freebsd/sys/arm/ti/
```

Edit `freebsd/sys/arm/ti/ti_scm_syscon.c`:

1. Add RTEMS kernel space header at the top:
```c
#include <machine/rtems-bsd-kernel-space.h>
```

2. Fix the include paths for generated headers (around line 50):
```c
#ifdef __rtems__
#include <rtems/bsd/local/syscon_if.h>
#else
#include "syscon_if.h"
#endif
#include <dev/extres/syscon/syscon.h>
#ifdef __rtems__
#include <rtems/bsd/local/clkdev_if.h>
#else
#include "clkdev_if.h"
#endif
```

3. Add driver module registration for ti_scm parent (around line 298):
```c
EARLY_DRIVER_MODULE(ti_scm_syscon, simplebus, ti_scm_syscon_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
/* Also attach to ti_scm parent for Linux device trees where scm_conf is a child of scm */
EARLY_DRIVER_MODULE(ti_scm_syscon, ti_scm, ti_scm_syscon_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
```

### 3. Modify syscon_generic.c

Edit `freebsd/sys/dev/extres/syscon/syscon_generic.c`:

1. Add RTEMS kernel space header at the top:
```c
#include <machine/rtems-bsd-kernel-space.h>
```

2. Fix the syscon_if.h include (around line 51):
```c
#ifdef __rtems__
#include <rtems/bsd/local/syscon_if.h>
#else
#include "syscon_if.h"
#endif
```

### 4. Create clkdev_if interface files

Create `rtemsbsd/include/rtems/bsd/local/clkdev_if.h`:

```c
#ifndef _clkdev_if_h_
#define _clkdev_if_h_

#include <machine/bus.h>

extern struct kobjop_desc clkdev_write_4_desc;
typedef int clkdev_write_4_t(device_t dev, bus_addr_t addr, uint32_t val);

static __inline int CLKDEV_WRITE_4(device_t dev, bus_addr_t addr, uint32_t val)
{
    kobjop_t _m;
    int rc;
    KOBJOPLOOKUP(((kobj_t)dev)->ops,clkdev_write_4);
    rc = ((clkdev_write_4_t *) _m)(dev, addr, val);
    return (rc);
}

extern struct kobjop_desc clkdev_read_4_desc;
typedef int clkdev_read_4_t(device_t dev, bus_addr_t addr, uint32_t *val);

static __inline int CLKDEV_READ_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
    kobjop_t _m;
    int rc;
    KOBJOPLOOKUP(((kobj_t)dev)->ops,clkdev_read_4);
    rc = ((clkdev_read_4_t *) _m)(dev, addr, val);
    return (rc);
}

extern struct kobjop_desc clkdev_modify_4_desc;
typedef int clkdev_modify_4_t(device_t dev, bus_addr_t addr,
                              uint32_t clear_mask, uint32_t set_mask);

static __inline int CLKDEV_MODIFY_4(device_t dev, bus_addr_t addr,
                                    uint32_t clear_mask, uint32_t set_mask)
{
    kobjop_t _m;
    int rc;
    KOBJOPLOOKUP(((kobj_t)dev)->ops,clkdev_modify_4);
    rc = ((clkdev_modify_4_t *) _m)(dev, addr, clear_mask, set_mask);
    return (rc);
}

extern struct kobjop_desc clkdev_device_lock_desc;
typedef void clkdev_device_lock_t(device_t dev);

static __inline void CLKDEV_DEVICE_LOCK(device_t dev)
{
    kobjop_t _m;
    KOBJOPLOOKUP(((kobj_t)dev)->ops,clkdev_device_lock);
    ((clkdev_device_lock_t *) _m)(dev);
}

extern struct kobjop_desc clkdev_device_unlock_desc;
typedef void clkdev_device_unlock_t(device_t dev);

static __inline void CLKDEV_DEVICE_UNLOCK(device_t dev)
{
    kobjop_t _m;
    KOBJOPLOOKUP(((kobj_t)dev)->ops,clkdev_device_unlock);
    ((clkdev_device_unlock_t *) _m)(dev);
}

#endif /* _clkdev_if_h_ */
```

Create `rtemsbsd/local/clkdev_if.c` with the default method implementations (see the full file in the repository).

### 5. Optional: Patch if_cpsw.c for broader device tree compatibility

The cpsw driver expects to find the syscon via `/opp-table` node's `syscon` property, which exists in FreeBSD device trees but not Linux ones. For Linux device trees, add fallback paths in `freebsd/sys/arm/ti/cpsw/if_cpsw.c` around line 1048:

```c
syscon = NULL;
opp_table = OF_finddevice("/opp-table");
if (opp_table != -1 && OF_hasprop(opp_table, "syscon")) {
    syscon_get_by_ofw_property(dev, opp_table, "syscon", &syscon);
}
if (syscon == NULL) {
    phandle_t scm_conf;
    /* Try common device tree paths for the control module syscon */
    scm_conf = OF_finddevice("/ocp/l4_wkup@44c00000/scm@210000/scm_conf@0");
    if (scm_conf == -1)
        scm_conf = OF_finddevice("/ocp/scm/scm_conf");
    if (scm_conf == -1)
        scm_conf = OF_finddevice("/ocp/l4_wkup/scm/scm_conf");
    if (scm_conf != -1) {
        syscon_get_by_ofw_node(dev, scm_conf, &syscon);
    }
}
if (syscon == NULL) {
    device_printf(dev, "Failed to find syscon node\n");
    cpswp_detach(dev);
    return (ENXIO);
}
```

## Building libbsd with Networking

### 1. Configure the buildset

Use or create a buildset INI file that enables networking. The `default.ini` buildset has most networking features enabled. Key modules:

```ini
[modules]
dev_net = on      # Network device drivers including cpsw
net = on          # Core networking
netinet = on      # IPv4
netinet6 = on     # IPv6
dhcpcd = on       # DHCP client daemon
```

### 2. Build libbsd

```bash
cd ~/rtems/src/rtems-libbsd

# Configure for BeagleBone Black
./waf configure --prefix=$HOME/rtems/7 \
    --rtems-bsps=arm/beagleboneblack \
    --buildset=buildset/default.ini

# Build
./waf

# Install
./waf install
```

## Application Configuration

### Driver References in init.c

Your RTEMS application must include driver references to ensure the network drivers are linked from libbsd.a:

```c
#include <rtems/bsd/bsd.h>
#include <bsp/nexus-devices.h>  /* Or define manually */

/* Device tree and bus drivers */
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);

/* TI control module drivers */
SYSINIT_DRIVER_REFERENCE(ti_scm, simplebus);
SYSINIT_DRIVER_REFERENCE(ti_scm_syscon, ti_scm);
SYSINIT_DRIVER_REFERENCE(ti_sysc, simplebus);

/* MMC/SD card drivers */
SYSINIT_DRIVER_REFERENCE(sdhci_ti, simplebus);
SYSINIT_DRIVER_REFERENCE(mmcsd, mmc);

/* Ethernet drivers */
SYSINIT_DRIVER_REFERENCE(cpswss, simplebus);
SYSINIT_DRIVER_REFERENCE(cpsw, cpswss);
SYSINIT_DRIVER_REFERENCE(ukphy, miibus);
```

### Network Initialization Code

Initialize the network in your application:

```c
#include <rtems/bsd/bsd.h>
#include <rtems/dhcpcd.h>

/* Initialize libbsd */
rtems_bsd_initialize();

/* Wait for network interface to appear */
/* The interface name is "cpsw0" */

/* Start DHCP client */
rtems_dhcpcd_start(NULL);

/* Or configure static IP using standard BSD socket APIs */
```

### Linking

Link your application with the BSD library and math library:

```python
# In wscript
bld.program(
    ...
    lib = ['bsd', 'm'],
    ...
)
```

## Device Tree Requirements

The BeagleBone Black requires a device tree blob (DTB) to be loaded by U-Boot. The DTB must contain:

- CPSW Ethernet controller node with `compatible = "ti,cpsw"` or `"ti,am335x-cpsw"`
- Control module syscon node with `compatible = "syscon"`
- Status set to `"okay"` for the Ethernet nodes

Linux-derived device trees (e.g., from `/boot/dtbs/` on a Debian BeagleBone image) work with the patches described above.

## Troubleshooting

### "Failed to find syscon node"

The cpsw driver cannot find the syscon device. Check:
1. Is `ti_scm_syscon` driver attached? Look for it in boot messages.
2. Is the device tree path correct? The path varies between device tree versions.
3. Is `SYSINIT_DRIVER_REFERENCE(ti_scm_syscon, ti_scm)` in your init.c?

### "Network interface cpsw0 not found"

The cpsw driver failed to attach. Check:
1. Is `cpswss` driver attached? Look for "3-port Switch Ethernet Subsystem" in boot messages.
2. Are all driver references present in init.c?
3. Is the device tree's Ethernet node status set to "okay"?

### No DHCP response

1. Check physical Ethernet connection
2. Verify `dhcpcd = on` in the libbsd buildset
3. Check that the interface is up: look for "link state changed to UP" messages

## References

- [RTEMS libbsd documentation](https://docs.rtems.org/)
- [FreeBSD cpsw driver](https://github.com/freebsd/freebsd-src/blob/master/sys/arm/ti/cpsw/if_cpsw.c)
- [AM335x Technical Reference Manual](https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf)
