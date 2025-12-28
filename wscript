#
# RTEMS SatTrack Controller wscript
#
from __future__ import print_function

rtems_version = "7"

try:
    import rtems_waf.rtems as rtems
except:
    print('error: no rtems_waf submodule')
    import sys
    sys.exit(1)

def init(ctx):
    rtems.init(ctx, version = rtems_version, long_commands = True)

def bsp_configure(conf, arch_bsp):
    pass

def options(opt):
    rtems.options(opt)

def configure(conf):
    rtems.configure(conf, bsp_configure = bsp_configure)

def build(bld):
    rtems.build(bld)

    # Common C flags
    common_cflags = '-g -O2'

    # Main application
    # Note: console-config.c must be listed first to override the BSP's console configuration
    bld(features = 'c cprogram',
        target = 'controller.exe',
        cflags = common_cflags,
        linkflags = ['-Wl,--allow-multiple-definition'],
        lib = ['bsd', 'm'],
        source = ['console-config.c', 'init.c', 'controller.c', 'log.c', 'sgp4.c', 'nmea.c', 'config.c'],)

