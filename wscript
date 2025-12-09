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
    bld(features = 'cxx cxxprogram',
        target = 'sattrack_controller.exe',
        cxxflags = '-std=c++17 -g -O2 -fno-exceptions -fno-rtti',
        source = ['init.cpp', 'controller.cpp', 'satellite.cpp', 'sgp4.cpp'],)