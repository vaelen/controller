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

    # Common C++ flags
    common_cxxflags = '-std=c++17 -g -O2 -fno-exceptions -fno-rtti'

    # Main application
    bld(features = 'cxx cxxprogram',
        target = 'sattrack_controller.exe',
        cxxflags = common_cxxflags,
        source = ['init.cpp', 'controller.cpp', 'satellite.cpp', 'sgp4.cpp'],)

    # Test executables - SGP4 algorithm tests
    bld(features = 'cxx cxxprogram',
        target = 'test_sgp4.exe',
        cxxflags = common_cxxflags,
        source = ['test_init.cpp', 'test_sgp4.cpp', 'sgp4.cpp'],)

    # Test executables - Satellite class tests
    bld(features = 'cxx cxxprogram',
        target = 'test_satellite.exe',
        cxxflags = common_cxxflags,
        source = ['test_init.cpp', 'test_satellite.cpp', 'satellite.cpp', 'sgp4.cpp'],)

    # Test executables - Coordinate system tests
    bld(features = 'cxx cxxprogram',
        target = 'test_coordinates.exe',
        cxxflags = common_cxxflags,
        source = ['test_init.cpp', 'test_coordinates.cpp', 'satellite.cpp', 'sgp4.cpp'],)