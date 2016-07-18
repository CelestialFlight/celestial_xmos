# The TARGET variable determines what target system the application is
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling
TARGET = STARTKIT

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to
# the project name
APP_NAME = celestial_xmos

# The USED_MODULES variable lists other module used by the application.
USED_MODULES = 

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
# XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to
# xcc for the final link (mapping) stage.
XCC_FLAGS = -Os -g -Wall

# The XCORE_ARM_PROJECT variable, if set to 1, configures this
# project to create both xCORE and ARM binaries.
XCORE_ARM_PROJECT = 0

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common

EXCLUDE_FILES = 

SOURCE_DIRS = bindings/common bindings/peripherals bindings/peripherals/fatfs bindings/sensors bindings/sensors/gps bindings/sensors/ultrasonic/HCSR04 celestial/modules celestial/modules/common celestial/modules/feedback celestial/modules/feedback/angle celestial/modules/feedback/rate celestial/modules/filters celestial/modules/filters/AHRS celestial/modules/filters/IMU celestial/modules/filters/lowpass celestial/modules/filters/recursive celestial/modules/model celestial/modules/model/regression celestial/modules/peripherals celestial/modules/peripherals/fatfs celestial/modules/sensors celestial/modules/sensors/barometer celestial/modules/sensors/barometer/BMP280 celestial/modules/sensors/barometer/MS56xx celestial/modules/sensors/gps celestial/modules/sensors/gps/ultraTrackers celestial/modules/sensors/imu celestial/modules/sensors/imu/MPU celestial/modules/sensors/magnetometer celestial/modules/sensors/magnetometer/LSM303 celestial/modules/sensors/magnetometer/MAG3110 celestial/modules/sensors/tachometer celestial/modules/sensors/tachometer/rawIRTachometer celestial/modules/sensors/ultrasonic celestial/modules/sensors/ultrasonic/HCSR04 celestial/modules/sensors/ultrasonic/raw src

ARM_ONLY_DIRS = mflight/tests
