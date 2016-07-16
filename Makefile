# The TARGET variable determines what target system the application is
# compiled for. It either refers to an XN file in the source directories
# or a valid argument for the --target option when compiling
TARGET = STARTKIT

# The APP_NAME variable determines the name of the final .xe file. It should
# not include the .xe postfix. If left blank the name will default to
# the project name
APP_NAME = mFlight_xmos

# The USED_MODULES variable lists other module used by the application.
USED_MODULES = 

# The flags passed to xcc when building the application
# You can also set the following to override flags for a particular language:
# XCC_XC_FLAGS, XCC_C_FLAGS, XCC_ASM_FLAGS, XCC_CPP_FLAGS
# If the variable XCC_MAP_FLAGS is set it overrides the flags passed to
# xcc for the final link (mapping) stage.
XCC_FLAGS = -O2 -g -Wall

# The XCORE_ARM_PROJECT variable, if set to 1, configures this
# project to create both xCORE and ARM binaries.
XCORE_ARM_PROJECT = 0

# The VERBOSE variable, if set to 1, enables verbose output from the make system.
VERBOSE = 0

XMOS_MAKE_PATH ?= ../..
-include $(XMOS_MAKE_PATH)/xcommon/module_xcommon/build/Makefile.common

EXCLUDE_FILES = 

SOURCE_DIRS = bindings/common bindings/peripherals bindings/peripherals/fatfs bindings/sensors bindings/sensors/gps bindings/sensors/ultrasonic/HCSR04 mflight mflight/modules/common mflight/modules/datalogging mflight/modules/feedback mflight/modules/feedback/angle mflight/modules/feedback/rate mflight/modules/filters mflight/modules/filters/AHRS mflight/modules/filters/IMU mflight/modules/filters/lowpass mflight/modules/filters/recursive mflight/modules/model mflight/modules/peripherals mflight/modules/peripherals/fatfs mflight/modules/sensors mflight/modules/sensors/barometer/BMP280 mflight/modules/sensors/barometer/MS56xx mflight/modules/sensors/gps/ultraTrackers mflight/modules/sensors/imu/MPU mflight/modules/sensors/magnetometer/LSM303 mflight/modules/sensors/magnetometer/MAG3110 mflight/modules/sensors/tachometer/rawIRTachometer mflight/modules/sensors/ultrasonic/HCSR04 mflight/modules/sensors/ultrasonic/raw src

ARM_ONLY_DIRS = mflight/tests
