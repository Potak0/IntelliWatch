#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := IntelliWatch

EXTRA_COMPONENT_DIRS := components/ahrs
						components/mpu9250
						components/lvgl_esp32_drivers
						components/ad1115
						components/BH1750
						components/BQ25896
						components/MAX30102
						components/DRV2605
						components/pictures
include $(IDF_PATH)/make/project.mk

