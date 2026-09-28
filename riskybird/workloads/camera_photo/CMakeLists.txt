# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(camera_photo)

target_sources(app PRIVATE src/main.c)

# The HM01B0 register map is camera_bringup's, not a second copy of it. Two
# copies of a sensor's register addresses drift, and the drift is silent.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../camera_bringup/src)
