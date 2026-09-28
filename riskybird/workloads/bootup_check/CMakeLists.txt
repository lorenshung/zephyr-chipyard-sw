# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bootup_check)

# The PMW3901 driver is shared with pmw3901_test rather than copied, for the
# reason sensor_check gives: there are already two copies and a third makes a
# fix land in three places.
# The ToF array library is the shared one under integration/, referenced the
# way vl35l5cx_test references it -- the XSHUT bring-up sequence is board
# knowledge and belongs in exactly one place.
target_sources(app PRIVATE
  src/main.c
  ../pmw3901_test/src/pmw3901.c
  ${RB_ROOT}/integration/zephyr/tof/src/tof.c
  ${RB_ROOT}/integration/zephyr/camera/src/camera.c
  # Attitude watchdog. Shared rather than copied: a safety guard that
  # exists in one workload's private copy is a guard the next workload
  # silently lacks.
  ${RB_ROOT}/integration/zephyr/safety/src/motor_guard.c
)
target_include_directories(app PRIVATE
  ../pmw3901_test/src
  ${RB_ROOT}/integration/zephyr/tof/include
  ${RB_ROOT}/integration/zephyr/camera/include
  ${RB_ROOT}/integration/zephyr/safety/include
  # The HM01B0 register map is camera_bringup's, not a second copy of it, for
  # the reason camera_photo's CMakeLists gives: two copies drift silently.
  ${CMAKE_CURRENT_SOURCE_DIR}/../camera_bringup/src
)
