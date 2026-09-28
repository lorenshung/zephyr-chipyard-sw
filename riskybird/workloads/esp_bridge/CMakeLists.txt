# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_bridge)

target_sources(app PRIVATE src/main.c)

# The motor duty-frame wire format is a contract shared with the FPGA flight
# controller and with workloads/esp_motors, so it lives in integration/ and every
# end compiles the same header rather than each keeping a copy that can drift.
#
# Added unconditionally even though only the `motors` build includes the header:
# the include path is harmless in the default build (nothing includes it, and
# HAVE_MOTORS compiles the whole motor half out), and making it conditional would
# mean the two builds disagree about their include paths, which is exactly the
# kind of difference that turns "works in the bridge build" into a link error in
# the one that drives motors.
set(RB_MOTOR_LINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/zephyr/motor_link/include)
if(NOT EXISTS ${RB_MOTOR_LINK_DIR}/riskybird/motor_link.h)
  message(FATAL_ERROR
    "esp_bridge: the shared motor-link header is missing at ${RB_MOTOR_LINK_DIR}. "
    "It comes from integration/zephyr/motor_link/ -- the same directory "
    "flight_controller --mode esp-motors passes as ROSE_MOTOR_LINK_DIR.")
endif()
target_include_directories(app PRIVATE ${RB_MOTOR_LINK_DIR})

# The motors build's duty ceiling, reachable from the build line. Same reason as
# workloads/esp_motors/CMakeLists.txt: RB_EXTRA_CFLAGS is dropped on this target
# (fpga_board=None), so a bare #ifndef in src/main.c could only ever be 25 %.
# Declared here, -DESP_LINK_MAX_DUTY_PCT=80 on the CMake line reaches the
# compiler. A cache variable: it persists in the build tree, so build a flight
# image into its own directory or with a pristine build, and read the value back
# from the boot banner ("duty ceiling NN%"). The default matches src/main.c.
set(ESP_LINK_MAX_DUTY_PCT 25 CACHE STRING
    "motors build: collective duty ceiling applied at the gate (percent)")
target_compile_definitions(app PRIVATE ESP_LINK_MAX_DUTY_PCT=${ESP_LINK_MAX_DUTY_PCT})
