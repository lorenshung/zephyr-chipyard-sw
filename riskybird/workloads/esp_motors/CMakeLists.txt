# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_motors)

target_sources(app PRIVATE src/main.c)

# The wire format is a contract shared with the FPGA flight controller, so it
# lives in integration/ and both ends compile the same header rather than each
# keeping a copy that can drift. The FPGA side reaches it through
# ROSE_MOTOR_LINK_DIR (see the flight_controller workload registration).
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/zephyr/motor_link/include)

# ---- the two knobs a flight test has to be able to reach ---------------------
#
# ESP_MOTORS_MAX_DUTY_PCT is the LAST ceiling before the gate. The FPGA has its
# own (MOTOR_MAX_DUTY), this one does not trust it, and a flight-configured FPGA
# image flashed against a default 25 % image here is clamped to 25 % and looks
# from the FPGA console exactly like a thrust failure. So raising it has to be
# possible, and has to be possible through the tool the operator already uses.
#
# IT WAS NOT, AND THAT IS WHY THESE LINES EXIST. Both knobs were bare #ifndef
# defaults in src/main.c with nothing in the build reaching them, and the
# documented way to set them --
#
#   RB_EXTRA_CFLAGS=-DESP_MOTORS_MAX_DUTY_PCT=80
#
# -- is SILENTLY IGNORED on this target. tools/rb/cli.py only assembles
# EXTRA_CFLAGS/EXTRA_CPPFLAGS inside `if board is not None`, and the esp32c6
# ZephyrTarget has fpga_board=None (it is not an FPGA target), so the whole
# block is skipped and the flag never reaches a compiler. Verified: with
# RB_EXTRA_CFLAGS set, and with --pristine always, no -DESP_MOTORS_MAX_DUTY_PCT
# appears anywhere in this build's compile_commands.json. The build succeeds,
# the banner still says 25 %, and nothing anywhere reports that the flag was
# dropped -- the worst possible failure for a safety ceiling, because the
# operator believes they raised it.
#
# RB_CMAKE_ARGS is applied unconditionally by the same function, so cache
# variables DO reach this file. Declaring them here and listing them in
# target_compile_definitions is what makes -D<name> on the CMake line mean
# something; a bare -D of a name that is not listed there is discarded, which is
# the same trap the flight controller's CMakeLists.txt documents for PID_MASS_KG
# and CS_GPIO_PIN. Set them with:
#
#   RB_CMAKE_ARGS="-DESP_MOTORS_MAX_DUTY_PCT=80" ./rb build esp_motors \
#       --target esp32c6 --pristine always
#
# --pristine always because these are cache variables: once set they PERSIST in
# the build tree until it is wiped, so a later build that omits the flag keeps
# the raised ceiling. Wipe between a flight configuration and a bench one, and
# confirm against the boot banner, which prints the value this build actually
# has.
#
# Defaults match the #ifndef values in src/main.c exactly, so adding this
# changes no existing build.
set(ESP_MOTORS_MAX_DUTY_PCT 25 CACHE STRING
    "duty ceiling applied last, before the gate (percent of full scale)")
set(ESP_MOTORS_TIMEOUT_MS 120 CACHE STRING
    "cut all four motors after this long with no valid frame (ms)")

target_compile_definitions(app PRIVATE
  ESP_MOTORS_MAX_DUTY_PCT=${ESP_MOTORS_MAX_DUTY_PCT}
  ESP_MOTORS_TIMEOUT_MS=${ESP_MOTORS_TIMEOUT_MS})
