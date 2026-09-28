# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_duty)

target_sources(app PRIVATE src/main.c)

# The wire format is the contract shared with workloads/esp_motors and with the
# flight controller's offload path. All three compile the same header rather
# than keeping copies that can drift; this one can name the path directly
# because, unlike the flight controller, it lives inside this repository.
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../integration/zephyr/motor_link/include)
