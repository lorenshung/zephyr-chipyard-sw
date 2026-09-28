# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(riskybird_state)

target_sources(app PRIVATE
  src/main.c
  src/madgwick.c
)

target_sources_ifdef(CONFIG_RB_STATE_SOURCE_REPLAY app PRIVATE src/imu_replay.c)
target_sources_ifdef(CONFIG_RB_STATE_SOURCE_BMI088 app PRIVATE src/imu_bmi088.c)
target_include_directories(app PRIVATE include)
target_link_libraries(app PRIVATE m)
