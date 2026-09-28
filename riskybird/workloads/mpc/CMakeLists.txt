# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(riskybird_mpc CXX)

if(NOT DEFINED RB_ROOT)
  get_filename_component(RB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()

if(RB_BOARD STREQUAL "rose-spike")
  if(NOT EXISTS "${RB_ROOT}/backends/RoSE/soc/sw/zephyr-rose/zephyr/module.yml")
    message(FATAL_ERROR "RoSE Zephyr module is missing")
  endif()
endif()

set(TINYMPC_DIR
  "${RB_ROOT}/backends/zephyr-chipyard-sw/samples/drone_control/tinympc")
if(NOT EXISTS "${TINYMPC_DIR}/external/admm/include/admm.hpp")
  message(FATAL_ERROR
    "Accelerated-TinyMPC is missing at ${TINYMPC_DIR}. "
    "Initialize the zephyr-chipyard-sw drone_control submodule.")
endif()

# The first FPGA workload targets the scalar Rocket present on both boards.
set(USE_CPU ON CACHE BOOL "Use scalar CPU matlib" FORCE)
set(USE_RVV OFF CACHE BOOL "Disable RVV" FORCE)
set(USE_RVVU OFF CACHE BOOL "Disable RVV unrolling" FORCE)
set(USE_HANDOPT OFF CACHE BOOL "Disable RVV hand optimization" FORCE)
set(MATLIB_EXTERNAL_BUILD ON CACHE BOOL "Zephyr owns compiler flags" FORCE)
include("${TINYMPC_DIR}/external/matlib/matlib_header.cmake")

# The KU040 shell deliberately uses a 32 KiB on-chip scratchpad. Preserve the
# full ten-step controller on Arty100T and use a three-step receding horizon on
# KU040 so the exact same model and cost matrices fit in physical memory.
if(RB_BOARD STREQUAL "ku040")
  target_compile_definitions(app PRIVATE NHORIZON=3)
endif()

target_sources(app PRIVATE
  src/main.cpp
  "${TINYMPC_DIR}/external/admm/src/admm_rvv.cpp"
)

target_include_directories(app PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${TINYMPC_DIR}/external/admm/include"
  "${TINYMPC_DIR}/external/matlib/include"
  "${TINYMPC_DIR}/examples"
)

target_compile_options(app PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
  $<$<COMPILE_LANGUAGE:CXX>:-include>
  $<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/rb_tinympc_compat.h>
  -fno-tree-vectorize
  -fno-tree-loop-vectorize
  -fno-tree-slp-vectorize
)
target_link_libraries(app PRIVATE m)
