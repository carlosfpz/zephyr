# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_dac)

if(DEFINED DEBUG_PRINTF)
  zephyr_compile_definitions(DEBUG_PRINTF=${DEBUG_PRINTF})
endif()
if(DEFINED SAME_PRIO)
  zephyr_compile_definitions(SAME_PRIO)
endif()
if(DEFINED STATIC_OBJS)
  zephyr_compile_definitions(STATIC_OBJS)
endif()
if(DEFINED FORKS)
  zephyr_compile_definitions(FORKS=${FORKS})
endif()

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

