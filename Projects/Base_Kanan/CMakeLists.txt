# Copyright (c) 2024 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET MBED_PROJECT)  # CHANGE `MBED_PROJECT` FOR RENAMING PROJECT'S .bin

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH} ${CMAKE_CURRENT_BINARY_DIR}/mbed-os)

add_executable(${APP_TARGET}
    main.cpp
    #======================= INCLUDE CUSTOM LIBRARIES (.cpp) =======================
    ../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.cpp
    ../../KRAI_library/Motor/Motor.cpp
    ../../KRAI_library/encoderKRAI/encoderKRAI.cpp
    # PID SEMENTARA
    ../../KRAI_library/MovingAverage/MovingAverage.cpp
    ../../KRAI_library/MiniPID/MiniPID.cpp
    ../../KRAI_library/ADRC_V2/ADRC_V2.cpp
    #-------------------------------------------------------------------------------
)

target_link_libraries(${APP_TARGET} mbed-os)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()