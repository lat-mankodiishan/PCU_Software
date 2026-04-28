cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Software_Hybrid)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Auto-pick every .c file in Core/Src that isn't already added by the
# CubeMX-generated CMakeLists. CONFIGURE_DEPENDS re-checks on every build,
# so dropping a new .c into Core/Src just builds without editing this file.
file(GLOB USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/MIDWARE/*/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/libcanard/*.c"
)
# dronecan_dsdlc emits codecs in a nested namespace tree (lat/powertrain/...,
# uavcan/protocol/...), so we recurse here. CONFIGURE_DEPENDS still forces
# re-glob each build.
file(GLOB_RECURSE CODEC_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/dronecan_codec/*.c"
)
list(REMOVE_ITEM USER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/gpio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/can.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dac.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/i2c.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/tim.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_timebase_tim.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f4xx.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES} ${CODEC_SOURCES})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/MIDWARE/FATFS_SD
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/libcanard
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/dronecan_codec/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
