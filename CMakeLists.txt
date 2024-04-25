#
# Copyright (c) 2024 Edge Impulse
#

cmake_minimum_required(VERSION 3.13.1)

set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/pm_static_${BOARD}.yml)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This firmware uses X-Nucleo IKS02A1 shield
set(SHIELD x_nucleo_iks02a1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("firmware-nordic-nrf7002dk"
          VERSION 0.1)

# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# Use hardware acceleration for DSP and Neural Network code
# You'll need to disable these on non-Arm cores
add_definitions(-DEIDSP_USE_CMSIS_DSP=1
                -DEIDSP_LOAD_CMSIS_DSP_SOURCES=1
                -DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DEIDSP_QUANTIZE_FILTERBANK=0
                -DARM_MATH_LOOPUNROLL
                -DMBEDTLS_PLATFORM_ZEROIZE_ALT
                )

# Add all required source files
add_subdirectory(ei-model/edge-impulse-sdk/cmake/zephyr)
add_subdirectory(firmware-sdk)
add_subdirectory(src/wifi)
add_subdirectory(src/inference)
add_subdirectory(src/sensors)
add_subdirectory(src)

# Include directories (everything in the SDK is already included here)
target_include_directories(app PRIVATE .)
target_include_directories(app PRIVATE ei-model)

# Use GLOB to include model files, because model file names differes whether model is EON compiled or not
RECURSIVE_FIND_FILE(MODEL_FILES ei-model/tflite-model "*.cpp")
target_sources(app PRIVATE ${MODEL_FILES})