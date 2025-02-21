# /* The Clear BSD License
#  *
#  * Copyright (c) 2025 EdgeImpulse Inc.
#  * All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted (subject to the limitations in the disclaimer
#  * below) provided that the following conditions are met:
#  *
#  *   * Redistributions of source code must retain the above copyright notice,
#  *   this list of conditions and the following disclaimer.
#  *
#  *   * Redistributions in binary form must reproduce the above copyright
#  *   notice, this list of conditions and the following disclaimer in the
#  *   documentation and/or other materials provided with the distribution.
#  *
#  *   * Neither the name of the copyright holder nor the names of its
#  *   contributors may be used to endorse or promote products derived from this
#  *   software without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
#  * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  */

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