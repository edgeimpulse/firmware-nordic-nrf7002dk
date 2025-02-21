/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "firmware-sdk/at_base64_lib.h"
#include "ei_device_nordic_nrf7002dk.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <cmath>

LOG_MODULE_REGISTER(base64, LOG_LEVEL_DBG);

bool read_encode_send(size_t address, size_t length)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *memory = dev->get_memory();
    // we are encoiding data into base64, so it needs to be divisible by 3
    const int buffer_size = 513;
    uint8_t* buffer = (uint8_t*)ei_malloc(buffer_size);
    if(buffer == nullptr) {
        LOG_ERR("Unable to allocate base64 encoder buffer");
        return false;
    }

    while (1) {
        LOG_DBG("Reading %d bytes from %d", length, address);
        size_t bytes_to_read = buffer_size;

        if (bytes_to_read > length) {
            bytes_to_read = length;
        }

        if (bytes_to_read == 0) {
            ei_free(buffer);
            return true;
        }

        if (memory->read_sample_data(buffer, address, bytes_to_read) != bytes_to_read) {
            ei_free(buffer);
            LOG_ERR("Failed to read samples memory");
            return false;
        }

        base64_encode((char *)buffer, bytes_to_read, ei_putchar);

        address += bytes_to_read;
        length -= bytes_to_read;
    }

    ei_free(buffer);
    return true;
}
