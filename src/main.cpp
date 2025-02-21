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

#include "ei_at_handlers.h"
#include "ei_device_nordic_nrf7002dk.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "sensors/ei_inertial_sensor.h"
#include "wifi/wifi.h"
#include "wifi/ei_ws_client.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <nrfx_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

int main(void)
{
    EiDeviceNRF7002DK *dev = static_cast<EiDeviceNRF7002DK*>(EiDeviceInfo::get_device());
    ATServer *at;
    int err = 0;

    /* output of printf is output immediately without buffering */
    setvbuf(stdout, NULL, _IONBF, 0);

    // /* Switch CPU core clock to 128 MHz */
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

    /* Initialize board uart */
    if(uart_init() != 0) {
        LOG_ERR("Init uart on board error occured\r\n");
    }

    /* Setup the accelerometer sensor */
    if(ei_inertial_init() == false) {
        LOG_ERR("Light sensor communication error occured");
    }

    // NOTE: This is required so the device ID that is based on BLE MAC is properly initialized
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE init failed (err %d)\n", err);
    }

    //we have to do this later because the BLE stack is not available on boot
    dev->init_device_id();

    ei_printf("Hello from Edge Impulse\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    at = ei_at_init();

    char ssid[128] = { 0 };
    char password[128] = { 0 };
    int security = 0;

    dev->get_wifi_config(ssid, password, &security);

    if (strlen(ssid) != 0) {
        ei_printf("Connecting to WiFi %s\n", ssid);
        cmd_wifi_connect(ssid, password, security);
        // waithing to connect to wifi
        if(cmd_wifi_connecting() != 0) {
            LOG_ERR("Failed to connect to WiFi\n");
            ei_printf("ERR: Failed to connect to WiFi\n");
        }
        else {
            // waiting to get dhcp config
            if(cmd_dhcp_configured() != 0) {
                LOG_ERR("Failed to configure DHCP\n");
                ei_printf("ERR: Failed to configure DHCP\n");
            }
            else {
                ei_sleep(300);
                LOG_INF("WiFi connected\n");
                ei_printf("WiFi connected\n");
                ei_ws_client_start(dev, nullptr);
            }
        }
    }

    at->print_prompt();
    LOG_INF("Entering infinite loop\n");
    while(1) {
        char data = uart_getchar();

        while(data != 0xFF) {
            if(is_inference_running() && data == 'b') {
                ei_stop_impulse();
                at->print_prompt();
                continue;
            }
            dev->set_serial_channel(UART);
            at->handle(data);
            data = uart_getchar();
        }
        ei_sleep(1);
    }
}
