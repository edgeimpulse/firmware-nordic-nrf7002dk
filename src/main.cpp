/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
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
