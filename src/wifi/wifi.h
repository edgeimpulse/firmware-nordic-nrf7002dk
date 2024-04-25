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

#ifndef WIFI_H_
#define WIFI_H_

#ifdef __cplusplus
extern "C" {
#endif

int cmd_wifi_scan(void);
void cmd_wifi_scan_done(void);
int cmd_wifi_connect(const char *ssid, const char *psk, int security);
int cmd_wifi_disconnect(void);
int cmd_wifi_connecting(void);
int cmd_dhcp_configured(void);
bool cmd_wifi_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_H_ */