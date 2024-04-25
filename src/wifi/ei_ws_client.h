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

#ifndef EI_WS_CLIENT_H
#define EI_WS_CLIENT_H

#include "firmware-sdk/ei_device_info_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HelloMsg,
    SampleStartMsg,
    SampleFailedMsg,
    SampleStartedMsg,
    SampleProcessingMsg,
    SampleUploadingMsg,
    SampleFinishedMsg,
    SnapshotFrameMsg,
} TxMsgType;

/**
 * @brief      Send a message to remote management servie
 * @param[in]  msg_type The message type
 * @param[in]  data     Optional additional data
 *                      For SampleFailedMsg, this is the error message
 *                      For SnapshotFrameMsg, this is the snapshot frame (base 64 encoded)
 * @return     True if message was sent successfully, false otherwise
*/
bool ei_ws_send_msg(TxMsgType msg_type, const char* data = nullptr);

/**
 * @brief      Start the websocket client thread
 * @param[in]  dev  Pointer to the device info object
 * @param[in]  handler  Callback function to call when a sample start message is received
*/
void ei_ws_client_start(EiDeviceInfo *dev, bool (*handler)(const char **, const int));

/**
 * @brief      Stop the websocket client thread
*/
void ei_ws_client_stop(void);

/**
 * @brief      Get connection status
 * @return     True if connected, false otherwise
*/
bool ei_ws_get_connection_status(void);

/**
 * @brief      Send a sample to remote management service from internal memory
 * @param[in]  address  Address of the sample in internal memory
 * @param[in]  length   Length of the sample
 * @return     True if sample was sent successfully, false otherwise
*/
bool ei_ws_send_sample(size_t address, size_t length, bool cbor = true);

#ifdef __cplusplus
};
#endif

#endif /* EI_WS_CLIENT_H */