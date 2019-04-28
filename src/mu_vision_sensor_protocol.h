// Copyright 2018 Morpx Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_PROTOCOL_H_
#define ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_PROTOCOL_H_

#include <stdlib.h>
#include <stdint.h>
#include "mu_vision_sensor_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MU vision sensor hardware interface function.
 * Get serial data number.
 * Return: byte number in serial buffer.
 */
typedef uint32_t (*MuVsUartAvailable)(void);
/*
 * MU vision sensor hardware interface function.
 * Read serial data.
 * Input :temp: unsigned char buffer pointer
 *        length: max buffer length
 * Return: actual buffer length,if return number less then (length),it means read time out.
 */
typedef uint32_t (*MuVsUartRead)(uint8_t* temp, uint8_t length);

/*
 * MU vision sensor hardware interface function.
 * Write serial data.
 * Input :temp: write unsigned char buffer pointer
 *        length: max buffer length
 * Return: actual buffer length,if return number less then (length),it means serial port write buffer is full.
 */
typedef uint32_t (*MuVsUartWrite)(uint8_t* temp, uint8_t length);

typedef struct {
  MuVsUartAvailable uart_avalible;
  MuVsUartRead uart_read;
  MuVsUartWrite uart_write;
} Uart;

mu_err_t mu_vs_uart_init(MuVsUartAvailable func_uart_available,
                         MuVsUartRead func_uart_read,
                         MuVsUartWrite func_uart_write);
mu_err_t mu_vs_uart_set(const uint8_t mu_address, const uint8_t reg_address, const uint8_t value);
mu_err_t mu_vs_uart_get(const uint8_t mu_address, const uint8_t reg_address, uint8_t* value);
mu_err_t mu_vs_uart_read(uint8_t* mu_address,
                         MuVsMessageVisionType* vision_type,
                         MuVsVisionState* vision_state);

#ifdef __cplusplus
}
#endif

#endif /* ARDUINO_LIB_MUVISIONSENSOR_SRC_MU_VISION_SENSOR_PROTOCOL_H_ */
