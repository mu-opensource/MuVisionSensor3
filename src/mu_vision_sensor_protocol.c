/*
 * mu_vision_sensor_protocol.c
 *
 *  Created on: 2018年8月7日
 *      Author: ysq
 */

#include "mu_vision_sensor_protocol.h"

static MuVsUartAvailable uart_available = NULL;
static MuVsUartRead uart_read = NULL;
static MuVsUartWrite uart_write = NULL;

static uint8_t sum_check(uint8_t* buf, uint8_t len) {
  uint32_t sum = 0;
  for (int i = 0; i < len; i++) {
    sum += buf[i];
  }
  return (uint8_t)(sum&0xFF);
}
static mu_err_t get_protocol_head(uint8_t* buf, uint8_t mu_address) {
  uint32_t read_len;
  uint32_t read_responce_timeout_count = 0;
  do {
    do {
      //TODO may get data from other serial device.
      read_len = uart_read(buf, 1);
      if (read_len == 0) return SERVER_RESPONSE_TIMEOUT;
    } while (buf[0] != MU_PROTOCOL_START);
    read_len = uart_read(&buf[1], 2);
    if (read_len != 2) return SERVER_RESPONSE_TIMEOUT;
    read_responce_timeout_count++;
    if (read_responce_timeout_count > 2) return SERVER_RESPONSE_TIMEOUT;
  } while(buf[2] != mu_address);
  return MU_OK;
}
static mu_err_t get_protocol_body(uint8_t* buf) {
  uint32_t read_len;
  read_len = uart_read(&buf[3], buf[1]-3);
  if (read_len != (uint8_t)(buf[1]-3)) return SERVER_RESPONSE_TIMEOUT;
  if (buf[buf[1]-1] != 0xED) return MU_ERROR_FAIL;
  if (buf[buf[1]-2] != sum_check(buf, buf[1]-2)) {
    return MU_ERROR_CHECK_SUM;
  }
  return MU_OK;
}

mu_err_t mu_vs_uart_init(MuVsUartAvailable func_uart_available,
                         MuVsUartRead func_uart_read,
                         MuVsUartWrite func_uart_write) {
  uart_available = func_uart_available;
  uart_read = func_uart_read;
  uart_write = func_uart_write;
  return MU_OK;
}

//| Start | Length | MU_Address | Command | Register | Data | Check | End |
//| 0xFF  |        |            |  0x01   |          |      |       | 0xED|
mu_err_t mu_vs_uart_set(const uint8_t mu_address, const uint8_t reg_address, const uint8_t value) {
  uint8_t data_buf[8] = {0};
  data_buf[0] = MU_PROTOCOL_START;
  data_buf[1] = 0x08;
  data_buf[2] = mu_address;
  data_buf[3] = MU_PROTOCOL_COMMADN_SET;
  data_buf[4] = reg_address;
  data_buf[5] = value;
  data_buf[6] = sum_check(data_buf, 6);
  data_buf[7] = MU_PROTOCOL_END;
  uint32_t len = sizeof(data_buf);
  uint32_t write_len = uart_write(data_buf, len);
  if (write_len < len) return CLIENT_WRITE_TIMEOUT;

  //Read response
  uint32_t read_responce_count = 0;
  mu_err_t err;
  while(1) {
    if(++read_responce_count > 3) {
      err = err==MU_OK ? SERVER_RESPONSE_TIMEOUT:err;
      return err;
    }
    err = get_protocol_head(data_buf, mu_address);
    if (err != MU_OK) return err;
    if (data_buf[1] > 8) {
      err = MU_ERROR_COMMAND;
      continue;
    }
    err = get_protocol_body(data_buf);
    if (err != MU_OK)
      return err;
    if (data_buf[3] == MU_ERROR_OK &&
        data_buf[4] == MU_PROTOCOL_COMMADN_SET &&
        data_buf[5] == reg_address) {
      return MU_OK;
    } else if (data_buf[1] == 6) {
      return data_buf[3];
    }
  }
}

mu_err_t mu_vs_uart_get(const uint8_t mu_address, const uint8_t reg_address, uint8_t* value) {
  uint8_t data_buf[8] = {0};
  data_buf[0] = MU_PROTOCOL_START;
  data_buf[1] = 0x07;
  data_buf[2] = mu_address;
  data_buf[3] = MU_PROTOCOL_COMMADN_GET;
  data_buf[4] = reg_address;
  data_buf[5] = sum_check(data_buf, 6);
  data_buf[6] = MU_PROTOCOL_END;
  uint32_t len = 7;
  uint32_t write_len = uart_write(data_buf, len);
  if (write_len < len) return CLIENT_WRITE_TIMEOUT;

  //Read response
  uint32_t read_responce_count = 0;
  mu_err_t err;
  while(1) {
    if(++read_responce_count > 3) {
      err = err==MU_OK ? SERVER_RESPONSE_TIMEOUT:err;
      return err;
    }
    err = get_protocol_head(data_buf, mu_address);
    if (err == SERVER_RESPONSE_TIMEOUT) return err;
    if (data_buf[1] > 8) {
      err = MU_ERROR_COMMAND;
      continue;
    }
    err = get_protocol_body(data_buf);
    if (err == SERVER_RESPONSE_TIMEOUT) return err;
    if (data_buf[3] == MU_ERROR_OK && data_buf[4] == MU_PROTOCOL_COMMADN_GET) {
      *value = data_buf[5];
      return MU_OK;
    } else if (data_buf[1] == 6) {
      return data_buf[3];
    }
  }
//  return MU_ERROR_FAIL;
}

mu_err_t mu_vs_uart_read(uint8_t* mu_address,
                         MuVsMessageVisionType* vision_type,
                         MuVsVisionState* vision_state) {
  uint8_t data_buf[23] = {0};
  mu_err_t err;
  uint32_t read_len;
//  err = get_protocol_head(data_buf, mu_address);
//  if (err != MU_OK) return err;
  do {
    //TODO may get data from other serial device.
    read_len = uart_read(data_buf, 1);
    if (read_len == 0) return SERVER_RESPONSE_TIMEOUT;
  } while (data_buf[0] != MU_PROTOCOL_START);
  read_len = uart_read(&data_buf[1], 2);
  if (read_len != 2) return SERVER_RESPONSE_TIMEOUT;

  if (data_buf[1] > 23) return MU_ERROR_COMMAND;
  err = get_protocol_body(data_buf);
  if (err != MU_OK) return err;
  if (data_buf[3] != MU_PROTOCOL_MESSAGE) return MU_ERROR_COMMAND;
  //Assignment
  *mu_address = data_buf[2];
  *vision_type = data_buf[4];
  vision_state->detect = data_buf[5];
  if (data_buf[5] == 0) return MU_OK;
  for (uint8_t i = 0; i < data_buf[5]; i++) {
    vision_state->detect = data_buf[5];
    vision_state->vision_result[0].x_value = data_buf[6+5*i];
    vision_state->vision_result[0].y_value = data_buf[7+5*i];
    vision_state->vision_result[0].width = data_buf[8+5*i];
    vision_state->vision_result[0].height = data_buf[9+5*i];
    vision_state->vision_result[0].color = data_buf[10+5*i];
  }

  return MU_OK;
}






