#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define CLOCK_SPD 400000
#define I2C_SLAVE_ADDR 0x68

esp_err_t init_i2c(void);

esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data);

#endif