#ifndef GAME_H
#define GAME_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_SLAVE_ADDR 0x68

#define SMPLRT_DIV 0x19

#define CLOCK_SPD 400000
#define TIMEOUT_MS 1000
#define DELAY_MS 1000

#define BUFFER_SIZE_READ 2
#define REG_ADDR_READ 0x75

#define BUFFER_SIZE_WRITE 2
#define REG_ADDR_WRITE 0x32
#define DATA_WRITE 0xf0 

static const char *TAG = "I2C";

extern uint8_t write_bytes[BUFFER_SIZE_WRITE], read_buffer[BUFFER_SIZE_READ];
extern uint8_t read_bytes

esp_err_t init_i2c(void);

/**
* @brief Lee un registro del sensor
*/
esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len);

/**
* @brief Escribe un byte a un registro del sensor
*/
esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data);

#endif