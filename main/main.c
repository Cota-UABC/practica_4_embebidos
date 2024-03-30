#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
//#include "i2c.h"

#define I2C_SLAVE_ADDR 0x68
#define CLOCK_SPD 400000

#define REG_ADDR_WRITE 0x33
#define BUFFER_SIZE_READ 10

static const char *TAG = "I2C";

esp_err_t init_i2c(void);
esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data);


esp_err_t init_i2c(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .scl_pullup_en = true,
        .sda_pullup_en = true,
        .master.clk_speed = CLOCK_SPD,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LEVEL1));
  
    return ESP_OK;
}

esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    ESP_ERROR_CHECK(
            i2c_master_write_read_device(I2C_NUM_0, 
                                        I2C_SLAVE_ADDR,
                                        &reg_addr,
                                        1,
                                        data,
                                        len,
                                        pdMS_TO_TICKS(2000)));

    ESP_LOGI(TAG, "Addr read: 0x%x",reg_addr);
    //ESP_LOG_BUFFER_HEX(TAG, read_buffer, len);
    for(int i=0;i<len;i++)
    {
        ESP_LOGI(TAG, "   Data: 0x%x",data[i]);
    }

    return ESP_OK;
}

esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t *write_bytes = (uint8_t *) malloc(2);
    write_bytes[0] = reg_addr;
    write_bytes[1] = data;

    ESP_ERROR_CHECK(
            i2c_master_write_to_device(I2C_NUM_0, 
                                        I2C_SLAVE_ADDR,
                                        (const uint8_t*)&write_bytes,
                                        2,
                                        pdMS_TO_TICKS(2000)));

    ESP_LOGI(TAG, "Bytes written:  ADRR: 0x%x Data: 0x%x",write_bytes[0],write_bytes[1]);

    free(write_bytes);

    return ESP_OK;
}

void app_main(void)
{
    uint8_t read_buffer[BUFFER_SIZE_READ];

    ESP_ERROR_CHECK(init_i2c());

    while(true)
    {

        ESP_ERROR_CHECK( device_register_write_byte(REG_ADDR_WRITE, 0xf6) );

        vTaskDelay(pdMS_TO_TICKS(1000));
                         
        ESP_ERROR_CHECK( device_register_read(REG_ADDR_WRITE, read_buffer, 1) );

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
