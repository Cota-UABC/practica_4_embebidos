#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c.h"

void app_main(void)
{
    ESP_ERROR_CHECK(init_i2c());

    while(true)
    {
        //write_bytes[1]++;
        //read_bytes = write_bytes[0];

        ESP_ERROR_CHECK(
            i2c_master_write_to_device(I2C_NUM_0, 
                                        I2C_SLAVE_ADDR,
                                        (const uint8_t*)&write_bytes,
                                        BUFFER_SIZE_WRITE,
                                        pdMS_TO_TICKS(2000)));
        ESP_LOGI(TAG, "Data written: ADRR:%x Data:%x",write_bytes[0],write_bytes[1]);                          

        ESP_ERROR_CHECK(
            i2c_master_write_read_device(I2C_NUM_0, 
                                        I2C_SLAVE_ADDR,
                                        &read_bytes,
                                        1,
                                        read_buffer,
                                        BUFFER_SIZE_READ,
                                        pdMS_TO_TICKS(2000)));

        ESP_LOGI(TAG, "Addr read: %x",read_bytes);
        ESP_LOG_BUFFER_HEX(TAG, read_buffer, BUFFER_SIZE_READ);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}
