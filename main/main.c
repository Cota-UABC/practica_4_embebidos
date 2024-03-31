#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
//#include "i2c.h"

#define LOW_POWER_MODE
//#define NO_LOW_POWER_MODE

#define I2C_SLAVE_ADDR 0x68
#define CLOCK_SPD 400000

//#define REG_ADDR_WRITE 0x19
#define WHO_AM_I 0x75
#define GYRO_OUT 0x43
#define GYRO_CONFIG 0x1B
#define ACCEL_OUT 0x3B
#define ACCEL_CONFIG 0x1C
#define POW_MANG 0x6B
#define POW_MANG2 0x6C
#define TEMP_OUT 0x41

#define BUFFER_SIZE_READ 10

#define RANGE_ACCEL 2
#define RANGE_GYRO 2000


static const char *TAG = "I2C";

esp_err_t init_i2c(void);
esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, uint8_t len, char *reg_name);
esp_err_t device_register_write_byte(uint8_t reg_addr, uint8_t data);
float raw_to_real_value(uint8_t raw_data_h, uint8_t raw_data_l, uint16_t range);


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

esp_err_t device_register_read(uint8_t reg_addr, uint8_t *data, uint8_t len, char *reg_name)
{
    ESP_ERROR_CHECK(
            i2c_master_write_read_device(I2C_NUM_0,
                                        I2C_SLAVE_ADDR,
                                        &reg_addr,
                                        1,
                                        data,
                                        len,
                                        pdMS_TO_TICKS(2000)));

    if(reg_name[0]){
		ESP_LOGI(TAG, "READ:");
		ESP_LOGI(TAG, "    Addr read: 0x%x",reg_addr);

		if(len <= 1)
			ESP_LOGI(TAG, "    %s= 0x%x",reg_name,data[0]);
		else
		{
			ESP_LOGI(TAG, "    %s:",reg_name);
			for(int i=0;i<len;i++)
				ESP_LOGI(TAG, "    Data= 0x%x",data[i]);
		}
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
                                        (const uint8_t*)write_bytes,
                                        2,
                                        pdMS_TO_TICKS(2000)));

    ESP_LOGI(TAG, "WRITE:");
    ESP_LOGI(TAG, "    Adrr: 0x%x Data: 0x%x",write_bytes[0],write_bytes[1]);

    free(write_bytes);

    return ESP_OK;
}

float raw_to_real_value(uint8_t raw_data_h, uint8_t raw_data_l, uint16_t range)
{
    float value_f;
	uint16_t value = (uint16_t)raw_data_h << 8 | raw_data_l;

	if(value>>15)
	{
	    value = ~value;
	    value++;
	    value_f = ((float)value / 0x8000) * (float)range;
	    value_f = value_f*-1;
	}
	else
	    value_f = ((float)value / 0x7fff) * (float)range;

	return value_f;
}

void app_main(void)
{
    uint8_t read_buffer[BUFFER_SIZE_READ];
#ifdef NO_LOW_POWER_MODE
    int16_t temp_i;
    float temp_f;
#endif

    ESP_ERROR_CHECK(init_i2c());

    //reset sensor
    ESP_ERROR_CHECK( device_register_write_byte(POW_MANG, 0x80) );
    //ESP_ERROR_CHECK( device_register_read(POW_MANG, read_buffer, 1,"POWER") );


#ifdef LOW_POWER_MODE
    //8MHz clock source sensor, turn off temp, and cycle mode
    ESP_ERROR_CHECK( device_register_write_byte(POW_MANG, 0x28) );
    //5 Hz Wake-up Frequency and Gyroscope in standby mode
    ESP_ERROR_CHECK( device_register_write_byte(POW_MANG2, 0x47) );
#else
    //8MHz clock source sensor
    ESP_ERROR_CHECK( device_register_write_byte(POW_MANG, 0x00) );
#endif
    ESP_ERROR_CHECK( device_register_read(POW_MANG, read_buffer, 1,"POWER") );

    //set full scale range Accelerometer
    ESP_ERROR_CHECK( device_register_write_byte(ACCEL_CONFIG, 0x00) );
    ESP_ERROR_CHECK( device_register_read(ACCEL_CONFIG, read_buffer, 1,"ACCELEROMETER CONFIG") );
#ifdef NO_LOW_POWER_MODE
	//set full scale range Gyro
    ESP_ERROR_CHECK( device_register_write_byte(GYRO_CONFIG, 0x18) );
    ESP_ERROR_CHECK( device_register_read(GYRO_CONFIG, read_buffer, 1,"GYRO CONFIG") );
#endif

    while(true)
    {
    	ESP_LOGI(TAG, "-------------------------------------------------");

    	ESP_ERROR_CHECK( device_register_read(WHO_AM_I, read_buffer, 1,"") );
    	 ESP_LOGI(TAG, "WHO AM I: 0x%x",read_buffer[0]);

    	ESP_ERROR_CHECK( device_register_read(ACCEL_OUT, read_buffer, 6,"") );
		ESP_LOGI(TAG, "ACCELEROMETER");
		ESP_LOGI(TAG, "  X: %.2f",raw_to_real_value(read_buffer[0],read_buffer[1],RANGE_ACCEL));
		ESP_LOGI(TAG, "  Y: %.2f",raw_to_real_value(read_buffer[2],read_buffer[3],RANGE_ACCEL));
		ESP_LOGI(TAG, "  Z: %.2f",raw_to_real_value(read_buffer[4],read_buffer[5],RANGE_ACCEL));

#ifdef NO_LOW_POWER_MODE
    	ESP_ERROR_CHECK( device_register_read(GYRO_OUT, read_buffer, 6,"") );
		ESP_LOGI(TAG, "GYROSCOPE");
		ESP_LOGI(TAG, "  X: %.2f",raw_to_real_value(read_buffer[0],read_buffer[1],RANGE_GYRO));
		ESP_LOGI(TAG, "  Y: %.2f",raw_to_real_value(read_buffer[2],read_buffer[3],RANGE_GYRO));
		ESP_LOGI(TAG, "  Z: %.2f",raw_to_real_value(read_buffer[4],read_buffer[5],RANGE_GYRO));

        ESP_ERROR_CHECK( device_register_read(TEMP_OUT, read_buffer, 2,"") );
        temp_i = (read_buffer[0] << 8) | read_buffer[1];
        temp_f = ((float)temp_i / 340.0) + 36.53;
        ESP_LOGI(TAG, "TEMPERATURE: %.2f C",temp_f);
#endif

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
