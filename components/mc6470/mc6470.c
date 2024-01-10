/*
 * A C library for the mCube MC6470 6-axis accelerometer and magnetometer sensor.
 * Copyright (C) 2022  eResearch, James Cook University
 * Author: NigelB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "mc6470.h"
#include "mc6470_accel.h"
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#include <stdarg.h>

#define SDA_PIN GPIO_NUM_19
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_NUM 0

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    int ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 350000,
    };

    i2c_param_config(i2c_master_port, &conf);
    ret = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    return ret;
}

uint32_t MC6470_I2C_Write(struct MC6470_Dev_t *dev, MC6470_Address_e address, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length)
{
    uint32_t ret;
    uint8_t write_buf[buffer_length + 1];

    write_buf[0] = reg_address;
    memcpy(&write_buf[1], buffer, sizeof(write_buf));

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, address, write_buf, sizeof(write_buf), portMAX_DELAY);
    if (ret != ESP_OK)
    {
        ESP_LOGE("mc6470.c", "MC6470_I2C_Write ERR %ld", ret);
        ret = MC6470_Status_ERROR;
    }
    else
    {
        ret = MC6470_Status_OK;
    }
    return ret;
}

uint32_t MC6470_I2C_Read(struct MC6470_Dev_t *dev, MC6470_Address_e address, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length)
{

    uint32_t ret;
    ret = i2c_master_write_read_device(I2C_MASTER_NUM, address, &reg_address, 1, buffer, buffer_length, portMAX_DELAY);
    if (ret != ESP_OK)
    {
        ESP_LOGE("mc6470.c", "MC6470_I2C_Write ERR %ld", ret);
        ret = MC6470_Status_ERROR;
    }
    else
    {
        ret = MC6470_Status_OK;
    }
    return ret;
}

int MC6470_printf(struct MC6470_Dev_t *dev, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char log_message[256];

    vsnprintf(log_message, sizeof(log_message), format, args);
    ESP_LOGI("mc6470", "%s", log_message);

    va_end(args);
    return 0;
}
void MC6470_delay_ms(unsigned long milliseconds)
{
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

void MC6470_delay_us(unsigned long microseconds)
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
}
void MC6470_Init(struct MC6470_Dev_t *dev, MC6470_Address_e address)
{
    i2c_master_init();
    dev->accel_address = address;
    // dev->mag_address = MC6470_MAG_ADDRESS;
};

uint32_t MC6470_begin(struct MC6470_Dev_t *dev)
{
    MC6470_printf(dev, "[MC6470 Accel] Address: 0x%02X\r\n", dev->accel_address);
    MC6470_printf(dev, "[MC6470 Mag  ] Address: 0x%02X\r\n", dev->mag_address);
    uint32_t result;
    result = MC6470_Accel_Init(dev);
    if (result != MC6470_Status_OK)
        ESP_LOGE("mc6470.c", "ERROR AT LINE %d WITH NO %ld", __LINE__, result);
    // result |= MC6470_Mag_Init(dev);
    bool found = false;
    result = MC6470_Accel_ChipIDs(dev, &found);
    if (!MC6470_IS_ERROR(result))
    {
        result = MC6470_Accel_get_Range_and_Resolution(dev, NULL, NULL);
        if (result != MC6470_Status_OK)
            ESP_LOGE("mc6470.c", "ERROR AT LINE %d WITH NO %ld", __LINE__, result);
        result = MC6470_Accel_set_Range_and_Resolution(dev, MC6470_ACCEL_OUTCFG_RANGE_8G, MC6470_ACCEL_OUTCFG_RES_14_Bits);
        if (result != MC6470_Status_OK)
            ESP_LOGE("mc6470.c", "ERROR AT LINE %d WITH NO %ld", __LINE__, result);
    }

    result = MC6470_Accel_set_OperationState(dev, MC6470_ACCEL_MODE_OPCON_Wake);
    if (result != MC6470_Status_OK)
        ESP_LOGE("mc6470.c", "ERROR AT LINE %d WITH NO %ld", __LINE__, result);
    return result;
};

uint32_t MC6470_check_ids(struct MC6470_Dev_t *dev)
{
    uint32_t result = 0;
    bool found = false;
    result |= MC6470_Accel_ChipIDs(dev, &found);

    return result;
};

uint32_t MC6470_getData(struct MC6470_Dev_t *dev, MC6470_MagReading *mag_data, MC6470_AccelReading *accel_data)
{
    RETURN_ERROR_IF_NULL(dev);
    uint32_t result = 0;
    if (mag_data != NULL)
    {
    }
    if (accel_data != NULL)
    {
        result |= MC6470_Accel_getData(dev, &accel_data->x, &accel_data->y, &accel_data->z);
    }

    return result;
};
