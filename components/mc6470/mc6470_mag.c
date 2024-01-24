#include "mc6470.h"
#include "mc6470_mag.h"
#define LOG_TAG ("mc6470_mag.c")
uint32_t MC6470_Mag_Init(struct MC6470_Dev_t *dev)
{
    uint8_t read_val;
    uint8_t write_val;
    MC6470_reg_addr reg_addr = MC6470_MAG_WHOIAM_REG;
    uint32_t ret;

    ret = MC6470_Mag_I2C_Read(dev, reg_addr, &read_val, sizeof(read_val));
    if (ret != MC6470_Status_OK)
        ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);

    ESP_LOGI(LOG_TAG, "WHOAMI REG 0x%x", read_val);

    write_val = 0x98; // set active mode and set Force State in active mode command (ref. mc6470 datasheet)
    reg_addr = MC6470_MAG_CTRL1_REG;
    ret = MC6470_Mag_I2C_Write(dev, reg_addr, &write_val, sizeof(write_val));
    if (ret != MC6470_Status_OK)
        ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);
    MC6470_delay_ms(10);
    ret = MC6470_Mag_I2C_Read(dev, reg_addr, &read_val, sizeof(read_val));
    if (ret != MC6470_Status_OK)
        ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);

    ESP_LOGI(LOG_TAG, "MC6470_MAG_CTRL1_REG 0x%x", read_val);

    write_val = 0x40; // start measurement in Force State command (ref. mc6470 datasheet)
    reg_addr = MC6470_MAG_CTRL3_REG;
    ret = MC6470_Mag_I2C_Write(dev, reg_addr, &write_val, sizeof(write_val));
    if (ret != MC6470_Status_OK)
        ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);
    MC6470_delay_ms(10);

    ret = MC6470_Mag_I2C_Read(dev, reg_addr, &read_val, sizeof(read_val));
    if (ret != MC6470_Status_OK)
        ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);

    ESP_LOGI(LOG_TAG, "MC6470_MAG_CTRL3_REG 0x%x", read_val);

    return MC6470_Status_OK;
}
uint32_t MC6470_Mag_hasData(struct MC6470_Dev_t *dev, bool *has_data)
{
    RETURN_ERROR_IF_NULL(dev);
    MC6470_reg_addr reg_addr = MC6470_MAG_STATUS_REG;
    uint32_t result = 0;
    uint8_t current = 0;

    result |= MC6470_Mag_I2C_Read(dev, reg_addr, &current, sizeof(current));
    int v = ((current & (0b00000001 << 6)) >> 6); // checks for 6th bit DRDY (ref. mc6470 datasheet)
                                                  // ESP_LOGI(LOG_TAG, "Reg Read [0x%02X]: 0x%02X Value: %i\r\n", reg_addr, current, v);
    if (!MC6470_IS_ERROR(result))
    {
        *has_data = v;
    }
    else
    {
        *has_data = false;
    }
    return result;
};
uint32_t MC6470_Mag_getData(struct MC6470_Dev_t *dev, float *x, float *y, float *z)
{
    RETURN_ERROR_IF_NULL(dev);
    uint8_t reg_addr = MC6470_MAG_CTRL3_REG;
    uint32_t ret;
    uint8_t write_val = 0x40; // start measurement in Force State command (ref. mc6470 datasheet)
    // ret = MC6470_Mag_I2C_Write(dev, reg_addr, &write_val, sizeof(write_val));
    // if (ret != MC6470_Status_OK)
    //     ESP_LOGE(LOG_TAG, "ERROR AT LINE %d WITH NO %ld", __LINE__, ret);
    // MC6470_delay_ms(10);
    reg_addr = MC6470_MAG_OUT_X_LSB_REG;

    uint8_t data[6] = {0};

    uint32_t result = MC6470_Mag_I2C_Read(dev, reg_addr, data, sizeof(data));
    bool has_data = false;
    short _x = 0;
    short _y = 0;
    short _z = 0;
    while (!has_data)
    {
        MC6470_Mag_hasData(dev, &has_data);
        // MC6470_delay_ms(700);
    }
    if (!MC6470_IS_ERROR(result))
    {
        _x = data[0] | (data[1] << 8);
        _y = data[2] | (data[3] << 8);
        _z = data[4] | (data[5] << 8);

        *x = (float)_x;
        *y = (float)_y;
        *z = (float)_z;
    }
    return result;
};

uint32_t MC6470_Mag_I2C_Write(struct MC6470_Dev_t *dev, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length)
{
    if (dev == NULL)
    {
        return MC6470_Status_Null_PTR_ERROR;
    }
    return MC6470_I2C_Write(dev, dev->mag_address, reg_address, buffer, buffer_length);
};

uint32_t MC6470_Mag_I2C_Read(struct MC6470_Dev_t *dev, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length)
{
    if (dev == NULL)
    {
        return MC6470_Status_Null_PTR_ERROR;
    }
    return MC6470_I2C_Read(dev, dev->mag_address, reg_address, buffer, buffer_length);
};
