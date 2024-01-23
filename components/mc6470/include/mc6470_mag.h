
#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
#ifndef __MC6470_MAG_H__
#define __MC6470_MAG_H__

#include "mc6470.h"
#include "stdint.h"

#define MC6470_MAG_SELF_TEST_REG (0x0C)
#define MC6470_MAG_MORE_INFO_VER_REG (0x0D)
#define MC6470_MAG_MORE_INFO_REG (0x0E)
#define MC6470_MAG_WHOIAM_REG (0x0F)
#define MC6470_MAG_OUT_X_LSB_REG (0x10)
#define MC6470_MAG_OUT_X_MSB_REG (0x11)
#define MC6470_MAG_OUT_Y_LSB_REG (0x12)
#define MC6470_MAG_OUT_Y_MSB_REG (0x13)
#define MC6470_MAG_OUT_Z_LSB_REG (0x14)
#define MC6470_MAG_OUT_Z_MSB_REG (0x15)
#define MC6470_MAG_STATUS_REG (0x18)
#define MC6470_MAG_TBD_REG (0x19)
#define MC6470_MAG_CTRL1_REG (0x1B)
#define MC6470_MAG_CTRL2_REG (0x1C)
#define MC6470_MAG_CTRL3_REG (0x1D)
#define MC6470_MAG_CTRL4_REG (0x1E)
#define MC6470_MAG_OFFSET_X_LSB_REG (0x20)
#define MC6470_MAG_OFFSET_X_MSB_REG (0x21)
#define MC6470_MAG_OFFSET_Y_LSB_REG (0x22)
#define MC6470_MAG_OFFSET_Y_MSB_REG (0x23)
#define MC6470_MAG_OFFSET_Z_LSB_REG (0x24)
#define MC6470_MAG_OFFSET_Z_MSB_REG (0x25)
#define MC6470_MAG_ITHR_L_REG (0x26)
#define MC6470_MAG_ITHR_H_REG (0x27)
#define MC6470_MAG_TEMP_REG (0x31)

    uint32_t MC6470_Mag_Init(struct MC6470_Dev_t *dev);
    uint32_t MC6470_Mag_getData(struct MC6470_Dev_t *dev, float *x, float *y, float *z);
    uint32_t MC6470_Mag_I2C_Write(struct MC6470_Dev_t *dev, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length);
    uint32_t MC6470_Mag_I2C_Read(struct MC6470_Dev_t *dev, MC6470_reg_addr reg_address, uint8_t *buffer, size_t buffer_length);
#ifdef __cplusplus
}
#endif
#endif