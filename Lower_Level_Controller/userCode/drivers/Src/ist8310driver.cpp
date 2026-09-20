/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310 initialization, register access and magnetic decoding.
  *             IST8310 初始化、寄存器访问及磁场解码。本工程直接使用 I2C3。
  * @note       当前不经过 MPU6500 转接；GPIO/I2C 外设由 CubeMX 初始化。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              Initial version.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

#define MAG_SEN 0.3f // 0.3 uT/LSB；有符号 16 位原始值乘此灵敏度。

#define IST8310_WHO_AM_I 0x00 // WHO_AM_I 寄存器地址 0x00。
#define IST8310_WHO_AM_I_VALUE 0x10 // 期望设备 ID 0x10。

#define IST8310_WRITE_REG_NUM 4 // 依次写入并回读检查四个配置寄存器。

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
    {
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x04}};

uint8_t ist8310_init(void)
{
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_GPIO_init();
    ist8310_com_init();

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);
    // Configure IST8310 and verify each value by readback. 配置 IST8310，并逐项回读检查。
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_ms(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }

    return IST8310_NO_ERROR;
}

// status_buf 共 7 字节：0x02 状态 + 从 0x03 起的三轴低/高字节，输出 uT。
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

void ist8310_read_mag(float mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
