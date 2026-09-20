#include "ControllerRtosHooks.h"
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
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

#include "ist8310driver_middleware.h"
#include "main.h"
#include "I2cBusAccess.h"


extern I2C_HandleTypeDef hi2c3;


void ist8310_GPIO_init(void)
{
}

void ist8310_com_init(void)
{
}


uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    LcBus_MemRead(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100); // 8 位寄存器地址，读 1 字节；运行期 IMU 任务独占，HAL 超时参数为 2 ms
    return res;
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    LcBus_MemWrite(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100); // 8 位寄存器地址，写 1 字节；运行期 HAL 超时参数为 2 ms

}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    LcBus_MemRead(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100); // len 为字节数，reg 为首寄存器；运行期 HAL 超时参数为 2 ms
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    LcBus_MemWrite(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100); // len 为字节数，reg 为首寄存器；运行期 HAL 超时参数为 2 ms
}
void ist8310_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}
void ist8310_delay_us(uint16_t us)
{
    LcTime_DelayUs(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET);
}
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET);
}
