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

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "main.h"

#define IST8310_IIC_ADDRESS (0x0E << 1) // 7 位地址 0x0E 左移一位，HAL 使用 0x1C。
#define IST8310_IIC_READ_MSB (0x80) // 历史未使用常量；本工程 I2C 读取不使用 SPI 读标志。

extern void ist8310_GPIO_init(void); // 兼容占位函数；实际 GPIO 已由 MX_GPIO_Init 配置。
extern void ist8310_com_init(void); // 兼容占位函数；实际 I2C3 已由 MX_I2C3_Init 配置。
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); // 复位引脚置高，释放硬件复位。
extern void ist8310_RST_L(void); // 复位引脚置低，进入硬件复位。

#endif
