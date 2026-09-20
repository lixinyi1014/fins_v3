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

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "main.h"

#define IST8310_DATA_READY_BIT 2 // 软件 status 的 bit2；不是芯片状态寄存器 DRDY 位，硬件 DRDY 为 bit0。

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;

extern uint8_t ist8310_init(void);
extern // status_buf 共 7 字节：0x02 状态 + 从 0x03 起的三轴低/高字节，输出 uT。
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(float mag[3]);
#endif
