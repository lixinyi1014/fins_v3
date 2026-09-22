#ifndef I2C_BUS_ACCESS_H
#define I2C_BUS_ACCESS_H
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
#if LC_USE_FREERTOS
    void LcBus_SetOwner(I2C_HandleTypeDef *bus, void *task);
    void LcBus_Begin(I2C_HandleTypeDef *bus, uint32_t budget_us);
    int LcBus_Ok(I2C_HandleTypeDef *bus);
    uint32_t LcBus_ErrorCount(I2C_HandleTypeDef *bus);
    uint32_t LcBus_RecoveryCount(I2C_HandleTypeDef *bus); // 出错后执行的总线恢复次数
    HAL_StatusTypeDef LcBus_Transmit(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t);
    HAL_StatusTypeDef LcBus_Receive(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t, uint32_t);
    HAL_StatusTypeDef LcBus_MemRead(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t,
                                    uint32_t);
    HAL_StatusTypeDef LcBus_MemWrite(I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t,
                                     uint32_t);
#else
#define LcBus_Transmit HAL_I2C_Master_Transmit
#define LcBus_Receive HAL_I2C_Master_Receive
#define LcBus_MemRead HAL_I2C_Mem_Read
#define LcBus_MemWrite HAL_I2C_Mem_Write
#define LcBus_Ok(bus) 1
#endif
#ifdef __cplusplus
}
#endif
#endif
