#include "ControllerRtosHooks.h"
#include "BMI088Middleware.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
static bool transfer_failed;
void BMI088_ClearTransferError() { transfer_failed = false; }
int BMI088_TransferOk() { return !transfer_failed; }

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    LcTime_DelayUs(us);
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    if (!transfer_failed && HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 2) != HAL_OK)
        transfer_failed = true;
    return rx_data;
}

