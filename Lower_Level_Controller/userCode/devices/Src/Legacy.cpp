#include "ControllerRtosHooks.h"
//
// Created by admin on 2023/11/28.
//

#include "Legacy.h"


volatile float vccMoni = 0;
volatile float vccBat = 0;

void bsp_ADC_vccMoni(){
    static uint8_t calibrationCplt = 0;
    static uint32_t calibrationCnt = 0;
    static uint32_t vrefSum = 0,vrefValue = 0;
    if(calibrationCplt == 0){
        if (calibrationCnt == 0){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else if(calibrationCnt < 200){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else{
            calibrationCplt = 1;
            vrefValue = vrefSum/200.0f;
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 2);
            vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

        }

    }else{
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 2);
        vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

    }
    vccBat = vccMoni/22.0f*222.0f;
}

/**
 * @brief // Using the UART redirection function, either a USB virtual COM port or a hardware UART can be selected; the corresponding driver must be installed for operation. //利用串口重定向函数，可选usb虚拟串口或硬件串口，使用需安装相应驱动
 * @param fmt
 * @param ...
 */
void usart_printf(const char *fmt,...){

        static uint8_t tx_buf[256] = {0};//TODO 爆栈？
        static va_list ap;
        static uint16_t len;

        va_start(ap,fmt);

        /*len = vsprintf((char*)tx_buf,fmt,ap);*/

        va_end(ap);

        //HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
//    CDC_Transmit_FS(tx_buf,len);

    }

    flash_data_t flashData;
    void bsp_flash_write(flash_data_t *)
{
    // TODO(PERSISTENCE): reserve a linker sector; version/CRC/length/atomic update required.
    // Disabled: the historical implementation can erase firmware and overrun the object.
    const char message[] = "FLASH=DISABLED_UNRESERVED_LAYOUT\r\n";
    LcSerial_Write(reinterpret_cast<const uint8_t *>(message), sizeof(message)-1);
}
void bsp_flash_read(flash_data_t *data)
{
    if (data) memset(data, 0, sizeof(*data)); // No valid persisted configuration exists in this release.
}
