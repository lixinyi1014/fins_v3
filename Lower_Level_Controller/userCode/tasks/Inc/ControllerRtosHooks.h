#ifndef CONTROLLER_RTOS_HOOKS_H
#define CONTROLLER_RTOS_HOOKS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
    uint32_t LcTime_NowUs(void);
    void LcTime_Start(void);
    void LcTime_DelayUs(uint32_t delay_us);
    int LcRuntime_IsRunning(void);
    void LcRuntime_ControlTickFromISR(void);
    void LcRuntime_ImuReadyFromISR(void);
    void LcRuntime_UartRxFromISR(uint16_t length);
    void LcRuntime_UartTxFromISR(void);
    void LcRuntime_UartErrorFromISR(void);
    int LcSerial_Write(const uint8_t *data, uint16_t length);
    int LcRuntime_OutputsStopped(uint32_t epoch);
    void LcRuntime_Assert(const char *file, int line);
    void LcSafetyHardware_Start(void);
    void LcSafetyHardware_Feed(void);
    void LcSafetyHardware_SetOutputEnabled(int enabled);
    int LcSafetyHardware_WatchdogReset(void);
#ifdef __cplusplus
}
#endif
#endif
