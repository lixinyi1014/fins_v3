#include "TaskSharedResources.h"
#if LC_IMU_ASYNC_ENABLED
#include "ImuDmaAcquisition.h"
using namespace lower_controller;
using namespace lower_controller::tasks;

bool lower_controller::tasks::PublishImuPacket(const RawImuPacket &packet, bool from_isr)
{
    if (!from_isr)
        return xQueueSend(imu_raw_samples.handle, &packet, 0) == pdPASS;
    BaseType_t wake = pdFALSE;
    bool ok = xQueueSendFromISR(imu_raw_samples.handle, &packet, &wake) == pdPASS;
    portYIELD_FROM_ISR(wake);
    return ok; // 仅复制原始帧，不在中断中解码或计算矩阵。
}
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (!tasks_running)
        return;
    const uint32_t now = LcTime_NowUs(); // TIM2 微秒时刻；DRDY 到达时间是采样代理时间，含传感器内部延迟。
    if (pin == GPIO_PIN_5)
        ImuDataReady(SensorKind::Gyroscope, now); // PC5，1000 Hz。
    else if (pin == GPIO_PIN_4)
        ImuDataReady(SensorKind::Accelerometer, now); // PC4，800 Hz。
    else if (pin == GPIO_PIN_3)
        ImuDataReady(SensorKind::Magnetometer, now); // PG3，实际频率由边沿计数观测。
}
#endif
