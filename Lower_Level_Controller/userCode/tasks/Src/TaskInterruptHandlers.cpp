#include "TaskSharedResources.h"
namespace { lower_controller::CommandLineAssembler command_lines; }
#include <string.h>
#include "ImuDmaAcquisition.h"

// TIM1 发布控制节拍及温控请求，UART6 交接收发事件；默认 DRDY/DMA 原始帧入队见 ImuSampleInterrupts.cpp。

using namespace lower_controller;
using namespace lower_controller::tasks;

extern "C" void LcRuntime_ControlTickFromISR(void)
{
    if (!tasks_running)
        return;
    const uint32_t now = LcTime_NowUs();
    CheckCommandTimeout(now);
    if (now - last_control_alive_us >= LC_TASK_STALL_TIMEOUT_US)
        SafetyStopFromISR(StopControlStall, true);
    ++control_release_sequence;
    control_release_us = LcTime_NowUs();
    ++controller_task_diagnostics.releases;
    BaseType_t wake = pdFALSE;
#if LC_IMU_ASYNC_ENABLED
    ImuDataReady(SensorKind::Temperature, control_release_us); // 150 Hz 温控请求；轴数据由各自 DRDY 触发。
#else
    vTaskNotifyGiveFromISR(imu_attitude_task, &wake); // 兼容模式：IMU 释放计数 +1。
#endif
    vTaskNotifyGiveFromISR(control_loop_task, &wake); // 控制释放计数 +1；优先级 4
    portYIELD_FROM_ISR(wake);
}
extern "C" void LcRuntime_ImuReadyFromISR(void)
{
    if (!tasks_running)
        return;
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(imu_dma_done, &wake); // 完成信号独立于 TIM1 的释放计数通知
    portYIELD_FROM_ISR(wake);
}
extern "C" void LcRuntime_UartRxFromISR(uint16_t length)
{
    if (!tasks_running)
        return;
    BaseType_t wake = pdFALSE;
    if (length > sizeof(uart_rx_buffer)) {
        command_lines.DiscardUntilDelimiter();
        ++controller_task_diagnostics.rx_drops;
    } else for (uint16_t i=0; i<length; ++i) {
        const char *line = command_lines.Feed(uart_rx_buffer[i], LcTime_NowUs());
        if (!line) continue;
        const size_t count = strlen(line);
        if (IsOffCommand(line, count)) {
            // Queue saturation cannot prevent a complete OFF line from stopping.
            const uint32_t mask = __get_PRIMASK();
            __disable_irq();
            outputs_stopped = 1; neutral_pwm_ready = 0; ++stop_epoch;
            if (!safety_fault_latched) controller_task_diagnostics.last_stop_reason = StopUserOff;
            LcSafetyHardware_SetOutputEnabled(0);
            __set_PRIMASK(mask);
        }
        ReceivedCommandPacket packet = {};
        packet.stop_epoch = stop_epoch;
        packet.received_us = LcTime_NowUs();
        packet.length = count;
        memcpy(packet.data, line, count);
        if (xQueueSendFromISR(received_commands.handle, &packet, &wake) != pdPASS)
            ++controller_task_diagnostics.rx_drops;
    }
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, sizeof(uart_rx_buffer)) != HAL_OK)
        ++controller_task_diagnostics.rx_errors;
    portYIELD_FROM_ISR(wake);
}
extern "C" void LcRuntime_UartTxFromISR(void)
{
    if (!tasks_running)
        return;
    BaseType_t wake = pdFALSE;
    xTaskNotifyFromISR(uart_transmit_task, 1U, eSetBits, &wake); // 0x01：发送完成位，唤醒发送任务
    portYIELD_FROM_ISR(wake);
}
extern "C" void LcRuntime_UartErrorFromISR(void)
{
    if (!tasks_running)
        return;
    ++controller_task_diagnostics.rx_errors;
    command_lines.DiscardUntilDelimiter();
    HAL_UART_AbortReceive(&huart6);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, sizeof(uart_rx_buffer));
    BaseType_t wake = pdFALSE;
    xTaskNotifyFromISR(uart_transmit_task, 2U, eSetBits, &wake); // 0x02：UART 错误位，发送任务走失败处理
    portYIELD_FROM_ISR(wake);
}
