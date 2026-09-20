#include "TaskSharedResources.h"
#include <string.h>

// 串口发送：从队列取数据副本 → 启动 UART6 中断发送 → 等完成/错误/超时。

namespace lower_controller
{
namespace tasks
{

void UartTransmitTask(void *)
{
    for (;;)
    {
        UartTxPacket packet;
        xQueueReceive(uart_tx_packets.handle, &packet,
                      portMAX_DELAY); // 无发送包时阻塞；本地 packet 持有本次异步发送数据
        uint32_t events;
        while (xTaskNotifyWait(0, UINT32_MAX, &events, 0) == pdTRUE)
        {
        } // 非阻塞清理旧发送事件；UINT32_MAX 清除全部通知位
        bool ok = HAL_UART_Transmit_IT(&huart6, packet.data, packet.length) == HAL_OK;
        if (ok)
            ok = xTaskNotifyWait(0, UINT32_MAX, &events, pdMS_TO_TICKS(LC_UART_TX_TIMEOUT_MS)) == pdTRUE &&
                 events == 1U; // 30 个 1 ms tick；只有完成位 0x01、无错误位才算成功
        if (!ok)
        {
            HAL_UART_AbortTransmit(&huart6);
            ++controller_task_diagnostics.tx_errors;
        }
        controller_task_diagnostics.uart_transmit_stack_free = uxTaskGetStackHighWaterMark(NULL);
    } // packet 的内存直到完成通知或 Abort 后才可复用
}

} // namespace tasks
} // namespace lower_controller

using namespace lower_controller;
using namespace lower_controller::tasks;

extern "C" int LcSerial_Write(const uint8_t *data, uint16_t length)
{
    if (!tasks_running)
        return HAL_UART_Transmit(&huart6, const_cast<uint8_t *>(data), length, 100) == HAL_OK;
    configASSERT(__get_IPSR() == 0);
    UartTxPacket packet = {};
    bool valid = length && length <= sizeof(packet.data); // 单包 1..192 字节；队列容量为 16 包
    if (valid)
    {
        packet.length = length;
        memcpy(packet.data, data, length);
        if (xQueueSend(uart_tx_packets.handle, &packet, 0) == pdPASS)
            return 1; // 0：不阻塞；1：数据副本入队成功，尚不代表发送完成
    }
    taskENTER_CRITICAL();
    ++controller_task_diagnostics.tx_drops;
    taskEXIT_CRITICAL();
    return 0;
}
