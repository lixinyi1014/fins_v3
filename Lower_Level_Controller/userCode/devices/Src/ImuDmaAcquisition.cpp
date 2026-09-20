#include "LowerControllerConfig.h"
#if LC_IMU_ASYNC_ENABLED
#include "ImuDmaAcquisition.h"
#include "IMU.h"
#include "ControllerRtosHooks.h"
#include <string.h>

namespace lower_controller
{
namespace
{
ImuDmaArbiter arbiter;
ImuPacketPublisher publish_packet;
RawImuPacket active_packet;
uint8_t tx[12], rx[12]; // 唯一 DMA 缓冲；完成后先复制入队，再允许下一次复用。
bool completion_pending, abort_pending;

bool HardwareIdle()
{
    return !((hdma_spi1_rx.Instance->CR | hdma_spi1_tx.Instance->CR) & DMA_SxCR_EN) &&
           !(hspi1.Instance->SR & SPI_SR_BSY);
}
void ClearFlags()
{
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_rx) |
                                            __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_spi1_rx) |
                                            __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_rx) |
                                            __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_rx) |
                                            __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_rx));
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_tx) |
                                            __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_spi1_tx) |
                                            __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_tx) |
                                            __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_tx) |
                                            __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_tx));
}
void ReleaseChipSelects()
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
void Publish(const RawImuPacket &packet, bool from_isr)
{
    if (!publish_packet || !publish_packet(packet, from_isr))
        ++arbiter.diagnostics.queue_drops;
}
void StartNext(uint32_t now_us)
{
    if (abort_pending || completion_pending || !HardwareIdle() || !arbiter.Begin(now_us, active_packet))
        return;
    memset(tx, 0xff, sizeof(tx));
    uint32_t length;
    if (active_packet.kind == SensorKind::Gyroscope)
    {
        tx[0] = 0x82;
        length = 8;
    }
    else if (active_packet.kind == SensorKind::Accelerometer)
    {
        tx[0] = 0x92;
        length = 11;
    }
    else
    {
        tx[0] = 0xa2;
        length = 4;
    }
    // 加速度：命令 + dummy + 六字节轴值 + 三字节 sensor_time；不能只读取最低时间字节。
    ClearFlags();
    hdma_spi1_tx.Instance->M0AR = reinterpret_cast<uint32_t>(tx);
    hdma_spi1_rx.Instance->M0AR = reinterpret_cast<uint32_t>(rx);
    hdma_spi1_tx.Instance->NDTR = length;
    hdma_spi1_rx.Instance->NDTR = length;
    if (active_packet.kind == SensorKind::Gyroscope)
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    __DMB();
    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}
void Service(uint32_t now_us, bool from_isr)
{
    if (abort_pending)
    {
        if (!HardwareIdle())
            return; // 不在 ISR 自旋；未停稳期间一直禁止复用内存。
        ReleaseChipSelects();
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1); // 停稳后依次读 DR/SR，排空中止传输遗留的 RX 数据并清 OVR。
        ClearFlags();
        arbiter.Fail();
        completion_pending = abort_pending = false;
    }
    if (completion_pending && HardwareIdle())
    {
        ReleaseChipSelects();
        RawImuPacket packet;
        if (arbiter.Complete(now_us, packet))
        {
            memcpy(packet.bytes, rx, sizeof(rx));
            Publish(packet, from_isr);
        }
        completion_pending = false;
    }
    StartNext(now_us);
}
} // namespace

void StartImuDmaAcquisition(ImuPacketPublisher publisher)
{
    publish_packet = publisher;
    completion_pending = abort_pending = false;
    ReleaseChipSelects();
    ClearFlags();
    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_ENABLE_IT(&hdma_spi1_tx, DMA_IT_TE | DMA_IT_DME);
    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_FE); // FE 位在 FCR，必须与 CR 中的 TC/TE/DME 分开设置。
    __HAL_DMA_ENABLE_IT(&hdma_spi1_tx, DMA_IT_FE);
}
void ImuDataReady(SensorKind kind, uint32_t time_us)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    arbiter.Request(kind, time_us);
    if (kind == SensorKind::Magnetometer)
    {
        RawImuPacket packet = {};
        packet.kind = kind;
        packet.stamp = {time_us, time_us, arbiter.diagnostics.edges[2]};
        Publish(packet, true);
    }
    else
        Service(time_us, true);
    __set_PRIMASK(mask);
}
void ImuDmaInterrupt()
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t rx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_rx) |
                               __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_rx) |
                               __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_rx);
    const uint32_t tx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_tx) |
                               __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_tx) |
                               __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_tx);
    if (__HAL_DMA_GET_FLAG(&hdma_spi1_rx, rx_errors) || __HAL_DMA_GET_FLAG(&hdma_spi1_tx, tx_errors))
    {
        abort_pending = true;
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
        ClearFlags();
    }
    else if (__HAL_DMA_GET_FLAG(&hdma_spi1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_rx)))
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_rx));
        completion_pending = true;
    }
    Service(LcTime_NowUs(), true);
    __set_PRIMASK(mask);
}
void PollImuDma(uint32_t now_us)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (arbiter.Busy() && now_us - active_packet.read_started_us > 2000U)
    {
        abort_pending = true;
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }
    Service(now_us, false);
    __set_PRIMASK(mask);
}
uint32_t MagnetometerSequence()
{
    return arbiter.diagnostics.edges[2];
}
ImuAcquisitionDiagnostics ReadImuAcquisitionDiagnostics()
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ImuAcquisitionDiagnostics result = arbiter.diagnostics;
    __set_PRIMASK(mask);
    return result;
}
} // namespace lower_controller
#endif
