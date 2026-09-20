#ifndef IMU_DMA_ACQUISITION_H
#define IMU_DMA_ACQUISITION_H
#include "ImuDmaArbiter.h"

namespace lower_controller
{
// 回调必须复制 packet；from_isr 决定使用 xQueueSendFromISR 还是 xQueueSend。
typedef bool (*ImuPacketPublisher)(const RawImuPacket &packet, bool from_isr);
void StartImuDmaAcquisition(ImuPacketPublisher publisher);
void ImuDataReady(SensorKind kind, uint32_t time_us);
void ImuDmaInterrupt();
void PollImuDma(uint32_t now_us);
uint32_t MagnetometerSequence();
ImuAcquisitionDiagnostics ReadImuAcquisitionDiagnostics();
} // namespace lower_controller
#endif
