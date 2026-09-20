#ifndef IMU_DMA_ARBITER_H
#define IMU_DMA_ARBITER_H
#include "SensorFusionData.h"

namespace lower_controller
{
struct ImuAcquisitionDiagnostics
{
    uint32_t edges[4], pending_overwrites, expired, ambiguous, dma_errors, queue_drops;
};

/* 单一 SPI 的仲裁状态，不访问 HAL 或 RTOS，便于用相同代码验证中断交错。
 * 每种传感器最多保留一个待读请求；覆盖意味着丢样，不能沿用被覆盖样本的时间戳。 */
class ImuDmaArbiter
{
  public:
    ImuDmaArbiter() : diagnostics{}, pending_{}, waiting_{}, active_{}, busy_(false), corrupt_(false)
    {
    }
    void Request(SensorKind kind, uint32_t time_us)
    {
        const unsigned index = static_cast<unsigned>(kind);
        if (index >= 4)
            return;
        SampleStamp stamp = {time_us, 0, ++diagnostics.edges[index]};
        if (kind == SensorKind::Magnetometer)
            return; // 磁力计使用 I2C3，不参加 SPI 仲裁。
        if (waiting_[index])
            ++diagnostics.pending_overwrites;
        pending_[index] = stamp;
        waiting_[index] = true;
        if (busy_ && active_.kind == kind)
            corrupt_ = true; // 读出期间同一寄存器又更新，本帧时间含义不再确定。
    }
    bool Begin(uint32_t now_us, RawImuPacket &packet)
    {
        if (busy_)
            return false;
        int chosen = -1;
        for (unsigned i = 0; i < 4; ++i)
        {
            if (!waiting_[i])
                continue;
            const uint32_t maximum_age = i == 0 ? 1000U : (i == 1 ? 1250U : 6667U);
            if (now_us - pending_[i].sample_us >= maximum_age)
            {
                waiting_[i] = false;
                ++diagnostics.expired;
                continue;
            }
            if (chosen < 0 || static_cast<int32_t>(pending_[i].sample_us - pending_[chosen].sample_us) < 0)
                chosen = i;
        }
        if (chosen < 0)
            return false;
        active_ = {};
        active_.kind = static_cast<SensorKind>(chosen);
        active_.stamp = pending_[chosen];
        active_.read_started_us = now_us;
        waiting_[chosen] = false;
        busy_ = true;
        corrupt_ = false;
        packet = active_;
        return true;
    }
    bool Complete(uint32_t now_us, RawImuPacket &packet)
    {
        if (!busy_)
            return false;
        packet = active_;
        packet.stamp.received_us = now_us;
        busy_ = false;
        if (corrupt_)
        {
            ++diagnostics.ambiguous;
            return false;
        }
        return true;
    }
    void Fail()
    {
        if (busy_)
            ++diagnostics.dma_errors;
        busy_ = false;
    }
    bool Busy() const
    {
        return busy_;
    }
    ImuAcquisitionDiagnostics diagnostics;

  private:
    SampleStamp pending_[4];
    bool waiting_[4]; // 序号允许 uint32_t 回绕到 0，不能拿序号 0 充当空槽。
    RawImuPacket active_;
    bool busy_, corrupt_;
};
} // namespace lower_controller
#endif
