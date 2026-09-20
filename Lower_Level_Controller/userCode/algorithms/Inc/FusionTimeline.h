#ifndef FUSION_TIMELINE_H
#define FUSION_TIMELINE_H
#include "AttitudeEskf.h"

namespace lower_controller
{

/* Bounded reordering, not historical replay. 有界排序，不是无限历史回放。
 * 等待 8 ms 给水压转换和总线读取留出交付时间；已落后于滤波状态的观测明确丢弃。
 * 所有内存固定，禁止在 1 kHz 路径上分配堆内存。 */
class FusionTimeline
{
  public:
    static const unsigned Capacity = 64;
    explicit FusionTimeline(const FusionConfiguration &configuration);
    bool Push(const FusionObservation &observation);
    unsigned ProcessReady(uint32_t now_us, unsigned maximum_events = 8);
    FusionState State(uint32_t now_us) const
    {
        return estimator_.State(now_us);
    }
    FusionDiagnostics Diagnostics() const;
    unsigned Pending() const
    {
        return size_;
    }
    AttitudeEskf &Estimator()
    {
        return estimator_;
    }

  private:
    const FusionConfiguration &configuration_;
    AttitudeEskf estimator_;
    FusionObservation pending_[Capacity];
    unsigned size_;
    uint32_t last_sequence_[5];
    bool sequence_seen_[5];
    uint32_t late_, duplicates_, overflows_;
};

} // namespace lower_controller
#endif
