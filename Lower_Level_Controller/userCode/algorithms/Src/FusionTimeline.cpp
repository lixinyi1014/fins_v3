#include "FusionTimeline.h"
#include <string.h>

namespace lower_controller
{

FusionTimeline::FusionTimeline(const FusionConfiguration &config)
    : configuration_(config), estimator_(config), size_(0), late_(0), duplicates_(0), overflows_(0)
{
    memset(last_sequence_, 0, sizeof(last_sequence_));
    memset(sequence_seen_, 0, sizeof(sequence_seen_));
}

bool FusionTimeline::Push(const FusionObservation &observation)
{
    const unsigned kind = static_cast<unsigned>(observation.kind);
    if (kind >= 5 || observation.kind == SensorKind::Temperature)
        return false;
    if (static_cast<int32_t>(observation.stamp.received_us - observation.stamp.sample_us) < 0)
    {
        ++late_;
        return false;
    }
    if (sequence_seen_[kind] && static_cast<int32_t>(observation.stamp.sequence - last_sequence_[kind]) <= 0)
    {
        ++duplicates_;
        return false;
    }
    sequence_seen_[kind] = true;
    last_sequence_[kind] = observation.stamp.sequence;
    if (size_ == Capacity)
    {
        ++overflows_;
        return false;
    } // 满时不覆盖未消费的陀螺仪；记录丢失，由时间缺口检查降级。
    unsigned insert = size_;
    while (insert > 0 &&
           (static_cast<int32_t>(observation.stamp.sample_us - pending_[insert - 1].stamp.sample_us) < 0 ||
            (observation.stamp.sample_us == pending_[insert - 1].stamp.sample_us &&
             kind < static_cast<unsigned>(pending_[insert - 1].kind))))
    {
        pending_[insert] = pending_[insert - 1];
        --insert;
    }
    pending_[insert] = observation;
    ++size_;
    return true;
}

unsigned FusionTimeline::ProcessReady(uint32_t now_us, unsigned maximum_events)
{
    unsigned processed = 0;
    while (size_ && processed < maximum_events)
    {
        const int32_t age = static_cast<int32_t>(now_us - pending_[0].stamp.sample_us);
        if (age < 0 || uint32_t(age) < configuration_.reorder_delay_us)
            break;
        const FusionObservation observation = pending_[0];
        for (unsigned i = 1; i < size_; ++i)
            pending_[i - 1] = pending_[i];
        --size_;
        ++processed;
        if (estimator_.Initialized() &&
            static_cast<int32_t>(observation.stamp.sample_us - estimator_.TimeUs()) < 0)
        {
            ++late_;
            continue; // 不用过去的数据纠正现在的状态，也不把时间戳改成 now_us。
        }
        switch (observation.kind)
        {
        case SensorKind::Gyroscope:
            estimator_.ProcessGyroscope(observation.data.vector, observation.stamp.sample_us);
            break;
        case SensorKind::Accelerometer:
            estimator_.ProcessAccelerometer(observation.data.vector, observation.stamp.sample_us);
            break;
        case SensorKind::Magnetometer:
            estimator_.ProcessMagnetometer(observation.data.vector, observation.stamp.sample_us);
            break;
        case SensorKind::Pressure:
            estimator_.ProcessPressure(observation.data.pressure);
            break;
        case SensorKind::Temperature:
            break;
        }
    }
    return processed;
}

FusionDiagnostics FusionTimeline::Diagnostics() const
{
    FusionDiagnostics d = estimator_.Diagnostics();
    d.late_observations += late_;
    d.duplicate_observations += duplicates_;
    d.event_overflows += overflows_;
    return d;
}

} // namespace lower_controller
