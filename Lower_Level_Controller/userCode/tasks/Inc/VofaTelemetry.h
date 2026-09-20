#ifndef LC_VOFA_TELEMETRY_H
#define LC_VOFA_TELEMETRY_H
#include "SensorFusionData.h"
#include "FusionConfiguration.h"
#include <math.h>
#include <stdio.h>

namespace lower_controller
{
// FireWater 固定 16 通道，整行入队；文本回执使用 '='，避免冒号被当成另一组曲线。
// 0..7: roll_deg,pitch_deg,yaw_deg,depth_m,p0_m,p1_m,p2_m,p3_m
// 8..15: STOP,READY,CAL,RAW_MASK,FAULT,FUSED_MASK,FLAGS,SEQ(0..65535)
// -9999 是缺测/过期占位，绝不是测得的深度或角度。SEQ 不动表示显示已不再更新。
inline float VofaValue(float value, bool valid, float low, float high)
{
    return valid && isfinite(value) && value >= low && value <= high ? value : -9999.0f;
}
inline int FormatVofaTelemetry(char *packet, size_t capacity, const FusionState &state,
                              bool frame_valid, const PressureArraySample &pressure,
                              uint32_t now, bool stopped, bool ready, bool calibrated,
                              bool fault, uint16_t sequence)
{
    const auto &c = GetFusionConfiguration();
    const bool fresh = frame_valid && now - state.published_us <= 20000U;
    const bool attitude_ok = fresh && (state.flags & FusionAttitudeValid) &&
                             now - state.gyro_sample_us <= 20000U;
    const bool depth_ok = fresh && (state.flags & FusionDepthValid) &&
                          now - state.pressure_sample_us <= 50000U;
    uint8_t raw_mask = pressure.stamp.sequence && now - pressure.stamp.sample_us <= 50000U
                          ? pressure.valid_mask & 15U : 0U;
    float depths[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!isfinite(pressure.pressure_pa[i]) || now - pressure.channel_sample_us[i] > 50000U)
            raw_mask &= uint8_t(~(1U << i));
        depths[i] = VofaValue((pressure.pressure_pa[i] - c.surface_pressure_pa[i]) /
                                 (c.water_density_kg_m3 * c.gravity_m_s2),
                             calibrated && (raw_mask & (1U << i)), c.pressure_min_m, c.pressure_max_m);
    }
    constexpr float degrees = 57.2957795131f;
    const int length = snprintf(packet, capacity,
        "vofa:%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%lu,%u\r\n",
        double(VofaValue(state.euler_rad[0] * degrees, attitude_ok, -180.01f, 180.01f)),
        double(VofaValue(state.euler_rad[1] * degrees, attitude_ok, -90.01f, 90.01f)),
        double(VofaValue(state.euler_rad[2] * degrees, attitude_ok, -180.01f, 180.01f)),
        double(VofaValue(state.depth_m, depth_ok, c.pressure_min_m, c.pressure_max_m)),
        double(depths[0]), double(depths[1]), double(depths[2]), double(depths[3]),
        unsigned(stopped), unsigned(ready && fresh), unsigned(calibrated), unsigned(raw_mask),
        unsigned(fault), unsigned(depth_ok ? state.pressure_mask : 0U),
        static_cast<unsigned long>(frame_valid ? state.flags : 0U), unsigned(sequence));
    return length > 0 && size_t(length) < capacity ? length : 0; // 不发送截断行。
}
}
#endif
