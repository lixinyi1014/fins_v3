#ifndef LC_VOFA_TELEMETRY_H
#define LC_VOFA_TELEMETRY_H
#include "SensorFusionData.h"
#include "FusionConfiguration.h"
#include <math.h>
#include <stdio.h>

namespace lower_controller
{
// FireWater 固定 16 通道，整行入队；文本回执使用 '='，避免冒号被当成另一组曲线。
// 0..3: the four legacy pressure values, exactly the same unit, offset and
// truncation as the old OutputData()/IMU print path.  This is deliberately
// the first four fields so the old four-number VOFA view is unchanged.
// 4..7: ESKF roll/pitch/yaw/depth (SI/degrees, -9999 until the estimator is valid).
// 8..15: STOP,READY,CAL,RAW_MASK,FAULT,FUSED_MASK,ERROR_CODE,SEQ.
// ERROR_CODE bits: 0..7 last stop reason, 8..15 cumulative I2C2 errors,
// 16..31 ESKF flags.  This lets the serial VOFA view show the failure cause
// without a separate command.
// -9999 是缺测/过期占位，绝不是测得的深度或角度。SEQ 不动表示显示已不再更新。
inline float VofaValue(float value, bool valid, float low, float high)
{
    return valid && isfinite(value) && value >= low && value <= high ? value : -9999.0f;
}
inline float LegacyVofaValue(float value, bool valid)
{
    const float checked = VofaValue(value, valid, -1000.0f, 100000.0f);
    return checked == -9999.0f ? checked : truncf(checked * 100.0f) / 100.0f;
}
inline int FormatVofaTelemetry(char *packet, size_t capacity, const FusionState &state,
                              bool frame_valid, const PressureArraySample &pressure,
                              const float legacy_pressure[4], bool legacy_valid, uint32_t now,
                              bool stopped, bool ready, bool calibrated,
                              bool fault, uint32_t diagnostic_code, uint16_t sequence)
{
    const auto &c = GetFusionConfiguration();
    const bool fresh = frame_valid && now - state.published_us <= 20000U;
    const bool attitude_ok = fresh && (state.flags & FusionAttitudeValid) &&
                             now - state.gyro_sample_us <= 20000U;
    const bool depth_ok = fresh && (state.flags & FusionDepthValid) &&
                          now - state.pressure_sample_us <= 50000U;
    uint8_t raw_mask = legacy_valid ? 15U : 0U;
    float depths[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        depths[i] = LegacyVofaValue(legacy_pressure[i], raw_mask & (1U << i));
    }
    constexpr float degrees = 57.2957795131f;
    const int length = snprintf(packet, capacity,
        "vofa:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,%u,%u,%lu,%u\r\n",
        double(depths[0]), double(depths[1]), double(depths[2]), double(depths[3]),
        double(VofaValue(state.euler_rad[0] * degrees, attitude_ok, -180.01f, 180.01f)),
        double(VofaValue(state.euler_rad[1] * degrees, attitude_ok, -90.01f, 90.01f)),
        double(VofaValue(state.euler_rad[2] * degrees, attitude_ok, -180.01f, 180.01f)),
        double(VofaValue(state.depth_m, depth_ok, c.pressure_min_m, c.pressure_max_m)),
        unsigned(stopped), unsigned(ready && fresh), unsigned(calibrated), unsigned(raw_mask),
        unsigned(fault), unsigned(depth_ok ? state.pressure_mask : 0U),
        static_cast<unsigned long>(diagnostic_code), unsigned(sequence));
    return length > 0 && size_t(length) < capacity ? length : 0; // 不发送截断行。
}

// Source compatibility for host tools that still call the pre-legacy-display
// signature.  The board path above always passes data_pressure[] explicitly.
inline int FormatVofaTelemetry(char *packet, size_t capacity, const FusionState &state,
                              bool frame_valid, const PressureArraySample &pressure,
                              uint32_t now, bool stopped, bool ready, bool calibrated,
                              bool fault, uint32_t diagnostic_code, uint16_t sequence)
{
    const auto &c = GetFusionConfiguration();
    float legacy[4] = {};
    const bool valid = pressure.stamp.sequence && now - pressure.stamp.sample_us <= 50000U;
    for (unsigned i = 0; i < 4; ++i)
        legacy[i] = (pressure.pressure_pa[i] - c.surface_pressure_pa[i]) /
                    (c.water_density_kg_m3 * c.gravity_m_s2);
    return FormatVofaTelemetry(packet, capacity, state, frame_valid, pressure, legacy, valid,
                               now, stopped, ready, calibrated, fault, diagnostic_code, sequence);
}

// Keep the pre-diagnostic host/test call source-compatible.  Board code uses
// the overload above so the live VOFA frame carries the stop/error code.
inline int FormatVofaTelemetry(char *packet, size_t capacity, const FusionState &state,
                              bool frame_valid, const PressureArraySample &pressure,
                              uint32_t now, bool stopped, bool ready, bool calibrated,
                              bool fault, uint16_t sequence)
{
    return FormatVofaTelemetry(packet, capacity, state, frame_valid, pressure, now,
                               stopped, ready, calibrated, fault, 0U, sequence);
}
}
#endif
