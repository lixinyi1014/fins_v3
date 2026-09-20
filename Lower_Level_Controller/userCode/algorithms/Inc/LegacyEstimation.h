/*
 * Small adapters around the unchanged legacy estimation equations.
 * 对旧估计公式建立独立输入输出接口；公式、运算顺序和 Mahony 参数均保留。
 */
#ifndef LOWER_LEGACY_ESTIMATION_H
#define LOWER_LEGACY_ESTIMATION_H

#include <math.h>
#include "LegacyControlData.h"
#include "MahonyAHRS.h"

namespace lower_controller {

/* q is scalar-first [w,x,y,z] and is the caller-owned persistent state.
 * q 按 [w,x,y,z] 排列，由调用者保存；旧 Mahony 含共享积分状态，仅用于现有单 IMU。
 * This function performs no peripheral access, logging or time reads.
 * 本函数只计算，不访问外设、不打印、不读取系统时间。 */
inline LegacyAttitudeResult EstimateLegacyAttitude(const ImuMeasurement& sample, float q[4])
{
    MahonyAHRSupdate(q,
        sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2], // x/y/z，rad/s
        sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2], // x/y/z，m/s^2
        sample.mag_uT[0], sample.mag_uT[1], sample.mag_uT[2]); // x/y/z，标定后的 uT 数值

    LegacyAttitudeResult result = {};
    result.yaw_rad = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    result.pitch_rad = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    result.roll_rad = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
    for (int i = 0; i < 3; ++i) result.gyro_rad_s[i] = sample.gyro_rad_s[i];
    return result;
}

/* Preserve the original mean and pressure-difference ordering exactly.
 * 按原有加减顺序求均值和压差；roll/pitch 差分不是姿态角。 */
inline LegacyPressureFeedback EstimateLegacyPressure(const float pressure[LC_PRESSURE_COUNT],
                                                      bool outer_loop_due, uint32_t sequence)
{
    LegacyPressureFeedback result = {};
    result.depth_legacy = (pressure[3] + pressure[2] + pressure[1] + pressure[0]) / 4;
    result.roll_difference_legacy = pressure[0] + pressure[1] - pressure[2] - pressure[3];
    result.pitch_difference_legacy = pressure[0] + pressure[3] - pressure[1] - pressure[2];
    result.outer_loop_due = outer_loop_due;
    result.measurement_sequence = sequence;
    return result;
}

/* Existing integer clamp/rounding for the eight thrusters.
 * 保留推进器的整数限幅及四舍五入；不要与舵机原有的 floor 换算混用。 */
inline int32_t LegacyThrusterPwmCount(int32_t pulse_us)
{
    if (pulse_us < LC_THRUSTER_MIN_US) pulse_us = LC_THRUSTER_MIN_US; // 下限 1000 us
    else if (pulse_us > LC_THRUSTER_MAX_US) pulse_us = LC_THRUSTER_MAX_US; // 上限 2000 us
    return (pulse_us * LC_PWM_COUNTS + LC_PWM_PERIOD_US / 2) / LC_PWM_PERIOD_US; // 4096 计数 / 20000 us，半周期值 10000 用于取整
}

} // namespace lower_controller
#endif
