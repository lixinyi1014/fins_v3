#ifndef TASK_MESSAGES_H
#define TASK_MESSAGES_H
#include "LegacyControlData.h"
#include <stddef.h>
#include <stdlib.h>
#include "SensorFusionData.h"
#include "CommandValidation.h"

namespace lower_controller
{
struct ControlRelease
{
    uint32_t sequence;
    uint32_t released_us;
};
struct ImuAttitudeFrame
{
    ControlRelease release;
    LegacyAttitudeResult attitude;
#if LC_IMU_ASYNC_ENABLED
    FusionState fusion; // SI 状态；未确认配置时只能记录，不能启用新的实物闭环
#endif
    uint32_t completed_us;
    bool valid;
};
enum class PressurePwmOperation : uint8_t
{
    Pressure,
    Output,
    Calibrate
};
struct PressurePwmRequest
{
    PressurePwmOperation operation;
    uint32_t epoch;
    uint32_t released_us;
    bool allow_motion;
    ThrusterPwmCommand thrusters;
    ServoPwmCommand servos;
};
struct PressurePwmReply
{
    LegacyPressureFeedback pressure;
#if LC_IMU_ASYNC_ENABLED
    PressureArraySample physical_pressure; // VOFA 使用服务任务交回的副本，不跨任务读取正在改写的压力。
#endif
    uint32_t completed_us;
    bool ok;
    bool pressure_valid;
};
struct ReceivedCommandPacket
{
    uint32_t stop_epoch;
    uint32_t received_us; // 真实收包时刻，不能用出队时刻刷新旧命令的存活时间。
    uint16_t length;
    char data[100];
};
struct UartTxPacket
{
    uint16_t length;
    uint8_t data[LC_UART_TX_PACKET_SIZE];
};

inline bool SequenceAtOrAfter(uint32_t candidate, uint32_t reference)
{
    return static_cast<int32_t>(candidate - reference) >= 0;
}
inline bool IsFresh(uint32_t now_us, uint32_t sample_us)
{
    return now_us - sample_us <= LC_STATE_MAX_AGE_US; // 20000 us，允许 uint32_t 回绕
}
inline bool IsOffCommand(const char *data, size_t length)
{
    return length == 3 && data[0] == 'O' && data[1] == 'F' && data[2] == 'F';
}
inline bool IsArmCommand(const char *data, size_t length)
{
    if (length == 2 && data[0] == 'O' && data[1] == 'N') return true;
    if (length < 4 || data[0] != 'T' || data[1] != 'E' || data[2] != 'S' || data[3] != ':') return false;
    // 此函数接收 ReceivedCommandPacket 中已补零的字符串；只有完整 8 路有效测试值才能启动。
    const char *cursor = data + 4;
    for (unsigned i = 0; i < 8; ++i)
    {
        char *end;
        const long value = strtol(cursor, &end, 10);
        if (end == cursor || value < LC_THRUSTER_MIN_US || value > LC_THRUSTER_MAX_US) return false;
        if (i == 7) return *end == '\0';
        if (*end != ',') return false;
        cursor = end + 1;
    }
    return false;
}
inline LegacyControlFeedback MakeControlFeedback(const ImuAttitudeFrame &imu,
                                                 const LegacyPressureFeedback &pressure)
{
    LegacyControlFeedback feedback = {};
    feedback.depth_legacy = pressure.depth_legacy;                       // 旧深度刻度
    feedback.roll_difference_legacy = pressure.roll_difference_legacy;   // p0+p1-p2-p3
    feedback.pitch_difference_legacy = pressure.pitch_difference_legacy; // p0+p3-p1-p2
    feedback.outer_loop_due =
        pressure.outer_loop_due;             // true：温度已读取且压力转换已启动，外环使用上一完整水压帧
    feedback.yaw_rad = imu.attitude.yaw_rad; // rad
    feedback.roll_rate_feedback_rad_s = -imu.attitude.gyro_rad_s[0]; // -gyro_x，rad/s
    feedback.pitch_rate_feedback_rad_s = imu.attitude.gyro_rad_s[1]; // +gyro_y，rad/s
    feedback.yaw_rate_feedback_rad_s = imu.attitude.gyro_rad_s[2];   // +gyro_z，rad/s
    return feedback;
}
} // namespace lower_controller
#endif
