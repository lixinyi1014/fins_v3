/*
 * Data contracts for the existing algorithms. No HAL or scheduler dependency.
 * 现有算法的数据接口，不依赖 HAL 或调度器。Legacy 明确表示保留旧单位和符号。
 */
#ifndef LOWER_LEGACY_CONTROL_DATA_H
#define LOWER_LEGACY_CONTROL_DATA_H

#include <stdint.h>
#include "LowerControllerConfig.h"

namespace lower_controller {

/* A calibrated nine-axis input in the existing IMU axes.
 * 已标定的九轴输入，轴顺序仍是旧 IMU 的 x/y/z，尚未宣称完成 NED 安装对齐。
 * read_*_ms are HAL read-window timestamps, NOT physical sample timestamps.
 * read_*_ms 仅记录读取窗口，不能用于宣称传感器同步，也不改变 Mahony 的固定 dt。 */
struct ImuMeasurement {
    float gyro_rad_s[3];
    float accel_m_s2[3];
    float mag_uT[3];
    float temperature_c;
    uint32_t sequence;
    uint32_t read_started_ms;
    uint32_t read_completed_ms;
};

struct LegacyAttitudeResult {
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    float gyro_rad_s[3];
};

/* Temperature-compensated pressure in the ORIGINAL numerical scale.
 * 温补后压强保留原公式量纲；不把 legacy 数值重新标为 Pa、米或弧度。
 * prom_valid_mask only records PROM CRC status; it is not runtime fault detection.
 * prom_valid_mask 只表示 PROM 校验结果，不代表本次 I2C 读取成功或传感器健康。 */
struct PressureMeasurement {
    float pressure_legacy[LC_PRESSURE_COUNT];
    uint8_t prom_valid_mask;
    uint32_t sequence;
    uint32_t read_completed_ms;
};

struct LegacyPressureFeedback {
    float depth_legacy;
    float roll_difference_legacy;
    float pitch_difference_legacy;
    bool outer_loop_due;
    uint32_t measurement_sequence;
};

/* Feedback copied once at the controller boundary.
 * 控制边界一次性拷贝反馈；横滚角速度已按旧控制器取负，俯仰和偏航保持原符号。
 * outer_loop_due retains the old pressure-state PHASE, not a new-frame event.
 * outer_loop_due 保留旧外环相位，不能解释为“刚收到一帧新水压”。 */
struct LegacyControlFeedback {
    float depth_legacy;
    float roll_difference_legacy;
    float pitch_difference_legacy;
    float yaw_rad;
    float roll_rate_feedback_rad_s;
    float pitch_rate_feedback_rad_s;
    float yaw_rate_feedback_rad_s;
    bool outer_loop_due;
};

/* Controller output before the existing driver clamp, indexed by PCA channel.
 * 控制输出按 PCA 通道排列，保留原始脉宽；限幅和计数换算由输出函数完成。 */
struct ThrusterPwmCommand {
    int32_t pulse_us[LC_THRUSTER_COUNT];
};

struct ServoPwmCommand {
    int32_t channel[LC_SERVO_COUNT];
    int32_t pulse_us[LC_SERVO_COUNT];
    bool in_range;
};

} // namespace lower_controller
#endif
