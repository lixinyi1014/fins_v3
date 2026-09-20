#ifndef SENSOR_FUSION_DATA_H
#define SENSOR_FUSION_DATA_H

#include <stdint.h>

namespace lower_controller
{

enum class SensorKind : uint8_t
{
    Gyroscope,
    Accelerometer,
    Magnetometer,
    Temperature,
    Pressure
};

/* Sampling time is distinct from delivery time. 采样时刻与交付时刻分开记录。
 * IMU 的 sample_us 为 DRDY 中断入口时间；水压为转换窗口中点估计，均不是硬件同步时钟。 */
struct SampleStamp
{
    uint32_t sample_us;
    uint32_t received_us;
    uint32_t sequence;
};

struct RawImuPacket
{
    SensorKind kind;
    SampleStamp stamp;
    uint32_t read_started_us;
    uint8_t bytes[12]; // 加速度：命令+dummy+6 字节三轴+3 字节 sensor time，共 11 字节。
};

struct PressureArraySample
{
    SampleStamp stamp;
    float pressure_pa[4];
    uint32_t channel_sample_us[4];
    uint8_t valid_mask; // bit 0..3 对应 TCA 0..3；仅本次通信/补偿有效的通道置 1。
    uint32_t calibration_epoch;
};

struct FusionObservation
{
    SensorKind kind;
    SampleStamp stamp;
    union
    {
        float vector[3]; // gyro: rad/s；accel: 向下重力方向对应的 m/s²；mag: uT。
        PressureArraySample pressure;
    } data;
};

enum FusionFlags : uint32_t
{
    FusionAttitudeValid = 1U << 0,
    FusionDepthValid = 1U << 1,
    FusionHeadingObserved = 1U << 2,
    FusionConfigurationReadyFlag = 1U << 3,
    FusionGyroContinuous = 1U << 4
};

/* SI state at the estimator time, not at the time the control task reads it.
 * SI 状态对应 source_us：FRD 机体系、NED 世界系；四元数由机体转到世界，[w,x,y,z]。 */
struct FusionState
{
    float quaternion_bn[4];
    float gyro_bias_rad_s[3];
    float body_rate_rad_s[3];
    float euler_rad[3]; // roll、pitch、yaw，均为 rad；与旧 ins_angle 的顺序不同。
    float depth_m;
    uint32_t source_us;
    uint32_t gyro_sample_us;
    uint32_t pressure_sample_us;
    uint32_t published_us;
    uint32_t sequence;
    uint32_t flags;
    uint8_t pressure_mask;
    uint8_t pressure_attitude_rank;
};

struct FusionDiagnostics
{
    uint32_t predictions, accel_updates, mag_updates, pressure_updates;
    uint32_t accel_rejected, mag_rejected, pressure_rejected, pressure_channel_rejected;
    uint32_t late_observations, duplicate_observations, event_overflows, gyro_gaps;
    uint32_t numerical_failures, configuration_rejections;
    float last_accel_nis, last_mag_nis, last_pressure_nis;
};

} // namespace lower_controller
#endif
