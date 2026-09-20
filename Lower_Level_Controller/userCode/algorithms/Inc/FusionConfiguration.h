#ifndef FUSION_CONFIGURATION_H
#define FUSION_CONFIGURATION_H
#include <stdint.h>

namespace lower_controller
{

enum class PressureModel : uint8_t
{
    Unconfirmed,
    MS5837_02BA,
    MS5837_30BA
};

struct FusionConfiguration
{
    float imu_to_body[3][3];
    float mag_to_body[3][3];
    float accel_to_down_sign;        // 原始加速度为比力时取 -1；须与安装旋转一起核对。
    float pressure_position_m[4][3]; // 按 TCA 0..3，不套用论文 1..4 的物理顺序。
    float surface_pressure_pa[4];    // 每路已标定的液面参考值，不能直接复制旧 legacy 偏置。
    PressureModel pressure_model;
    float water_density_kg_m3, gravity_m_s2;
    float magnetic_reference_n[3], magnetic_reference_uT;
    float gyro_noise_rad_sqrt_s, bias_noise_rad_s_sqrt_s;
    float accel_direction_sigma, mag_direction_sigma;
    float pressure_covariance_m2[4][4]; // 原始深度观测联合协方差；压差协方差由 DΣDᵀ 生成。
    float accel_norm_gate_m_s2, mag_norm_gate_uT;
    float pressure_jump_margin_m, pressure_max_rate_m_s;
    float pressure_min_m, pressure_max_m;
    float pressure_attitude_min_depth_m; // 空气中不满足静水压差模型，只用 IMU 估计姿态。
    float maximum_correction_rad;
    uint32_t maximum_tilt_coast_us;
    float maximum_tilt_variance_rad2;
    uint32_t reorder_delay_us, maximum_gyro_gap_us, maximum_pressure_skew_us;
    bool installation_configured, pressure_calibration_confirmed, magnetic_reference_confirmed;
    bool magnetic_reference_from_startup; // 相对启动航向，不代表真北或完成硬/软铁实测标定。
};

const FusionConfiguration &GetFusionConfiguration();
// 仅在调度器启动前提交四路液面压力；调用者必须保证在空气中静止校准。
bool SetStartupSurfacePressure(const float pressure_pa[4]);
bool ValidateFusionConfiguration(const FusionConfiguration &config);
bool FusionConfigurationReady(const FusionConfiguration &config);

} // namespace lower_controller
#endif
