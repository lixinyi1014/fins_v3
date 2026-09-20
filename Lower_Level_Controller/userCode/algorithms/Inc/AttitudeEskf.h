#ifndef ATTITUDE_ESKF_H
#define ATTITUDE_ESKF_H
#include "SensorFusionData.h"
#include "FusionConfiguration.h"

namespace lower_controller
{

/* Paper §5.7–5.9: nominal [q_bn,b_g], error [dtheta,db_g].
 * 论文姿态 ESKF：7 维名义状态、6 维误差状态；深度单独作姿态补偿加权估计。
 * 所有调用均由一个估计任务按采样时间排序，类内部不访问 HAL、队列或全局传感器。 */
class AttitudeEskf
{
  public:
    explicit AttitudeEskf(const FusionConfiguration &configuration);
    void Reset();
    bool Initialize(const float down_m_s2[3], const float *mag_uT, uint32_t time_us);
    bool ProcessGyroscope(const float rate_rad_s[3], uint32_t time_us);
    bool ProcessAccelerometer(const float down_m_s2[3], uint32_t time_us);
    bool ProcessMagnetometer(const float field_uT[3], uint32_t time_us);
    bool ProcessPressure(const PressureArraySample &pressure);
    FusionState State(uint32_t published_us) const;
    const float (*Covariance() const)[6]
    {
        return covariance_;
    }
    const FusionDiagnostics &Diagnostics() const
    {
        return diagnostics_;
    }
    uint32_t TimeUs() const
    {
        return time_us_;
    }
    bool Initialized() const
    {
        return initialized_;
    }

    // 观测模型独立公开，供有限差分测试核对右乘误差的雅可比符号。
    static void DirectionModel(const float quaternion[4], const float reference[3], float predicted[3],
                               float h[3][6]);
    static void PressureRow(const float quaternion[4], const float position_m[3], float &predicted,
                            float h[6]);

  private:
    bool AdvanceTo(uint32_t time_us);
    bool Correct(const float *residual, const float h[3][6], const float *noise, unsigned dimension,
                 float &nis);
    bool CorrectDirection(const float vector[3], const float reference[3], float sigma, float &nis);
    const FusionConfiguration &configuration_;
    float quaternion_[4], bias_[3], last_rate_[3], covariance_[6][6];
    float pressure_previous_m_[4];
    uint32_t pressure_previous_us_[4];
    bool pressure_previous_valid_[4];
    float depth_m_;
    float magnetic_reference_n_[3], magnetic_reference_uT_;
    bool magnetic_reference_ready_;
    uint32_t time_us_, last_gyro_us_, last_pressure_us_, last_mag_us_, sequence_, pressure_epoch_;
    uint32_t last_tilt_correction_us_;
    uint8_t pressure_mask_, pressure_rank_;
    bool initialized_, have_gyro_, depth_valid_, heading_observed_, gyro_continuous_, configuration_valid_;
    FusionDiagnostics diagnostics_;
};

} // namespace lower_controller
#endif
