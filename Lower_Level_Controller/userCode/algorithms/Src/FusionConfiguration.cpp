#include "FusionConfiguration.h"
#include "FusionMath.h"

namespace lower_controller
{
namespace
{
FusionConfiguration BuildConfiguration()
{
    FusionConfiguration c = {};
    for (unsigned i = 0; i < 3; ++i)
    {
        c.imu_to_body[i][i] = i == 1 ? 1.0f : -1.0f;
    }
    // RoboMaster official C-board INS_task.c: BMI -> board = Rz(-90 deg), IST -> board = I.
    // The decoders here retain sensor axes. Therefore IST -> BMI = Rz(+90 deg),
    // and IST -> body must compose this board-internal rotation with the vehicle installation.
    const float mag_to_imu[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            for (unsigned k = 0; k < 3; ++k)
                c.mag_to_body[i][j] += c.imu_to_body[i][k] * mag_to_imu[k][j];
    c.accel_to_down_sign = -1.0f;
    c.installation_configured = true; // 已按本次要求选用原版安装假设，不等同于实测标定。
    c.magnetic_reference_confirmed = false;
    c.magnetic_reference_from_startup = true; // 学习本次启动的当地磁场，yaw 是相对航向。
    // 保留旧 roll=-gyro_x、pitch=+gyro_y；为保持右手旋转，yaw=-gyro_z。
    // 采用 C 板原始 +Z 朝上的安装假设，R=diag(-1,+1,-1)，det(R)=+1。
    // 官方 STEP/手册确认 36x36 mm、孔径2.5 mm，与仓库简化C板一致。
    // 简化块没有接口特征，绕板法线转180度仍可匹配；整板朝向沿用旧控制安装约定。
    // TODO(IMU_INSTALLATION)：孔位不能唯一确定整板朝向；见 docs/FINAL_INTEGRATION_20260916.md。
    c.pressure_model = PressureModel::MS5837_30BA; // 用户确认，且 Structure BOM C17 一致。
    c.water_density_kg_m3 = 1000.0f; // 初始淡水参数；海水或实测密度必须显式更改。
    c.gravity_m_s2 = 9.80665f;
    c.magnetic_reference_n[0] = 1.0f;
    c.magnetic_reference_uT = 50.0f;
    c.gyro_noise_rad_sqrt_s = 0.01f;
    c.bias_noise_rad_s_sqrt_s = 0.001f;
    c.accel_direction_sigma = 0.03f;
    c.mag_direction_sigma = 0.05f;
    for (unsigned i = 0; i < 4; ++i)
        c.pressure_covariance_m2[i][i] = 0.02f * 0.02f;
    c.accel_norm_gate_m_s2 = 1.5f;
    c.mag_norm_gate_uT = 15.0f;
    c.pressure_jump_margin_m = 0.05f;
    c.pressure_max_rate_m_s = 1.0f;
    c.pressure_min_m = -1.0f;
    c.pressure_max_m = 300.0f;
    c.pressure_attitude_min_depth_m = 0.05f; // 所有参与通道至少 5 cm 水头后才融合压差姿态。
    c.maximum_correction_rad = 0.35f;
    // TODO(HW): validate against logged rejection rates; 250 ms coast, 10-degree RMS bound.
    c.maximum_tilt_coast_us = 250000U;
    c.maximum_tilt_variance_rad2 = 0.030461742f;
    c.reorder_delay_us = 8000U;
    c.maximum_gyro_gap_us = 10000U;
    c.maximum_pressure_skew_us = 2000U;
    // Structure/zh-CN/外形.STEP：压力传感器接口中心，已按装配变换核对。
    // 接口圆柱 #90932：局部圆心 (2.452381722,8.974201395,15.415698431) mm。
    // 经四个装配变换，口沿中心跨度为 249.4 × 138.2 mm；原点选四口中心。
    // V3.3设计.pdf p7 给出编号：1前左、2后左、3后右、4前右，依次接 TCA0..3。
    // 此图替代此前从旧 site 推定的前后顺序。前=半球罩，右=面向前方的右侧。
    // 2026-09-16 用户明确以连接口作为位置；此几何项已确认，不再等待膜片偏移实测。
    // 电气通道仍按上方 V3.3 接线表；STEP 几何本身不包含线束去向。
    // 用户指定质心/浮心共处几何中心：以四口平均点为工程几何参考中心，偏置取0。
    // 当前PWM单位控制不另建质心/浮心恢复力矩或牛顿推力模型；原浮力PWM预置保留。
    const float position[4][3] = {{0.1247f,-0.0691f,0}, {-0.1247f,-0.0691f,0},
                                {-0.1247f,0.0691f,0}, {0.1247f,0.0691f,0}};
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned j = 0; j < 3; ++j) c.pressure_position_m[i][j] = position[i][j];
    // 液面参考在上电自动校准成功后填写；不得用固定大气压跳过实际采样。
    return c;
}
FusionConfiguration &MutableConfiguration()
{
    static FusionConfiguration configuration = BuildConfiguration();
    return configuration;
}
bool RotationValid(const float r[3][3])
{
    using namespace fusion_math;
    if (!Finite(&r[0][0], 9))
        return false;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            if (fabsf(Dot(r[i], r[j]) - (i == j ? 1.0f : 0.0f)) > 1e-3f)
                return false;
    const float determinant = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                              r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                              r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    return fabsf(determinant - 1.0f) < 1e-3f; // 拒绝镜像换轴；旋转必须保持右手关系。
}
} // namespace
const FusionConfiguration &GetFusionConfiguration()
{
    return MutableConfiguration();
}
bool SetStartupSurfacePressure(const float pressure_pa[4])
{
    for (unsigned i = 0; i < 4; ++i)
        if (!isfinite(pressure_pa[i]) || pressure_pa[i] < 80000 || pressure_pa[i] > 120000)
            return false;
    auto &c = MutableConfiguration();
    for (unsigned i = 0; i < 4; ++i) c.surface_pressure_pa[i] = pressure_pa[i];
    c.pressure_calibration_confirmed = true;
    return true;
}
bool ValidateFusionConfiguration(const FusionConfiguration &c)
{
    using namespace fusion_math;
    float l[16];
    return (c.pressure_model == PressureModel::Unconfirmed ||
            c.pressure_model == PressureModel::MS5837_02BA ||
            c.pressure_model == PressureModel::MS5837_30BA) &&
           RotationValid(c.imu_to_body) && RotationValid(c.mag_to_body) &&
           (c.accel_to_down_sign == 1 || c.accel_to_down_sign == -1) &&
           Finite(&c.pressure_position_m[0][0], 12) && Finite(c.surface_pressure_pa, 4) &&
           c.water_density_kg_m3 >= 900 && c.water_density_kg_m3 <= 1300 && c.gravity_m_s2 > 9 &&
           c.gravity_m_s2 < 11 && c.gyro_noise_rad_sqrt_s > 0 && isfinite(c.gyro_noise_rad_sqrt_s) &&
           c.bias_noise_rad_s_sqrt_s > 0 && isfinite(c.bias_noise_rad_s_sqrt_s) &&
           c.accel_direction_sigma > 0 && isfinite(c.accel_direction_sigma) && c.mag_direction_sigma > 0 &&
           isfinite(c.mag_direction_sigma) && c.accel_norm_gate_m_s2 > 0 &&
           isfinite(c.accel_norm_gate_m_s2) && c.mag_norm_gate_uT > 0 && isfinite(c.mag_norm_gate_uT) &&
           c.pressure_jump_margin_m > 0 && isfinite(c.pressure_jump_margin_m) &&
           c.pressure_max_rate_m_s >= 0 && isfinite(c.pressure_max_rate_m_s) && isfinite(c.pressure_min_m) &&
           isfinite(c.pressure_max_m) && c.pressure_min_m < c.pressure_max_m &&
           isfinite(c.pressure_attitude_min_depth_m) && c.pressure_attitude_min_depth_m >= 0 &&
           c.pressure_attitude_min_depth_m <= 1.0f &&
           c.maximum_tilt_coast_us >= 20000U && c.maximum_tilt_coast_us <= 500000U &&
           isfinite(c.maximum_tilt_variance_rad2) && c.maximum_tilt_variance_rad2 > 0 &&
           c.maximum_tilt_variance_rad2 <= 0.1f &&
           c.maximum_correction_rad > 0 && c.maximum_correction_rad <= 0.5f && c.reorder_delay_us >= 1000 &&
           c.reorder_delay_us <= 12000 && c.maximum_gyro_gap_us >= 2000 && c.maximum_gyro_gap_us <= 20000 &&
           c.maximum_pressure_skew_us > 0 && c.maximum_pressure_skew_us <= 5000 &&
           Finite(c.magnetic_reference_n, 3) && Norm(c.magnetic_reference_n) > 0.5f &&
           c.magnetic_reference_uT > 0 && isfinite(c.magnetic_reference_uT) &&
           Cholesky(&c.pressure_covariance_m2[0][0], l, 4);
}
bool FusionConfigurationReady(const FusionConfiguration &c)
{
    if (!ValidateFusionConfiguration(c) || !c.installation_configured || !c.pressure_calibration_confirmed ||
        c.pressure_model == PressureModel::Unconfirmed)
        return false;
    float max_area = 0;
    for (unsigned i = 1; i < 4; ++i)
        for (unsigned j = i + 1; j < 4; ++j)
        {
            const float x1 = c.pressure_position_m[i][0] - c.pressure_position_m[0][0];
            const float y1 = c.pressure_position_m[i][1] - c.pressure_position_m[0][1];
            const float x2 = c.pressure_position_m[j][0] - c.pressure_position_m[0][0];
            const float y2 = c.pressure_position_m[j][1] - c.pressure_position_m[0][1];
            max_area = fmaxf(max_area, fabsf(x1 * y2 - x2 * y1));
        }
    for (unsigned i = 0; i < 4; ++i)
        if (c.surface_pressure_pa[i] < 80000 || c.surface_pressure_pa[i] > 120000)
            return false;
    return max_area > 1e-5f; // 拒绝重合/共线的阵列；当前接口坐标已由 STEP 核对。
}
} // namespace lower_controller
