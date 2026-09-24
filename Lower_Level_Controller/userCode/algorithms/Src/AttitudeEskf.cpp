#include "AttitudeEskf.h"
#include "FusionMath.h"
#include <string.h>

namespace lower_controller
{
using namespace fusion_math;

// 压力通道连续被拒多少帧之后，放弃冻住的旧基准、拿当前读数重新起头。
// 压力阵列帧率 50 Hz，25 帧约 0.5 s。本层刻意只依赖 <stdint.h>，
// 不引板级配置头，所以常量放在这里而不是 LowerControllerConfig.h。
static constexpr uint8_t kPressureRejectReseedFrames = 25U;

AttitudeEskf::AttitudeEskf(const FusionConfiguration &configuration) : configuration_(configuration)
{
    configuration_valid_ = ValidateFusionConfiguration(configuration_);
    memset(&diagnostics_, 0, sizeof(diagnostics_));
    Reset();
}

void AttitudeEskf::Reset()
{
    memset(quaternion_, 0, sizeof(quaternion_));
    quaternion_[0] = 1;
    memset(bias_, 0, sizeof(bias_));
    memset(last_rate_, 0, sizeof(last_rate_));
    memset(covariance_, 0, sizeof(covariance_));
    for (unsigned i = 0; i < 6; ++i)
        covariance_[i][i] = i < 3 ? 0.1f * 0.1f : 0.03f * 0.03f;
    memset(pressure_previous_valid_, 0, sizeof(pressure_previous_valid_));
    memset(pressure_reject_streak_, 0, sizeof(pressure_reject_streak_));
    depth_m_ = 0;
    memcpy(magnetic_reference_n_, configuration_.magnetic_reference_n, sizeof(magnetic_reference_n_));
    magnetic_reference_uT_ = configuration_.magnetic_reference_uT;
    magnetic_reference_ready_ = configuration_.magnetic_reference_confirmed;
    time_us_ = last_gyro_us_ = last_pressure_us_ = last_mag_us_ = sequence_ = pressure_epoch_ = 0;
    last_tilt_correction_us_ = 0;
    pressure_mask_ = pressure_rank_ = 0;
    initialized_ = have_gyro_ = depth_valid_ = heading_observed_ = gyro_continuous_ = false;
    // 复位状态不会抹掉累计诊断，便于看到多次失锁/重对准。
}

/** @brief Coarse alignment before small-error filtering. 小误差滤波前先粗对准。
 * @param down_m_s2 已统一为“向下”的加速度观测，m/s²；不接受明显平动加速度。
 * @param mag_uT 可为空；磁场参考未确认时只对准横滚/俯仰，偏航零点不代表地理北。 */
bool AttitudeEskf::Initialize(const float down_m_s2[3], const float *mag_uT, uint32_t time_us)
{
    if (!configuration_valid_ || !Finite(down_m_s2, 3) ||
        fabsf(Norm(down_m_s2) - configuration_.gravity_m_s2) > configuration_.accel_norm_gate_m_s2)
        return false;
    const float roll = atan2f(down_m_s2[1], down_m_s2[2]);
    const float pitch =
        atan2f(-down_m_s2[0], sqrtf(down_m_s2[1] * down_m_s2[1] + down_m_s2[2] * down_m_s2[2]));
    float yaw = 0;
    heading_observed_ = false;
    if (mag_uT && configuration_.magnetic_reference_confirmed && Finite(mag_uT, 3) &&
        fabsf(Norm(mag_uT) - configuration_.magnetic_reference_uT) <= configuration_.mag_norm_gate_uT)
    {
        float tilt[4], r[3][3], world[3];
        EulerQuaternion(roll, pitch, 0, tilt);
        QuaternionMatrix(tilt, r);
        Rotate(r, mag_uT, world);
        const float *ref = configuration_.magnetic_reference_n;
        if (hypotf(world[0], world[1]) > 1e-3f && hypotf(ref[0], ref[1]) > 1e-3f)
        {
            yaw = atan2f(ref[1], ref[0]) - atan2f(world[1], world[0]);
            heading_observed_ = true;
            last_mag_us_ = time_us;
        }
    }
    EulerQuaternion(roll, pitch, yaw, quaternion_);
    Normalize(quaternion_, 4);
    time_us_ = time_us;
    initialized_ = true;
    last_tilt_correction_us_ = time_us;
    ++sequence_;
    if (!heading_observed_)
        covariance_[2][2] = 1.0f; // 无磁粗对准时保留较大的航向不确定性。
    return true;
}

/** @brief Propagate to the event time using held gyro input. 用上一个陀螺仪输入传播到事件时间。
 * 事件间采用零阶保持；Phi 左上为 I-[omega]x*dt，右上为 -I*dt。
 * dt 来自微秒时间戳；不是把 1/150 s 偷换成另一固定步长。 */
bool AttitudeEskf::AdvanceTo(uint32_t time_us)
{
    if (!initialized_)
        return false;
    const int32_t interval = static_cast<int32_t>(time_us - time_us_);
    if (interval < 0)
    {
        ++diagnostics_.late_observations;
        return false;
    }
    if (interval == 0)
        return true;
    if (!have_gyro_ || uint32_t(time_us - last_gyro_us_) > configuration_.maximum_gyro_gap_us)
        return false;
    const float dt = float(interval) * 1e-6f;
    float omega[3], increment[3], delta_q[4], candidate_q[4];
    for (unsigned i = 0; i < 3; ++i)
    {
        omega[i] = last_rate_[i] - bias_[i];
        increment[i] = omega[i] * dt;
    }
    RotationVectorQuaternion(increment, delta_q);
    QuaternionProduct(quaternion_, delta_q, candidate_q);
    float phi[6][6] = {}, skew[3][3];
    Skew(omega, skew);
    for (unsigned i = 0; i < 6; ++i)
        phi[i][i] = 1;
    for (unsigned i = 0; i < 3; ++i)
    {
        phi[i][i + 3] = -dt;
        for (unsigned j = 0; j < 3; ++j)
            phi[i][j] -= skew[i][j] * dt;
    }
    float ap[6][6] = {}, next[6][6] = {};
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            for (unsigned k = 0; k < 6; ++k)
                ap[i][j] += phi[i][k] * covariance_[k][j];
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            for (unsigned k = 0; k < 6; ++k)
                next[i][j] += ap[i][k] * phi[j][k];
    const float gyro_q = configuration_.gyro_noise_rad_sqrt_s * configuration_.gyro_noise_rad_sqrt_s;
    const float bias_q = configuration_.bias_noise_rad_s_sqrt_s * configuration_.bias_noise_rad_s_sqrt_s;
    for (unsigned i = 0; i < 3; ++i)
    {
        next[i][i] += gyro_q * dt + bias_q * dt * dt * dt / 3;
        next[i + 3][i + 3] += bias_q * dt;
        next[i][i + 3] -= bias_q * dt * dt / 2;
        next[i + 3][i] -= bias_q * dt * dt / 2;
    }
    if (!Normalize(candidate_q, 4) || !Finite(&next[0][0], 36))
    {
        ++diagnostics_.numerical_failures;
        return false;
    }
    memcpy(quaternion_, candidate_q, sizeof(quaternion_));
    memcpy(covariance_, next, sizeof(covariance_));
    time_us_ = time_us;
    ++diagnostics_.predictions;
    ++sequence_;
    return true;
}

bool AttitudeEskf::ProcessGyroscope(const float rate_rad_s[3], uint32_t time_us)
{
    if (!Finite(rate_rad_s, 3) || Norm(rate_rad_s) > 40.0f)
        return false; // 排除 NaN/Inf 和超出 ±2000°/s 量级的输入。
    if (have_gyro_ && static_cast<int32_t>(time_us - last_gyro_us_) <= 0)
    {
        ++diagnostics_.duplicate_observations;
        return false;
    }
    if (have_gyro_ && time_us - last_gyro_us_ > configuration_.maximum_gyro_gap_us)
    {
        ++diagnostics_.gyro_gaps;
        Reset(); // 长缺口不能靠保持旧角速度伪装成有效姿态；下一份加速度重新粗对准。
    }
    if (initialized_ && have_gyro_ && !AdvanceTo(time_us))
        return false;
    if (initialized_ && !have_gyro_)
        time_us_ = time_us;
    memcpy(last_rate_, rate_rad_s, sizeof(last_rate_));
    last_gyro_us_ = time_us;
    have_gyro_ = true;
    gyro_continuous_ = true;
    return true;
}

void AttitudeEskf::DirectionModel(const float q[4], const float reference[3], float predicted[3],
                                  float h[3][6])
{
    float r[3][3], skew[3][3];
    QuaternionMatrix(q, r);
    memset(h, 0, sizeof(float) * 18);
    for (unsigned i = 0; i < 3; ++i)
        predicted[i] = r[0][i] * reference[0] + r[1][i] * reference[1] + r[2][i] * reference[2];
    Skew(predicted, skew);
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            h[i][j] = skew[i][j]; // 右乘误差：H=[h]x，而不是负号。
}

void AttitudeEskf::PressureRow(const float q[4], const float position[3], float &predicted, float h[6])
{
    float r[3][3], skew[3][3];
    QuaternionMatrix(q, r);
    Skew(position, skew);
    predicted = Dot(r[2], position);
    memset(h, 0, sizeof(float) * 6);
    for (unsigned j = 0; j < 3; ++j)
        for (unsigned k = 0; k < 3; ++k)
            h[j] -= r[2][k] * skew[k][j];
    // 近水平时 H=[y,-x,0,0,0,0]；后三列为零不代表零偏永远不可校正。
}

/** @brief One sequential observation correction. 对当前中间状态做一次序贯校正。
 * 先计算 NIS，再用 Joseph 形式更新协方差；大修正、奇异创新矩阵均拒绝。
 * 误差四元数右乘注入，并用 I-[dtheta]x/2 重置局部误差协方差坐标。 */
bool AttitudeEskf::Correct(const float *residual, const float h[3][6], const float *noise, unsigned n,
                           float &nis)
{
    if (n == 0 || n > 3 || !Finite(residual, n) || !Finite(noise, n * n))
        return false;
    float ph[6][3] = {}, innovation[9] = {}, l[9] = {}, solved[3] = {};
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < n; ++j)
            for (unsigned k = 0; k < 6; ++k)
                ph[i][j] += covariance_[i][k] * h[j][k];
    for (unsigned i = 0; i < n; ++i)
        for (unsigned j = 0; j < n; ++j)
        {
            innovation[i * n + j] = noise[i * n + j];
            for (unsigned k = 0; k < 6; ++k)
                innovation[i * n + j] += h[i][k] * ph[k][j];
        }
    if (!Cholesky(innovation, l, n))
    {
        ++diagnostics_.numerical_failures;
        return false;
    }
    SolveCholesky(l, residual, solved, n);
    nis = 0;
    for (unsigned i = 0; i < n; ++i)
        nis += residual[i] * solved[i];
    const float nis_limit[3] = {10.828f, 13.816f,
                                16.266f}; // 1/2/3 维，初始选用卡方 99.9% 门限，需用实测噪声复核。
    if (!isfinite(nis) || nis > nis_limit[n - 1])
        return false;
    float gain[6][3] = {}, error[6] = {};
    for (unsigned i = 0; i < 6; ++i)
    {
        SolveCholesky(l, ph[i], gain[i], n);
        for (unsigned j = 0; j < n; ++j)
            error[i] += gain[i][j] * residual[j];
    }
    if (!Finite(error, 6) || Norm(error) > configuration_.maximum_correction_rad)
        return false;
    float a[6][6] = {}, ap[6][6] = {}, next[6][6] = {};
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
        {
            a[i][j] = i == j ? 1.0f : 0.0f;
            for (unsigned k = 0; k < n; ++k)
                a[i][j] -= gain[i][k] * h[k][j];
        }
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            for (unsigned k = 0; k < 6; ++k)
                ap[i][j] += a[i][k] * covariance_[k][j];
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
        {
            for (unsigned k = 0; k < 6; ++k)
                next[i][j] += ap[i][k] * a[j][k];
            for (unsigned k = 0; k < n; ++k)
                for (unsigned m = 0; m < n; ++m)
                    next[i][j] += gain[i][k] * noise[k * n + m] * gain[j][m];
        }
    float reset[6][6] = {}, skew[3][3];
    Skew(error, skew);
    for (unsigned i = 0; i < 6; ++i)
        reset[i][i] = 1;
    for (unsigned i = 0; i < 3; ++i)
        for (unsigned j = 0; j < 3; ++j)
            reset[i][j] -= 0.5f * skew[i][j];
    memset(ap, 0, sizeof(ap));
    memset(a, 0, sizeof(a));
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            for (unsigned k = 0; k < 6; ++k)
                ap[i][j] += reset[i][k] * next[k][j];
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            for (unsigned k = 0; k < 6; ++k)
                a[i][j] += ap[i][k] * reset[j][k];
    for (unsigned i = 0; i < 6; ++i)
        for (unsigned j = 0; j < 6; ++j)
            next[i][j] = 0.5f * (a[i][j] + a[j][i]);
    float check[36], candidate[4], dq[4];
    RotationVectorQuaternion(error, dq);
    QuaternionProduct(quaternion_, dq, candidate);
    if (!Normalize(candidate, 4) || !Cholesky(&next[0][0], check, 6))
    {
        ++diagnostics_.numerical_failures;
        return false;
    }
    memcpy(quaternion_, candidate, sizeof(quaternion_));
    memcpy(covariance_, next, sizeof(covariance_));
    for (unsigned i = 0; i < 3; ++i)
        bias_[i] += error[i + 3];
    ++sequence_;
    return true;
}

bool AttitudeEskf::CorrectDirection(const float vector[3], const float reference[3], float sigma, float &nis)
{
    float unit[3] = {vector[0], vector[1], vector[2]}, ref[3] = {reference[0], reference[1], reference[2]};
    if (!Normalize(unit, 3) || !Normalize(ref, 3))
        return false;
    float predicted[3], h[3][6], residual[3], noise[9] = {};
    DirectionModel(quaternion_, ref, predicted, h);
    for (unsigned i = 0; i < 3; ++i)
    {
        residual[i] = unit[i] - predicted[i];
        noise[i * 3 + i] = sigma * sigma;
    }
    return Correct(residual, h, noise, 3, nis);
}

bool AttitudeEskf::ProcessAccelerometer(const float down[3], uint32_t time_us)
{
    if (!Finite(down, 3) ||
        fabsf(Norm(down) - configuration_.gravity_m_s2) > configuration_.accel_norm_gate_m_s2)
    {
        ++diagnostics_.accel_rejected;
        return false;
    }
    if (!initialized_)
        return Initialize(down, nullptr, time_us);
    const float reference[3] = {0, 0, 1};
    if (!AdvanceTo(time_us) ||
        !CorrectDirection(down, reference, configuration_.accel_direction_sigma, diagnostics_.last_accel_nis))
    {
        ++diagnostics_.accel_rejected;
        return false;
    }
    last_tilt_correction_us_ = time_us;
    ++diagnostics_.accel_updates;
    return true;
}

bool AttitudeEskf::ProcessMagnetometer(const float field[3], uint32_t time_us)
{
    if (!Finite(field, 3) || !AdvanceTo(time_us))
    {
        ++diagnostics_.mag_rejected;
        return false;
    }
    const float magnitude = Norm(field);
    if (!magnetic_reference_ready_)
    {
        if (!configuration_.magnetic_reference_from_startup || magnitude < 5 || magnitude > 100)
        {
            ++diagnostics_.mag_rejected;
            return false;
        }
        float rotation[3][3], world[3];
        QuaternionMatrix(quaternion_, rotation);
        Rotate(rotation, field, world);
        if (hypotf(world[0], world[1]) < 0.1f*magnitude)
        {
            ++diagnostics_.mag_rejected;
            return false; // 水平分量太小，不能提供可靠航向参考。
        }
        memcpy(magnetic_reference_n_, world, sizeof(world));
        Normalize(magnetic_reference_n_, 3);
        magnetic_reference_uT_ = magnitude;
        magnetic_reference_ready_ = true;
        // 参考绑定到启动时的世界系，不把未知磁倾角硬写成 [1,0,0]。
        // 这是相对航向基准，磁力计硬/软铁补偿仍沿用原代码。
    }
    if (fabsf(magnitude - magnetic_reference_uT_) > configuration_.mag_norm_gate_uT)
    {
        ++diagnostics_.mag_rejected;
        return false;
    }
    if (!heading_observed_)
    {
        float angles[3], tilt[4], rotation[3][3], world[3];
        QuaternionEuler(quaternion_, angles);
        EulerQuaternion(angles[0], angles[1], 0, tilt);
        QuaternionMatrix(tilt, rotation);
        Rotate(rotation, field, world);
        const float *reference = magnetic_reference_n_;
        if (hypotf(world[0], world[1]) < 1e-3f || hypotf(reference[0], reference[1]) < 1e-3f)
        {
            ++diagnostics_.mag_rejected;
            return false;
        }
        angles[2] = atan2f(reference[1], reference[0]) - atan2f(world[1], world[0]);
        EulerQuaternion(angles[0], angles[1], angles[2], quaternion_);
        // 首次航向粗对准后重建保守协方差，清交叉项；此时还没有允许偏航闭环。
        memset(covariance_, 0, sizeof(covariance_));
        for (unsigned i = 0; i < 6; ++i)
            covariance_[i][i] = i < 3 ? 0.01f : 0.0009f;
        ++sequence_;
    }
    else if (!CorrectDirection(field, magnetic_reference_n_, configuration_.mag_direction_sigma,
                               diagnostics_.last_mag_nis))
    {
        ++diagnostics_.mag_rejected;
        return false;
    }
    ++diagnostics_.mag_updates;
    heading_observed_ = true;
    last_mag_us_ = time_us;
    return true;
}

/** @brief Rebuild the pressure observation from its usable subset. 按本次可用子集重建水压观测。
 * 选第一个有效通道为参考，形成 m-1 条差；共享参考造成的非对角协方差必须保留。
 * 少于两点时不做水压姿态校正；一只有效水压计仍可在已有姿态下提供深度。 */
bool AttitudeEskf::ProcessPressure(const PressureArraySample &pressure)
{
    if (!FusionConfigurationReady(configuration_) || !AdvanceTo(pressure.stamp.sample_us))
    {
        ++diagnostics_.configuration_rejections;
        return false;
    }
    if (pressure.calibration_epoch != pressure_epoch_)
    {
        memset(pressure_previous_valid_, 0, sizeof(pressure_previous_valid_));
        memset(pressure_reject_streak_, 0, sizeof(pressure_reject_streak_));
        pressure_epoch_ = pressure.calibration_epoch;
        depth_valid_ = false;
    }
    unsigned ids[4], count = 0;
    float depth[4] = {};
    pressure_mask_ = pressure_rank_ = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!(pressure.valid_mask & (1U << i)))
            continue;
        const uint32_t t = pressure.channel_sample_us[i];
        depth[i] = (pressure.pressure_pa[i] - configuration_.surface_pressure_pa[i]) /
                   (configuration_.water_density_kg_m3 * configuration_.gravity_m_s2); // Pa/(kg/m³·m/s²)=m。
        /* 三道门分开记，否则只知道"通道被拒"，查不出是量程、采样偏斜还是跳变。
         * 现场表现完全不同：偏斜说明 I2C2 读一帧的时间被拉长（多半是推进器
         * 一转就出现的总线干扰），跳变说明潜器动得比门限快，量程说明零点或
         * 传感器本身有问题。 */
        const bool range_ok = isfinite(depth[i]) && depth[i] >= configuration_.pressure_min_m &&
                              depth[i] <= configuration_.pressure_max_m;
        const bool skew_ok = pressure.stamp.sample_us - t <= configuration_.maximum_pressure_skew_us;
        bool jump_ok = true;
        if (pressure_previous_valid_[i])
        {
            const int32_t elapsed = static_cast<int32_t>(t - pressure_previous_us_[i]);
            if (elapsed <= 0)
                jump_ok = false;
            else if (fabsf(depth[i] - pressure_previous_m_[i]) >
                     configuration_.pressure_jump_margin_m +
                         configuration_.pressure_max_rate_m_s * float(elapsed > 50000 ? 50000 : elapsed) * 1e-6f)
                jump_ok = false;
        }
        const bool valid = range_ok && skew_ok && jump_ok;
        if (!valid)
        {
            if (!range_ok) ++diagnostics_.pressure_reject_range;
            if (!skew_ok) ++diagnostics_.pressure_reject_skew;
            if (!jump_ok) ++diagnostics_.pressure_reject_jump;
            diagnostics_.last_pressure_reject_bits =
                (range_ok ? 0U : 1U) | (skew_ok ? 0U : 2U) | (jump_ok ? 0U : 4U);
        }
        if (valid)
        {
            ids[count++] = i;
            pressure_mask_ |= uint8_t(1U << i);
        }
        else
        {
            ++diagnostics_.pressure_channel_rejected;
            /* 跳变门是限速率用的，但它比较的是"上一次被接受"的深度，而被拒通道
             * 不会更新 pressure_previous_*，且 elapsed 被封顶在 50 ms。两者合起来，
             * 允许偏差被永久钉死在 jump_margin + max_rate*0.05 ≈ 10 cm：通道只要
             * 掉线期间潜器移动超过这个距离，就再也回不来，直到重新标定或断电。
             * 实测就是这样丢光四路后 READY 灭掉且不自恢复。所以连续被拒够久之后
             * 主动丢掉冻住的基准，让它拿当前读数重新起头 —— 绝对量程门、偏斜门和
             * 相对几何 NIS 门仍然在，坏掉的传感器不会因此被无条件放行。 */
            if (pressure_reject_streak_[i] < 255U)
                ++pressure_reject_streak_[i];
            if (pressure_previous_valid_[i] &&
                pressure_reject_streak_[i] >= kPressureRejectReseedFrames)
            {
                pressure_previous_valid_[i] = false;
                pressure_reject_streak_[i] = 0;
                ++diagnostics_.pressure_channel_reseeds;
            }
        }
    }
    if (count == 0)
    {
        /* 不再清 depth_valid_。这里是"本帧没产出新深度"，不是"深度不可信"：
         * last_pressure_us_ 只在成功时更新，而 FusedStateUsable 已经要求
         * now - pressure_sample_us <= 50 ms，持续拒绝自然会因为过期而失效。
         * 原来在这里额外清一个硬标志属于重复把关，代价是单帧野值就能立刻
         * 锁停整机 —— 实测 395 次更新里拒 1 帧就停了。 */
        ++diagnostics_.pressure_rejected;
        ++diagnostics_.pressure_reject_empty;
        return false;
    }
    bool submerged = true;
    for (unsigned i = 0; i < count; ++i)
        submerged = submerged && depth[ids[i]] >= configuration_.pressure_attitude_min_depth_m;
    if (count >= 2 && submerged)
    {
        const unsigned n = count - 1, ref = ids[0];
        float residual[3] = {}, h[3][6] = {}, noise[9] = {}, ref_prediction, ref_h[6];
        PressureRow(quaternion_, configuration_.pressure_position_m[ref], ref_prediction, ref_h);
        for (unsigned a = 0; a < n; ++a)
        {
            const unsigned i = ids[a + 1];
            float prediction, row[6];
            PressureRow(quaternion_, configuration_.pressure_position_m[i], prediction, row);
            residual[a] = (depth[i] - depth[ref]) - (prediction - ref_prediction);
            for (unsigned k = 0; k < 6; ++k)
                h[a][k] = row[k] - ref_h[k];
            for (unsigned b = 0; b < n; ++b)
            {
                const unsigned j = ids[b + 1];
                const auto &sigma = configuration_.pressure_covariance_m2;
                noise[a * n + b] =
                    sigma[i][j] - sigma[i][ref] - sigma[ref][j] + sigma[ref][ref]; // DΣDᵀ，保留非对角项。
            }
        }
        // 数值求当前几何秩；三点也可能因共线或特殊姿态退化，不能只看传感器数量。
        float basis[3][3] = {};
        for (unsigned a = 0; a < n; ++a)
        {
            float v[3] = {h[a][0], h[a][1], h[a][2]};
            for (unsigned b = 0; b < pressure_rank_; ++b)
            {
                const float projection = Dot(v, basis[b]);
                for (unsigned k = 0; k < 3; ++k)
                    v[k] -= projection * basis[b][k];
            }
            if (Norm(v) > 1e-5f && Normalize(v, 3))
            {
                memcpy(basis[pressure_rank_], v, sizeof(v));
                ++pressure_rank_;
            }
        }
        if (pressure_rank_ && Correct(residual, h, noise, n, diagnostics_.last_pressure_nis)) {
            ++diagnostics_.pressure_updates;
            // A single difference only constrains one tilt direction.
            if (pressure_rank_ >= 2) last_tilt_correction_us_ = pressure.stamp.sample_us;
        }
        else if (pressure_rank_)
        {
            /* 压差姿态被 NIS 拒绝，只放弃姿态这一项，深度照算。
             *
             * 原来这里直接 return，等于让深度给压差姿态连坐。但两者是彼此独立的
             * 观测：深度是四路的加权平均，压差是四路之间的差。水面附近浮着时，
             * 波浪让压差和姿态预测差出几厘米是常态（实测 1062 帧拒了 93 帧，
             * 8.8%），而四路的平均值一直是好的 —— 通道级三道门一次都没拒过。
             * 连坐的结果就是连续拒三帧就超过 50 ms 新鲜度门，整机停机。
             *
             * 姿态本来就还有加速度计在修正，少一次压差更新不影响可用性；
             * 而深度是垂直控制唯一的反馈，不该因为压差吵架就没有。 */
            ++diagnostics_.pressure_rejected;
            ++diagnostics_.pressure_reject_nis;
            pressure_rank_ = 0; // 本帧不提供姿态信息，也不刷新 last_tilt_correction_us_
        } // 相对几何 NIS 拒绝时，整帧也不用于新深度；不能姿态拒绝却继续吸收同一异常阵列。
    }
    // GLS depth: (1ᵀΣ⁻¹1)⁻¹1ᵀΣ⁻¹(z-R_s*gamma). 每路先减姿态引起的安装高度，再加权。
    float sigma[16] = {}, l[16], ones[4] = {1, 1, 1, 1}, weights[4] = {}, sum = 0, value = 0;
    for (unsigned i = 0; i < count; ++i)
        for (unsigned j = 0; j < count; ++j)
            sigma[i * count + j] = configuration_.pressure_covariance_m2[ids[i]][ids[j]];
    if (!Cholesky(sigma, l, count))
    {
        ++diagnostics_.numerical_failures;
        return false; // 本帧无产出；深度是否还可用交给新鲜度门判定
    }
    SolveCholesky(l, ones, weights, count);
    for (unsigned i = 0; i < count; ++i)
    {
        float prediction, row[6];
        PressureRow(quaternion_, configuration_.pressure_position_m[ids[i]], prediction, row);
        value += weights[i] * (depth[ids[i]] - prediction);
        sum += weights[i];
    }
    if (!isfinite(value) || !isfinite(sum) || sum <= 0)
    {
        ++diagnostics_.numerical_failures;
        return false; // 同上
    }
    // Commit trusted baselines only after the complete array/geometry/depth update succeeds.
    for (unsigned n = 0; n < count; ++n) {
        const unsigned i = ids[n];
        pressure_previous_m_[i] = depth[i];
        pressure_previous_us_[i] = pressure.channel_sample_us[i];
        pressure_previous_valid_[i] = true;
        pressure_reject_streak_[i] = 0;
    }
    depth_m_ = value / sum;
    depth_valid_ = true;
    last_pressure_us_ = pressure.stamp.sample_us;
    ++sequence_;
    return true;
}

FusionState AttitudeEskf::State(uint32_t published_us) const
{
    FusionState s = {};
    memcpy(s.quaternion_bn, quaternion_, sizeof(quaternion_));
    memcpy(s.gyro_bias_rad_s, bias_, sizeof(bias_));
    for (unsigned i = 0; i < 3; ++i)
        s.body_rate_rad_s[i] = last_rate_[i] - bias_[i];
    QuaternionEuler(quaternion_, s.euler_rad);
    s.depth_m = depth_m_;
    s.source_us = time_us_;
    s.gyro_sample_us = last_gyro_us_;
    s.pressure_sample_us = last_pressure_us_;
    s.published_us = published_us;
    s.sequence = sequence_;
    s.pressure_mask = pressure_mask_;
    s.pressure_attitude_rank = pressure_rank_;
    // Trace of attitude covariance projected onto the gravity tangent plane:
    // excludes unobservable yaw, while bounding both observable tilt directions.
    const float down_n[3] = {0, 0, 1};
    float down_b[3], h[3][6];
    DirectionModel(quaternion_, down_n, down_b, h);
    float tilt_variance = 0;
    bool covariance_ok = true;
    for (unsigned i = 0; i < 3; ++i) {
        covariance_ok = covariance_ok && Finite(covariance_[i], 3) && covariance_[i][i] >= 0;
        tilt_variance += covariance_[i][i];
        for (unsigned j = 0; j < 3; ++j)
            tilt_variance -= down_b[i] * covariance_[i][j] * down_b[j];
    }
    if (initialized_ && have_gyro_ && covariance_ok && isfinite(tilt_variance) &&
        tilt_variance >= -1e-6f && tilt_variance <= configuration_.maximum_tilt_variance_rad2 &&
        published_us - last_tilt_correction_us_ <= configuration_.maximum_tilt_coast_us)
        s.flags |= FusionAttitudeValid;
    if (depth_valid_)
        s.flags |= FusionDepthValid;
    if (heading_observed_ && published_us - last_mag_us_ <= 100000U)
        s.flags |= FusionHeadingObserved;
    if (FusionConfigurationReady(configuration_))
        s.flags |= FusionConfigurationReadyFlag;
    if (gyro_continuous_ && have_gyro_ &&
        published_us - last_gyro_us_ <= configuration_.maximum_gyro_gap_us + configuration_.reorder_delay_us)
        s.flags |= FusionGyroContinuous;
    return s;
}

} // namespace lower_controller
