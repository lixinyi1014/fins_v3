#include "FusedController.h"
#include "FusionMath.h"
#include "FusionConfiguration.h"
#include <string.h>

namespace lower_controller
{
using namespace fusion_math;
namespace
{
FusedControlConfiguration BuildControllerConfiguration()
{
    FusedControlConfiguration c = {};
    c.gains_configured = true; // 已选用原版转换初值，仍需实机整定。
    c.allocation_configured = true;
    c.position_xy_enabled = false; // 下位机没有 x/y 感知；位置环结构已就绪，等上位机接入。
    c.quadratic_thrust = false;    // 保持原工程的线性 PWM 当量映射。
    // TODO(TUNING)：以下是待台架验证的初值，不是已完成实机整定。
    // 旧 PID: Ki 每次累加、Kd 每次差分；新 PID: Ki/s、Kd*s。
    // 因此 Ki_new=Ki_old*f，Kd_new=Kd_old/f。速率环比例项可直接沿用。
    c.rate[0] = {8, 0, 0, 50, 200};                 // RollInPID
    c.rate[1] = {8, 0, 0, 50, 200};                 // PitchInPID
    c.rate[2] = {5, 0, 0, 100, 400};                // YawInPID
    c.angle[2] = {2, 0.01f * 50, 2.0f / 50, 5, 20}; // YawOutPID，rad -> rad/s。
    // 这里的 units_per_m 只服务于保留的旧控制增益换算；新 ESKF 已在 pressure_pa
    // 接口上使用绝对压力和液面参考，不能把旧控制标度当成传感器单位。
    const auto &physical = GetFusionConfiguration();
    const float units_per_m = physical.water_density_kg_m3 * physical.gravity_m_s2 / 2000.0f;
    // 论文 6.2 的三个位置单环；z 通道沿用原深度 PID 增益，x/y 暂用同量级初值。
    // TODO(TUNING)：x/y 增益在上位机位置反馈接入并确定坐标系后重新整定。
    c.translation[0] = {10 * units_per_m, 0, 0, 50, 200};
    c.translation[1] = {10 * units_per_m, 0, 0, 50, 200};
    c.translation[2] = {10 * units_per_m, 0.02f * 150 * units_per_m, 10.0f / 150 * units_per_m, 50, 200};
    // 用同一份 CAD 几何计算压差/rad 的幅值，避免位置变了而增益仍用旧跨度。
    float roll_span = 0, pitch_span = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        roll_span += fabsf(physical.pressure_position_m[i][1]);
        pitch_span += fabsf(physical.pressure_position_m[i][0]);
    }
    const float roll_scale = roll_span * units_per_m, pitch_scale = pitch_span * units_per_m;
    c.angle[0] = {0.5f * roll_scale, 0.002f * 50 * roll_scale, roll_scale / 50, 2.5f, 10};
    c.angle[1] = {pitch_scale, 0.005f * 50 * pitch_scale, pitch_scale / 50, 2.5f, 10};
    // 论文 3.2/3.4 的推进器作用点与带符号正推力方向，单位 m，机体 FRD 系。
    // 数值取自 docs/CAD_GEOMETRY_20260916.json，顺序为 PCA 通道 0..7：
    // 0 前左水平, 1 前左垂直, 2 后左垂直, 3 后左水平, 4 后右水平, 5 后右垂直, 6 前右垂直, 7 前右水平。
    // 垂直四推正方向取 +Zb（向下为正），与旧混控"正深度指令下沉"一致；
    // 水平四推沿用 CAD 轴向，四推同向正命令产生 -Xb（后退），与旧 FrontPWM 符号一致。
    const float position[LC_THRUSTER_COUNT][3] = {
        {0.166452f, -0.120016f, 0.002f}, {0.054f, -0.107008f, -0.015f},
        {-0.054f, -0.107008f, -0.015f},  {-0.142410f, -0.144057f, 0.002f},
        {-0.142410f, 0.144057f, 0.002f}, {-0.054f, 0.107008f, -0.015f},
        {0.054f, 0.107008f, -0.015f},    {0.166452f, 0.120016f, 0.002f}};
    const float axis[LC_THRUSTER_COUNT][3] = {
        {-0.7071068f, -0.7071068f, 0.0f}, {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},               {-0.7071068f, 0.7071068f, 0.0f},
        {-0.7071068f, -0.7071068f, 0.0f}, {0.0f, 0.0f, 1.0f},
        {0.0f, 0.0f, 1.0f},               {-0.7071068f, 0.7071068f, 0.0f}};
    const int signs[LC_THRUSTER_COUNT] = LC_THRUSTER_SIGNS_INIT;
    for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
    {
        memcpy(c.thrusters.position_m[i], position[i], sizeof(position[i]));
        memcpy(c.thrusters.axis[i], axis[i], sizeof(axis[i]));
        // k_T 未做系泊推力标定：取 1，推力单位即"PWM 增量当量"，与旧固定混控同标度。
        c.thrusters.thrust_coefficient[i] = 1.0f;
        // 正反桨成对布置，轴向反扭矩按抵消处理；标定后可填入 k_Q*sigma_i。
        c.thrusters.torque_coefficient[i] = 0.0f;
        c.thrusters.electrical_sign[i] = float(signs[i]);
        c.allocation_weight[i] = 1.0f; // 论文 3.5 的 W；等权即最小二范数分配。
    }
    c.thrusters.configured = true;
    // 论文 2 的恢复力参数：质量、浮心、配重均未实测，默认关闭前馈。
    // 实测后填写 weight_n、buoyancy_n、重心/浮心位置和 newton_per_unit 并置 configured=true；
    // 启用时应同时把 Propeller 的 FloatPWM 预置改回中位，避免与前馈重复补偿。
    c.restoring = {};
    return c;
}
bool GainsValid(const SiPidGains &g)
{
    return isfinite(g.kp) && isfinite(g.ki) && isfinite(g.kd) && isfinite(g.integral_limit) &&
           isfinite(g.output_limit) && g.kp > 0 && g.ki >= 0 && g.kd >= 0 && g.integral_limit >= 0 &&
           g.output_limit > 0;
}
/** @brief 论文 3.2：由几何构造 6x8 分配矩阵 B_T，行序 X Y Z K M N。 */
void BuildAllocationMatrix(const ThrusterGeometry &t, float b[6][LC_THRUSTER_COUNT])
{
    for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
    {
        const float k = t.thrust_coefficient[i];
        const float *a = t.axis[i], *r = t.position_m[i];
        b[0][i] = k * a[0];
        b[1][i] = k * a[1];
        b[2][i] = k * a[2];
        // k_T*(r x a) + k_Q*sigma*a：力臂力矩加轴向反扭矩。
        b[3][i] = k * (r[1] * a[2] - r[2] * a[1]) + t.torque_coefficient[i] * a[0];
        b[4][i] = k * (r[2] * a[0] - r[0] * a[2]) + t.torque_coefficient[i] * a[1];
        b[5][i] = k * (r[0] * a[1] - r[1] * a[0]) + t.torque_coefficient[i] * a[2];
    }
}
} // namespace
const FusedControlConfiguration &GetFusedControlConfiguration()
{
    static const FusedControlConfiguration configuration = BuildControllerConfiguration();
    return configuration;
}
bool FusedControlConfigurationReady(const FusedControlConfiguration &c)
{
    if (!c.gains_configured || !c.allocation_configured || !c.thrusters.configured)
        return false;
    for (unsigned j = 0; j < 3; ++j)
        if (!GainsValid(c.translation[j]) || !GainsValid(c.angle[j]) || !GainsValid(c.rate[j]))
            return false;
    for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
    {
        if (!Finite(c.thrusters.position_m[i], 3) || !Finite(c.thrusters.axis[i], 3))
            return false;
        if (fabsf(Norm(c.thrusters.axis[i]) - 1.0f) > 1e-3f) // 推力方向必须是单位向量
            return false;
        if (!isfinite(c.thrusters.thrust_coefficient[i]) || c.thrusters.thrust_coefficient[i] <= 0)
            return false;
        if (!isfinite(c.allocation_weight[i]) || c.allocation_weight[i] <= 0)
            return false;
        if (c.thrusters.electrical_sign[i] != 1.0f && c.thrusters.electrical_sign[i] != -1.0f)
            return false;
    }
    // 论文 3.5：八推在六维上必须满行秩，否则存在无法产生的广义力方向。
    float b[6][LC_THRUSTER_COUNT], gram[36] = {}, factor[36];
    BuildAllocationMatrix(c.thrusters, b);
    for (unsigned a = 0; a < 6; ++a)
        for (unsigned d = 0; d < 6; ++d)
            for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
                gram[a * 6 + d] += b[a][i] * b[d][i] / c.allocation_weight[i];
    return Cholesky(gram, factor, 6);
}
bool FusedStateUsable(const FusionState &s, uint32_t now, bool heading)
{
    const uint32_t required =
        FusionAttitudeValid | FusionDepthValid | FusionGyroContinuous | FusionConfigurationReadyFlag;
    return (s.flags & required) == required && (!heading || (s.flags & FusionHeadingObserved)) &&
           now - s.gyro_sample_us <= 20000U && now - s.pressure_sample_us <= 50000U &&
           now - s.published_us <= 20000U && Finite(s.euler_rad, 3) && Finite(s.body_rate_rad_s, 3) &&
           isfinite(s.depth_m) && fabsf(s.euler_rad[0]) <= 0.7853982f &&
           fabsf(s.euler_rad[1]) <= 0.7853982f; // 45°，超出本级小倾角控制工作区。
}
/** @brief 论文 3.5：一次算出加权伪逆 B_T^+ = W^-1 B^T (B W^-1 B^T)^-1 及通道标定增益。
 * 几何是常量，因此只在构造时计算；运行期分配退化为一次 8x6 乘法。 */
void FusedController::BuildAllocation()
{
    memset(pseudo_inverse_, 0, sizeof(pseudo_inverse_));
    memset(channel_gain_, 0, sizeof(channel_gain_));
    allocation_ready_ = false;
    if (!FusedControlConfigurationReady(configuration_))
        return;
    float b[6][LC_THRUSTER_COUNT], gram[36] = {}, factor[36];
    BuildAllocationMatrix(configuration_.thrusters, b);
    for (unsigned a = 0; a < 6; ++a)
        for (unsigned d = 0; d < 6; ++d)
            for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
                gram[a * 6 + d] += b[a][i] * b[d][i] / configuration_.allocation_weight[i];
    if (!Cholesky(gram, factor, 6))
        return;
    // 逐列解 (B W^-1 B^T) y = e_j 得到逆矩阵第 j 列，再左乘 W^-1 B^T。
    for (unsigned j = 0; j < 6; ++j)
    {
        float unit[6] = {}, y[6];
        unit[j] = 1.0f;
        SolveCholesky(factor, unit, y, 6);
        for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
        {
            float value = 0;
            for (unsigned a = 0; a < 6; ++a)
                value += b[a][i] * y[a];
            pseudo_inverse_[i][j] = value / configuration_.allocation_weight[i];
        }
    }
    if (!Finite(&pseudo_inverse_[0][0], LC_THRUSTER_COUNT * 6))
        return;
    // 论文 6.4/6.5 的 kappa：把虚拟控制量标定成"单位指令对应单个推进器满幅指令"，
    // 使原来以 PWM 增量整定的 PID 增益在新分配路径下保持同一物理含义。
    // 当前几何下 kappa = [2.828, 2.828, 4.0, 0.428, 0.216, 0.810]，
    // 深度/横滚/俯仰/偏航四列与旧固定混控逐推进器数值完全一致。
    for (unsigned j = 0; j < 6; ++j)
    {
        float peak = 0;
        for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
            peak = fmaxf(peak, fabsf(pseudo_inverse_[i][j]));
        if (!(peak > 1e-6f))
            return; // 该通道无法由八推产生
        channel_gain_[j] = 1.0f / peak;
    }
    allocation_ready_ = true;
}
/** @brief 论文 2：重力与浮力恢复力 g(eta)，输出已换算为推力指令单位。 */
void FusedController::RestoringFeedforward(const FusionState &state, float tau[6]) const
{
    memset(tau, 0, sizeof(float) * 6);
    const auto &p = configuration_.restoring;
    if (!p.configured || !(p.newton_per_unit > 0) || !isfinite(p.weight_n) || !isfinite(p.buoyancy_n))
        return;
    float r[3][3];
    QuaternionMatrix(state.quaternion_bn, r); // R_b^n：机体系到世界系
    // 世界系 NED 中重力沿 +Z、浮力沿 -Z；用 R^T 转到机体系，即取 R 的第三行。
    float gravity_b[3], buoyancy_b[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        gravity_b[i] = r[2][i] * p.weight_n;
        buoyancy_b[i] = -r[2][i] * p.buoyancy_n;
    }
    const float *rg = p.center_of_gravity_m, *rb = p.center_of_buoyancy_m;
    float force[3], moment[3];
    for (unsigned i = 0; i < 3; ++i)
        force[i] = gravity_b[i] + buoyancy_b[i];
    moment[0] = rg[1] * gravity_b[2] - rg[2] * gravity_b[1] + rb[1] * buoyancy_b[2] - rb[2] * buoyancy_b[1];
    moment[1] = rg[2] * gravity_b[0] - rg[0] * gravity_b[2] + rb[2] * buoyancy_b[0] - rb[0] * buoyancy_b[2];
    moment[2] = rg[0] * gravity_b[1] - rg[1] * gravity_b[0] + rb[0] * buoyancy_b[1] - rb[1] * buoyancy_b[0];
    // Fossen 约定 g(eta) 在方程左侧，推进器需提供 -(重力+浮力) 的广义力来配平。
    for (unsigned i = 0; i < 3; ++i)
    {
        tau[i] = -force[i] / p.newton_per_unit;
        tau[i + 3] = -moment[i] / p.newton_per_unit;
    }
    if (!Finite(tau, 6))
        memset(tau, 0, sizeof(float) * 6);
}
void FusedController::Reset()
{
    memset(translation_, 0, sizeof(translation_));
    memset(angle_, 0, sizeof(angle_));
    memset(rate_, 0, sizeof(rate_));
    memset(desired_rate_, 0, sizeof(desired_rate_));
    previous_us_ = phase_ = 0;
    outer_dt_ = 0;
    started_ = saturated_ = previous_yaw_ = previous_position_ = false;
}
float FusedController::Step(const SiPidGains &g, PidMemory &m, float error, float dt, bool freeze)
{
    const float derivative = m.initialized ? (error - m.previous_error) / dt : 0;
    m.derivative += dt / (0.05f + dt) * (derivative - m.derivative); // 微分一阶滤波，时间常数 50 ms。
    const float previous_integral = m.integral;
    if (!freeze || error * m.integral < 0)
        m.integral = Clamp(m.integral + g.ki * error * dt, -g.integral_limit, g.integral_limit);
    float raw = g.kp * error + m.integral + g.kd * m.derivative;
    if ((raw > g.output_limit && error > 0) || (raw < -g.output_limit && error < 0))
        m.integral = previous_integral;
    raw = g.kp * error + m.integral + g.kd * m.derivative;
    m.previous_error = error;
    m.initialized = true;
    return Clamp(raw, -g.output_limit, g.output_limit);
}
FusedControlResult FusedController::Compute(const FusionState &state, const FusedControlRequest &request,
                                            uint32_t now)
{
    FusedControlResult result = {};
    for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
        result.command.pulse_us[i] = LC_THRUSTER_NEUTRAL_US; // 1550 us。
    if (!allocation_ready_ || !FusedStateUsable(state, now, request.yaw_enabled) ||
        !isfinite(request.depth_m) || request.depth_m < 0 || request.depth_m > 100 ||
        !Finite(request.euler_rad, 3) || fabsf(request.euler_rad[0]) > 0.523599f ||
        fabsf(request.euler_rad[1]) > 0.523599f)
    {
        Reset();
        return result;
    } // 目标倾角限 30°。
    if (!request.closed_loop)
    {
        Reset();
        result.command = request.base;
        result.valid = true;
        return result;
    }
    const float dt = started_ ? float(now - previous_us_) * 1e-6f : 1.0f / 150.0f;
    if (dt <= 0 || dt > 0.020f)
    {
        Reset();
        return result;
    }
    previous_us_ = now;
    started_ = true;
    outer_dt_ += dt;
    if (request.yaw_enabled != previous_yaw_)
    {
        angle_[2] = {};
        rate_[2] = {};
        desired_rate_[2] = 0;
        previous_yaw_ = request.yaw_enabled;
        phase_ = 0;
    }
    // ---- 论文 6.3：姿态外环（50 Hz）给出期望欧拉角变化率，再经 T^-1 转为机体系角速度。
    if (phase_ == 0)
    {
        float euler_rate[3] = {};
        for (unsigned i = 0; i < 3; ++i)
        {
            if (i == 2 && !request.yaw_enabled)
                continue;
            const float error = i == 2 ? WrapPi(request.euler_rad[i] - state.euler_rad[i])
                                       : request.euler_rad[i] - state.euler_rad[i];
            euler_rate[i] = Step(configuration_.angle[i], angle_[i], error, outer_dt_, saturated_);
        }
        const float roll = state.euler_rad[0], pitch = state.euler_rad[1];
        desired_rate_[0] = euler_rate[0] - sinf(pitch) * euler_rate[2];
        desired_rate_[1] = cosf(roll) * euler_rate[1] + sinf(roll) * cosf(pitch) * euler_rate[2];
        desired_rate_[2] = -sinf(roll) * euler_rate[1] + cosf(roll) * cosf(pitch) * euler_rate[2];
        outer_dt_ = 0;
    }
    phase_ = (phase_ + 1U) % 3U; // 150 Hz / 3 = 名义 50 Hz 角度外环。
    // ---- 论文 6.2：三个位置单环在世界系计算，再用 R^T 转到机体系执行。
    // x/y 需要上位机下发位置反馈；没有新鲜反馈时这两路输出 0 并清积分。
    const bool position_active =
        configuration_.position_xy_enabled && request.position_valid &&
        Finite(request.position_measured_n_m, 2) && Finite(request.position_target_n_m, 2);
    if (position_active != previous_position_)
    {
        translation_[0] = {};
        translation_[1] = {};
        previous_position_ = position_active;
    }
    float world_effort[3] = {};
    if (position_active)
        for (unsigned i = 0; i < 2; ++i)
            world_effort[i] =
                Step(configuration_.translation[i], translation_[i],
                     request.position_target_n_m[i] - request.position_measured_n_m[i], dt, saturated_);
    world_effort[2] =
        Step(configuration_.translation[2], translation_[2], request.depth_m - state.depth_m, dt, saturated_);
    float rotation[3][3], body_effort[3];
    QuaternionMatrix(state.quaternion_bn, rotation); // R_b^n
    for (unsigned i = 0; i < 3; ++i)                 // u_t = (R_b^n)^T u_t^n
        body_effort[i] = rotation[0][i] * world_effort[0] + rotation[1][i] * world_effort[1] +
                         rotation[2][i] * world_effort[2];
    // ---- 角速度内环（150 Hz）给出三个转动虚拟控制通道。
    float angular_effort[3];
    for (unsigned i = 0; i < 3; ++i)
        angular_effort[i] = i == 2 && !request.yaw_enabled
                                ? 0
                                : Step(configuration_.rate[i], rate_[i],
                                       desired_rate_[i] - state.body_rate_rad_s[i], dt, saturated_);
    for (unsigned i = 0; i < 3; ++i)
    {
        result.effort[i] = body_effort[i];
        result.effort[i + 3] = angular_effort[i];
    }
    // ---- 合成期望广义力：kappa 标定加恢复力前馈（未标定时为 0）。
    float tau[6], feedforward[6];
    RestoringFeedforward(state, feedforward);
    for (unsigned j = 0; j < 6; ++j)
        tau[j] = channel_gain_[j] * result.effort[j] + feedforward[j];
    if (!Finite(tau, 6))
    {
        Reset();
        FusedControlResult failed = {};
        for (unsigned channel = 0; channel < LC_THRUSTER_COUNT; ++channel)
            failed.command.pulse_us[channel] = LC_THRUSTER_NEUTRAL_US;
        return failed;
    }
    // ---- 论文 3.5：加权伪逆分配 q = B_T^+ tau。
    for (unsigned i = 0; i < LC_THRUSTER_COUNT; ++i)
    {
        float q = 0;
        for (unsigned j = 0; j < 6; ++j)
            q += pseudo_inverse_[i][j] * tau[j];
        // 论文 3.2 的 q_i = n_i|n_i|：需要时反解转速指令，默认线性 PWM 当量。
        float n = q;
        if (configuration_.quadratic_thrust)
            n = (q >= 0 ? 1.0f : -1.0f) * sqrtf(fabsf(q));
        result.thrust[i] = q;
        float pulse = float(request.base.pulse_us[i]) + configuration_.thrusters.electrical_sign[i] * n;
        if (!isfinite(pulse))
        {
            Reset();
            FusedControlResult failed = {};
            for (unsigned channel = 0; channel < LC_THRUSTER_COUNT; ++channel)
                failed.command.pulse_us[channel] = LC_THRUSTER_NEUTRAL_US;
            return failed;
        }
        // V3.3 当前死区 1510..1610 us，中位 1550：正向 +60、反向 -40。
        // 极小非零量仍受 PCA 的约 4.88 us 分辨率影响，实际启动阈值待入水核对。
        if (pulse > LC_THRUSTER_NEUTRAL_US)
            pulse += LC_THRUSTER_DEADZONE_HIGH_US - LC_THRUSTER_NEUTRAL_US;
        else if (pulse < LC_THRUSTER_NEUTRAL_US)
            pulse -= LC_THRUSTER_NEUTRAL_US - LC_THRUSTER_DEADZONE_LOW_US;
        const float bounded = Clamp(pulse, LC_THRUSTER_MIN_US, LC_THRUSTER_MAX_US);
        result.saturated |= bounded != pulse;
        result.command.pulse_us[i] = static_cast<int32_t>(bounded);
    }
    saturated_ = result.saturated;
    result.valid = true;
    return result;
}
} // namespace lower_controller
