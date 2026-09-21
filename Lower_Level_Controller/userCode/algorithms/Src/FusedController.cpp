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
    // TODO(TUNING)：以下是待台架验证的初值，不是已完成实机整定。
    // 旧 PID: Ki 每次累加、Kd 每次差分；新 PID: Ki/s、Kd*s。
    // 因此 Ki_new=Ki_old*f，Kd_new=Kd_old/f。速率环比例项可直接沿用。
    c.rate[0] = {8, 0, 0, 50, 200}; // RollInPID
    c.rate[1] = {8, 0, 0, 50, 200}; // PitchInPID
    c.rate[2] = {5, 0, 0, 100, 400}; // YawInPID
    c.angle[2] = {2, 0.01f*50, 2.0f/50, 5, 20}; // YawOutPID，rad -> rad/s。
    // 30BA 在约 20°C 时旧兼容公式约为 P(Pa)/2000，并非厘米。
    // TODO：旧低温补偿混合了型号，以下仅为 20°C 附近的线性估算。
    const auto &physical = GetFusionConfiguration();
    const float units_per_m = physical.water_density_kg_m3*physical.gravity_m_s2/2000.0f;
    c.depth = {10*units_per_m, 0.02f*150*units_per_m,
               10.0f/150*units_per_m, 50, 200};
    // 用同一份 CAD 几何计算压差/rad 的幅值，避免位置变了而增益仍用旧跨度。
    float roll_span = 0, pitch_span = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        roll_span += fabsf(physical.pressure_position_m[i][1]);
        pitch_span += fabsf(physical.pressure_position_m[i][0]);
    }
    const float roll_scale = roll_span*units_per_m, pitch_scale = pitch_span*units_per_m;
    c.angle[0] = {0.5f*roll_scale, 0.002f*50*roll_scale, roll_scale/50, 2.5f, 10};
    c.angle[1] = {pitch_scale, 0.005f*50*pitch_scale, pitch_scale/50, 2.5f, 10};
    const int vertical[4] = LC_VERTICAL_CHANNELS_INIT;
    const int horizontal[4] = LC_HORIZONTAL_CHANNELS_INIT;
    const int signs[8] = LC_THRUSTER_SIGNS_INIT;
    const int factors[4][3] = {{-1,-1,-1},{-1,-1,1},{-1,1,-1},{-1,1,1}};
    for (unsigned i = 0; i < 4; ++i)
    {
        const int channel = vertical[i], sign = signs[channel];
        c.allocation[channel][0] = -sign*factors[i][0];
        c.allocation[channel][1] = sign*factors[i][1];
        c.allocation[channel][2] = sign*factors[i][2];
        c.allocation[horizontal[i]][3] = (i < 2 ? -1 : 1)*signs[horizontal[i]];
    }
    // 新 FRD 定义：向下力 Fz 为正，Mx=y*Fz，My=-x*Fz；因此正 pitch 对应前上后下。
    // 沿用旧正反桨、通道和向下脉宽符号；roll/pitch 列按这个物理定义生成。
    // STEP水平轴+旧W前进约定：正FRD偏航需左侧减、右侧加sign*PWM。
    // 旧偏航反馈符号不同，不能照抄其分配列；已用CAD叉乘验证四轴恢复力矩。
    // TODO(TUNING)：转换初值不是实机整定；首次低输出验证逐轴恢复方向。
    return c;
}
bool GainsValid(const SiPidGains &g)
{
    return isfinite(g.kp) && isfinite(g.ki) && isfinite(g.kd) && isfinite(g.integral_limit) &&
           isfinite(g.output_limit) && g.kp > 0 && g.ki >= 0 && g.kd >= 0 && g.integral_limit >= 0 &&
           g.output_limit > 0;
}
} // namespace
const FusedControlConfiguration &GetFusedControlConfiguration()
{
    static const FusedControlConfiguration configuration = BuildControllerConfiguration();
    return configuration;
}
bool FusedControlConfigurationReady(const FusedControlConfiguration &c)
{
    if (!c.gains_configured || !c.allocation_configured || !GainsValid(c.depth))
        return false;
    for (unsigned j = 0; j < 3; ++j)
        if (!GainsValid(c.angle[j]) || !GainsValid(c.rate[j]))
            return false;
    // 四个控制方向必须线性独立；仅检查“非零”无法发现深度/俯仰被配成相同方向。
    float gram[16] = {}, factor[16];
    for (unsigned i = 0; i < 8; ++i)
    {
        if (!Finite(c.allocation[i], 4))
            return false;
        for (unsigned a = 0; a < 4; ++a)
            for (unsigned b = 0; b < 4; ++b)
                gram[a * 4 + b] += c.allocation[i][a] * c.allocation[i][b];
    }
    return Cholesky(gram, factor, 4);
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
void FusedController::Reset()
{
    memset(&depth_, 0, sizeof(depth_));
    memset(angle_, 0, sizeof(angle_));
    memset(rate_, 0, sizeof(rate_));
    memset(desired_rate_, 0, sizeof(desired_rate_));
    previous_us_ = phase_ = 0;
    outer_dt_ = 0;
    started_ = saturated_ = previous_yaw_ = false;
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
    for (unsigned i = 0; i < 8; ++i)
        result.command.pulse_us[i] = LC_THRUSTER_NEUTRAL_US; // 1550 us。
    if (!FusedControlConfigurationReady(configuration_) ||
        !FusedStateUsable(state, now, request.yaw_enabled) || !isfinite(request.depth_m) ||
        request.depth_m < 0 || request.depth_m > 100 || !Finite(request.euler_rad, 3) ||
        fabsf(request.euler_rad[0]) > 0.523599f || fabsf(request.euler_rad[1]) > 0.523599f)
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
    result.effort[0] = Step(configuration_.depth, depth_, request.depth_m - state.depth_m, dt, saturated_);
    for (unsigned i = 0; i < 3; ++i)
        result.effort[i + 1] = i == 2 && !request.yaw_enabled
                                   ? 0
                                   : Step(configuration_.rate[i], rate_[i],
                                          desired_rate_[i] - state.body_rate_rad_s[i], dt, saturated_);
    for (unsigned i = 0; i < 8; ++i)
    {
        float pulse = float(request.base.pulse_us[i]);
        for (unsigned j = 0; j < 4; ++j)
            pulse += configuration_.allocation[i][j] * result.effort[j];
        if (!isfinite(pulse))
        {
            Reset();
            FusedControlResult failed = {};
            for (unsigned channel = 0; channel < 8; ++channel)
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
