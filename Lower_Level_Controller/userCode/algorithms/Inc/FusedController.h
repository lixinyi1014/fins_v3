#ifndef FUSED_CONTROLLER_H
#define FUSED_CONTROLLER_H
#include "SensorFusionData.h"
#include "LegacyControlData.h"

namespace lower_controller
{
struct SiPidGains
{
    float kp, ki, kd, integral_limit, output_limit;
};

/* 论文 3.2：第 i 个推进器的作用点、带符号正推力方向与系数。
 * axis 是“命令为正时推力指向的机体系单位向量”，与电机机械转向无关；
 * 电气正反桨符号 electrical_sign 只在最后写 PWM 时施加，与旧代码的 Sign[] 一致。
 * 位置与方向取自 docs/CAD_GEOMETRY_20260916.json（同一份装配变换核对过）。
 * thrust_coefficient 为 k_T：当前没有系泊推力标定，取 1，单位即“PWM 增量当量推力”。 */
struct ThrusterGeometry
{
    float position_m[LC_THRUSTER_COUNT][3];
    float axis[LC_THRUSTER_COUNT][3];
    float thrust_coefficient[LC_THRUSTER_COUNT];
    float torque_coefficient[LC_THRUSTER_COUNT]; // k_Q*sigma_i；正反桨成对抵消时为 0
    float electrical_sign[LC_THRUSTER_COUNT];
    bool configured;
};

/* 论文 2：重力与浮力恢复力 g(eta)。CAD 里没有质量、浮心和配重信息，
 * 因此默认 configured=false，前馈输出 0，悬停配平仍由原 FloatPWM 预置提供。
 * newton_per_unit 为“1 单位推力指令对应多少牛顿”，需系泊推力试验测得；
 * 未标定（<=0）时同样关闭前馈，避免把牛顿量直接当成 PWM 当量使用。 */
struct RestoringParameters
{
    float weight_n, buoyancy_n;
    float center_of_gravity_m[3], center_of_buoyancy_m[3];
    float newton_per_unit;
    bool configured;
};

struct FusedControlConfiguration
{
    SiPidGains translation[3]; // 论文 6.2：世界系 x,y,z 位置单环；z 由水压深度闭环
    SiPidGains angle[3], rate[3];
    float allocation_weight[LC_THRUSTER_COUNT]; // 论文 3.5 的 W 对角元，必须为正
    ThrusterGeometry thrusters;
    RestoringParameters restoring;
    bool gains_configured, allocation_configured;
    // 下位机没有任何 x/y 感知，位置环结构已实现但默认禁用；
    // 上位机相机系统接入后，在请求里给出 position_measured_n_m 并置位即可启用。
    bool position_xy_enabled;
    // 论文 3.2 的 q_i = n_i|n_i|：true 时用 n = sgn(q)sqrt(|q|) 反解转速。
    // 默认 false，保持与原工程一致的线性 PWM 当量映射。
    bool quadratic_thrust;
};

struct FusedControlRequest
{
    float depth_m, euler_rad[3]; // 前右下机体系、NED；角顺序 roll,pitch,yaw。
    float position_target_n_m[2];   // 世界系 x,y 目标
    float position_measured_n_m[2]; // 世界系 x,y 反馈，由上位机下发
    bool position_valid;            // 反馈新鲜可用；为假时 x,y 通道输出 0
    bool closed_loop, yaw_enabled;
    ThrusterPwmCommand base; // 原版本浮力补偿和水平开环预置，单位 us。
};

struct FusedControlResult
{
    ThrusterPwmCommand command;
    bool valid, saturated;
    float effort[6];                 // 六个虚拟控制通道，顺序 X Y Z K M N
    float thrust[LC_THRUSTER_COUNT]; // 分配得到的每路推力指令（PWM 当量）
};
const FusedControlConfiguration &GetFusedControlConfiguration();
bool FusedControlConfigurationReady(const FusedControlConfiguration &config);
bool FusedStateUsable(const FusionState &state, uint32_t now_us, bool need_heading);

/* 输入为 SI 反馈，输出为工程 PWM 增量，未声称建立牛顿/转速标定模型。
 * 角度外环每三次执行一次，速率/位置环每次执行；所有积分和微分使用真实秒数。
 * 控制链路（论文 6.1）：位置环给世界系平移控制量 -> R^T 转机体系 ->
 * 与姿态双环的三个转动通道合成期望广义力 -> B_T 加权伪逆分配到八个推进器。 */
class FusedController
{
  public:
    explicit FusedController(const FusedControlConfiguration &config) : configuration_(config)
    {
        BuildAllocation();
        Reset();
    }
    void Reset();
    FusedControlResult Compute(const FusionState &state, const FusedControlRequest &request, uint32_t now_us);
    // 论文 3.5 的加权伪逆 B_T^+（8x6，列序 X Y Z K M N）；几何非法时 AllocationReady() 为假。
    const float (*PseudoInverse() const)[6]
    {
        return pseudo_inverse_;
    }
    // 各虚拟通道到推进器指令的标定增益 kappa（论文 6.4/6.5）。
    const float *ChannelGain() const
    {
        return channel_gain_;
    }
    bool AllocationReady() const
    {
        return allocation_ready_;
    }

  private:
    struct PidMemory
    {
        float integral, previous_error, derivative;
        bool initialized;
    };
    static float Step(const SiPidGains &gains, PidMemory &memory, float error, float dt, bool freeze);
    void BuildAllocation();
    void RestoringFeedforward(const FusionState &state, float tau[6]) const;
    const FusedControlConfiguration &configuration_;
    PidMemory translation_[3], angle_[3], rate_[3];
    float pseudo_inverse_[LC_THRUSTER_COUNT][6];
    float channel_gain_[6];
    bool allocation_ready_;
    uint32_t previous_us_;
    unsigned phase_;
    float outer_dt_, desired_rate_[3];
    bool started_, saturated_, previous_yaw_, previous_position_;
};
} // namespace lower_controller
#endif
