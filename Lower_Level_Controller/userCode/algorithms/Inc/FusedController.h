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
struct FusedControlConfiguration
{
    SiPidGains depth, angle[3], rate[3];
    float allocation[8][4]; // PCA 0..7 的脉宽增量 = 本行 · [深度,横滚,俯仰,偏航]虚拟控制量。
    bool gains_configured, allocation_configured;
};
struct FusedControlRequest
{
    float depth_m, euler_rad[3]; // 前右下机体系、NED；角顺序 roll,pitch,yaw。
    bool closed_loop, yaw_enabled;
    ThrusterPwmCommand base; // 原版本浮力补偿和水平开环预置，单位 us。
};
struct FusedControlResult
{
    ThrusterPwmCommand command;
    bool valid, saturated;
    float effort[4];
};
const FusedControlConfiguration &GetFusedControlConfiguration();
bool FusedControlConfigurationReady(const FusedControlConfiguration &config);
bool FusedStateUsable(const FusionState &state, uint32_t now_us, bool need_heading);

/* 输入为 SI 反馈，输出为工程 PWM 增量，未声称建立牛顿/转速标定模型。
 * 角度外环每三次执行一次，速率/深度环每次执行；所有积分和微分使用真实秒数。 */
class FusedController
{
  public:
    explicit FusedController(const FusedControlConfiguration &config) : configuration_(config)
    {
        Reset();
    }
    void Reset();
    FusedControlResult Compute(const FusionState &state, const FusedControlRequest &request, uint32_t now_us);

  private:
    struct PidMemory
    {
        float integral, previous_error, derivative;
        bool initialized;
    };
    static float Step(const SiPidGains &gains, PidMemory &memory, float error, float dt, bool freeze);
    const FusedControlConfiguration &configuration_;
    PidMemory depth_, angle_[3], rate_[3];
    uint32_t previous_us_;
    unsigned phase_;
    float outer_dt_, desired_rate_[3];
    bool started_, saturated_, previous_yaw_;
};
} // namespace lower_controller
#endif
