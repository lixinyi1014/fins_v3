#include "Propeller.h"
#if LC_IMU_ASYNC_ENABLED
#include "FusionMath.h"
#include <stdlib.h>

lower_controller::FusedControlRequest Propeller_I2C::BuildFusedControlRequest() const
{
    lower_controller::FusedControlRequest request = {};
    request.depth_m = Target_depth * 0.01f; // 兼容 H:n 的 n/10 cm，即 n/1000 m；DN/UP 每次 ±0.01 m。
    request.euler_rad[0] = Target_roll;
    request.euler_rad[1] = Target_pitch;
    request.euler_rad[2] = Target_yaw;
    request.closed_loop = flag_float;
    request.yaw_enabled = flag_angle;
    request.depth_hold = depth_hold_enabled;
    request.base = output_command_; // 未开闭环时保留 TES 的手动请求，最终仍受总停止门控及输出限幅。
    if (flag_float)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            /* 垂直四路的基准：开深度闭环时用 FloatPWM 浮力补偿预置，
             * 关闭时用中位。FloatPWM 里有两路离中位 90~100 us，是一股常驻
             * 垂直推力；关了深度环只是把 PID 那一项清零，这股预置推力还在推，
             * 正浮力的机体只会升得更快。既然"关深度环"的意思是把深浅交还给
             * 浮力配平，垂直方向就该完全不主动出力，只留姿态稳定的增量。 */
            request.base.pulse_us[Parameter.InID[i]] =
                depth_hold_enabled ? Parameter.FloatPWM[i] : Parameter.InitPWM;
            request.base.pulse_us[Parameter.OutID[i]] = state_PWM_map[motion_state][i];
        }
    }
    return request;
}
bool Propeller_I2C::SetFusedTarget(const char *command)
{
    if (!fused_feedback_enabled)
        return false;
    float values[4];
    const char *cursor = command;
    for (unsigned i = 0; i < 4; ++i)
    {
        char *end;
        values[i] = strtof(cursor, &end);
        if (end == cursor || !isfinite(values[i]))
            return false;
        if (i < 3)
        {
            if (*end != ',')
                return false;
            cursor = end + 1;
        }
        else
        {
            while (*end == '\r' || *end == '\n' || *end == ' ')
                ++end;
            if (*end)
                return false;
        }
    }
    if (values[0] < 0 || values[0] > 100 || fabsf(values[1]) > 30 || fabsf(values[2]) > 30 ||
        fabsf(values[3]) > 180)
        return false;
    Target_depth = values[0] * 100; // FSET:深度m,横滚deg,俯仰deg,偏航deg；四项通过检查后一次提交。
    Target_roll = deg2rad(values[1]);
    Target_pitch = deg2rad(values[2]);
    Target_yaw = deg2rad(values[3]);
    return true;
}
#endif
