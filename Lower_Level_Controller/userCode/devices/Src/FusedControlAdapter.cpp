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
    request.base = output_command_; // 未开闭环时保留 TES 的手动请求，最终仍受总停止门控及输出限幅。
    if (flag_float)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            request.base.pulse_us[Parameter.InID[i]] = Parameter.FloatPWM[i];
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
