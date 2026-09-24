//
// Created by admin on 2023/10/30.
//

#ifndef CONTROL_FRAME_MAIN_PWM_H
#define CONTROL_FRAME_MAIN_PWM_H
#include "FusedController.h"

#include "Usermain.h"
#include "Extension.h"
#include "PID.h"
#include "LegacyControlData.h"
#include <array>
#include <unordered_map>
#include <functional>

using namespace std;

class Propeller_I2C;

enum Motion_State{
    STOP,
    FLOAT,
    FRONT,
    BACK,
    LEFT,
    RIGHT,
    CLOCKWISE,
    ANTICLOCKWISE,
    MOTION_STATE_NUM
};

typedef struct Propeller_Command{
    uint8_t len;
    const char* str;
    void (Propeller_I2C::*handler)();
}Propeller_Command_t;


struct Propeller_Parameter_t{
    std::array<int8_t, 8> Sign;
    std::array<uint8_t, 4> InID;
    std::array<uint8_t, 4> OutID;
    int32_t InitPWM;
    std::array<int32_t, 8> Compensation; // 死区补偿值
    std::array<int32_t, 4> FloatPWM;
    std::array<int32_t, 4> FrontPWM;
    std::array<int32_t, 4> BackPWM;
    std::array<int32_t, 4> LeftPWM;
    std::array<int32_t, 4> RightPWM;
    std::array<int32_t, 4> ClockwisePWM;
    std::array<int32_t, 4> AnticlockwisePWM;
    std::array<int32_t, 4> StopPWM;
    PID_Regulator_t DepthPID_P;
    PID_Regulator_t PitchPID_P;
    PID_Regulator_t RollPID_P;
    PID_Regulator_t YawPID_P;
    PID_Regulator_t YawInPID_P;
    PID_Regulator_t YawOutPID_P;
    PID_Regulator_t RollInPID_P;
    PID_Regulator_t RollOutPID_P;
    PID_Regulator_t PitchInPID_P;
    PID_Regulator_t PitchOutPID_P;

    Propeller_Parameter_t() = default;
};

typedef struct Propeller_PWM_Component{
    float Depth;
    float Roll,Pitch,Yaw;
    float Vx,Vy,Vz;
    float Roll_angle, Pitch_angle, Yaw_angle;
} Propeller_PWM_Component_t;


class Propeller_I2C: public Device
{
private:
    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    // Existing requested pulse widths; neutral comes from Parameter.InitPWM (V33: 1550 us).
    // 原始请求脉宽；中位值由版本参数给出，当前 V33 为 1550 us。
    lower_controller::ThrusterPwmCommand output_command_ = {};
    //int32_t data_receive[5];
    float Target_depth;
    float Target_roll = 0.0;
    float Target_roll_v = 0.0;
    float Target_pitch = 0.0;
    float Target_pitch_v = 0.0;
    float Target_yaw = 0.0;
    float Target_yaw_v = 0.0;
    float yaw_diff;
    float yaw_v_diff;
    float pitch_diff;
    float pitch_v_diff;
    float roll_diff;
    float roll_v_diff;
    float Target_speed[3];
    bool flag_float;
    bool flag_angle;
    bool flag_range;
    bool flag_roll;
    bool flag_PWM_output;
    Motion_State motion_state;
    int roll_state;
    int roll_state_total;
    PID DepthPID, RollPID, PitchPID;
    PID VxPID, VyPID, YawPID;
    PID RollAnglePID, PitchAnglePID, YawAnglePID, YawInPID, YawOutPID;
    PID RollInPID, RollOutPID;
    PID PitchInPID, PitchOutPID;

    void float_ctrl(const lower_controller::LegacyControlFeedback& feedback);
    void speed_ctrl();
    void angle_ctrl(const lower_controller::LegacyControlFeedback& feedback);
    float deg2rad(float);
    float rad2deg(float);
    float normalize_angle(float euler_angle);

    void vertical_PWM_allocation(const lower_controller::LegacyControlFeedback& feedback);
    void horizontal_PWM_allocation(const lower_controller::LegacyControlFeedback& feedback);

    unordered_map<char, Motion_State> msg_state_map;
    std::array<std::array<int32_t, 4>, MOTION_STATE_NUM> state_PWM_map;
    Propeller_PWM_Component_t PWM_component;
    Propeller_Parameter_t Parameter;
    static Propeller_Command_t propeller_command[];

public:
    void Init();
    void Handle();
    void Receive();
    void StopControl();
#if LC_IMU_ASYNC_ENABLED
    static constexpr bool fused_feedback_enabled = true; // 本固件固定使用 ESKF + SI 控制。
    lower_controller::FusionState fused_feedback = {};
    lower_controller::FusedControlRequest BuildFusedControlRequest() const;
    bool SetFusedTarget(const char *command);
    // 深度闭环开关，DEP:ON / DEP:OFF 切换；关掉时垂直推进器只稳姿态。
    bool depth_hold_enabled = LC_DEPTH_HOLD_DEFAULT != 0;
#endif

    // Snapshot -> controller/mixer -> output driver. ComputeControl performs no I/O.
    // 快照 -> PID/混控 -> 输出驱动；计算函数只接收反馈并生成脉宽。
    lower_controller::LegacyControlFeedback CaptureFeedback() const;
    const lower_controller::ThrusterPwmCommand& ComputeControl(const lower_controller::LegacyControlFeedback& feedback);
    void WriteOutput(const lower_controller::ThrusterPwmCommand& command);
    
    void command_test();
    void command_set_angle_closeloop();
    void command_set_angle_value();
};


/*extern void Anglectrl_servo();

extern void Speedctrl_propeller_init();

extern void Speedctrl_propeller();

extern void Anglectrl_servo_init();
*/

#endif //CONTROL_FRAME_MAIN_PWM_H
