//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"
#include "LegacyEstimation.h"
#if LC_USE_FREERTOS
#include "FusionConfiguration.h"
// TODO(标度)：30BA 的旧兼容公式在 20°C 附近为 P(Pa)/2000；低温时须验证旧混合温补。
static float LegacyDepthFromCm(float cm)
{
    const auto &c = lower_controller::GetFusionConfiguration();
    return cm*0.01f*c.water_density_kg_m3*c.gravity_m_s2/2000.0f;
}
#endif
#include <algorithm>
#include <cmath>

// V33
std::array<int8_t, 8> Sign = LC_THRUSTER_SIGNS_INIT; // {1,-1,1,1,-1,-1,1,-1}；Mark propeller direction: Clockwise = 1, Counter-Clockwise = -1.; the index corresponds to the propeller ID. // 推进器正反桨，正1反-1，序号为推进器序号
std::array<uint8_t, 4> InID = LC_VERTICAL_CHANNELS_INIT;                // {1,2,6,5}；V33-1,2 Channel indices on the expansion board for the four internal thrusters: front-left, rear-left, front-right, rear-right. //内部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
std::array<uint8_t, 4> OutID = LC_HORIZONTAL_CHANNELS_INIT;               // {0,3,7,4}；V33-1,2 Channel indices on the expansion board for the four external thrusters: front-left, rear-left, front-right, rear-right. //外部的4个推进器接到扩展板上的序号，左前-左后-右前-右后
int32_t InitPWM = LC_THRUSTER_NEUTRAL_US; // 1550 us；Initial PWM value for thruster initialization. // 推进器初始化的PWM
std::array<int32_t, 8> Compensation = {LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US, LC_THRUSTER_DEADZONE_US}; // 各路 50 us；Dead-zone compensation value. // 死区补偿值
// Vertical thrusters.
// 垂直推进器
std::array<int32_t, 4> FloatPWM = {
    InitPWM,
    InitPWM - Sign[InID[1]] * 100,
    InitPWM,
    InitPWM - Sign[InID[3]] * 90
};
// Horizontal thrusters.
// 水平推进器
uint8_t longitudinal_speed = 80;
uint8_t lateral_speed = 80;
uint8_t rotate_speed = 40;
std::array<int32_t, 4> StopPWM = {InitPWM, InitPWM, InitPWM, InitPWM};
std::array<int32_t, 4> FrontPWM = {
    InitPWM - Sign[OutID[0]] * longitudinal_speed,
    InitPWM - Sign[OutID[1]] * longitudinal_speed,
    InitPWM - Sign[OutID[2]] * longitudinal_speed,
    InitPWM - Sign[OutID[3]] * longitudinal_speed
};
std::array<int32_t, 4> BackPWM = {
    InitPWM + Sign[OutID[0]] * longitudinal_speed,
    InitPWM + Sign[OutID[1]] * longitudinal_speed,
    InitPWM + Sign[OutID[2]] * longitudinal_speed,
    InitPWM + Sign[OutID[3]] * longitudinal_speed
};
std::array<int32_t, 4> LeftPWM = {
    InitPWM + Sign[OutID[0]] * lateral_speed,
    InitPWM - Sign[OutID[1]] * lateral_speed,
    InitPWM - Sign[OutID[2]] * lateral_speed,
    InitPWM + Sign[OutID[3]] * lateral_speed
};
std::array<int32_t, 4> RightPWM = {
    InitPWM - Sign[OutID[0]] * lateral_speed,
    InitPWM + Sign[OutID[1]] * lateral_speed,
    InitPWM + Sign[OutID[2]] * lateral_speed,
    InitPWM - Sign[OutID[3]] * lateral_speed
};
std::array<int32_t, 4> ClockwisePWM = {
    InitPWM - Sign[OutID[0]] * rotate_speed,
    InitPWM - Sign[OutID[1]] * rotate_speed,
    InitPWM + Sign[OutID[2]] * rotate_speed,
    InitPWM + Sign[OutID[3]] * rotate_speed
};
std::array<int32_t, 4> AntiClockwisePWM = {
    InitPWM + Sign[OutID[0]] * rotate_speed,
    InitPWM + Sign[OutID[1]] * rotate_speed,
    InitPWM - Sign[OutID[2]] * rotate_speed,
    InitPWM - Sign[OutID[3]] * rotate_speed
};
// PID
PID_Regulator_t DepthPID(10, 0.02, 10, 100, 50, 50, 200);
PID_Regulator_t PitchPID(10, 0.02, 50, 50, 25, 25, 100);
PID_Regulator_t RollPID(1.8, 0.02, 100, 50, 25, 25, 100);
PID_Regulator_t YawPID(12, 0.06, 300, 10, 100, 50, 300);
PID_Regulator_t YawInPID(5, 0, 0, 200, 100, 100, 400);
PID_Regulator_t YawOutPID(2, 0.01, 2, 10, 5, 5, 20);
PID_Regulator_t RollInPID(8, 0, 0, 100, 50, 50, 200);
PID_Regulator_t RollOutPID(0.5, 0.002, 1, 5, 2.5, 2.5, 10);
PID_Regulator_t PitchInPID(8, 0, 0, 100, 50, 50, 200);
PID_Regulator_t PitchOutPID(1, 0.005, 1, 5, 2.5, 2.5, 10);
// IMU
// PID_Regulator_t PitchInPID(2, 0, 40, 100, 50, 50, 200);
// PID_Regulator_t PitchOutPID(7, 0.5, 100, 5, 2.5, 2.5, 10);

Propeller_Parameter_t Parameter_V33 = {
    Sign, InID, OutID, InitPWM,
    Compensation,
    FloatPWM, FrontPWM, BackPWM, LeftPWM, RightPWM,
    ClockwisePWM, AntiClockwisePWM, StopPWM,
    DepthPID, PitchPID, RollPID, YawPID,
    YawInPID, YawOutPID, RollInPID, RollOutPID, PitchInPID, PitchOutPID
};

Propeller_Command_t Propeller_I2C::propeller_command[] = {
    {3, "ACL", &Propeller_I2C::command_set_angle_closeloop},
    {3, "ANG", &Propeller_I2C::command_set_angle_value},
    {3, "TES", &Propeller_I2C::command_test}
};

void Propeller_I2C::Init()
{
    Parameter = Parameter_V33;

    DepthPID.PIDInfo = Parameter.DepthPID_P;
    PitchPID.PIDInfo = Parameter.PitchPID_P;
    RollPID.PIDInfo = Parameter.RollPID_P;
    YawPID.PIDInfo = Parameter.YawPID_P;

    PitchAnglePID.PIDInfo = Parameter.PitchPID_P;
    RollAnglePID.PIDInfo = Parameter.RollPID_P;
    YawAnglePID.PIDInfo = Parameter.YawPID_P;

    YawInPID.PIDInfo = Parameter.YawInPID_P;
    YawOutPID.PIDInfo = Parameter.YawOutPID_P;

    RollInPID.PIDInfo = Parameter.RollInPID_P;
    RollOutPID.PIDInfo = Parameter.RollOutPID_P;

    PitchInPID.PIDInfo = Parameter.PitchInPID_P;
    PitchOutPID.PIDInfo = Parameter.PitchOutPID_P;

    Target_speed[0] = 0;
    Target_speed[1] = 0;
    flag_float = false;
    flag_roll = false;
    flag_PWM_output = false;
    flag_angle = false;
    roll_state = 0;
    roll_state_total = 1;
    motion_state = STOP;

    msg_state_map['W'] = FRONT;
    msg_state_map['S'] = BACK;
    msg_state_map['A'] = LEFT;
    msg_state_map['D'] = RIGHT;
    msg_state_map['Z'] = FLOAT;
    msg_state_map['E'] = CLOCKWISE;
    msg_state_map['Q'] = ANTICLOCKWISE;

    state_PWM_map[STOP]          = Parameter.StopPWM;
    state_PWM_map[FRONT]         = Parameter.FrontPWM;
    state_PWM_map[BACK]          = Parameter.BackPWM;
    state_PWM_map[LEFT]          = Parameter.LeftPWM;
    state_PWM_map[RIGHT]         = Parameter.RightPWM;
    state_PWM_map[FLOAT]         = Parameter.StopPWM;
    state_PWM_map[CLOCKWISE]     = Parameter.ClockwisePWM;
    state_PWM_map[ANTICLOCKWISE] = Parameter.AnticlockwisePWM;

    TCA_SetChannel(LC_PWM_MUX_CHANNEL);  // 4：PWM 扩展板所在 TCA 通道；Expand a single I2C bus into eight channels. // 1路 I2C 扩展为 8路
    HAL_Delay(5);
    PCA_Write(PCA9685_MODE1, 0x0);
    PCA_Setfreq(LC_PWM_FREQUENCY_HZ); // 50 Hz；Hz
    for (int i = 0; i < PROPELLER_NUM; ++i)
    {
        output_command_.pulse_us[i] = Parameter.InitPWM;
				
        // PCA channels 0..7 follow the original V33 wiring.
        PCA_Setpwm(i, 0, floor(output_command_.pulse_us[i] * LC_PWM_COUNTS / LC_PWM_PERIOD_US + 0.5f)); // 4096 计数 / 20000 us
    }

#if !LC_USE_FREERTOS
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
#endif
};

// Configure the PWM parameters according to UART commands.
// 根据串口指令设置PWM参数
void Propeller_I2C::Receive()
{
    int32_t data_receive[8];
    uint8_t command_num = sizeof(propeller_command) / sizeof(Propeller_Command_t);
    for (uint8_t i=0; i<command_num; i++)
    {
        if (strncmp((char *)RxBuffer, propeller_command[i].str, propeller_command[i].len) == 0)
        {
            (this->*(propeller_command[i].handler))();
        }
    }
    if (flag_float)
    {
			// Update the PWM outputs of the four outer-ring motors for front/back/left/right motion (open-loop motion is possible when yaw-angle control is disabled).
        // 外圈四个电机更新为前后左右运动的PWM(在没有yaw角控制的情况下可开环运动)
        if (strncmp((char *)RxBuffer, "DN", 2) == 0)
        {
#if LC_IMU_ASYNC_ENABLED
            Target_depth += fused_feedback_enabled ? 1.0f : LegacyDepthFromCm(1);
#elif LC_USE_FREERTOS
            Target_depth += LegacyDepthFromCm(1);
#else
            Target_depth += 1;
#endif
            return;
        }

        else if (strncmp((char *)RxBuffer, "UP", 2) == 0)
        {
#if LC_IMU_ASYNC_ENABLED
            Target_depth = fused_feedback_enabled ? fmaxf(0.0f, Target_depth-1.0f) : Target_depth-LegacyDepthFromCm(1);
#elif LC_USE_FREERTOS
            Target_depth -= LegacyDepthFromCm(1);
#else
            Target_depth -= 1;
#endif
            return;
        }

        else if (msg_state_map.count(RxBuffer[0]))
        {
            motion_state = msg_state_map[RxBuffer[0]];
            // if(flag_angle){
            //     for (int i = 0; i < 4; ++i)
            //     {
            //         output_command_.pulse_us[Parameter.OutID[i]] = output_command_.pulse_us[Parameter.OutID[i]] + state_PWM_map[motion_state][i] - Parameter.InitPWM;
            //     }
            // }
            // else{
            //     for (int i = 0; i < 4; ++i)
            //     {
            //         output_command_.pulse_us[Parameter.OutID[i]] = state_PWM_map[motion_state][i];
            //     }
            // }
        }

				// Disable PID control and reset the PWM outputs to their initial values.
        // 关闭PID控制，更新为初始的PWM
        else if (strncmp((char *)RxBuffer, "OFF", 3) == 0)
        {
            flag_float = false;
            flag_angle = false;
            for (int i = 0; i < 8; ++i)
            {
                output_command_.pulse_us[i] = Parameter.InitPWM;
            }
            Target_depth = 30;
            output_command_.pulse_us[Parameter.OutID[0]] = output_command_.pulse_us[Parameter.OutID[1]] = output_command_.pulse_us[Parameter.OutID[2]] = output_command_.pulse_us[Parameter.OutID[3]] = Parameter.InitPWM;
        }

        // // 外圈四个电机更新为给定的PWM，更新深度
        // if (strncmp((char *)RxBuffer, "PRO:", 4) == 0)
        // {
        //     char *data_str = (char *)RxBuffer + 4;
        //     char *token = strtok(data_str, ",");
        //     int i = 0;
        //     while (token != NULL && i < 5)
        //     {
        //         data_receive[i] = atoi(token);
        //         token = strtok(NULL, ",");
        //         i++;
        //     }
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         output_command_.pulse_us[Parameter.OutID[i]] = data_receive[i];
        //     }
        //     Target_depth = data_receive[4] / 10.0;
        // }

        // // 更新速度与角速度
        // if (strncmp((char *)RxBuffer, "VEL:", 4) == 0)
        // {
        //     char *data_str = (char *)RxBuffer + 4;
        //     char *token = strtok(data_str, ",");
        //     int i = 0;
        //     while (token != NULL && i < 3)
        //     {
        //         data_receive[i] = atoi(token);
        //         token = strtok(NULL, ",");
        //         i++;
        //     }
        //     Target_speed[0] = data_receive[0];
        //     Target_speed[1] = data_receive[1];
        //     Target_yaw = data_receive[2] * 3.14 / 180;
        // }

        // Update Yaw. //更新Yaw角度
        else if (strncmp((char *)RxBuffer, "RPY:", 4) == 0)
        {
            if (strncmp((char *)RxBuffer, "RPY:ON", 6) == 0){
                flag_angle = true;
            }
            else if (strncmp((char *)RxBuffer, "RPY:OFF", 7) == 0){
                flag_angle = false;
                output_command_.pulse_us[Parameter.OutID[0]] = output_command_.pulse_us[Parameter.OutID[1]] = output_command_.pulse_us[Parameter.OutID[2]] = output_command_.pulse_us[Parameter.OutID[3]] = Parameter.InitPWM;
            }
        //     else {
        //         char *data_str = (char *)RxBuffer + 4;
        //         char *token = strtok(data_str, ",");
        //         int i = 0;
        //         while (token != NULL && i < 3)
        //         {
        //             data_receive[i] = atoi(token);
        //             token = strtok(NULL, ",");
        //             i++;
        //         }
        //         Target_roll = data_receive[0] * 3.14 / 180;
        //         Target_pitch = data_receive[1] * 3.14 / 180;
        //         Target_yaw = data_receive[2] * 3.14 / 180;
        //     }
        }

        // // 当开启角度控制时先默认输出角度数据，可以通过串口输入改为输出PWM数据
        // if(flag_angle){
        //     if (strncmp((char *)RxBuffer, "PWM:BEG", 7) == 0){
        //         flag_PWM_output = true;
        //     }
        //     else if (strncmp((char *)RxBuffer, "PWM:END", 7) == 0){
        //         flag_PWM_output = false;
        //     }
        // }

        // Update Depth. //更新深度
        else if (strncmp((char *)RxBuffer, "H:", 2) == 0)
        {
            char *data_str = (char *)RxBuffer + 2;
            Target_depth = atoi(data_str) / 10.0;
#if LC_IMU_ASYNC_ENABLED
            if (!fused_feedback_enabled) Target_depth = LegacyDepthFromCm(Target_depth);
#elif LC_USE_FREERTOS
            Target_depth = LegacyDepthFromCm(Target_depth);
#endif
        }
    }

    else
    {
				// Enable PID control and initialize the PWM outputs to their default values.
        // 开启PID控制，更新为初始PWM
        if (strncmp((char *)RxBuffer, "ON", 2) == 0)
        {
            flag_float = true;
            for (int i = 0; i < 8; ++i)
            {
                output_command_.pulse_us[i] = Parameter.InitPWM;
            }
#if LC_IMU_ASYNC_ENABLED
            if (fused_feedback_enabled) Target_depth = fmaxf(0.0f, fused_feedback.depth_m) * 100.0f; // 新模式 ON 保持当前深度；内部协议缓存仍按 cm。
            else
#endif
#if LC_USE_FREERTOS
            Target_depth = PressureSensor::pressure_sensor.data_depth; // ON 保持当前深度，不套用旧 30 刻度最小目标。
#else
            Target_depth = (PressureSensor::pressure_sensor.data_depth - 1.0 > 30.0) ? (PressureSensor::pressure_sensor.data_depth - 1.0) : 30.0;
#endif
        }
    }
}

void Propeller_I2C::StopControl()
{
    Target_roll = Target_pitch = Target_yaw = 0.0f;
    DepthPID.Reset(); RollInPID.Reset(); RollOutPID.Reset();
    PitchInPID.Reset(); PitchOutPID.Reset(); YawInPID.Reset(); YawOutPID.Reset();
    flag_float = false;
    flag_angle = false;
    motion_state = STOP;
    Target_depth = 30;
    for (int i = 0; i < LC_THRUSTER_COUNT; ++i) output_command_.pulse_us[i] = LC_THRUSTER_NEUTRAL_US; // 8 路，1550 us
}

void Propeller_I2C::Handle()
{
    // Retain the existing mux selection before control and the channel write order.
    // 保留原来的选通道位置和输出顺序；显式分离快照、计算和写出。
    TCA_SetChannel(LC_PWM_MUX_CHANNEL); // 4：PWM 扩展板所在 TCA 通道
    const lower_controller::LegacyControlFeedback feedback = CaptureFeedback(); // 旧压力刻度、yaw(rad)、角速度(rad/s)
    const lower_controller::ThrusterPwmCommand& command = ComputeControl(feedback); // 8 路 PWM 请求值，单位 us
    WriteOutput(command); // 请求值限幅到 1000..2000 us 后写出
}

lower_controller::LegacyControlFeedback Propeller_I2C::CaptureFeedback() const
{
    lower_controller::LegacyControlFeedback feedback = {};
    // Copy only a few scalar fields while preserving the caller's interrupt mask.
    // 短临界区只拷贝标量，防止姿态被 DMA 中断更新一半；恢复调用前的屏蔽状态。
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const lower_controller::LegacyPressureFeedback& pressure = PressureSensor::pressure_sensor.Feedback();
    feedback.depth_legacy = pressure.depth_legacy;
    feedback.roll_difference_legacy = pressure.roll_difference_legacy;
    feedback.pitch_difference_legacy = pressure.pitch_difference_legacy;
    feedback.outer_loop_due = pressure.outer_loop_due;
    feedback.yaw_rad = IMU::imu.attitude.yaw;
    feedback.roll_rate_feedback_rad_s = IMU::imu.attitude.neg_roll_v;
    feedback.pitch_rate_feedback_rad_s = IMU::imu.attitude.pitch_v;
    feedback.yaw_rate_feedback_rad_s = IMU::imu.attitude.yaw_v;
    __set_PRIMASK(interrupt_mask);
    return feedback;
}

const lower_controller::ThrusterPwmCommand& Propeller_I2C::ComputeControl(
    const lower_controller::LegacyControlFeedback& feedback)
{
    vertical_PWM_allocation(feedback);
    horizontal_PWM_allocation(feedback);
    // Borrowed view of existing command storage; UART command semantics stay unchanged.
    // 返回内部存储的借用视图；控制任务必须复制到 BusRequest 后再跨任务发送。
    return output_command_;
}

void Propeller_I2C::WriteOutput(const lower_controller::ThrusterPwmCommand& command)
{
    // Caller owns/selects I2C2 channel 4; preserve per-channel clamp/rounding and writes.
    // 调用方负责选通道；按原顺序逐通道限幅、换算和写入，不修改原始请求脉宽。
    for (int i = 0; i < PROPELLER_NUM; ++i) {
        const int32_t duty_count = lower_controller::LegacyThrusterPwmCount(command.pulse_us[i]); // us 转 PCA 计数，1550 us 对应约 317
        PCA_Setpwm(i, 0, duty_count); // i=PCA 0..7；duty_count 为计数值
    }
}

void Propeller_I2C::float_ctrl(const lower_controller::LegacyControlFeedback& feedback)
{
        PWM_component.Depth = DepthPID.PIDCalc(Target_depth, feedback.depth_legacy); // 反馈：四路压力均值，旧深度刻度

        // pid for roll and pitch
        // outer_loop_due preserves the legacy 3:1 inner/outer cadence and phase.
        // 显式布尔接口保留原内外环 3:1 节拍及相位，控制器不再读取驱动状态。
        if (feedback.outer_loop_due) // true：旧姿态外环触发相位，使用上一帧压力
        {
            // roll_diff = PressureSensor::pressure_sensor.data_roll;
            // roll_diff = IMU::imu.attitude.roll;
            // Target_roll_v = RollOutPID.PIDCalc(Target_roll, IMU::imu.attitude.roll);
            Target_roll_v = RollOutPID.PIDCalc(Target_roll, feedback.roll_difference_legacy); // 反馈：p0+p1-p2-p3，旧压差
            // pitch_diff = PressureSensor::pressure_sensor.data_pitch;
            // pitch_diff = IMU::imu.attitude.pitch;
            // Target_pitch_v = PitchOutPID.PIDCalc(Target_pitch, IMU::imu.attitude.pitch);
            Target_pitch_v = PitchOutPID.PIDCalc(Target_pitch, feedback.pitch_difference_legacy); // 反馈：p0+p3-p1-p2，旧压差
        }
        // PWM_component.Roll = RollInPID.PIDCalc(Target_roll_v, IMU::imu.attitude.roll_v); // IMU
        PWM_component.Roll = RollInPID.PIDCalc(Target_roll_v, feedback.roll_rate_feedback_rad_s); // 反馈：-gyro_x，rad/s
        PWM_component.Pitch = PitchInPID.PIDCalc(Target_pitch_v, feedback.pitch_rate_feedback_rad_s); // 反馈：+gyro_y，rad/s
}

void Propeller_I2C::speed_ctrl()
{
}

// Dual-loop PID control for yaw angle.
// 双环pid控制Yaw角
void Propeller_I2C::angle_ctrl(const lower_controller::LegacyControlFeedback& feedback)
{
    if (feedback.outer_loop_due) // true：旧姿态外环触发相位，使用上一帧压力
    {
        yaw_diff = normalize_angle(feedback.yaw_rad - Target_yaw); // yaw 为 rad
        Target_yaw_v = YawOutPID.PIDCalc(0.0, yaw_diff);
    }
    PWM_component.Yaw = YawInPID.PIDCalc(Target_yaw_v, feedback.yaw_rate_feedback_rad_s); // 反馈：+gyro_z，rad/s
}

float Propeller_I2C::deg2rad(float degree){
    return degree * PI / 180;
}

float Propeller_I2C::rad2deg(float rad){
    return rad * 180 / PI;
}

// PWM allocation for the internal (vertical-axis) thrusters.
// 垂直方向（内）推进器PWM分配
void Propeller_I2C::vertical_PWM_allocation(const lower_controller::LegacyControlFeedback& feedback)
{
    if(flag_float){
        float_ctrl(feedback);

        constexpr int8_t factors[4][3] = {
        {-1, -1, -1},  // Motor0
        {-1, -1,  1},  // Motor1
        {-1,  1, -1},  // Motor2
        {-1,  1,  1}   // Motor3
        };

        float depth = PWM_component.Depth;
        float pitch = PWM_component.Pitch;
        float roll = PWM_component.Roll;

        for(int i=0; i<4; i++)
        {
            uint8_t idx = Parameter.InID[i];
            int8_t sign = Parameter.Sign[idx];
            int32_t base = Parameter.FloatPWM[i];
            int32_t comp = Parameter.Compensation[idx];
            int32_t pwm = base - sign * (depth * factors[i][0] + roll * factors[i][1] + pitch * factors[i][2]);
            if (pwm > Parameter.InitPWM) pwm += comp;
            else if(pwm < Parameter.InitPWM) pwm -= comp; // Compensate for the motor dead zone. // 补偿电机死区
            output_command_.pulse_us[idx] = pwm;
        }
    }
    else{
        for(int i=0; i<PROPELLER_NUM; i++){
            //output_command_.pulse_us[i] = Parameter.InitPWM;
        }
    }
}

// PWM allocation for the horizontal-axis thrusters.
// 水平方向推进器PWM分配
void Propeller_I2C::horizontal_PWM_allocation(const lower_controller::LegacyControlFeedback& feedback)
{
    float yaw;
	  if(flag_float){
			if(flag_angle){
        angle_ctrl(feedback);
        yaw = PWM_component.Yaw;
			}
			else{
					yaw = 0;
			}
			for (int i=0; i<4; i++)
			{
					uint8_t idx = Parameter.OutID[i];
					int32_t sign = Parameter.Sign[idx];
					int32_t base = state_PWM_map[motion_state][i];
					int32_t comp = Parameter.Compensation[idx];
					int32_t pwm = base + ((i<2) ? sign : - sign) * yaw;
					if (pwm > Parameter.InitPWM) pwm += comp;
					else if(pwm < Parameter.InitPWM) pwm -= comp; // Compensate for the motor dead zone. // 补偿电机死区
					output_command_.pulse_us[idx] = pwm;
			}
		}
}


float Propeller_I2C::normalize_angle(float angle)
{
    angle = std::fmod(angle + PI, 2 * PI);
    if (angle < 0) angle += 2 * PI;
    return angle - PI;
}

void Propeller_I2C::command_set_angle_value(){
    if (flag_angle && flag_float)
    {
        char* data_str = (char *)RxBuffer + 4;
        Target_yaw = deg2rad(atoi(data_str));
    }
}

void Propeller_I2C::command_set_angle_closeloop(){
    if (flag_float)
    {
        char* data_str = (char *)RxBuffer + 4;
        if (strncmp(data_str, "ON", 2) == 0)
        {
            flag_angle = true;
#if LC_IMU_ASYNC_ENABLED
            Target_yaw = fused_feedback_enabled ? fused_feedback.euler_rad[2] : IMU::imu.attitude.yaw; // rad；模式决定参考姿态。
#else
            Target_yaw = IMU::imu.attitude.yaw;
#endif
        }
        else if (strncmp(data_str, "OF", 2) == 0)
        {
            flag_angle = false;
        }
    }
}

void Propeller_I2C::command_test(){
    if (!flag_float && !flag_angle)
    {
        char* data_str = (char *)RxBuffer + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < 8)
        {
            output_command_.pulse_us[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}
