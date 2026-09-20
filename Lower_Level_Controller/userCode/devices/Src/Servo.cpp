//
// Created by admin on 2023/12/5.
//
#include "Servo.h"
#include "CommandValidation.h"

int32_t ID_V30[4] = {8, 9, 10, 11}; // Channel indices on the PWM expansion board for the four servos: upper-left, lower-left, upper-right, lower-right. // 4个舵机接在PWM扩展板的编号，左上-左下-右上-右下
int32_t ID_V31[4] = {8, 9, 10, 11};
int32_t ID_V32[4] = {8, 9, 11, 10};
int32_t ID_V33[4] = LC_SERVO_V33_CHANNELS_INIT; // {8,9,10,11}
int32_t ID_V40[4] = {8, 9, 10, 11}; // Test needed. //还需测试

void Servo::Init()
{
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        data[i] = 1500;
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
#if !LC_USE_FREERTOS
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
#endif
}

void Servo_I2C::Init()
{
    switch (Robot_Version)
    {
    case V30:
        std::memcpy(ID, ID_V30, sizeof(ID));
        break;
    case V31:
        std::memcpy(ID, ID_V31, sizeof(ID));
        break;
    case V32:
        std::memcpy(ID, ID_V32, sizeof(ID));
        break;
    case V33:
        std::memcpy(ID, ID_V33, sizeof(ID));
        break;
    case V40:
        std::memcpy(ID, ID_V40, sizeof(ID));
        break;
    }
    TCA_SetChannel(LC_PWM_MUX_CHANNEL); // 4：PWM 扩展板所在 TCA 通道
    PCA_Write(PCA9685_MODE1, 0x0);
    PCA_Setfreq(LC_PWM_FREQUENCY_HZ); // 50 Hz；Hz
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        data[i] = 1500;
        // V3.3选定ID={8,9,10,11}，初始化与运行使用同一通道表；见docs/PORT_MAP_V33.md。
        PCA_Setpwm(ID[i], 0, floor(data[i] * LC_PWM_COUNTS / LC_PWM_PERIOD_US + 0.5f)); // 4096 计数 / 20000 us
    }
#if !LC_USE_FREERTOS
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
#endif
}

void Servo::Receive()
{
    data_extract(RxBuffer, data, SERVO_NUM);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo_I2C::Receive()
{
    data_extract(RxBuffer, data, SERVO_NUM);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo::Handle()
{
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
}

void Servo_I2C::Handle()
{
    TCA_SetChannel(LC_PWM_MUX_CHANNEL); // 4：PWM 扩展板所在 TCA 通道
    WriteOutput(BuildOutput()); // 4 路舵机脉宽，单位 us
}

lower_controller::ServoPwmCommand Servo_I2C::BuildOutput() const
{
    lower_controller::ServoPwmCommand command = {};
    command.in_range = true;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < SERVO_NUM; ++i) {
        command.channel[i] = ID[i];
        command.pulse_us[i] = data[i];
    }
    __set_PRIMASK(interrupt_mask);
    // Preserve the all-or-nothing range check for the four servos.
    // 保留四路整体范围检查：任意一路越界，本轮四路都不写出。
    for (int i = 0; i < SERVO_NUM; ++i) {
        if (command.pulse_us[i] < LC_SERVO_MIN_US || command.pulse_us[i] > LC_SERVO_MAX_US) // 有效范围：500..2500 us
            command.in_range = false;
    }
    return command;
}

void Servo_I2C::WriteOutput(const lower_controller::ServoPwmCommand& command)
{
    if (command.in_range) {
        for (int i = 0; i < SERVO_NUM; ++i) {
            // Retain the original INTEGER division before floor; retuning is a later change.
            // 保留原先先整数除法、再 floor 的换算顺序，不在职责拆分时改变脉宽。
            PCA_Setpwm(command.channel[i], 0,
                floor(command.pulse_us[i] * LC_PWM_COUNTS / LC_PWM_PERIOD_US + 0.5f)); // 4096 计数 / 20000 us
        }
    }
}

void Servo::data_extract(uint8_t *rx, int32_t *data, int32_t num)
{
    int32_t candidate[SERVO_NUM];
    if (num != SERVO_NUM || strncmp(reinterpret_cast<char *>(rx), "MOT:", 4) ||
        !lower_controller::ParseIntegerList(reinterpret_cast<char *>(rx)+4, SERVO_NUM,
                                           LC_SERVO_MIN_US, LC_SERVO_MAX_US, candidate)) return;
    memcpy(data, candidate, sizeof(candidate));
}
void Servo_I2C::data_extract(uint8_t *rx, int32_t *data, int32_t num)
{
    int32_t candidate[SERVO_NUM];
    if (num != SERVO_NUM || strncmp(reinterpret_cast<char *>(rx), "MOT:", 4) ||
        !lower_controller::ParseIntegerList(reinterpret_cast<char *>(rx)+4, SERVO_NUM,
                                           LC_SERVO_MIN_US, LC_SERVO_MAX_US, candidate)) return;
    candidate[2] = 3000-candidate[2];
    memcpy(data, candidate, sizeof(candidate));
}
