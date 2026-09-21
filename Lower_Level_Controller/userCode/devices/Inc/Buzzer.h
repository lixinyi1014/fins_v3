//
// Created by admin on 2023/12/9.
//

#ifndef CONTROL_FRAME_MAIN_BUZZER_H
#define CONTROL_FRAME_MAIN_BUZZER_H

#include "Usermain.h"

#define BUZZER_CLOCK_FREQUENCY 168000000U // STM32F407 当前时钟树下 TIM4 输入时钟
#define BUZZER_CLOCK htim4
#define BUZZER_CLOCK_CHANNEL TIM_CHANNEL_3
#define REFERENCE_MAX_VOL_CCR 5000
#define BUZZER_TIMER_TICK_HZ 2000000U // TIM4 168 MHz / (PSC 83 + 1)
#define BUZZER_TIMER_PRESCALER 83U

class Buzzer: public Device{

    float bsp_BuzzerOn(float _freq,float _targetVolPct);
    void bsp_BuzzerOff();

public:

    uint32_t BuzzerFlag;
    uint8_t buzzerWorkingFlag;

    void Init();
    void StartupBeep(); // 所有设备与水面标定成功后发出一次短音
    void Receive();
    void Handle();

};


#endif //CONTROL_FRAME_MAIN_BUZZER_H
