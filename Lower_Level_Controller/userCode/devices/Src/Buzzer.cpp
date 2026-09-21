//
// Created by admin on 2023/12/9.
//
#include "Buzzer.h"

void Buzzer::Init(){
    BuzzerFlag = 0;
    buzzerWorkingFlag = 0;
    // TIM4 原始配置的 168 MHz 计数频率无法直接容纳可听频率的 ARR；
    // 降到 2 MHz 后，1 kHz 音调只需 ARR=1999，寄存器不会溢出。
    BUZZER_CLOCK.Instance->PSC = BUZZER_TIMER_PRESCALER;
    HAL_TIM_Base_Start(&BUZZER_CLOCK);
    HAL_TIM_PWM_Start(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);

}

void Buzzer::StartupBeep(){
    // 该函数只在 Usermain 完成全部设备初始化和液面标定后调用。
    bsp_BuzzerOn(1000.0f, 1.0f); // 1 kHz、约 50% 占空比
    HAL_Delay(180);
    bsp_BuzzerOff();
}

void Buzzer::Receive(){

}

void Buzzer::Handle(){
    if(BuzzerFlag <= 3) {
        bsp_BuzzerOn(50,1);
        BuzzerFlag++;
    }
    else bsp_BuzzerOff();
}

float Buzzer::bsp_BuzzerOn(float _freq,float _targetVolPct){



    uint16_t arr,cpr;
    arr = (uint16_t)(BUZZER_TIMER_TICK_HZ / _freq) - 1U;
    //cpr = (uint16_t)(duty*(float)arr);
    cpr = (uint16_t)(_targetVolPct*REFERENCE_MAX_VOL_CCR*0.5);
    if(cpr > arr/2){
        cpr = arr/2;
    }


    if(!buzzerWorkingFlag){
        buzzerWorkingFlag = 1;

    }
    BUZZER_CLOCK.Instance->ARR = arr;
    __HAL_TIM_SetCompare(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL,cpr);

    return (float)cpr/(float)arr*2;
}

void Buzzer::bsp_BuzzerOff(){
    if(buzzerWorkingFlag){
        buzzerWorkingFlag = 0;
        HAL_TIM_PWM_Stop(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);
    }
}
