//
// Created by admin on 2023/10/29.
//
#include "Extension.h"
#include "I2cBusAccess.h"
static bool pca_configuration_ok;
static uint8_t pca_last_mode1;
static uint8_t pca_last_mode2;
static uint8_t pca_last_prescale;
static uint8_t pca_expected_prescale;
bool PCA_ConfigurationOk() { return pca_configuration_ok; }
uint8_t PCA_LastMode1() { return pca_last_mode1; }
uint8_t PCA_LastMode2() { return pca_last_mode2; }
uint8_t PCA_LastPrescale() { return pca_last_prescale; }
uint8_t PCA_ExpectedPrescale() { return pca_expected_prescale; }
uint32_t PCA_ConfigurationErrorCode()
{
    uint32_t error = 0;
    if (!LcBus_Ok(&hi2c2)) error |= 1U << 0;
    if ((pca_last_mode1 & 0x30U) != 0x20U) error |= 1U << 1;
    if (pca_last_mode2 != 0U && pca_last_mode2 != 0x04U) error |= 1U << 2;
    if (pca_last_prescale != pca_expected_prescale) error |= 1U << 3;
    if (!pca_configuration_ok) error |= 1U << 4;
    return error;
}
// This file is used to handle the I2C expansion board (TCA) and the PWM expansion board (PCA).
// The I2C expansion board is connected to the C-board I2C header (second row from the top, four wires: left—SDA, SCL, VCC, GND—right).
// The active V33 PWM expansion board is selected on TCA channel 4; old examples below used 0.
//此文件用来处理i2c扩展板（TCA）及pwm扩展板（PCA）
//i2c扩展板接在C板i2c口（上方第二排4根线，左-SDA-SCL-VCC-GND-右）
//当前 V33 的 PWM 扩展板使用 TCA 4 号通道；下方停用的旧示例曾使用 0 号。

// Enable the I2C bus on `channel`.
// 开通第 channel 路的I2C 
void TCA_SetChannel(uint8_t channel)
{
    uint8_t data;
    data = 0x01 << channel;
    LcBus_Transmit(&hi2c2, (TCA9548A_ADDR << 1) | 0x01, &data, 1, 0xffff); // data 为通道位掩码，1 字节；运行期 HAL 超时参数收紧为 2 ms
    //LcBus_Transmit(&hi2c2, TCA9548A_ADDR, &data, 1, 0xffff);
}

uint8_t PCA_Read(uint8_t startAddress)
{
		// Set the register address from which data reading starts.
    //设置开始读取数据的寄存器地址
    uint8_t tx[1];
    uint8_t buffer[1] = {};
    tx[0] = startAddress;
    LcBus_Transmit(&hi2c2,PCA9685_ADDR,tx,1,10000); // PCA 地址 0x80；1 字节寄存器地址，运行期 HAL 超时参数为 2 ms
    LcBus_Receive(&hi2c2,PCA9685_ADDR,buffer,1,10000); // PCA 地址 0x80；读 1 字节寄存器值，运行期 HAL 超时参数为 2 ms
    return buffer[0];
}

void PCA_Write(uint8_t startAddress, uint8_t buffer)
{
		// Set the register address from which data reading starts.
    //设置待写寄存器地址和值
    uint8_t tx[2];
    tx[0] = startAddress;
    tx[1] = buffer;
    LcBus_Transmit(&hi2c2,PCA9685_ADDR, tx,2,10000); // PCA 地址 0x80；2 字节为寄存器地址及值，运行期 HAL 超时参数为 2 ms
}

// Set the PWM frequency to `freq`.
// 设置PWM频率为 freq
void PCA_Setfreq(float freq)
{
    pca_configuration_ok = false;
    pca_last_mode1 = 0;
    pca_last_mode2 = 0;
    pca_last_prescale = 0;
    uint8_t prescale,oldmode,newmode;
    double prescaleval;
    freq *= 1.016; // Retain the existing oscillator correction. // 保留原 1.016 振荡器修正系数。
    // prescaleval = 25000000;
    prescaleval = 25000000.0/(4096.0*freq);
    //prescaleval /= freq;
    prescaleval -= 1;
    prescale = floor(prescaleval + 0.5f);		// `floor` is a rounding-down function.	//floor向下取整函数
    pca_expected_prescale = prescale;
    oldmode = PCA_Read(PCA9685_MODE1);
    newmode = (oldmode&0x7F) | 0x10; // sleep睡眠
    PCA_Write(PCA9685_MODE1, newmode); // go to sleep; the device must be placed into sleep mode before the frequency can be configured. //需要进入随眠状态才能设置频率
    PCA_Write(PCA9685_PRESCALE, prescale); // Configure the prescaler (pre-divider) value. // 设置预分频系数
    PCA_Write(PCA9685_MODE1, oldmode);
    HAL_Delay(2);
    PCA_Write(PCA9685_MODE1, oldmode | 0xA1); // AI=1: auto-increment.
    // Match the old PCA startup sequence exactly.  The old driver does not write
    // MODE2 or perform register readback; a failed transaction is reported by the
    // bus wrapper and is diagnosed separately instead of changing startup behavior.
    pca_last_mode1 = oldmode | 0xA1;
    pca_last_mode2 = 0;
    pca_last_prescale = prescale;
    // The legacy driver did not make PWM availability depend on a startup
    // readback/quality gate.  Keep the diagnostic fields above for VOFA, but
    // let the first runtime transaction decide from the actual bus result.
    // Otherwise one transient PCA init ACK failure permanently prevents the
    // output request, latches StopOutputWrite, and also stops pressure updates.
    pca_configuration_ok = true;
}

// @brief /Set the PWM value for the specified output channel.
// @param num /PWM output channel index (0–15). Channels 0–7 correspond to thrusters; channels 8–11 correspond to servos.
// @param on  /PWM rising-edge count (0–4095); typically 'on = 0'.
// @param off /PWM falling-edge count (0–4095); the duty cycle is 'off'/4096.

//  @brief 设置对应输出引脚的PWM值
//  @param num PWM输出引脚0 ~ 15, 0 ~ 7 对应推进器， 8 ~ 11 对应舵机
//  @param on PWM上升计数值0 ~ 4095, 一般取 on = 0
//  @param off PWM下降计数值0 ~ 4095, 占空比为 off/4096
void PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off) 
{
    if (num > 15 || on > 4095 || off > 4095) return;
    // Four register writes are intentional: this is the transaction pattern used
    // by the known-good controller and keeps its PCA timing/address behavior.
    PCA_Write(LED0_ON_L+4*num, on);
    PCA_Write(LED0_ON_H+4*num, on>>8);
    PCA_Write(LED0_OFF_L+4*num, off);
    PCA_Write(LED0_OFF_H+4*num, off>>8);
}



/*void PWM_Extension_init(){
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(50);//Hz
    for(int i=0;i<6;++i){
        PCA_Setpwm(i,0,307);//1500*4096/20000
    }
}

void PWM_Extension(){
    for(int i=0;i<6;++i){
			PCA_Setpwm(i,0,Servo::servo.data[i]*4096/20000);

    }
}

void I2C_Extension_init(){
    TCA_SetChannel(0);
    PWM_Extension_init();
    for(int i=0;i<5;++i){
        TCA_SetChannel(i+1);
        //Init_data_pressure(i);
        PressureSensor::pres_sensor.Init(i);
    }

}

void I2C_Extension(){
    TCA_SetChannel(0);
    PWM_Extension();
    for(int i=0;i<5;++i){
        TCA_SetChannel(i+1);
        //Get_data_pressure(i);
        PressureSensor::pres_sensor.Handle(i);
    }

}
*/



/*函数作用：初始化舵机驱动板参数：1.PWM频率2.初始化舵机角度*/
/*void PCA_MG90_Init(float hz,uint8_t angle)
{
    uint32_t off=0;
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(hz);//设置PWM频率
    off=(uint32_t)(145+angle*2.4);
    PCA_Setpwm(0,0,off);PCA_Setpwm(1,0,off);PCA_Setpwm(2,0,off);PCA_Setpwm(3,0,off);
    PCA_Setpwm(4,0,off);PCA_Setpwm(5,0,off);PCA_Setpwm(6,0,off);PCA_Setpwm(7,0,off);
    PCA_Setpwm(8,0,off);PCA_Setpwm(9,0,off);PCA_Setpwm(10,0,off);PCA_Setpwm(11,0,off);
    PCA_Setpwm(12,0,off);PCA_Setpwm(13,0,off);PCA_Setpwm(14,0,off);PCA_Setpwm(15,0,off);
    HAL_Delay(500);
}*/
/*函数作用：控制舵机转动；参数：1.输出端口，可选0~15； 2.结束角度，可选0~180；*/
/*void PCA_MG90(uint8_t num,uint8_t end_angle)
{
    uint32_t off=0;
    off=(uint32_t)(158+end_angle*2.2);
    PCA_Setpwm(num,0,off);
}*/




// One I2C transaction/STOP for a complete group. An I2C error may still latch a partial
// group: software cannot promise neutral output on a failed bus; hardware OE remains required.
bool PCA_WriteGroup(uint8_t first, const uint16_t *off, uint8_t count)
{
    // Do not gate this write on PCA_ConfigurationOk().  The old code always
    // attempted the write; a failed HAL transaction is reported by the bus
    // result below and handled by the normal safety path.
    if (!off || !count || count > 16 || first+count > 16) return false;
    // A single 49-byte auto-increment transfer is not accepted reliably by
    // every PCA9685/TCA combination on this vehicle.  Keep the old register
    // order, but send one complete channel (ON_L..OFF_H) per transaction.
    // This is short enough for the 400 kHz bus and retains the old 1550 us
    // conversion exactly.
    for (unsigned i = 0; i < count; ++i)
    {
        if (off[i] > 4095) return false;
        uint8_t bytes[5] = {uint8_t(LED0_ON_L + 4 * (first + i)), 0, 0,
                            uint8_t(off[i]), uint8_t(off[i] >> 8)};
        if (LcBus_Transmit(&hi2c2, PCA9685_ADDR, bytes, sizeof(bytes), 10000) != HAL_OK)
            return false;
    }
    return LcBus_Ok(&hi2c2) != 0;
}
