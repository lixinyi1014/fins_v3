//
// Created by LEGION on 2021/10/4.
//
//This file contains the main function and the interrupt service routines (ISRs).
//The usage of each module follows three steps: initialization, data acquisition, and execution.
//Initialization runs in main; TIM1 releases control and temperature requests; DRDY drives IMU sampling.
//UART interrupts copy received packets; the control task parses commands.
//
//此文件为主函数和中断服务函数所在地
//每个元件的使用包含三个步骤，初始化、获取数据、执行
//初始化在 main 中完成；TIM1 释放控制与温控请求，DRDY 驱动 IMU 采集。
//串口中断复制数据入队，控制任务解析命令。
#include "Usermain.h"
#include "I2cBusAccess.h"
#include "IMU.h"
#include "Propeller.h"
#include "Servo.h"
#include "Sensor.h"
#include "Legacy.h"
#include "Watchdog.h"
#include "LED.h"
#include "Buzzer.h"
#include "gpio.h"
#include "UART_Base.h"
#include "ControllerTasks.h"


//Adjust according to the vehicle version: V30, V31, V32, V33, V40.
//根据潜器版本调整，V30，V31，V32，V33，V40
VERSION_E Robot_Version = LC_ROBOT_VERSION; // V33


// Statically instantiated objects.
// 静态实例化对象
// IMU imu;
static Servo servo;
static Servo_I2C servo_i2c;
static Propeller_I2C propeller_i2c;
static Sonar sonar;
static Watchdog watchdog;
static LED led;
static Buzzer buzzer;


// Array of device pointers for unified management and invocation of device operations.
// V3.3 production device list: PCA thrusters/servos, onboard IMU, four pressures and LED.
// V3.3设备列表已选定：扩展板推进器/舵机、板载IMU、四路水压及LED。
static Device *device[] = {
        &propeller_i2c,                    //Propeller（PWM control via the PWM expansion board） 推进器（PWM扩展板控制）
        //&propeller,                      //Propeller（PWM control via the Robomaster_C board） 推进器（C板PWM控制）
				&servo_i2c,                        //Servo（PWM control via the PWM expansion board） 舵机（PWM扩展板控制）
        &IMU::imu,
        &PressureSensor::pressure_sensor, // 先将 PWM 写为中位，再进行耗时的传感器初始化。
				//&servo,                          //Servo（PWM control via the Robomaster_C board） 舵机（C板PWM控制）
        //&watchdog,                       //Watchdog 窗口看门狗
        &led,                              //LED
        //&buzzer                          //Buzzer 蜂鸣器
};

// Macro definition for the number of devices.
// 定义设备数量宏
#define DEVICE_NUM (sizeof(device) / sizeof(device[0]))

volatile bool system_init_flag = false;
uint8_t RxBuffer[SERIAL_LENGTH_MAX] = {0};
volatile uint8_t key_raw_state = 1;
uint32_t key_last_stamp;
volatile int32_t time_start = 0, time_end = 0, time_interval = 0, cnt = 0;

// Timer interrupt service routine (ISR) for periodic device processing.
// 定时器中断服务函数, 用于周期性处理设备
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim7) { HAL_IncTick(); return; } // TIM7：1 ms HAL 时间基准
    if (system_init_flag && htim == &htim1) LcRuntime_ControlTickFromISR(); // TIM1：150 Hz
}

// UART receive interrupt service routine (ISR) for handling device-specific UART reception tasks.
// 串口接收中断服务函数, 用于处理设备的串口接收任务
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)
{
    if (huart == &huart6) LcRuntime_UartRxFromISR(datasize); // datasize 单位为字节；中断复制入队，控制任务解析
}

// UART transmit interrupt callback function.
// 串口发送中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) { LcRuntime_UartTxFromISR(); return; } // 通知值 0x01：UART6 当前包发送完成
    #define HANDLE_UART_TX(ID)                            \
        do {                                              \
            if (huart == uartHandleList[ID]) {            \
                UARTBaseLite<ID>::GetInstance().OnTxDone(); \
                return;                                   \
            }                                             \
        } while (0)

    HANDLE_UART_TX(1);


    #undef HANDLE_UART_TX
    // if(huart == &huart1) {
    //     UARTBaseLite<1>::GetInstance().OnTxDone();
    // }
    // else if(huart == &huart6){
    //     UARTBaseLite<6>::GetInstance().OnTxDone();
    // }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) LcRuntime_UartErrorFromISR(); // 重启接收并通知错误位 0x02；这里不解析命令
}

// 默认 GPIO 回调位于 tasks/Src/ImuSampleInterrupts.cpp；PC4/PC5/PG3 对应三类 DRDY。
// PA0 按键当前没有动作，不应在此另定义第二个 HAL_GPIO_EXTI_Callback。

/**
  * @brief  The application entry point.
  * @retval int
  */
// Entry point of the main function.
// 主函数入口
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    const bool watchdog_recovery = LcSafetyHardware_WatchdogReset() != 0;
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    //MX_IWDG_Init();
    MX_USB_DEVICE_Init();

    LcTime_Start(); // Before every sensor init and microsecond delay.
    const char banner[] = "BOOT=V33 " LC_FIRMWARE_ID " UART6_115200_8E1\r\n";
    LcSerial_Write(reinterpret_cast<const uint8_t *>(banner), sizeof(banner)-1);

    // 当前 LC_IWDG_ENABLED=0，与原工程一致不启用看门狗。
    // 因此不能因上一次固件留下的 RCC IWDGRST 标志而进入永久恢复锁。
#if LC_IWDG_ENABLED
    if (watchdog_recovery)
    {
        // IWDG 复位后仍在计时。先尝试中位，再停留于恢复锁定，避免传感器长初始化造成复位循环。
        LcSafetyHardware_Start();
        LcBus_Begin(&hi2c2, 100000U);
        propeller_i2c.Init();
        const char locked[] = "CAL=LOCK_WATCHDOG_RESET POWER_CYCLE_AT_SURFACE\r\n";
        uint32_t last_notice = HAL_GetTick() - 1000U;
        for (;;)
        {
            LcSafetyHardware_SetOutputEnabled(0);
            LcSafetyHardware_Feed();
            if (HAL_GetTick() - last_notice >= 1000U)
            {
                last_notice = HAL_GetTick();
                LcSerial_Write(reinterpret_cast<const uint8_t *>(locked), sizeof(locked)-1);
            }
            HAL_Delay(20);
        }
    }
#else
    (void)watchdog_recovery; // 仅保留复位原因读取接口；禁用 IWDG 时不阻断正常启动。
#endif

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    //RemoteControl::init();
    // Legacy Flash layout is not used by ESKF; do not read unversioned calibration into RAM.
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

		// Initialize all devices.
    // 初始化所有设备
    const char *const stage[] = {"THRUSTERS", "SERVOS", "IMU", "PRESSURE", "LED"};
    static_assert(sizeof(stage)/sizeof(stage[0]) == DEVICE_NUM, "Update boot stage names with device[]");
    for(int i = 0; i < DEVICE_NUM; ++i){
        char text[192];
        int length = snprintf(text, sizeof(text), "BOOT=INIT_%s\r\n", stage[i]);
        LcSerial_Write(reinterpret_cast<const uint8_t *>(text), length);
        LcBus_Begin(&hi2c2, 3000000U);
        LcBus_Begin(&hi2c3, 3000000U);
        device[i] -> Init();
        const uint32_t bus2_errors = LcBus_ErrorCount(&hi2c2);
        const uint32_t bus3_errors = LcBus_ErrorCount(&hi2c3);
        const uint32_t pca_error = PCA_ConfigurationErrorCode();
        // Match the legacy startup contract: PCA has no register-readback gate;
        // the pressure device is accepted when its PROM CRCs are valid.  Bus and
        // PCA details remain in the diagnostic line but must not block reaching
        // the legacy MS5837 read path.
        const bool init_ok = (i != 2 || IMU::imu.InitError() == 0) &&
            (i != 3 || (LcBus_Ok(&hi2c2) && PressureSensor::pressure_sensor.PromValid()));
        if (!init_ok) {
            // No scheduler/arming after failed initialization. Repeat both human-readable
            // and FireWater-compatible numeric diagnostics so the failing layer is visible.
            __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 0);
            LcSafetyHardware_SetOutputEnabled(0);
            for (;;) {
                length = snprintf(text, sizeof(text),
                                  "BOOT=FAIL_%s ERR=%lu I2C2E=%lu I2C3E=%lu PCAE=%lu M1=%u M2=%u PRE=%u/%u IMU=%u RESTART_AT_SURFACE\r\n",
                                  stage[i], static_cast<unsigned long>(
                                      (i == 0 || i == 1) ? pca_error : 0U),
                                  static_cast<unsigned long>(bus2_errors),
                                  static_cast<unsigned long>(bus3_errors),
                                  static_cast<unsigned long>(pca_error),
                                  unsigned(PCA_LastMode1()), unsigned(PCA_LastMode2()),
                                  unsigned(PCA_LastPrescale()), unsigned(PCA_ExpectedPrescale()),
                                  unsigned(IMU::imu.InitError()));
                if (length > 0 && unsigned(length) < sizeof(text))
                    LcSerial_Write(reinterpret_cast<const uint8_t *>(text), length);
                char vofa_error[192];
                const int vofa_length = snprintf(vofa_error, sizeof(vofa_error),
                    "vofa:-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,1,0,0,%lu,%lu,%lu,%lu,%u\r\n",
                    static_cast<unsigned long>(pca_error),
                    static_cast<unsigned long>(bus2_errors),
                    static_cast<unsigned long>(bus3_errors),
                    static_cast<unsigned long>(pca_error), unsigned(i));
                if (vofa_length > 0 && unsigned(vofa_length) < sizeof(vofa_error))
                    LcSerial_Write(reinterpret_cast<const uint8_t *>(vofa_error), vofa_length);
                HAL_Delay(1000);
            }
        }
        length = snprintf(text, sizeof(text), "BOOT=OK_%s\r\n", stage[i]);
        LcSerial_Write(reinterpret_cast<const uint8_t *>(text), length);
    }

    // TODO(操作)：四个压力口置于空气中静止上电；不能在水下重启来重新定义水面。
    const char begin[] = "CAL=START KEEP_PRESSURE_PORTS_IN_AIR\r\n";
    LcSerial_Write(reinterpret_cast<const uint8_t *>(begin), sizeof(begin)-1);
    const bool calibration_ok = PressureSensor::pressure_sensor.CalibrateAtStartup();
    const char *calibration_message = calibration_ok
        ? "CAL=OK\r\n" : "CAL=FAIL RESTART_AT_SURFACE\r\n";
    LcSerial_Write(reinterpret_cast<const uint8_t *>(calibration_message), strlen(calibration_message));
    const auto &cal = PressureSensor::pressure_sensor.startup_calibration;
    char calibration_detail[96];
    const int detail_length = snprintf(calibration_detail, sizeof(calibration_detail),
        "CAL=DETAIL state=%lu channel=%lu samples=%lu\r\n",
        static_cast<unsigned long>(cal.state), static_cast<unsigned long>(cal.failed_channel),
        static_cast<unsigned long>(cal.samples));
    LcSerial_Write(reinterpret_cast<const uint8_t *>(calibration_detail), detail_length);
    if (calibration_ok)
    {
        // 只有所有设备初始化及水面标定都成功，才提示“启动完成”。
        buzzer.Init();
        buzzer.StartupBeep();
        const char beep_message[] = "BOOT=BEEP_OK\r\n";
        LcSerial_Write(reinterpret_cast<const uint8_t *>(beep_message), sizeof(beep_message)-1);
    }
    const char controller_message[] = "CTRL=ESKF STOPPED VOFA_FIREWATER_16CH\r\n";
    LcSerial_Write(reinterpret_cast<const uint8_t *>(controller_message), sizeof(controller_message)-1);
		
		// Set the initialization-complete flag.
    // 设置初始化完成标志
    system_init_flag = true;
    time_start = HAL_GetTick();

    StartControllerTasks(device, DEVICE_NUM, &propeller_i2c, &servo_i2c, &led); // 当前 5 个 Device；创建 IMU、控制、I2C2、UART TX 四任务后启动调度

}

void send_float(float value, uint8_t decimalPlaces, bool endSign){
    bool isNegative = 0;
    uint8_t len = 0;
    char buffer_float[20] = {0};
		
		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_float[0] = '-';
        len = 1;
    }

		// Convert the integer part.
    // 转换整数部分
    int32_t intPart = (int)value;
    len += sprintf(buffer_float+len, "%d", intPart);
		
		// Process the fractional part.
    // 处理小数部分
    if (decimalPlaces > 0) {
        buffer_float[len++] = '.';

        float fraction = value - (float)intPart;
        for (uint8_t i = 0; i < decimalPlaces; i++) {
            fraction *= 10;
            int digit = (int)fraction;
            buffer_float[len++] = digit + '0';
            fraction -= digit;
        }
    }

		// If this is the final element, append a newline; otherwise, append a comma.
    // 若结束则换行，否则是逗号
    if(endSign)
        buffer_float[len++] = '\n'; // 行结束符；下方单独写 '\0' 作为 C 字符串终止符
    else
        buffer_float[len++] = ',';

    buffer_float[len] = '\0';

    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    LcSerial_Write(reinterpret_cast<uint8_t *>(buffer_float), len); // len 为有效字节数；运行期复制入队，函数返回后局部缓存可复用
}

void send_int(int32_t value, bool endSign){
    char buffer_int[20] = {0};
    uint8_t len = 0;
    bool isNegative = 0;

		// Handle negative values.
    // 处理负数
    if(value < 0){
        isNegative = 1;
        value = -value;
    }
    if(isNegative) {
        buffer_int[0] = '-';
        len = 1;
    }

    len += sprintf(buffer_int+len, "%d", value);

    if(endSign)
        buffer_int[len++] = '\n';
    else
        buffer_int[len++] = ',';

    buffer_int[len] = '\0';
    // HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, 0x00ff);
    LcSerial_Write(reinterpret_cast<uint8_t *>(buffer_int), len); // len 为有效字节数；运行期复制入队，函数返回后局部缓存可复用
}
