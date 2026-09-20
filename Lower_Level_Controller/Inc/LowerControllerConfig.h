/*
 * Active V33 configuration for the FreeRTOS controller.
 * 当前 V33 配置：DRDY 异步采集、ESKF 姿态/深度反馈、150 Hz SI 控制。
 * See docs/CONFIG_AND_INTERFACES.md before changing a rate, unit or mapping.
 * 修改频率、单位或编号前，请同步核对配置与接口文档。
 */
#ifndef LOWER_CONTROLLER_CONFIG_H
#define LOWER_CONTROLLER_CONFIG_H

#ifndef LC_USE_FREERTOS
#define LC_USE_FREERTOS              1
#endif
// 本固件固定使用 DRDY + ESKF，不能关闭异步采集来退回旧控制。
#ifndef LC_IMU_ASYNC_ENABLED
#define LC_IMU_ASYNC_ENABLED LC_USE_FREERTOS
#endif
#if LC_USE_FREERTOS && !LC_IMU_ASYNC_ENABLED
#error "This firmware requires DRDY acquisition and the ESKF controller."
#endif
#define LC_RTOS_TICK_HZ              1000U
#define LC_IMU_DMA_TIMEOUT_MS        4U
#define LC_IMU_RESULT_TIMEOUT_MS     8U
#define LC_I2C_TIMEOUT_MS            2U
#define LC_BUS_BUDGET_US             5000U
#define LC_CALIBRATION_BUDGET_US     8000000U
#define LC_STATE_MAX_AGE_US          20000U
#define LC_UART_TX_TIMEOUT_MS        30U
#define LC_UART_RX_QUEUE_LENGTH      8U
#define LC_UART_TX_QUEUE_LENGTH      16U
#define LC_UART_TX_PACKET_SIZE       192U // 一整行 VOFA 数据原子入队，避免分段日志交错。
#define LC_VOFA_PERIOD_US            100000U // 默认 10 Hz；只影响显示，不改变控制频率。

// 上位机每 100 ms 发 HB；500 ms 未收到新心跳则停止，恢复心跳不会自动启动。
#define LC_COMMAND_TIMEOUT_US       500000U
#define LC_TASK_STALL_TIMEOUT_US    100000U
#define LC_BUS_REPLY_TIMEOUT_MS     10U
#define LC_STARTUP_PRESSURE_SAMPLES 100U
#define LC_STARTUP_PRESSURE_SETTLE_MS 1000U
#define LC_STARTUP_PRESSURE_SPREAD_PA 200.0f // TODO：实测静止噪声后调整；约 2 cm 水头。
#define LC_IWDG_ENABLED             1
// TODO(HARDWARE)：Making_Instructions 接线图未连接 OE，也未指定急停 GPIO。
// 原装配接线只能用 I2C 写中位；总线失效时须切断推进器电源。
// OE 需另接上拉和可用 GPIO，并验证电调丢失 PWM 后停止；原资料不能替代此实测。
// 接线后设为 1，并定义 LC_PCA_OE_GPIO_PORT / LC_PCA_OE_GPIO_PIN；高电平禁止输出。
#ifndef LC_PCA_OE_ENABLED
#define LC_PCA_OE_ENABLED           0
#endif
#if LC_PCA_OE_ENABLED && (!defined(LC_PCA_OE_GPIO_PORT) || !defined(LC_PCA_OE_GPIO_PIN))
#error "Define the wired PCA9685 OE GPIO port/pin before enabling the hardware output gate."
#endif

/* Clock tree: HSE -> PLL -> 168 MHz HCLK; APB1 /4, APB2 /2.
 * 时钟树：12 MHz 外部晶振，经 PLL 得到 168 MHz；TIM1 输入为 168 MHz。 */
#define LC_HSE_HZ                    12000000U
#define LC_PLL_M                     6U
#define LC_PLL_N                     168U
#define LC_PLL_P                     2U
#define LC_PLL_Q                     7U
#define LC_HCLK_HZ                   ((LC_HSE_HZ / LC_PLL_M) * LC_PLL_N / LC_PLL_P)
#define LC_APB1_DIVIDER              4U
#define LC_APB2_DIVIDER              2U
#define LC_TIM1_INPUT_HZ             ((LC_HCLK_HZ / LC_APB2_DIVIDER) * 2U)
#define LC_TIM1_PRESCALER            (1120U - 1U)
#define LC_TIM1_PERIOD               (1000U - 1U)
#define LC_CONTROL_HZ                (LC_TIM1_INPUT_HZ / (LC_TIM1_PRESCALER + 1U) / (LC_TIM1_PERIOD + 1U))
#define LC_MAHONY_SAMPLE_HZ          150.0f
#define LC_PRESSURE_CYCLE_STEPS      3U
#define LC_PRESSURE_FRAME_HZ         (LC_CONTROL_HZ / LC_PRESSURE_CYCLE_STEPS)

/* These ODRs describe the existing BMI088 register table, not the 150 Hz read loop.
 * 默认 DRDY 按各自 ODR 发起采集；TIM1 仅保持 150 Hz 控制及温控请求。 */
#define LC_GYRO_ODR_HZ               1000U
#define LC_ACCEL_ODR_HZ              800U
/* Compatibility read rate only; default acquisition uses fresh IST8310 DRDY events.
 * 下方 150 Hz 仅属兼容路径；默认磁力计按 DRDY 和状态位读取，未声称其内部 ODR。 */
#define LC_MAG_READ_NOMINAL_HZ       LC_CONTROL_HZ
#define LC_PRESSURE_OSR              1024U
#define LC_PRESSURE_CONVERSION_US    3000U
#define LC_I2C_CLOCK_HZ              400000U
#define LC_COMMAND_UART_BAUD         115200U
#define LC_FIRMWARE_ID              "ESKF_VOFA_20260916_R2"

/* Existing output calibration; PWM pulse widths are in microseconds.
 * 输出沿用已有标定；脉宽单位为微秒，1610 是当前版本的初始化/停止值。 */
// 按用户选择沿用原版推进器/电调及其供电方案；额定电压不属于固件配置参数。
// 资料确认：TCA0..3 接四个 MS5837-30BA；TCA4 接 PCA；PCA0..7 推进器、8..11 舵机。
#define LC_ROBOT_VERSION             V33
#define LC_PRESSURE_COUNT            4
#define LC_THRUSTER_COUNT            8
#define LC_SERVO_COUNT               4
#define LC_PWM_MUX_CHANNEL           4
#define LC_PWM_FREQUENCY_HZ          50
#define LC_PWM_PERIOD_US             20000
#define LC_PWM_COUNTS                4096
#define LC_THRUSTER_NEUTRAL_US        1610
#define LC_THRUSTER_MIN_US            1000
#define LC_THRUSTER_MAX_US            2000
#define LC_THRUSTER_DEADZONE_US       50
// V3.3设计.pdf p9 的原机实测死区；1610 中位保留，ESKF 按上下边界分别补偿。
// LC_THRUSTER_DEADZONE_US 仅保留给旧参考实现；当前闭环使用以下两项。
#define LC_THRUSTER_DEADZONE_LOW_US   1570
#define LC_THRUSTER_DEADZONE_HIGH_US  1670
// TODO(THRUSTER_DEADZONE)：原机记录并非逐台电调标定，入水低推力测试仍需核对。
#define LC_SERVO_MIN_US               500
#define LC_SERVO_MAX_US               2500

/* Group order: front-left, rear-left, front-right, rear-right.
 * 组内顺序：前左、后左、前右、后右；Sign 按 PCA 物理通道索引。 */
// 原 Propeller.cpp 的通道映射已与 V3.3设计.pdf p7 一致：推进器 1..8 对应 PCA0..7。
// zh-CN/外形.STEP 已定位八个推进器；垂直轴间距为前后 108、左右 214.01532 mm。
// 水平轴与纵向 45 度，接口坐标及舵机轴位置见 docs/CAD_GEOMETRY_20260916.md。
// STEP 确认几何位置；通道、正反桨与 1610 us 中位继续按原版采用。
// PCA0..7: 前左水平、前左垂直、后左垂直、后左水平、后右水平、后右垂直、前右垂直、前右水平。
// PCA8..11: 左上(横轴)、左下(竖轴)、右上(横轴)、右下(竖轴)；V3.3设计p7编号1..4。
// 口号指外接PCA9685的0起始通道，不是C板自身的PWM接口。接线表见docs/PORT_MAP_V33.md。
#define LC_VERTICAL_CHANNELS_INIT    {1, 2, 6, 5}
#define LC_HORIZONTAL_CHANNELS_INIT  {0, 3, 7, 4}
#define LC_THRUSTER_SIGNS_INIT        {1, -1, 1, 1, -1, -1, 1, -1}
#define LC_SERVO_V33_CHANNELS_INIT    {8, 9, 10, 11}

/* Original hardware timing remains in use; active SI control uses elapsed seconds.
 * 保留原硬件频率；当前 SI 控制的积分/微分按实际秒数计算。 */
#if LC_HCLK_HZ != 168000000U || LC_CONTROL_HZ != 150U || LC_PRESSURE_FRAME_HZ != 50U
#error "This hardware profile requires 168 MHz / 150 Hz / 50 Hz."
#endif
#if LC_PLL_P != 2U || LC_APB1_DIVIDER != 4U || LC_APB2_DIVIDER != 2U
#error "Update the RCC divider mapping together with the clock contract."
#endif
#if LC_PWM_FREQUENCY_HZ * LC_PWM_PERIOD_US != 1000000
#error "PWM frequency and pulse-width conversion period must agree."
#endif
#if LC_THRUSTER_DEADZONE_LOW_US >= LC_THRUSTER_NEUTRAL_US || LC_THRUSTER_NEUTRAL_US >= LC_THRUSTER_DEADZONE_HIGH_US
#error "The stop pulse must lie inside the documented thruster dead zone."
#endif
#if LC_PRESSURE_COUNT != 4 || LC_THRUSTER_COUNT != 8 || LC_SERVO_COUNT != 4
#error "The legacy geometry and mixers require four pressures, eight thrusters and four servos."
#endif
#ifdef __cplusplus
static_assert(LC_MAHONY_SAMPLE_HZ == static_cast<float>(LC_CONTROL_HZ),
              "Mahony's fixed time step must match the nominal scheduler rate.");
#endif

#endif
