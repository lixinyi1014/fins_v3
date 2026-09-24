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
#define LC_BUS_BUDGET_US             20000U
#define LC_CALIBRATION_BUDGET_US     8000000U
#define LC_STATE_MAX_AGE_US          20000U
#define LC_UART_TX_TIMEOUT_MS        30U
#define LC_UART_RX_QUEUE_LENGTH      8U
#define LC_UART_TX_QUEUE_LENGTH      16U
#define LC_UART_TX_PACKET_SIZE       192U // 一整行 VOFA 数据原子入队，避免分段日志交错。
#define LC_VOFA_PERIOD_US            20000U // 旧版水压完成一帧约 50 Hz；只影响显示，不改变 150 Hz 调度。

// 上位机每 100 ms 发 HB；500 ms 未收到新心跳则停止，恢复心跳不会自动启动。
#define LC_COMMAND_TIMEOUT_US       500000U
#define LC_TASK_STALL_TIMEOUT_US    100000U
// 控制任务等待 I2C2 总线任务回复的上限；超时即 FatalStop(StopBusReplyTimeout)，永久锁存。
// 必须覆盖“整组事务预算 + 预算检查点之后仍可能开始的一笔 HAL 调用(I2cBusAccess 上限 10 ms)”。
// 曾被改为 10 ms：只要一笔 I2C 事务出错卡满 10 ms 就被判致命，水压停采、四路显示 -9999。
#define LC_BUS_REPLY_TIMEOUT_MS     40U
#define LC_I2C_HAL_CALL_MAX_MS      10U // 与 I2cBusAccess.cpp kLongTransactionTimeoutMs 保持一致
// 单笔 HAL 超时下限。MS5837 读取原先只给 1 ms：总线任务被高优先级 IMU 任务抢占 >1 ms 时，
// HAL 会在字节传输中途放弃，从机可能一直拉低 SDA，导致 I2C2 永久 BUSY。正常传输远达不到此超时。
#define LC_I2C_HAL_CALL_MIN_MS      5U
#if LC_BUS_REPLY_TIMEOUT_MS * 1000U < LC_BUS_BUDGET_US + LC_I2C_HAL_CALL_MAX_MS * 1000U + 5000U
#error "LC_BUS_REPLY_TIMEOUT_MS must cover the I2C2 bus budget plus one HAL call and margin."
#endif
#define LC_STARTUP_PRESSURE_SAMPLES 100U
#define LC_STARTUP_PRESSURE_SETTLE_MS 1000U
#define LC_STARTUP_PRESSURE_SPREAD_PA 200.0f // TODO：实测静止噪声后调整；约 2 cm 水头。
// 上电标定必须在空气中完成：空气里四路之间没有水柱差，读数应当彼此接近。
// 1500 Pa 约 15 cm 水头，宽到容得下 MS5837 各片的出厂偏差，窄到能识破“在水里标零点”。
#define LC_STARTUP_PRESSURE_CHANNEL_SPREAD_PA 1500.0f
// FusedStateUsable 判定“反馈还在本级控制工作区内”的倾角上限（度）。0 = 不限制。
// 原值硬编码 45°：超过就 READY 灭、控制环以 StopFeedback 停机，也就是潜器一旦
// 倾过头，推进器恰好在最需要它扶正的时候被切掉，只能靠人捞。对一台会大角度
// 机动的潜器，这个失效模式比“超出线性化工作区”本身更危险，所以默认关掉。
// 关掉的代价：任何姿态下控制器都会继续驱动推进器，包括翻过来的时候；
// 小倾角线性化在大角度下只是变钝，不会发散，但也谈不上还在设计工作区内。
// 想恢复原行为改成 45.0f 即可，判据本身仍然在。
#define LC_CONTROL_TILT_LIMIT_DEG 0.0f
// 深度闭环开关的上电默认值。1=保持深度，0=只稳姿态、深度交给浮力配平。
// 运行期可用 DEP:ON / DEP:OFF 切换。
#define LC_DEPTH_HOLD_DEFAULT 1

/* ---- 姿态整定（现场调这三组就够） ------------------------------------
 * 姿态角度外环比例增益，单位 (rad/s)/rad。
 *
 * 原代码把旧 PID 增益换算到弧度制时用了 units_per_m = rho*g/2000 = 4.905，
 * 但 legacy 压力单位是 hPa，1 m 水 = 98.1 hPa，正确系数是 rho*g/100。
 * 用实测数据核对过三次（ESKF 深度 vs 四路均值 95.9；IMU 俯仰 vs 前后压差
 * 107.8；IMU 横滚 vs 左右压差 119），都在 96~119，不可能是 4.905。
 * 结果是姿态外环增益小了 20 倍：20 度俯仰偏差只输出 11.9 us，而死区补偿
 * 本身就要 50~60 us，浮力配平预置用 90~100 us —— 等于没有权限。
 *
 * 旧增益按正确标度换算过来的等效值：横滚 0.5*0.2764m*98.1 = 13.6，
 * 俯仰 1.0*0.4988m*98.1 = 48.9。这只是参照，不是上限。
 *
 * 结构上限：角度外环输出限幅 10 rad/s，速率内环 LC_ATTITUDE_RATE_KP = 8
 * us/(rad/s)，相乘得**单通道最大权限 80 us，kp 再大也突破不了**。
 * 未饱和时出力 = 8 * kp * 偏差(rad) = 0.1396 * kp us/度，
 * 所以 kp 实际决定的是"多大偏差达到满权限"：
 *     kp=12 -> 47.7 度    kp=24 -> 23.9 度
 *     kp=36 -> 15.9 度    kp=48.9 -> 11.7 度（接近开关式控制）
 *
 * 2026-09-23 实测确定 IMU 装反并修正之后，两个通道都取 36：
 * 约 16 度就给满权限，横滚力臂只有俯仰的 55%（0.0691 vs 0.1247 m），
 * 同样 80 us 产生的力矩更小，所以不按旧版那样让 roll 只有 pitch 的一半。
 * 振荡就对半砍；若满权限仍然不够，kp 不是正确的旋钮，改 LC_ATTITUDE_RATE_KP。 */
/* IMU 整板装机朝向。0 = 原约定，1 = 绕板法线再转 180 度。
 *
 * docs/FINAL_INTEGRATION_20260916.md 第 58 行：官方孔位存在两个相差 180 度的
 * 正确旋转，四孔残差都小于 1e-5 mm，"孔位匹配不能唯一决定插座朝前还是朝后"。
 * 当时按旧约定取了其中一个，并留了 TODO(IMU_INSTALLATION)。
 *
 * 这两个朝向相差一个绕法线的 180 度，效果是**横滚和俯仰同时反号**，
 * 而竖直方向（z 轴对角元两种取法都是 -1）不受影响 ——
 * 也就是说改这个开关不会动"哪边是下"，只会动前后和左右。
 *
 * 判定办法（空气中，不解锁，压力阵列不参与）：把机头抬起约 30 度看 HUD 俯仰，
 * FRD 约定下抬头应当是正值；读到负值就把这里改成 1。 */
#ifndef LC_IMU_BOARD_YAW_180
#define LC_IMU_BOARD_YAW_180 1   // 2026-09-23 实测确定：空气中抬头/右舷下都读负，原约定反了
#endif

#ifndef LC_ROLL_ANGLE_KP
#define LC_ROLL_ANGLE_KP 36.0f
#endif
#ifndef LC_PITCH_ANGLE_KP
#define LC_PITCH_ANGLE_KP 48.0f
#endif
/* 姿态速率内环比例增益，单位 us/(rad/s)，横滚与俯仰分开。
 * 它和角度外环的输出限幅（10 rad/s）共同决定该通道的最大权限：
 *     最大权限 = 本增益 * 10 us
 * 旧版两轴都是 8 -> 80 us。**已经打满还扶不动时该加的是这个数，不是 kp**：
 * 饱和之后 kp 再大也一点力都多不出来，只会让它更早进饱和。
 *
 * 2026-09-23：横滚在 kp=36 / 速率 8 下表现良好，不动；俯仰因为机体头重脚轻
 * 有一股常驻低头力矩，长期顶在 80 us 上，所以单独放到 12 -> 120 us。
 * 上限受推进器 1000..2000 us 与死区补偿约束。 */
#ifndef LC_ROLL_RATE_KP
#define LC_ROLL_RATE_KP 8.0f
#endif
#ifndef LC_PITCH_RATE_KP
#define LC_PITCH_RATE_KP 12.0f
#endif

/* 偏航闭环增益。偏航反馈本来就是弧度，不涉及 units_per_m 那个换算问题，
 * 但原值极弱：kp=2、速率 kp=5，每度只有 0.175 us，90 度偏差也才 15.7 us，
 * 而手动转向的开环预置就是 40 us —— 等于拉不住平移带来的偏航。
 * 现值：每度 2.79 us，满权限 160 us，约 57 度进饱和。
 * 注意 ACL 打开后 FusedStateUsable 会把"航向已观测"列为必需项。 */
#ifndef LC_YAW_ANGLE_KP
#define LC_YAW_ANGLE_KP 20.0f
#endif
#ifndef LC_YAW_RATE_KP
#define LC_YAW_RATE_KP 8.0f
#endif

/* 姿态配平前馈，单位 us（推进器脉宽当量），直接叠加在对应通道的控制量上。
 * 符号按分配矩阵注释：正俯仰 = 前上后下，正横滚 = 左上右下。
 * 机体头重脚轻（低头）时给 LC_PITCH_TRIM_US 正值抬头。
 * 0 = 不配平。整定办法见下：从 0 开始每次加 10，直到松手能大致保持水平。 */
#ifndef LC_PITCH_TRIM_US
#define LC_PITCH_TRIM_US 60.0f
#endif
#ifndef LC_ROLL_TRIM_US
#define LC_ROLL_TRIM_US 0.0f
#endif
// 源程序的 legacy 接口继续返回原有数值；物理压力接口直接复用旧工程
// CompensateMs5837() 的 Pa 结果，避免在驱动层重复解释 D1/D2 或改变单位。
// 与原始工程保持一致：原工程的 MX_IWDG_Init() 和 watchdog 设备均未加入运行路径。
// 当前先关闭 IWDG，避免旧复位标志或一次总线超时把调试板锁在恢复循环中。
// 重新做水下安全测试前，再单独评估并启用看门狗及独立动力急停。
#define LC_IWDG_ENABLED             0
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
 * 输出沿用当前实物标定；脉宽单位为微秒，1550 是当前版本的初始化/停止值。 */
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
#define LC_THRUSTER_NEUTRAL_US        1550
#define LC_THRUSTER_MIN_US            1000
#define LC_THRUSTER_MAX_US            2000
#define LC_THRUSTER_DEADZONE_US       50
// V3.3 实物标定：1550 us 为中位；按原死区相对中位的 -40/+60 us 保留为 1510..1610。
// LC_THRUSTER_DEADZONE_US 仅保留给旧参考实现；当前闭环使用以下两项。
#define LC_THRUSTER_DEADZONE_LOW_US   1510
#define LC_THRUSTER_DEADZONE_HIGH_US  1610
// TODO(THRUSTER_DEADZONE)：原机记录并非逐台电调标定，入水低推力测试仍需核对。
#define LC_SERVO_MIN_US               500
#define LC_SERVO_MAX_US               2500

/* Group order: front-left, rear-left, front-right, rear-right.
 * 组内顺序：前左、后左、前右、后右；Sign 按 PCA 物理通道索引。 */
// 原 Propeller.cpp 的通道映射已与 V3.3设计.pdf p7 一致：推进器 1..8 对应 PCA0..7。
// zh-CN/外形.STEP 已定位八个推进器；垂直轴间距为前后 108、左右 214.01532 mm。
// 水平轴与纵向 45 度，接口坐标及舵机轴位置见 docs/CAD_GEOMETRY_20260916.md。
// STEP 确认几何位置；通道、正反桨与 1550 us 中位按当前实物标定采用。
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
