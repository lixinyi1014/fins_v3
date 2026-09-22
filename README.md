# FinsROV V3.3 下位机固件

FinsROV V3.3 水下机器人的下位机控制器，运行在 RoboMaster C 板（STM32F407IG）上。
FreeRTOS 多任务架构，用 ESKF 融合 IMU、磁力计和四路水压计，估计姿态与深度，再由 SI 单位的 PID 闭环控制 8 路推进器和 4 路舵机。

- **主控**：RoboMaster C 板，STM32F407IG，168 MHz
- **传感器**：板载 BMI088（陀螺 1000 Hz / 加计 800 Hz）、IST8310 磁力计；4 × MS5837-30BA 水压计（经 TCA9548A 选通）
- **输出**：PCA9685 PWM 扩展板，8 路推进器 + 4 路舵机
- **上位机通信**：UART6（板上丝印 UART1），115200、8E1

---

## 当前状态（2026-09-23）

| 项目 | 状态 |
|---|---|
| 空气中上电、水压标定 | ✅ 已上机验证 |
| ESKF 姿态 / 深度输出，READY=1 | ✅ 已上机验证 |
| 连续运行约 10 分钟 | ✅ 正常，I2C 偶发错误可自动恢复 |
| 解锁后推进器闭环运行 | ⚠️ 未验证 |
| 水下运行、水压差姿态修正 | ⚠️ 未验证 |

已知问题见文末[已知问题](#已知问题)。

---

## 目录结构

```
Lower_Level_Controller/
├── MDK-ARM/RM_Frame_C.uvprojx   ← 主 Keil 工程（用这个）
├── CubeMX/                      CubeMX 生成的备份工程，平时不用
├── Inc/ Src/                    CubeMX 外设初始化；Inc/LowerControllerConfig.h 为全局配置
├── userCode/
│   ├── algorithms/              ESKF、融合时间线、PID/混控、水压补偿
│   ├── devices/                 IMU、水压计、推进器、PCA/TCA 驱动，入口 Usermain.cpp
│   ├── drivers/ MiddleWares/    BMI088 / IST8310 底层驱动
│   └── tasks/                   FreeRTOS 任务、I2C 总线访问层、VOFA 遥测
├── Firmware/                    ⚠️ 旧版预构建镜像（2026-09-16），不含之后的修复
├── Firmware_UART2/              改用板上 UART2 通信的临时版本（同样是旧版）
├── docs/                        接线、上板、接口、几何等文档
└── tools/build_firmware.py      命令行调用 Arm Compiler 6 编译（不烧录）
```

---

## 编译与烧录

1. 用 **Keil MDK 5** 打开 `Lower_Level_Controller/MDK-ARM/RM_Frame_C.uvprojx`。
   需要 **Arm Compiler 6**（开发时用的是 6.24）和 **STM32F4xx_DFP** 器件包。
2. **Rebuild**，输出文件为 `MDK-ARM/RM_Frame_C/RM_Frame_C.hex`。
3. 用 ST-Link（SWD）连接 C 板，**Flash → Download**。起始地址 `0x08000000`。

> `Firmware/FinsROV_ESKF.hex` 是 9 月 16 日的旧镜像，**没有**包含 ESKF 标定、IMU DMA、I2C 总线恢复等修复。请以 Keil 的编译输出为准。

---

## 首次上电检查

1. **断开推进器动力电源。**
2. 四个水压计口**置于空气中**，机体静止后上电。
   启动时会在空气中采样 100 组压力作为水面参考，**不要在水下上电**。
3. VOFA+：选择对应 COM 口，**115200 / 8 数据位 / 偶校验 / 1 停止位**，协议选 **FireWater**。上电后自动输出 16 通道，不需要发任何命令。
4. 正常时应看到：

   | 通道 | 期望值 |
   |---|---|
   | I10 CAL | 1 |
   | I11 RAW_MASK | 15 |
   | I12 FAULT | 0 |
   | I13 FUSED_MASK | 15 |
   | I9 READY | 1 |
   | I4～I7 | 有数值，不是 -9999 |
   | I15 SEQ | 持续增长 |

5. 倾斜机体，确认 I4（roll）/ I5（pitch）的方向与实际一致。

---

## VOFA 16 通道

| 通道 | 含义 | 单位 / 说明 |
|---|---|---|
| I0～I3 | 水压计 1～4（TCA0 前左、TCA1 后左、TCA2 后右、TCA3 前右） | 旧版刻度，约等于 mbar，零点为上电时的空气读数；1 mbar ≈ 1 cm 水深 |
| I4 | 横滚 roll | 度 |
| I5 | 俯仰 pitch | 度 |
| I6 | 偏航 yaw | 度，**相对上电时的航向**，不是地理北 |
| I7 | 融合深度 | 米 |
| I8 | STOP | 1 = 输出停止（推进器保持中位），0 = 已解锁 |
| I9 | READY | 1 = 控制所需反馈全部可用，可以解锁 |
| I10 | CAL | 1 = 上电水压标定成功 |
| I11 | RAW_MASK | 拿到新鲜原始读数的水压计，按 1/2/4/8 相加 |
| I12 | FAULT | 1 = 严重故障锁存，需要重启 |
| I13 | FUSED_MASK | ESKF 实际采用的水压通道，按 1/2/4/8 相加 |
| I14 | ERROR_CODE | 见下 |
| I15 | SEQ | 包序号，持续增长说明数据在刷新（65535 后回 0） |

**-9999 表示缺测、过期或未满足显示条件**，不是真实的角度或深度。

**I14 解码：**
- `I14 ÷ 65536` 取整 = ESKF 标志：1 姿态有效、2 深度有效、4 已观测航向、8 配置就绪、16 陀螺连续。正常为 **31**。
- `(I14 ÷ 256) mod 256` = I2C2 累计错误次数。
- `I14 mod 256` = **上电后第一次**停止的原因，之后不再更新；上电时为 6 属正常。原因编号：1 心跳超时、2 控制任务卡住、3 总线回复超时、4 断言、5 输出写失败、6 反馈不可用、7 控制周期超时、8 控制计算无效、9 人工 OFF、10 请求重新标定、11 温控故障。

---

## 串口命令

每条命令以 **CR/LF** 结尾，最长 99 个字符，整行须在 250 ms 内发完。

| 命令 | 作用 |
|---|---|
| `HB` | 心跳，约每 100 ms 发一次；只续期，不会解锁 |
| `ON` | 解锁，**只发一次**。成功后回复 `CTRL=ARMED ESKF`，STOP 变 0 |
| `OFF` | 停止，推进器立即回到中位 |
| `TES:v0,...,v7` | 手动 8 路推进器 PWM（1000～2000 µs），仅在关闭闭环时使用，仍需心跳 |
| `ACL:ON` / `ACL:OF` | 偏航闭环开 / 关（默认关；开启时以当前航向为目标） |
| `VOFA:ON` / `VOFA:OFF` | 开 / 关 VOFA 曲线输出（不影响控制） |
| `STAT` | 一行状态：STOP、READY、CAL、ESKF 标志、水压通道、故障、停止原因 |
| `DIAG` | 一行诊断：控制周期耗时、超期次数、I2C 错误、温度 |
| `EDIAG` | 七行详细诊断，见下 |

完整命令表见 [docs/CONFIG_AND_INTERFACES.md](Lower_Level_Controller/docs/CONFIG_AND_INTERFACES.md)。

**解锁流程**：持续每 100 ms 发 `HB` → 确认 READY=1 → 发一次 `ON`。
超过 500 ms 没有心跳、收到 `OFF`、反馈失效、控制周期超时或总线出错，都会自动停止；停止后即使心跳恢复也**不会**自动重新解锁，必须再发 `ON`。

> ⚠️ `ON` 会立即进入闭环并带有固定浮力补偿，推进器**会转**。不要循环或定时发送 `ON`。

### EDIAG 输出

| 行 | 内容 | 重点看 |
|---|---|---|
| EDIAG1 | READY 失败原因（位掩码）、等待超时、ESKF 标志、IMU 温度 | `NR=0` 表示 READY 条件全部满足 |
| EDIAG2 | 姿态帧无效原因统计、陀螺/加计数据年龄 | `OK` 应接近 `PUB` |
| EDIAG3 | IMU 各类包数 / 中断数、DMA 错误、队列丢包 | `PKT` 应接近 `EDGE` |
| EDIAG4 | ESKF 预测 / 加计 / 磁力计更新与拒绝次数 | `AR`、`MR`、`GAP` 应不增长 |
| EDIAG5 | 水压更新与拒绝次数 | 空气中 `PU=0` 属正常 |
| EDIAG6 | IMU 任务 CPU 占用、各环节最长耗时（µs）、超期次数 | `CYCLE` < 6667，`MISS` 不增长；耗时最大值读后清零 |
| EDIAG7 | I2C2 / I2C3 错误与自动恢复次数、丢弃的水压帧、最后一次错误码 | `ERR` 应等于 `REC` |

**最后一次错误码 `E2=` / `E3=`**（十六进制）：
- **最高一位：** 操作类型，1 发送、2 接收、3 读寄存器、4 写寄存器、B 请求时间预算用尽。
- **接下来两位：** 从机地址，70 = TCA9548A，76 = MS5837，40 = PCA9685。
- **再两位：** HAL 返回值，1 错误、2 忙、3 超时。
- **最后两位：** 错误类型，01 总线错误、02 仲裁丢失、04 无应答、20 超时。

---

## 运行架构

入口为 `userCode/devices/Src/Usermain.cpp`：
1. 推进器先写中位；
2. 初始化各设备；
3. 空气中标定水压；
4. 启动以下四个任务。

| 任务 | 职责 | 优先级 |
|---|---|---|
| `ImuFusionTask` | 解码 SPI DMA 的 IMU 数据、IMU 温控、按时间排序观测、运行 ESKF | 5 |
| `ControlLoopTask` | 150 Hz 控制主循环：解析命令、检查反馈、PID 与混控、发出 PWM 请求、诊断输出 | 4 |
| `PressurePwmTask` | 独占 I2C2：水压采集（3 个周期一帧，50 Hz）、写 PCA9685 | 3 |
| `UartTransmitTask` | 串口异步发送 | 2 |

- **时基：** TIM1 产生 150 Hz 控制节拍；TIM2 提供 1 µs 时间戳；TIM7 为 HAL 毫秒时基。
- **IMU 采集：** 由数据就绪中断触发，PC5 为陀螺、PC4 为加计、PG3 为磁力计。陀螺和加计走 SPI1 + DMA。
- **总线分工：** I2C2 连接 TCA9548A（通道 0～3 为水压计，通道 4 为 PCA9685）；I2C3 连接磁力计。
- **配置入口：** 全局参数在 `Inc/LowerControllerConfig.h`，ESKF 参数和安装几何在 `userCode/algorithms/Src/FusionConfiguration.cpp`。

---

## 安全与停止机制

- **默认停止：** 上电默认停止，只能由 `ON` 解锁；水压标定失败或反馈无效时拒绝解锁。
- **停止时的输出：** 8 路推进器写中位 **1550 µs**，舵机保持当前请求值；重新解锁前必须先成功写出一组中位。
- **总线回复超时：** 控制任务等待总线任务回复最多 **40 ms**，超时即严重故障锁存（FAULT=1），需要重启。
- **I2C 自动恢复：** I2C 出错后，在下一次请求前自动执行总线恢复（软件复位外设 + 9 个 SCL 时钟 + STOP + 重新初始化）；出错的水压帧整帧丢弃。
- **死机上报：** 发生 HardFault、断言失败或栈溢出时，串口每秒重复输出一行 `CRASH=...`，包含出错的 PC、故障寄存器、任务名或文件行号。
- **看门狗：** 当前**未启用**（`LC_IWDG_ENABLED 0`）。

> ⚠️ PCA9685 的 OE 引脚未接线，I2C 故障时软件无法保证推进器回到中位。**必须有独立的动力切断手段。** 调试断点也不能当作推进器急停。

---

## 已知问题

- **控制周期余量小：** 水压一步约 3 ms，写 8 路 PWM 约 2.5 ms，解锁后接近 6.67 ms 的周期上限。若 `EDIAG6` 的 `MISS` 持续增长，需要把 I2C 改为中断 / DMA 方式。
- **IMU 任务 CPU 占用约 41%**（`EDIAG6 LOAD≈414`），原因尚未定位。
- **I2C2 偶发错误：** 约 10 分钟出现 3 次，能自动恢复；错误类型待 `EDIAG7 E2=` 确认是电气问题还是时序问题。
- **IMU 安装方向**沿用原版约定，待确认（`TODO(IMU_INSTALLATION)`）。
- **偏航**以上电航向为零点，不是地理北。
- **推进器死区** 1510～1610 µs 沿用原记录，未逐台标定。

---

## 文档

| 文档 | 内容 |
|---|---|
| [docs/PORT_MAP_V33.md](Lower_Level_Controller/docs/PORT_MAP_V33.md) | V3.3 接线与端口表 |
| [docs/BOARD_BRINGUP.md](Lower_Level_Controller/docs/BOARD_BRINGUP.md) | ST-Link 烧录、VOFA 设置、上板步骤 |
| [docs/CONFIG_AND_INTERFACES.md](Lower_Level_Controller/docs/CONFIG_AND_INTERFACES.md) | 配置参数与完整命令接口 |
| [docs/FINAL_INTEGRATION_20260916.md](Lower_Level_Controller/docs/FINAL_INTEGRATION_20260916.md) | 集成记录与待办 |
| [docs/CAD_GEOMETRY_20260916.md](Lower_Level_Controller/docs/CAD_GEOMETRY_20260916.md) | 推进器、水压计安装几何 |

`MANIFEST_SHA256.json` 是 9 月 16 日交付包的校验清单，与当前源码不对应。
