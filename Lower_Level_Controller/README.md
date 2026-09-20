# FinsROV 下位机：ESKF 控制固件

当前版本：ESKF_VOFA_20260916_R2，V33 / RoboMaster C 板 / STM32F407IG / 四个 MS5837-30BA。

默认且唯一的控制路径为 **异步传感器采集 → ESKF 姿态与深度 → SI PID → 原版推进器 PWM**。
FreeRTOS 运行四个任务，控制频率 150 Hz，姿态外环/完整水压帧 50 Hz。
旧控制器不参与输出，旧 Mahony 不参与默认运行，无需发送反馈切换命令。
上电先保持推进器停止，完成压力校准及估计器初始化后，仍需人工发 ON 才会启动。

本版完成审查中多项软件缺陷修复，仍只适用于按步骤进行台架验证，不能直接认定可可靠下水。
逐项处理结果：[23项复核与修复记录](docs/AUDIT_RESOLUTION_20260916.md)。
位置依据：[19份STEP的接口与装配坐标核对](docs/CAD_GEOMETRY_20260916.md)。
本轮结果：[最终对接与修正](docs/FINAL_INTEGRATION_20260916.md)；照接表：[推进器/舵机/压力端口](docs/PORT_MAP_V33.md)。

## 上板入口

- 推荐工程：`MDK-ARM/RM_Frame_C.uvprojx`，Arm Compiler 6.24，ST-Link，目标 RM_Frame_C。
- 已生成固件：`Firmware/FinsROV_ESKF.hex`；二进制另附，Flash 起址 0x08000000。
- 主入口：`userCode/devices/Src/Usermain.cpp::main()`。
- 详细操作：[首次上电与停机](docs/BOARD_BRINGUP.md)。
- 参数依据：[配置与接口](docs/CONFIG_AND_INTERFACES.md)。
- 任务分工：[运行框架](docs/FREERTOS_RUNTIME.md)。

串口 UART6：115200、8 数据位、偶校验、1 停止位。每条命令带 CRLF，一次完整发送。

1. 四个压力口在空气中静止上电，等待 `CAL=OK`。
2. 发 `STAT`，确认 `READY=1,CAL=1,FAULT=0`。STOP=1 是正常待机。
3. 每约 100 ms 发 `HB`，需要运动时另发一次 `ON`。
4. `OFF` 停止；500 ms 无心跳也停止，恢复心跳不自动重启。
5. VOFA 选择 FireWater，默认显示 16 通道、约10Hz；角度为度、深度为米，通道表见操作文档。
   `VOFA:ON/OFF` 只开关显示；`FLOG:ON/OFF` 是兼容别名，已取消旧 F/Q 两种日志。

## 已采用与仍待实测

推进器通道、正反桨、50 Hz PWM、1610 us 中位、1000～2000 us 限幅、浮力补偿，以及传感器原有偏置/磁场补偿沿用原版。
新 PID 速率环比例增益照原值；其余按采样周期和物理单位换算。几何采用用户指定的 STEP 接口中心，TCA 顺序按《V3.3设计》p7改为前左、后左、后右、前右。
新闭环按PDF p9的1570..1670us死区，采用+60/-40us补偿，1610us停止不变。
`installation_configured/gains_configured/allocation_configured=true` 表示已选择参数，不代表已经做过实机标定。

已由 zh-CN STEP 确认四个压力接口、八个推进器、四个舵机轴及舱内板件的设计位置；压力位置数值与代码一致，不再保留膜片位置 TODO。
本轮已按官方例程修正BMI088/IST8310板内轴差，并修正新FRD偏航闭环分配符号。质心与浮心按用户要求共处四压力口平均几何参考中心，偏置为0。
仍待实物确认：整块C板安装朝向、实际接线与推力方向、新闭环整定、独立动力急停。官方模型与仓库的36×36mm、直径2.5mm孔阵列匹配，但180°旋转仍能对孔，不能唯一确定朝向。两份总装后盖相对密封舱的朝向差180°，舱内位置按共同密封舱坐标对齐，详见几何记录。
当前 OE 接线尚未确认、代码宏关闭，因此没有已验证的硬件 PWM 禁止功能。看门狗和 OFF 不能代替独立切断推进器电源。
磁航向相对本次启动，不是地理真北。空气中不使用水压差校正姿态。

本固件已完成主机模拟、驱动解码、Arm 编译/链接及本机 Keil 5.43a / AC6.24 离线构建，未烧录、未做水下实测。
已修复 Keil 的编译器版本选择与 FreeRTOS LTO 链接错误，保留无效浮点检查；Keil 构建为0错误、1条FreeRTOS断言常量警告。
测试、临时缓存及旧阶段文档已从交付目录清理，修改前源码和验证材料归档在 workspace 的 `.finsrov-backups`。
