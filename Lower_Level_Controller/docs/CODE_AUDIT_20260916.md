# 下位机工程审查记录（2026-09-16）

> 本文保留修复前审查证据。后续修改及剩余项见 [复核与修复结果](AUDIT_RESOLUTION_20260916.md)；原行号可能已移动。

本轮目的：先识别未完善实现、硬件假设和实机验证缺口，提出方案。没有修改生产源码、替换 Firmware 文件或执行烧录。

**结论：主控制链已经实现，但还不能认定为实机验证完成的固件。** 当前确实运行 ESKF + SI PID；有几项不带 TODO 的真实缺陷需要先处理。配置项写成 true 只意味着选择了参数，不等于接线、方向、控制稳定性通过验收。

**范围和证据**

检查了 userCode 各模块的源文件/头文件、Src/Inc 外设与中断、启动/链接配置、两套 Keil 工程入口、CMake、构建脚本和文档；对 ST/CMSIS/FreeRTOS/USB 库检查了使用接口、宏配置、相关时序行为与构建接入，没有宣称逐行审计整套第三方库。
本轮通过工具运行工程自带 build_firmware.py，154 个源文件编译成功、链接成功。脚本没有继承 Keil 全部选项，详见第 4 项，因此本次生成的 axf 仅作审查证据。
原 Firmware 三个文件以及 build-info 中列出的源文件、头文件和工程文件 SHA-256 均匹配清单。它只证明文件一致，不证明实机通过；清单本身也标记 hardware_tested=false。
最小主机复现使用生产 PID/ESKF/控制有效性代码，只有 PID 对 HAL 的头文件依赖做了替身：
- temperature_pid_finite=0：预置非零内存后，构造 PID 参数并按实际方式复制，可得到 NaN。
- after_1999_rejected_accel: rejected=1999 state_usable=1 mask=1：持续拒绝加速度、只有一路压力、无磁观测，约两秒后仍可用于不启用偏航的控制。
- Arm O2 汇编中 BMI088_delay_us 函数只剩 bx lr，延时循环消失。

**一、优先修复的软件问题**

1. **微秒延时是空转循环，优化后消失。优先级高，已确认。**
   位置：[userCode/MiddleWares/Src/BMI088Middleware.cpp:22](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/MiddleWares/Src/BMI088Middleware.cpp:22)。
   BMI088 初始化多处调用此函数等待器件内部处理。当前普通局部变量循环没有可观察作用，Arm O2 把整个等待删了。本轮验证的是“延时不存在”，不是断言每块板必然初始化失败；慢 SPI 和其他调用开销可能暂时掩盖问题。
   方案：用 DWT 周期计数或提前启用的微秒定时器实现有界等待，按寄存器访问/模式转换分别遵守手册时间；SPI 读写返回错误时停止该步骤。不能仅加 volatile 就宣称微秒准确，也不能在初始化时调用尚未启动的 TIM2。ist8310_delay_us、旧 PressureSensor::Delay_us 也有同类空循环，应一并清理。
   器件写入确有等待要求，见 [Bosch BMI088 数据手册第 45 页](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。

2. **温控 PID 历史状态未初始化。优先级高，已复现。**
   位置：[userCode/algorithms/Inc/PID.h:26](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Inc/PID.h:26)、[userCode/devices/Src/IMU.cpp:43](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/IMU.cpp:43)。
   带参数构造函数只填写增益/限幅，没填 err、errSum 等状态；IMU::Init 的局部 _tempPID 随后整体复制给温控 PID。进入闭环时会使用栈里原有内容，可能产生随机积分或 NaN。
   方案：构造函数初始化所有成员；赋值参数后显式 Reset；PID 输入/输出做有限值检查，异常关闭加热。本项影响当前实际温控，不只是停用的旧推进器 PID。
   顺带修正：预热阶段超过 45°C 后仍持续最大设定功率，累计超过 200 次才切换；计数不要求连续，且“积分预置一半”的代码实际改的是积分限幅。应增加超温独立关断、预热超时、连续温度合格判断，并真正设置积分状态。IMU.h 写的 PWM 最大 2000 与 TIM10 ARR=4999 也应明确统一含义。

3. **持续收到坏加速度，也可能一直保持“姿态有效”。优先级高，已复现。**
   位置：[userCode/algorithms/Src/AttitudeEskf.cpp:307](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/AttitudeEskf.cpp:307)、[userCode/algorithms/Src/AttitudeEskf.cpp:538](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/AttitudeEskf.cpp:538)、[userCode/tasks/Src/ImuFusionTask.cpp:143](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/ImuFusionTask.cpp:143)。
   任务检查“最近有没有收到加速度包”，估计器检查初始化与陀螺连续性；没有检查“多久没成功用观测纠正姿态”。持续超门限的加速度仍刷新任务接收时间。只有一路压力时也不做压力姿态校正，便可能长期只靠陀螺推算却保持 READY。
   方案：记录最后一次成功的加速度/磁场/压差校正时间，按横滚、俯仰、偏航分别评估可观测性和协方差；允许短暂机动时降级，超过规定时间或不确定度上限则锁存停止。阈值需实测，不能一拒绝观测就立即停机。

4. **两种 Arm 构建入口的浮点选项不同。优先级高，已确认。**
   位置：[tools/build_firmware.py:29](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/tools/build_firmware.py:29)、[MDK-ARM/RM_Frame_C.uvprojx:341](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/MDK-ARM/RM_Frame_C.uvprojx:341)。
   Keil 显式使用 -fno-fast-math -fno-finite-math-only；脚本只使用 -O2，没有读取 MiscControls。本轮脚本编译 Sensor.cpp 明确出现 -Wnan-infinity-disabled 警告，会使 NaN/Inf 检查的语义不可靠。
   方案：统一编译选项来源，至少补齐这两个选项；为真实目标编译增加 NaN/Inf 保护验证。不能把脚本“编译通过”当成与 Keil 构建行为一致。本次审查产物不要拿来替换交付固件。

5. **初始化通信失败的传播不完整，失败时可能等很久。优先级高。**
   位置：[userCode/devices/Src/Extension.cpp:15](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Extension.cpp:15)、[userCode/tasks/Src/I2cBusAccess.cpp:22](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/I2cBusAccess.cpp:22)、[userCode/devices/Src/Usermain.cpp:195](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Usermain.cpp:195)。
   TCA/PCA 写函数和设备 Init 返回 void，主函数只打印 RETURNED；尚未设置总线预算时保留旧超时，存在 10000/65535 ms 参数。BMI088_read_write_byte 忽略 HAL 返回值，失败时返回的局部字节也未初始化。正常启动的 IWDG 要等控制任务运行后才开启。
   方案：初始化也用总预算和有限重试；每一步返回明确错误码；PCA MODE1/MODE2/PRESCALE 回读验证；区分 BOOT=OK 和 BOOT=FAIL；无法确认中位写出时禁止解锁。初始化错误应能被串口/LED 看见，不能只停在断言死循环。

6. **PWM 多字节与多通道更新不是一组提交。优先级高，实测波形待确认。**
   位置：[userCode/devices/Src/Extension.cpp:75](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Extension.cpp:75)、[userCode/tasks/Src/PressurePwmTask.cpp:60](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/PressurePwmTask.cpp:60)。
   每路四次独立写，12 路共 48 次写；跨越低/高字节边界时寄存器会经历中间值。例如从计数 266 改为 246，先写低字节会短暂组合成 502。是否体现为某个异常脉冲取决于更新时机，必须用逻辑分析仪查。
   OFF/期限超时若发生在整组写出中途，已经写出的早期通道不会在这次循环中回头写中位；OE 未启用时需要后续周期修正。
   方案：配置自动递增与在 STOP 更新，成组突发写入；急停代次变化后重写整组中位；按需要配合 OE。对“写出中途 OFF”“总线失败”“跨字节脉宽变化”注入测试。硬件支持的更新机制见 [NXP PCA9685 数据手册](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)。

7. **串口依赖 IDLE 分包，不能可靠处理被拆开的命令。优先级中高。**
   位置：[userCode/tasks/Src/TaskInterruptHandlers.cpp:38](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/TaskInterruptHandlers.cpp:38)。
   同一 IDLE 包里的多行能拆开，但不同 IDLE 包之间不拼接。一次 PC write 并不构成端到端串口帧边界保证。半截 MOT/H 等命令还可能被旧解析器先执行。
   方案：环形接收缓存 + 持久行缓冲，完整 CRLF 行才执行；设置长度、超时与溢出丢弃策略，避免把超长行的尾巴当新命令；测试所有拆包位置。中断急停检测也应基于明确定义的完整命令帧，保留队列满时 OFF 仍生效的能力。

8. **旧命令解析仍允许部分参数和非法后缀。优先级中。**
   位置：[userCode/devices/Src/Servo.cpp:137](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Servo.cpp:137)、[userCode/devices/Src/Propeller.cpp:321](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Propeller.cpp:321)。
   MOT:1600 只改第一路；MOT:1600abc,... 可由 atoi 当作 1600；H:abc 变为零，单字母运动命令只看首字符。TES/ON/FSET 已有较严格检查，不能据此推断所有命令都严格。
   方案：统一解析接口，检查命令全名、字段数、分隔符、范围、溢出、尾部字符，临时结构全部通过后一次提交；失败返回 REJECT 且保持原目标。对普通运动/舵机命令也检查停止代次与时效，避免旧队列命令跨停机边界生效。

9. **故障诊断不足以解释部分自动停止。优先级中。**
   位置：[userCode/tasks/Src/ControlLoopTask.cpp:254](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/ControlLoopTask.cpp:254)、[userCode/tasks/Src/TaskSharedState.cpp:54](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/TaskSharedState.cpp:54)、[Src/stm32f4xx_it.c:90](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Src/stm32f4xx_it.c:90)。
   反馈门控/周期超时调用普通 LatchOutputStop，不更新 WHY；控制截止时间、丢样等主要留在调试变量。用户可能看到 STOP=1、FAULT=0、WHY=0，却不知道发生了什么。HardFault 等处理器仅死循环，无寄存器快照；复位后故障位置丢失。
   方案：所有停止入口带原因及时间；新增低频 DIAG 输出耗时、队列丢包、采集计数、拒绝原因和栈余量；故障时保存 PC/LR/CFSR/HFSR/复位原因到保留区，启动后打印。不要在高优先级中断里格式化长日志。

**二、仍处于假设或待标定状态的参数**

10. **IMU 与磁力计到机体的轴映射是假设。**
    位置：[userCode/algorithms/Src/FusionConfiguration.cpp:10](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/FusionConfiguration.cpp:10)。
    现在两者都用 diag(-1,+1,-1)，加速度比力到重力另乘 -1；installation_configured=true 不验证实物朝向。矩阵数学上合法，不代表装反的板子也正确。
    方案：断开推进器动力，记录放平、右侧下压、抬头、水平转向时的角度和角速度符号。机体系为前/右/下，右侧下压应正 roll，抬头应正 pitch；分别确认 BMI088 和 IST8310 芯片轴与整机安装，必要时采用不同矩阵。

11. **传感器校准常数沿用旧机，不能等同于本机实测。**
    位置：[userCode/drivers/Inc/BMI088driver.h:44](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/drivers/Inc/BMI088driver.h:44)、[userCode/devices/Src/IMU.cpp:54](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/IMU.cpp:54)、[userCode/devices/Inc/IMU.h:61](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Inc/IMU.h:61)。
    加速度矩阵 C01..C09、偏置 0.07/-0.17/0、磁力计偏置/缩放均固定；陀螺启动偏置设为零。ESKF 会估计陀螺偏置，但还需要合格观测和收敛时间。
    方案：恒温静止统计陀螺偏置，六面法标定加速度，整机装配后标定磁力计，再测试电机通电/转动对磁场的影响。现有未调用的 calibrate_offset 不能直接启用：它把加速度三轴均值全减掉，会把静止重力也当成零偏。
    航向只相对本次启动，不是地理真北；当前首次合格单帧即学习磁参考，应改为多帧稳定性评估。

12. **压力传感器几何用了 CAD 口沿，膜片位置尚未实测。**
    位置：[userCode/algorithms/Src/FusionConfiguration.cpp:46](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/FusionConfiguration.cpp:46)。
    四口参考跨度 249.4×138.2 mm，z 全为 0；实际膜片高度误差会被当成姿态差。传感器通道按现有文档选定，仍要核对本机接线。
    方案：逐路辨识 TCA0..3，测敏感膜片相对同一机体原点的 x/y/z；静水放平及已知倾角时核对压力差方向和幅度。不能靠长期调零掩盖几何错配。

13. **上电归零依赖“在空气中静止”的操作前提。**
    位置：[userCode/devices/Src/Sensor.cpp:69](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Sensor.cpp:69)。
    四路各 100 样本，稳定性阈值 200 Pa。浅水下稳定启动的压力也可能落在 80–120 kPa 范围，软件无法仅靠此范围知道它是在空气中。IWDG 复位有恢复锁，但普通复位/掉电重启仍走校准。
    方案：首次必须空气中归零；产品化时增加明确校准流程和经过版本/CRC 验证的参考保存，按复位原因进入保守状态。水下恢复不要直接重定义水面。密度当前 1000 kg/m³，盐水环境需改；噪声门限应用实际静止记录确定。

14. **融合噪声、门限、同步时间是假定初值。**
    位置：[userCode/algorithms/Src/FusionConfiguration.cpp:25](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/FusionConfiguration.cpp:25)、[userCode/devices/Src/PressureArrayAcquisition.cpp:7](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/PressureArrayAcquisition.cpp:7)。
    压力标准差按 2 cm、通道相关性按零、磁/加速度门限固定；压力时间戳取“发命令结束+1.5 ms”，DRDY 是中断到达代理时间，重排窗口固定 8 ms，不能称为硬件同步。
    方案：分别记录空气/静水/电机运行时的原始数据和时间戳，统计方差与相关性，检查创新和拒绝率；测真实交付延迟，再选排序窗口。还要考虑推进器水流对静水压力模型的干扰。
    额外缺口：压力突跳拒绝后仍更新上一压力基准，持续异常第二帧可能重新被接受。是否允许这种快速重捕获需明确；更稳妥的是保留最近可信值并设置多帧重新确认。
    当前深度是姿态补偿后的压力加权结果，并未实现含垂向速度的动态深度滤波器。

15. **推进器映射和推力分配仍须实物逐路验证。**
    位置：[Inc/LowerControllerConfig.h:109](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Inc/LowerControllerConfig.h:109)、[userCode/algorithms/Src/FusedController.cpp:38](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/FusedController.cpp:38)。
    通道、正反桨已有明确表，不是空映射；分配矩阵只用符号和预定组合，数学上检查满秩不代表真实受力方向正确。控制输出单位是 PWM 增量，尚无牛顿推力标定模型。
    方案：建立 PCA 通道—物理位置—PWM 正负变化—实际推力方向表，限输出逐路验证，再验证 roll/pitch/depth/yaw 的恢复方向。电机型号、桨、安装、舵机角度变化后需重新核对。

16. **PID 和固定浮力补偿是初值，ON 后即可能有明显输出。**
    位置：[userCode/algorithms/Src/FusedController.cpp:11](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/algorithms/Src/FusedController.cpp:11)、[userCode/devices/Src/Propeller.cpp:30](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Propeller.cpp:30)、[userCode/devices/Src/FusedControlAdapter.cpp:19](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/FusedControlAdapter.cpp:19)。
    PID 主要从旧单位/周期换算。FloatPWM 给两路保留 -100/+90 us 偏置，闭环又加死区补偿。即便姿态和深度误差都为零，也不等于八路中位；按当前表相应两路可到约 1470/1760 us。gains_configured/allocation_configured=true 不代表整定完成。
    方案：先确认机械浮力/重心和输出方向；增加调试限幅、输出变化率限制、可关闭/渐入的浮力预置；先调速率内环，再调角度外环，再调深度，最后整合航向和移动。不能仅凭旧 PID 数值直接带动力验证整机。

17. **PWM 实际周期、中位、死区与舵机行程尚待测量。**
    位置：[userCode/devices/Src/Extension.cpp:47](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Extension.cpp:47)、[Inc/LowerControllerConfig.h:88](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Inc/LowerControllerConfig.h:88)、[userCode/devices/Src/Servo.cpp:137](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Servo.cpp:137)。
    PCA 频率沿用 1.016 修正，但脉宽换算固定 20 ms；1610、1570..1670 us 和 +60/-40 us 补偿不是逐台电调测值。无小误差滞回，微小噪声也可能跳出死区。舵机全路 500..2500 us、第三路反向是旧机经验，不能证明机械无碰撞。
    方案：用逻辑分析仪测真实周期和脉宽，校准 PCA 时钟；逐路测停止区/正反启动阈值，视实测添加滞回；为每只舵机配置中心、反向和机械安全端点。

18. **150 Hz 控制期限还缺真实板上证据。**
    位置：[userCode/tasks/Src/ControlLoopTask.cpp:236](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/ControlLoopTask.cpp:236)、[userCode/tasks/Src/PressurePwmTask.cpp:60](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/tasks/Src/PressurePwmTask.cpp:60)。
    每周期约 6.667 ms，包含压力步骤、估计交接、48 次 PWM 寄存器写、诊断格式化及更高优先级 IMU 任务抢占。代码有期限门控，但“设置了150 Hz”不等于每周期都完成。外部 PWM 波形仍是 50 Hz。
    方案：先突发写减少 I2C 占用；将显示格式化移出控制关键路径；统计典型/最坏耗时、栈余量、丢样、deadline_misses，在串口、USB及传感器故障注入下长时间运行验证。SPI 当前约 328.125 kbit/s，提速需先修延时并按手册/波形验证。

19. **硬件急停尚未落实，OE 函数当前是空操作。带动力前必须处理。**
    位置：[Inc/LowerControllerConfig.h:41](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Inc/LowerControllerConfig.h:41)、[Src/iwdg.c:74](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Src/iwdg.c:74)。
    LC_PCA_OE_ENABLED=0 时 SetOutputEnabled 什么也不做；控制器停机/复位不等于独立 PCA 停止输出。I2C 故障时可能无法写入 1610 us。HardFault 等也没有直接拉 OE 的路径，即使后续接线仍需补齐。
    方案：独立切断推进器动力，并验证电调失去 PWM 的行为；若加 OE，则上拉保证复位默认禁用、明确 GPIO、显式 MODE2 输出状态，异常路径直接关闭。OE 同时影响舵机，禁 PWM 不能等同于切动力。未接实物时不能只把宏改为 1。

**三、遗留接口和工程维护缺口**

20. **CMake/GCC 入口当前不完整，CubeMX 文件与实际代码有偏差。**
    位置：[CMakeLists.txt:7](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/CMakeLists.txt:7)、[Src/main.c:71](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/Src/main.c:71)、[RM_Frame_C.ioc:510](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/RM_Frame_C.ioc:510)。
    CMake 指定 GCC，main.c 却对非 Arm 编译器 #error，并依赖 rt_sys.h/rt_misc.h。Matrix.cpp 包含 matrix.h 与真实文件名大小写不一致，在区分大小写的平台会失败。
    .ioc 的 TIM10 是 PSC=167、ARR=999，当前 tim.c 是 PSC=0、ARR=4999；IWDG 分频也不同。再生成可能覆盖实际行为。
    方案：近期固定唯一 Keil 入口；补齐 GCC retarget 后再验证 CMake。同步 .ioc 与手工扩展、检查再生成 diff，保留可复现的构建与打包脚本。

21. **旧 Flash 参数保存代码不能直接启用。**
    位置：[userCode/devices/Src/Legacy.cpp:71](C:/Users/lixin/Desktop/workspace/FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main/Code/Lower_Level_Controller/userCode/devices/Src/Legacy.cpp:71)。
    read 的 memcpy 只复制 sizeof(struct)/4 字节：16 字节结构只读 4 字节。write 的结束指针先按结构体步长相加，导致多读多写；擦除传 nullptr 作为错误扇区输出参数；均未核对返回状态。参数扇区也未从固件链接空间排除。
    当前保存调用被注释，启动读到的 flashData 不供新 ESKF 校准使用，所以不要误报为现有主控制依赖旧 Flash。
    方案：需要持久化时重写，固定长度、版本、CRC、有效标记、断电保护、独立扇区；修复前禁用写入口。

22. **若干接口只有壳或没有接入当前流程。**
    - USB CDC 初始化了，但 CDC_Receive_FS 只重新挂接接收，没有解析控制命令；默认遥测走 UART6。插板载 USB 出现 COM，不意味着能用它控制。
    - usart_printf 的格式化/发送被注释；_sys_write 直接返回成功却不输出；clock()/time() 返回未初始化变量。应明确不支持或实现正确替代，业务时间用 LcTime_NowUs。
    - speed_ctrl() 空实现，VEL 分支注释，W/A/S/D 是固定 PWM 开环移动，没有水平速度闭环。
    - ADC 电压监测函数没接入任务，没有电池电压告警闭环；“TODO adc校准”不能靠补一句校准 API 就算完成。需先确认分压与参考、调用采样、滤波和阈值。
    - Sonar、板载直出 Servo、旧 Watchdog、Buzzer 未在当前 device[] 启用；不能仅取消注释就共用现有 UART/TIM。PA0 按键无业务动作。
    - BMI088/IST8310 的 GPIO_init/com_init 空函数是兼容钩子，真实初始化已由 MX_* 完成，不应都当漏实现。
    - 旧 UARTBaseLite 的异步发送立即出队，缓冲复用/错误处理不健全；当前 UART6 使用新的独立发送任务，不能据此误判当前发送队列有同样问题。
    - 旧平面拟合、Mahony/Madgwick、Map.h 等保留代码不在默认输出路径；Map.h 重复键插入会泄漏，旧平面求解也不能作为已验证备用控制器。
    方案：明确保留、禁用、待开发三种状态，把历史代码移出生产目标或建立独立参考目录；只恢复本次实际需要的接口。

23. **测试材料和现场诊断还没有随工程形成闭环。**
    README 说测试归档在备份目录；交付工程缺少直接可运行的完整回归入口。已有测试通过记录不覆盖本轮发现的温控、延时优化、持续观测拒绝等场景。
    方案：把关键回归测试纳入工程，覆盖分包/畸形命令、OFF 中断插入、总线失败、观测失效、NaN、启动和复位路径；每次修复重新构建同一生产目标，打包源码哈希/编译选项/固件标识，保留上板日志和逻辑分析仪记录。

**关于“影子映射”和 TODO 的准确解释**

- 影子模式通常指只计算、比较新算法，实际输出仍由另一控制器决定。当前 ControlLoopTask 对 FB:SHADOW 和 FB:LEGACY 都拒绝，真正输出是 FusedController 的 result.command。不能把它当成“ON 也不会驱动”的影子测试固件。
- IMU 换轴、压力通道与推进器分配是实实在在参与控制的映射，区别只在于有些已按资料选择但没有实物验收。
- 设备数量、V33 舵机 8..11 通道、旧压力偏置等 TODO 不都是缺功能：数量已有统一宏；通道已有配置；新压力零点由启动标定覆盖旧常数。
- 原版低温混合公式仍在 legacy 数值计算与初值解释中，但新 ESKF 压力输入使用独立 CompensateMs5837(30BA)，不能说新 ESKF 直接把旧压力公式当 Pa。
- 表中的“待测”不应靠删注释或把 configured 改 true 来解决；需要测试记录和验收标准。

**建议执行顺序**

1. 先修延时、温控初始化、构建选项、通信解析、诊断和观测失效判定；整理唯一可复现的生产构建入口。
2. 改进 PCA 初始化/成组写出；不接推进器动力，用示波器/逻辑分析仪验证中位、变更、OFF、复位和通信故障。
3. 烧录后先验证 BOOT、CAL、STAT、VOFA、DRDY 和长时间期限统计，再做换轴/水压/磁场标定。
4. 完成独立动力急停，逐路限输出测试推进器和舵机方向，得到本机通道/死区/行程表。
5. 受控水槽内逐环调 PID，逐步恢复浮力预置与输出范围，验证失联、拒绝观测、传感器故障及掉电行为。

完整编译日志：[build-report.json](C:/Users/lixin/Desktop/workspace/.finsrov-work/audit-20260916/arm-build/build-report.json)。
最小复现：[reproduce.cpp](C:/Users/lixin/Desktop/workspace/.finsrov-work/audit-20260916/reproduce.cpp)。
延时汇编：[BMI088Middleware.s](C:/Users/lixin/Desktop/workspace/.finsrov-work/audit-20260916/BMI088Middleware.s:86)。

