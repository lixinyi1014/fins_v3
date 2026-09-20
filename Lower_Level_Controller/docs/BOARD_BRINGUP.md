# V3.3：第一次用 ST-Link 烧录、用 VOFA 看读数

适用：用户已确认的 V3.3、RoboMaster C 板 STM32F407IG、ST-Link、四个 MS5837-30BA。
固件标识 ESKF_VOFA_20260916_R2。已完成电脑端构建与模拟检查，尚未烧录、未做实物验证。

第一次只完成：**烧录成功 → VOFA 持续更新 → 校准完成 → 手动倾斜方向正确**。
推进器动力断开，不发 ON/TES，也不需要 HB。正常启动后本来就会显示数据。

## 1. 分清两条连接

| 用途 | 连接 | 软件 |
|---|---|---|
| 把程序写入芯片，叫“烧录” | 电脑 USB → ST-Link → C 板 SWD | Keil |
| 看角度、压力、状态；发命令 | 电脑 USB 串口链路 → C 板 UART6 | VOFA+ |

推进器、舵机、压力计各接哪个口，先看 [V3.3完整端口表](PORT_MAP_V33.md)。
**官方外壳丝印“UART1”的三针口对应程序中的UART6；丝印“UART2”的四针口是MCU UART1，不是本程序的VOFA口。**

只连接 ST-Link 的 SWD 不会自动把 UART6 读数送进 VOFA。C 板自身 USB 也不是本固件的 UART6 输出口。

所有接线在断电时进行。按《V3.3设计》第8页插座 Pin1 方向数针脚，不按线颜色猜。

| C 板 SWD | 接 ST-Link |
|---|---|
| 1 SWDIO | SWDIO / DIO |
| 2 SWCLK | SWCLK / CLK |
| 3 GND | GND |
| 4 3.3V | 烧录器有 VTref/VAPP 电压检测输入时，接该输入 |

C 板通过本机原有供电链路供电，推进器支路断开。ST-Link 标“3.3V”的脚可能是电源输出，不能和 VTref 检测输入混为一谈；不能不经核对就与已供电的板并接。不要用烧录器给推进器、舵机和整机供电。

若已有配套 TTL 转差分、USB 转差分模块，沿用配套链路，VOFA 选它生成的 COM。
PDF把四线通信叫“485”，仓库也出现“422”；两线半双工与四线全双工不能随意互换。本固件没有两线RS-485的DE方向控制配置。

若在桌面直接使用 **3.3V逻辑 USB-TTL**：

| C 板 UART6 | 接 USB-TTL |
|---|---|
| 1 GND | GND |
| 2 TXD | RXD |
| 3 RXD | TXD |

TX接对方RX，共地；USB-TTL的VCC不接。RS-232或RS-485信号不能直接接这三个TTL引脚。

## 2. Keil 烧录，一步一步来

双击当前 workspace 的工程：

`C:\Users\lixin\Desktop\workspace\FinsROV-An-Underwater-Camera-Based-Multi-Robot-Platform-main\Code\Lower_Level_Controller\MDK-ARM\RM_Frame_C.uvprojx`

桌面另一份原工程保留作参照。目标选 RM_Frame_C。

1. 按 **Alt+F7**，打开 Options for Target。
2. Device 选 **STM32F407IGHx**；Target 中 Arm Compiler 选 **V6.24**。本机 Keil 5.43a 内已安装该版本。
3. 当前工程已用 O2、保留无效浮点检查，并关闭 Link-Time Optimization。不要再勾 LTO。
4. Debug 选 **ST-Link Debugger**，点击 Settings。
5. ST-Link 接电脑，C 板已接SWD且供电。适配器选自己的ST-Link；Port选 **SW / SWD**，速度先 **1MHz**。
6. SW Device应识别 ARM CoreSight SW-DP 一类设备；空白/报错时先查供电、共地、DIO/CLK，不继续下载。
7. Flash Download 页确认 **STM32F4xx 1MB Flash**，起址 **0x08000000**、大小 **0x00100000**。缺失则Add添加。
8. 勾 Program、Verify，擦除选择所需扇区即可；可勾 Reset and Run。Utilities 页选 Use Debug Driver。
9. 主工程输出目录已放入本轮同版预构建AXF/HEX，未改源码时可直接进行下一步。若要自己构建或改过源码，点 **Project → Rebuild all target files**，等最后一行 **0 Error(s)**。
10. 点 **Flash → Download**，确认下载/校验完成、没有 Flash Download failed。编译成功和烧录成功是两件事。
11. 四个压力口都在空气中，机体静止，控制板断电再上电一次，完成此次压力归零。

当前离线Keil构建保留1条FreeRTOS固定中断优先级断言警告，不妨碍生成固件；新出现的其他警告/错误应记录检查。
正常构建输出是 `MDK-ARM\RM_Frame_C\RM_Frame_C.axf/.hex`；Keil Download使用该工程输出。
`Firmware\FinsROV_ESKF.hex/.bin/.axf` 是另附的预构建文件。以后修改源码、在Keil编译，不会自动更新Firmware副本。
不要把HEX作为源码加进工程。使用上面选定的主工程输出；预构建与源码重建二者任选其一，不混用桌面旧工程。

| 卡点 | 先检查 |
|---|---|
| Compiler V6.23 unavailable | 是否打开workspace新工程；Target应选已安装V6.24 |
| No ST-Link detected | ST-Link USB连接、驱动、适配器选择 |
| No target connected / SW Device空白 | C板供电、共地、DIO/CLK、线长；降到100kHz再试 |
| 缺DFP或Flash algorithm | Pack Installer安装STM32F4xx_DFP；本机已有2.14.0 |
| L6137E / vTaskSwitchContext | 检查是否误开LTO，当前工程已关闭 |
| 许可/代码大小限制 | 保留报错，使用学校已有有效许可，不通过删保护逻辑解决 |
| 下载完成但没曲线 | 查UART6和VOFA，不据此判定烧录失败 |

IWDG在Keil暂停时仍运行。首次用正常运行的程序和VOFA，别把打断点导致的复位锁定误认成传感器坏了。

## 3. VOFA 设置与读数

不同版本面板位置可能不同，核对项目和值：

| 项目 | 设置 |
|---|---|
| 通信方式 | Serial / 串口 |
| COM | 串口转换器插入后新增的口，不固定COM28 |
| 波特率 | 115200 |
| 数据位 | 8 |
| 校验 | Even / 偶校验 |
| 停止位 | 1 |
| 流控 | None / 无 |
| 协议 | **FireWater** |

源码UART的9B+EVEN包含校验位，PC软件应选8数据位+偶校验。
关闭占用同一个COM的手柄程序、其他串口软件。VOFA打开后，固件正常启动会自动产生16个通道，约100ms更新一行。
不需JustFloat、不需写解析插件、不需先ON。显示帧是 `vofa:数字,数字,...`，以真实CRLF结束。

拖入波形控件，右键绑定I0/I1/I2；另建一个绑定I3。没绑定通道的图会空白。
可以点击通道名重命名，角度保留2位小数、深度3位。状态值看右侧数值，不和角度共用纵轴。

| 通道 | 含义 | 空气中首次观察 |
|---|---|---|
| I0 | roll 横滚，度 | 水平接近0；右侧压低应向正变化 |
| I1 | pitch 俯仰，度 | 水平接近0；半球罩端抬高应向正变化 |
| I2 | yaw 偏航，度 | 相对启动航向；从上看顺时针转动应增大，允许±180°跳变 |
| I3 | 融合深度，m | 接近0 |
| I4 | 传感器1 / TCA0，前左，水头m | 接近0 |
| I5 | 传感器2 / TCA1，后左，水头m | 接近0 |
| I6 | 传感器3 / TCA2，后右，水头m | 接近0 |
| I7 | 传感器4 / TCA3，前右，水头m | 接近0 |
| I8 | STOP：1停止，0输出已放行 | **1** |
| I9 | READY：当前控制需要的反馈可用 | 四路接齐且初始化后 **1** |
| I10 | CAL：四路压力校准成功 | **1** |
| I11 | RAW_MASK：新鲜原始压力通道 | 四路正常 **15** |
| I12 | FAULT：严重故障锁存 | **0** |
| I13 | FUSED_MASK：估计器接受的压力通道 | 稳定四路通常 **15** |
| I14 | FLAGS：状态位之和 | 1姿态、2深度、4磁航向、8配置、16陀螺连续；通常27或31 |
| I15 | SEQ：显示包序号，65535后回0 | **持续变化** |

I4..I7是各传感器所在位置的水头，水中倾斜时本来就不相等。
**-9999表示缺测/过期/不满足显示条件，不是真实角度或深度。** RAW_MASK按1、2、4、8相加；14意味着缺TCA0。
串口断开后VOFA可能保留最后一帧；即使READY=1，也要先看I15是否持续变化。
只有裸板、没接压力阵列时，可以先看角度，但CAL=0、READY=0是预期情况。

文本窗口可能看到以下阶段信息：

```text
BOOT=V33 ESKF_VOFA_20260916_R2 UART6_115200_8E1
BOOT=INIT_THRUSTERS
...
BOOT=INIT_IMU
BOOT=OK_IMU
...
CAL=START KEEP_PRESSURE_PORTS_IN_AIR
CAL=OK
CAL=DETAIL state=2 channel=4 samples=100
CTRL=ESKF STOPPED VOFA_FIREWATER_16CH
```

OK表示该设备的软件初始化检查通过（包括通信状态/关键寄存器或PROM检查），不证明电机、坐标和下水测试通过。成功时channel=4表示没有单路失败，不是第5个传感器。
停在BOOT=INIT_IMU，应查IMU初始化；CAL=FAIL应结合失败通道查压力连接与稳定性。
看门狗恢复时每秒重复 `CAL=LOCK_WATCHDOG_RESET POWER_CYCLE_AT_SURFACE`，不启动正常任务和曲线，也不接受ON。排查后回到空气中断电重启。
VOFA晚打开、错过开机文字时，用STAT查询即可。

## 4. 先学三条命令

在VOFA左侧添加命令，选 **Hex / 十六进制**，粘贴整行字节并单次发送。这样不需要猜文本框会不会把反斜杠r、n转成换行。

| 功能 | 命令 | Hex字节 |
|---|---|---|
| 查诊断 | DIAG | 44 49 41 47 0D 0A |
| 查状态 | STAT | `53 54 41 54 0D 0A` |
| 停止 | OFF | `4F 46 46 0D 0A` |
| 开显示 | VOFA:ON | `56 4F 46 41 3A 4F 4E 0D 0A` |
| 暂停显示 | VOFA:OFF | `56 4F 46 41 3A 4F 46 46 0D 0A` |

VOFA:OFF只暂停曲线数据，**不停止推进器**。FLOG:ON/OFF现在是同一显示开关的兼容别名。
STAT文字示例：

```text
STAT=ESKF,STOP=1,READY=1,CAL=1,FLAGS=31,MASK=15,FAULT=0,WHY=0,CAL_CH=4,CAL_N=100
```

不要求FLAGS、WHY等每次完全相同。CAL_CH=0..3是失败的TCA通道，4是全局原因或成功时无单路故障，结合CAL判断。
WHY记录停止原因：1心跳超时、2控制任务卡住、3总线回复超时、4断言、5输出写失败、6反馈不可用、7周期超时、8控制计算无效、9人工OFF、10请求重新校准、11温控故障。严重故障原因保留；普通待机不反复覆盖首个原因，启动不清除历史原因。WHY不是当前故障位，应结合READY/FAULT。
文本使用等号，只有vofa后使用冒号，防止FireWater把文字回执混进曲线。

## 5. 第一次实操验收

1. 控制板、TCA、PCA、四水压计按原装配接好并供电，推进器动力断开；压力口都在空气中。
2. 按最终安装方向放置C板，机体静止上电，等CAL完成。先看I15持续变化，再看STOP=1、CAL=1、FAULT=0，等READY=1。
3. 放平看I0/I1是否接近0；约180°或两轴对不上时查安装换轴，不用VOFA显示偏置遮住它。
4. 站在机尾朝半球罩看，轻压右侧约10°，I0应正向变化；放平再抬高半球罩端约10°，I1应正向变化。一次只做一个动作。
5. 放回水平，应回到附近。空气中倾斜不会产生水柱压差，本固件在空气中不融合压差姿态。
6. 值为-9999或CAL失败：发STAT，记录CAL_CH、CAL_N、RAW_MASK、最后一条BOOT/CAL文字，再查该路供电、插头、SCL/SDA。

进一步验证压力编号：开机归零后，让压力传感器逐个进入浅水，观察I4..I7哪一路增大。此操作无需ON，不必把未验漏的舱体放进水中；不要吹、挤敏感膜片代替水头测试。

## 6. 后续启动和停机

姿态/编号确认、动力急停可用后，才进入推进器测试。固定机器人，按推进器允许的工况在水中做低输出试验，不在桌面开启定深闭环。

持续启动需要约每100ms发送HB（Hex `48 42 0D 0A`），再**单次**ON（`4F 4E 0D 0A`）。
VOFA版本的定时发送入口可能不同；没有确认能稳定发送HB前，不做持续启动，也不放宽固件500ms超时。
**不要循环发送ON/TES**，否则会在停止后被新的启动命令再次启动。
CTRL=ARMED / STOP=0表示软件放行，不证明推力方向正确。

可以先断开推进器动力，满足READY后只发一次ON、不再发HB：约0.5秒后应回STOP=1，显示另有约0.1秒刷新间隔。传感器或时序保护也可能更早停止。然后验证OFF后恢复HB仍不自动启动。

正常停止：OFF → 控制板 → IIC → PCA → 八路1610us → 电调停止，舵机保留请求。
控制板/IIC故障时，中位可能发不出去，PCA可能仍发旧PWM；看门狗只复位控制板，不切电调动力。
因此必须能直接切断八个推进器的共同动力支路，尽量保留控制板供电看原因。开关/接触器要能承受实际直流电压电流，普通小按钮不能直接承载总电流。具体动力切断装置尚未落实。
OE也尚未接，宏仍为0；不能只改成1。OE只禁PWM，且影响同板舵机，不等于切断动力。

## 7. 本次依据和剩余TODO

V3.3设计p7：压力顺序改成前左、后左、后右、前右；推进器、舵机通道核对一致。p8：SWD/UART/IIC线序。p9：死区1570..1670us。
停止仍1610us；新闭环正向补60us、反向补40us；TES仍代表原始PWM。约0.59V缺完整测量条件，不能拿来设供电或替代PWM脉宽。
已遍历 zh-CN 的19份STEP；按用户指定的接口中心，压力间距确认为249.4×138.2mm，此位置项关闭。八个推进器、四个舵机轴和C板也已定位；官方C板模型与安装孔匹配，BMI/IST板内相对轴已修正。简化模型绕法线180°仍匹配，整板安装方向未唯一确定，详见 FINAL_INTEGRATION_20260916.md。质心/浮心按用户指定取共同几何参考中心，不再要求测这两个点。
V4指南是另一套RTSS-2026工程、DAPLink/OpenOCD和网线电力载波流程；本V3.3不照搬IP设置、24V接线或烧录命令。

剩余5类实物工作：

- IMU方向：板内轴差已修；通过放平、右侧压低、抬头确认整板朝向，随后才确定是否改安装矩阵。
- 压力实测：静态噪声、已知水头、实际通道接线；接口位置已确认，不要求再测内部膜片偏移。
- 推进器/闭环整定：逐路方向、电调死区、浮力平衡、新PID初值。
- 动力急停/OE：需要实物接线与验证，软件不能替代。
- 真实运行：150Hz期限、干扰、实际串口模块、心跳和持续运行，尚无上板数据。

官方参考：[Keil烧录](https://www.keil.com/support/man/docs/uv4cl/uv4cl_flash_programming.htm)、[下载配置](https://www.keil.com/support/man/docs/uv4cl/uv4cl_fl_usingflashmenu.htm)、[ST-Link手册](https://www.st.com/resource/en/user_manual/um1075-stlink-v2-in-circuit-debugger-stlink-v2-1-stlink-v2-ct-stlink-v3-pg00061083.pdf)、[FireWater格式](https://www.vofa.plus/docs/learning/dataengines/firewater/)、[VOFA命令与通道](https://www.vofa.plus/docs/learning/start/data_cmd_parameter/)、[波形绑定](https://www.vofa.plus/docs/learning/start/software_interface/)。


## 2026-09-16 修复版的检查方法

首次仍断开推进器动力，先只观察。VOFA 串口接收文字中发送 DIAG（十六进制 44 49 41 47 0D 0A）：
- US/MAX 是当前/累计最慢控制周期（微秒）；150Hz 预算约6667us。MISS持续增加表示需要先排查时序。
- T 为IMU温度（摄氏度）；HEAT=0表示未锁存温控故障，1表示非法温度或达到55°C，2表示预热180秒超时。HEAT不是“预热已经完成”。
- ACC_REJ/P_REJ是累计拒绝次数。偶发拒绝可能是正常保护；静止持续增长且READY=0，需保存读数检查方向/噪声/传感器。
- WHY=6表示软件拒绝当前反馈。启动早期可能留下6，即使之后READY=1也不自动清历史。不能只凭WHY非零判定当前坏了。
- BOOT=FAIL_*会重复打印故障设备；初始化失败时不启动调度器，此时STAT/DIAG不会有回复。记录重复文字，检查对应接线后在空气中重新上电。

温控目标仍45°C，55°C/180s是待本机验证的保护初值。姿态横滚/俯仰有效校正允许短时中断250ms，再久就不允许控制；只有单路压力不能延长这个时间。
压力跳变不再刷新比较基准。长时间丢样后真实深度变化很大时也可能持续拒绝；先停止、回到水面排查，不能水下重启消除报警。
PWM现在整组写入；写到一半收到完整OFF，会在本次写完成后补写八路中位。仍不能替代硬件动力急停。
ON包含原版固定浮力补偿，不能当“中位测试”。测静止PWM看STOP=1；单路动力检查按受控流程使用TES。
