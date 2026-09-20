# 四个任务怎样协作

默认工程入口是 `userCode/devices/Src/Usermain.cpp::main()`，初始化后调用
`ControllerTaskStartup.cpp::StartControllerTasks()`。这里仍然只创建四个业务任务。

| 当前任务文件与函数 | 做什么 | 等待及交付 | 优先级 / 栈 |
|---|---|---|---|
| `Src/ImuFusionTask.cpp::ImuFusionTask()` | 解码异步样本、温控、ESKF | 等原始帧队列；按采样时间处理观测；每个控制释放点交付一致姿态快照 | 5 / 2048 字 = 8192 B |
| `Src/ControlLoopTask.cpp::ControlLoopTask()` | 解析命令、检查反馈、运行新 SI 控制器、组织输出和遥测 | 等 TIM1 150 Hz 通知；水压请求→姿态结果→控制计算→输出请求→回复 | 4 / 1536 字 = 6144 B |
| `Src/PressurePwmTask.cpp::PressurePwmTask()` | 独占 I2C2，执行水压步骤、推进器/舵机写出 | 等请求队列；水压新帧另送估计任务，事务结果回复控制任务 | 3 / 1536 字 = 6144 B |
| `Src/UartTransmitTask.cpp::UartTransmitTask()` | 串口异步发送和超时处理 | 等发送队列；等待 UART6 完成/错误通知后释放当前包 | 2 / 512 字 = 2048 B |

优先级数值越大越优先，栈的 1 字是 4 字节。水压和 PWM 共用 I2C2/TCA，因此保留一个服务任务。
当前要求 LC_IMU_ASYNC_ENABLED=1，默认只创建 ImuFusionTask。

| 辅助文件 | 含义 |
|---|---|
| `ControllerTaskStartup.cpp` | 静态内存、任务/队列创建、I2C 所有者、调度器及断言钩子 |
| `ImuSampleInterrupts.cpp` | PC5/PC4/PG3 的 DRDY 分发，DMA 原始帧按值入队；没有矩阵解算 |
| `TaskInterruptHandlers.cpp` | TIM1 控制通知和温控请求，UART6 收包、OFF 锁存、发送通知 |
| `TaskSharedState.cpp` | 队列实例、停止代次、控制释放时间、调试状态快照 |
| `TaskSharedResources.h` | 以上对象的任务内部声明；不是新的执行线程 |
| `I2cBusAccess.cpp` | 总线所有权、事务间预算检查、HAL 超时和错误计数 |

继续阅读：[当前运行框架](../../docs/FREERTOS_RUNTIME.md)、[首次上电](../../docs/BOARD_BRINGUP.md)。
