#include "TaskSharedResources.h"
#include "I2cBusAccess.h"
#include "ImuDmaAcquisition.h"

// 启动入口：准备静态内存 → 创建四任务与通信对象 → 指定 I2C 所属任务 → 启动调度器。

using namespace lower_controller;
using namespace lower_controller::tasks;

namespace
{
StaticSemaphore_t imu_dma_done_storage;
StaticTask_t control_loop_tcb, imu_attitude_tcb, pressure_pwm_tcb, uart_transmit_tcb, idle_tcb;
StackType_t control_loop_stack[1536], imu_attitude_stack[2048], pressure_pwm_stack[1536],
    uart_transmit_stack[512], idle_stack[256];
} // namespace

void StartControllerTasks(Device *const *devices, uint32_t count, Propeller_I2C *propeller, Servo_I2C *servo,
                          LED *led)
{
    device_list = devices;
    device_count = count;
    thrusters = propeller;
    servos = servo;
    status_led = led;
    LcTime_Start(); // 启动 TIM2：1 MHz 计数，每计数 1 us
    imu_attitude_frames.Create();
    pressure_pwm_requests.Create();
    pressure_pwm_replies.Create();
    received_commands.Create();
    uart_tx_packets.Create();
#if LC_IMU_ASYNC_ENABLED
    imu_raw_samples.Create();
    pressure_fusion_samples.Create();
    StartImuDmaAcquisition(
        PublishImuPacket); // 队列建立后接入 DRDY；调度器启动前中断仍受 tasks_running 门控。
#endif
    imu_dma_done =
        xSemaphoreCreateBinaryStatic(&imu_dma_done_storage); // 初始为空的二值信号量；只表示 DMA 完成或错误
    configASSERT(imu_dma_done);
    control_loop_task = xTaskCreateStatic(ControlLoopTask, "control_loop", 1536, NULL, 4, control_loop_stack,
                                          &control_loop_tcb); // 6144 字节栈，优先级 4
#if LC_IMU_ASYNC_ENABLED
    imu_attitude_task = xTaskCreateStatic(ImuFusionTask, "imu_fusion", 2048, NULL, 5, imu_attitude_stack,
                                          &imu_attitude_tcb); // 8192 字节，优先级 5。
#else
    imu_attitude_task = xTaskCreateStatic(ImuAttitudeTask, "imu_attitude", 2048, NULL, 5, imu_attitude_stack,
                                          &imu_attitude_tcb); // 上一阶段回归模式。
#endif
    pressure_pwm_task = xTaskCreateStatic(PressurePwmTask, "pressure_pwm", 1536, NULL, 3, pressure_pwm_stack,
                                          &pressure_pwm_tcb); // 6144 字节栈，优先级 3
    uart_transmit_task = xTaskCreateStatic(UartTransmitTask, "uart_tx", 512, NULL, 2, uart_transmit_stack,
                                           &uart_transmit_tcb); // 2048 字节栈，优先级 2
    configASSERT(control_loop_task && imu_attitude_task && pressure_pwm_task && uart_transmit_task);
    LcBus_SetOwner(&hi2c2, pressure_pwm_task); // I2C2 仅总线任务可访问：水压 TCA 0..3，PWM TCA 4
    LcBus_SetOwner(&hi2c3, imu_attitude_task); // I2C3 仅 IMU 任务可访问：IST8310 磁力计
    vTaskStartScheduler();                     // 按任务优先级开始调度；正常情况下不返回 main
    LcRuntime_Assert(__FILE__, __LINE__);
}
extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *depth)
{
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = 256;
}
namespace
{
/* Crash report over USART6 by register polling. 死机现场上报：不依赖 RTOS/HAL/中断，
 * 直接轮询 USART6 寄存器（已按 115200 8E1 初始化）。每秒重复一次，串口晚连上也能看到。 */
void CrashPutc(char c)
{
    for (uint32_t guard = 0; !(USART6->SR & USART_SR_TXE) && guard < 200000U; ++guard)
    {
    }
    USART6->DR = static_cast<uint8_t>(c);
}
void CrashPuts(const char *s)
{
    while (s && *s)
        CrashPutc(*s++);
}
void CrashHex(uint32_t v)
{
    CrashPuts("0x");
    for (int i = 7; i >= 0; --i)
        CrashPutc("0123456789ABCDEF"[(v >> (i * 4)) & 0xFU]);
}
void CrashDec(int32_t v)
{
    char b[12];
    int n = 0;
    uint32_t u = v < 0 ? uint32_t(-v) : uint32_t(v);
    do { b[n++] = char('0' + u % 10U); u /= 10U; } while (u);
    if (v < 0) CrashPutc('-');
    while (n) CrashPutc(b[--n]);
}
void CrashPrepareUart()
{
    // 停掉中断驱动的收发，避免 HAL 回调与轮询输出冲突。
    USART6->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE);
    USART6->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE);
}
void CrashDelay()
{
    for (volatile uint32_t i = 0; i < 12000000U; ++i)
    {
    } // 约 1 s（168 MHz，未启用缓存假设下的粗略值）
}
const char *CurrentTaskName()
{
    return tasks_running ? pcTaskGetName(NULL) : "boot";
}
} // namespace
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *name)
{
    FatalStop(StopAssertion);
    __disable_irq();
    controller_task_diagnostics.assert_file = "stack overflow";
    controller_task_diagnostics.assert_line = 0;
    CrashPrepareUart();
    for (;;)
    {
        CrashPuts("\r\nCRASH=STACK_OVERFLOW TASK=");
        CrashPuts(name);
        CrashPuts("\r\n");
        CrashDelay();
    }
}
extern "C" void LcRuntime_Assert(const char *file, int line)
{
    FatalStop(StopAssertion);
    __disable_irq();
    controller_task_diagnostics.assert_file = file;
    controller_task_diagnostics.assert_line = line;
    const char *task = CurrentTaskName();
    CrashPrepareUart();
    for (;;)
    {
        CrashPuts("\r\nCRASH=ASSERT FILE=");
        CrashPuts(file);
        CrashPuts(" LINE=");
        CrashDec(line);
        CrashPuts(" TASK=");
        CrashPuts(task);
        CrashPuts("\r\n");
        CrashDelay();
    }
}
// Called first thing from HardFault_Handler with the EXC_RETURN value captured there.
extern "C" void LcCrash_HardFault(uint32_t exc_return)
{
    __disable_irq();
    // EXC_RETURN bit2=1：出错时正在任务中(PSP)，栈帧[5]=LR、[6]=PC 即出错指令地址。
    // bit2=0：出错发生在中断里(MSP)，此处只能给出被打断任务的 PSP 帧和故障寄存器。
    const bool from_task = (exc_return & 4U) != 0;
    const uint32_t *frame = reinterpret_cast<const uint32_t *>(__get_PSP());
    const uint32_t pc = frame[6], lr = frame[5];
    const uint32_t cfsr = SCB->CFSR, hfsr = SCB->HFSR, bfar = SCB->BFAR, mmfar = SCB->MMFAR;
    const char *task = CurrentTaskName();
    CrashPrepareUart();
    for (;;)
    {
        CrashPuts("\r\nCRASH=HARDFAULT IN=");
        CrashPuts(from_task ? "TASK" : "ISR");
        CrashPuts(" TASK=");
        CrashPuts(task);
        CrashPuts(" PC=");
        CrashHex(pc);
        CrashPuts(" LR=");
        CrashHex(lr);
        CrashPuts(" CFSR=");
        CrashHex(cfsr);
        CrashPuts(" HFSR=");
        CrashHex(hfsr);
        CrashPuts(" BFAR=");
        CrashHex(bfar);
        CrashPuts(" MMFAR=");
        CrashHex(mmfar);
        CrashPuts(" EXC=");
        CrashHex(exc_return);
        CrashPuts("\r\n");
        CrashDelay();
    }
}
