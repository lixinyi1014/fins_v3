#include "TaskSharedResources.h"
#include "Sensor.h"

// 四任务共用的队列、设备指针、节拍快照和停止锁存；临界区保护仍保持原样。

namespace lower_controller
{
namespace tasks
{
#if LC_IMU_ASYNC_ENABLED
StaticQueue<RawImuPacket, 32> imu_raw_samples;
StaticQueue<PressureArraySample, 4> pressure_fusion_samples;
FusionState latest_fusion_state = {};
FusionDiagnostics fusion_diagnostics = {};
ImuAcquisitionDiagnostics imu_acquisition_diagnostics = {};
#endif

StaticQueue<ImuAttitudeFrame, 1> imu_attitude_frames;     // 1 帧姿态快照；新结果覆盖未消费的旧帧
StaticQueue<PressurePwmRequest, 1> pressure_pwm_requests; // 1 个请求；控制任务等待回复后才提交下一个
StaticQueue<PressurePwmReply, 1> pressure_pwm_replies;    // 1 份回复；按值交回压力反馈及事务状态
StaticQueue<ReceivedCommandPacket, LC_UART_RX_QUEUE_LENGTH>
    received_commands; // 8 包；每包最多 99 字节命令，另留 1 字节终止符
StaticQueue<UartTxPacket, LC_UART_TX_QUEUE_LENGTH>
    uart_tx_packets; // 16 包，每包最多 96 字节；队列持有数据副本
SemaphoreHandle_t imu_dma_done;
TaskHandle_t control_loop_task, imu_attitude_task, pressure_pwm_task, uart_transmit_task;
Device *const *device_list;
uint32_t device_count;
Propeller_I2C *thrusters;
Servo_I2C *servos;
LED *status_led;
volatile uint32_t tasks_running, outputs_stopped = 1, stop_epoch = 1;
volatile uint32_t control_release_sequence, control_release_us;
volatile uint32_t safety_fault_latched, last_control_alive_us, last_command_us;
volatile uint32_t have_command_heartbeat;
volatile uint32_t neutral_pwm_ready;
uint8_t uart_rx_buffer[100];

ControlRelease ReadLatestControlRelease()
{
    taskENTER_CRITICAL();
    ControlRelease release = {control_release_sequence,
                              control_release_us}; // 成对复制释放序号与 us 时间，避免 TIM1 更新到一半
    taskEXIT_CRITICAL();
    return release;
}
uint32_t ReadStopEpoch()
{
    taskENTER_CRITICAL();
    uint32_t value = stop_epoch;
    taskEXIT_CRITICAL();
    return value;
}
void LatchOutputStop(uint32_t reason)
{
    taskENTER_CRITICAL();
    if (!safety_fault_latched && (!outputs_stopped || !controller_task_diagnostics.last_stop_reason))
        controller_task_diagnostics.last_stop_reason = reason;
    if (!outputs_stopped)
    {
        outputs_stopped = 1;
        neutral_pwm_ready = 0;
        ++stop_epoch;
    } // 0→1 锁存停止；代次递增使先前输出请求失效
    LcSafetyHardware_SetOutputEnabled(0);
    taskEXIT_CRITICAL();
}
bool TryEnableOutputs(uint32_t epoch)
{
    bool enabled = false;
    taskENTER_CRITICAL();
    if (epoch == stop_epoch && neutral_pwm_ready && !safety_fault_latched && have_command_heartbeat &&
        LcTime_NowUs() - last_command_us < LC_COMMAND_TIMEOUT_US &&
        PressureSensor::pressure_sensor.StartupCalibrationOk())
    {
        outputs_stopped = 0;
        enabled = true;
        LcSafetyHardware_SetOutputEnabled(1); // 先前的中位 PWM 已写入，后续新请求才允许运动。
    }
    taskEXIT_CRITICAL();
    return enabled;
}

void SafetyStopFromISR(StopReason reason, bool fatal)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!outputs_stopped) { outputs_stopped = 1; neutral_pwm_ready = 0; ++stop_epoch; }
    if (!safety_fault_latched) controller_task_diagnostics.last_stop_reason = reason;
    if (fatal && !safety_fault_latched) { safety_fault_latched = 1; ++controller_task_diagnostics.safety_faults; }
    LcSafetyHardware_SetOutputEnabled(0);
    __set_PRIMASK(mask);
}
void FatalStop(StopReason reason) { SafetyStopFromISR(reason, true); }
void CheckCommandTimeout(uint32_t now)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!outputs_stopped && (!have_command_heartbeat || now - last_command_us >= LC_COMMAND_TIMEOUT_US))
    {
        ++controller_task_diagnostics.command_timeouts;
        SafetyStopFromISR(StopCommandTimeout, false);
    }
    __set_PRIMASK(mask);
}
void ControlCycleCompleted(uint32_t now)
{
    last_control_alive_us = now;
    if (!safety_fault_latched) LcSafetyHardware_Feed();
}

} // namespace tasks
} // namespace lower_controller

using namespace lower_controller::tasks;

volatile ControllerTaskDiagnostics controller_task_diagnostics = {};

extern "C" int LcRuntime_IsRunning(void)
{
    return tasks_running != 0;
}
extern "C" int LcRuntime_OutputsStopped(uint32_t epoch)
{
    return outputs_stopped || epoch != stop_epoch;
}
