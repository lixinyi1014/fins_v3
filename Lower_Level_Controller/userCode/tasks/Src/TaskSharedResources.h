#ifndef TASK_SHARED_RESOURCES_H
#define TASK_SHARED_RESOURCES_H
#include "ControllerTasks.h"
#include "TaskMessages.h"
#include "ImuDmaArbiter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 仅 tasks/Src 内部使用；队列/状态在 TaskSharedState.cpp 中各定义一次，任务栈在启动文件中。
namespace lower_controller
{
namespace tasks
{

template <class T, size_t N> struct StaticQueue
{
    StaticQueue_t control;
    uint8_t storage[sizeof(T) * N];
    QueueHandle_t handle;
    void Create()
    {
        handle = xQueueCreateStatic(N, sizeof(T), storage, &control);
        configASSERT(handle);
    }
};

extern StaticQueue<ImuAttitudeFrame, 1> imu_attitude_frames;
extern StaticQueue<PressurePwmRequest, 1> pressure_pwm_requests;
extern StaticQueue<PressurePwmReply, 1> pressure_pwm_replies;
extern StaticQueue<ReceivedCommandPacket, LC_UART_RX_QUEUE_LENGTH> received_commands;
extern StaticQueue<UartTxPacket, LC_UART_TX_QUEUE_LENGTH> uart_tx_packets;
#if LC_IMU_ASYNC_ENABLED
extern StaticQueue<RawImuPacket, 32> imu_raw_samples;
extern StaticQueue<PressureArraySample, 4> pressure_fusion_samples;
extern FusionState latest_fusion_state; // 由 IMU 任务写，调试器查看；控制使用队列中的一致副本。
extern FusionDiagnostics fusion_diagnostics;
extern ImuAcquisitionDiagnostics imu_acquisition_diagnostics;
bool PublishImuPacket(const RawImuPacket &packet, bool from_isr);
#endif
extern SemaphoreHandle_t imu_dma_done;
extern TaskHandle_t control_loop_task, imu_attitude_task, pressure_pwm_task, uart_transmit_task;
extern Device *const *device_list;
extern uint32_t device_count;
extern Propeller_I2C *thrusters;
extern Servo_I2C *servos;
extern LED *status_led;
extern volatile uint32_t tasks_running, outputs_stopped, stop_epoch;
extern volatile uint32_t control_release_sequence, control_release_us;
extern uint8_t uart_rx_buffer[100];

ControlRelease ReadLatestControlRelease();
uint32_t ReadStopEpoch();
void LatchOutputStop(uint32_t reason = 6);
bool TryEnableOutputs(uint32_t epoch);
enum StopReason { StopCommandTimeout = 1, StopControlStall = 2, StopBusReplyTimeout = 3, StopAssertion = 4, StopOutputWrite = 5, StopFeedback = 6, StopDeadline = 7, StopInvalidControl = 8, StopUserOff = 9, StopCalibration = 10, StopHeater = 11 };
extern volatile uint32_t neutral_pwm_ready;
extern volatile uint32_t safety_fault_latched, last_control_alive_us, last_command_us;
extern volatile uint32_t have_command_heartbeat;
void CheckCommandTimeout(uint32_t now);
void SafetyStopFromISR(StopReason reason, bool fatal);
void FatalStop(StopReason reason);
void ControlCycleCompleted(uint32_t now);

void ImuAttitudeTask(void *argument); // LC_IMU_ASYNC_ENABLED=0 时的上一阶段任务
void ImuFusionTask(void *argument);   // 默认任务：异步解码与 ESKF
void ControlLoopTask(void *argument);
void PressurePwmTask(void *argument);
void UartTransmitTask(void *argument);

// 任务处理步骤也供主机回归测试直接调用；设备层只使用 Inc 中的公开接口。
struct PressurePublication
{
    uint32_t completed_us, sequence;
};
PressurePwmReply ProcessPressurePwmRequest(const PressurePwmRequest &request,
                                           PressurePublication &publication);
PressurePwmRequest BuildNeutralOutputRequest();
PressurePwmReply SubmitPressurePwmRequest(const PressurePwmRequest &request);
bool DispatchReceivedCommands(bool &arm_requested, uint32_t &arm_epoch);
bool WaitForImuAttitude(const ControlRelease &release, ImuAttitudeFrame &frame);

} // namespace tasks
} // namespace lower_controller
#endif
