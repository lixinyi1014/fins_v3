#include "IMU.h"
#include "TaskSharedResources.h"
#include "Sensor.h"
#include <string.h>
#include "FusedController.h"
#include "FusionConfiguration.h"
#include "VofaTelemetry.h"
#include <stdio.h>

// 控制周期协调：处理命令 → 请求水压 → 等姿态 → 计算控制量 → 等待 PWM 写出。

namespace lower_controller
{
namespace tasks
{
#if LC_IMU_ASYNC_ENABLED
namespace
{
FusedController fused_controller(GetFusedControlConfiguration());
bool vofa_logging = true; // 上电默认显示；停止/未就绪时也输出，便于排查故障。
void LogVofaState(const FusionState &state, bool frame_valid, const PressureArraySample &pressure,
                  uint32_t now, bool ready)
{
    static uint32_t previous_us = 0;
    static uint16_t sequence = 0;
    if (!vofa_logging || now - previous_us < LC_VOFA_PERIOD_US) return;
    previous_us = now;
    char packet[LC_UART_TX_PACKET_SIZE];
    const int length = FormatVofaTelemetry(packet, sizeof(packet), state, frame_valid, pressure, now,
        outputs_stopped != 0, ready && !safety_fault_latched, PressureSensor::pressure_sensor.StartupCalibrationOk(),
        safety_fault_latched != 0, ++sequence);
    if (length) LcSerial_Write(reinterpret_cast<const uint8_t *>(packet), length);
}
bool PacketEquals(const char *text, const char *command)
{
    const size_t length = strlen(command);
    if (strncmp(text, command, length))
        return false;
    text += length;
    while (*text == '\r' || *text == '\n' || *text == ' ')
        ++text;
    return !*text;
}
void ReplyMode(bool accepted, const char *reason)
{
    char text[96];
    int length = accepted ? snprintf(text, sizeof(text), "FB=OK\r\n")
                          : snprintf(text, sizeof(text), "FB=REJECT %s\r\n", reason);
    if (length > 0 && unsigned(length) < sizeof(text))
        LcSerial_Write(reinterpret_cast<const uint8_t *>(text), length);
}
} // namespace
#endif

PressurePwmReply SubmitPressurePwmRequest(const PressurePwmRequest &request)
{
    PressurePwmReply reply = {};
    if (safety_fault_latched) return reply;
    const TickType_t limit = pdMS_TO_TICKS(LC_BUS_REPLY_TIMEOUT_MS);
    if (xQueueSend(pressure_pwm_requests.handle, &request, limit) != pdPASS ||
        xQueueReceive(pressure_pwm_replies.handle, &reply, limit) != pdPASS)
    {
        ++controller_task_diagnostics.bus_reply_timeouts;
        FatalStop(StopBusReplyTimeout);
        // 回复可能稍后到达：本次启动永久禁止复用请求/回复链，等待 IWDG 复位，避免错配下一笔。
        return {};
    }
    return reply;
}
PressurePwmRequest BuildNeutralOutputRequest()
{
    PressurePwmRequest request = {};
    request.operation = PressurePwmOperation::Output;
    request.epoch = ReadStopEpoch(); // 附带停止代次；写出前与最新代次比较
    for (int i = 0; i < LC_THRUSTER_COUNT; ++i)
        request.thrusters.pulse_us[i] = LC_THRUSTER_NEUTRAL_US; // 8 路，1610 us
    request.servos = servos->BuildOutput(); // 4 路舵机 us 快照；停止只强制推进器中位，舵机保留原请求
    return request;
}
bool DispatchReceivedCommands(bool &arm_requested, uint32_t &arm_epoch)
{
    static uint32_t dispatched_epoch = 1;
    ReceivedCommandPacket packet;
    auto synchronize_stop = [&]()
    {
        uint32_t epoch = ReadStopEpoch();
        if (epoch == dispatched_epoch)
            return;
        vTaskSuspendAll();
        thrusters->StopControl(); // 停止代次变更时清除模式及 8 路旧请求，包括 TES 留下的值
        xTaskResumeAll();
        dispatched_epoch = epoch;
        arm_requested = false;
    };
    synchronize_stop();
    CheckCommandTimeout(LcTime_NowUs());
    if (arm_requested && LcTime_NowUs() - last_command_us >= LC_COMMAND_TIMEOUT_US)
        arm_requested = false; // 等待中位时过期的 ON，也不能被之后的 HB 重新激活。
    for (uint32_t n = 0;
         n < LC_UART_RX_QUEUE_LENGTH && xQueueReceive(received_commands.handle, &packet, 0) == pdPASS; ++n)
    { // 每轮最多取 8 包；0 表示不等待新命令
        synchronize_stop();
        if (packet.length == 2 && !memcmp(packet.data, "HB", 2))
        {
            if (!safety_fault_latched && LcTime_NowUs() - packet.received_us < LC_COMMAND_TIMEOUT_US)
            {
                taskENTER_CRITICAL();
                last_command_us = packet.received_us;
                have_command_heartbeat = 1;
                taskEXIT_CRITICAL();
            }
            continue; // 心跳只延长存活时间，绝不解除停止锁存。
        }
#if LC_IMU_ASYNC_ENABLED
        if (PacketEquals(packet.data, "FLOG:ON") || PacketEquals(packet.data, "FLOG:OFF") ||
            PacketEquals(packet.data, "VOFA:ON") || PacketEquals(packet.data, "VOFA:OFF"))
        {
            vofa_logging = PacketEquals(packet.data, "FLOG:ON") || PacketEquals(packet.data, "VOFA:ON");
            const char *message = vofa_logging ? "VOFA=ON FIREWATER_16CH\r\n" : "VOFA=OFF\r\n";
            LcSerial_Write(reinterpret_cast<const uint8_t *>(message), strlen(message));
            continue;
        }
        if (!strncmp(packet.data, "FB:", 3))
        {
            // 正式路径固定 ESKF；不再保留运行中切回旧控制/影子模式的入口。
            const bool accepted = PacketEquals(packet.data, "FB:ESKF");
            if (!accepted) ++controller_task_diagnostics.feedback_mode_rejections;
            ReplyMode(accepted, "ESKF_only");
            continue;
        }
        if (PacketEquals(packet.data, "DIAG")) {
            char status[LC_UART_TX_PACKET_SIZE];
            const int length = snprintf(status, sizeof(status),
                "DIAG=US=%lu,MAX=%lu,MISS=%lu,RX=%lu,I2C=%lu,ACC_REJ=%lu,P_REJ=%lu,T=%.1f,HEAT=%u,WHY=%lu\r\n",
                (unsigned long)controller_task_diagnostics.last_cycle_us,
                (unsigned long)controller_task_diagnostics.max_cycle_us,
                (unsigned long)controller_task_diagnostics.deadline_misses,
                (unsigned long)controller_task_diagnostics.rx_drops,
                (unsigned long)controller_task_diagnostics.i2c2_errors,
                (unsigned long)fusion_diagnostics.accel_rejected,
                (unsigned long)fusion_diagnostics.pressure_channel_rejected,
                double(IMU::imu.pro_data.temp), unsigned(IMU::imu.HeaterFault()),
                (unsigned long)controller_task_diagnostics.last_stop_reason);
            if (length > 0 && unsigned(length) < sizeof(status))
                LcSerial_Write(reinterpret_cast<const uint8_t *>(status), length);
            continue;
        }
        if (PacketEquals(packet.data, "STAT"))
        {
            const bool ready = PressureSensor::pressure_sensor.StartupCalibrationOk() &&
                !safety_fault_latched && !IMU::imu.HeaterFault() && FusedStateUsable(thrusters->fused_feedback, LcTime_NowUs(),
                                                       thrusters->BuildFusedControlRequest().yaw_enabled);
            char status[LC_UART_TX_PACKET_SIZE];
            const int length = snprintf(status, sizeof(status),
                "STAT=ESKF,STOP=%u,READY=%u,CAL=%u,FLAGS=%lu,MASK=%u,FAULT=%u,WHY=%lu,CAL_CH=%lu,CAL_N=%lu\r\n",
                unsigned(outputs_stopped != 0), unsigned(ready),
                unsigned(PressureSensor::pressure_sensor.StartupCalibrationOk()),
                static_cast<unsigned long>(thrusters->fused_feedback.flags),
                unsigned(thrusters->fused_feedback.pressure_mask), unsigned(safety_fault_latched != 0),
                static_cast<unsigned long>(controller_task_diagnostics.last_stop_reason),
                static_cast<unsigned long>(PressureSensor::pressure_sensor.startup_calibration.failed_channel),
                static_cast<unsigned long>(PressureSensor::pressure_sensor.startup_calibration.samples));
            if (length > 0 && unsigned(length) < sizeof(status))
                LcSerial_Write(reinterpret_cast<const uint8_t *>(status), length);
            continue;
        }
        // Every modifying command (including servo and FSET) must belong to the current stop epoch.
        if (!IsOffCommand(packet.data, packet.length) &&
            (packet.stop_epoch != ReadStopEpoch() ||
             LcTime_NowUs()-packet.received_us >= LC_COMMAND_TIMEOUT_US)) {
            ReplyMode(false, "stale_command");
            continue;
        }
        if (!strncmp(packet.data, "FSET:", 5))
        {
            bool accepted = thrusters->SetFusedTarget(packet.data + 5);
            ReplyMode(accepted, "target_or_mode");
            continue;
        }
#endif
        if (!ValidLegacyCommand(packet.data)) {
            ReplyMode(false, "invalid_command");
            continue;
        }
        if (IsOffCommand(packet.data, packet.length))
            arm_requested = false;
        if ((!strncmp(packet.data, "TES", 3) || !strncmp(packet.data, "ON", 2)) &&
            !IsArmCommand(packet.data, packet.length))
            continue; // 畸形启动包不能进入旧 atoi/strtok 分支，也不能刷新心跳。
        if (packet.length >= 2 && packet.data[0] == 'C' && packet.data[1] == 'A')
        {
            LatchOutputStop(StopCalibration);
            const char message[] = "CAL=RESTART_AT_SURFACE\r\n";
            LcSerial_Write(reinterpret_cast<const uint8_t *>(message), sizeof(message)-1);
            arm_requested = false;
            continue; // 当前产品仅上电自动标定；运行期不改参考、不用长阻塞校准绕开看门狗。
        }
        if (IsArmCommand(packet.data, packet.length))
        {
            if (packet.stop_epoch != ReadStopEpoch() || safety_fault_latched ||
                !PressureSensor::pressure_sensor.StartupCalibrationOk() ||
                LcTime_NowUs() - packet.received_us >= LC_COMMAND_TIMEOUT_US)
            {
                const char text[] = "CTRL=REJECT calibration_fault_or_stale_command\r\n";
                LcSerial_Write(reinterpret_cast<const uint8_t *>(text), sizeof(text)-1);
                continue;
            }
#if LC_IMU_ASYNC_ENABLED
            if (!FusedStateUsable(thrusters->fused_feedback, LcTime_NowUs(),
                                 thrusters->BuildFusedControlRequest().yaw_enabled))
            {
                const char text[] = "CTRL=REJECT feedback_not_ready USE_STAT\r\n";
                LcSerial_Write(reinterpret_cast<const uint8_t *>(text), sizeof(text)-1);
                continue;
            }
#endif
            taskENTER_CRITICAL();
            last_command_us = packet.received_us; // ON/TES 给首次启动一个 500 ms 心跳宽限期。
            have_command_heartbeat = 1;
            taskEXIT_CRITICAL();
            arm_requested = true;
            arm_epoch = packet.stop_epoch;
        }
        vTaskSuspendAll(); // 暂停任务切换以保护旧 strtok 解析；中断仍可接收命令
        for (uint32_t i = 0; i < device_count; ++i)
        {
            memcpy(RxBuffer, packet.data,
                   sizeof(RxBuffer));  // 复制 100 字节含终止符；避免上一模块 strtok 改写后续模块输入
            device_list[i]->Receive(); // 同一控制任务解析；每个模块收到完整的原始命令
        }
        memset(RxBuffer, 0, sizeof(RxBuffer));
        xTaskResumeAll();
    }
    return false; // 校准只在上电进行。
}
bool WaitForImuAttitude(const ControlRelease &release, ImuAttitudeFrame &frame)
{
    const TickType_t started = xTaskGetTickCount();
    const TickType_t limit = pdMS_TO_TICKS(LC_IMU_RESULT_TIMEOUT_MS); // 8 ms
    do
    {
        TickType_t elapsed = xTaskGetTickCount() - started;
        if (elapsed >= limit)
            return false;
        if (xQueueReceive(imu_attitude_frames.handle, &frame, limit - elapsed) != pdPASS)
            return false; // 旧帧被丢弃后只等待剩余 tick，不重新计满 8 ms
        if (SequenceAtOrAfter(frame.release.sequence, release.sequence))
            return frame.valid; // 仅接受本次或更新释放点的有效姿态帧
    } while (true);
}
void ControlLoopTask(void *)
{
    last_control_alive_us = LcTime_NowUs();
    LcSafetyHardware_Start();
    tasks_running = 1;
    configASSERT(HAL_UARTEx_ReceiveToIdle_IT(&huart6, uart_rx_buffer, sizeof(uart_rx_buffer)) ==
                 HAL_OK); // UART6：115200、8E1；100 字节独立接收缓存
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    configASSERT(HAL_TIM_Base_Start_IT(&htim1) == HAL_OK); // TIM1：150 Hz，只通知任务
    bool arm_requested = false;
    uint32_t arm_epoch = 0;
    for (;;)
    {
        uint32_t pending =
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 阻塞等 TIM1；返回累计次数，pdTRUE 一次清空计数
        ControlRelease release = ReadLatestControlRelease();
        if (pending > 1U)
        {
            controller_task_diagnostics.control_overruns += pending - 1U;
            LatchOutputStop(StopDeadline); // 不补跑积压控制周期
        }
        DispatchReceivedCommands(arm_requested, arm_epoch);

        PressurePwmRequest pressure_request = {};
        pressure_request.operation = PressurePwmOperation::Pressure;
        const PressurePwmReply pressure = SubmitPressurePwmRequest(
            pressure_request); // 推进一个水压步骤并取得回复；正常三步一帧，名义 50 Hz
        ImuAttitudeFrame imu = {};
        bool imu_ok =
            WaitForImuAttitude(release, imu); // 等待与当前释放序号匹配的姿态结果，最多 8 个 1 ms tick
        uint32_t now = LcTime_NowUs();
#if LC_IMU_ASYNC_ENABLED
        thrusters->fused_feedback = imu.fusion;
        const bool ready = PressureSensor::pressure_sensor.StartupCalibrationOk() &&
            !safety_fault_latched && !IMU::imu.HeaterFault() && pressure.ok && imu_ok &&
            FusedStateUsable(imu.fusion, now, thrusters->BuildFusedControlRequest().yaw_enabled);
        const bool feedback_failed = !ready;
#else
        const bool ready = false; // 新固件要求 DRDY/ESKF，不允许落入旧控制路径。
        const bool feedback_failed = true;
#endif
        if (feedback_failed)
        {
            LatchOutputStop(IMU::imu.HeaterFault() ? StopHeater : StopFeedback);
            arm_requested = false;
        }

        PressurePwmRequest output = BuildNeutralOutputRequest();
        if (ready)
        {
            if (arm_requested)
            {
                if (TryEnableOutputs(arm_epoch))
                {
                    arm_requested = false;
                    const char text[] = "CTRL=ARMED ESKF\r\n";
                    LcSerial_Write(reinterpret_cast<const uint8_t *>(text), sizeof(text)-1);
                }
                // 急停后需先收到整组中位写出成功，再放行本次新 ON；否则本轮继续写中位。
            }
            if (!LcRuntime_OutputsStopped(output.epoch))
            {
                output.allow_motion = true;
                output.released_us = release.released_us;
#if LC_IMU_ASYNC_ENABLED
                const auto result = fused_controller.Compute(imu.fusion, thrusters->BuildFusedControlRequest(), now);
                if (result.valid) output.thrusters = result.command;
                else
                {
                    LatchOutputStop(StopInvalidControl);
                    output.allow_motion = false;
                }
#endif
            }
        }
        if (LcRuntime_OutputsStopped(output.epoch))
        {
            ++controller_task_diagnostics.stopped_cycles;
#if LC_IMU_ASYNC_ENABLED
            fused_controller.Reset(); // OFF、超时、传感器失效后清积分；恢复时不携带旧环路记忆。
#endif
        }
        if (!SubmitPressurePwmRequest(output).ok)
            FatalStop(StopOutputWrite); // 输出写失败：禁止重启输出、不再喂狗；OE 接线后立即禁止 PWM。
        status_led->Handle();
        ++controller_task_diagnostics.control_cycles;
        uint32_t elapsed = LcTime_NowUs() - release.released_us;
        if (elapsed >= 1000000U / LC_CONTROL_HZ)
        {
            ++controller_task_diagnostics.deadline_misses;
            LatchOutputStop(StopDeadline);
            ulTaskNotifyTake(pdTRUE, 0); // 超过约 6667 us，清理已积压节拍后等下一次释放
        }
#if LC_IMU_ASYNC_ENABLED
        // PWM 写出和期限检查之后再取停止状态；故障时也给出带缺测标记的显示帧。
        LogVofaState(imu.fusion, imu_ok, pressure.physical_pressure, LcTime_NowUs(), ready);
#endif
        const uint32_t including_log_us = LcTime_NowUs() - release.released_us;
        if (elapsed < 1000000U / LC_CONTROL_HZ && including_log_us >= 1000000U / LC_CONTROL_HZ)
        {
            ++controller_task_diagnostics.deadline_misses;
            LatchOutputStop(StopDeadline);
            ulTaskNotifyTake(pdTRUE, 0);
        }
        elapsed = including_log_us;
        controller_task_diagnostics.last_cycle_us = elapsed;
        ControlCycleCompleted(LcTime_NowUs());
        if (elapsed > controller_task_diagnostics.max_cycle_us)
            controller_task_diagnostics.max_cycle_us = elapsed;
        controller_task_diagnostics.control_loop_stack_free =
            uxTaskGetStackHighWaterMark(NULL); // 单位：StackType_t 字，4 字节/字
    }
}

} // namespace tasks
} // namespace lower_controller
