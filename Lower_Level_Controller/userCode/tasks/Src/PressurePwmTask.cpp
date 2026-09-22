#include "TaskSharedResources.h"
#include "Sensor.h"
#include "I2cBusAccess.h"
#include "LegacyEstimation.h"
#include <string.h>

// I2C2 水压与 PWM 服务：按请求完成水压步骤、校准或推进器/舵机写出，再交回回复。

namespace lower_controller
{
namespace tasks
{
namespace
{
/* PCA9685 自己保持各通道寄存器，不需要每个 150 Hz 周期把 12 路原样重写一遍。
 * 原先每周期逐通道写 12 次（约 3~4 ms），加上水压一步（约 3 ms）超过 6.67 ms 控制周期，
 * 频繁触发 StopDeadline，解锁后推进器会被反复打停。这里只写有变化的通道，
 * 另外每周期轮流多刷新 1 路（12 个周期一轮，约 12.5 Hz），不再一次性全量重写造成 4 ms 尖峰。
 * 只有影子失效（启动或写失败）或 I2C2 出过错时才全量重写。 */
uint16_t pwm_shadow[12];
bool pwm_shadow_valid = false;
uint32_t pwm_refresh_cursor = 0, pwm_last_bus_errors = 0;
/** 写 mask 中标记的通道；成功后更新影子，失败则作废影子以强制下次全量重写。 */
bool WritePwmChannels(const uint16_t counts[12], uint16_t mask)
{
    if (!mask) return true;
    TCA_SetChannel(LC_PWM_MUX_CHANNEL); // 4；选通与写入属于同一个总线请求
    for (unsigned i = 0; i < 12; ++i)
    {
        if (!(mask & (1U << i))) continue;
        if (!PCA_WriteGroup(i, &counts[i], 1) || !LcBus_Ok(&hi2c2))
        {
            pwm_shadow_valid = false;
            return false;
        }
        pwm_shadow[i] = counts[i];
    }
    return true;
}
} // namespace

PressurePwmReply ProcessPressurePwmRequest(const PressurePwmRequest &request,
                                           PressurePublication &publication)
{
    PressurePwmReply reply = {};
    LcBus_Begin(&hi2c2, request.operation == PressurePwmOperation::Calibrate
                            ? LC_CALIBRATION_BUDGET_US
                            : LC_BUS_BUDGET_US); // 8 s 校准 / 当前配置的整组压力事务预算
    if (request.operation == PressurePwmOperation::Pressure)
    {
        PressureSensor::pressure_sensor.Handle(); // 总线任务内先 Acquire 再 Estimate；一帧只滤波一次
#if LC_IMU_ASYNC_ENABLED
        const auto &physical = PressureSensor::pressure_sensor.PhysicalMeasurement();
        static uint32_t published_physical = 0;
        if (physical.stamp.sequence != published_physical)
        {
            published_physical = physical.stamp.sequence;
            if (xQueueSend(pressure_fusion_samples.handle, &physical, 0) != pdPASS)
                ++controller_task_diagnostics.pressure_sample_drops;
        }
#endif
        const auto &measurement = PressureSensor::pressure_sensor.LastMeasurement();
        if (measurement.sequence != publication.sequence)
        {
            publication.sequence = measurement.sequence;
            publication.completed_us = LcTime_NowUs();
        }
    }
    else if (request.operation == PressurePwmOperation::Calibrate)
    {
        if (!PressureSensor::pressure_sensor.PromValid())
            return reply; // 四路 PROM 标志须全有效，否则回复 ok=false
        float saved_offsets[SENSOR_NUM];
        memcpy(saved_offsets, PressureSensor::pressure_sensor.data_pressure_offset, sizeof(saved_offsets));
        PressureSensor::pressure_sensor.flag_calibrate = true;
        PressureSensor::pressure_sensor.Acquire();
#if LC_IMU_ASYNC_ENABLED
        PressureSensor::pressure_sensor.ResetTimedAcquisition();
#endif
        PressureSensor::pressure_sensor.ps_state =
            PS_HANDLE_STATE::GET_TEMPERATURE; // 校准后重新发起温度转换，避免续用校准前的 ADC 阶段
        if (!LcBus_Ok(&hi2c2))
            memcpy(PressureSensor::pressure_sensor.data_pressure_offset, saved_offsets,
                   sizeof(saved_offsets)); // 失败恢复四路旧偏置，不提交部分校准结果
    }
    else
    {
        if (request.allow_motion && LcTime_NowUs()-request.released_us >= 1000000U/LC_CONTROL_HZ)
            LatchOutputStop(StopDeadline);
        const bool stopped = !request.allow_motion || LcRuntime_OutputsStopped(request.epoch);
        uint16_t counts[12] = {};
        bool all_neutral = true;
        for (unsigned i=0; i<LC_THRUSTER_COUNT; ++i) {
            const int32_t pulse = stopped ? LC_THRUSTER_NEUTRAL_US : request.thrusters.pulse_us[i];
            all_neutral = all_neutral && pulse == LC_THRUSTER_NEUTRAL_US;
            counts[i] = LegacyThrusterPwmCount(pulse);
        }
        if (!request.servos.in_range) return reply;
        uint16_t servo_mask = 0;
        for (unsigned i=0; i<4; ++i) {
            const unsigned channel = request.servos.channel[i];
            const int32_t pulse = request.servos.pulse_us[i];
            if (channel < 8 || channel > 11 || (servo_mask & (1U<<channel)) ||
                pulse < LC_SERVO_MIN_US || pulse > LC_SERVO_MAX_US) return reply;
            servo_mask |= 1U<<channel;
            counts[channel] = pulse * LC_PWM_COUNTS / LC_PWM_PERIOD_US; // Preserve servo integer conversion.
        }
        const uint32_t bus_errors = LcBus_ErrorCount(&hi2c2);
        const bool full = !pwm_shadow_valid || bus_errors != pwm_last_bus_errors;
        uint16_t dirty = uint16_t(1U << (pwm_refresh_cursor % 12U)); // 轮流刷新 1 路
        for (unsigned i = 0; i < 12; ++i)
            if (full || counts[i] != pwm_shadow[i]) dirty |= uint16_t(1U << i);
        if (!WritePwmChannels(counts, dirty)) return reply;
        ++pwm_refresh_cursor;
        if (full)
        {
            pwm_shadow_valid = true;
            pwm_last_bus_errors = bus_errors;
        }
        if (request.allow_motion && LcTime_NowUs()-request.released_us >= 1000000U/LC_CONTROL_HZ)
            LatchOutputStop(StopDeadline);
        // OFF can interrupt the HAL transfer. On completion recheck and immediately replace
        // the entire thruster group, instead of waiting for another control cycle.
        if (!all_neutral && LcRuntime_OutputsStopped(request.epoch)) {
            for (unsigned i=0; i<LC_THRUSTER_COUNT; ++i)
                counts[i] = LegacyThrusterPwmCount(LC_THRUSTER_NEUTRAL_US);
            if (!WritePwmChannels(counts, uint16_t((1U << LC_THRUSTER_COUNT) - 1U))) return reply;
            all_neutral = true;
        }
        if (all_neutral && LcBus_Ok(&hi2c2)) neutral_pwm_ready = 1;
    }
    reply.pressure = PressureSensor::pressure_sensor.Feedback();
#if LC_IMU_ASYNC_ENABLED
    reply.physical_pressure = PressureSensor::pressure_sensor.PhysicalMeasurement();
#endif
    reply.completed_us = publication.completed_us; // 上一完整水压帧完成时间；中间转换步骤不刷新它
    reply.pressure_valid =
        PressureSensor::pressure_sensor.PromValid(); // 仅代表四路 PROM 标志有效，不是物理测量合理性检查
#if LC_IMU_ASYNC_ENABLED
    reply.pressure_valid =
        reply.pressure_valid && PressureSensor::pressure_sensor.LegacyFrameValid(); // 旧闭环必须完整四路。
#endif
    reply.ok = LcBus_Ok(&hi2c2) != 0; // 仅代表本请求未记录总线失败，不代表设备已实际响应运动
    return reply;
}
void PressurePwmTask(void *)
{
    PressurePublication publication = {};
    for (;;)
    {
        PressurePwmRequest request;
        configASSERT(xQueueReceive(pressure_pwm_requests.handle, &request, portMAX_DELAY) ==
                     pdPASS); // 无请求时阻塞；每次取出独立的请求副本
        const uint32_t started = LcTime_NowUs();
        PressurePwmReply reply = ProcessPressurePwmRequest(request, publication);
        const uint32_t took = LcTime_NowUs() - started;
        volatile uint32_t &slot = request.operation == PressurePwmOperation::Output
                                      ? timing_diagnostics.bus_output_max
                                      : timing_diagnostics.bus_pressure_max;
        if (took > slot) slot = took; // 总线任务实际处理耗时(含被抢占时间)
        xQueueSend(pressure_pwm_replies.handle, &reply, portMAX_DELAY);
        controller_task_diagnostics.i2c2_errors = LcBus_ErrorCount(&hi2c2);
        controller_task_diagnostics.pressure_pwm_stack_free = uxTaskGetStackHighWaterMark(NULL);
    }
}

} // namespace tasks
} // namespace lower_controller
