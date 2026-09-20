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

PressurePwmReply ProcessPressurePwmRequest(const PressurePwmRequest &request,
                                           PressurePublication &publication)
{
    PressurePwmReply reply = {};
    LcBus_Begin(&hi2c2, request.operation == PressurePwmOperation::Calibrate
                            ? LC_CALIBRATION_BUDGET_US
                            : LC_BUS_BUDGET_US); // 8 s 校准 / 5000 us 普通请求
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
        TCA_SetChannel(LC_PWM_MUX_CHANNEL); // 4；选通与整组读写属于同一个总线请求
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
        if (!PCA_WriteGroup(0, counts, 12)) return reply;
        if (request.allow_motion && LcTime_NowUs()-request.released_us >= 1000000U/LC_CONTROL_HZ)
            LatchOutputStop(StopDeadline);
        // OFF can interrupt the HAL transfer. On completion recheck and immediately replace
        // the entire thruster group, instead of waiting for another control cycle.
        if (!all_neutral && LcRuntime_OutputsStopped(request.epoch)) {
            for (unsigned i=0; i<LC_THRUSTER_COUNT; ++i)
                counts[i] = LegacyThrusterPwmCount(LC_THRUSTER_NEUTRAL_US);
            if (!PCA_WriteGroup(0, counts, LC_THRUSTER_COUNT)) return reply;
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
        PressurePwmReply reply = ProcessPressurePwmRequest(request, publication);
        xQueueSend(pressure_pwm_replies.handle, &reply, portMAX_DELAY);
        controller_task_diagnostics.i2c2_errors = LcBus_ErrorCount(&hi2c2);
        controller_task_diagnostics.pressure_pwm_stack_free = uxTaskGetStackHighWaterMark(NULL);
    }
}

} // namespace tasks
} // namespace lower_controller
