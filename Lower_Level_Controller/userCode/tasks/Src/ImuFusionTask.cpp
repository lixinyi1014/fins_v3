#include "TaskSharedResources.h"
#if LC_IMU_ASYNC_ENABLED
#include "ImuDmaAcquisition.h"
#include "FusionTimeline.h"
#include "FusionMath.h"
#include "IMU.h"
#include "I2cBusAccess.h"
#include <string.h>

namespace lower_controller
{
namespace tasks
{
volatile ImuFrameDiagnostics imu_frame_diagnostics = {};
namespace
{
FusionTimeline timeline(GetFusionConfiguration()); // 唯一拥有 ESKF 状态的对象，固定容量 64 条。
ImuMeasurement calibrated_sample = {};
uint32_t gyro_time, accel_time, mag_time, temperature_time;
bool have_gyro, have_accel, have_mag;

/** @brief Decode one complete sample in task context. 在任务中解码一份完整样本。
 * DRDY 时刻与 DMA 完成时刻分别保留；标定后的数据只交给当前 ESKF。 */
void DecodeImuPacket(const RawImuPacket &packet)
{
    const auto &config = GetFusionConfiguration();
    FusionObservation observation = {};
    observation.kind = packet.kind;
    observation.stamp = packet.stamp;
    float decoded[3] = {};
    switch (packet.kind)
    {
    case SensorKind::Gyroscope: ++imu_frame_diagnostics.gyro_packets; break;
    case SensorKind::Accelerometer: ++imu_frame_diagnostics.accel_packets; break;
    case SensorKind::Magnetometer: ++imu_frame_diagnostics.mag_packets; break;
    case SensorKind::Temperature: ++imu_frame_diagnostics.temperature_packets; break;
    default: break;
    }
    if (packet.kind == SensorKind::Gyroscope)
    {
        BMI088_gyro_read_over(const_cast<uint8_t *>(packet.bytes + 1),
                              decoded); // 首字节是 SPI 命令回读，轴值从 +1 开始。
        for (unsigned i = 0; i < 3; ++i)
            calibrated_sample.gyro_rad_s[i] = decoded[i] - IMU::imu.raw_data.gyro_offset[i];
        memcpy(IMU::imu.raw_data.gyro, decoded, sizeof(decoded));
        memcpy(IMU::imu.pro_data.gyro, calibrated_sample.gyro_rad_s,
               sizeof(decoded)); // 兼容调试字段仍为传感器轴、rad/s。
        fusion_math::Rotate(config.imu_to_body, calibrated_sample.gyro_rad_s, observation.data.vector);
        gyro_time = packet.stamp.sample_us;
        have_gyro = true;
    }
    else if (packet.kind == SensorKind::Accelerometer)
    {
        BMI088_accel_read_over(const_cast<uint8_t *>(packet.bytes + 2), decoded,
                               &IMU::imu.raw_data.time); // +2 跳过命令及 dummy；单位 m/s²。
        for (unsigned i = 0; i < 3; ++i)
            calibrated_sample.accel_m_s2[i] = decoded[i] - IMU::imu.raw_data.accel_offset[i];
        memcpy(IMU::imu.raw_data.accel, decoded, sizeof(decoded));
        memcpy(IMU::imu.pro_data.accel, calibrated_sample.accel_m_s2,
               sizeof(decoded)); // 保留旧符号；ESKF 在下方单独转为 FRD 向下观测。
        fusion_math::Rotate(config.imu_to_body, calibrated_sample.accel_m_s2, observation.data.vector);
        for (unsigned i = 0; i < 3; ++i)
            observation.data.vector[i] *= config.accel_to_down_sign; // 比力转向下重力观测，当前暂定 -1。
        accel_time = packet.stamp.sample_us;
        have_accel = true;
    }
    else if (packet.kind == SensorKind::Temperature)
    {
        BMI088_temperature_read_over(const_cast<uint8_t *>(packet.bytes + 2), &calibrated_sample.temperature_c);
        IMU::imu.UpdateTemperature(calibrated_sample.temperature_c);
        if (isfinite(calibrated_sample.temperature_c) && calibrated_sample.temperature_c >= -40 &&
            calibrated_sample.temperature_c <= 85)
        {
            IMU::imu.raw_data.temp = IMU::imu.pro_data.temp = calibrated_sample.temperature_c;
            temperature_time = packet.stamp.sample_us;
        } // 名义 150 Hz；沿用原温控 PID。
        return; // 温度只驱动加热，不是姿态观测。
    }
    else if (packet.kind == SensorKind::Magnetometer)
    {
        if (packet.stamp.sequence != MagnetometerSequence() ||
            LcTime_NowUs() - packet.stamp.sample_us > 3000U)
        {
            ++controller_task_diagnostics.mag_stale_samples;
            return;
        }
        uint8_t bytes[7] = {};
        LcBus_Begin(&hi2c3, 2000U); // 磁力计专用 I2C3；本次读取预算 2 ms。
        if (LcBus_MemRead(&hi2c3, 0x1c, 0x02, I2C_MEMADD_SIZE_8BIT, bytes, 7, 1) != HAL_OK ||
            !(bytes[0] & 1U) || packet.stamp.sequence != MagnetometerSequence())
        {
            ++controller_task_diagnostics.mag_bus_rejections;
            return;
        }
        // 状态 + 三轴连续读取；传输期间出现新 DRDY 时丢弃，避免旧时间戳配上新寄存器值。
        ist8310_real_data_t magnetic = {};
        ist8310_read_over(bytes, &magnetic);
        const float offsets[3] = {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z};
        const float scales[3] = {MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z};
        for (unsigned i = 0; i < 3; ++i)
            calibrated_sample.mag_uT[i] = (magnetic.mag[i] - offsets[i]) * scales[i];
        memcpy(IMU::imu.raw_data.mag, magnetic.mag, sizeof(magnetic.mag));
        memcpy(IMU::imu.pro_data.mag, calibrated_sample.mag_uT, sizeof(magnetic.mag)); // uT。
        fusion_math::Rotate(config.mag_to_body, calibrated_sample.mag_uT, observation.data.vector);
        observation.stamp.received_us = LcTime_NowUs();
        mag_time = packet.stamp.sample_us;
        have_mag = true;
    }
    else
        return;
    timeline.Push(observation); // 每份观测只入队一次，按采样代理时间排序，不复用上一磁场作新观测。
}
} // namespace

void ImuFusionTask(void *)
{
    uint32_t previous_release = 0;
    uint32_t load_window_start = LcTime_NowUs(), load_busy_us = 0, stack_check_releases = 0;
    for (;;)
    {
        RawImuPacket packet;
        const bool received = xQueueReceive(imu_raw_samples.handle, &packet, pdMS_TO_TICKS(1)) == pdPASS;
        const uint32_t busy_start = LcTime_NowUs(); // 从取到包到本轮结束的时间，用于估算本任务负载。
        if (received)
            DecodeImuPacket(packet);
        // 最多阻塞 1 ms：即使 DRDY 消失，也要推进超时处理和发布无效状态。
        uint32_t now = LcTime_NowUs();
        PollImuDma(now);
        if (now - temperature_time > 100000U)
        {
            __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 0);
            ++controller_task_diagnostics.heater_stale_cycles; // 100 ms 无有效温度则关闭加热。
        }
        PressureArraySample pressure;
        while (xQueueReceive(pressure_fusion_samples.handle, &pressure, 0) == pdPASS)
        {
            FusionObservation observation = {};
            observation.kind = SensorKind::Pressure;
            observation.stamp = pressure.stamp;
            observation.data.pressure = pressure;
            timeline.Push(observation);
        }
        timeline.ProcessReady(now,
                              8); // 每轮最多处理 8 条；固定等待 8000 us，迟到超过窗口的观测丢弃。
        const ControlRelease release = ReadLatestControlRelease();
        if (release.sequence && release.sequence != previous_release)
        {
            // PERF FIX: 状态快照和诊断只在每个 150 Hz 控制节拍生成一次，而不是每个 IMU 包一次
            // (约 2000 次/秒)。控制任务和 EDIAG 只读取节拍时刻的快照。
            latest_fusion_state = timeline.State(now);
            fusion_diagnostics = timeline.Diagnostics();
            imu_acquisition_diagnostics = ReadImuAcquisitionDiagnostics();
            ImuAttitudeFrame frame = {};
            frame.release = release;
            frame.fusion = latest_fusion_state;
            const bool continuous = !previous_release || release.sequence - previous_release == 1U;
            previous_release = release.sequence;
            frame.valid = continuous && have_gyro && have_accel && now - gyro_time <= 3000U &&
                          now - accel_time <= 3000U;
            if (frame.valid)
            {
                frame.attitude = {
                    frame.fusion.euler_rad[2], frame.fusion.euler_rad[1], frame.fusion.euler_rad[0], {0, 0, 0}};
                memcpy(frame.attitude.gyro_rad_s, frame.fusion.body_rate_rad_s, sizeof(frame.attitude.gyro_rad_s));
                frame.valid = (frame.fusion.flags & (FusionAttitudeValid | FusionGyroContinuous)) ==
                              (FusionAttitudeValid | FusionGyroContinuous);
                // 运行时不再调用旧 Mahony；姿态有效性完全来自新估计器。
            }
            // Record the first failing condition so EDIAG shows why the frame was rejected.
            ++imu_frame_diagnostics.published;
            imu_frame_diagnostics.last_gyro_age_us = have_gyro ? now - gyro_time : 0xffffffffU;
            imu_frame_diagnostics.last_accel_age_us = have_accel ? now - accel_time : 0xffffffffU;
            imu_frame_diagnostics.last_flags = frame.fusion.flags;
            if (frame.valid) ++imu_frame_diagnostics.valid;
            else if (!continuous) ++imu_frame_diagnostics.discontinuous;
            else if (!have_gyro) ++imu_frame_diagnostics.no_gyro;
            else if (!have_accel) ++imu_frame_diagnostics.no_accel;
            else if (now - gyro_time > 3000U) ++imu_frame_diagnostics.gyro_stale;
            else if (now - accel_time > 3000U) ++imu_frame_diagnostics.accel_stale;
            else ++imu_frame_diagnostics.bad_flags;
            if (!frame.valid)
                LatchOutputStop();
            frame.completed_us = LcTime_NowUs();
            xQueueOverwrite(imu_attitude_frames.handle, &frame);
            controller_task_diagnostics.i2c3_errors = LcBus_ErrorCount(&hi2c3);
            // PERF FIX: uxTaskGetStackHighWaterMark 逐字节扫描空闲栈(8 KB 栈约 2 万周期)。
            // 原先每个 IMU 包调用一次，约占 1/4 CPU，饿死了低优先级的控制/I2C2 任务，
            // 使 MS5837 的 1 ms HAL 超时误触发 → I2C2 错误 → StopBusReplyTimeout 致命锁存。改为约每秒一次。
            if (++stack_check_releases >= LC_CONTROL_HZ)
            {
                stack_check_releases = 0;
                controller_task_diagnostics.imu_attitude_stack_free = uxTaskGetStackHighWaterMark(NULL);
            }
        }
        const uint32_t end = LcTime_NowUs();
        load_busy_us += end - busy_start;
        if (end - load_window_start >= 1000000U)
        {
            imu_frame_diagnostics.load_permille = uint32_t(uint64_t(load_busy_us) * 1000U / (end - load_window_start));
            load_busy_us = 0;
            load_window_start = end;
        }
    }
}
} // namespace tasks
} // namespace lower_controller
#endif
