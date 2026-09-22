#include "TaskSharedResources.h"
#include "IMU.h"
#include "I2cBusAccess.h"
#include <string.h>
#include <math.h>

// 上一阶段兼容实现，仅 LC_IMU_ASYNC_ENABLED=0 编译。默认任务位于 ImuFusionTask.cpp。

namespace lower_controller
{
namespace tasks
{

#if !LC_IMU_ASYNC_ENABLED
void ImuAttitudeTask(void *)
{
    uint32_t previous_sequence = 0;
    for (;;)
    {
        uint32_t pending =
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 阻塞等 TIM1；返回累计次数，pdTRUE 一次清空计数
        ImuAttitudeFrame frame = {};
        frame.release = ReadLatestControlRelease();
        bool continuous = previous_sequence == 0 || frame.release.sequence - previous_sequence == 1U;
        previous_sequence = frame.release.sequence;
        if (pending > 1)
            controller_task_diagnostics.imu_overruns += pending - 1;
        while (xSemaphoreTake(imu_dma_done, 0) == pdPASS)
        {
        }                  // 开始新帧前清除残留完成信号；0 表示不阻塞
        IMU::imu.Handle(); // 仅 IDLE 时启动陀螺仪 DMA；随后由中断衔接加速度和温度
        if (xSemaphoreTake(imu_dma_done, pdMS_TO_TICKS(LC_IMU_DMA_TIMEOUT_MS)) == pdPASS)
        {                                          // 4 ms
            LcBus_Begin(&hi2c3, LC_BUS_BUDGET_US); // 磁力计总线由本任务独占，沿用当前总线事务预算
            ImuMeasurement measurement = {};
            if (IMU::imu.FinishAcquisition(measurement))
            { // 在任务中消费 READY 缓冲并解码；失败不交付有效帧
                if (!LcBus_Ok(&hi2c3))
                    memset(measurement.mag_uT, 0,
                           sizeof(measurement.mag_uT)); // 三轴置 0，进入原 Mahony 无磁场分支
                IMU::imu.UpdateEstimate(measurement);   // Mahony 固定 dt=1/150 s
                frame.attitude.yaw_rad = IMU::imu.attitude.yaw;
                frame.attitude.pitch_rad = IMU::imu.attitude.pitch;
                frame.attitude.roll_rad = IMU::imu.attitude.roll;
                memcpy(frame.attitude.gyro_rad_s, measurement.gyro_rad_s, sizeof(measurement.gyro_rad_s));
                frame.valid = continuous && isfinite(frame.attitude.yaw_rad) &&
                              isfinite(frame.attitude.pitch_rad) && isfinite(frame.attitude.roll_rad);
            }
        }
        else
        {
            ++controller_task_diagnostics.imu_timeouts;
            IMU::imu.AbortAcquisition(); // 请求终止 DMA；未确认停稳时禁止复用接收缓冲
        }
        if (!frame.valid)
            LatchOutputStop();
        frame.completed_us = LcTime_NowUs(); // 软件解算完成时刻，单位 us；不是传感器物理采样时间
        xQueueOverwrite(imu_attitude_frames.handle, &frame); // 长度 1，复制最新结果；旧帧不积压
        controller_task_diagnostics.i2c3_errors = LcBus_ErrorCount(&hi2c3);
        controller_task_diagnostics.imu_attitude_stack_free = uxTaskGetStackHighWaterMark(NULL);
    }
}

#endif
} // namespace tasks
} // namespace lower_controller
