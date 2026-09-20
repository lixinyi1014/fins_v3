#include "BMI088Middleware.h"
//
// Created by LEGION on 2021/10/17.
//

#include "IMU.h"
#include "ist8310driver.h"
#include "UART_Base.h"
#include "Sensor.h"
#include "Usermain.h"
#include "LegacyEstimation.h"
#include "ControllerRtosHooks.h"
#include "I2cBusAccess.h"
#include "ImuDmaAcquisition.h"

IMU IMU::imu;

bool is_output_angle = 0;
// bool is_output_accl = 0;

void IMU::Init()
{
    init_error_ = 0;
    IMU_temp_PWM(0);
    LcTime_Start();
    BMI088_ClearTransferError();
    if (BMI088_init() != BMI088_NO_ERROR || !BMI088_TransferOk()) {
        init_error_ = 1;
        return;
    }
#ifdef IMU_USE_MAG
    if (ist8310_init() != IST8310_NO_ERROR || !LcBus_Ok(&hi2c3)) {
        init_error_ = 2;
        return;
    }
#endif
    // BMI088_init already writes and reads back ACC_PWR_CTRL=0x04.
    PID_Regulator_t _tempPID(1500, 0.08, 0, MPU6500_TEMP_PWM_MAX, MPU6500_TEMP_PWM_MAX, 0, MPU6500_TEMP_PWM_MAX);

    tempPid.Reset(&_tempPID);
    first_temperate = temperature_stable_samples_ = heater_fault_ = 0;
    heating_started_ = false;

    pro_data.ins_quat[0] = 1.0f;
    pro_data.ins_quat[1] = 0.0f;
    pro_data.ins_quat[2] = 0.0f;
    pro_data.ins_quat[3] = 0.0f;
    pro_data.ins_angle[0] = 0.0f;
    pro_data.ins_angle[1] = 0.0f;
    pro_data.ins_angle[2] = 0.0f;
    raw_data.gyro_offset[0] = 0.0f;
    raw_data.gyro_offset[1] = 0.0f;
    raw_data.gyro_offset[2] = 0.0f;
    raw_data.accel_offset[0] = 0.07f;
    raw_data.accel_offset[1] = -0.17f;
    raw_data.accel_offset[2] = 0.0f;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) { init_error_ = 3; return; }
    SPI1_DMA_init((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    dma_state = IMU_DMA_IDLE;

}

void IMU::Handle()
{
		// Normal operation.
    // 普通使用
    // BMI088_read(raw_data.gyro, raw_data.accel, &raw_data.temp);
    // ist8310_read_mag(raw_data.mag);    // Read magnetometer data and store it in `mag`. //读取磁力计数据，存储在mag中
    // offset_correction();
    // MahonyAHRSupdate(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2], pro_data.mag[0], pro_data.mag[1], pro_data.mag[2]);    // Compute the quaternion from the acquired accelerometer, gyroscope, and magnetometer measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪和磁力计数据进行计算得到四元数，直接在数组quat中更新
	// // MahonyAHRSupdateIMU(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2]);    // Compute the quaternion from the acquired accelerometer and gyroscope measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪数据进行计算得到四元数，直接在数组quat中更新
	// get_angle(pro_data.ins_quat, pro_data.ins_angle, pro_data.ins_angle+1, pro_data.ins_angle+2);    // Compute Euler angles (in radians) from the quaternion, and update them in-place in the `INS_angle` array. //对四元数计算得到弧度制欧拉角，直接在INS_angle数组中更新

		// Enable DMA transfer for the IMU.
    // 开启IMU DMA传输
    if (dma_state == IMU_DMA_IDLE)
    {
        read_started_ms_ = HAL_GetTick();
        dma_state = IMU_DMA_READ_GYRO;
        trigger_gyro_dma();
    }
    // if(PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE){
    //     for(int i=0; i<3;i++){
    //         send_float(pro_data.gyro[i], 2, 0);
    //     }
    // }
}

void IMU::Receive()
{
    if (strncmp((char *)RxBuffer, "BEG", 3) == 0)
    {
        flag_Test = true;
    }
    if (strncmp((char *)RxBuffer, "END", 3) == 0)
    {
        flag_Test = false;
    }
    if (strncmp((char *)RxBuffer, "IVA:BEG", 7) == 0)
    {
        is_output_angle = 1;
    }
    if (strncmp((char *)RxBuffer, "IVA:END", 7) == 0)
    {
        is_output_angle = 0;
    }

    // if (strncmp((char *)RxBuffer, "AVA:BEG", 7) == 0)
    // {
    //     is_output_accl = 1;
    // }
    // if (strncmp((char *)RxBuffer, "AVA:BEG", 7) == 0)
    // {
    //     is_output_accl = 0;
    // }
    
}

void IMU::trigger_gyro_dma(void)
{
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    }
}

void IMU::trigger_accel_dma(void)
{
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_dma_tx_buf, (uint32_t)buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    }
}

void IMU::trigger_temp_dma(void)
{
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_temp_dma_tx_buf, (uint32_t)buf.accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    }
}

void IMU::DMA_IT_Handle()
{
#if LC_IMU_ASYNC_ENABLED
    if (LcRuntime_IsRunning()) { lower_controller::ImuDmaInterrupt(); return; }
#endif
#if LC_USE_FREERTOS
    uint32_t rx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_rx) | __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_rx) | __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_rx);
    uint32_t tx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_tx) | __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_tx) | __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_tx);
    if (__HAL_DMA_GET_FLAG(&hdma_spi1_rx, rx_errors) || __HAL_DMA_GET_FLAG(&hdma_spi1_tx, tx_errors)) {
        __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, rx_errors | __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_rx));
        __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, tx_errors);
        dma_state = IMU_DMA_ABORTED;
        LcRuntime_ImuReadyFromISR(); // 任一段传输出错，交由任务终止；不解码残缺帧
        return;
    }
#endif
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        switch (dma_state)
        {
        case IMU_DMA_IDLE:
        case IMU_DMA_READY:
        case IMU_DMA_ABORTED:
            break;
        case IMU_DMA_READ_GYRO:
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
#if !LC_USE_FREERTOS
            BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, raw_data.gyro);
#endif
            dma_state = IMU_DMA_READ_ACCEL;
            trigger_accel_dma();
            break;
        case IMU_DMA_READ_ACCEL:
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
#if !LC_USE_FREERTOS
            BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, raw_data.accel, &raw_data.time);
#endif
            dma_state = IMU_DMA_READ_TEMP;
            trigger_temp_dma();
            break;
        case IMU_DMA_READ_TEMP: {
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
#if LC_USE_FREERTOS
            dma_state = IMU_DMA_READY;
            LcRuntime_ImuReadyFromISR(); // 缓冲区保持不变，直到 IMU 任务消费完成
#else
            BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &raw_data.temp);
            imu_temp_control(raw_data.temp);
            // Retain the old order: magnetometer/calibration -> Mahony -> publication -> telemetry.
            // 保留原顺序：磁力计读取及标定 -> Mahony -> 发布兼容字段 -> 遥测。
            const lower_controller::ImuMeasurement sample = ReadMeasurement(); // 角速度 rad/s、加速度 m/s^2、磁场 uT
            UpdateEstimate(sample); // 固定 dt=1/150 s，姿态输出为 rad
            OutputLegacyTelemetry();
            dma_state = IMU_DMA_IDLE;
#endif
            break;
        }
        }
    }
}

bool IMU::FinishAcquisition(lower_controller::ImuMeasurement& sample)
{
    if (dma_state != IMU_DMA_READY) { AbortAcquisition(); return false; } // 只消费三段传输均结束的 READY 帧；其他状态不解码
#if LC_USE_FREERTOS
    uint32_t rx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_rx) | __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_rx) | __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_rx);
    uint32_t tx_errors = __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_spi1_tx) | __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_spi1_tx) | __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_spi1_tx);
    if (__HAL_DMA_GET_FLAG(&hdma_spi1_rx, rx_errors) || __HAL_DMA_GET_FLAG(&hdma_spi1_tx, tx_errors)) {
        AbortAcquisition();
        return false;
    }
#endif
    BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, raw_data.gyro);
    BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, raw_data.accel, &raw_data.time);
    BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &raw_data.temp);
    imu_temp_control(raw_data.temp);
    sample = ReadMeasurement(); // rad/s、m/s^2、uT；磁力计访问仅在任务执行
    dma_state = IMU_DMA_IDLE; // 原始缓冲已复制到 sample，允许下一轮 DMA 复用
    return true;
}

void IMU::AbortAcquisition()
{
#if LC_USE_FREERTOS
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    dma_state = IMU_DMA_ABORTED;
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    __set_PRIMASK(mask);
    uint32_t started = LcTime_NowUs();
    while ((hdma_spi1_rx.Instance->CR | hdma_spi1_tx.Instance->CR) & DMA_SxCR_EN) {
        if (LcTime_NowUs() - started >= 1000U) return; // 1000 us；未停稳时禁止复用缓冲区
    }
    mask = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_spi1_rx));
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn); // 清除旧 RX 完成中断，避免终止后误交付上一帧
    dma_state = IMU_DMA_IDLE;
    __set_PRIMASK(mask);
#endif
}

lower_controller::ImuMeasurement IMU::ReadMeasurement()
{
    // I2C3 acquisition stays here; the estimator itself performs no peripheral access.
    // 磁力计仍在这里经 I2C3 读取；估计函数不再访问外设。
    ist8310_read_mag(raw_data.mag);
    offset_correction();
    lower_controller::ImuMeasurement sample = {};
    for (int i = 0; i < 3; ++i) {
        sample.gyro_rad_s[i] = pro_data.gyro[i];
        sample.accel_m_s2[i] = pro_data.accel[i];
        sample.mag_uT[i] = pro_data.mag[i];
    }
    sample.temperature_c = raw_data.temp;
    sample.sequence = ++measurement_sequence_;
    sample.read_started_ms = read_started_ms_;
    sample.read_completed_ms = HAL_GetTick();
    return sample;
}

void IMU::UpdateEstimate(const lower_controller::ImuMeasurement& sample)
{
    const lower_controller::LegacyAttitudeResult result =
        lower_controller::EstimateLegacyAttitude(sample, pro_data.ins_quat); // 四元数顺序 [w,x,y,z]
    attitude_update(result);
}

void IMU::OutputLegacyTelemetry()
{
    // Preserve the original pressure-state gate and four-field serial output.
    // 旧回归路径保留门控和四字段格式；FreeRTOS 路径由控制任务发送同一帧。
    if (PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE) {
        for (int i = 0; i < 3; ++i)
            send_float(PressureSensor::pressure_sensor.data_pressure[i], 2, 0);
        send_float(PressureSensor::pressure_sensor.data_pressure[3], 2, 1);
    }
}

// DMA transfer-complete interrupt.
// DMA传输完成中断
void DMA2_Stream0_IRQHandler(void)
{
    IMU::imu.DMA_IT_Handle();
}

void IMU::ErrorHandle()
{
}

void IMU::imu_temp_control(float temp)
{
    // TODO(HW): measure warmup on this assembly; thresholds are conservative initial limits.
    // TIM10 ARR=4999; 2000 counts is approximately 40% duty, not full heater power.
    if (!heating_started_) { heating_started_ms_ = HAL_GetTick(); heating_started_ = true; }
    if (!isfinite(temp) || temp < -40.0f || temp >= 55.0f) heater_fault_ = 1;
    if (!first_temperate && HAL_GetTick() - heating_started_ms_ >= 180000U) heater_fault_ = 2;
    if (heater_fault_) { tempPid.Reset(); IMU_temp_PWM(0); return; }
    if (!first_temperate) {
        if (temp >= IMU_TARGET_TEMP) ++temperature_stable_samples_;
        else temperature_stable_samples_ = 0; // Require consecutive warm samples.
        if (temperature_stable_samples_ < 200U) {
            IMU_temp_PWM(temp >= IMU_TARGET_TEMP ? MPU6500_TEMP_PWM_MAX / 2 : MPU6500_TEMP_PWM_MAX - 1);
            return;
        }
        tempPid.Reset();
        tempPid.PIDInfo.errSum = (MPU6500_TEMP_PWM_MAX / 2.0f) / tempPid.PIDInfo.ki;
        first_temperate = 1;
    }
    const float output = tempPid.PIDCalc(IMU_TARGET_TEMP, temp);
    if (!isfinite(output)) { heater_fault_ = 1; IMU_temp_PWM(0); return; }
    IMU_temp_PWM(static_cast<uint16_t>(fmaxf(0, fminf(MPU6500_TEMP_PWM_MAX, output))));
}

void IMU::IMU_temp_PWM(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief Measure the gyroscope and accelerometer biases and store them in the `raw_data.accel_offset` and `raw_data.gyro_offset` arrays. 测得陀螺仪和加速度计零偏值，存入raw_data.accel_offset和raw_data.gyro_offset数组
 * @param accel
 * @param _accel
 */
void IMU::calibrate_offset()
{
    float accel[3], gyro[3], temp;
    for (int i = 0; i < 1000; i++)
    {
        BMI088_read(gyro, accel, &temp);
        raw_data.accel_offset[0] += accel[0];
        raw_data.accel_offset[1] += accel[1];
        raw_data.accel_offset[2] += accel[2];
        raw_data.gyro_offset[0] += gyro[0];
        raw_data.gyro_offset[1] += gyro[1];
        raw_data.gyro_offset[2] += gyro[2];
        HAL_Delay(1);
    }
    raw_data.accel_offset[0] /= 1000;
    raw_data.accel_offset[1] /= 1000;
    raw_data.accel_offset[2] /= 1000;
    raw_data.gyro_offset[0] /= 1000;
    raw_data.gyro_offset[1] /= 1000;
    raw_data.gyro_offset[2] /= 1000;
}

void IMU::offset_correction()
{
    for (int i=0; i<3; i++)
    {
        pro_data.gyro[i] = raw_data.gyro[i] - raw_data.gyro_offset[i];
        pro_data.accel[i] = raw_data.accel[i] - raw_data.accel_offset[i];
    }
    pro_data.mag[0] = (raw_data.mag[0] - MAG_OFFSET_X) * MAG_SCALE_X;
    pro_data.mag[1] = (raw_data.mag[1] - MAG_OFFSET_Y) * MAG_SCALE_Y;
    pro_data.mag[2] = (raw_data.mag[2] - MAG_OFFSET_Z) * MAG_SCALE_Z;
}

void IMU::attitude_update(const lower_controller::LegacyAttitudeResult& result)
{
		// `ins_angle[0]`, `ins_angle[1]`, and `ins_angle[2]` correspond to yaw, pitch, and roll, respectively.
    // ins_angle 012对应yaw，pitch，roll
    pro_data.ins_angle[0] = attitude.yaw = result.yaw_rad;
    pro_data.ins_angle[1] = attitude.pitch = result.pitch_rad;
    pro_data.ins_angle[2] = attitude.roll = result.roll_rad;
    attitude.yaw_v = result.gyro_rad_s[2];
    attitude.pitch_v = result.gyro_rad_s[1];
    attitude.roll_v = result.gyro_rad_s[0];
    attitude.neg_yaw_v = - attitude.yaw_v;
    attitude.neg_pitch_v = - attitude.pitch_v;
    attitude.neg_roll_v = - attitude.roll_v;
}
