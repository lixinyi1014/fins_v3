#include "Sensor.h"

/*
 * Disabled deliberately.  This was the newer timed/acquire path that added
 * freshness and validity gating.  The active firmware uses the old
 * PressureSensor::Handle_all() state machine verbatim so read timing, checks,
 * and units remain identical to the known-good controller.
 */
#if 0
#if LC_IMU_ASYNC_ENABLED
#include "PressureCompensation.h"
#include "ControllerRtosHooks.h"
#include "I2cBusAccess.h"

/** @brief Deliver time-stamped pressure arrays without blocking for conversion.
 * 三阶段仍各占一个 150 Hz 释放点：发 D2 → 读 D2/发 D1 → 读 D1，名义阵列频率 50 Hz。
 * MS5837 没有本工程可用的 DRDY 引脚；采样代理时刻采用 D1 命令结束 + 1500 us。
 * 这只是 3 ms 转换窗口中点估计，不是假称硬件精确同步。每路时刻单独保留。 */
void PressureSensor::AcquireTimedPressure()
{
    using namespace lower_controller;
    const uint32_t started = LcTime_NowUs();
    if (ps_state != PS_HANDLE_STATE::GET_TEMPERATURE &&
        started - conversion_started_us_ < LC_PRESSURE_CONVERSION_US)
        return;
    if (ps_state == PS_HANDLE_STATE::GET_TEMPERATURE)
        acquisition_mask_ = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        const uint32_t elapsed = LcTime_NowUs() - started;
        if (elapsed >= LC_BUS_BUDGET_US)
        {
            acquisition_mask_ &= uint8_t((1U << i) - 1U);
            break;
        }
        // 各通道可以独立失败，但共享一个 5000 us 总预算，不能每换一路就重新获得 5 ms。
        LcBus_Begin(&hi2c2, LC_BUS_BUDGET_US - elapsed);
        if (!flag_ok[i] || (ps_state != PS_HANDLE_STATE::GET_TEMPERATURE && !(acquisition_mask_ & (1U << i))))
            continue;
        TCA_SetChannel(i); // TCA 0..3 与 physical_sample_ 下标一一对应。
        uint8_t command = 0x54, bytes[3] = {};
        if (ps_state != PS_HANDLE_STATE::GET_TEMPERATURE)
        {
            command = 0x00;
            LcBus_Transmit(&hi2c2, 0xec, &command, 1, 1);
            LcBus_Receive(&hi2c2, 0xec, bytes, 3, 1);
            const uint32_t adc = uint32_t(bytes[0]) * 65536U + uint32_t(bytes[1]) * 256U + bytes[2];
            if (!adc || adc == 0xffffffU)
            {
                acquisition_mask_ &= uint8_t(~(1U << i));
                continue;
            }
            if (ps_state == PS_HANDLE_STATE::GET_PRESSURE)
            {
                temperature_adc_[i] = adc;
                command = 0x44;
            }
            else
                pressure_adc_[i] = adc;
        }
        if (ps_state != PS_HANDLE_STATE::CALCULATE)
        {
            LcBus_Transmit(&hi2c2, 0xec, &command, 1, 1);
            conversion_started_us_ = LcTime_NowUs();
            if (ps_state == PS_HANDLE_STATE::GET_PRESSURE)
                pressure_time_[i] = conversion_started_us_ + LC_PRESSURE_CONVERSION_US / 2U;
        }
        if (!LcBus_Ok(&hi2c2))
            acquisition_mask_ &= uint8_t(~(1U << i));
        else if (ps_state == PS_HANDLE_STATE::GET_TEMPERATURE)
            acquisition_mask_ |= uint8_t(1U << i);
    }
    if (ps_state == PS_HANDLE_STATE::GET_TEMPERATURE)
    {
        ps_state = PS_HANDLE_STATE::GET_PRESSURE;
        return;
    }
    if (ps_state == PS_HANDLE_STATE::GET_PRESSURE)
    {
        ps_state = PS_HANDLE_STATE::CALCULATE;
        return;
    }
    PressureArraySample physical = {};
    physical.stamp.sequence = physical_sample_.stamp.sequence + 1U;
    physical.stamp.received_us = LcTime_NowUs();
    physical.calibration_epoch = calibration_epoch_;
    PressureMeasurement sample = {};
    sample.sequence = last_measurement_.sequence + 1U;
    sample.read_completed_ms = HAL_GetTick();
    bool have_time = false;
    for (unsigned i = 0; i < 4; ++i)
    {
        physical.channel_sample_us[i] = pressure_time_[i];
        if (!(acquisition_mask_ & (1U << i)))
            continue;
        if (!have_time || static_cast<int32_t>(pressure_time_[i] - physical.stamp.sample_us) > 0)
        {
            physical.stamp.sample_us = pressure_time_[i];
            have_time = true;
        }
        const auto compensated = CompensateMs5837(GetFusionConfiguration().pressure_model, Cal_C[i],
                                                  pressure_adc_[i], temperature_adc_[i]);
        if (compensated.valid)
        {
            physical.pressure_pa[i] = compensated.pressure_pa;
            physical.valid_mask |= uint8_t(1U << i);
        }
        // 下方仅为旧反馈数值兼容：原公式混有型号分支，不能将其当作新 ESKF 的 Pa 输入。
        sample.prom_valid_mask |= uint8_t(1U << i);
        D2_Temp = temperature_adc_[i];
        D1_Pres = pressure_adc_[i];
        dT = D2_Temp - (((uint32_t)Cal_C[i][5]) * 256l);
        SENS = (int64_t)Cal_C[i][1] * 65536l + ((int64_t)Cal_C[i][3] * dT) / 128l;
        OFF_ = (int64_t)Cal_C[i][2] * 131072l + ((int64_t)Cal_C[i][4] * dT) / 64l;

        TEMP = 2000l + (int64_t)(dT)*Cal_C[i][6] / 8388608LL;
        if (TEMP < 2000) // low temp
        {

            Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        }
        else
        { // high temp
            Ti = 2LL * int64_t(dT) * int64_t(dT) / 137438953472LL;
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
        OFF2 = OFF_ - OFFi;
        SENS2 = SENS - SENSi;
        sample.pressure_legacy[i] = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
    }
    if (!have_time)
        physical.stamp.sample_us = physical.stamp.received_us;
    physical_sample_ = physical; // 型号未确认时 valid_mask=0，模型检查保留，不猜测型号。
    legacy_frame_valid_ = acquisition_mask_ == 0x0f;
    if (legacy_frame_valid_)
        last_measurement_ = sample; // 旧四路组合量不接受部分阵列；新 ESKF 自行处理有效子集。
    ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;
}
#endif
#endif // 0: legacy Handle_all() is the active pressure acquisition path
