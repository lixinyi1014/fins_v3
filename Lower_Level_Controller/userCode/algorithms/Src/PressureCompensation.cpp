#include "PressureCompensation.h"
#include <math.h>

namespace lower_controller
{

/** @brief Temperature compensation with an explicit model. 按明确型号做一、二阶温度补偿。
 * 02BA 与 30BA 的 SENS/OFF 系数、二阶项和压力缩放不同；不能只改函数名。
 * 返回值统一为 Pa、°C；用 double 中间量避免 32 位 dT*dT 和低温平方项溢出。
 * 公式依据 TE MS5837 数据手册，来源与测试向量见 docs/PHASE456_IMPLEMENTATION.md。 */
CompensatedPressure CompensateMs5837(PressureModel model, const uint32_t c[7], uint32_t d1, uint32_t d2)
{
    CompensatedPressure result = {};
    if ((model != PressureModel::MS5837_02BA && model != PressureModel::MS5837_30BA) || d1 == 0 || d2 == 0 ||
        d1 >= 0xFFFFFFU || d2 >= 0xFFFFFFU)
        return result;
    for (unsigned i = 1; i < 7; ++i)
        if (c[i] == 0 || c[i] > 65535U)
            return result;
    const double dt = double(d2) - double(c[5]) * 256.0;
    const double temperature = 2000.0 + dt * double(c[6]) / 8388608.0;
    double sensitivity, offset, temperature_correction = 0, offset_correction = 0, sensitivity_correction = 0;
    const double cold = (temperature - 2000.0) * (temperature - 2000.0);
    if (model == PressureModel::MS5837_02BA)
    {
        sensitivity = double(c[1]) * 65536.0 + double(c[3]) * dt / 128.0;
        offset = double(c[2]) * 131072.0 + double(c[4]) * dt / 64.0;
        if (temperature < 2000.0)
        {
            temperature_correction = 11.0 * dt * dt / 34359738368.0;
            offset_correction = 31.0 * cold / 8.0;
            sensitivity_correction = 63.0 * cold / 32.0;
        }
    }
    else
    {
        // MS5837-30BA：沿用旧工程 Sensor.cpp 的一阶系数。
        // C1*2^16、C2*2^17、C3*dT/2^7、C4*dT/2^6；旧路径已在实物上输出正常。
        sensitivity = double(c[1]) * 65536.0 + double(c[3]) * dt / 128.0;
        offset = double(c[2]) * 131072.0 + double(c[4]) * dt / 64.0;
        if (temperature < 2000.0)
        {
            // 二阶项与旧工程 MS5837_30BA_GetData() 完全一致。
            temperature_correction = 11.0 * dt * dt / 34359738368.0;
            offset_correction = 31.0 * cold / 8.0;
            sensitivity_correction = 63.0 * cold / 32.0;
        }
        else
        {
            temperature_correction = 2.0 * dt * dt / 137438953472.0;
            offset_correction = cold / 16.0;
            sensitivity_correction = 0.0;
        }
    }
    const double numerator =
        double(d1) * (sensitivity - sensitivity_correction) / 2097152.0 - (offset - offset_correction);
    // 旧工程的结果单位是 mbar；这里统一换成 Pa，因此乘 100。
    // 30BA 的原始结果为 numerator/32768 mbar。
    result.pressure_pa =
        float(model == PressureModel::MS5837_02BA ? numerator / 32768.0 : numerator / 32768.0 * 100.0);
    result.temperature_c = float((temperature - temperature_correction) / 100.0);
    const float maximum = model == PressureModel::MS5837_02BA ? 200000.0f : 3000000.0f;
    result.valid = isfinite(result.pressure_pa) && isfinite(result.temperature_c) &&
                   result.pressure_pa >= 1000.0f && result.pressure_pa <= maximum &&
                   result.temperature_c >= -20.0f && result.temperature_c <= 85.0f;
    return result;
}

} // namespace lower_controller
