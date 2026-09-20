#ifndef PRESSURE_COMPENSATION_H
#define PRESSURE_COMPENSATION_H
#include "FusionConfiguration.h"

namespace lower_controller
{

struct CompensatedPressure
{
    float pressure_pa, temperature_c;
    bool valid;
};

// C[0..6] 为 PROM，D1/D2 为 24 位 ADC；调用者另行核对 PROM CRC 和本次传输状态。
CompensatedPressure CompensateMs5837(PressureModel model, const uint32_t coefficients[7], uint32_t d1,
                                     uint32_t d2);

} // namespace lower_controller
#endif
