#include "PID.h"
#include <math.h>
void PID::Reset() {
     PIDInfo.ref = 0;
     PIDInfo.fdb = 0;
     PIDInfo.err[0] = 0;
     PIDInfo.err[1] = 0;
     PIDInfo.err[2] = 0;
     PIDInfo.err[3] = 0;
     PIDInfo.errSum = 0;

     PIDInfo.componentKp = 0;
     PIDInfo.componentKi = 0;
     PIDInfo.componentKd = 0;

     PIDInfo.output = 0;

}
void PID::Reset(PID_Regulator_t *pidRegulator) {
    if(pidRegulator != nullptr)PIDInfo = *pidRegulator;
    Reset(); // Loading gains must never import an earlier controller's history.
}

/**
 * Compute the PID control output.
 * @param target   /Reference (setpoint) value.
 * @param feedback /Feedback (measured) value.
 * @return /Control output.
 */
/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PID::PIDCalc(float target,float feedback) {
    if (!isfinite(target) || !isfinite(feedback) || !isfinite(PIDInfo.errSum) ||
        !isfinite(PIDInfo.err[2]) || !isfinite(PIDInfo.kp) || !isfinite(PIDInfo.ki) ||
        !isfinite(PIDInfo.kd) || !isfinite(PIDInfo.componentKpMax) ||
        !isfinite(PIDInfo.componentKiMax) || !isfinite(PIDInfo.componentKdMax) ||
        !isfinite(PIDInfo.outputMax) || PIDInfo.componentKpMax < 0 ||
        PIDInfo.componentKiMax < 0 || PIDInfo.componentKdMax < 0 || PIDInfo.outputMax < 0) {
        Reset();
        return 0;
    }
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    if (PIDInfo.ki != 0) {
        PIDInfo.errSum += PIDInfo.err[3];
        const float integral_limit = PIDInfo.componentKiMax / fabsf(PIDInfo.ki);
        INRANGE(PIDInfo.errSum, -integral_limit, integral_limit);
    } else PIDInfo.errSum = 0;

    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;

    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);

    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    if (!isfinite(PIDInfo.output)) { Reset(); return 0; }
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);

    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}

float PID::PIDCalc(float target, float feedback, float max) {
    PIDInfo.outputMax = max;
    return PID::PIDCalc(target,feedback);
}
