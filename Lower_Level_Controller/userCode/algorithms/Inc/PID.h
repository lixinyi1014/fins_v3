//
// Created by LEGION on 2021/10/19.
//

#ifndef RM_FRAME_C_PID_H
#define RM_FRAME_C_PID_H

#include "Usermain.h"

struct PID_Regulator_t {
    float ref;
    float fdb;
    float err[4];
    float errSum;
    float kp;
    float ki;
    float kd;
    float componentKp;
    float componentKi;
    float componentKd;
    float componentKpMax;
    float componentKiMax;
    float componentKdMax;
    float output;
    float outputMax;
    PID_Regulator_t() : ref(0), fdb(0), err{}, errSum(0), kp(0), ki(0), kd(0),
        componentKp(0), componentKi(0), componentKd(0), componentKpMax(0),
        componentKiMax(0), componentKdMax(0), output(0), outputMax(0) {};
    PID_Regulator_t(float kp, float ki, float kd, float pM, float iM, float dM, float oM):
        ref(0), fdb(0), err{}, errSum(0), kp(kp), ki(ki), kd(kd),
        componentKp(0), componentKi(0), componentKd(0), componentKpMax(pM),
        componentKiMax(iM), componentKdMax(dM), output(0), outputMax(oM) {};
    PID_Regulator_t(const PID_Regulator_t& OTHER){
        std::memcpy(this, &OTHER, sizeof(PID_Regulator_t));
    }
};


class PID{
public:
    PID_Regulator_t PIDInfo{};
    void Reset(PID_Regulator_t * pidRegulator);
    void Reset();
    float PIDCalc(float target,float feedback);
    float PIDCalc(float target,float feedback,float max);
};

#endif //RM_FRAME_C_PID_H
