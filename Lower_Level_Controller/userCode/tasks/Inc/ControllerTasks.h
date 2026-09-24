#ifndef CONTROLLER_TASKS_H
#define CONTROLLER_TASKS_H
#include "Usermain.h"
#include "Propeller.h"
#include "Servo.h"
#include "LED.h"
#include "ControllerRtosHooks.h"

struct ControllerTaskDiagnostics
{
    uint32_t command_timeouts, bus_reply_timeouts, safety_faults, last_stop_reason;
    // 停机那一刻的 not_ready_mask。not_ready_mask 每个控制周期都会被覆盖，
    // 事后再发 EDIAG 拿到的是恢复后的值，查不出当时到底是哪一项没成立。
    uint32_t stop_not_ready_mask;
    // 七个 READY 条件各自导致过多少次停机判定，按 not_ready_mask 的位序。
    uint32_t not_ready_counts[7];
    uint32_t pressure_sample_drops, feedback_mode_rejections, mag_stale_samples, mag_bus_rejections,
        heater_stale_cycles;
    uint32_t releases, control_cycles, control_overruns, imu_overruns, imu_timeouts;
    uint32_t rx_drops, rx_errors, tx_drops, tx_errors, i2c2_errors, i2c3_errors;
    uint32_t stopped_cycles, calibrations, calibration_failures, deadline_misses, last_cycle_us, max_cycle_us;
    uint32_t control_loop_stack_free, imu_attitude_stack_free, pressure_pwm_stack_free,
        uart_transmit_stack_free;
    const char *assert_file;
    int assert_line;
};
extern volatile ControllerTaskDiagnostics controller_task_diagnostics;
void StartControllerTasks(Device *const *devices, uint32_t count, Propeller_I2C *propeller, Servo_I2C *servo,
                          LED *led);
#endif
