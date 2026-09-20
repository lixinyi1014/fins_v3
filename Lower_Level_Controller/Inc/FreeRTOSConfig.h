#ifndef LOWER_FREERTOS_CONFIG_H
#define LOWER_FREERTOS_CONFIG_H
#include <stdint.h>
#include "LowerControllerConfig.h"

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configCPU_CLOCK_HZ                      LC_HCLK_HZ // 168000000 Hz；与 HCLK 配置一致
#define configTICK_RATE_HZ                      LC_RTOS_TICK_HZ // 1000 Hz；1 tick=1 ms
#define configMAX_PRIORITIES                    6
#define configMINIMAL_STACK_SIZE                256
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configQUEUE_REGISTRY_SIZE               8
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TIMERS                        0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configUSE_NEWLIB_REENTRANT              0
#define configUSE_TICKLESS_IDLE                 0
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (15U << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5U << 4U) // 0x50；使用 FromISR 的 NVIC 优先级数值须 ≥5
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler

#ifdef __cplusplus
extern "C" {
#endif
void LcRuntime_Assert(const char *file, int line);
#ifdef __cplusplus
}
#endif
#define configASSERT(condition) do { if (!(condition)) LcRuntime_Assert(__FILE__, __LINE__); } while (0)
#endif
