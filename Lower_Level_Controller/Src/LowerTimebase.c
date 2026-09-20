#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ControllerRtosHooks.h"

static uint32_t microseconds_ready;

HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    (void)priority;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq(); // 读取实际 APB1 时钟；系统配置完成后为 42 MHz
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) timer_hz *= 2U; // APB1 分频不为 1 时定时器时钟翻倍，运行期为 84 MHz
    __HAL_RCC_TIM7_CLK_ENABLE();
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = timer_hz / 1000000U - 1U; // 计数频率 1 MHz
    htim7.Init.Period = 999U; // 1000 计数 = 1 ms HAL 时间基准
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(TIM7_IRQn, 2U, 0U); // 优先级 2，不调用 FreeRTOS API
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    uwTickPrio = 2U;
    return HAL_TIM_Base_Start_IT(&htim7);
}

void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE); }

void LcTime_Start(void)
{
    if (microseconds_ready) return; // Initialization and task startup share one continuous clock.
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    uint32_t timer_hz = HAL_RCC_GetPCLK1Freq(); // 读取实际 APB1 时钟；系统配置完成后为 42 MHz
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) timer_hz *= 2U; // APB1 分频不为 1 时定时器时钟翻倍，运行期为 84 MHz
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = timer_hz / 1000000U - 1U; // 运行期 84 MHz/(83+1)=1 MHz
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN; // 1 MHz 自由计数，无中断，约 71.6 分钟回绕
    microseconds_ready = 1;
}

uint32_t LcTime_NowUs(void) { return microseconds_ready ? TIM2->CNT : HAL_GetTick() * 1000U; }

void LcTime_DelayUs(uint32_t delay_us)
{
    LcTime_Start(); // Also valid before the scheduler and before sensor initialization.
    const uint32_t started = LcTime_NowUs();
    while ((uint32_t)(LcTime_NowUs() - started) < delay_us) { }
}

void HAL_Delay(uint32_t delay_ms)
{
    if (__get_IPSR() == 0 && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms) + 1U); // 1 ms/tick，额外 1 tick 保留最短等待；阻塞本任务并让出 CPU
    } else if (__get_IPSR() != 0 && microseconds_ready) {
        while (delay_ms--) {
            uint32_t started = LcTime_NowUs();
            while (LcTime_NowUs() - started < 1000U) { }
        }
    } else {
        uint32_t started = HAL_GetTick();
        uint32_t wait = delay_ms < HAL_MAX_DELAY ? delay_ms + 1U : delay_ms;
        while (HAL_GetTick() - started < wait) { }
    }
}
