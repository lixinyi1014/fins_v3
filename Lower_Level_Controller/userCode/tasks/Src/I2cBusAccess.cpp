#include "I2cBusAccess.h"
#include "ControllerRtosHooks.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2, hi2c3;
namespace
{
struct BusState
{
    TaskHandle_t owner;
    uint32_t started_us, budget_us, errors;
    bool failed;
};
BusState state2 = {}, state3 = {};
BusState &State(I2C_HandleTypeDef *bus)
{
    configASSERT(bus == &hi2c2 || bus == &hi2c3);
    return bus == &hi2c2 ? state2 : state3;
}
bool Allowed(I2C_HandleTypeDef *bus)
{
    BusState &state = State(bus);
    if (!LcRuntime_IsRunning() && !state.budget_us)
    {
        state.started_us = LcTime_NowUs();
        state.budget_us = 3000000U; // Startup fallback, never an unbounded legacy timeout.
    }
    configASSERT(__get_IPSR() == 0);
    if (LcRuntime_IsRunning())
        configASSERT(xTaskGetCurrentTaskHandle() == state.owner);
    if (state.failed)
        return false; // 本请求一旦失败，后续事务直接拒绝，直到下一次 Begin
    if (__HAL_I2C_GET_FLAG(bus, I2C_FLAG_BUSY) != RESET)
    {
        state.failed = true; // 单主机总线空闲时才发起事务，避开 HAL 内部的 25 ms BUSY 等待
        ++state.errors;
        return false;
    }
    if (LcTime_NowUs() - state.started_us >= state.budget_us)
    { // 单位 us；事务之间检查，不能强行中断已经开始的 HAL 调用
        state.failed = true;
        ++state.errors;
        return false;
    }
    return true;
}
uint32_t Timeout(I2C_HandleTypeDef *bus, uint32_t requested)
{
    if (!LcRuntime_IsRunning())
    {
        // 启动阶段只有 main 一个执行者，沿用旧裸机初始化的单事务节拍；
        // 但把原来可能为 0xffff 的无限接近等待收紧为 10 ms，避免无设备时卡死。
        return requested > 10U ? 10U : requested;
    }
    return requested > LC_I2C_TIMEOUT_MS
               ? LC_I2C_TIMEOUT_MS
               : requested; // 运行期传给 HAL 的超时参数最多 2 ms；不是整笔事务的硬时间上界
}
HAL_StatusTypeDef Record(I2C_HandleTypeDef *bus, HAL_StatusTypeDef status)
{
    if (status != HAL_OK)
    {
        State(bus).failed = true;
        ++State(bus).errors;
    }
    return status;
}
} // namespace
extern "C" void LcBus_SetOwner(I2C_HandleTypeDef *bus, void *task)
{
    State(bus).owner = static_cast<TaskHandle_t>(task);
}
extern "C" void LcBus_Begin(I2C_HandleTypeDef *bus, uint32_t budget_us)
{
    BusState &state = State(bus);
    state.failed = false;
    state.started_us = LcTime_NowUs(); // 记录本次请求起点，单位 us；不清零累计错误数
    state.budget_us = budget_us;
}
extern "C" int LcBus_Ok(I2C_HandleTypeDef *bus)
{
    return !State(bus).failed;
}
extern "C" uint32_t LcBus_ErrorCount(I2C_HandleTypeDef *bus)
{
    return State(bus).errors;
}
extern "C" HAL_StatusTypeDef LcBus_Transmit(I2C_HandleTypeDef *bus, uint16_t addr, uint8_t *data,
                                            uint16_t length, uint32_t timeout)
{
    // 启动阶段没有任务竞争总线，不做运行期 BUSY/owner 门禁；这与旧代码的
    // HAL_I2C_Master_Transmit 初始化路径一致。进入调度器后仍必须经过 Allowed。
    if (!LcRuntime_IsRunning())
        return Record(bus, HAL_I2C_Master_Transmit(bus, addr, data, length, Timeout(bus, timeout)));
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Master_Transmit(bus, addr, data, length, Timeout(bus, timeout)));
}
extern "C" HAL_StatusTypeDef LcBus_Receive(I2C_HandleTypeDef *bus, uint16_t addr, uint8_t *data,
                                           uint16_t length, uint32_t timeout)
{
    memset(data, 0, length);
    if (!LcRuntime_IsRunning())
        return Record(bus, HAL_I2C_Master_Receive(bus, addr, data, length, Timeout(bus, timeout)));
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Master_Receive(bus, addr, data, length, Timeout(bus, timeout)));
}
extern "C" HAL_StatusTypeDef LcBus_MemRead(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                           uint16_t reg_size, uint8_t *data, uint16_t length,
                                           uint32_t timeout)
{
    memset(data, 0, length);
    if (!LcRuntime_IsRunning())
        return Record(bus, HAL_I2C_Mem_Read(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)));
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Mem_Read(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)));
}
extern "C" HAL_StatusTypeDef LcBus_MemWrite(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                            uint16_t reg_size, uint8_t *data, uint16_t length,
                                            uint32_t timeout)
{
    if (!LcRuntime_IsRunning())
        return Record(bus, HAL_I2C_Mem_Write(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)));
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Mem_Write(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)));
}
