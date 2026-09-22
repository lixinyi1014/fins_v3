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
    uint32_t started_us, budget_us, errors, recoveries;
    // 最后一次错误：bit0..7 HAL ErrorCode(1 BERR,2 ARLO,4 AF/NACK,8 OVR,0x20 TIMEOUT)，
    // bit8..15 HAL 返回值(1 ERROR,2 BUSY,3 TIMEOUT)，bit16..23 7 位从机地址，
    // bit24..27 操作(1 发,2 收,3 读寄存器,4 写寄存器,0xB 请求时间预算用尽)。
    uint32_t last_error;
    bool failed, needs_recovery;
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
    // Keep the legacy HAL behavior: HAL itself waits/recovers a transient BUSY state.
    // The old firmware did not reject a transaction merely because BUSY was observed
    // immediately after the previous STOP; doing so makes PCA/MS5837 startup fail.
    if (LcTime_NowUs() - state.started_us >= state.budget_us)
    { // 单位 us；事务之间检查，不能强行中断已经开始的 HAL 调用
        state.failed = true;
        ++state.errors;
        state.last_error = 0xBU << 24;
        return false;
    }
    return true;
}
uint32_t Timeout(I2C_HandleTypeDef *bus, uint32_t requested)
{
    // Pressure conversion reads still pass the old 1 ms timeout.  PCA writes
    // in the legacy driver passed 10000/0xffff; capping those to 2 ms caused
    // a single output NACK/timeout and then stopped the pressure service too.
    // Keep a bounded but legacy-compatible window for the longer PCA/TCA
    // transactions; the per-request bus budget remains the outer limit.
    constexpr uint32_t kLongTransactionTimeoutMs = LC_I2C_HAL_CALL_MAX_MS; // LowerControllerConfig.h 中与回复超时联动校验
    if (requested < LC_I2C_HAL_CALL_MIN_MS)
        requested = LC_I2C_HAL_CALL_MIN_MS; // 抢占不应被误判为总线超时（见配置注释）
    return requested > kLongTransactionTimeoutMs ? kLongTransactionTimeoutMs : requested;
}
/* STM32F4 I2C lock-up recovery. 出错后从机可能停在字节中途并一直拉低 SDA，外设 BUSY 位随之常置，
 * 此后每笔 HAL 调用都会先空等 25 ms 再失败，且永远不会自行恢复。标准处理：软件复位外设，
 * 引脚临时改为开漏 GPIO，在 SCL 上最多打 9 个时钟让从机释放 SDA，再发 STOP，最后重新初始化。 */
void RecoverBus(I2C_HandleTypeDef *bus)
{
    GPIO_TypeDef *scl_port, *sda_port;
    uint16_t scl, sda;
    if (bus == &hi2c2) { scl_port = GPIOF; scl = GPIO_PIN_1; sda_port = GPIOF; sda = GPIO_PIN_0; }
    else { scl_port = GPIOA; scl = GPIO_PIN_8; sda_port = GPIOC; sda = GPIO_PIN_9; } // I2C3: PA8 SCL, PC9 SDA
    bus->Instance->CR1 |= I2C_CR1_SWRST;
    bus->Instance->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_DeInit(bus); // MspDeInit 释放引脚复用；State 回到 RESET，随后 Init 会重新执行 MspInit
    GPIO_InitTypeDef pin = {};
    pin.Mode = GPIO_MODE_OUTPUT_OD;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_SET);
    pin.Pin = scl;
    HAL_GPIO_Init(scl_port, &pin);
    pin.Pin = sda;
    HAL_GPIO_Init(sda_port, &pin);
    LcTime_DelayUs(5);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(sda_port, sda) == GPIO_PIN_RESET; ++i)
    {
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_RESET);
        LcTime_DelayUs(5);
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET);
        LcTime_DelayUs(5);
    }
    // STOP：SCL 高时 SDA 由低变高。
    HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_RESET);
    LcTime_DelayUs(5);
    HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_RESET);
    LcTime_DelayUs(5);
    HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET);
    LcTime_DelayUs(5);
    HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_SET);
    LcTime_DelayUs(5);
    HAL_I2C_Init(bus);
}
HAL_StatusTypeDef Record(I2C_HandleTypeDef *bus, HAL_StatusTypeDef status, uint16_t addr, uint32_t op)
{
    if (status != HAL_OK)
    {
        State(bus).last_error = (bus->ErrorCode & 0xFFU) | ((uint32_t(status) & 0xFFU) << 8) |
                                ((uint32_t(addr >> 1) & 0xFFU) << 16) | ((op & 0xFU) << 24);
        State(bus).failed = true;
        State(bus).needs_recovery = true; // 下一次 Begin（同一拥有者任务、两笔请求之间）执行总线恢复
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
    if (state.needs_recovery)
    {
        state.needs_recovery = false;
        ++state.recoveries;
        RecoverBus(bus); // 只在请求边界执行：此刻本总线没有进行中的事务
    }
    state.failed = false;
    state.started_us = LcTime_NowUs(); // 记录本次请求起点，单位 us；不清零累计错误数
    state.budget_us = budget_us;
}
extern "C" uint32_t LcBus_RecoveryCount(I2C_HandleTypeDef *bus)
{
    return State(bus).recoveries;
}
extern "C" uint32_t LcBus_LastError(I2C_HandleTypeDef *bus)
{
    return State(bus).last_error;
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
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Master_Transmit(bus, addr, data, length, Timeout(bus, timeout)), addr, 1);
}
extern "C" HAL_StatusTypeDef LcBus_Receive(I2C_HandleTypeDef *bus, uint16_t addr, uint8_t *data,
                                           uint16_t length, uint32_t timeout)
{
    memset(data, 0, length);
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Master_Receive(bus, addr, data, length, Timeout(bus, timeout)), addr, 2);
}
extern "C" HAL_StatusTypeDef LcBus_MemRead(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                           uint16_t reg_size, uint8_t *data, uint16_t length,
                                           uint32_t timeout)
{
    memset(data, 0, length);
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Mem_Read(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)), addr, 3);
}
extern "C" HAL_StatusTypeDef LcBus_MemWrite(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                            uint16_t reg_size, uint8_t *data, uint16_t length,
                                            uint32_t timeout)
{
    if (!Allowed(bus))
        return HAL_TIMEOUT;
    return Record(bus, HAL_I2C_Mem_Write(bus, addr, reg, reg_size, data, length, Timeout(bus, timeout)), addr, 4);
}
