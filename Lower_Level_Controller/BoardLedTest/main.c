/*
 * RoboMaster C board: standalone RGB LED smoke test.
 *
 * This image deliberately does not use HAL, FreeRTOS, USB, UART, I2C, SPI,
 * PWM expansion boards, pressure sensors, or the external crystal.  It is
 * intended to answer one question only: can the STM32F407 execute code and
 * drive the on-board RGB LED?
 *
 * LED mapping from the RoboMaster C board configuration:
 *   PH10 = LED_B, PH11 = LED_G, PH12 = LED_R.
 * The board LED is driven high in the board project, so one selected bit is
 * set high and the other two are set low.
 */

#include <stdint.h>

#define RCC_BASE       0x40023800UL
#define RCC_AHB1ENR    (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_AHB1ENR_GPIOHEN (1UL << 7)

#define GPIOH_BASE     0x40021C00UL
#define GPIOH_MODER    (*(volatile uint32_t *)(GPIOH_BASE + 0x00UL))
#define GPIOH_OTYPER   (*(volatile uint32_t *)(GPIOH_BASE + 0x04UL))
#define GPIOH_OSPEEDR  (*(volatile uint32_t *)(GPIOH_BASE + 0x08UL))
#define GPIOH_PUPDR    (*(volatile uint32_t *)(GPIOH_BASE + 0x0CUL))
#define GPIOH_ODR      (*(volatile uint32_t *)(GPIOH_BASE + 0x14UL))

#define LED_B          (1UL << 10)
#define LED_G          (1UL << 11)
#define LED_R          (1UL << 12)
#define LED_MASK       (LED_B | LED_G | LED_R)

static void Delay(volatile uint32_t count)
{
    while (count-- != 0U)
        __asm volatile("nop");
}

static void LedInit(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    (void)RCC_AHB1ENR; /* complete the peripheral-clock write before GPIO access */

    GPIOH_MODER &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)) | (3UL << (12U * 2U)));
    GPIOH_MODER |=  ((1UL << (10U * 2U)) | (1UL << (11U * 2U)) | (1UL << (12U * 2U)));
    GPIOH_OTYPER &= ~LED_MASK; /* push-pull */
    GPIOH_OSPEEDR &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)) | (3UL << (12U * 2U)));
    GPIOH_PUPDR &= ~((3UL << (10U * 2U)) | (3UL << (11U * 2U)) | (3UL << (12U * 2U)));
    GPIOH_ODR &= ~LED_MASK;
}

static void Show(uint32_t led)
{
    GPIOH_ODR = (GPIOH_ODR & ~LED_MASK) | (led & LED_MASK);
}

int main(void)
{
    LedInit();
    for (;;)
    {
        Show(LED_R);
        Delay(12000000U);
        Show(LED_G);
        Delay(12000000U);
        Show(LED_B);
        Delay(12000000U);
        Show(0U);
        Delay(6000000U);
    }
}
