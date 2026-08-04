#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() runs from startup */

/*
 * Phantom-Rail -- Translator (Black Pill #1): the I2C1 MASTER.
 *
 * Role in the rig:
 *   - Reads the source/battery voltage on PA0 (ADC1_IN0) behind a 120k/15k
 *     divider (/9, ~29 V full scale) -- the "VREF" the sim board displays.
 *   - Talks to the flight controller over USART1 (PA9/PA10, MSP).
 *   - Drives the LM51772 sim board (Black Pill #2) as I2C1 MASTER on PB6/PB7,
 *     and gates it via an enable/disable GPIO.
 *
 * Status: SKELETON. This pass brings up the ADC source-sense reader and a
 * PC13 liveness heartbeat so the board is testable on the bench. Still TODO:
 * USART1/MSP link to the FC, the I2C1 master transfer that pushes the reading
 * to the slave, UVLO/hysteresis, and the static-selector truth table.
 *
 * Conventions (see PROJECT.md): register-level CMSIS, no HAL; delays derive
 * from SystemCoreClock; default HSI (16 MHz) is fine.
 */

/* ---- Source-sense scaling: 120k / 15k = /9 divider, 12-bit ADC ---- */
#define VDIV_MUL 9U        /* V_source = V_pin * 9                       */
#define VREF_MV  3300U     /* ADC full-scale reference (VDDA) millivolts */
#define ADC_MAX  4095U     /* 12-bit right-aligned                       */

/* Latest source reading in millivolts. Exposed (volatile, non-static) so it
 * survives -Og and can be watched over SWD while there is no other output. */
volatile uint32_t g_source_mv = 0;

/* -------- microsecond delays via the DWT cycle counter -------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

static void delay_ms(uint32_t ms) { while (ms--) delay_us(1000U); }

/* -------- on-board LED heartbeat (PC13, active low) -------- */
static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (13 * 2));
    GPIOC->MODER |=  (1u << (13 * 2));   /* general-purpose output */
    GPIOC->ODR   |=  (1u << 13);         /* LED off (cathode on PC13) */
}
static inline void led_toggle(void) { GPIOC->ODR ^= (1u << 13); }

/* -------- ADC1_IN0 on PA0 (source sense) -------- */
static void adc_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA0 -> analog mode (MODER = 11), no pull */
    GPIOA->MODER |=  (3u << (0 * 2));
    GPIOA->PUPDR &= ~(3u << (0 * 2));

    /* 12-bit (CR1.RES = 00, reset default). Longest sample time for the
     * high-impedance 120k top resistor: SMP0 = 480 cycles. The ADC prescaler
     * stays at its /2 reset default (8 MHz from a 16 MHz PCLK2, within spec). */
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;   /* channel 0 sample time = 480 cycles */
    ADC1->SQR1   = 0;                /* L = 0 -> one conversion in the sequence */
    ADC1->SQR3   = 0;                /* 1st (only) conversion = channel 0 (PA0) */
    ADC1->CR2   |= ADC_CR2_ADON;     /* power up the ADC */
    delay_us(10);                    /* tSTAB settling */
}

/* One blocking single conversion of channel 0. */
static uint16_t adc_read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) { }
    return (uint16_t)ADC1->DR;       /* reading DR clears EOC */
}

int main(void)
{
    SystemCoreClockUpdate();         /* make timing track the real clock tree */
    dwt_init();
    led_init();
    adc_init();

    for (;;) {
        uint16_t raw    = adc_read();
        uint32_t mv_pin = (uint32_t)raw * VREF_MV / ADC_MAX;
        g_source_mv     = mv_pin * VDIV_MUL;   /* undo the /9 divider */

        led_toggle();                /* ~2 Hz heartbeat = alive + sampling */
        delay_ms(250);
    }
}
