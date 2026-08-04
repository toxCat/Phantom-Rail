#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() runs from startup */
#include "i2c1_master.h"  /* inter-board I2C1 link (PB6/PB7) */
#include "phantom_link.h" /* shared wire format (../Protocol) */

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

/* ---- Source-sense scaling ----
 * Resistor divider on PA0: R_top = 120k, R_bottom = 15k. The pin sees
 * V_source * 15/(120+15) = V_source / 9, so V_source = V_pin * 135/15.
 * Expressed as the real resistor values so it is trivial to re-trim if you
 * measure the actual parts (bench check: 22.94 V pack read 2.528 V at PA0
 * -> 22.75 V computed, ~0.8% low, well inside resistor tolerance). */
#define VDIV_NUM 135U      /* R_top + R_bottom, in k                     */
#define VDIV_DEN 15U       /* R_bottom, in k                            */
#define VREF_MV  3300U     /* ADC full-scale reference (VDDA) millivolts */
#define ADC_MAX  4095U     /* 12-bit right-aligned                       */

/* ---- Pack detection (LiPo) ----
 * Cell count via the Betaflight convention: the smallest S whose max-charge
 * voltage (CELL_MAX_MV per cell) still covers the reading. Assumes a charged
 * or storage pack -- a deeply flat pack can read one cell low, same caveat as
 * every FC that guesses cells from bulk voltage. */
#define CELL_MAX_MV 4300U  /* per-cell ceiling used for the S estimate   */
#define SRC_MIN_MV  5000U  /* below this: nothing meaningful is connected */

/* Latest values, exposed (volatile, non-static) so they survive -Og and can
 * be watched over SWD. */
volatile uint32_t g_source_mv = 0;   /* source voltage, millivolts */
volatile uint8_t  g_cells     = 0;   /* detected LiPo cell count   */
volatile uint8_t  g_link_ok   = 0;   /* 1 = last I2C1 push was ACKed */

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
    i2c1_master_init();              /* link to the LM51772 sim (PB6/PB7) */

    for (;;) {
        /* 1. Read the source voltage behind the /9 divider. */
        uint16_t raw    = adc_read();
        uint32_t mv_pin = (uint32_t)raw * VREF_MV / ADC_MAX;
        uint32_t src_mv = mv_pin * VDIV_NUM / VDIV_DEN;
        g_source_mv     = src_mv;

        /* 2. Detect the pack. */
        uint8_t cells = 0;
        uint8_t flags = 0;
        if (src_mv >= SRC_MIN_MV) {
            cells  = (uint8_t)(src_mv / CELL_MAX_MV) + 1U;
            flags |= PR_FLAG_VALID;
        }
        g_cells = cells;

        /* 3. Push the telemetry frame to the sim over I2C1. u16 caps at
         *    65.535 V, far above any pack we sense. */
        uint16_t vin16 = (src_mv > 65535U) ? 65535U : (uint16_t)src_mv;
        uint8_t  frame[PR_FRAME_LEN];
        pr_build_telemetry(frame, vin16, cells, flags);
        g_link_ok = (uint8_t)i2c1_master_write(PR_I2C_ADDR, frame, PR_FRAME_LEN);

        led_toggle();                /* ~2 Hz heartbeat = alive + sampling */
        delay_ms(250);
    }
}
