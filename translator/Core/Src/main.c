#include "stm32f4xx.h"    /* CMSIS device header; SystemInit() runs from startup */
#include "i2c1_master.h"  /* register-addressed I2C1 master (PB6/PB7) */
#include "lm51772_regs.h" /* shared LM51772 register map (../Protocol) */
#include "usart1.h"       /* USART1 (PA9/PA10) to the FC */
#include "msp.h"          /* MSP client: read RC channels from Betaflight */

/*
 * Phantom-Rail -- Translator (Black Pill #1): the I2C1 MASTER / host controller.
 *
 * Role in the rig:
 *   - Reads the source/battery voltage on PA0 (ADC1_IN0) behind a 120k/15k
 *     divider (/9), detects the pack, and runs the sag monitor.
 *   - Drives the N-channel Power FET (PA1) as the physical power-path switch.
 *   - Acts as the host for the LM51772 sim (Black Pill #2): writes its control
 *     registers (VOUT_TARGET, ILIM_THRESHOLD, CONV_EN2) and reads status --
 *     exactly as a flight controller would drive the real IC. The battery view
 *     is passed to the sim's display via Phantom-Rail extension registers.
 *   - Over-current soft-fail: sets ILIM on the sim, reads back CC_OPERATION,
 *     debounces, and cuts the FET.
 *
 * Conventions: register-level CMSIS, no HAL; delays derive from SystemCoreClock.
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

/* ---- Power FET + sag monitor (Task 1 + 2) ----
 * N-channel Power FET gate on PA1, active-high. Per-cell voltage (against the
 * latched cell count) sorts the pack into three bands:
 *   >= 3.6 V .. 4.2 V : charged   -> FET on
 *   3.2 V .. < 3.6 V  : sag/LOW    -> FET on, "LOW" warning on the sim
 *   < 3.2 V           : critical   -> FET off, "(x_X)" on the sim
 * (>4.2 V is treated as over-voltage / bad reading -> FET off.) */
#define PWRFET_PIN   1U    /* PA1 */
#define CELL_CRIT_MV 3200U /* below: critical, cut the FET, show "(x_X)"     */
#define CELL_LOW_MV  3600U /* 3.2-3.6: sag warning, FET stays on            */
#define CELL_HI_MV   4200U /* 3.6-4.2: charged; above: over-voltage guard   */

/* ---- Over-current soft-fail (via the LM51772 model) ----
 * The host writes ILIM_THRESHOLD (CUR_LIMIT_MA) to the sim; the sim compares
 * its ACS709 reading to that limit and raises CC_OPERATION / IOUT_OC. The host
 * reads CC_OPERATION back and cuts the FET only when it stays set continuously
 * for OC_DEBOUNCE_MS (a motor-ramp transient must not nuisance-trip; the LM51772
 * handles fast limiting itself -- this FET is the sustained-fault backstop). The
 * latch holds until the pack is removed (re-arm), which also clears the sim's
 * faults. The translator never senses current directly. */
#define CUR_LIMIT_MA  2000U /* 2.0 A soft limit (5 A in the product)      */
#define OC_DEBOUNCE_MS 1000U /* CC must persist this long to cut          */

/* ---- FC link (Task 2): MSP master on USART1 ----
 * The transmitter's GVAR/pot/switch values reach Betaflight as RC channels; we
 * read them with MSP_RC and map them to the LM51772 controls. The operator's
 * pot rides on one AUX channel (voltage) and a switch on another (enable).
 * Betaflight MSP_RC order is AETR then AUX1.. -> AUX1 = index 4, AUX2 = 5. */
#define MSP_BAUD           115200U
#define MSP_CH_COUNT       8U      /* channels to fetch                    */
#define MSP_CH_VOLTAGE     4U      /* AUX1: output-voltage pot             */
#define MSP_CH_FET         5U      /* AUX2: Power-FET enable switch        */
#define FET_ON_US          1500U   /* AUX >= this us -> enable             */
#define FC_LINK_TIMEOUT_MS 500U    /* no fresh MSP this long -> fail-safe  */
#define VOUT_MIN_MV        3300U   /* pot 1000 us -> 3.3 V (IC lower clamp) */
#define VOUT_MAX_MV        24000U  /* pot 2000 us -> 24 V                  */
#define VOUT_DEFAULT_MV    12000U  /* held until the FC first commands     */

/* Latest values, exposed (volatile, non-static) so they survive -Og and can
 * be watched over SWD. */
volatile uint32_t g_source_mv = 0;   /* source voltage, millivolts */
volatile uint8_t  g_cells     = 0;   /* detected LiPo cell count   */
volatile uint8_t  g_link_ok   = 0;   /* 1 = last I2C1 push was ACKed */
volatile uint16_t g_adc_raw   = 0;   /* last raw ADC count (0..4095) */
volatile uint8_t  g_adc_ok    = 0;   /* 1 = last conversion completed */
volatile uint16_t g_cell_mv   = 0;   /* per-cell voltage, millivolts */
volatile uint8_t  g_fet_on    = 0;   /* Power FET state driven on PA1 */
volatile uint8_t  g_oc_fault  = 0;   /* 1 = over-current cutoff latched */
volatile uint8_t  g_oc_active = 0;   /* 1 = CC asserted, debounce running */
volatile uint8_t  g_pd_status = 0;   /* last USB_PD_STATUS_0 read from sim */
volatile uint16_t g_vout_cmd_mv = VOUT_DEFAULT_MV; /* commanded VOUT (FC pot) */
volatile uint8_t  g_fc_enable = 0;   /* FC-commanded enable (aux switch) */
volatile uint8_t  g_fc_link   = 0;   /* 1 = fresh MSP data from the FC */

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

/* -------- Power FET gate on PA1 (active-high, Task 1) -------- */
static void pwrfet_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;          /* (also on for the ADC) */
    GPIOA->MODER &= ~(3u << (PWRFET_PIN * 2));
    GPIOA->MODER |=  (1u << (PWRFET_PIN * 2));     /* general-purpose output */
    GPIOA->ODR   &= ~(1u << PWRFET_PIN);           /* start OFF (fail-safe) */
}
static inline void pwrfet_set(uint8_t on)
{
    if (on) GPIOA->ODR |=  (1u << PWRFET_PIN);
    else    GPIOA->ODR &= ~(1u << PWRFET_PIN);
}

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

/* One single conversion of channel 0, with a timeout so a mis-configured ADC
 * can never wedge the control loop (and silence the I2C link). */
static uint16_t adc_read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = 2000U * (SystemCoreClock / 1000000U);   /* 2 ms budget */
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if ((DWT->CYCCNT - start) > ticks) { g_adc_ok = 0; return 0; }
    }
    g_adc_ok  = 1;
    g_adc_raw = (uint16_t)ADC1->DR;  /* reading DR clears EOC */
    return g_adc_raw;
}

int main(void)
{
    SystemCoreClockUpdate();         /* make timing track the real clock tree */
    dwt_init();
    led_init();
    adc_init();
    pwrfet_init();                   /* Power FET gate on PA1, starts OFF */
    i2c1_master_init();              /* link to the LM51772 sim (PB6/PB7) */
    usart1_init(MSP_BAUD);           /* MSP link to the FC (PA9/PA10) */

    for (;;) {
        /* 1. Read the source voltage behind the /9 divider. */
        uint16_t raw    = adc_read();
        uint32_t mv_pin = (uint32_t)raw * VREF_MV / ADC_MAX;
        uint32_t src_mv = mv_pin * VDIV_NUM / VDIV_DEN;
        g_source_mv     = src_mv;

        /* 2. Detect the pack, latching the cell count at plug-in. */
        uint8_t batt = 0;
        if (src_mv >= SRC_MIN_MV) {
            if (g_cells == 0) g_cells = (uint8_t)(src_mv / CELL_MAX_MV) + 1U;
            batt |= PR_BATT_VALID;
        } else {
            g_cells = 0;                  /* disconnected -> re-detect next pack */
        }
        uint8_t cells = g_cells;

        /* 3. Sag flags + battery-ok veto. The battery no longer commands the
         *    enable (the FC does) -- it only VETOes: a critical pack disables
         *    the output regardless of what the transmitter asks. */
        uint16_t per_cell = (cells > 0) ? (uint16_t)(src_mv / cells) : 0U;
        g_cell_mv = per_cell;

        uint8_t batt_ok = 0;
        if (batt & PR_BATT_VALID) {
            if (per_cell < CELL_CRIT_MV)      batt |= PR_BATT_CRIT;      /* veto */
            else if (per_cell < CELL_LOW_MV) { batt |= PR_BATT_LOW; batt_ok = 1; }
            else if (per_cell <= CELL_HI_MV)   batt_ok = 1;
        }

        /* 3b. Poll the FC (MSP_RC) for the transmitter's pot/switch commands. */
        uint16_t ch[MSP_CH_COUNT];
        int nch = msp_read_rc(ch, MSP_CH_COUNT);
        static uint32_t fc_stamp = 0;
        if (nch > (int)MSP_CH_FET) {
            g_fc_link = 1; fc_stamp = DWT->CYCCNT;
            uint16_t us = ch[MSP_CH_VOLTAGE];
            if (us < 1000U) us = 1000U;
            if (us > 2000U) us = 2000U;
            g_vout_cmd_mv = (uint16_t)(VOUT_MIN_MV +
                (uint32_t)(us - 1000U) * (VOUT_MAX_MV - VOUT_MIN_MV) / 1000U);
            g_fc_enable = (ch[MSP_CH_FET] >= FET_ON_US) ? 1U : 0U;
        } else if (g_fc_link &&
                   (DWT->CYCCNT - fc_stamp) / (SystemCoreClock / 1000U) >= FC_LINK_TIMEOUT_MS) {
            g_fc_link = 0;                        /* link lost -> fail-safe */
        }
        uint8_t fc_enable = g_fc_link ? g_fc_enable : 0U;

        /* Final enable: the FC commands it, the battery can veto it. */
        uint8_t enable = (uint8_t)(fc_enable && batt_ok);

        /* 4. Drive the LM51772 registers on the sim (host writes). */
        uint16_t vin16 = (src_mv > 65535U) ? 65535U : (uint16_t)src_mv;
        uint16_t vcode = lm_vout_from_mv(g_vout_cmd_mv, 1);  /* div20=1 (reset) */
        int ok = 1;
        /* 4a. Battery view for the sim's display (extension registers). */
        uint8_t ext[5] = { PR_EXT_VIN_LSB, (uint8_t)vin16, (uint8_t)(vin16 >> 8),
                           cells, batt };
        ok &= i2c1_master_write(LM_ADDR, ext, 5);
        /* 4b. Current limit. */
        uint8_t ilim[2] = { LM_REG_ILIM, lm_ilim_from_ma(CUR_LIMIT_MA) };
        ok &= i2c1_master_write(LM_ADDR, ilim, 2);
        /* 4c. Output voltage target (LSB then MSB, auto-increment). */
        uint8_t vt[3] = { LM_REG_VOUT_LSB, (uint8_t)(vcode & 0xFFu),
                          (uint8_t)((vcode >> 8) & 0x0Fu) };
        ok &= i2c1_master_write(LM_ADDR, vt, 3);
        /* 4d. Enable the power stage (CONV_EN2). */
        uint8_t en[2] = { LM_REG_PD_CONTROL0, (uint8_t)(enable ? LM_CONV_EN2 : 0U) };
        ok &= i2c1_master_write(LM_ADDR, en, 2);
        g_link_ok = (uint8_t)ok;
        if (!ok) i2c1_master_recover();   /* self-heal a stuck/booting link */

        /* 5. Read CC_OPERATION back and debounce the over-current cutoff. */
        uint8_t pd0 = 0;
        if (i2c1_master_read_reg(LM_ADDR, LM_REG_PD_STATUS0, &pd0, 1)) g_pd_status = pd0;
        else                                                          i2c1_master_recover();
        uint8_t cc = (pd0 & LM_CC_OPERATION) ? 1U : 0U;

        static uint32_t oc_since = 0;
        if (!(batt & PR_BATT_VALID)) {
            g_oc_fault = 0; g_oc_active = 0;         /* pack removed -> re-arm */
        } else if (cc) {
            if (!g_oc_active) { g_oc_active = 1; oc_since = DWT->CYCCNT; }
            else if ((DWT->CYCCNT - oc_since) / (SystemCoreClock / 1000U) >= OC_DEBOUNCE_MS) {
                g_oc_fault = 1;                       /* sustained CC -> soft fail */
            }
        } else {
            g_oc_active = 0;                          /* dropped below limit -> reset */
        }

        /* 5b. On the transition to no-pack, clear the sim's latched faults. */
        static uint8_t was_valid = 0;
        if (!(batt & PR_BATT_VALID) && was_valid) {
            uint8_t cf[2] = { LM_REG_CLEAR_FAULTS, 0x00u };   /* access clears */
            i2c1_master_write(LM_ADDR, cf, 2);
        }
        was_valid = (batt & PR_BATT_VALID) ? 1U : 0U;

        /* 6. Drive the physical Power FET: enabled AND not over-current. */
        uint8_t fet_on = (uint8_t)(enable && !g_oc_fault);
        pwrfet_set(fet_on);
        g_fet_on = fet_on;

        led_toggle();                /* heartbeat = alive + sampling */
        delay_ms(100);
    }
}
