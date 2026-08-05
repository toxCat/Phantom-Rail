#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() must have run */
#include "i2c1_slave.h"
#include "phantom_link.h"

/* ---- I2C1 pins: PB6 = SCL (AF4), PB7 = SDA (AF4) ---- */
#define SCL_PIN 6
#define SDA_PIN 7

/* -------- slave state (all touched from the I2C1 ISRs) -------- */
static volatile uint8_t  s_rxbuf[16];
static volatile uint32_t s_rxlen  = 0;

static volatile uint16_t s_vin    = 0;   /* last good source millivolts   */
static volatile uint8_t  s_cells  = 0;   /* last good cell count           */
static volatile uint8_t  s_flags  = 0;   /* last good flags                */
static volatile uint8_t  s_fresh  = 0;   /* set on new frame, cleared on read */
static volatile uint8_t  s_have   = 0;   /* 1 once any valid frame arrived */
static volatile uint32_t s_stamp  = 0;   /* DWT cycle count at last frame  */

/* Link diagnostics (bench bring-up): how many times our address was matched
 * on the bus, and how many valid frames we parsed. Divergence tells us exactly
 * where the link breaks (see i2c1_slave_diag). */
static volatile uint32_t s_addr_hits = 0;
static volatile uint32_t s_frames    = 0;

/* Transmit side (Task 3): when the master READS us, we stream the current
 * frame the main loop last published. s_dir_tx distinguishes a read (we
 * transmit) from a write (we receive) so STOP doesn't parse a stale rx buf. */
static volatile uint8_t  s_txbuf[8];
static volatile uint8_t  s_txlen  = 0;
static volatile uint8_t  s_txidx  = 0;
static volatile uint8_t  s_dir_tx = 0;   /* 1 = current transfer is a master read */

/* Validate the just-received buffer and latch it if it's a good frame. */
static void slave_commit(void)
{
    uint16_t vin;
    uint8_t  cells, flags;
    if (pr_parse_telemetry((const uint8_t *)s_rxbuf, s_rxlen, &vin, &cells, &flags)) {
        s_vin   = vin;
        s_cells = cells;
        s_flags = flags;
        s_stamp = DWT->CYCCNT;
        s_have  = 1;
        s_fresh = 1;
        s_frames++;
    }
    s_rxlen = 0;
}

void i2c1_slave_init(uint8_t addr7)
{
    /* DWT running for age timestamps (idempotent if the LCD already enabled it) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    /* Peripheral + port clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB6/PB7 -> alternate function (MODER = 10), open-drain, no internal pull */
    GPIOB->MODER  &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->MODER  |=  ((2u << (SCL_PIN * 2)) | (2u << (SDA_PIN * 2)));
    GPIOB->OTYPER |=  (1u << SCL_PIN) | (1u << SDA_PIN);
    GPIOB->OSPEEDR|=  (3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2));
    GPIOB->PUPDR  &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));

    /* AF4 = I2C1 on both pins (both in AFR[0], nibbles 6 and 7) */
    GPIOB->AFR[0] &= ~((0xFu << (SCL_PIN * 4)) | (0xFu << (SDA_PIN * 4)));
    GPIOB->AFR[0] |=  ((4u   << (SCL_PIN * 4)) | (4u   << (SDA_PIN * 4)));

    /* APB1 clock in MHz (needed even in slave mode for the input filter) */
    uint32_t pclk1 = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    if (ppre1 >= 4u) pclk1 >>= (ppre1 - 3u);
    uint32_t freq_mhz = pclk1 / 1000000U;

    /* Reset, then configure while disabled */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2  = (freq_mhz & I2C_CR2_FREQ)
               | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;
    I2C1->OAR1 = (1u << 14) | ((uint32_t)addr7 << 1);  /* 7-bit; bit14 kept 1 */

    I2C1->CR1 |= I2C_CR1_PE;    /* enable, then... */
    I2C1->CR1 |= I2C_CR1_ACK;   /* ...ACK so we acknowledge our address + data */

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* I2C1 event interrupt: address match, byte received, stop. */
void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & I2C_SR1_ADDR) {          /* addressed: clear ADDR (SR1 read + SR2 read) */
        uint32_t sr2 = I2C1->SR2;      /* SR2.TRA: 1 = we transmit (master read) */
        s_dir_tx = (uint8_t)((sr2 & I2C_SR2_TRA) ? 1U : 0U);
        s_rxlen  = 0;
        s_txidx  = 0;
        s_addr_hits++;
    }
    if (sr1 & I2C_SR1_RXNE) {          /* data byte from the master (write) */
        uint8_t d = (uint8_t)I2C1->DR;
        if (s_rxlen < sizeof(s_rxbuf)) s_rxbuf[s_rxlen++] = d;
    }
    if (sr1 & I2C_SR1_TXE) {           /* master read: feed the current frame */
        I2C1->DR = (s_txidx < s_txlen) ? s_txbuf[s_txidx++] : 0x00U;
    }
    if (sr1 & I2C_SR1_STOPF) {         /* master issued STOP: clear (SR1 read + CR1 write) */
        (void)I2C1->SR1;
        I2C1->CR1 |= I2C_CR1_PE;
        if (!s_dir_tx) slave_commit();  /* only a write frame is parsed */
    }
}

/* I2C1 error interrupt: just clear the sticky error flags so we recover. */
void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_AF)   I2C1->SR1 &= ~I2C_SR1_AF;
    if (sr1 & I2C_SR1_BERR) I2C1->SR1 &= ~I2C_SR1_BERR;
    if (sr1 & I2C_SR1_ARLO) I2C1->SR1 &= ~I2C_SR1_ARLO;
    if (sr1 & I2C_SR1_OVR)  I2C1->SR1 &= ~I2C_SR1_OVR;
}

int i2c1_slave_get(uint16_t *vin_mv, uint8_t *cells, uint8_t *flags)
{
    __disable_irq();
    *vin_mv = s_vin;
    *cells  = s_cells;
    *flags  = s_flags;
    int fresh = s_fresh;
    s_fresh = 0;
    __enable_irq();
    return fresh;
}

uint32_t i2c1_slave_age_ms(void)
{
    if (!s_have) return 0xFFFFFFFFu;
    uint32_t cyc = DWT->CYCCNT - s_stamp;          /* unsigned delta wraps fine */
    return cyc / (SystemCoreClock / 1000U);
}

void i2c1_slave_diag(uint32_t *addr_hits, uint32_t *frames)
{
    *addr_hits = s_addr_hits;   /* 32-bit reads are atomic on Cortex-M4 */
    *frames    = s_frames;
}

void i2c1_slave_set_current(uint16_t i_ma, uint8_t cflags)
{
    uint8_t tmp[PR_CUR_FRAME_LEN];
    pr_build_current(tmp, i_ma, cflags);
    __disable_irq();                       /* swap the TX frame atomically */
    for (uint32_t i = 0; i < PR_CUR_FRAME_LEN; i++) s_txbuf[i] = tmp[i];
    s_txlen = PR_CUR_FRAME_LEN;
    __enable_irq();
}
