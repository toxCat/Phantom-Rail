#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() must have run */
#include "i2c1_slave.h"
#include "lm51772_regs.h"

/* ---- I2C1 pins: PB6 = SCL (AF4), PB7 = SDA (AF4) ---- */
#define SCL_PIN 6
#define SDA_PIN 7

/* The LM51772 register file. Written by the host over I2C (in the ISR) and by
 * the main loop (status bytes); single-byte accesses are atomic on Cortex-M4. */
static volatile uint8_t  s_regs[256];

/* I2C transaction state (ISR only). */
static volatile uint8_t  s_reg_ptr  = 0;   /* auto-incrementing register pointer */
static volatile uint8_t  s_wr_phase = 0;   /* 0 = next write byte is the pointer  */
static volatile uint8_t  s_dir_tx   = 0;   /* 1 = current transfer is a read      */

/* CLEAR_FAULTS latch + diagnostics. */
static volatile uint8_t  s_clear_faults = 0;
static volatile uint32_t s_addr_hits    = 0;
static volatile uint32_t s_transactions = 0;

/* ---- register access rules (called from the ISR) ---- */
static void reg_write(uint8_t reg, uint8_t val)
{
    switch (reg) {
        case LM_REG_CLEAR_FAULTS: s_clear_faults = 1;          break; /* W-to-clear */
        case LM_REG_STATUS_BYTE:  /* read-only, ignore host write */  break;
        case LM_REG_PD_STATUS0:   /* read-only, ignore host write */  break;
        case LM_REG_VOUT_MSB:     s_regs[reg] = val & 0x0Fu;    break; /* only [3:0] */
        default:                  s_regs[reg] = val;            break;
    }
}

static uint8_t reg_read(uint8_t reg)
{
    switch (reg) {
        case LM_REG_CLEAR_FAULTS: s_clear_faults = 1; return 0x00u;
        case LM_REG_VOUT_MSB:     return s_regs[reg] & 0x0Fu;
        default:                  return s_regs[reg];
    }
}

void i2c1_slave_init(uint8_t addr7)
{
    lm_regs_reset((uint8_t *)s_regs);

    /* DWT running for the main loop's timers (idempotent). */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER  &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->MODER  |=  ((2u << (SCL_PIN * 2)) | (2u << (SDA_PIN * 2)));
    GPIOB->OTYPER |=  (1u << SCL_PIN) | (1u << SDA_PIN);
    GPIOB->OSPEEDR|=  (3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2));
    GPIOB->PUPDR  &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->AFR[0] &= ~((0xFu << (SCL_PIN * 4)) | (0xFu << (SDA_PIN * 4)));
    GPIOB->AFR[0] |=  ((4u   << (SCL_PIN * 4)) | (4u   << (SDA_PIN * 4)));

    uint32_t pclk1 = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    if (ppre1 >= 4u) pclk1 >>= (ppre1 - 3u);
    uint32_t freq_mhz = pclk1 / 1000000U;

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2  = (freq_mhz & I2C_CR2_FREQ)
               | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;
    I2C1->OAR1 = (1u << 14) | ((uint32_t)addr7 << 1);   /* 7-bit; bit14 kept 1 */
    I2C1->CR1 |= I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_ACK;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* I2C1 event interrupt: register-addressed read/write with auto-increment. */
void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & I2C_SR1_ADDR) {                 /* addressed: read SR2 clears ADDR */
        uint32_t sr2 = I2C1->SR2;
        s_dir_tx = (uint8_t)((sr2 & I2C_SR2_TRA) ? 1U : 0U);
        if (!s_dir_tx) s_wr_phase = 0;        /* write: first byte = reg pointer */
        s_addr_hits++;
    }
    if (sr1 & I2C_SR1_RXNE) {                 /* byte from the host (write) */
        uint8_t d = (uint8_t)I2C1->DR;
        if (s_wr_phase == 0) {
            s_reg_ptr = d;
            s_wr_phase = 1;
            if (d == LM_REG_CLEAR_FAULTS) s_clear_faults = 1;   /* access clears */
        } else {
            reg_write(s_reg_ptr, d);
            s_reg_ptr++;
        }
    }
    if (sr1 & I2C_SR1_TXE) {                  /* host read: feed from the pointer */
        I2C1->DR = reg_read(s_reg_ptr);
        s_reg_ptr++;
    }
    if (sr1 & I2C_SR1_STOPF) {                /* end of a write transaction */
        (void)I2C1->SR1;
        I2C1->CR1 |= I2C_CR1_PE;
        s_transactions++;
    }
}

/* I2C1 error interrupt: clear sticky flags. AF is the normal end of a read
 * (host NACKs the last byte). */
void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_AF)   { I2C1->SR1 &= ~I2C_SR1_AF; s_transactions++; }
    if (sr1 & I2C_SR1_BERR)   I2C1->SR1 &= ~I2C_SR1_BERR;
    if (sr1 & I2C_SR1_ARLO)   I2C1->SR1 &= ~I2C_SR1_ARLO;
    if (sr1 & I2C_SR1_OVR)    I2C1->SR1 &= ~I2C_SR1_OVR;
}

uint8_t lm_reg_get(uint8_t reg)          { return s_regs[reg]; }
void    lm_reg_set(uint8_t reg, uint8_t val) { s_regs[reg] = val; }

uint8_t lm_take_clear_faults(void)
{
    uint8_t f = s_clear_faults;
    s_clear_faults = 0;
    return f;
}

void i2c1_slave_diag(uint32_t *addr_hits, uint32_t *transactions)
{
    *addr_hits    = s_addr_hits;
    *transactions = s_transactions;
}
