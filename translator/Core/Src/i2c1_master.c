#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() must have run */
#include "i2c1_master.h"

/* ---- I2C1 pins: PB6 = SCL (AF4), PB7 = SDA (AF4) ---- */
#define SCL_PIN 6
#define SDA_PIN 7

/* Per-transfer timeout so a missing / unpowered slave fails fast instead of
 * hanging the master's control loop. */
#define I2C_TIMEOUT_US 5000U

/* Wait for an SR1 flag with a timeout; also bail on ACK failure (NACK).
 * Returns 1 if the flag came up, 0 on NACK or timeout. */
static uint8_t wait_sr1(uint32_t flag)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);
    for (;;) {
        uint32_t sr1 = I2C1->SR1;
        if (sr1 & flag)       return 1;
        if (sr1 & I2C_SR1_AF) return 0;   /* slave did not ACK       */
        if ((DWT->CYCCNT - start) > ticks) return 0;  /* dead/stuck bus */
    }
}

/* (Re)configure the I2C1 peripheral registers. The SWRST clears a stuck BUSY
 * or any latched error, so this doubles as the bus-recovery routine. Timing is
 * derived from PCLK1 (APB1 prescaler PPRE1 = RCC->CFGR[12:10]). */
static void periph_config(void)
{
    uint32_t pclk1 = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    if (ppre1 >= 4u) pclk1 >>= (ppre1 - 3u);
    uint32_t freq_mhz = pclk1 / 1000000U;

    I2C1->CR1 = I2C_CR1_SWRST;                    /* hold in reset...      */
    I2C1->CR1 = 0;                                /* ...release            */
    I2C1->CR2   = freq_mhz & I2C_CR2_FREQ;        /* APB1 clock in MHz     */
    I2C1->CCR   = pclk1 / (2U * 100000U);         /* std mode -> 100 kHz   */
    I2C1->TRISE = freq_mhz + 1U;                  /* std-mode max rise 1us */
    I2C1->CR1  |= I2C_CR1_PE;                      /* enable                */
}

void i2c1_master_init(void)
{
    /* Peripheral + port clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB6/PB7 -> alternate function (MODER = 10) */
    GPIOB->MODER &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->MODER |=  ((2u << (SCL_PIN * 2)) | (2u << (SDA_PIN * 2)));

    /* Open-drain (I2C requires it; external pull-ups on the link) */
    GPIOB->OTYPER |= (1u << SCL_PIN) | (1u << SDA_PIN);

    /* High speed, no internal pull (the link carries its own pull-ups) */
    GPIOB->OSPEEDR |= (3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2));
    GPIOB->PUPDR   &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));

    /* Both pins are AF4 = I2C1, and both live in AFR[0] (nibbles 6 and 7). */
    GPIOB->AFR[0] &= ~((0xFu << (SCL_PIN * 4)) | (0xFu << (SDA_PIN * 4)));
    GPIOB->AFR[0] |=  ((4u   << (SCL_PIN * 4)) | (4u   << (SDA_PIN * 4)));

    periph_config();
}

/* Recover a wedged bus (e.g. a stuck BUSY after the slave NACKed our early
 * start-up frames before it was listening). Cheap: SWRST + re-enable, no GPIO
 * churn. Call after a failed transfer so the next attempt starts clean. */
void i2c1_master_recover(void)
{
    periph_config();
}

int i2c1_master_write(uint8_t addr7, const uint8_t *data, uint32_t len)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);

    /* Wait for the bus to be free */
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((DWT->CYCCNT - start) > ticks) return 0;
    }

    /* START (EV5: SB) */
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }

    /* Address + write; SB cleared by the SR1 read in wait_sr1 then this DR write */
    I2C1->DR = (uint8_t)(addr7 << 1);
    if (!wait_sr1(I2C_SR1_ADDR)) {                /* EV6, or AF on no slave */
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0;
    }
    (void)I2C1->SR1; (void)I2C1->SR2;             /* clear ADDR (read SR1,SR2) */

    /* Data bytes */
    for (uint32_t i = 0; i < len; i++) {
        if (!wait_sr1(I2C_SR1_TXE)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }
        I2C1->DR = data[i];
    }
    /* Wait for the last byte to fully shift out (EV8_2: BTF) before STOP */
    if (!wait_sr1(I2C_SR1_BTF)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1;
}

int i2c1_master_read(uint8_t addr7, uint8_t *buf, uint32_t n)
{
    if (n == 0) return 0;

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((DWT->CYCCNT - start) > ticks) return 0;
    }

    I2C1->CR1 |=  I2C_CR1_ACK;                    /* ACK incoming bytes    */
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |=  I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }

    I2C1->DR = (uint8_t)((addr7 << 1) | 1U);      /* address + read        */
    if (!wait_sr1(I2C_SR1_ADDR)) {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0;
    }

    /* The last byte must be NACKed, so the tail differs by count. See RM0383
     * "Master receiver" (EV6_1 / EV7 / EV7_1). */
    if (n == 1U) {
        I2C1->CR1 &= ~I2C_CR1_ACK;                /* NACK the only byte    */
        (void)I2C1->SR1; (void)I2C1->SR2;         /* clear ADDR            */
        I2C1->CR1 |= I2C_CR1_STOP;
        if (!wait_sr1(I2C_SR1_RXNE)) return 0;
        buf[0] = (uint8_t)I2C1->DR;
    } else if (n == 2U) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |=  I2C_CR1_POS;                 /* NACK falls on byte 2 */
        (void)I2C1->SR1; (void)I2C1->SR2;         /* clear ADDR            */
        if (!wait_sr1(I2C_SR1_BTF)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }
        I2C1->CR1 |= I2C_CR1_STOP;
        buf[0] = (uint8_t)I2C1->DR;
        buf[1] = (uint8_t)I2C1->DR;
    } else {
        (void)I2C1->SR1; (void)I2C1->SR2;         /* clear ADDR            */
        uint32_t i = 0;
        while (n - i > 3U) {                       /* ACK up to N-3         */
            if (!wait_sr1(I2C_SR1_RXNE)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }
            buf[i++] = (uint8_t)I2C1->DR;
        }
        /* 3 bytes left: BTF => DataN-2 in DR, DataN-1 in shift register */
        if (!wait_sr1(I2C_SR1_BTF)) { I2C1->CR1 |= I2C_CR1_STOP; return 0; }
        I2C1->CR1 &= ~I2C_CR1_ACK;                /* NACK DataN            */
        buf[i++] = (uint8_t)I2C1->DR;             /* DataN-2               */
        I2C1->CR1 |= I2C_CR1_STOP;
        buf[i++] = (uint8_t)I2C1->DR;             /* DataN-1               */
        if (!wait_sr1(I2C_SR1_RXNE)) return 0;
        buf[i++] = (uint8_t)I2C1->DR;             /* DataN                 */
    }

    I2C1->CR1 |=  I2C_CR1_ACK;                    /* restore defaults      */
    I2C1->CR1 &= ~I2C_CR1_POS;
    return 1;
}
