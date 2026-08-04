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

    /* Derive PCLK1 (APB1) from SystemCoreClock and the APB1 prescaler
     * (RCC->CFGR PPRE1, bits [12:10]: 0xx=/1, 100=/2, 101=/4, 110=/8, 111=/16). */
    uint32_t pclk1 = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    if (ppre1 >= 4u) pclk1 >>= (ppre1 - 3u);
    uint32_t freq_mhz = pclk1 / 1000000U;

    /* Reset, then configure while disabled (PE = 0) */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2   = freq_mhz & I2C_CR2_FREQ;        /* APB1 clock in MHz     */
    I2C1->CCR   = pclk1 / (2U * 100000U);         /* std mode -> 100 kHz   */
    I2C1->TRISE = freq_mhz + 1U;                  /* std-mode max rise 1us */
    I2C1->CR1  |= I2C_CR1_PE;                      /* enable                */
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
