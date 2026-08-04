#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() must have run */
#include "lcd1602.h"

/* ---- Pin assignment: PB10 = I2C2_SCL (AF4), PB3 = I2C2_SDA (AF9) ----
 * The LCD lives on the F411's hardware I2C2 peripheral so that I2C1
 * (PB6/PB7) stays free for the inter-black-pill slave link. Mind the AF
 * asymmetry: SCL is AF4 but SDA is AF9 -- setting both to AF4 leaves SDA
 * clocking but never ACKing. */
#define SCL_PIN 10
#define SDA_PIN 3

/* ---- PCF8574 -> HD44780 bit mapping (standard backpack layout) ---- */
#define LCD_RS 0x01   /* P0 */
#define LCD_RW 0x02   /* P1 (held 0; we only write) */
#define LCD_EN 0x04   /* P2 */
#define LCD_BL 0x08   /* P3 backlight on */
/* P4..P7 carry the 4-bit data nibble */

/* Per-transfer timeout so a missing / mis-addressed backpack fails fast
 * (blank screen) instead of hanging the MCU. */
#define I2C_TIMEOUT_US 10000U

/* Active 7-bit backpack address. Starts at the compile-time default but
 * lcd_init() overwrites it with whatever actually ACKs on the bus. */
static uint8_t s_addr = LCD_ADDR;

/* -------- microsecond delay via the DWT cycle counter -------- */
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

void lcd_delay_ms(uint32_t ms) { delay_ms(ms); }

/* -------- hardware I2C2 bring-up (register level, no HAL) --------
 * Standard-mode 100 kHz. Timing is derived from PCLK1 so it stays correct
 * if the PLL is brought up (provided SystemCoreClockUpdate() has run). */
static void i2c2_init(void)
{
    /* Peripheral + port clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    /* PB10/PB3 -> alternate function (MODER = 10) */
    GPIOB->MODER &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->MODER |=  ((2u << (SCL_PIN * 2)) | (2u << (SDA_PIN * 2)));

    /* Open-drain (I2C requires it; backpack supplies the pull-ups) */
    GPIOB->OTYPER |= (1u << SCL_PIN) | (1u << SDA_PIN);

    /* High speed, no internal pull (the PCF8574 board pulls both lines up) */
    GPIOB->OSPEEDR |= (3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2));
    GPIOB->PUPDR   &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));

    /* Alternate-function select -- note the asymmetry: SCL=AF4, SDA=AF9.
     * PB10 lives in AFR[1] (nibble 10-8), PB3 in AFR[0] (nibble 3). */
    GPIOB->AFR[1] &= ~(0xFu << ((SCL_PIN - 8) * 4));
    GPIOB->AFR[1] |=  (4u   << ((SCL_PIN - 8) * 4));   /* AF4  = I2C2_SCL */
    GPIOB->AFR[0] &= ~(0xFu << (SDA_PIN * 4));
    GPIOB->AFR[0] |=  (9u   << (SDA_PIN * 4));         /* AF9  = I2C2_SDA */

    /* Derive PCLK1 (APB1) from SystemCoreClock and the APB1 prescaler
     * (RCC->CFGR PPRE1, bits [12:10]: 0xx=/1, 100=/2, 101=/4, 110=/8, 111=/16). */
    uint32_t pclk1 = SystemCoreClock;
    uint32_t ppre1 = (RCC->CFGR >> 10) & 0x7u;
    if (ppre1 >= 4u) pclk1 >>= (ppre1 - 3u);
    uint32_t freq_mhz = pclk1 / 1000000U;

    /* Reset the peripheral, then configure it while disabled (PE=0) */
    I2C2->CR1 = I2C_CR1_SWRST;
    I2C2->CR1 = 0;

    I2C2->CR2   = freq_mhz & I2C_CR2_FREQ;        /* APB1 clock in MHz     */
    I2C2->CCR   = pclk1 / (2U * 100000U);         /* std mode -> 100 kHz   */
    I2C2->TRISE = freq_mhz + 1U;                  /* std-mode max rise 1us */
    I2C2->CR1  |= I2C_CR1_PE;                      /* enable                */
}

/* Wait for an SR1 flag with a timeout; also bail on an ACK failure (NACK).
 * Returns 1 if the flag came up, 0 on NACK or timeout. */
static uint8_t i2c_wait_sr1(uint32_t flag)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);
    for (;;) {
        uint32_t sr1 = I2C2->SR1;
        if (sr1 & flag)          return 1;
        if (sr1 & I2C_SR1_AF)    return 0;   /* slave did not ACK      */
        if ((DWT->CYCCNT - start) > ticks) return 0;  /* dead/stuck bus */
    }
}

/* Write one byte to the PCF8574 output port over I2C2.
 * Returns 1 on ACK/success, 0 on NACK/timeout (never hangs). */
static uint8_t pcf_write(uint8_t data)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);

    /* Wait for the bus to be free */
    while (I2C2->SR2 & I2C_SR2_BUSY) {
        if ((DWT->CYCCNT - start) > ticks) return 0;
    }

    /* START (EV5: SB set) */
    I2C2->CR1 |= I2C_CR1_START;
    if (!i2c_wait_sr1(I2C_SR1_SB)) { I2C2->CR1 |= I2C_CR1_STOP; return 0; }

    /* Address + write. SB is cleared by the SR1 read above followed by the
     * DR write here. */
    I2C2->DR = (uint8_t)(s_addr << 1);
    if (!i2c_wait_sr1(I2C_SR1_ADDR)) {           /* EV6, or AF on wrong addr */
        I2C2->SR1 &= ~I2C_SR1_AF;                /* clear the NACK flag      */
        I2C2->CR1 |= I2C_CR1_STOP;
        return 0;
    }
    (void)I2C2->SR1; (void)I2C2->SR2;            /* clear ADDR (read SR1,SR2) */

    /* Data byte (EV8_1: TXE), then wait for it to shift out (EV8_2: BTF) */
    if (!i2c_wait_sr1(I2C_SR1_TXE)) { I2C2->CR1 |= I2C_CR1_STOP; return 0; }
    I2C2->DR = data;
    if (!i2c_wait_sr1(I2C_SR1_BTF)) { I2C2->CR1 |= I2C_CR1_STOP; return 0; }

    /* STOP */
    I2C2->CR1 |= I2C_CR1_STOP;
    return 1;
}

/* Address-only probe: START, addr+W, look for ACK, STOP. Returns 1 if a
 * slave acknowledges its 7-bit address, 0 on NACK / timeout. Used to sniff
 * out which PCF8574 variant is on the bus without writing anything to it. */
static uint8_t i2c_probe(uint8_t addr7)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = I2C_TIMEOUT_US * (SystemCoreClock / 1000000U);
    while (I2C2->SR2 & I2C_SR2_BUSY) {
        if ((DWT->CYCCNT - start) > ticks) return 0;
    }

    I2C2->CR1 |= I2C_CR1_START;
    if (!i2c_wait_sr1(I2C_SR1_SB)) { I2C2->CR1 |= I2C_CR1_STOP; return 0; }

    I2C2->DR = (uint8_t)(addr7 << 1);
    uint8_t ack = i2c_wait_sr1(I2C_SR1_ADDR);
    if (ack) { (void)I2C2->SR1; (void)I2C2->SR2; }  /* clear ADDR */
    else     { I2C2->SR1 &= ~I2C_SR1_AF; }          /* clear NACK */

    I2C2->CR1 |= I2C_CR1_STOP;
    return ack;
}

/* -------- HD44780 in 4-bit mode via the PCF8574 -------- */
static void lcd_write_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t d = (uint8_t)(nib << 4) | LCD_BL | (rs ? LCD_RS : 0);
    pcf_write(d | LCD_EN);     /* EN high, data valid */
    delay_us(1);
    pcf_write(d & (uint8_t)~LCD_EN);  /* EN low -> latch */
    delay_us(50);
}

static void lcd_send(uint8_t val, uint8_t rs)
{
    lcd_write_nibble((uint8_t)(val >> 4), rs);
    lcd_write_nibble((uint8_t)(val & 0x0F), rs);
}

static void lcd_cmd(uint8_t c) { lcd_send(c, 0); }

void lcd_putc(char c) { lcd_send((uint8_t)c, 1); }

void lcd_print(const char *s) { while (*s) lcd_putc(*s++); }

void lcd_clear(void) { lcd_cmd(0x01); delay_ms(2); }

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t base[2] = { 0x00, 0x40 };
    lcd_cmd((uint8_t)(0x80 | (base[row & 1] + col)));
}

uint8_t lcd_init(void)
{
    dwt_init();
    i2c2_init();

    /* Auto-detect the backpack address: try the compile-time default first,
     * then the two standard PCF8574 / PCF8574A addresses. Whichever ACKs
     * becomes the active address; return 0 if the bus stays silent. */
    static const uint8_t cand[] = { LCD_ADDR, 0x27, 0x3F };
    uint8_t found = 0;
    for (unsigned i = 0; i < sizeof(cand); i++) {
        if (i2c_probe(cand[i])) { s_addr = cand[i]; found = cand[i]; break; }
    }
    if (!found) return 0;   /* nobody home -- skip the (futile) panel init */

    delay_ms(50);   /* HD44780 power-on settling */

    /* 4-bit init incantation (datasheet figure) */
    lcd_write_nibble(0x03, 0); delay_ms(5);
    lcd_write_nibble(0x03, 0); delay_us(150);
    lcd_write_nibble(0x03, 0); delay_us(150);
    lcd_write_nibble(0x02, 0); delay_us(150);   /* switch to 4-bit */

    lcd_cmd(0x28);              /* function set: 4-bit, 2 lines, 5x8 */
    lcd_cmd(0x08);              /* display off */
    lcd_cmd(0x01); delay_ms(2); /* clear */
    lcd_cmd(0x06);              /* entry mode: increment, no shift */
    lcd_cmd(0x0C);              /* display on, cursor off, blink off */

    return found;
}
