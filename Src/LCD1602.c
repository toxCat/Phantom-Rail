#include "stm32f4xx.h"   /* CMSIS device header; SystemInit() must have run */
#include "lcd1602.h"

/* ---- Pin assignment: PB8 = SCL, PB9 = SDA (open-drain, bit-banged) ---- */
#define SCL_PIN 8
#define SDA_PIN 9

/* ---- PCF8574 -> HD44780 bit mapping (standard backpack layout) ---- */
#define LCD_RS 0x01   /* P0 */
#define LCD_RW 0x02   /* P1 (held 0; we only write) */
#define LCD_EN 0x04   /* P2 */
#define LCD_BL 0x08   /* P3 backlight on */
/* P4..P7 carry the 4-bit data nibble */

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

/* -------- open-drain line control on GPIOB --------
 * Open-drain: writing 1 to the ODR releases the line (external pull-up
 * drives it high); writing 0 actively pulls it low. IDR still reflects the
 * real pin level in output mode, so we can sample SDA for ACK. */
static inline void scl_high(void) { GPIOB->BSRR = (1u << SCL_PIN); }
static inline void scl_low (void) { GPIOB->BSRR = (1u << (SCL_PIN + 16)); }
static inline void sda_high(void) { GPIOB->BSRR = (1u << SDA_PIN); }
static inline void sda_low (void) { GPIOB->BSRR = (1u << (SDA_PIN + 16)); }
static inline uint8_t sda_read(void) { return (GPIOB->IDR >> SDA_PIN) & 1u; }

#define I2C_HALF() delay_us(5)   /* ~100 kHz */

static void i2c_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* MODER = 01 (output) for both pins */
    GPIOB->MODER &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->MODER |=  ((1u << (SCL_PIN * 2)) | (1u << (SDA_PIN * 2)));

    /* OTYPER = 1 (open-drain) */
    GPIOB->OTYPER |= (1u << SCL_PIN) | (1u << SDA_PIN);

    /* PUPDR = 01 (internal pull-up as a backstop; backpack also pulls up) */
    GPIOB->PUPDR &= ~((3u << (SCL_PIN * 2)) | (3u << (SDA_PIN * 2)));
    GPIOB->PUPDR |=  ((1u << (SCL_PIN * 2)) | (1u << (SDA_PIN * 2)));

    scl_high();
    sda_high();
}

static void i2c_start(void)
{
    sda_high(); scl_high(); I2C_HALF();
    sda_low();  I2C_HALF();
    scl_low();  I2C_HALF();
}

static void i2c_stop(void)
{
    sda_low();  I2C_HALF();
    scl_high(); I2C_HALF();
    sda_high(); I2C_HALF();
}

/* Returns 1 if the slave ACKed, 0 otherwise. */
static uint8_t i2c_write_byte(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) sda_high(); else sda_low();
        b <<= 1;
        I2C_HALF();
        scl_high(); I2C_HALF();
        scl_low();
    }
    /* 9th clock: release SDA and sample the ACK the slave pulls low */
    sda_high();
    I2C_HALF();
    scl_high(); I2C_HALF();
    uint8_t ack = (sda_read() == 0);
    scl_low();
    return ack;
}

/* Write one byte to the PCF8574 output port. */
static void pcf_write(uint8_t data)
{
    i2c_start();
    i2c_write_byte((LCD_ADDR << 1) | 0);   /* write */
    i2c_write_byte(data);
    i2c_stop();
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

void lcd_init(void)
{
    dwt_init();
    i2c_gpio_init();

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
}i
