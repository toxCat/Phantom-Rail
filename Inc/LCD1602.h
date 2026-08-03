#ifndef LCD1602_H
#define LCD1602_H

#include <stdint.h>

/*
 * Register-level (CMSIS, no HAL) bit-bang I2C driver for a 16x2 HD44780
 * LCD behind a PCF8574 "I2C backpack", for the STM32F411 "black pill".
 *
 * Pins (bit-banged, open-drain, relying on the backpack's onboard pull-ups):
 *     PB8 = SCL
 *     PB9 = SDA
 *
 * This deliberately does NOT use a hardware I2C peripheral, so that the
 * chip's I2C1 (PB6/PB7) stays free for the inter-black-pill slave link.
 */

/* 7-bit address of the PCF8574 backpack.
 * 0x27 = PCF8574,  0x3F = PCF8574A. Override at compile time if needed. */
#ifndef LCD_ADDR
#define LCD_ADDR 0x27
#endif

void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);   /* row 0..1, col 0..15 */
void lcd_print(const char *s);
void lcd_putc(char c);

/* Simple blocking delay, exposed for the demo (DWT-based, us-accurate). */
void lcd_delay_ms(uint32_t ms);

#endif /* LCD1602_H */
