#ifndef LCD1602_H
#define LCD1602_H

#include <stdint.h>

/*
 * Register-level (CMSIS, no HAL) hardware-I2C2 driver for a 16x2 HD44780
 * LCD behind a PCF8574 "I2C backpack", for the STM32F411 "black pill".
 *
 * Pins (hardware I2C2, open-drain, relying on the backpack's onboard pull-ups):
 *     PB10 = SCL  (AF4)
 *     PB3  = SDA  (AF9)
 *
 * The LCD runs on the F411's hardware I2C2 peripheral so that I2C1 (PB6/PB7)
 * stays free for the inter-black-pill slave link. On the F411 both PB6/PB7 and
 * PB8/PB9 are I2C1 mappings, so the LCD cannot share I2C1 with the link --
 * hence hardware I2C2 on PB10/PB3.
 */

/* 7-bit address of the PCF8574 backpack.
 * 0x27 = PCF8574,  0x3F = PCF8574A. Override at compile time if needed. */
#ifndef LCD_ADDR
#define LCD_ADDR 0x27
#endif

/* Initialise the panel. Brings up I2C2, then auto-detects the PCF8574
 * address (compile-time LCD_ADDR, else 0x27, else 0x3F). Returns the 7-bit
 * address that ACKed, or 0 if the bus is silent (no backpack / bad wiring /
 * no pull-ups) -- in which case the panel is left untouched. */
uint8_t lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);   /* row 0..1, col 0..15 */
void lcd_print(const char *s);
void lcd_putc(char c);

/* Simple blocking delay, exposed for the demo (DWT-based, us-accurate). */
void lcd_delay_ms(uint32_t ms);

#endif /* LCD1602_H */
