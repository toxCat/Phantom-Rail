#include "stm32f4xx.h"
#include "lcd1602.h"

/*
 * LM51772 Sim -- LCD bring-up demo (second black pill).
 *
 * Purpose: prove the PCF8574 + 1602 render path before any FC / I2C-link
 * traffic exists. IN and OUT values are STUBBED here; the real IN voltage
 * will arrive over I2C from the first black pill (translator), and OUT is
 * the simulated LM51772 setpoint.
 *
 * Screen flow:
 *   1. splash "LM51772 Sim"
 *   2. clear
 *   3. row0: "IN:nS XX.XXV"     (n = detected cell count)
 *      row1: "OUT:XX.XXV"
 */

/* Format millivolts as exactly "XX.XX" (5 chars + NUL). */
static void fmt_volts(char *buf, uint32_t mv)
{
    uint32_t whole = mv / 1000U;
    uint32_t cv    = (mv % 1000U) / 10U;   /* centivolts, 2 digits */
    if (whole > 99U) whole = 99U;
    buf[0] = (char)('0' + whole / 10U);
    buf[1] = (char)('0' + whole % 10U);
    buf[2] = '.';
    buf[3] = (char)('0' + cv / 10U);
    buf[4] = (char)('0' + cv % 10U);
    buf[5] = '\0';
}

/* Naive cell count: smallest S in 1..8 with mv/S <= 4.20 V/cell.
 * Real detection belongs on the first black pill (it owns the ADC). */
static uint8_t cell_count(uint32_t mv)
{
    for (uint8_t s = 1; s <= 8; s++)
        if (mv <= (uint32_t)s * 4200U) return s;
    return 8;
}

/* Build a padded 16-char row (+NUL) and push it at the given line. */
static void draw_in(uint8_t s, uint32_t mv)
{
    char v[6], line[17];
    int n = 0;
    fmt_volts(v, mv);
    line[n++] = 'I'; line[n++] = 'N'; line[n++] = ':';
    line[n++] = (char)('0' + s); line[n++] = 'S'; line[n++] = ' ';
    for (int i = 0; i < 5; i++) line[n++] = v[i];
    line[n++] = 'V';
    while (n < 16) line[n++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, 0);
    lcd_print(line);
}

static void draw_out(uint32_t mv)
{
    char v[6], line[17];
    int n = 0;
    fmt_volts(v, mv);
    line[n++] = 'O'; line[n++] = 'U'; line[n++] = 'T'; line[n++] = ':';
    for (int i = 0; i < 5; i++) line[n++] = v[i];
    line[n++] = 'V';
    while (n < 16) line[n++] = ' ';
    line[16] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(line);
}

int main(void)
{
    lcd_init();

    /* 1-2: splash, then clear */
    lcd_set_cursor(0, 0);
    lcd_print("LM51772 Sim");
    lcd_delay_ms(1500);
    lcd_clear();

    uint32_t t = 0;
    for (;;) {
        /* ---- STUB DATA: replace with I2C value from first black pill ---- */
        uint32_t in_mv  = 7400U + (t % 1000U);   /* fake ramp 7.40 -> 8.40 V */
        uint32_t out_mv = 12000U;                /* fake setpoint 12.00 V   */
        /* ---------------------------------------------------------------- */

        uint8_t s = cell_count(in_mv);
        draw_in(s, in_mv);
        draw_out(out_mv);

        lcd_delay_ms(250);
        t += 100U;
    }
}
