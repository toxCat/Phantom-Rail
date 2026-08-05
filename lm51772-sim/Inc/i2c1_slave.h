#ifndef I2C1_SLAVE_H
#define I2C1_SLAVE_H
/*
 * Register-level (CMSIS, no HAL) I2C1 SLAVE on PB6/PB7 for the inter-board
 * link from the translator (Black Pill #1). Interrupt-driven so the LCD's
 * blocking I2C2 writes in the main loop can never make us drop bytes.
 *
 * The board-to-board link needs external pull-ups on SDA/SCL (~4.7 k to 3V3)
 * and a common ground; the STM32 pins are open-drain only. PB6/PB7 (I2C1) and
 * the LCD's PB10/PB3 (I2C2) do not overlap.
 */
#include <stdint.h>

/* Bring up I2C1 as a slave at the given 7-bit address and enable its IRQs. */
void i2c1_slave_init(uint8_t addr7);

/* Copy out the latest telemetry. Returns 1 if a *fresh* frame has arrived
 * since the previous call (and clears the fresh flag); 0 otherwise. The out
 * params are always written with the last-known values. */
int i2c1_slave_get(uint16_t *vin_mv, uint8_t *cells, uint8_t *flags);

/* Milliseconds since the last valid frame (for link-loss detection).
 * Returns a large value if no frame has ever arrived. */
uint32_t i2c1_slave_age_ms(void);

/* Bring-up diagnostics: total address matches seen on the bus, and total valid
 * frames parsed. addr_hits==0 => the master never reached us; addr_hits>0 with
 * frames==0 => addressed but the bytes/parse failed. */
void i2c1_slave_diag(uint32_t *addr_hits, uint32_t *frames);

/* Publish the latest ACS709 current (mA) + flags for the master to READ back
 * (Task 3). Cheap to call every loop; the frame is swapped in atomically. */
void i2c1_slave_set_current(uint16_t i_ma, uint8_t cflags);

#endif /* I2C1_SLAVE_H */
