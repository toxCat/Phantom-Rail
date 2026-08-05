#ifndef I2C1_MASTER_H
#define I2C1_MASTER_H
/*
 * Register-level (CMSIS, no HAL) I2C1 master on PB6/PB7 for the inter-board
 * link to the LM51772 sim. Standard-mode 100 kHz, timing derived from PCLK1.
 * Requires the DWT cycle counter to be running (main.c enables it) for its
 * transfer timeouts.
 *
 * NOTE: the board-to-board link needs external pull-ups on SDA and SCL
 * (~4.7 k to 3V3) and a common ground; the STM32 pins are open-drain only.
 */
#include <stdint.h>

void i2c1_master_init(void);

/* Write len bytes to the 7-bit address addr7 (single START..STOP frame).
 * Returns 1 if every byte was ACKed, 0 on NACK / timeout / stuck bus.
 * Never blocks indefinitely. */
int i2c1_master_write(uint8_t addr7, const uint8_t *data, uint32_t len);

/* Reset the I2C1 peripheral to clear a wedged/stuck-BUSY state. Call after a
 * failed write so the master self-heals (e.g. once the slave finishes booting)
 * instead of needing a manual reset. */
void i2c1_master_recover(void);

#endif /* I2C1_MASTER_H */
