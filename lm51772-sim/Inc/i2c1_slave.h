#ifndef I2C1_SLAVE_H
#define I2C1_SLAVE_H
/*
 * I2C1 slave presenting the LM51772 register interface (see
 * ../Protocol/lm51772_regs.h). Interrupt-driven and clock-stretch-safe so the
 * blocking LCD writes on the main loop never drop bytes. Implements the
 * register-addressed protocol: a write sets the register pointer then streams
 * data (auto-increment); a read returns data from the pointer (auto-increment).
 */
#include <stdint.h>

/* Bring up I2C1 as the register slave at addr7 and load reset values. */
void i2c1_slave_init(uint8_t addr7);

/* Main-loop access to the register file (single-byte accesses are atomic). */
uint8_t lm_reg_get(uint8_t reg);
void    lm_reg_set(uint8_t reg, uint8_t val);

/* Returns 1 (once) if CLEAR_FAULTS (0x03) was accessed since the last call. */
uint8_t lm_take_clear_faults(void);

/* Bring-up diagnostics: address matches and completed transactions. */
void i2c1_slave_diag(uint32_t *addr_hits, uint32_t *transactions);

#endif /* I2C1_SLAVE_H */
