#ifndef USART1_H
#define USART1_H
/*
 * Register-level (CMSIS, no HAL) USART1 on PA9 (TX) / PA10 (RX), 8N1, for the
 * MSP link to the flight controller. Blocking with DWT timeouts (the DWT cycle
 * counter must be running -- main.c enables it).
 */
#include <stdint.h>

void usart1_init(uint32_t baud);

/* Blocking transmit of len bytes. */
void usart1_write(const uint8_t *data, uint32_t len);

/* Read one byte with a timeout. Returns 1 and stores it in *out on success,
 * 0 on timeout. Overrun/framing errors are cleared and skipped. */
int usart1_read_byte(uint8_t *out, uint32_t timeout_us);

#endif /* USART1_H */
