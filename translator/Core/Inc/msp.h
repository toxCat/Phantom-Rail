#ifndef MSP_H
#define MSP_H
/*
 * Minimal MSP v1 master over USART1, enough to poll a Betaflight FC for the RC
 * channel values that carry the transmitter's GVAR/pot/switch settings.
 * Request:  '$' 'M' '<' size cmd crc          (crc = XOR of size,cmd,payload)
 * Response: '$' 'M' '>' size cmd payload crc
 */
#include <stdint.h>

#define MSP_RC   105u   /* returns RC channels as u16 little-endian (us)      */

/* Poll MSP_RC and fill ch[] (up to max_ch) with the channel values in us.
 * Returns the channel count on success, or -1 on timeout / bad frame. */
int msp_read_rc(uint16_t *ch, uint8_t max_ch);

#endif /* MSP_H */
