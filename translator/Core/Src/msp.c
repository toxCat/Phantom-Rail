#include "msp.h"
#include "usart1.h"

/* Per-byte receive timeout. The FC answers a request within a few ms; if it is
 * absent the first read simply times out and we bail. */
#define MSP_BYTE_TIMEOUT_US 6000u

/* Read a MSP v1 response for the given command into payload[] (up to max).
 * Returns the payload length on success, -1 on timeout / framing / CRC error. */
static int msp_recv(uint8_t cmd, uint8_t *payload, uint8_t max)
{
    uint8_t b;

    /* Resync to the '$' 'M' '>' preamble, tolerating leading noise. */
    int found = 0;
    for (int i = 0; i < 64; i++) {
        if (!usart1_read_byte(&b, MSP_BYTE_TIMEOUT_US)) return -1;
        if (b == '$') { found = 1; break; }
    }
    if (!found) return -1;
    if (!usart1_read_byte(&b, MSP_BYTE_TIMEOUT_US) || b != 'M') return -1;
    if (!usart1_read_byte(&b, MSP_BYTE_TIMEOUT_US) || b != '>') return -1;

    uint8_t size, rcmd;
    if (!usart1_read_byte(&size, MSP_BYTE_TIMEOUT_US)) return -1;
    if (!usart1_read_byte(&rcmd, MSP_BYTE_TIMEOUT_US)) return -1;
    uint8_t crc = (uint8_t)(size ^ rcmd);

    uint8_t n = 0;
    for (uint8_t i = 0; i < size; i++) {
        if (!usart1_read_byte(&b, MSP_BYTE_TIMEOUT_US)) return -1;
        crc ^= b;
        if (i < max) payload[i] = b;
        n++;
    }
    uint8_t rxcrc;
    if (!usart1_read_byte(&rxcrc, MSP_BYTE_TIMEOUT_US)) return -1;
    if (rxcrc != crc || rcmd != cmd) return -1;
    return (n > max) ? max : n;
}

int msp_read_rc(uint16_t *ch, uint8_t max_ch)
{
    /* Request: no payload, crc = cmd. */
    uint8_t req[6] = { '$', 'M', '<', 0u, (uint8_t)MSP_RC, (uint8_t)MSP_RC };
    usart1_write(req, sizeof(req));

    uint8_t pl[36];                         /* up to 18 channels */
    int plen = msp_recv((uint8_t)MSP_RC, pl, sizeof(pl));
    if (plen < 2) return -1;

    uint8_t n = (uint8_t)(plen / 2);
    if (n > max_ch) n = max_ch;
    for (uint8_t i = 0; i < n; i++) {
        ch[i] = (uint16_t)((uint16_t)pl[2 * i] | ((uint16_t)pl[2 * i + 1] << 8));
    }
    return n;
}
