#ifndef PHANTOM_LINK_H
#define PHANTOM_LINK_H
/*
 * Phantom-Rail inter-board I2C1 link.
 *
 *   translator (Black Pill #1)  ==  I2C1 MASTER  (PB6/PB7)
 *   lm51772-sim (Black Pill #2)  ==  I2C1 SLAVE   (address PR_I2C_ADDR)
 *
 * The master owns the ADC/source sense; it detects the pack and pushes a small
 * telemetry frame to the sim, which renders it on the LCD. This header is the
 * single source of truth for the wire format and is #included verbatim by both
 * projects (-I../Protocol), so the two sides cannot drift out of sync.
 *
 * Frame (master -> slave write), PR_FRAME_LEN bytes:
 *   [0] CMD    = PR_CMD_TELEMETRY
 *   [1] VIN_L  source millivolts, low byte   (u16 little-endian)
 *   [2] VIN_H  source millivolts, high byte
 *   [3] CELLS  detected LiPo cell count (S), 0 = unknown
 *   [4] FLAGS  bit0 = PR_FLAG_VALID (a real source is present)
 *   [5] XOR    XOR of bytes [0..4] (link integrity check)
 */
#include <stdint.h>

#define PR_I2C_ADDR       0x42u    /* 7-bit slave address of the sim board   */
#define PR_I2C_HZ         100000u  /* standard-mode SCL                      */

#define PR_CMD_TELEMETRY  0x10u    /* frame[0] for a telemetry update        */
#define PR_FRAME_LEN      6u       /* total bytes on the wire                */
#define PR_FLAG_VALID     0x01u    /* frame[4] bit0: source reading is valid */

/* XOR of the first n bytes -- the frame's integrity byte. */
static inline uint8_t pr_xor(const uint8_t *b, uint32_t n)
{
    uint8_t x = 0;
    for (uint32_t i = 0; i < n; i++) x ^= b[i];
    return x;
}

/* Fill buf (>= PR_FRAME_LEN) with a telemetry frame ready to put on the wire. */
static inline void pr_build_telemetry(uint8_t *buf, uint16_t vin_mv,
                                      uint8_t cells, uint8_t flags)
{
    buf[0] = (uint8_t)PR_CMD_TELEMETRY;
    buf[1] = (uint8_t)(vin_mv & 0xFFu);
    buf[2] = (uint8_t)(vin_mv >> 8);
    buf[3] = cells;
    buf[4] = flags;
    buf[5] = pr_xor(buf, 5);
}

/* Validate and unpack a received frame.
 * Returns 1 on a good telemetry frame (out params written), 0 otherwise. */
static inline int pr_parse_telemetry(const uint8_t *buf, uint32_t len,
                                     uint16_t *vin_mv, uint8_t *cells,
                                     uint8_t *flags)
{
    if (len != PR_FRAME_LEN)               return 0;
    if (buf[0] != (uint8_t)PR_CMD_TELEMETRY) return 0;
    if (buf[5] != pr_xor(buf, 5))          return 0;
    *vin_mv = (uint16_t)((uint16_t)buf[1] | ((uint16_t)buf[2] << 8));
    *cells  = buf[3];
    *flags  = buf[4];
    return 1;
}

#endif /* PHANTOM_LINK_H */
