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
 *   [3] CELLS  detected LiPo cell count (S), latched at plug-in, 0 = unknown
 *   [4] FLAGS  bitfield: VALID | LOW | CRIT | FET_ON (see below)
 *   [5] XOR    XOR of bytes [0..4] (link integrity check)
 */
#include <stdint.h>

#define PR_I2C_ADDR       0x42u    /* 7-bit slave address of the sim board   */
#define PR_I2C_HZ         100000u  /* standard-mode SCL                      */

#define PR_CMD_TELEMETRY  0x10u    /* frame[0] for a telemetry update        */
#define PR_FRAME_LEN      6u       /* total bytes on the wire                */

/* frame[4] FLAGS bits (battery sag monitor state, decided on the translator) */
#define PR_FLAG_VALID     0x01u    /* a real source is present               */
#define PR_FLAG_LOW       0x02u    /* 3.2-3.6 V/cell: sag warning ("LOW")    */
#define PR_FLAG_CRIT      0x04u    /* < 3.2 V/cell: critical, Power FET cut  */
#define PR_FLAG_FET_ON    0x08u    /* Power FET currently enabled            */
#define PR_FLAG_OC        0x10u    /* over-current soft-fail latched (FET cut) */

/* Reverse direction: a master READ of the slave returns the sim's ACS709
 * current reading (Task 3). Current-telemetry frame, PR_CUR_FRAME_LEN bytes:
 *   [0] I_L    current milliamps, low byte   (u16 little-endian)
 *   [1] I_H    current milliamps, high byte
 *   [2] CFLAGS bit0 = PR_CFLAG_VALID (sensor reading valid)
 *   [3] XOR    XOR of bytes [0..2]
 */
#define PR_CUR_FRAME_LEN  4u
#define PR_CFLAG_VALID    0x01u

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

/* Fill buf (>= PR_CUR_FRAME_LEN) with a current-telemetry frame (slave side). */
static inline void pr_build_current(uint8_t *buf, uint16_t i_ma, uint8_t cflags)
{
    buf[0] = (uint8_t)(i_ma & 0xFFu);
    buf[1] = (uint8_t)(i_ma >> 8);
    buf[2] = cflags;
    buf[3] = pr_xor(buf, 3);
}

/* Validate and unpack a current-telemetry frame (master side).
 * Returns 1 on a good frame (out params written), 0 otherwise. */
static inline int pr_parse_current(const uint8_t *buf, uint32_t len,
                                   uint16_t *i_ma, uint8_t *cflags)
{
    if (len != PR_CUR_FRAME_LEN)  return 0;
    if (buf[3] != pr_xor(buf, 3)) return 0;
    *i_ma   = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    *cflags = buf[2];
    return 1;
}

#endif /* PHANTOM_LINK_H */
