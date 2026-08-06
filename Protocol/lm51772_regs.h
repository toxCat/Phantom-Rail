#ifndef LM51772_REGS_H
#define LM51772_REGS_H
/*
 * Phantom-Rail inter-board I2C1 link, phase 2: the sim (Black Pill #2) now
 * mimics the TI LM51772 buck-boost controller's I2C register interface, and the
 * translator (Black Pill #1) is the host controller that reads/writes it -- the
 * same way a flight controller would drive the real IC. Register set + encodings
 * are from the LM51772 datasheet (SNVSC22D). This header is #included verbatim
 * by both projects (-I../Protocol) so the two sides can't drift.
 *
 * Wire protocol (standard I2C, register-addressed, auto-increment):
 *   write:  S  ADDR+W  REG  D0 [D1 ...]  P          (REG auto-increments)
 *   read:   S  ADDR+W  REG  Sr  ADDR+R  D0 [D1 ...] nA  P
 *
 * The board-to-board link needs external pull-ups on SDA/SCL and a common
 * ground (unchanged from phase 1).
 */
#include <stdint.h>

/* 7-bit target address: 0x6A (ADDR/SLOPE -> GND) or 0x6B (-> VCC2). */
#define LM_ADDR              0x6Au

/* ---- Register map (datasheet Table 8-1) ---- */
#define LM_REG_CLEAR_FAULTS  0x03u   /* accessing clears latched faults        */
#define LM_REG_ILIM          0x0Au   /* ILIM_THRESHOLD (current limit code)    */
#define LM_REG_VOUT_LSB      0x0Cu   /* VOUT_TARGET1 LSB [7:0]                  */
#define LM_REG_VOUT_MSB      0x0Du   /* VOUT_TARGET1 MSB [3:0] (upper nibble 0)*/
#define LM_REG_PD_STATUS0    0x21u   /* USB_PD_STATUS_0 (bit6 CC_OPERATION)    */
#define LM_REG_STATUS_BYTE   0x78u   /* fault/status low byte                  */
#define LM_REG_PD_CONTROL0   0x81u   /* CONV_EN2 (bit0), FORCE_DISCH (bit1)    */
#define LM_REG_MFR_D0        0xD0u
#define LM_REG_MFR_D8        0xD8u   /* holds SEL_FB_DIV20 (bit7)              */
#define LM_REG_IVP_VOLTAGE   0xDAu

/* USB_PD_CONTROL_0 (0x81) bits */
#define LM_CONV_EN2          0x01u   /* 1 = power stage enabled (reset = 1)    */
#define LM_FORCE_DISCH       0x02u   /* 1 = force Vout discharge               */

/* STATUS_BYTE (0x78) bits */
#define LM_ST_BUSY           0x80u
#define LM_ST_OFF            0x40u   /* device not providing VOUT / unit off   */
#define LM_ST_VOUT_OV        0x20u
#define LM_ST_IOUT_OC        0x10u   /* output over-current fault (latched)    */
#define LM_ST_VIN_UV         0x08u   /* input under-voltage fault              */
#define LM_ST_TEMP           0x04u
#define LM_ST_CML            0x02u
#define LM_ST_OTHER          0x01u

/* USB_PD_STATUS_0 (0x21) bits */
#define LM_CC_OPERATION      0x40u   /* instantaneous: in constant-current/ILIM */

/* MFR_SPECIFIC_D8 (0xD8) bits */
#define LM_SEL_FB_DIV20      0x80u   /* 1 (reset): 20mV step 3.3-48V; 0: 10mV 1-24V */

/* ---- Phantom-Rail EXTENSION registers (NOT part of the real LM51772) ----
 * The real IC exposes no VIN telemetry, but the bench wants the battery/sag
 * display kept, so the host stashes it in reserved high addresses. Clearly not
 * datasheet registers -- ignore for IC-fidelity purposes. */
#define PR_EXT_VIN_LSB       0xE0u   /* battery millivolts, u16 little-endian  */
#define PR_EXT_VIN_MSB       0xE1u
#define PR_EXT_CELLS         0xE2u   /* detected LiPo cell count               */
#define PR_EXT_BATT          0xE3u   /* battery flags (below)                  */
#define PR_BATT_VALID        0x01u
#define PR_BATT_LOW          0x02u   /* 3.2-3.6 V/cell sag                     */
#define PR_BATT_CRIT         0x04u   /* < 3.2 V/cell                          */

/* ---- Encodings ---- */

/* VOUT_TARGET 12-bit code from the two registers. */
static inline uint16_t lm_vout_code(uint8_t lsb, uint8_t msb)
{
    return (uint16_t)(((uint16_t)(msb & 0x0Fu) << 8) | lsb);
}

/* Nominal output millivolts from a code, given SEL_FB_DIV20 (Eq. 2 / Eq. 3). */
static inline uint16_t lm_vout_to_mv(uint16_t code, uint8_t div20)
{
    uint32_t mv = div20 ? (uint32_t)code * 20u : (uint32_t)code * 10u;
    uint32_t lo = div20 ? 3300u : 1000u;
    uint32_t hi = div20 ? 48000u : 24000u;
    if (mv < lo) mv = lo;
    if (mv > hi) mv = hi;
    return (uint16_t)mv;
}

/* Inverse: 12-bit code for a desired output millivolts. */
static inline uint16_t lm_vout_from_mv(uint16_t mv, uint8_t div20)
{
    uint32_t step = div20 ? 20u : 10u;
    uint32_t code = ((uint32_t)mv + step / 2u) / step;   /* rounded */
    if (code > 0x0FFFu) code = 0x0FFFu;
    return (uint16_t)code;
}

/* ILIM_THRESHOLD code -> current-limit milliamps (0.05 A/step, floor 0.5 A at
 * <=0x0A, ceiling 7 A at >=0x8C). */
static inline uint16_t lm_ilim_to_ma(uint8_t code)
{
    if (code <= 0x0Au) return 500u;
    if (code >= 0x8Cu) return 7000u;
    return (uint16_t)code * 50u;
}

/* Inverse: nearest ILIM code for a desired current-limit in milliamps. */
static inline uint8_t lm_ilim_from_ma(uint16_t ma)
{
    uint32_t c = ((uint32_t)ma + 25u) / 50u;   /* rounded, 50 mA/step */
    if (c < 0x0Bu) c = 0x0Bu;
    if (c > 0x8Cu) c = 0x8Cu;
    return (uint8_t)c;
}

/* Load a 256-entry register file with the LM51772 reset values. */
static inline void lm_regs_reset(uint8_t *r)
{
    for (int i = 0; i < 256; i++) r[i] = 0;
    r[LM_REG_ILIM]        = 0x64u;   /* 5 A                    */
    r[LM_REG_VOUT_LSB]    = 0x58u;   /* with MSB -> code 0x258 */
    r[LM_REG_VOUT_MSB]    = 0x02u;   /* = 12.00 V @ 20mV step  */
    r[LM_REG_PD_STATUS0]  = 0x00u;
    r[LM_REG_STATUS_BYTE] = 0x00u;
    r[LM_REG_PD_CONTROL0] = 0x01u;   /* CONV_EN2 = 1 (enabled) */
    r[0xD0] = 0x32u; r[0xD1] = 0x09u; r[0xD2] = 0x40u; r[0xD3] = 0x20u;
    r[0xD4] = 0x03u; r[0xD5] = 0x3Fu; r[0xD6] = 0x15u; r[0xD7] = 0x28u;
    r[0xD8] = 0x84u; r[0xD9] = 0x2Cu; r[0xDA] = 0xFFu;
}

#endif /* LM51772_REGS_H */
