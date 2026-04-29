/**
 * @file  subcode.c
 * @brief CD6 (CXD2500BQ) initialisation and Q-channel subcode decoder.
 *
 * This module sits between the raw bit-reader in driver.c (cd6_read_subcode)
 * and the rest of the firmware.  Its responsibilities are:
 *
 *   1. Arming / disarming the subcode reader via start_subcode_reading() /
 *      stop_subcode_reading().
 *
 *   2. On each call to subcode_module() (once per main-loop iteration),
 *      calling cd6_read_subcode() to check for a new SCOR edge and, if one
 *      has fired, decoding the 10-byte raw frame in Q_buffer in-place.
 *
 *   3. BCD → hex conversion: the CD standard encodes all time values in BCD
 *      (Binary-Coded Decimal).  The firmware works entirely in hex internally,
 *      so all BCD fields are converted on receipt.
 *
 *   4. Providing is_subcode() to test whether the current Q_buffer frame
 *      belongs to a particular disc area or address type.
 *
 * ── Q-channel frame layout ──────────────────────────────────────────────
 *
 *  Byte  Field   Description
 *   0    CONAD   Control nibble [7:4] + Address nibble [3:0]
 *   1    TNO     Track number (BCD): 0=lead-in, 0xAA=lead-out, 01–99=program
 *   2    INDEX   Index within track (BCD); special values in TOC (0xA0/A1/A2)
 *   3    RMIN    Relative minutes (BCD)
 *   4    RSEC    Relative seconds (BCD)
 *   5    RFRM    Relative frames  (BCD)
 *   6    ZERO    Reserved (always 0)
 *   7    AMIN    Absolute minutes (BCD)
 *   8    ASEC    Absolute seconds (BCD)
 *   9    AFRM    Absolute frames  (BCD)
 *
 * Address mode (CONAD & 0x0F) determines what the time fields mean:
 *   0x01 — Mode 1: standard disc time (used for > 99% of frames)
 *   0x02 — Mode 2: UPC/EAN catalogue number
 *   0x03 — Mode 3: ISRC code (International Standard Recording Code)
 *   0x05 — Mode 5: multi-session TOC pointer (CD-ROM XA / Photo-CD)
 *
 * ── Special TOC entries (Mode 1, in lead-in, INDEX = 0xA0/A1/A2) ───────
 *
 *  0xA0: First track number (AMIN field only, BCD)
 *  0xA1: Last  track number (AMIN field only, BCD)
 *  0xA2: Disc lead-out start time (full AMIN:ASEC:AFRM, BCD)
 *
 * After BCD conversion these are available in Q_buffer at positions 1–9.
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 */

#include <stdint.h>
#include <string.h>

#include "defs.h"
#include "serv_def.h"
#include "dsic2.h"
#include "driver.h"

/* Q_buffer index names (match subcode_frame_t in defs.h) */
#define QB_CONAD   0   /**< Control + Address byte                 */
#define QB_TNO     1   /**< Track number                           */
#define QB_INDEX   2   /**< Index within track                     */
#define QB_RMIN    3   /**< Relative minutes                       */
#define QB_RSEC    4   /**< Relative seconds                       */
#define QB_RFRM    5   /**< Relative frames                        */
#define QB_ZERO    6   /**< Reserved / always zero                 */
#define QB_AMIN    7   /**< Absolute minutes                       */
#define QB_ASEC    8   /**< Absolute seconds                       */
#define QB_AFRM    9   /**< Absolute frames                        */

/* =========================================================================
 * Module state
 * ====================================================================== */

/** 1 while the subcode reader is armed; 0 when stopped. */
static uint8_t subcode_reading     = 0;

/**
 * Set to 1 when start_subcode_reading() is called; cleared to 0 when the
 * first frame after the arm request has been decoded.  This ensures that
 * is_subcode() returns FALSE for frames that were captured before the caller
 * asked to start reading — stale data from an earlier position.
 */
static uint8_t new_subcode_request = 0;

/**
 * Set to 1 when subcode_module() successfully decodes a frame.
 * Cleared by servo.c (servo_monitor_state) once it has consumed the flag.
 * This acts as a "fresh data" flag for the servo loop's subcode-monitor timeout.
 */
uint8_t i_can_read_subcode = 0;

/**
 * 1 if the disc is a standard pressed CD; 0 if it is a CDR/CDRW.
 * Determined in strtstop.c from the Q_buffer[QB_TNO] value in the lead-in:
 * TNO > 90 in the lead-in indicates a CDR.
 */
static uint8_t cd_disc = 1;

/* =========================================================================
 * BCD conversion helpers
 * ====================================================================== */

/**
 * @brief  Convert a single BCD byte to hex.
 * @param  bcd  Binary-coded decimal value (e.g. 0x59 for 59 seconds).
 * @return Equivalent hex value (e.g. 59).
 *
 * Example: 0x59 → (5 × 10) + 9 = 59.
 */
static uint8_t bcd_to_hex(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10u) + (bcd & 0x0Fu));
}

/**
 * @brief  Convert three consecutive BCD bytes (min, sec, frm) to hex in place.
 * @param  buf  Pointer to the first of three consecutive bytes.
 */
static void bcd_to_hex_time_buf(uint8_t *buf)
{
    buf[0] = bcd_to_hex(buf[0]);
    buf[1] = bcd_to_hex(buf[1]);
    buf[2] = bcd_to_hex(buf[2]);
}

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the CXD2500BQ to default N=1 settings.
 *
 * Sends the startup register sequence that configures the CXD2500BQ for
 * single-speed playback with audio output enabled.  This must be called
 * once during player_init() after driver_init().
 *
 * Sequence:
 *   SPEED_CONTROL_N1 — configure PLL for 1× CLV speed
 *   DAC_OUTPUT_MODE  — select audio DAC output (vs CD-ROM mode)
 *   MOT_OUTPUT_MODE  — configure motor drive output
 *   MOT_GAIN_12CM_N1 — set motor gain for a 12 cm disc at 1×
 */
void cd6_init(void)
{
    cd6_wr(SPEED_CONTROL_N1);
    cd6_wr(DAC_OUTPUT_MODE);
    cd6_wr(MOT_OUTPUT_MODE);
    /* EBU_OUTPUT_MODE is a no-op on this hardware revision */
    cd6_wr(MOT_GAIN_12CM_N1);
}

/**
 * @brief  Arm the subcode reader for the next Q-channel frame.
 *
 * Sets new_subcode_request = 1 so that is_subcode() refuses to match any
 * frame left in Q_buffer from before this call.  Also clears scor_edge to
 * ensure we do not act on a stale SCOR interrupt that fired before we
 * started reading.
 */
void start_subcode_reading(void)
{
    subcode_reading     = 1;
    new_subcode_request = 1;
    extern volatile uint8_t scor_edge;
    scor_edge = 0;   /* discard any SCOR edge that arrived before this arm */
}

/**
 * @brief  Disarm the subcode reader.
 *
 * Called from play.c when the drive enters IDLE_MODE and subcode decoding
 * is no longer needed (saves unnecessary processing each tick).
 */
void stop_subcode_reading(void)
{
    subcode_reading = 0;
}

/** @return The most recent peak audio level (low byte). */
uint8_t give_peak_level_low(void)  { return peak_level_low;  }

/** @return The most recent peak audio level (high byte). */
uint8_t give_peak_level_high(void) { return peak_level_high; }

/** @return 1 if the disc is a standard pressed CD; 0 for CDR. */
uint8_t is_cd_disc(void) { return cd_disc; }

/**
 * @brief  Test whether the current Q_buffer frame matches the requested area.
 *
 * Returns FALSE if new_subcode_request == 1 (no frame decoded since arm).
 * The 'mode' parameter selects which area/type to match; see defs.h for
 * the ALL_SUBCODES … LEADOUT_AREA constants.
 *
 * Notes on special cases:
 *   ABS_TIME in lead-in (tno=0): only valid for CDR discs, not pressed CDs,
 *     because a pressed CD's lead-in frames use tno=0 with rmin > 90 as a
 *     marker — which would otherwise look like a valid program-area frame.
 *
 *   FIRST_LEADIN_AREA: detects the first frame of the lead-in by testing
 *     address=1 and tno=0.  The rmin > 90 / cd_disc logic matches the
 *     same "CDR lead-in marker" convention described above.
 *
 * @param  mode  One of the area constants from defs.h.
 * @return TRUE if the current Q_buffer matches; FALSE otherwise.
 */
uint8_t is_subcode(uint8_t mode)
{
    if (new_subcode_request) return FALSE;   /* no fresh frame yet */

    uint8_t conad_lo = Q_buffer[QB_CONAD] & 0x0Fu;   /* address nibble */
    uint8_t tno      = Q_buffer[QB_TNO];
    uint8_t rmin     = Q_buffer[QB_RMIN];

    switch (mode) {

    case ALL_SUBCODES:
        return TRUE;   /* any decoded frame qualifies */

    case ABS_TIME:
        if (conad_lo == 0x01) {
            if (tno != 0) return TRUE;           /* program area — always valid  */
            /* Lead-in (tno=0): only accept if CDR (rmin ≤ 90) */
            return (uint8_t)((rmin > 90u || cd_disc) ? FALSE : TRUE);
        }
        return FALSE;

    case CATALOG_NR:
        return (uint8_t)(conad_lo == 0x02 ? TRUE : FALSE);

    case ISRC_NR:
        return (uint8_t)(conad_lo == 0x03 ? TRUE : FALSE);

    case FIRST_LEADIN_AREA:
        /* Mode 1, tno = 0 (lead-in), and passes the CDR marker test */
        if (conad_lo == 0x01 && tno == 0)
            return (uint8_t)((rmin > 90u || cd_disc) ? TRUE : FALSE);
        return FALSE;

    case LEADIN_AREA:
        /* Any mode-1 frame in the lead-in (address bits 00 or 01) */
        return (uint8_t)(((Q_buffer[QB_CONAD] & 0x03u) == 0x01 && tno == 0)
                         ? TRUE : FALSE);

    case PROGRAM_AREA:
        /* Mode-1 frame with a valid track number (01–99) */
        return (uint8_t)((conad_lo == 0x01 && tno != 0 && tno != 0xAA)
                         ? TRUE : FALSE);

    case LEADOUT_AREA:
        /* Mode-1 frame with tno = 0xAA (lead-out sentinel) */
        return (uint8_t)((conad_lo == 0x01 && tno == 0xAAu) ? TRUE : FALSE);

    default:
        return FALSE;
    }
}

/**
 * @brief  Copy the current absolute disc time from Q_buffer to *p.
 *
 * In the program area (tno ≠ 0) the absolute time is in the a_time fields
 * (bytes 7–9).  In the lead-in (tno = 0) the convention is reversed: the
 * r_time fields (bytes 3–5) carry the lead-in absolute address.
 *
 * All values in Q_buffer have already been converted to hex by subcode_module().
 *
 * @param  p  Destination cd_time_t to fill.
 */
void move_abstime(cd_time_t *p)
{
    if (Q_buffer[QB_TNO] != 0) {
        p->min = Q_buffer[QB_AMIN];
        p->sec = Q_buffer[QB_ASEC];
        p->frm = Q_buffer[QB_AFRM];
    } else {
        /* Lead-in: use r_time as the absolute position */
        p->min = Q_buffer[QB_RMIN];
        p->sec = Q_buffer[QB_RSEC];
        p->frm = Q_buffer[QB_RFRM];
    }
}

/**
 * @brief  Decode a new Q-channel frame if SCOR has fired.
 *
 * Must be called once per main-loop iteration.  When a new SCOR edge is
 * detected and a raw frame has been captured by cd6_read_subcode(), this
 * function:
 *
 *   1. Clears new_subcode_request so is_subcode() can match the new data.
 *   2. Sets i_can_read_subcode for the servo monitor timeout.
 *   3. Converts all BCD time fields to hex in-place in Q_buffer.
 *   4. Updates hex_abs_min (used by the brake-table in driver.c).
 *
 * Only Mode-1 (address=1) frames and Mode-5 (multi-session, address=5)
 * frames are decoded; other modes are passed through unchanged.
 *
 * Special TOC entries (INDEX = 0xA0, 0xA1, 0xA2):
 *   - 0xA0 (first track): only AMIN is a valid BCD track number; the
 *     ASEC and AFRM fields carry the disc type byte and are not converted.
 *   - 0xA1 (last track):  same — only AMIN is converted.
 *   - 0xA2 (disc end):    normal full-time conversion for AMIN:ASEC:AFRM.
 */
void subcode_module(void)
{
    if (!subcode_reading) return;

    if (cd6_read_subcode()) {
        /* A fresh frame was clocked in — process it */
        new_subcode_request = 0;
        i_can_read_subcode  = 1;

        uint8_t conad_lo = Q_buffer[QB_CONAD] & 0x0Fu;

        if (conad_lo == 0x01) {
            /* ── Mode 1 (standard time-code frame) ── */

            /* Convert TNO, skipping the lead-out sentinel 0xAA which is
             * a fixed pattern, not a BCD value. */
            if (Q_buffer[QB_TNO] != 0xAAu)
                Q_buffer[QB_TNO] = bcd_to_hex(Q_buffer[QB_TNO]);

            /* Relative time is always a BCD time triple */
            bcd_to_hex_time_buf(&Q_buffer[QB_RMIN]);

            if (Q_buffer[QB_INDEX] == 0xA0u || Q_buffer[QB_INDEX] == 0xA1u) {
                /* TOC pointer entries: only AMIN is a BCD track number */
                Q_buffer[QB_AMIN] = bcd_to_hex(Q_buffer[QB_AMIN]);
            } else {
                /* Normal program / lead-in / lead-out frame */
                bcd_to_hex_time_buf(&Q_buffer[QB_AMIN]);
                /* INDEX is a BCD number unless it is the special lead-out 0xA2 */
                if (Q_buffer[QB_INDEX] != 0xA2u)
                    Q_buffer[QB_INDEX] = bcd_to_hex(Q_buffer[QB_INDEX]);
            }

            /* Update the brake-table position index.
             * hex_abs_min = 0 in lead-in (tno=0) so inner brake times apply. */
            if (Q_buffer[QB_TNO] != 0x00u)
                hex_abs_min = Q_buffer[QB_AMIN];
            else
                hex_abs_min = 0;

        } else if (conad_lo == 0x05) {
            /* ── Mode 5 (multi-session TOC pointer, Photo-CD / CD-ROM XA) ──
             *
             * These frames identify the start address of the next session.
             * We only convert the time triple if it contains a valid BCD
             * address (rmin ≠ 0xFF, which is a "no next session" sentinel). */
            if (Q_buffer[QB_TNO]   == 0x00u &&
                Q_buffer[QB_INDEX] == 0xB0u &&
                Q_buffer[QB_RMIN]  != 0xFFu)
            {
                bcd_to_hex_time_buf(&Q_buffer[QB_RMIN]);
            }
        }
        /* Modes 2 and 3 (catalogue number, ISRC) are stored verbatim. */
    }
}
