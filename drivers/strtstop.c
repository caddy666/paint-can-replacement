/**
 * @file  strtstop.c
 * @brief Spindle motor start-up / stop sequencer and disc-type detection.
 *
 * Implements the start_stop() command function and its background executor
 * execute_start_stop_functions(), which are called from player.c.
 *
 * ── State encoding ──────────────────────────────────────────────────────
 *
 * start_stop_process:
 *   High nibble — current command (SS_IDLE / SS_START_UP / … / SS_MOTOR_OFF)
 *   Low  nibble — sub-phase index within the current command
 *
 * start_stop_status:
 *   High nibble — BUSY / READY / CD_ERROR_STATE
 *   Low  nibble — sub-phase (mirrors start_stop_process low nibble)
 *
 * ── Disc-type detection ─────────────────────────────────────────────────
 *
 * get_disk_type() is called during SS_START_UP after the servo has locked.
 * It determines whether the disc is a standard pressed CD or a CDR/CDRW by
 * reading the lead-in Q-channel data and inspecting Q_buffer[1] (TNO field)
 * while in the lead-in area:
 *
 *   TNO > 90 in the lead-in → CDR (CDR discs encode the recording date there)
 *   TNO ≤ 90               → pressed CD
 *
 * The algorithm:
 *   Phase 0: arm subcode reader, wait for a frame.
 *   Phase 1: if FIRST_LEADIN_AREA found → jump to phase 3.
 *            if ABS_TIME found → we are in the program area → jump back (phase 2).
 *            on timeout → SUBCODE_TIMEOUT_ERROR.
 *   Phase 2: seek to absolute time 00:02:00 (just inside lead-in) and restart.
 *   Phase 3: wait for 5 lead-in frames; on finding FIRST_LEADIN_AREA
 *            extract TNO to determine disc type.
 *            if in PROGRAM_AREA → jump into lead-in and retry.
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 */

#include <stdint.h>
#include <string.h>

#include "defs.h"
#include "serv_def.h"
#include "dsic2.h"
#include "driver.h"
#include "timer.h"

/* =========================================================================
 * External references
 * ====================================================================== */

extern volatile uint8_t timers[];
#define play_timer  timers[TIMER_PLAY]   /**< Shared with play.c for subcode timeouts */

extern uint8_t player_error;

/* Servo module API */
extern void    servo_start(void);
extern void    servo_stop(void);
extern void    servo_n1(void);
extern void    servo_n2(void);
extern void    servo_jump(int jump_size);
extern uint8_t get_servo_process_state(void);

/* Subcode module API */
extern void    start_subcode_reading(void);
extern void    stop_subcode_reading(void);
extern uint8_t is_subcode(uint8_t mode);
extern uint8_t is_cd_disc(void);

/* jump_time() performs a seek to an absolute disc time; defined in play.c.
 * We call it during disc-type detection to return to the lead-in. */
extern uint8_t jump_time(cd_time_t *t);

/* Shared with servo.c so it can skip radial init on subsequent spin-ups */
extern uint8_t disc_size_known;

/* =========================================================================
 * Shared store — union shared with play.c and shock.c
 *
 * The three overlay members are never active at the same time:
 *   toc_info     — used during TOC reading / disc-type detection
 *   scan_info    — used during track scanning (not yet implemented)
 *   sledge_cal_info — used during sledge calibration (not yet implemented)
 *
 * This union maps onto the same memory as play.c's 'store' union, matching
 * the original firmware's shared-memory layout.
 * ====================================================================== */
typedef union {
    struct {
        uint8_t   status1;
        uint8_t   status2;
        uint8_t   counter;              /**< Retry counter for lead-in search    */
        uint8_t   last_read_tno;
        uint8_t   first_mode5_pointer;
        cd_time_t tmp_time;             /**< Temporary seek target               */
        cd_time_t start_next_leadin_area;
    } toc_info;
    struct {
        int       tracks;
        cd_time_t time1;
        cd_time_t time2;
        cd_time_t time3;   /**< time3.min used as retry counter */
    } scan_info;
    struct {
        uint8_t        samples;
        uint16_t       offtrack_1;
        uint16_t       offtrack_2;
        cd_time_t      tmp_time;
        uint8_t        counter;
        uint16_t       ref_value;
    } sledge_cal_info;
} start_stop_store_t;

/* Private to this module.  The original 8051 firmware overlaid this union
 * with play.c's 'store' via shared RAM.  On the Pico2 each module has its
 * own copy — no physical RAM sharing is needed. */
static start_stop_store_t sst;

/* =========================================================================
 * Module state
 * ====================================================================== */

/** Combined command + phase register (high nibble = cmd, low nibble = phase). */
static uint8_t start_stop_process  = 0;

/** Combined status register (high nibble = BUSY/READY/ERROR, low = phase). */
static uint8_t start_stop_status   = 0;

/**
 * Set when a start_stop() command is active; cleared when the command
 * has completed (high nibble of start_stop_status ≠ BUSY).
 * Checked by player.c to distinguish a re-entry for the same command from
 * a fresh command with a different parameter.
 */
uint8_t start_stop_command_busy = 0;

/** Set once disc type has been determined for the current disc. */
static uint8_t disc_type_known     = 0;

/** 1 = pressed CD, 0 = CDR (set by get_disk_type()). */
static uint8_t cd_disc_local       = 1;

/* =========================================================================
 * init_for_new_disc — reset per-disc state
 *
 * Called by player_init() (first boot) and implicitly whenever a new disc
 * is loaded.  Clears the disc-size-known and disc-type-known flags so that
 * the next spin-up will re-run the detection routines.
 * ====================================================================== */

void init_for_new_disc(void)
{
    disc_size_known   = 0;   /* servo.c will re-run radial init */
    disc_type_known   = 0;
    cd_disc_local     = 1;   /* assume pressed CD until proven otherwise */
}

/* =========================================================================
 * get_disk_type — determine whether the disc is a pressed CD or CDR
 *
 * Returns READY when determination is complete, BUSY while still trying,
 * CD_ERROR_STATE on timeout.
 *
 * Phase summary (low nibble of start_stop_process):
 *   0 → arm subcode, set timeout, advance to 1
 *   1 → wait for FIRST_LEADIN_AREA (→ phase 3) or ABS_TIME (→ phase 2)
 *   2 → seek to 00:02:00 using jump_time(), then restart at phase 3
 *   3 → set retry counter, advance to 4
 *   4 → wait for FIRST_LEADIN_AREA to extract TNO; if in PROGRAM_AREA
 *       jump back into lead-in and restart at phase 5
 *   5 → wait for servo jump to complete, then retry from phase 4
 * ====================================================================== */

static uint8_t get_disk_type(void)
{
    if (disc_type_known) return READY;   /* already determined for this disc */

    switch (start_stop_process & 0x0Fu) {

    case 0:
        /* Start reading and wait for a frame */
        start_subcode_reading();
        play_timer = SUBCODE_TIMEOUT_VALUE;
        start_stop_process++;
        /* fall through immediately to case 1 */

    case 1:
        /* Classify the current disc area from the first frame received */
        if (is_subcode(FIRST_LEADIN_AREA)) {
            /* Already in the lead-in — skip ahead to disc-type sampling */
            start_stop_process = (uint8_t)((start_stop_process & 0xF0u) | 0x03u);
        } else if (is_subcode(ABS_TIME)) {
            /* We are in the program area — need to seek into the lead-in */
            start_stop_process++;   /* → phase 2 */
        } else if (play_timer == 0) {
            player_error = SUBCODE_TIMEOUT_ERROR;
            return CD_ERROR_STATE;
        }
        break;

    case 2: {
        /* Seek to absolute time 00:02:00 (two seconds from disc start) which
         * is deep enough into the lead-in to read TOC data reliably. */
        sst.toc_info.tmp_time.min = 0;
        sst.toc_info.tmp_time.sec = 2;
        sst.toc_info.tmp_time.frm = 0;
        uint8_t es = jump_time(&sst.toc_info.tmp_time);
        if (es == READY) {
            start_stop_process++;   /* → phase 3 */
            start_subcode_reading();
            play_timer = SUBCODE_TIMEOUT_VALUE;
        } else {
            return es;   /* still seeking or error */
        }
        break;
    }

    case 3:
        /* Initialise retry counter and proceed to sampling phase */
        sst.toc_info.counter = 5;
        play_timer = SUBCODE_TIMEOUT_VALUE;
        start_stop_process++;
        /* fall through to case 4 */

    case 4:
        if (is_subcode(FIRST_LEADIN_AREA)) {
            /* Read the TNO field: pressed CDs have BCD track numbers (01–99
             * after hex conversion, so ≤ 99); CDRs use tno > 90 as a marker. */
            cd_disc_local   = (Q_buffer[1] > 90u) ? 0u : 1u;
            disc_type_known = 1;
            return READY;
        } else if (is_subcode(PROGRAM_AREA)) {
            /* We drifted into the program area — jump back into lead-in */
            servo_jump(TRACKS_INTO_LEADIN);
            start_stop_process++;   /* → phase 5 */
        } else if (play_timer == 0) {
            player_error = SUBCODE_TIMEOUT_ERROR;
            return CD_ERROR_STATE;
        }
        break;

    case 5: {
        /* Wait for the jump back into lead-in to complete */
        uint8_t es = get_servo_process_state();
        if (es == READY) {
            sst.toc_info.counter--;
            if (sst.toc_info.counter == 0) {
                player_error = TOC_READ_ERROR;
                return CD_ERROR_STATE;
            }
            start_subcode_reading();
            play_timer = SUBCODE_TIMEOUT_VALUE;
            start_stop_process--;   /* back to phase 4 to try reading again */
        } else {
            return es;
        }
        break;
    }

    default:
        break;
    }

    return BUSY;
}

/* =========================================================================
 * start_stop — command entry point
 *
 * Called from the player dispatch table with one of the SS_* constants.
 *
 * The command is latched on the first call (when !start_stop_command_busy
 * or a different command arrives).  Subsequent calls with the same command
 * simply return the current high-nibble status.
 *
 * execute_start_stop_functions() drives the actual state machine;
 * start_stop() only latches the command and reports the result.
 * ====================================================================== */

uint8_t start_stop(uint8_t cmd)
{
    if ((start_stop_process >> 4) != cmd || !start_stop_command_busy) {
        /* New command — initialise both command and status registers */
        start_stop_process      = (uint8_t)(cmd << 4);
        start_stop_status       = (uint8_t)(BUSY << 4);
        start_stop_command_busy = 1;
    }
    if ((start_stop_status >> 4) != BUSY)
        start_stop_command_busy = 0;   /* command has finished */

    return (uint8_t)(start_stop_status >> 4);
}

/* =========================================================================
 * execute_start_stop_functions — background executor
 *
 * Called unconditionally once per main-loop iteration.  Drives the active
 * command through its phases by updating start_stop_status.
 *
 * SS_IDLE — nothing to do; set status to READY on first call.
 *
 * SS_START_UP — three phases:
 *   0: call servo_start() and wait for servo to report READY.
 *   1: servo ready — arm subcode reader and start disc-type detection.
 *   2: disc-type known — command complete.
 *
 * SS_STOP / SS_SPEED_N1 / SS_SPEED_N2 / SS_MOTOR_OFF — two phases:
 *   0: issue the appropriate servo command.
 *   1: wait for servo to report READY.
 *   SS_MOTOR_OFF skips phase 1 (immediate completion, no feedback needed).
 * ====================================================================== */

void execute_start_stop_functions(void)
{
    uint8_t cmd = start_stop_process >> 4;

    if (cmd == SS_IDLE) {
        /* SS_IDLE completes on the first call; the low-nibble flag
         * prevents re-assertion on subsequent calls while still active. */
        if (!(start_stop_status & 0x0Fu))
            start_stop_status = (uint8_t)((READY << 4) | 0x01u);
        return;
    }

    if (cmd == SS_START_UP) {
        switch (start_stop_status & 0x0Fu) {

        case 0:
            /* Phase 0: kick off the servo spin-up sequence */
            servo_start();
            start_stop_status++;   /* → phase 1 */
            /* fall through to check immediately */

        case 1: {
            /* Phase 1: wait for the servo to achieve focus + motor speed */
            uint8_t es = get_servo_process_state();
            if (es == READY) {
                start_subcode_reading();
                start_stop_process &= 0xF0u;   /* reset sub-phase counter for get_disk_type */
                start_stop_status++;            /* → phase 2 */
            } else {
                if (es == CD_ERROR_STATE)
                    start_stop_status = (uint8_t)((CD_ERROR_STATE << 4) | 0x04u);
                break;
            }
        }
        /* fall through to phase 2 */

        case 2: {
            /* Phase 2: determine disc type (CDR vs pressed CD) */
            uint8_t es = get_disk_type();
            if (es == READY)
                start_stop_status = (uint8_t)((READY << 4) | 0x03u);
            else {
                if (es == CD_ERROR_STATE)
                    start_stop_status = (uint8_t)((CD_ERROR_STATE << 4) | 0x03u);
                break;
            }
        }
        /* fall through — command complete */

        case 3:
        default:
            break;
        }
        return;
    }

    /* SS_STOP / SS_SPEED_N1 / SS_SPEED_N2 / SS_MOTOR_OFF */
    switch (start_stop_status & 0x0Fu) {

    case 0:
        /* Issue the servo command appropriate for this sub-command */
        if      (cmd == SS_SPEED_N1) servo_n1();
        else if (cmd == SS_SPEED_N2) servo_n2();
        else                         servo_stop();

        if (cmd == SS_MOTOR_OFF)
            /* SS_MOTOR_OFF completes immediately (no servo feedback needed) */
            start_stop_status = (uint8_t)((READY << 4) | 0x02u);
        else
            start_stop_status++;   /* → phase 1 */
        break;

    case 1: {
        /* Phase 1: wait for servo to confirm the speed change or stop */
        uint8_t es = get_servo_process_state();
        if (es == READY)
            start_stop_status = (uint8_t)((READY << 4) | 0x02u);
        else {
            if (es == CD_ERROR_STATE)
                start_stop_status = (uint8_t)((CD_ERROR_STATE << 4) | 0x02u);
            break;
        }
    }
    /* fall through — command complete */

    case 2:
    default:
        break;
    }
}
