/**
 * @file  play.c
 * @brief Play module — seeking, pause, TOC reading, subcode retrieval,
 *        speed change, and track jumping.
 *
 * This module implements the high-level disc-playing operations that are
 * invoked by the player dispatch table in player.c.  Its two entry points
 * are:
 *
 *   play()                  — command entry point; called from the dispatch
 *                             table with a PLAY_* constant.  Latches the
 *                             command and returns the current status.
 *
 *   execute_play_functions() — background tick; called unconditionally every
 *                              main-loop iteration to advance the active
 *                              command through its step functions.
 *
 * ── State encoding ──────────────────────��───────────────────────────��───
 *
 * play_process:
 *   High nibble — active play command (PLAY_IDLE … PLAY_JUMP_TRACKS)
 *   Low  nibble — step index within the current command (index into
 *                 play_processes[cmd][step])
 *
 * play_status:
 *   High nibble — BUSY / READY / CD_ERROR_STATE
 *   Low  nibble — play mode (IDLE_MODE / PAUSE_MODE / TRACKING_MODE / TOC_MODE)
 *                 RETURN_TO_TARGET bit is also stored here for speed changes
 *
 * ── Play modes ───────────────────────────────────────���─────────────────
 *
 *   IDLE_MODE     — laser muted, subcode reader off
 *   PAUSE_MODE    — laser on disc, servo locked; position saved in store
 *   TRACKING_MODE — playing normally; shock detector active
 *   TOC_MODE      — reading Table of Contents from lead-in area
 *
 * ── jump_time() algorithm ───────────────────────────────────────────────
 *
 * jump_time() is the core seek-to-absolute-time routine used by multiple
 * callers (play itself, shock.c, strtstop.c).  It implements a multi-jump
 * binary-search approach:
 *
 *   Phase 0+0 (initial): arm subcode reader, set timeout.
 *   Phase 1+0 (subcode received):
 *     - Read current position from Q_buffer.
 *     - If at target (0 grooves away): count remaining frames via SCOR.
 *     - Otherwise: call servo_jump(groove_delta) and advance to phase 2.
 *   Phase 0+1 (wait for servo): poll get_servo_process_state().
 *     - On READY: check jump flags; if OK increment jumps_counter and
 *       loop back to phase 1 (re-read subcode to verify position).
 *     - On ERROR: propagate.
 *   Phase 1+1 (on target, counting frames): wait for zero_scor_counter().
 *
 * The two-bit phase encoding (jump_phase0, jump_phase1) acts as a 2-bit
 * state variable: 01=initial, 10=servo waiting, 11=on-target SCOR count,
 * 00=not started.  jumps_counter guards against infinite retry loops.
 *
 * ── Phase variables ───────────────��────────────────────────���────────────
 *
 * Each step function uses the module-level play_phase0 / play_phase1 flags
 * as a local 2-bit state variable.  execute_play_functions() clears both
 * flags when advancing to the next step, so each step starts fresh.
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 * All 8051-specific constructs removed.
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
#define play_timer timers[TIMER_PLAY]   /**< Subcode / seek timeout timer */

extern uint8_t  player_error;
extern interface_field_t player_interface;

/* From subcode.c */
extern uint8_t  is_subcode(uint8_t mode);
extern void     start_subcode_reading(void);
extern void     stop_subcode_reading(void);
extern void     move_abstime(cd_time_t *p);

/* From servo.c */
extern void     servo_jump(int jump_size);
extern uint8_t  get_servo_process_state(void);
extern uint8_t  get_jump_status(void);
extern uint8_t  servo_tracking(void);

/* From shock.c */
extern uint8_t  shock_detector_off(void);
extern uint8_t  shock_detector_on(void);

/**
 * play_target_time — the seek target for the current play/seek operation.
 *
 * Written by copy_parameters() (JUMP_TO_ADDRESS) and referenced by
 * shock_detector_on() via an extern in shock.c so that the shock module
 * can use the same target as its initial progress reference.
 */
cd_time_t play_target_time;

/* From utils/maths.c */
extern int      calc_tracks(const cd_time_t *a, const cd_time_t *b);
extern uint8_t  compare_time(const cd_time_t *a, const cd_time_t *b);
extern void     add_time(const cd_time_t *a, const cd_time_t *b, cd_time_t *r);
extern void     subtract_time(const cd_time_t *a, const cd_time_t *b, cd_time_t *r);

/* From driver.c / timer.c */
extern int      zero_scor_counter(void);
extern void     init_scor_counter(uint8_t count);

/* Forward declarations for static functions defined later in this file
 * that are called by step functions defined earlier. */
static void     set_subcode_buffer(void);
static uint8_t  req_subc_stored_subc(void);

/* =========================================================================
 * BCD / hex conversion helpers
 *
 * The host (CD32 Amiga chipset) communicates all disc times in BCD.
 * Internally we work in hex.  These helpers convert at the boundary.
 * ====================================================================== */

/** Convert a single hex byte (0–99) to BCD. */
static uint8_t hex_to_bcd(uint8_t hex)
{
    uint8_t tens = hex / 10u;
    return (uint8_t)((tens << 4) | (hex - tens * 10u));
}

/** Convert a cd_time_t from hex to BCD into a separate destination. */
static void hex_to_bcd_time(const cd_time_t *src, cd_time_t *dst)
{
    dst->min = hex_to_bcd(src->min);
    dst->sec = hex_to_bcd(src->sec);
    dst->frm = hex_to_bcd(src->frm);
}

/** Convert a cd_time_t from BCD to hex into a separate destination. */
static void bcd_to_hex_time_local(const cd_time_t *src, cd_time_t *dst)
{
    dst->min = (uint8_t)(((src->min >> 4) * 10u) + (src->min & 0x0Fu));
    dst->sec = (uint8_t)(((src->sec >> 4) * 10u) + (src->sec & 0x0Fu));
    dst->frm = (uint8_t)(((src->frm >> 4) * 10u) + (src->frm & 0x0Fu));
}

/* =========================================================================
 * Play mode constants (stored in low nibble of play_status)
 * ====================================================================== */

#define IDLE_MODE             0x00u  /**< Laser muted; not playing             */
#define PAUSE_MODE            0x01u  /**< Servo locked; position frozen        */
#define TRACKING_MODE         0x02u  /**< Normal CLV playback                  */
#define TOC_MODE              0x03u  /**< Reading TOC from lead-in             */

/**
 * Bit flag combined into play_status low nibble during a speed change.
 * Set by PLAY_PREPARE_SPEED_CHANGE when it captures the current position;
 * cleared by PLAY_RESTORE_SPEED_CHANGE when the seek back has completed.
 */
#define RETURN_TO_TARGET      0x08u

/* =========================================================================
 * Shared store
 *
 * A union shared with strtstop.c (which declares the same type) and
 * indirectly with shock.c.  The play_subcode overlay holds the current
 * Q-channel frame in BCD format (as expected by the COMMO host protocol).
 * The play_times overlay holds up to four cd_time_t working values for
 * seek operations.
 *
 * Only one overlay is active at a time — play_subcode during READ_SUBCODE
 * and PAUSE_ON; play_times during seeks, jumps, and speed changes.
 * ====================================================================== */

typedef union {
    struct {
        uint8_t   conad;         /**< Control + Address byte (BCD to host)   */
        uint8_t   tno;           /**< Track number (BCD)                     */
        uint8_t   index;         /**< Index (BCD)                            */
        cd_time_t r_time;        /**< Relative time (BCD)                    */
        uint8_t   zero;          /**< Reserved                               */
        cd_time_t a_time;        /**< Absolute time (BCD)                    */
        uint8_t   left;          /**< Left-channel peak level                */
        uint8_t   right;         /**< Right-channel peak level               */
    } play_subcode;
    struct {
        cd_time_t low_time;      /**< Lower time bound for binary-search seek */
        cd_time_t high_time;     /**< Upper time bound                        */
        cd_time_t target_time;   /**< Current seek target / pause address     */
        cd_time_t tmp_time;      /**< Temporary working value                 */
    } play_times;
} play_store_t;

play_store_t store;   /* exported to strtstop.c / shock.c as 'store' */

/* =========================================================================
 * Module state
 * ====================================================================== */

static uint8_t play_process = 0;   /**< Packed command (high) + step (low)    */
static uint8_t play_status  = 0;   /**< Packed status (high) + mode (low)     */

/** Shadow of the CXD2500 volume register (write-only; we track it in SW). */
static uint8_t volume_level = 0x80u;

/** Number of servo_jump() calls made in the current jump_time() invocation. */
static uint8_t jumps_counter = 0;

/**
 * Per-step phase variables — act as a 2-bit state within each step function.
 * Cleared by execute_play_functions() whenever the step advances (READY returned).
 */
static uint8_t play_phase0 = 0;
static uint8_t play_phase1 = 0;

/**
 * Phase variables used specifically within jump_time() (which is called
 * from multiple step functions and cannot use play_phase0/1 directly).
 */
static uint8_t jump_phase0 = 0;
static uint8_t jump_phase1 = 0;

/**
 * Timer reload flags used in monitor_subcodes() to implement the multi-level
 * timeout before declaring SUBCODE_NOT_FOUND.
 */
static uint8_t reload_play_timer0 = 0;
static uint8_t reload_play_timer1 = 0;

/** Set to 1 once any valid subcode frame has been received in the current scan. */
static uint8_t track_found          = 0;

/** If 1, audio remains muted after a seek completes (used for certain host modes). */
static uint8_t mute_on_after_search = 0;

/* Exposed to cmd_hndl.c and the Dispatcher for status queries */
uint8_t play_monitor      = 0;   /**< 1 = command complete; background monitor active */
uint8_t play_command_busy = 0;   /**< 1 = a play command is currently executing       */

/* =========================================================================
 * Public helpers called from cmd_hndl.c / service.c
 * ====================================================================== */

/** @return 1 if the drive is currently in PAUSE_MODE. */
uint8_t drive_is_pausing(void)
{
    return (play_status & 0x0Fu) == PAUSE_MODE ? 1u : 0u;
}

/**
 * @brief  Set the CXD2500 DAC output mode.
 *
 * Special values 0xF0 / 0xF1 set / clear the mute_on_after_search flag
 * rather than being written directly to the CXD2500.  All other values
 * are passed to cd6_wr().
 */
void set_dac_mode(uint8_t dac_mode)
{
    if      (dac_mode == 0xF0u) mute_on_after_search = 1;
    else if (dac_mode == 0xF1u) mute_on_after_search = 0;
    else                        cd6_wr(dac_mode);
}

/** Write the given mode byte directly to the CXD2500. */
void set_play_mode(uint8_t mode) { cd6_wr(mode); }

/** Attenuate (−6 dB) the audio output. */
void attenuate_on(void)          { cd6_wr(ATTENUATE); }

/* =========================================================================
 * Mute helpers
 * ====================================================================== */

/** Mute audio; return READY (used as a step function). */
uint8_t mute_on(void)
{
    cd6_wr(MUTE);
    return READY;
}

/**
 * @brief  Unmute audio; return READY (used as a step function).
 *
 * Respects mute_on_after_search: if the host requested that audio remain
 * muted after a seek (e.g. to prevent a pop), this is a no-op.
 */
uint8_t mute_off(void)
{
    if (!mute_on_after_search) cd6_wr(FULL_SCALE);
    return READY;
}

/* =========================================================================
 * jump_time — seek the laser to an absolute disc time
 *
 * This is the core seek engine.  It is called by:
 *   - search_for_address()  (JUMP_TO_ADDRESS command)
 *   - jump_to_toc()         (READ_TOC command)
 *   - restore_after_speed_change() (PLAY_RESTORE_SPEED_CHANGE)
 *   - strtstop.c:get_disk_type()
 *   - shock.c:shock_recover()
 *
 * The algorithm repeatedly reads the current position (via subcode) and
 * calls servo_jump(groove_delta) until the laser is on the target groove,
 * then counts down the remaining fractional frames using the SCOR counter.
 *
 * jump_phase encoding:
 *   jump_phase0=0, jump_phase1=0 — not started; initialise on first entry
 *   jump_phase0=1, jump_phase1=0 — waiting for subcode (→ calc groove delta)
 *   jump_phase0=0, jump_phase1=1 — waiting for servo jump to complete
 *   jump_phase0=1, jump_phase1=1 — on target; counting SCOR frames
 * ====================================================================== */

uint8_t jump_time(cd_time_t *pt)
{
    /* ── Entry / initialise ── */
    if (!jump_phase0 && !jump_phase1) {
        start_subcode_reading();
        play_timer    = SUBCODE_TIMEOUT_VALUE;
        jumps_counter = 0;
        jump_phase0   = 1;   /* → "waiting for subcode" state */
    }

    /* ── Phase 1+0: subcode received — calculate groove delta ── */
    if (jump_phase0 && !jump_phase1) {
        if (is_subcode(ABS_TIME)) {
            move_abstime(&store.play_times.tmp_time);
            int nr = calc_tracks(&store.play_times.tmp_time, pt);

            if (nr == 0) {
                /* We are on the target track — count fractional frames */
                cd_time_t delta;
                subtract_time(pt, &store.play_times.tmp_time, &delta);
                init_scor_counter(delta.frm);
                /* → "on target, counting SCOR" state */
                jump_phase1 = 1;
            } else {
                /* Jump to the target groove */
                servo_jump(nr);
                /* → "waiting for servo" state */
                jump_phase1 = 1;
                jump_phase0 = 0;
            }
        } else if (is_subcode(FIRST_LEADIN_AREA)) {
            /* We are in the lead-in; jump outward to program area first */
            servo_jump(TRACKS_OUTOF_LEADIN);
            jump_phase1 = 1;
            jump_phase0 = 0;
        } else if (play_timer == 0) {
            player_error = SUBCODE_TIMEOUT_ERROR;
            jump_phase0  = 0;
            return CD_ERROR_STATE;
        }
    }

    /* ── Phase 0+1: waiting for servo jump to complete ── */
    if (!jump_phase0 && jump_phase1) {
        uint8_t es = get_servo_process_state();
        if (es == READY) {
            if (get_jump_status() & NO_HF_ON_TARGET) {
                /* HF absent at target: landed in an unreadable area */
                player_error = JUMP_ERROR;
                jump_phase1  = 0;
                return CD_ERROR_STATE;
            }
            jumps_counter++;
            if (jumps_counter > 20u) {
                /* Safety valve: if we haven't converged after 20 jumps,
                 * declare failure rather than spinning forever. */
                player_error = JUMP_ERROR;
                jump_phase1  = 0;
                return CD_ERROR_STATE;
            }
            /* Jump complete — loop back to re-read subcode and verify position */
            jump_phase1 = 0;
            /* jump_phase0 is already 0; next iteration will re-initialise */
        } else if (es == CD_ERROR_STATE) {
            jump_phase1 = 0;
            return CD_ERROR_STATE;
        }
    }

    /* ── Phase 1+1: on target, counting fractional frames ── */
    if (jump_phase0 && jump_phase1) {
        if (zero_scor_counter()) {
            /* Frame counter has reached zero: we are exactly at the target */
            jump_phase0 = 0;
            jump_phase1 = 1;   /* leave in READY state (see caller conventions) */
            return READY;
        }
    }

    return BUSY;
}

/* =========================================================================
 * Step functions
 *
 * Each function is one step in a play_processes[][] row.  They follow the
 * contract:
 *   Return PROCESS_READY — sequence complete; entire command done.
 *   Return READY         — this step done; execute_play_functions advances
 *                          to the next step (clears play_phase0/1).
 *   Return CD_ERROR_STATE— this step failed; command aborted.
 *   Return BUSY          — still in progress; retry next tick.
 * ====================================================================== */

/* ── PLAY_IDLE ────────��─────────────────────────────────────────────── */
/**
 * Mute audio, disarm shock detector, stop subcode reader.
 * Switches the play mode to IDLE_MODE and immediately completes.
 */
static uint8_t playing_idle(void)
{
    mute_on();
    shock_detector_off();
    stop_subcode_reading();
    jumps_counter = 0;
    play_status   = (uint8_t)((play_status & 0xF0u) | IDLE_MODE);
    return PROCESS_READY;
}

/* ── PLAY_STARTUP ────────────��────────────────────────────���─────────── */
/** No-op — returns PROCESS_READY immediately (start-up handled by strtstop). */
static uint8_t play_process_ready(void) { return PROCESS_READY; }

/* ── PAUSE_ON ───────────���───────────────────────────────────────────── */
/**
 * Phase 0+1: arm subcode reader, wait for an ABS_TIME frame.
 * When received, copy the current Q_buffer to store.play_subcode (BCD format
 * for transmission to the host) and return READY.
 */
static uint8_t pause_on_init(void)
{
    if (!play_phase0) {
        start_subcode_reading();
        play_timer  = SUBCODE_TIMEOUT_VALUE;
        play_phase0 = 1;
    }
    if (is_subcode(ABS_TIME)) {
        set_subcode_buffer();
        return READY;
    }
    if (play_timer == 0) { player_error = SUBCODE_TIMEOUT_ERROR; return CD_ERROR_STATE; }
    return BUSY;
}

/**
 * Record PAUSE_MODE in the play_status low nibble and signal step complete.
 */
static uint8_t set_pause_mode(void)
{
    play_status = (uint8_t)((play_status & 0xF0u) | PAUSE_MODE);
    return PROCESS_READY;
}

/* ── PAUSE_OFF ──────────────────────────────────────────────��───────── */
/**
 * Read current absolute time from Q_buffer and save it as the target
 * for shock_detector_on() to use as the initial reference.
 */
static uint8_t restore_target_address(void)
{
    if (!play_phase0) {
        start_subcode_reading();
        play_timer  = SUBCODE_TIMEOUT_VALUE;
        play_phase0 = 1;
    }
    if (is_subcode(ABS_TIME)) {
        move_abstime(&store.play_times.target_time);
        return READY;
    }
    if (play_timer == 0) { player_error = SUBCODE_TIMEOUT_ERROR; return CD_ERROR_STATE; }
    return BUSY;
}

/**
 * Switch to TRACKING_MODE and update the shock module's reference position.
 */
static uint8_t set_tracking_mode(void)
{
    play_status = (uint8_t)((play_status & 0xF0u) | TRACKING_MODE);
    play_target_time = store.play_times.target_time;
    return PROCESS_READY;
}

/* ── JUMP_TO_ADDRESS ────────────────────────────────────────────────── */
/**
 * Extract the seek target from player_interface.param1/2/3 (mm:ss:ff in
 * hex) and also write it to play_target_time for the shock module.
 */
static uint8_t copy_parameters(void)
{
    store.play_times.target_time.min = player_interface.param1;
    store.play_times.target_time.sec = player_interface.param2;
    store.play_times.target_time.frm = player_interface.param3;
    play_target_time = store.play_times.target_time;
    return READY;
}

/**
 * Drive the laser to the seek target using the iterative jump_time() engine.
 * Returns the result of jump_time() directly.
 */
static uint8_t search_for_address(void)
{
    return jump_time(&store.play_times.target_time);
}

/* ── PLAY_READ_SUBCODE ───────────────────────��──────────────────────── */
/**
 * Return the current Q-channel frame to the host.
 *
 * If the drive is paused, check whether the paused Q_buffer matches the
 * host's requested address type (param1 = address nibble, param2 = index).
 * If not paused, scan through subcode frames until a matching frame is found
 * or the search times out.
 *
 * The result address is written to player_interface.param1 as a pointer
 * cast — a legacy convention from the original firmware where param1 was
 * used to return data addresses.
 *
 * Three-level timeout:
 *   Two 255-tick (~2 s) windows (reload_play_timer0 and reload_play_timer1)
 *   before declaring SUBCODE_NOT_FOUND.  During each window at least one
 *   subcode frame must be received; if none arrive within the window the
 *   error is SUBCODE_TIMEOUT_ERROR.
 */
static uint8_t monitor_subcodes(void)
{
    /* ── Phase 0+0: initialise ── */
    if (!play_phase1 && !play_phase0) {
        if ((play_status & 0x0Fu) == PAUSE_MODE) {
            /* Paused: the subcode is already in store.play_subcode */
            if (req_subc_stored_subc()) {
                player_interface.param1 = (uint8_t)(uintptr_t)&store.play_subcode;
                return PROCESS_READY;
            }
            player_error = ILLEGAL_PARAMETER;
            return CD_ERROR_STATE;
        }
        start_subcode_reading();
        play_timer          = 255u;
        reload_play_timer0  = 1;
        reload_play_timer1  = 1;
        track_found         = 0;
        play_phase0         = 1;
    }

    /* ── Phase 1+0: scan for matching frame ── */
    if (!play_phase1 && play_phase0) {
        if (play_timer == 0) {
            if (!track_found) {
                player_error = SUBCODE_TIMEOUT_ERROR;
                return CD_ERROR_STATE;
            }
            /* At least one frame was seen — reset for next window */
            track_found = 0;
            play_timer  = 255u;
            if (reload_play_timer1) {
                if (reload_play_timer0) reload_play_timer0 = 0;
                else { reload_play_timer1 = 0; reload_play_timer0 = 1; }
            } else {
                if (reload_play_timer0) reload_play_timer0 = 0;
                else {
                    player_error = SUBCODE_NOT_FOUND;
                    return CD_ERROR_STATE;
                }
            }
        }

        if (is_subcode(ALL_SUBCODES)) {
            track_found = 1;
            if ((play_status & 0x0Fu) == TOC_MODE && is_subcode(PROGRAM_AREA)) {
                /* TOC mode: if we have drifted into the program area,
                 * jump back into the lead-in to continue reading TOC data. */
                servo_jump(TRACKS_INTO_LEADIN);
                play_phase1 = 1;
                play_phase0 = 0;   /* → phase 0+1: waiting for jump */
            } else {
                set_subcode_buffer();
                if (req_subc_stored_subc()) {
                    player_interface.param1 = (uint8_t)(uintptr_t)&store.play_subcode;
                    return PROCESS_READY;
                }
                start_subcode_reading();
            }
        }
    }

    /* ── Phase 0+1: waiting for lead-in jump to complete ── */
    if (play_phase1 && !play_phase0) {
        uint8_t es = get_servo_process_state();
        if (es == READY) {
            start_subcode_reading();
            play_phase1 = 0;
            play_phase0 = 1;   /* back to scanning */
        } else {
            return es;
        }
    }

    return BUSY;
}

/* ── PLAY_READ_TOC ───────────────────────��──────────────────────��───── */
/**
 * If param2 == 0: enter TOC_MODE (continuous TOC reading).
 * If param2 != 0: remain in IDLE_MODE (one-shot TOC query).
 */
static uint8_t set_toc_mode(void)
{
    if (player_interface.param2 == 0)
        play_status = (uint8_t)((play_status & 0xF0u) | TOC_MODE);
    else
        play_status = (uint8_t)((play_status & 0xF0u) | IDLE_MODE);
    return PROCESS_READY;
}

/**
 * Seek to absolute time 00:02:00 (inside the lead-in) then jump
 * to the start of the lead-in (TRACKS_INTO_LEADIN) to position
 * the laser at the beginning of the TOC.
 */
static uint8_t jump_to_toc(void)
{
    if (!play_phase0) {
        /* First entry: seek to a position near the start of the lead-in */
        store.play_times.target_time.min = 0;
        store.play_times.target_time.sec = 2;
        store.play_times.target_time.frm = 0;
        uint8_t es = jump_time(&store.play_times.target_time);
        if (es == READY) {
            /* Now perform the final jump to the very start of the lead-in */
            servo_jump(TRACKS_INTO_LEADIN);
            play_phase0 = 1;
        } else {
            return es;
        }
    } else {
        /* Second entry: wait for the lead-in jump to complete */
        return get_servo_process_state();
    }
    return BUSY;
}

/* ── PLAY_PREPARE_SPEED_CHANGE ──────────────────────────────────────── */
/**
 * Record the current play position before changing speed so that
 * PLAY_RESTORE_SPEED_CHANGE can seek back to the same point afterward.
 *
 * Phase 0+0: if currently tracking, mute audio and set phase.
 * Phase 1+0: disarm shock, arm subcode, wait for a frame.
 *   - If in lead-in: speed change while reading TOC — no position to save.
 *   - If in program area: save the position and set RETURN_TO_TARGET.
 */
static uint8_t prepare_before_speed_change(void)
{
    if (!play_phase1 && !play_phase0) {
        if ((play_status & 0x0Fu) != TRACKING_MODE) return READY;
        mute_on();
        play_phase0 = 1;
    }

    if (!play_phase1 && play_phase0) {
        uint8_t es = shock_detector_off();
        if (es == READY) {
            start_subcode_reading();
            play_timer  = SUBCODE_TIMEOUT_VALUE;
            play_phase1 = 1;
            play_phase0 = 0;
        } else return es;
    }

    if (play_phase1 && !play_phase0) {
        if (is_subcode(LEADIN_AREA)) return READY;   /* in lead-in — no save needed */
        if (is_subcode(ABS_TIME)) {
            move_abstime(&store.play_times.target_time);
            play_status |= RETURN_TO_TARGET;
            return READY;
        }
        if (play_timer == 0) {
            player_error = SUBCODE_TIMEOUT_ERROR;
            return CD_ERROR_STATE;
        }
    }
    return BUSY;
}

/* ── PLAY_RESTORE_SPEED_CHANGE ────────────────────────────���─────────── */
/**
 * After the speed change has completed, seek back to the saved position
 * (if RETURN_TO_TARGET was set) and re-arm the shock detector.
 *
 * Phase 0+0: if TRACKING_MODE and RETURN_TO_TARGET: seek to saved address.
 *            if TRACKING_MODE but not RETURN_TO_TARGET: just unmute.
 *            otherwise: nothing to do (were not tracking when change started).
 * Phase 1+0: wait for jump_time() to complete.
 * Phase 0+1: re-arm shock detector.
 */
static uint8_t restore_after_speed_change(void)
{
    if (!play_phase1 && !play_phase0) {
        if ((play_status & 0x07u) == TRACKING_MODE) {
            if (play_status & RETURN_TO_TARGET) {
                play_status &= (uint8_t)~RETURN_TO_TARGET;
                play_phase0  = 1;
            } else {
                mute_off();
                play_phase1 = 1;
            }
        } else return READY;
    }

    if (!play_phase1 && play_phase0) {
        uint8_t es = jump_time(&store.play_times.target_time);
        if (es == READY) {
            mute_off();
            play_phase1 = 1;
            play_phase0 = 0;
        } else return es;
    }

    if (play_phase1 && !play_phase0) {
        return shock_detector_on();
    }
    return BUSY;
}

/* ── PLAY_JUMP_TRACKS ───────────────────────────────────────────────── */
/**
 * Jump ±N tracks relative to the current position.
 *
 * param1:param2 (16-bit, high:low) is the signed groove count.  We negate
 * it before passing to servo_jump() because the player uses "positive = later
 * in the program" while servo_jump() convention is "positive = outward".
 * Actually servo_jump already negates, so we pass the raw value.
 */
static uint8_t jump_tracks(void)
{
    int_hl_t nr;
    nr.b.high = player_interface.param1;
    nr.b.low  = player_interface.param2;

    if (!play_phase0) {
        servo_jump(nr.val);
        play_phase0 = 1;
        return BUSY;
    }
    return get_servo_process_state();
}

/* =========================================================================
 * set_subcode_buffer — copy Q_buffer → store.play_subcode (hex → BCD)
 *
 * The COMMO protocol transmits all subcode data in BCD (as it appears on
 * the disc), but Q_buffer holds hex values after subcode_module() converts
 * them.  This function re-encodes to BCD for the host.
 * ====================================================================== */

static void set_subcode_buffer(void)
{
    /* Raw copy of all 10 bytes first */
    store.play_subcode.conad         = Q_buffer[0];
    store.play_subcode.tno           = Q_buffer[1];
    store.play_subcode.index         = Q_buffer[2];
    store.play_subcode.r_time.min    = Q_buffer[3];
    store.play_subcode.r_time.sec    = Q_buffer[4];
    store.play_subcode.r_time.frm    = Q_buffer[5];
    store.play_subcode.zero          = Q_buffer[6];
    store.play_subcode.a_time.min    = Q_buffer[7];
    store.play_subcode.a_time.sec    = Q_buffer[8];
    store.play_subcode.a_time.frm    = Q_buffer[9];

    uint8_t conad_lo = store.play_subcode.conad & 0x0Fu;

    if (conad_lo == 0x01u) {
        /* Standard time-code frame: re-encode all time fields to BCD */
        if (store.play_subcode.tno != 0xAAu)
            store.play_subcode.tno = hex_to_bcd(store.play_subcode.tno);
        hex_to_bcd_time(&store.play_subcode.r_time, &store.play_subcode.r_time);
        if (store.play_subcode.index == 0xA0u ||
            store.play_subcode.index == 0xA1u) {
            /* TOC pointer: only AMIN is a track number */
            store.play_subcode.a_time.min =
                hex_to_bcd(store.play_subcode.a_time.min);
        } else {
            hex_to_bcd_time(&store.play_subcode.a_time,
                            &store.play_subcode.a_time);
            if (store.play_subcode.index != 0xA2u)
                store.play_subcode.index = hex_to_bcd(store.play_subcode.index);
        }
    } else if (conad_lo == 0x05u) {
        /* Multi-session pointer: re-encode r_time if it contains a valid address */
        if (store.play_subcode.tno   == 0x00u &&
            store.play_subcode.index == 0xB0u &&
            store.play_subcode.r_time.min != 0xFFu)
        {
            hex_to_bcd_time(&store.play_subcode.r_time,
                            &store.play_subcode.r_time);
        }
    }
}

/**
 * @brief  Check whether the subcode in store matches what the host requested.
 *
 * player_interface.param1 = requested address nibble (0 = any)
 * player_interface.param2 = requested index byte    (0xFF = any)
 */
static uint8_t req_subc_stored_subc(void)
{
    if (player_interface.param1 == 0) return TRUE;
    if ((store.play_subcode.conad & 0x0Fu) == player_interface.param1) {
        if (store.play_subcode.index == player_interface.param2 ||
            player_interface.param2 == 0xFFu)
            return TRUE;
    }
    return FALSE;
}

/* =========================================================================
 * play — command entry point
 *
 * Called from the player dispatch table.  On the first call with a new
 * command (different command or !play_command_busy) it initialises state
 * and performs pre-flight validation.  Subsequent calls with the same
 * command just return the current status.
 * ====================================================================== */

uint8_t play(uint8_t play_cmd)
{
    if ((play_process >> 4) != play_cmd || !play_command_busy) {
        /* New command — reset state */
        play_process = (uint8_t)(play_cmd << 4);
        play_phase0  = 0;
        play_phase1  = 0;
        play_command_busy = 1;
        play_monitor      = 0;

        /* Clear the RETURN_TO_TARGET flag for most commands (speed-restore
         * keeps it because it was set by the prepare step). */
        if (play_cmd == PLAY_RESTORE_SPEED_CHANGE)
            play_status &= 0x0Fu;
        else
            play_status &= 0x07u;

        /* Pre-flight validation: some commands are only valid in specific
         * play modes.  If invalid, set the error and complete immediately
         * (play_monitor = 1 bypasses the step functions). */
        switch (play_cmd) {

        case PAUSE_ON:
            /* Cannot pause if not tracking, or if reading TOC */
            if (!servo_tracking() || (play_status & 0x0Fu) == TOC_MODE) {
                if (player_error == NO_ERROR) player_error = ILLEGAL_COMMAND;
                play_status  |= (uint8_t)(CD_ERROR_STATE << 4);
                play_monitor  = 1;
            } else if ((play_status & 0x0Fu) == PAUSE_MODE) {
                /* Already paused — report success immediately */
                play_status  |= (uint8_t)(READY << 4);
                play_monitor  = 1;
            }
            break;

        case PAUSE_OFF:
            if (!servo_tracking() || (play_status & 0x0Fu) != PAUSE_MODE) {
                play_monitor = 1;
                if (servo_tracking() || (play_status & 0x0Fu) == TRACKING_MODE)
                    play_status |= (uint8_t)(READY << 4);
                else {
                    if (player_error == NO_ERROR) player_error = ILLEGAL_COMMAND;
                    play_status |= (uint8_t)(CD_ERROR_STATE << 4);
                }
            }
            break;

        case PLAY_READ_SUBCODE:
            if (!servo_tracking()) {
                if (player_error == NO_ERROR) player_error = ILLEGAL_COMMAND;
                play_status  |= (uint8_t)(CD_ERROR_STATE << 4);
                play_monitor  = 1;
            }
            break;

        case PLAY_SET_VOLUME:
        case PLAY_PREPARE_SPEED_CHANGE:
            if (player_error != NO_ERROR) {
                play_status  |= (uint8_t)(CD_ERROR_STATE << 4);
                play_monitor  = 1;
            }
            break;

        case PLAY_JUMP_TRACKS:
            if (!servo_tracking() ||
                (play_status & 0x0Fu) == TOC_MODE ||
                (play_status & 0x0Fu) == PAUSE_MODE)
            {
                if (player_error == NO_ERROR) player_error = ILLEGAL_COMMAND;
                play_status  |= (uint8_t)(CD_ERROR_STATE << 4);
                play_monitor  = 1;
            }
            break;

        default:
            break;
        }

        if (!play_monitor)
            play_status |= (uint8_t)(BUSY << 4);
    }

    if ((play_status >> 4) != BUSY)
        play_command_busy = 0;

    return (uint8_t)(play_status >> 4);
}

/* =========================================================================
 * Play process dispatch table
 *
 * play_processes[play_cmd][step] gives the step function for each command.
 * NULL entries terminate the sequence early (PROCESS_READY is returned
 * automatically when a NULL is encountered).
 * ====================================================================== */

#define MAX_PLAY_STEPS  5

typedef uint8_t (*play_fn_t)(void);

static const play_fn_t play_processes[][MAX_PLAY_STEPS] = {
/*  0 PLAY_IDLE                */  { playing_idle,                NULL,                NULL,                 NULL,              NULL              },
/*  1 PLAY_STARTUP             */  { play_process_ready,          NULL,                NULL,                 NULL,              NULL              },
/*  2 PAUSE_ON                 */  { mute_on,                     shock_detector_off,  pause_on_init,        set_pause_mode,    NULL              },
/*  3 PAUSE_OFF                */  { restore_target_address,      shock_detector_on,   set_tracking_mode,    NULL,              NULL              },
/*  4 JUMP_TO_ADDRESS          */  { copy_parameters,             search_for_address,  shock_detector_on,    set_tracking_mode, NULL              },
/*  5 PLAY_TRACK               */  { play_process_ready,          NULL,                NULL,                 NULL,              NULL              },
/*  6 PLAY_READ_SUBCODE        */  { monitor_subcodes,            NULL,                NULL,                 NULL,              NULL              },
/*  7 PLAY_READ_TOC            */  { jump_to_toc,                 set_toc_mode,        play_process_ready,   NULL,              NULL              },
/*  8 PLAY_PREPARE_SPEED_CHANGE*/  { prepare_before_speed_change, play_process_ready,  NULL,                 NULL,              NULL              },
/*  9 PLAY_RESTORE_SPEED_CHANGE*/  { restore_after_speed_change,  play_process_ready,  NULL,                 NULL,              NULL              },
/* 10 PLAY_SET_VOLUME          */  { play_process_ready,          NULL,                NULL,                 NULL,              NULL              },
/* 11 PLAY_JUMP_TRACKS         */  { shock_detector_off,          jump_tracks,         restore_target_address, shock_detector_on, play_process_ready },
};

/* =========================================================================
 * Background monitors
 *
 * These run when play_monitor == 1 (command complete, but background work
 * is still needed to keep the disc in the correct position).
 * ====================================================================== */

/**
 * @brief  Keep the laser on the paused position.
 *
 * During PAUSE_MODE the servo may drift slightly as the disc rotates.
 * We compare the current Q_buffer absolute time against the stored pause
 * address (in BCD in store.play_subcode).  If we have drifted we issue
 * a corrective servo_jump().
 *
 * The pause address is in BCD (as captured by set_subcode_buffer()); we
 * convert to hex for comparison.
 */
static void monitor_pausing(void)
{
    if (get_servo_process_state() != READY) return;

    if (is_subcode(ABS_TIME)) {
        cd_time_t current, pause_hex;

        /* Current position is already in hex (from subcode_module()) */
        current.min = Q_buffer[7];
        current.sec = Q_buffer[8];
        current.frm = Q_buffer[9];

        /* Convert the stored BCD pause address to hex for comparison */
        if (store.play_subcode.tno == 0)
            bcd_to_hex_time_local(&store.play_subcode.r_time, &pause_hex);
        else
            bcd_to_hex_time_local(&store.play_subcode.a_time, &pause_hex);

        if (compare_time(&current, &pause_hex) == BIGGER) {
            /* Drifted past the pause point — seek back */
            servo_jump(calc_tracks(&current, &pause_hex));
        } else {
            cd_time_t diff;
            subtract_time(&pause_hex, &current, &diff);
            if (diff.min != 0 || diff.sec != 0) {
                cd_time_t target;
                add_time(&diff, &current, &target);
                servo_jump(calc_tracks(&current, &target));
            }
        }
        start_subcode_reading();

    } else if (is_subcode(FIRST_LEADIN_AREA)) {
        /* Laser fell into lead-in due to a severe shock — jump outward */
        servo_jump(100);
    }
}

/**
 * @brief  Keep the laser in the lead-in during TOC reading.
 *
 * During TOC_MODE the servo may drift into the program area.  If it does,
 * we jump back into the lead-in to continue reading TOC entries.
 */
static void monitor_toc_reading(void)
{
    if (get_servo_process_state() == READY) {
        if (is_subcode(PROGRAM_AREA))
            servo_jump(TRACKS_INTO_LEADIN);
    }
}

/* =========================================================================
 * execute_play_functions — called once per main-loop iteration
 *
 * Drives the active play command through its step functions (when
 * !play_monitor) and runs the appropriate background monitor (when
 * play_monitor == 1).
 * ====================================================================== */

void execute_play_functions(void)
{
    if (!play_monitor) {
        uint8_t cmd  = play_process >> 4;
        uint8_t step = play_process & 0x0Fu;

        if (cmd  >= 12u) goto monitor;
        if (step >= MAX_PLAY_STEPS) goto monitor;

        play_fn_t fn = play_processes[cmd][step];
        if (fn == NULL) goto monitor;

        switch (fn()) {
        case PROCESS_READY:
            /* Command is fully complete */
            play_status  = (uint8_t)((play_status & 0x0Fu) | (uint8_t)(READY << 4));
            play_monitor = 1;
            break;

        case READY:
            /* This step is done — advance to the next step and clear phase vars */
            play_process++;
            play_phase0 = 0;
            play_phase1 = 0;
            break;

        case CD_ERROR_STATE:
            play_status  = (uint8_t)((play_status & 0x0Fu) | (uint8_t)(CD_ERROR_STATE << 4));
            play_monitor = 1;
            break;

        case BUSY:
        default:
            break;
        }
    }

monitor:
    if (play_monitor) {
        uint8_t mode = play_status & 0x0Fu;
        uint8_t stat = play_status >> 4;

        if (stat == READY) {
            if      (mode == PAUSE_MODE) monitor_pausing();
            else if (mode == TOC_MODE)   monitor_toc_reading();
            /* TRACKING_MODE: shock.c handles; IDLE_MODE: nothing to do */
        }
    }
}
