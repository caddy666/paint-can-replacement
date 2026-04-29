/**
 * @file  shock.c
 * @brief Shock / scratch detector and recovery module.
 *
 * Monitors disc progress during playback by verifying that the absolute
 * disc time advances by a plausible amount each interval.  When progress
 * stalls (scratch, shock) or jumps unexpectedly (shock forward), the module:
 *   1. Mutes the audio immediately.
 *   2. Seeks the laser back to the last known good position + 40 frames.
 *   3. Unmutes once the seek succeeds.
 *
 * ── Progress window ───────────────────────────────────────────────────
 *
 *   Each interval is PROGRESS_N1 (40 ticks × 8 ms = 320 ms) at 1× speed,
 *   or PROGRESS_N2 (20 ticks = 160 ms) at 2× speed.
 *
 *   At 75 frames per second and 1× speed, 320 ms = 24 frames of expected
 *   progress.  The window accepts progress in the range:
 *     Lower bound:  last_absolute_time + 00:00:15 (15 frames ≈ 200 ms)
 *     Upper bound:  last_absolute_time + 00:01:00 (1 second = 75 frames)
 *
 *   If current_time < lower_bound: too slow (stall / repeat) → seek back.
 *   If current_time > upper_bound: too fast (skip forward) → seek back.
 *   If lower_bound ≤ current_time ≤ upper_bound: normal → update reference.
 *
 * ── Recovery ──────────────────────────────────────────────────────────
 *
 *   The seek target is last_absolute_time + 00:00:40 (40 frames ahead of
 *   the last known good position).  This gives the drive a small amount of
 *   run-up before the point where the fault occurred.
 *
 *   If jump_time() returns CD_ERROR_STATE the target is unreachable (e.g.
 *   end of disc); we advance last_absolute_time forward and try again.
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 */

#include <stdint.h>
#include <string.h>

#include "defs.h"
#include "driver.h"
#include "serv_def.h"
#include "timer.h"

/* =========================================================================
 * External references
 * ====================================================================== */

extern volatile uint8_t timers[];
#define progress_timer timers[TIMER_PROGRESS]   /**< Interval between progress checks */

extern int n1_speed;   /* 1 = single speed, 0 = double speed (from driver.c) */

/* From subcode.c */
extern uint8_t is_subcode(uint8_t mode);
extern void    move_abstime(cd_time_t *p);

/* From play.c: performs a seek to an absolute disc time */
extern uint8_t jump_time(cd_time_t *t);

/* From utils/maths.c */
extern void    add_time(const cd_time_t *a, const cd_time_t *b, cd_time_t *r);
extern uint8_t compare_time(const cd_time_t *a, const cd_time_t *b);

/* =========================================================================
 * Timing constants (in 8 ms timer ticks)
 * ====================================================================== */
#define PROGRESS_N1   40u   /**< 40 × 8 ms = 320 ms progress-check interval at 1× */
#define PROGRESS_N2   20u   /**< 20 × 8 ms = 160 ms progress-check interval at 2× */

/* =========================================================================
 * Module state
 * ====================================================================== */

/**
 * Phase flag:
 *   0 — monitoring phase: counting down progress_timer, checking disc time.
 *   1 — recovery phase:   seeking to last_absolute_time + 40 frames.
 */
static uint8_t   shock_phase0          = 0;

/** Non-zero while shock detection is active (between shock_detector_on and _off). */
uint8_t          shock_recovery_active = 0;

/** Disc time at the end of the last successful progress check. */
static cd_time_t last_absolute_time;

/* =========================================================================
 * Static time-allocation pool
 *
 * The original 8051 firmware used a dynamic alloc/free_struct_time() heap
 * to temporarily hold two cd_time_t values during the progress check.
 * We replace that with a fixed two-entry static array — this is a bare-metal
 * target with no heap, and we never need more than two temps simultaneously.
 * ====================================================================== */

static cd_time_t time_pool[2];

/** Clear both pool entries at the start of each shock_recover() call. */
static void reset_time_pool(void)
{
    memset(time_pool, 0, sizeof(time_pool));
}

/* =========================================================================
 * Mute helpers — thin wrappers so the intent is self-documenting
 * ====================================================================== */

static uint8_t mute_on(void)  { cd6_wr(MUTE);       return READY; }
static uint8_t mute_off(void) { cd6_wr(FULL_SCALE);  return READY; }

/* =========================================================================
 * init_shock_registers — arm the progress monitor
 *
 * Called when playback starts (from shock_detector_on) and each time a
 * normal progress update is recorded.  Reloads the timer and captures the
 * current absolute disc time as the new reference.
 * ====================================================================== */

void init_shock_registers(void)
{
    progress_timer = n1_speed ? PROGRESS_N1 : PROGRESS_N2;
    move_abstime(&last_absolute_time);
}

/* =========================================================================
 * shock_detector_off / shock_detector_on
 *
 * Called from play.c to bracket periods of active playback.  The detector
 * is off during TOC reads, pauses, and seek operations because disc time
 * will not advance normally during those states.
 * ====================================================================== */

/**
 * @brief  Disarm the shock detector.
 *
 * Called at the start of any non-playing operation (pause, seek, stop).
 */
uint8_t shock_detector_off(void)
{
    shock_recovery_active = 0;
    return READY;
}

/**
 * @brief  Arm the shock detector for active playback.
 *
 * Called when playback resumes.  play_target_time is set by play.c to the
 * target seek address before calling shock_detector_on(), so the initial
 * reference time is the position we are about to play from.
 */
uint8_t shock_detector_on(void)
{
    extern cd_time_t play_target_time;
    progress_timer        = n1_speed ? PROGRESS_N1 : PROGRESS_N2;
    last_absolute_time    = play_target_time;
    shock_recovery_active = 1;
    shock_phase0          = 0;
    return READY;
}

/* =========================================================================
 * shock_recover — main tick (called once per main-loop iteration)
 *
 * Phase 0 (monitoring):
 *   Wait for progress_timer to expire.  When it does, read the current
 *   absolute disc time and compare it against the progress window:
 *
 *       O = last_absolute_time
 *       B = O + 00:00:15  (lower bound: minimum expected progress)
 *       N = current Q_buffer absolute time
 *       U = O + 00:01:00  (upper bound: maximum expected progress)
 *
 *   B < N < U → normal progress: update last_absolute_time and reload timer.
 *   N ≤ B     → too slow (stuck / repeating): shock_phase0 = 1, mute.
 *   N ≥ U     → too fast (shock forward):     shock_phase0 = 1, mute.
 *
 *   If in lead-in while we should be playing: shock_phase0 = 1, mute.
 *
 * Phase 1 (recovery):
 *   Seek to last_absolute_time + 00:00:40 using jump_time().
 *   On READY: unmute, reset to phase 0 and update reference.
 *   On ERROR: advance last_absolute_time to the failed target and retry
 *             (the disc may be scratched over a wider area; keep seeking
 *              forward until we find playable data).
 * ====================================================================== */

void shock_recover(void)
{
    if (!shock_recovery_active) return;

    reset_time_pool();
    cd_time_t *position = &time_pool[0];   /* current disc time          */
    cd_time_t *window   = &time_pool[1];   /* reused for both bounds     */

    if (!shock_phase0) {
        /* ── Phase 0: progress monitoring ── */
        if (progress_timer != 0) return;   /* interval not yet expired */

        if (is_subcode(ABS_TIME)) {
            move_abstime(position);

            /* Check lower bound: position must be > last + 15 frames */
            window->min = 0; window->sec = 0; window->frm = 0x0F;
            add_time(&last_absolute_time, window, window);

            if (compare_time(position, window) == BIGGER) {
                /* Progress detected — check upper bound */
                window->min = 0; window->sec = 1; window->frm = 0;
                add_time(&last_absolute_time, window, window);

                if (compare_time(window, position) == SMALLER) {
                    /* Position is more than 1 s ahead of reference — shock */
                    shock_phase0 = 1;
                    mute_on();
                } else {
                    /* Within window: normal playback — advance reference */
                    init_shock_registers();
                }
            } else {
                /* Position did not advance enough — stall or scratch */
                shock_phase0 = 1;
                mute_on();
            }

        } else if (is_subcode(FIRST_LEADIN_AREA)) {
            /* Laser ended up in the lead-in during playback — severe shock */
            shock_phase0 = 1;
            mute_on();
        }
        /* If neither condition is true the subcode simply isn't ready yet;
         * we will check again next tick. */
    }

    if (shock_phase0) {
        /* ── Phase 1: recovery seek ──
         * Target = last known good position + 40 frames.
         * 0x28 = 40 decimal = 40 frames (less than 75 so no carry into seconds). */
        window->min = 0; window->sec = 0; window->frm = 0x28;
        add_time(&last_absolute_time, window, window);

        uint8_t status = jump_time(window);
        if (status == READY) {
            /* Arrived at target: unmute, reset monitor */
            shock_phase0 = 0;
            init_shock_registers();
            mute_off();
        } else if (status == CD_ERROR_STATE) {
            /* Target unreachable: advance the reference point past the bad spot
             * and try seeking further along. */
            last_absolute_time = *window;
        }
        /* BUSY: still seeking; continue next tick */
    }
}
