/**
 * @file  timer.h
 * @brief Software timer subsystem — 8 ms tick via Pico hardware repeating timer.
 *
 * A single hardware repeating_timer fires every 8 ms and decrements all
 * non-zero entries in the timers[] array.  Modules obtain a timer by
 * writing a count; they poll for expiry by testing for zero.
 *
 * The 8 ms period was chosen to match the original 8051 firmware, where
 * T0 produced an 8 ms interrupt.  At 75 CD frames per second this gives
 * approximately 6 software ticks per frame, which is enough resolution
 * for all servo and play timing without overloading the CPU.
 *
 * The SCOR (subcode clock) falling-edge interrupt is also installed here
 * because it shares the GPIO interrupt handler infrastructure.
 */

#pragma once
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Timer IDs
 *
 * Each ID is an index into timers[].  Assign exclusive ownership to avoid
 * collisions; the comments below describe each timer's primary user.
 * ---------------------------------------------------------------------- */
typedef enum {
    TIMER_SERVO       = 0,  /**< servo.c:  servo state-machine general timeout     */
    TIMER_MODULE      = 1,  /**< (spare)   available for future use                */
    TIMER_COMMO       = 2,  /**< commo.c:  inter-byte gap timeout                  */
    TIMER_KICK_BRAKE  = 3,  /**< servo.c:  kick/brake phase duration during jumps  */
    TIMER_PROGRESS    = 4,  /**< shock.c:  shock-detection sampling interval       */
    TIMER_SIMULATION  = 5,  /**< driver.c: simulated motor-start signal timer      */
    TIMER_SEARCH      = 6,  /**< (spare)   available for future use                */
    TIMER_PLAY        = 7,  /**< play.c / strtstop.c: subcode and play timeouts    */
    TIMER_COUNT             /**< Total number of timers (must be last)             */
} timer_id_t;

/* -------------------------------------------------------------------------
 * Global timer array
 *
 * Each entry is decremented to zero and held there.  Write a count to arm;
 * read zero to detect expiry.  Writes are not atomic vs the ISR, but since
 * all writes happen in the main loop (not in IRQ context) and the ISR only
 * decrements, a write that races with a decrement can miss one tick at
 * most — acceptable for the timing tolerances in this firmware.
 * ---------------------------------------------------------------------- */
extern volatile uint8_t timers[TIMER_COUNT];

/* -------------------------------------------------------------------------
 * SCOR (subcode clock) interrupt state
 *
 * scor_edge is set to 1 by the GPIO ISR on every falling edge of PIN_SCOR.
 * It is cleared by cd6_read_subcode() after being consumed.
 *
 * scor_counter is a down-counter decremented by the same ISR.  It is
 * loaded by init_scor_counter() and tested with zero_scor_counter() to
 * wait for an exact frame count during seek operations.
 * ---------------------------------------------------------------------- */
extern volatile uint8_t scor_counter;
extern volatile uint8_t scor_edge;

/* -------------------------------------------------------------------------
 * Legacy blocking-delay variable
 *
 * Set delay_byte to a count and call delay() to block for approximately
 * delay_byte × 500 µs.  Matches the original 8051 firmware's delay()
 * which polled a down-counter decremented by the T0 ISR.
 * ---------------------------------------------------------------------- */
extern volatile uint8_t delay_byte;

/* -------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------- */

/**
 * @brief  Initialise the 8 ms repeating timer and the SCOR GPIO interrupt.
 *
 * Called once from player_init() before any sub-module that uses timers[].
 * The GPIO direction and pull-up for PIN_SCOR must be configured by
 * driver_init() before timer_init() is called (player_init() ensures this
 * order: driver_init() → timer_init()).
 */
void timer_init(void);

/**
 * @brief  Blocking delay — approximately delay_byte × 500 µs.
 *
 * Uses sleep_us() internally; the CPU is blocked for the entire duration.
 * Only used during the power-on reset sequence in driver.c.
 */
void delay(void);

/**
 * @brief  Blocking delay of n × 500 µs.
 * @param  n  Number of 500 µs periods to wait.
 */
void delay_us_500x(uint8_t n);
