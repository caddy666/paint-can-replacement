/**
 * @file  timer.c
 * @brief Software timer subsystem and SCOR interrupt.
 *
 * ── Software timers ───────────────────────────────────────────────────────
 *
 * Replaces the 8051 T0 hardware timer with a RP2350 repeating_timer that
 * fires every 8 ms and decrements all non-zero entries in the timers[] array.
 *
 * The 8 ms period matches the original 8051 T0 overflow period (11.0592 MHz
 * crystal, 12-clock mode, 8-bit auto-reload, 205-count preload).
 * At 75 CD frames per second, 8 ms ≈ 0.6 frames — fine enough that timing
 * constants are expressed in frames without needing sub-frame resolution.
 *
 * Timer values are uint8_t, so the maximum delay is 255 × 8 ms = 2.04 s.
 * Longer delays are formed by counting multiple expirations in the module
 * code (e.g. the motor spin-up uses two back-to-back timer loads).
 *
 * The timer ISR is scheduled with a negative period (-8000 µs), which means
 * the RP2350 alarm fires 8000 µs after the PREVIOUS callback ENDED, not
 * started.  This prevents drift accumulation if a callback occasionally
 * takes a few microseconds to return.
 *
 * ── SCOR interrupt ────────────────────────────────────────────────────────
 *
 * SCOR (subcode clock output) is a falling-edge signal from the CXD2500BQ
 * that fires once per CD frame (75 Hz, every ~13.3 ms).  The ISR serves two
 * purposes:
 *
 *   scor_edge = 1   — signals subcode_module() that a new Q-channel frame
 *                     has been clocked into the CXD2500's serial output
 *                     register and is ready to be shifted out via QDA/QCL.
 *
 *   scor_counter--  — counts down a frame-accurate timer used by the servo
 *                     module to position the laser after a seek.  The servo
 *                     loads scor_counter with the number of frames to wait
 *                     before sampling the subcode address, which is more
 *                     accurate than the 8 ms software timer because it is
 *                     synchronised directly to the disc rotation.
 *
 * ── delay() ───────────────────────────────────────────────────────────────
 *
 * The original 8051 firmware had a busy-wait delay() that polled a counter
 * decremented by the T0 ISR (every ~500 µs at its rate).  We replace it
 * with sleep_us(500) × delay_byte, preserving the timing while removing the
 * shared-variable dependency.  This is only used during IC reset sequencing
 * (reset_dsic2_cd6) where blocking is acceptable.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdint.h>

#include "timer.h"
#include "gpio_map.h"
#include <stddef.h>

/* =========================================================================
 * Software timer array — one entry per timer_id_t (see timer.h)
 *
 * Volatile: written by the 8 ms ISR on the RP2350 alarm hardware; read
 * by main-loop code.  The RP2350 runs single-core for this firmware so
 * there is no SMP hazard, but volatile prevents the compiler from caching
 * timer values in registers across loop iterations.
 *
 * Non-atomic write warning: the ISR decrements each byte independently
 * with a single byte write, which is inherently atomic on a byte-addressed
 * bus.  Code that SETS a timer (e.g. servo_timer = 40) must do so from the
 * main loop only, never from the ISR.
 * ====================================================================== */
volatile uint8_t timers[TIMER_COUNT] = {0};

/* =========================================================================
 * SCOR interrupt state
 * ====================================================================== */

/** Incremented by the SCOR ISR each frame; decremented by the servo module
 *  to count frames elapsed since a seek command was issued. */
volatile uint8_t scor_counter = 0;

/** Set to 1 by the SCOR ISR when a new subcode frame is available.
 *  Cleared by subcode_module() after the frame has been shifted out. */
volatile uint8_t scor_edge    = 0;

/* =========================================================================
 * Delay counter (legacy blocking delay helper)
 * ====================================================================== */

/** Loaded by caller; delay() busy-waits for delay_byte × 500 µs. */
volatile uint8_t delay_byte = 0;

/* =========================================================================
 * Private: 8 ms repeating timer ISR
 * ====================================================================== */
static struct repeating_timer s_hw_timer;

/**
 * @brief  Decrement all non-zero software timers.
 *
 * Called every 8 ms by the RP2350 repeating_timer hardware.  Each timer
 * counts down from its loaded value to zero and then stops (saturates at 0).
 * The main-loop code checks for timer == 0 as the expiry condition.
 *
 * @return true to keep the repeating timer active.
 */
static bool timer_callback(struct repeating_timer *t)
{
    (void)t;
    for (int i = 0; i < TIMER_COUNT; i++) {
        if (timers[i]) timers[i]--;
    }
    return true;
}

/* =========================================================================
 * Private: SCOR GPIO interrupt (falling edge on PIN_SCOR)
 * ====================================================================== */

/**
 * @brief  Service the SCOR falling-edge interrupt.
 *
 * Each edge represents exactly one CD frame boundary (75 Hz).  We:
 *   1. Set scor_edge = 1 so subcode_module() knows to shift out a new frame.
 *   2. Decrement scor_counter if non-zero, providing a frame-accurate timer
 *      for use by the servo module during seek convergence.
 */
static void scor_gpio_callback(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;

    scor_edge = 1;

    if (scor_counter > 0)
        scor_counter--;
}

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the software timer bank and SCOR interrupt.
 *
 * Starts the 8 ms repeating alarm and registers the SCOR falling-edge
 * callback.  The SCOR GPIO direction and pull configuration are handled
 * by driver_init(); timer_init() only registers the interrupt handler.
 *
 * Called once from player_init() during firmware boot.
 */
void timer_init(void)
{
    /* Negative period = schedule next fire 8000 µs after previous callback END
     * (avoids jitter accumulation compared to scheduling from callback START). */
    add_repeating_timer_us(-8000, timer_callback, NULL, &s_hw_timer);

    /* SCOR falling-edge interrupt — PIN_SCOR is already configured as an input
     * with pull-up by driver_init() before timer_init() is called. */
    gpio_set_irq_enabled_with_callback(
        PIN_SCOR,
        GPIO_IRQ_EDGE_FALL,
        true,
        &scor_gpio_callback
    );
}

/**
 * @brief  Blocking delay: waits delay_byte × 500 µs.
 *
 * Reproduces the original 8051 delay() which polled a counter decremented
 * at ~2 kHz (every 500 µs).  Only used during IC power-on reset sequencing
 * (reset_dsic2_cd6()) where a precise millisecond-range delay is required
 * before the ICs accept their first commands.
 */
void delay(void)
{
    do {
        sleep_us(500);
    } while (--delay_byte);
}

/**
 * @brief  Blocking delay of n × 500 µs (parametric form of delay()).
 *
 * Convenience wrapper used when the caller cannot write to delay_byte first.
 *
 * @param  n  Number of 500 µs steps to wait.
 */
void delay_us_500x(uint8_t n)
{
    sleep_us((uint32_t)n * 500u);
}
