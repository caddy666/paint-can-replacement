/**
 * @file  main.c
 * @brief CD32 Pico2 firmware — entry point and cooperative main loop.
 *
 * ── System overview ───────────────────────────────────────────────────────
 *
 * This firmware replaces the original 8051 MCU in the Commodore CD32 CD
 * drive (Philips/Commodore 1992–1993).  It runs on a Raspberry Pi Pico 2
 * (RP2350) and implements the same command/status protocol seen by the
 * Amiga chipset (Akiko / Paula), while using RP2350-native peripherals for
 * all hardware interfaces.
 *
 * ── Hardware interfaces ───────────────────────────────────────────────────
 *
 *   CXD2500BQ (CD6)  — Sony CD signal processor: EFM decoder, CLV servo
 *                      loop, audio DAC.  Write-only 3-wire bit-bang SPI.
 *                      Pins: UCL/UDAT/ULAT (GPIO 2–4).
 *
 *   DSIC2            — Sony CXA1372 servo IC: focus, spindle, radial, and
 *                      sledge actuator control.  Bidirectional 3-wire serial.
 *                      Pins: SICL/SIDA/SILD (GPIO 5–7).
 *
 *   Q-channel        — Subcode data clocked out of CXD2500 via QDA/QCL
 *                      (GPIO 8–9); synchronised by SCOR falling-edge interrupt
 *                      (GPIO 12).
 *
 *   COMMO bus        — Proprietary 3-wire serial between this firmware and
 *                      the Amiga chipset: DATA (bidir), CLK (host-driven),
 *                      DIR (our direction control).  Pins: GPIO 13–15.
 *
 * ── Cooperative multitasking ──────────────────────────────────────────────
 *
 * There is no RTOS.  All modules are non-blocking state machines that
 * advance by one step per main-loop iteration.  The loop is:
 *
 *   1. command_handler()  — write a pending COMMO command into player_interface
 *                           if the player is idle.  Check for command completion
 *                           and queue a STATUS_UPDATE response.
 *
 *   2. COMMO_INTERFACE()  — advance the serial RX/TX state machine by one
 *                           step (receive one byte or transmit one byte).
 *
 *   3. Dispatcher()       — route completed RX frames to the command handler;
 *                           queue STATUS/Q/ID packets for transmission.
 *
 *   4. player()           — advance the active disc-control process by one
 *                           step, then run servo(), subcode_module(), and
 *                           shock_recover() as background tasks.
 *
 * The order is significant:
 *   - command_handler before player: a new command written into
 *     player_interface this tick will be picked up by player() in the same
 *     iteration, minimising latency.
 *   - COMMO_INTERFACE before Dispatcher: the Dispatcher sees the freshest
 *     receive result from the COMMO layer on every tick.
 *
 * ── Initialisation sequence ───────────────────────────────────────────────
 *
 *   player_init()           — calls timer_init(), driver_init(),
 *                              reset_dsic2_cd6(), servo_init(), cd6_init()
 *   COMMO_INIT()            — configures COMMO GPIO pins and clears state
 *   Init_command_handler()  — resets pending command slot
 *   enable_scor_counter()   — enables SCOR falling-edge interrupt
 *                              (replaces 8051 "EA=1; EX0=1")
 *
 * Hardware: Raspberry Pi Pico 2 (RP2350), running at default 125 MHz.
 */

#include "pico/stdlib.h"
#include <stdio.h>

#include "defs.h"
#include "player.h"
#include "commo.h"
#include "cmd_hndl.h"
#include "driver.h"
#include "timer.h"
#include "gpio_map.h"
#include "hardware/gpio.h"

/* Forward declaration for Dispatcher() defined in core/dispatcher.c.
 * Not in a header because the Dispatcher is only called from main(). */
extern void Dispatcher(void);

/* =========================================================================
 * main
 * ====================================================================== */

int main(void)
{
    /* Pico SDK board init: configures clocks and enables USB/UART stdio.
     * stdio_init_all() is a no-op if neither USB nor UART stdio is enabled
     * in CMakeLists.txt, but harmless to call in either case. */
    stdio_init_all();

    /* ── Subsystem initialisation ─────────────────────────────────────── */

    /* player_init() performs the complete hardware bring-up sequence:
     *   timer_init()        — start 8 ms repeating timer
     *   driver_init()       — configure all GPIO pins
     *   reset_dsic2_cd6()   — assert reset, wait, release (with delays)
     *   servo_init()        — set servo state machine to INIT_DSIC2
     *   cd6_init()          — send CXD2500 startup register sequence
     * After player_init(), player_interface is in the idle/ready state. */
    player_init();

    /* Initialise the COMMO serial interface (GPIO + state machine). */
    COMMO_INIT();

    /* Initialise the command handler (clear pending slot). */
    Init_command_handler();

    /* Enable the SCOR falling-edge interrupt.
     * On the 8051 this was "EA=1; EX0=1" (global enable + external int 0). */
    enable_scor_counter();

    /* Signal boot complete on the status LED (onboard GPIO 25). */
    gpio_put(PIN_LED_STATUS, 1);

    /* ── Main loop — cooperative multitasking, no RTOS ───────────────── */
    for (;;) {
        command_handler();    /* 1. dispatch pending command to player   */
        COMMO_INTERFACE();    /* 2. service serial RX/TX state machine   */
        Dispatcher();         /* 3. route packets and status updates     */
        player();             /* 4. advance disc control + background tasks */
    }

    return 0;   /* unreachable */
}
