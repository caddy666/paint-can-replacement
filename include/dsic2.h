/**
 * @file  dsic2.h
 * @brief DSIC2 (Sony CXA1372) servo IC — register constants and timing.
 *
 * The DSIC2 is a combined focus / radial / sledge servo controller that
 * sits between the RP2350 and the drive's analogue actuators.  It receives
 * serial commands over the three-wire SICL/SIDA/SILD bus and returns an
 * 8-bit status byte on the bidirectional SIDA line.
 *
 * Bus protocol (implemented in driver.c):
 *   All writes are 8 bits, MSB first, clocked on the rising edge of SICL.
 *   After the last bit SILD is pulsed low briefly to latch the value.
 *   For a read the master releases SIDA to GPIO_IN before clocking.
 *
 * Status byte layout (read via rd_dsic2()):
 *   Bit 0 — FE  : focus error  (0 = in focus, 1 = out of focus)
 *   Bit 1 — TE  : track error  (0 = on track,  1 = off track)
 *   Bit 2 — SW  : sledge home switch (0 = at home / innermost position)
 *   Bits 3–7 — additional diagnostic flags (not used in this firmware)
 *
 * Multi-byte commands:
 *   A command to the DSIC2 typically consists of a command-select byte
 *   followed by one or more data bytes, all sent with individual wr_dsic2()
 *   calls before the final latch.  See the SRCOMM3 / SRCOMM5 / SRSLEDGE
 *   sequences below.
 */

#pragma once
#include <stdint.h>

/* -------------------------------------------------------------------------
 * DSIC2 single-byte commands
 * ---------------------------------------------------------------------- */
#define PRESET      0x00  /**< Software reset — returns all servo loops to safe state  */
#define LASER_ON    0x01  /**< Enable the laser-diode drive current                    */
#define LASER_OFF   0x00  /**< Disable the laser-diode drive current                  */

/* -------------------------------------------------------------------------
 * Multi-byte jump command headers
 *
 * Short jump (3-word packet):  SRCOMM3, groove_high, groove_low, RAD_STAT3
 *   Used for jumps within ±MAX grooves (actuator only, no sledge movement).
 *
 * Long jump  (6-word packet):  SRCOMM5, brake, speed, groove_high, groove_low, RAD_STAT5
 *   Used for larger jumps; the 'brake' byte specifies deceleration distance
 *   and 'speed' controls the sledge assist power.
 *
 * Sledge command (2-word packet):  SRSLEDGE, direction_power
 *   Drives the sledge motor directly with an 8-bit signed power/direction byte.
 * ---------------------------------------------------------------------- */
#define SRCOMM3     0x40  /**< Select 3-word short jump command register  */
#define SRCOMM5     0x50  /**< Select 6-word long  jump command register  */
#define SRSLEDGE    0x60  /**< Select 2-word sledge drive register        */

/* -------------------------------------------------------------------------
 * Radial status selectors (written as the final word of a jump command)
 *
 * Tells the DSIC2 which internal status register to expose on the next read.
 * ---------------------------------------------------------------------- */
#define RAD_STAT3   0x02  /**< Request radial status after a short jump   */
#define RAD_STAT5   0x03  /**< Request radial status after a long jump    */

/* -------------------------------------------------------------------------
 * Sledge direction / power constants (second byte of an SRSLEDGE pair)
 *
 * Bit 7 selects direction (1 = inward toward lead-in, 0 = outward).
 * The lower 7 bits encode motor power (0 = coast/stop, 0x7F = full outward).
 * ---------------------------------------------------------------------- */
#define SLEDGE_UOUT_IN   0x80  /**< Move sledge toward lead-in  (full inward power)   */
#define SLEDGE_UOUT_OUT  0x7F  /**< Move sledge toward lead-out (full outward power)   */
#define SLEDGE_UOUT_OFF  0x00  /**< Stop sledge motor (coast)                         */
#define SLEDGE_UOUT_JMP  0x40  /**< Reduced sledge power used during groove jumps     */

/* -------------------------------------------------------------------------
 * Radial servo helper commands (written via wr_dsic2() directly)
 * ---------------------------------------------------------------------- */
#define RAD_INITIALIZE_TIME   38  /**< Radial init hold time: 38 × 8 ms = 304 ms      */

/* -------------------------------------------------------------------------
 * Timing constants (8 ms software-timer ticks unless noted)
 *
 * All timeouts are written directly to entries in the timers[] array; the
 * ISR in timer.c decrements each non-zero entry every 8 ms.
 * ---------------------------------------------------------------------- */
#define FOCUS_TIME_OUT          50  /**< Max time to acquire focus: 50 × 8 = 400 ms    */
#define TIME_DOUBLE_FOCUS_CHECK  3  /**< Double-check delay:         3 × 8 =  24 ms    */
#define SPEEDUP_TIME            25  /**< Mode-1 kick phase:         25 × 8 = 200 ms    */
#define NOMINAL_SPEED_TIME     100  /**< Wait for ≥75% speed:      100 × 8 = 800 ms    */
#define MAX_RETRIES              3  /**< Number of focus/radial retries before error    */
#define HALF_SLEDGE_IN_TIME     63  /**< Half the max sledge-home time: 63 × 8 ≈ 500 ms*/
#define SLEDGE_OUT_TIME         13  /**< Sledge-out pulse duration: 13 × 8 ≈ 104 ms    */
#define SUBCODE_TIME_OUT        25  /**< Wait for first subcode frame: 25 × 8 = 200 ms */
#define SUBCODE_MONITOR_TIMEOUT 25  /**< Max gap between subcode frames: 200 ms        */
#define SKATING_DELAY_CHECK      3  /**< Delay before sampling off-track value: 24 ms  */
#define SKATING_SAMPLE_TIME      3  /**< Sampling interval during jump: 24 ms          */
#define STOP_TIME_OUT          125  /**< Max time for active brake to stop disc: 1 s   */
#define MOT_OFF_STOP_TIME       63  /**< Extra wait after motor-off command: 500 ms    */
#define EXTRA_STOP_DELAY        13  /**< Delay before sledge re-homes after stop: 104 ms*/
#define N2_TO_N1_BRAKE_TIME     13  /**< Active-brake duration for 2× → 1× transition  */
#define F_REC_IN_SLEDGE         13  /**< Max sledge-in time during focus recovery: 104 ms*/
#define R_REC_OUT_SLEDGE         5  /**< Sledge-out time during radial recovery: 40 ms  */
#define UPTO_N2_TIME            10  /**< Settling time when checking for 2× speed: 80 ms*/
#define SUBCODE_TIMEOUT_VALUE   50  /**< Subcode wait used in strtstop / play: 400 ms   */

/* -------------------------------------------------------------------------
 * Jump distance constants (in grooves / tracks)
 *
 * A standard CD has approximately 22,188 grooves (tracks) on a 12 cm disc.
 * One groove ≈ 1.6 µm pitch, making the total disc radius about 35 mm.
 *
 * TRACKS_INTO_LEADIN is negative (jump inward) and sized to jump all the
 * way from the program area back into the lead-in, which contains the TOC.
 * TRACKS_OUTOF_LEADIN jumps outward far enough to clear the lead-in and
 * land in the program area.
 * ---------------------------------------------------------------------- */
#define TRACKS_INTO_LEADIN    (-588)  /**< Jump back to lead-in (≈ 0.94 mm inward)   */
#define TRACKS_OUTOF_LEADIN    100    /**< Jump out of lead-in into program area      */

/* -------------------------------------------------------------------------
 * Jump distance thresholds
 *
 * The jump algorithm in servo.c selects between three strategies:
 *   |grooves| < MAX           → short jump (radial actuator only)
 *   MAX ≤ |grooves| ≤ BRAKE_2 → medium jump (sledge assist, short brake)
 *   |grooves| > BRAKE_2       → long  jump (sledge assist, full kick/brake)
 * ---------------------------------------------------------------------- */
#define MAX              150   /**< Maximum short-jump distance (actuator reach)   */
#define BRAKE_2_DIS_MAX 3000   /**< Transition from medium to long jump strategy   */
#define BRAKE_DIS_MAX   3000   /**< Long-jump brake distance limit                 */
