/**
 * @file  serv_def.h
 * @brief Servo state-machine states and CXD2500BQ mode constants.
 *
 * Ported from the original Philips/Commodore firmware (1992-1993).
 *
 * The servo state machine in servo.c drives the physical disc-mechanics in
 * a fixed sequence:
 *
 *   Power-on idle
 *     └─ INIT_DSIC2       Reset the DSIC2 servo IC
 *         └─ INIT_SLEDGE  Home the optical sledge to the innermost position
 *             └─ SERVO_IDLE  Wait for a start request
 *                 └─ START_FOCUS       Enable laser and focus servo
 *                     └─ CHECK_FOCUS   Wait for focus lock
 *                         └─ DOUBLE_CHECK_FOCUS  Confirm stable focus
 *                             └─ START_TTM    Kick spindle (Mode 1 start)
 *                                 └─ TTM_SPEED_UP  Ramp to nominal speed
 *                                     └─ CHECK_TTM   Wait for MOT_STRT_1
 *                                         └─ START_RADIAL  Init radial servo
 *                                             └─ INIT_RADIAL  Wait + small jump
 *                                                 └─ SERVO_MONITOR  Normal play
 *
 * During SERVO_MONITOR the laser tracks the groove and the state machine
 * handles JUMP_SERVO requests and speed changes (UPTO_N2 / DOWNTO_N1).
 * Error paths lead to FOCUS_RECOVER, RADIAL_RECOVER, or SLEDGE_*_RECOVER
 * states, each of which retries before returning CD_ERROR_STATE.
 */

#pragma once

/* -------------------------------------------------------------------------
 * Servo state-machine state IDs
 *
 * These are indices into servo_functions_array[] in servo.c.  The numeric
 * values must match the array positions exactly.
 * ---------------------------------------------------------------------- */
#define INIT_DSIC2               0  /**< Reset DSIC2 with PRESET command               */
#define INIT_SLEDGE              1  /**< Drive sledge toward inner stop                */
#define CHECK_IN_SLEDGE          2  /**< Wait for sledge home switch to close          */
#define CHECK_OUT_SLEDGE         3  /**< Wait for sledge to clear the home switch      */
#define SERVO_IDLE               4  /**< Sledge homed; awaiting START_FOCUS request    */
#define START_FOCUS              5  /**< Enable laser + focus servo; arm timeout       */
#define CHECK_FOCUS              6  /**< Poll DSIC2 focus-error bit until in-focus     */
#define DOUBLE_CHECK_FOCUS       7  /**< Wait 24 ms and confirm focus has not drifted  */
#define START_TTM                8  /**< Send MOT_STRTM1 (kick) to CXD2500            */
#define TTM_SPEED_UP             9  /**< Wait for motor to accelerate past kick phase  */
#define CHECK_TTM               10  /**< Wait for MOT_STRT_1 (≥75% nominal speed)      */
#define START_RADIAL            11  /**< Optionally initialise radial gain             */
#define INIT_RADIAL             12  /**< Wait for RAD_INITIALIZE_TIME; jump –10 grooves*/
#define SERVO_MONITOR           13  /**< Normal tracking; handle jump/speed requests   */
#define RADIAL_RECOVER          14  /**< Retry radial lock after loss of tracking      */
#define FOCUS_RECOVER           15  /**< Retract sledge, re-acquire focus              */
#define SLEDGE_INSIDE_RECOVER   16  /**< Wait for sledge to reach inner position       */
#define SLEDGE_OUTSIDE_RECOVER  17  /**< Wait for focus after moving sledge outward    */
#define STOP_SERVO              18  /**< Begin brake / motor-off sequence              */
#define CHECK_STOP_SERVO        19  /**< Wait for disc to decelerate to full stop      */
#define JUMP_SERVO              20  /**< Calculate and execute groove jump             */
#define CHECK_JUMP              21  /**< Monitor HF and tracking after a jump          */
#define WAIT_SUBCODE            22  /**< Wait for first valid Q-channel after a jump   */
#define UPTO_N2                 23  /**< Transition from 1× to 2× speed               */
#define DOWNTO_N1               24  /**< Transition from 2× back to 1× speed          */

/* -------------------------------------------------------------------------
 * CXD2500BQ status-type selectors (used with status_cd6() in driver.c)
 *
 * These are pseudo-addresses that driver.c's status_cd6() maps to the
 * simulated motor status flags.  On the real hardware they were read from
 * dedicated CXD2500 status pins; we model them in software.
 * ---------------------------------------------------------------------- */
#define SUBCODE_READY   0x20  /**< Subcode clock edge received (SCOR fired)    */
#define MOT_STRT_1      0x21  /**< Motor has reached ≥75% nominal speed        */
#define MOT_STRT_2      0x22  /**< Motor has reached nominal speed (2nd check) */
#define MOT_STOP        0x23  /**< Motor has decelerated to a standstill        */
#define PLL_LOCK        0x24  /**< CXD2500 PLL has locked to disc clock        */
#define MOTOR_OVERFLOW  0x27  /**< Motor speed overflow (disc spinning too fast)*/

/* -------------------------------------------------------------------------
 * CXD2500BQ motor-mode constants (written via cd6_wr() in driver.c)
 *
 * These are in the range 0x18–0x1F so they pass through the default case
 * in cd6_wr() only if not intercepted; driver.c maps them to actual
 * CXD2500 register values in the switch statement.
 * ---------------------------------------------------------------------- */
#define MOT_OFF_ACTIVE    0x18  /**< Motor off (free-run / stopped)                   */
#define MOT_BRM1_ACTIVE   0x19  /**< Brake mode 1 — gentle brake                     */
#define MOT_BRM2_ACTIVE   0x1A  /**< Brake mode 2 — active brake (timed)             */
#define MOT_STRTM1_ACTIVE 0x1B  /**< Start mode 1 — kick (short high-current pulse)  */
#define MOT_STRTM2_ACTIVE 0x1C  /**< Start mode 2 — rough servo (ramp to speed)      */
#define MOT_JMPM_ACTIVE   0x1D  /**< Jump mode — rough servo during long groove jump  */
#define MOT_JMPM1_ACTIVE  0x1E  /**< Jump mode 1 variant                             */
#define MOT_PLAYM_ACTIVE  0x1F  /**< Play mode — closed-loop CLV servo               */

/* -------------------------------------------------------------------------
 * Disc size constants (used for motor gain and brake-table selection)
 * ---------------------------------------------------------------------- */
#define DISC_8CM   0  /**< Single-density 8 cm mini-disc (lower motor gain)  */
#define DISC_12CM  1  /**< Standard 12 cm disc (default; higher motor gain)  */

/* -------------------------------------------------------------------------
 * Jump status bit flags (returned by get_jump_status() in servo.c)
 *
 * Accumulated during a jump sequence and checked by play.c to decide
 * whether the seek target was actually reached.
 * ---------------------------------------------------------------------- */
#define NO_HF_ON_TARGET         0x01  /**< HF absent at target (wrong area or no disc) */
#define RADIAL_RECOVERY_DONE    0x02  /**< Radial recovery was needed during jump       */
#define FOCUS_RECOVERY_DONE     0x04  /**< Focus recovery was needed during jump        */
#define MOTOR_NOT_ON_SPEED      0x08  /**< Motor speed not maintained during jump       */

/* -------------------------------------------------------------------------
 * Default speed setting
 * ---------------------------------------------------------------------- */
#define N  1   /**< N=1 → single speed; N=2 → double speed (compile-time default) */

/* -------------------------------------------------------------------------
 * CXD2500BQ register values
 *
 * These are the raw 8-bit words written to the CXD2500BQ via cxd2500_wr().
 * They encode the PLL speed divider for a 33.8688 MHz crystal.
 * ---------------------------------------------------------------------- */
#define SPEED_CONTROL_N1    0xB3  /**< CLV servo: 1× (1.2–1.4 m/s linear velocity)  */
#define SPEED_CONTROL_N2    0xBB  /**< CLV servo: 2× (2.4–2.8 m/s linear velocity)  */
#define MOT_GAIN_8CM_N1     0x41  /**< Motor gain G=4.0  (8 cm disc at 1×)           */
#define MOT_GAIN_12CM_N1    0x44  /**< Motor gain G=12.8 (12 cm disc at 1×)          */
#define MOT_GAIN_8CM_N2     0x43  /**< Motor gain G=8.0  (8 cm disc at 2×)           */
#define MOT_GAIN_12CM_N2    0x46  /**< Motor gain G=12.8 (12 cm disc at 2×)          */

/* -------------------------------------------------------------------------
 * DAC / audio output mode constants (high-level identifiers for cd6_wr())
 *
 * Values in 0x30–0x35 are intercepted by the switch in driver.c:cd6_wr()
 * and translated to audio_cxd2500() calls or shadow-register updates.
 * ---------------------------------------------------------------------- */
#define NORMAL_MODE   0x00  /**< Audio level-meter mode: normal                    */
#define LEVEL_MODE    0x04  /**< Audio level-meter mode: VU-style                  */
#define PEAK_MODE     0x08  /**< Audio level-meter mode: peak-hold                 */

#define DAC_OUTPUT_MODE  0x30  /**< Set CXD2500 output to audio DAC mode (0x89)    */
#define MOT_OUTPUT_MODE  0x31  /**< Set CXD2500 motor output mode     (0xD0)       */
#define EBU_OUTPUT_MODE  0x32  /**< EBU digital output — not connected on CD32     */
#define MUTE             0x33  /**< Mute audio output (set mute bit in audio_cntrl)*/
#define FULL_SCALE       0x34  /**< Unmute audio output                            */
#define ATTENUATE        0x35  /**< Attenuate audio output (−6 dB)                 */
