/**
 * @file  servo.c
 * @brief Spindle / focus / radial / sledge servo state machine.
 *
 * This module drives the mechanical and electro-optical sub-systems of the
 * CD drive through a single state-machine function — servo() — that is
 * called once per main-loop iteration.  The state machine is indexed by
 * servo_state (a value from serv_def.h) into servo_functions_array[].
 *
 * ── Physical sub-systems controlled ────────────────────────────────────
 *
 *  Laser       — switched on/off via DSIC2 PRESET+LASER_ON/OFF commands.
 *                Must be on before focus can be attempted.
 *
 *  Focus servo — the DSIC2 focus loop moves the objective lens
 *                perpendicularly to the disc surface to maintain focus.
 *                Lock is detected by polling the FE (focus-error) bit in
 *                the DSIC2 status byte.
 *
 *  Spindle     — the CXD2500BQ CLV servo keeps the disc surface velocity
 *                constant.  Startup proceeds in two stages:
 *                  Mode 1 (kick): 0xE8 — high-current pulse to overcome
 *                                        static friction.
 *                  Mode 2 (rough): 0xEE — open-loop ramp to approximately
 *                                         correct speed.
 *                  Play mode: 0xE6 — closed-loop CLV tracking.
 *
 *  Radial servo — the DSIC2 tracking loop keeps the laser on the current
 *                groove.  Must be initialised (rad_start / rad_hold) once
 *                per disc insertion.  Lock is detected via the TE
 *                (tracking-error) bit in the DSIC2 status byte.
 *
 *  Sledge      — a coarse mechanical stage that moves the laser assembly
 *                radially.  Used for large jumps (> MAX grooves) and to
 *                recover from sledge-end crashes.  Driven by the DSIC2
 *                SRSLEDGE command.
 *
 * ── Jump strategy ────────────────────────────────────────────────────
 *
 *  |grooves| < MAX (150):
 *    Short jump — SRCOMM3 packet, actuator only, no sledge.
 *
 *  MAX ≤ |grooves| ≤ BRAKE_2_DIS_MAX (3000):
 *    Medium jump — SRCOMM5 with proportional brake distance, no kick.
 *
 *  |grooves| > BRAKE_2_DIS_MAX:
 *    Long jump — SRCOMM5 with maximum brake, plus a timed sledge kick
 *    (outward) or brake (inward) to assist the coarse movement.
 *
 * ── Error-recovery hierarchy ──────────────────────────────────────────
 *
 *  RADIAL_RECOVER → re-initialises the radial servo; may involve a short
 *                   sledge move outward before re-locking.
 *  FOCUS_RECOVER  → turns off radial, cuts motor, retracts sledge inward,
 *                   then retries focus acquisition.
 *  Both decrement servo_retries; at zero the module sets servo_exec_state
 *  to CD_ERROR_STATE so player.c can trigger the error-handling sequence.
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 * All 8051-specific constructs removed; `bit` → `uint8_t`; `idat` removed.
 */

#include <stdlib.h>    /* abs() */
#include <stdint.h>

#include "defs.h"
#include "serv_def.h"
#include "dsic2.h"
#include "driver.h"
#include "timer.h"

/* =========================================================================
 * External references — provided by driver.c / timer.c / subcode.c
 * ====================================================================== */

extern volatile uint8_t timers[];

/* Convenient aliases for the two timer entries used in this module */
#define servo_timer      timers[TIMER_SERVO]       /* general state-machine timeout    */
#define kick_brake_timer timers[TIMER_KICK_BRAKE]  /* timed kick/brake phase for jumps */

extern uint8_t  player_error;    /* written here on error; read by player.c     */
extern uint8_t  hex_abs_min;     /* absolute disc minute position; used for zone */
extern uint8_t  simulation_timer;/* motor-speed simulation down-counter          */
extern volatile uint8_t scor_edge;

/* Provided by subcode.c: start_subcode_reading() arms the Q-channel decoder;
 * i_can_read_subcode is set to 1 when a valid frame has been decoded. */
extern void    start_subcode_reading(void);
extern uint8_t i_can_read_subcode;

/* =========================================================================
 * Servo module state
 * ====================================================================== */

static uint8_t servo_exec_state;       /**< Overall result: BUSY / READY / CD_ERROR_STATE */
static uint8_t servo_state;            /**< Current position in the state machine          */
static uint8_t servo_requested_state;  /**< Next target state set by the public API        */
static uint8_t servo_retries;          /**< Remaining focus/radial retry count             */

static uint16_t off_track_value;       /**< Last sampled DSIC2 off-track magnitude         */

int_hl_t grooves;                      /**< Requested jump distance in grooves (signed)    */

static uint8_t n2_speed_req;           /**< 1 = caller has requested 2× speed             */
static uint8_t rad_recover_in_jump;    /**< Set if radial recovery occurred during a jump  */
static uint8_t foc_recover_in_jump;    /**< Set if focus recovery occurred during a jump   */
static uint8_t no_efm_in_jump;         /**< Set if HF was absent when we arrived at target */

/* These are exported to service.c and strtstop.c so they can read disc state */
uint8_t initialized;      /**< 1 after the first successful radial init for this disc */
uint8_t motor_started;    /**< 1 once the spindle kick command has been issued        */
uint8_t motor_on_speed;   /**< 1 once MOT_STRT_1 (≥75% speed) has been confirmed     */
uint8_t reload_servo_timer;/**< Used in stop sequences that need two timer rounds     */
uint8_t disc_size;         /**< DISC_8CM or DISC_12CM — affects motor gain            */
uint8_t disc_size_known;   /**< 1 once disc diameter has been determined              */

static uint8_t kick;   /**< 1 = we are in the kick phase of a long outward jump */
static uint8_t brk;    /**< 1 = we are in the brake phase of a long inward jump  */

/* n1_speed is defined in driver.c and shared here so the servo module can
 * apply correct gain and brake calculations for the current speed setting. */
extern int n1_speed;

/* =========================================================================
 * DSIC2 status readers
 *
 * Each helper reads one raw status byte from the DSIC2 and extracts the
 * relevant bit.  We read a fresh byte each time to get the latest value.
 * ====================================================================== */

/**
 * @brief  Return 1 if the DSIC2 focus loop is locked (FE bit clear).
 *
 * FE = focus-error bit (bit 0).  The DSIC2 asserts it (1) when the
 * objective lens is out of focus, and clears it (0) when locked.
 * We invert so the function returns 1 for "in focus".
 */
static uint8_t dsic_in_focus(void)
{
    uint8_t v = rd_dsic2();
    return (v & 0x01u) ? 1u : 0u;   /* bit 0 = FE: 0=in-focus, 1=out-of-focus */
}

/**
 * @brief  Return 1 if the DSIC2 radial loop is locked (TE bit clear).
 *
 * TE = tracking-error bit (bit 1).  The DSIC2 asserts it (1) when the
 * laser is between grooves, and clears it (0) when locked on a groove.
 */
static uint8_t dsic_on_track(void)
{
    uint8_t v = rd_dsic2();
    return (v & 0x02u) ? 0u : 1u;   /* bit 1 = TE: 0=on-track, 1=off-track */
}

/**
 * @brief  Read the 16-bit signed off-track counter from two DSIC2 reads.
 *
 * The DSIC2 maintains an internal counter of grooves traversed since the
 * last jump command.  We read the high byte first, then the low byte, to
 * construct a signed 16-bit value.  This is used in check_jump_state() to
 * decide whether we are converging on the target.
 */
static int16_t read_off_track_value(void)
{
    uint8_t  hi = rd_dsic2();
    uint8_t  lo = rd_dsic2();
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

/**
 * @brief  Return CLOSED (0) if the sledge home switch is engaged.
 *
 * The sledge home switch is wired to bit 2 of the DSIC2 status register.
 * CLOSED means the sledge is at the innermost (lead-in) position.
 */
static uint8_t sledge_switch(void)
{
    uint8_t v = rd_dsic2();
    return (v & 0x04u) ? OPEN : CLOSED;
}

/* =========================================================================
 * Sledge and radial helpers
 * ====================================================================== */

/** Drive the sledge inward (toward disc centre / lead-in) at full power. */
static void sledge_in(void)
{
    wr_dsic2(SRSLEDGE);
    wr_dsic2(SLEDGE_UOUT_IN);
}

/** Drive the sledge outward (toward disc edge / lead-out) at full power. */
static void sledge_out(void)
{
    wr_dsic2(SRSLEDGE);
    wr_dsic2(SLEDGE_UOUT_OUT);
}

/** Stop the sledge motor. */
void sledge_off(void)
{
    wr_dsic2(SRSLEDGE);
    wr_dsic2(SLEDGE_UOUT_OFF);
}

/**
 * @brief  Start radial gain/offset initialisation.
 *
 * rad_start() puts the DSIC2 into its automatic calibration mode where it
 * sweeps the radial actuator to determine the correct gain and DC offset.
 * The caller must wait RAD_INITIALIZE_TIME ticks before calling rad_hold().
 */
void rad_start(void)
{
    wr_dsic2(0x21);   /* DSIC2 radial-init start command */
}

/**
 * @brief  End radial calibration and engage the closed-loop tracking servo.
 *
 * After the initialisation sweep has completed, rad_hold() locks in the
 * measured gain/offset and switches the DSIC2 to closed-loop tracking.
 */
void rad_hold(void)
{
    wr_dsic2(0x22);   /* DSIC2 radial-hold (engage tracking) command */
}

/** Write an array of bytes to the DSIC2, one per wr_dsic2() call. */
static void wr_dsic2_array(const uint8_t *arr, uint8_t len)
{
    while (len--) wr_dsic2(*arr++);
}

/* =========================================================================
 * Motor gain setting
 *
 * The appropriate motor gain depends on disc size (8 cm has less mass) and
 * speed (higher speed → higher angular momentum → needs more gain).
 * ====================================================================== */

static void set_motor_gain(void)
{
    if (disc_size == DISC_8CM)
        cd6_wr(n1_speed ? MOT_GAIN_8CM_N1 : MOT_GAIN_8CM_N2);
    else
        cd6_wr(n1_speed ? MOT_GAIN_12CM_N1 : MOT_GAIN_12CM_N2);
}

/* =========================================================================
 * Public servo helpers (called from service.c for diagnostic commands)
 * ====================================================================== */

/** Enable laser and return READY. */
uint8_t switch_laser_on(void)
{
    wr_dsic2(PRESET);
    wr_dsic2(LASER_ON);
    return READY;
}

/** Disable laser (static — not exposed to service.c, used internally). */
static void switch_laser_off(void)
{
    wr_dsic2(PRESET);
    wr_dsic2(LASER_OFF);
}

/**
 * @brief  Disable the focus servo loop while leaving the laser on.
 *
 * Sends the two-byte DSIC2 focus-off sequence and returns READY.
 * The laser remains powered so the DSIC2 can re-acquire focus without
 * the full laser power-on sequence.
 */
uint8_t switch_focus_off(void)
{
    static const uint8_t focus_off_array[] = { 0x11, 0x00 };
    wr_dsic2_array(focus_off_array, sizeof(focus_off_array));
    return READY;
}

/** Disable radial servo and park the sledge; return READY. */
uint8_t turn_radial_off(void)
{
    rad_hold();     /* freeze the actuator before stopping the sledge */
    sledge_off();
    return READY;
}

/** Disable focus servo and laser together; return READY. */
uint8_t turn_focus_laser_off(void)
{
    switch_focus_off();
    switch_laser_off();
    return READY;
}

/** Enable laser and focus servo together; return READY. */
uint8_t turn_laser_focus_on(void)
{
    switch_laser_on();
    static const uint8_t focus_on_array[] = { 0x10, 0x01 };
    wr_dsic2_array(focus_on_array, sizeof(focus_on_array));
    return READY;
}

/* forward declaration needed because service.c calls jump_servo_state */
void jump_servo_state(void);

/* =========================================================================
 * Active-brake guard
 *
 * Active braking is only safe when a disc of known size is present and the
 * door is closed — we must not try to brake against an empty or open drive.
 * ====================================================================== */

static uint8_t active_brake_ok(void)
{
    if (!disc_size) return 0;      /* disc size not yet known */
    if (!door_closed()) return 0;  /* door is open — no disc */
    return 1;
}

/* =========================================================================
 * Jump calculation helpers
 *
 * For long jumps the radial actuator alone cannot bridge the distance; we
 * must also move the sledge.  We time the sledge assistance with a kick
 * (outward direction) or brake (inward) pulse calculated from off_track_value
 * and the current disc zone (get_area()).
 * ====================================================================== */

/**
 * @brief  Calculate kick-phase duration for a long outward jump.
 *
 * A larger off-track distance means a longer jump, which needs a longer kick
 * to get the sledge moving.  The zone correction factors in disc geometry:
 * the groove pitch is wider near the inner edge so a given groove-count jump
 * covers less physical distance there.
 */
static uint8_t calc_kick(void)
{
    int time;
    uint8_t offset;

    if      (get_area() == 1) offset = 6;
    else if (get_area() == 2) offset = (off_track_value > 4000) ? 6 : 1;
    else    offset = n1_speed ? 0 : ((off_track_value > 10000) ? 9 : 0);

    /* Proportional term: off_track_value / 8 × 3 / 128 ≈ off_track × 0.003 */
    time = (int)(off_track_value >> 3);
    time = time * 3;
    time = time >> 7;
    time += n1_speed ? offset : (int)(offset << 2);
    return (uint8_t)time;
}

/**
 * @brief  Calculate brake-phase duration for a long inward jump.
 *
 * Inward jumps need active braking to stop the sledge at the target;
 * the required duration is zone-dependent and longer at double speed.
 */
static uint8_t calc_brake(void)
{
    int time;
    if (n1_speed) {
        if      (get_area() == 1) time = 6;
        else if (get_area() == 2) time = 1;
        else                      time = 0;
    } else {
        if      (get_area() == 1) time = 16;
        else if (get_area() == 2) time = 3;
        else                      time = 0;
    }
    return (uint8_t)time;
}

/**
 * @brief  Execute a short jump (actuator only, no sledge movement).
 *
 * Sends the three-word SRCOMM3 packet: command header, groove_high,
 * groove_low, then RAD_STAT3 to set the DSIC2 status read pointer.
 */
static void jump_short(void)
{
    wr_dsic2(SRCOMM3);
    wr_dsic2(grooves.b.high);
    wr_dsic2(grooves.b.low);
    wr_dsic2(RAD_STAT3);
}

/**
 * @brief  Execute a long jump with sledge assistance.
 *
 * Sends the six-word SRCOMM5 packet: command header, brake distance,
 * sledge power (SLEDGE_UOUT_JMP), groove_high, groove_low, RAD_STAT5.
 *
 * @param brake  Signed brake distance: negative means the DSIC2 will
 *               decelerate the actuator by that many grooves before the
 *               expected target.
 */
static void jump_long(int8_t brake)
{
    wr_dsic2(SRCOMM5);
    wr_dsic2((uint8_t)brake);
    wr_dsic2(SLEDGE_UOUT_JMP);
    wr_dsic2(grooves.b.high);
    wr_dsic2(grooves.b.low);
    wr_dsic2(RAD_STAT5);
}

/* =========================================================================
 * State functions — one per state ID in serv_def.h
 *
 * Each function executes for exactly one main-loop iteration, then returns.
 * State transitions are performed by writing to servo_state.
 * servo_exec_state carries the module-level result (BUSY/READY/ERROR).
 * ====================================================================== */

/**
 * INIT_DSIC2 — software-reset the DSIC2 servo IC.
 *
 * Sends PRESET (0x00) followed by 0x00 to put the IC in a known state.
 * Advances immediately to INIT_SLEDGE.
 */
static void init_dsic2_state(void)
{
    wr_dsic2(PRESET);
    wr_dsic2(0x00);
    servo_state = INIT_SLEDGE;
}

/**
 * INIT_SLEDGE — home the sledge to the innermost position.
 *
 * If the sledge home switch is already closed (sledge is at home), we
 * drive the sledge outward briefly to clear the switch, then recheck.
 * Otherwise we drive inward until the switch closes.
 *
 * Two-reload scheme: HALF_SLEDGE_IN_TIME is loaded twice, giving the
 * sledge up to 2 × ~500 ms = 1 s to reach home before declaring an error.
 */
static void init_sledge_state(void)
{
    if (sledge_switch() == CLOSED) {
        /* Already at home — drive out to create room for the inward stroke */
        sledge_out();
        servo_timer = SLEDGE_OUT_TIME;
        servo_state = CHECK_OUT_SLEDGE;
    } else {
        /* Drive inward; reload_servo_timer gives us a second chance */
        sledge_in();
        servo_timer        = HALF_SLEDGE_IN_TIME;
        reload_servo_timer = 1;
        servo_state        = CHECK_IN_SLEDGE;
    }
}

/**
 * CHECK_IN_SLEDGE — wait for the sledge to reach the home switch.
 *
 * On switch closure we immediately drive the sledge out to the start of
 * the program area (CHECK_OUT_SLEDGE).  On timeout with one reload
 * remaining we try once more.  On second timeout we set SLEDGE_ERROR.
 */
static void check_in_sledge_state(void)
{
    if (servo_timer == 0) {
        if (reload_servo_timer) {
            /* First timeout — give the sledge a second half-period */
            servo_timer        = HALF_SLEDGE_IN_TIME;
            reload_servo_timer = 0;
        } else {
            /* Second timeout — sledge is stuck; report error */
            sledge_off();
            servo_state             = SERVO_IDLE;
            servo_requested_state   = SERVO_IDLE;
            player_error            = SLEDGE_ERROR;
            servo_exec_state        = CD_ERROR_STATE;
        }
    } else {
        if (sledge_switch() == CLOSED) {
            /* Home switch closed — reverse direction to position for play */
            sledge_off();
            sledge_out();
            servo_timer = SLEDGE_OUT_TIME;
            servo_state = CHECK_OUT_SLEDGE;
        }
    }
}

/**
 * CHECK_OUT_SLEDGE — wait for the sledge to clear the home switch.
 *
 * We need the sledge slightly outward of the home switch to give the
 * radial actuator room to initialise.  Timeout means the sledge is
 * blocked outward.
 */
static void check_out_sledge_state(void)
{
    if (servo_timer == 0) {
        sledge_off();
        servo_state           = SERVO_IDLE;
        servo_requested_state = SERVO_IDLE;
        player_error          = SLEDGE_ERROR;
        servo_exec_state      = CD_ERROR_STATE;
    } else {
        if (sledge_switch() == OPEN) {
            /* Sledge has cleared the switch — park here */
            sledge_off();
            servo_state = SERVO_IDLE;
        }
    }
}

/**
 * SERVO_IDLE — wait for an external request.
 *
 * servo_requested_state is written by the public API functions.  The only
 * transitions out of SERVO_IDLE are to START_FOCUS (normal spin-up) or
 * JUMP_SERVO (which is rejected here — we must be monitoring before jumping).
 */
static void servo_idle_state(void)
{
    if (servo_requested_state == START_FOCUS) {
        servo_state           = START_FOCUS;
        servo_requested_state = SERVO_MONITOR;  /* default next target */
    } else if (servo_requested_state == JUMP_SERVO) {
        /* Jump requested while not yet tracking — cannot comply */
        servo_requested_state = SERVO_IDLE;
        servo_exec_state      = CD_ERROR_STATE;
        player_error          = ILLEGAL_COMMAND;
    } else if (servo_exec_state != CD_ERROR_STATE) {
        servo_exec_state = READY;
    }
}

/**
 * START_FOCUS — enable the laser and focus servo, arm timeout.
 */
static void start_focus_state(void)
{
    turn_laser_focus_on();
    servo_timer   = FOCUS_TIME_OUT;   /* ~400 ms to acquire focus */
    servo_state   = CHECK_FOCUS;
    servo_retries = MAX_RETRIES;
}

/**
 * CHECK_FOCUS — poll for focus lock.
 *
 * We sample the DSIC2 FE bit every main-loop iteration (~1–2 ms).  On
 * lock we add a double-check delay to confirm the lock is stable.
 * On timeout we enter STOP_SERVO with FOCUS_ERROR.
 */
static void check_focus_state(void)
{
    if (servo_timer == 0) {
        turn_focus_laser_off();
        player_error          = FOCUS_ERROR;
        servo_exec_state      = CD_ERROR_STATE;
        servo_state           = STOP_SERVO;
        servo_requested_state = SERVO_IDLE;
    } else if (dsic_in_focus()) {
        /* Focus locked — wait a short time to confirm it is stable */
        servo_timer = TIME_DOUBLE_FOCUS_CHECK;
        servo_state = DOUBLE_CHECK_FOCUS;
    }
}

/**
 * DOUBLE_CHECK_FOCUS — wait 24 ms and re-confirm focus.
 *
 * A brief lock can be caused by the lens oscillating through focus.
 * After the double-check delay we verify focus is still held.
 */
static void double_check_focus_state(void)
{
    if (servo_timer == 0) {
        servo_state = dsic_in_focus() ? START_TTM : FOCUS_RECOVER;
    }
}

/**
 * START_TTM — issue the spindle kick command (Mode 1).
 *
 * Mode 1 applies a high-current pulse to overcome the spindle's static
 * friction and start it rotating.  We then wait SPEEDUP_TIME ticks for
 * the motor to accelerate before switching to Mode 2.
 */
static void start_ttm_state(void)
{
    cd6_wr(MOT_STRTM1_ACTIVE);
    servo_timer   = SPEEDUP_TIME;     /* ~200 ms Mode-1 kick phase */
    servo_state   = TTM_SPEED_UP;
    motor_started = 1;
}

/**
 * TTM_SPEED_UP — wait for the motor to accelerate past the kick phase.
 *
 * If focus is lost during spin-up (disc wobble can briefly defocus the
 * lens) we go straight to FOCUS_RECOVER rather than waiting for timeout.
 */
static void ttm_speedup_state(void)
{
    if (dsic_in_focus()) {
        if (servo_timer == 0) {
            /* Kick phase complete — switch to rough servo (Mode 2) */
            cd6_wr(MOT_STRTM2_ACTIVE);
            servo_state = CHECK_TTM;
        }
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * CHECK_TTM — wait for the spindle to reach ≥75% nominal speed.
 *
 * status_cd6(MOT_STRT_1) returns 1 once simulation_timer (loaded by
 * cd6_wr(MOT_STRTM2_ACTIVE) to ~75 ticks) reaches zero.
 *
 * Once at speed we verify that HF (high-frequency) is present — if not,
 * there is no disc or the disc is unreadable.
 */
static void check_ttm_state(void)
{
    if (dsic_in_focus()) {
        if (status_cd6(MOT_STRT_1)) {
            /* Motor at speed — configure gain for disc size and speed */
            set_motor_gain();
            motor_on_speed = 1;
            if (!hf_present()) {
                /* No HF signal: disc surface is not reflective */
                servo_state           = STOP_SERVO;
                servo_requested_state = SERVO_IDLE;
                servo_exec_state      = CD_ERROR_STATE;
                player_error          = HF_DETECTOR_ERROR;
            } else {
                servo_state = START_RADIAL;
            }
        }
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * START_RADIAL — begin radial servo initialisation.
 *
 * The first time this is called for a disc we run the full DSIC2
 * calibration sequence (rad_start + wait RAD_INITIALIZE_TIME).
 * If radial was already initialised (initialized==1, e.g. after a jump
 * recovery) we skip the calibration and proceed immediately.
 */
static void start_radial_state(void)
{
    servo_timer = 0;
    if (dsic_in_focus()) {
        if (!initialized) {
            rad_start();
            servo_timer = RAD_INITIALIZE_TIME;   /* ~300 ms calibration */
        }
        servo_state = INIT_RADIAL;
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * INIT_RADIAL — wait for calibration and perform the initial groove jump.
 *
 * After calibration we jump –10 grooves (inward) to shake loose any
 * initial static friction in the actuator, then proceed to SERVO_MONITOR
 * via the JUMP_SERVO state.
 */
static void init_radial_state(void)
{
    if (dsic_in_focus()) {
        if (servo_timer == 0) {
            rad_hold();
            initialized = 1;
            grooves.val = -10;   /* initial normalisation jump */
            servo_state = JUMP_SERVO;
        }
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * UPTO_N2 — transition from 1× to 2× speed.
 *
 * We wait until the motor has been in rough-servo mode long enough
 * (servo_timer counts up from the Mode-2 transition) then engage play
 * mode and arm the subcode reader to confirm the new speed.
 */
static void upto_n2_state(void)
{
    if (servo_timer < UPTO_N2_TIME) {
        cd6_wr(MOT_PLAYM_ACTIVE);
        start_subcode_reading();
        servo_state = WAIT_SUBCODE;
        servo_timer = SUBCODE_TIME_OUT;
    }
}

/**
 * DOWNTO_N1 — transition from 2× back to 1× speed.
 *
 * At 2× speed we apply an active brake (MOT_BRM2) to decelerate the disc,
 * then switch the PLL speed register to N1 and wait for the motor to
 * re-lock at 1× before engaging play mode.
 */
static void downto_n1_state(void)
{
    if (!n1_speed) {
        /* We are still at 2× — wait for MOT_STRT_2 or timeout */
        if (!status_cd6(MOT_STRT_2) || servo_timer == 0) {
            cd6_wr(SPEED_CONTROL_N1);
            cd6_wr(MOT_STRTM2_ACTIVE);
            n1_speed = 1;
            set_motor_gain();
        }
    } else {
        /* Now at 1× — wait for motor to reach 1× nominal speed */
        if (status_cd6(MOT_STRT_1)) {
            cd6_wr(MOT_PLAYM_ACTIVE);
            start_subcode_reading();
            servo_state = WAIT_SUBCODE;
            servo_timer = SUBCODE_TIME_OUT;
        }
    }
}

/**
 * SERVO_MONITOR — normal tracking state.
 *
 * This is the steady state during playback.  We check for:
 *   - Jump requests from servo_jump()
 *   - Speed change requests (n2_speed_req vs n1_speed)
 *   - Loss of focus → FOCUS_RECOVER
 *   - Loss of tracking → RADIAL_RECOVER
 *
 * i_can_read_subcode is set by subcode_module() when a valid Q-channel
 * frame has been decoded.  We use it to reset the monitor timeout: if the
 * subcode decoder is running correctly, we know the laser is on-track.
 */
static void servo_monitor_state(void)
{
    if (dsic_in_focus()) {
        if (dsic_on_track() && (i_can_read_subcode || servo_timer != 0)) {

            if (i_can_read_subcode) {
                /* Fresh subcode frame confirms tracking — reset timeout */
                i_can_read_subcode = 0;
                servo_timer        = SUBCODE_MONITOR_TIMEOUT;
                servo_retries      = MAX_RETRIES;
            }

            if (servo_requested_state == JUMP_SERVO) {
                /* A jump has been requested — execute it now */
                servo_state           = JUMP_SERVO;
                servo_requested_state = SERVO_MONITOR;
            } else if (!n1_speed && !n2_speed_req) {
                /* Currently at 2× but 1× has been requested — brake down */
                servo_state = DOWNTO_N1;
                cd6_wr(MOT_BRM2_ACTIVE);
                servo_timer = N2_TO_N1_BRAKE_TIME;
            } else if (n1_speed && n2_speed_req) {
                /* Currently at 1× but 2× has been requested — ramp up */
                servo_state = UPTO_N2;
                cd6_wr(SPEED_CONTROL_N2);
                cd6_wr(MOT_STRTM2_ACTIVE);
                n1_speed = 0;
                set_motor_gain();
            } else {
                /* Steady tracking — signal READY to the caller */
                servo_requested_state = SERVO_MONITOR;
                servo_exec_state      = READY;
            }
        } else {
            /* Tracking lost (TE asserted or subcode timeout) */
            servo_state = RADIAL_RECOVER;
        }
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * RADIAL_RECOVER — recover from loss of radial tracking.
 *
 * We attempt to re-acquire tracking by briefly stopping the radial actuator
 * then re-initialising.  If the sledge is at the inner stop we move it
 * outward first.  After MAX_RETRIES failures we declare RADIAL_ERROR or
 * SUBCODE_TIMEOUT_ERROR (if focus and tracking look fine but no subcode
 * arrived, which indicates a defective disc area).
 */
static void radial_recover_state(void)
{
    servo_retries--;
    if (servo_retries == 0) {
        /* Exhausted retries — determine the root cause */
        if (!hf_present())
            player_error = HF_DETECTOR_ERROR;
        else if (dsic_in_focus() && dsic_on_track())
            player_error = SUBCODE_TIMEOUT_ERROR;   /* mechanically OK but no data */
        else
            player_error = RADIAL_ERROR;

        servo_exec_state      = CD_ERROR_STATE;
        servo_state           = STOP_SERVO;
        servo_requested_state = SERVO_IDLE;
    } else {
        turn_radial_off();
        if (sledge_switch() == CLOSED) {
            /* At inner stop — move outward before retrying */
            sledge_out();
            servo_timer = R_REC_OUT_SLEDGE;
            servo_state = SLEDGE_OUTSIDE_RECOVER;
        } else {
            /* Already positioned — re-initialise radial from here */
            servo_state = INIT_RADIAL;
            servo_timer = 0;
        }
    }
}

/**
 * FOCUS_RECOVER — recover from loss of focus.
 *
 * Focus loss typically means the disc has been disturbed (shock) or the
 * lens has drifted off the reflective layer.  We stop the motor, retract
 * the sledge inward to a known position, then re-attempt the full focus
 * acquisition sequence.
 */
static void focus_recover_state(void)
{
    turn_radial_off();
    cd6_wr(MOT_OFF_ACTIVE);   /* stop motor immediately */
    servo_retries--;
    if (servo_retries == 0) {
        servo_exec_state      = CD_ERROR_STATE;
        servo_state           = STOP_SERVO;
        servo_requested_state = SERVO_IDLE;
        /* no_efm_in_jump: if HF was absent when focus was lost during a jump,
         * report HF_DETECTOR_ERROR (likely a gap in the disc surface). */
        player_error = no_efm_in_jump ? HF_DETECTOR_ERROR : FOCUS_ERROR;
    } else {
        servo_timer = F_REC_IN_SLEDGE;
        servo_state = SLEDGE_INSIDE_RECOVER;
        sledge_in();   /* retract to known inner position */
    }
}

/**
 * SLEDGE_INSIDE_RECOVER — wait for the sledge to reach the inner position.
 *
 * Transition to CHECK_FOCUS once the sledge is home or the timer expires.
 */
static void sledge_inside_recover_state(void)
{
    if (servo_timer == 0 || sledge_switch() == CLOSED) {
        sledge_off();
        servo_timer = FOCUS_TIME_OUT;
        servo_state = CHECK_FOCUS;
    }
}

/**
 * SLEDGE_OUTSIDE_RECOVER — wait for focus after moving the sledge outward.
 *
 * During RADIAL_RECOVER we may have moved the sledge outward.  We wait
 * here for focus to be confirmed before proceeding to INIT_RADIAL.
 */
static void sledge_outside_recover_state(void)
{
    if (dsic_in_focus()) {
        if (servo_timer == 0) {
            sledge_off();
            servo_state = INIT_RADIAL;
            servo_timer = 0;
        }
    } else {
        servo_state = FOCUS_RECOVER;
    }
}

/**
 * STOP_SERVO — begin the disc-stopping sequence.
 *
 * Two paths:
 *
 *  a) Motor at speed, disc in focus, on track, and braking is safe:
 *     Apply active brake (MOT_BRM2) and wait STOP_TIME_OUT for the
 *     disc to decelerate.  This gives a controlled stop without stress
 *     on the sledge.
 *
 *  b) Motor not on speed, or other conditions not met for active brake:
 *     Cut motor immediately, turn off radial and focus, and wait for
 *     the disc to coast to rest (MOT_OFF_STOP_TIME).
 *
 *  c) Motor was never started:
 *     Just turn off laser and wait.
 */
static void stop_servo_state(void)
{
    servo_state = CHECK_STOP_SERVO;

    if (motor_started) {
        if (motor_on_speed && dsic_on_track() && dsic_in_focus() && active_brake_ok()) {
            /* Controlled active brake */
            cd6_wr(MOT_BRM2_ACTIVE);
            servo_timer = STOP_TIME_OUT;
        } else {
            /* Uncontrolled stop: cut everything and wait */
            turn_radial_off();
            turn_focus_laser_off();
            cd6_wr(MOT_OFF_ACTIVE);
            servo_timer        = MOT_OFF_STOP_TIME;
            reload_servo_timer = 1;
            initialized        = 0;
            motor_on_speed     = (uint8_t)!n1_speed;   /* clear if at 1×, preserve if 2× */
            if (!active_brake_ok()) {
                servo_timer   = 0;
                motor_started = 0;
            }
        }
    } else {
        /* Motor was never started — just cut the laser */
        turn_focus_laser_off();
        servo_timer        = 0;
        reload_servo_timer = 0;
    }
}

/**
 * CHECK_STOP_SERVO — wait for the disc to come to rest.
 *
 * Three scenarios:
 *
 *  a) A new spin-up was requested while braking: interrupt the stop and
 *     go straight to START_FOCUS (e.g. disc ejection command while playing).
 *
 *  b) Motor was at speed (active brake applied): wait for MOT_STOP or
 *     timeout, then cut power, re-home sledge, and settle.
 *
 *  c) Motor was not at speed (coasting stop): count down two timer
 *     periods (reload_servo_timer) to allow the disc to decelerate fully,
 *     then re-home the sledge.
 */
static void check_stop_servo_state(void)
{
    if (servo_requested_state == START_FOCUS) {
        /* New spin-up request arrived — abort stop immediately */
        servo_state           = START_FOCUS;
        servo_requested_state = SERVO_MONITOR;
        turn_radial_off();
        cd6_wr(MOT_OFF_ACTIVE);
        motor_on_speed = 0;

    } else if (motor_started) {
        if (initialized) {
            /* Active-brake path: wait for MOT_STOP status flag */
            if (status_cd6(MOT_STOP) || servo_timer == 0) {
                turn_radial_off();
                turn_focus_laser_off();
                cd6_wr(MOT_OFF_ACTIVE);
                /* Remaining brake time scaled × 2 becomes the coast-to-stop wait */
                uint8_t elapsed = (uint8_t)STOP_TIME_OUT - servo_timer;
                servo_timer        = (elapsed & 0x80u) ? 255u : (uint8_t)(elapsed << 1);
                reload_servo_timer = 1;
                motor_started      = 0;
            }
        } else {
            /* Coasting stop: two-stage wait via reload_servo_timer */
            if (servo_timer == 0) {
                servo_timer = MOT_OFF_STOP_TIME;
                if (reload_servo_timer)      reload_servo_timer = 0;
                else if (motor_on_speed)     motor_on_speed     = 0;
                else                         motor_started      = 0;
            }
        }

    } else {
        /* Motor fully stopped */
        if (servo_timer == 0) {
            if (reload_servo_timer) {
                /* One more settle period */
                servo_timer        = EXTRA_STOP_DELAY;
                reload_servo_timer = 0;
            } else {
                /* Re-home sledge and reset servo to idle */
                servo_state = INIT_SLEDGE;
                cd6_wr(SPEED_CONTROL_N1);   /* restore default speed setting */
                n1_speed       = 1;
                set_motor_gain();
                initialized    = 0;
                motor_on_speed = 0;
                if (servo_requested_state == STOP_SERVO)
                    servo_exec_state = READY;
            }
        }
    }
}

/**
 * JUMP_SERVO — calculate and execute a groove jump.
 *
 * The jump strategy is selected by |grooves|:
 *
 *   Short  (< MAX):        SRCOMM3, actuator only.
 *   Medium (< BRAKE_2):    SRCOMM5 with proportional brake, no kick.
 *   Long   (≥ BRAKE_2):    SRCOMM5 + sledge kick (outward) or brake (inward).
 *
 * For long jumps we also set cd6_wr(MOT_STRTM1_ACTIVE) for outward kicks or
 * cd6_wr(MOT_BRM2_ACTIVE) for inward braking to help the motor recover the
 * CLV servo after the coarse sledge movement.
 *
 * After the jump command is sent we set servo_state = CHECK_JUMP and arm
 * a SKATING_DELAY_CHECK timer before sampling the off-track value.
 */
void jump_servo_state(void)
{
    int8_t brake_dist;

    kick = 0;
    brk  = 0;

    off_track_value = (uint16_t)abs(grooves.val);

    if (off_track_value < (uint16_t)MAX) {
        jump_short();
    } else if (off_track_value <= (uint16_t)BRAKE_2_DIS_MAX) {
        /* Proportional brake: -(off_track+16)/32 – 1 */
        brake_dist = (int8_t)(-(int8_t)(((off_track_value + 16u) >> 5u))) - 1;
        cd6_wr(MOT_OFF_ACTIVE);   /* motor off for medium jump */
        jump_long(brake_dist);
    } else {
        /* Maximum brake distance for long jump */
        brake_dist = (int8_t)((int)BRAKE_DIS_MAX / -16);
        if (grooves.val > 0) {
            /* Outward jump: kick the sledge to assist */
            kick_brake_timer = calc_kick();
            cd6_wr(MOT_STRTM1_ACTIVE);
            kick = 1;
        } else {
            /* Inward jump: brake the sledge to assist */
            brk              = 1;
            kick_brake_timer = calc_brake();
            cd6_wr(MOT_BRM2_ACTIVE);
        }
        jump_long(brake_dist);
    }

    servo_state = CHECK_JUMP;
    servo_timer = SKATING_DELAY_CHECK;   /* brief wait before first off-track sample */
}

/**
 * CHECK_JUMP — monitor convergence on the target groove.
 *
 * While converging:
 *   - If kick or brake phase has expired, cut the motor assist.
 *   - If dsic_on_track(): engage play mode, arm subcode reader.
 *   - If timer expired: sample the off-track counter.
 *     - If counter is decreasing: we are converging; update reference.
 *     - If counter stopped decreasing or went backwards: RADIAL_RECOVER.
 *
 * If focus is lost during the jump we set no_efm_in_jump = 1 (HF absence
 * means the disc surface was not readable at the target position).
 */
static void check_jump_state(void)
{
    if (dsic_in_focus()) {
        if (hf_present()) {
            /* Stop kick/brake once timed phase expires */
            if ((kick || brk) && kick_brake_timer == 0) {
                cd6_wr(MOT_OFF_ACTIVE);
                kick = 0;
                brk  = 0;
            }

            if (dsic_on_track()) {
                /* Locked on target groove — engage play mode */
                cd6_wr(MOT_PLAYM_ACTIVE);
                start_subcode_reading();
                servo_state = WAIT_SUBCODE;
                servo_timer = SUBCODE_TIME_OUT;
            } else if (servo_timer == 0) {
                /* Sample the current off-track distance */
                uint16_t new_ot = (uint16_t)abs(read_off_track_value());
                if (new_ot <= off_track_value) {
                    if ((int)(off_track_value - new_ot) < 10) {
                        /* Distance is barely decreasing — declare radial error */
                        servo_state         = RADIAL_RECOVER;
                        rad_recover_in_jump = 1;
                    } else {
                        /* Still converging — update reference */
                        off_track_value = new_ot;
                    }
                } else if (new_ot > 200u) {
                    /* Distance increased significantly — off course */
                    servo_state         = RADIAL_RECOVER;
                    rad_recover_in_jump = 1;
                }
                servo_timer = SKATING_SAMPLE_TIME;   /* next sample in 24 ms */
            }
        } else {
            /* No HF at this location — wrong area or scratched disc */
            no_efm_in_jump = 1;
            servo_state    = FOCUS_RECOVER;
            kick = 0; brk = 0;
        }
    } else {
        foc_recover_in_jump = 1;
        servo_state         = FOCUS_RECOVER;
        kick = 0; brk = 0;
    }
}

/**
 * WAIT_SUBCODE — wait for the first valid Q-channel frame after a jump.
 *
 * After engaging play mode we wait for subcode_module() to confirm that
 * the CXD2500 is decoding valid EFM data at the new position.  If the
 * SCOR flag does not arrive within SUBCODE_TIME_OUT ticks the jump has
 * missed and we try RADIAL_RECOVER.
 */
static void wait_subcode_state(void)
{
    if (!status_cd6(SUBCODE_READY)) {
        /* Subcode has arrived — we are confirmed on track */
        servo_state = SERVO_MONITOR;
        servo_timer = SUBCODE_MONITOR_TIMEOUT;
    } else if (servo_timer == 0) {
        /* No subcode within timeout — not on a valid groove */
        servo_state         = RADIAL_RECOVER;
        rad_recover_in_jump = 1;
    }
}

/* =========================================================================
 * State dispatch table
 *
 * servo_functions_array[servo_state]() is called once per main-loop tick.
 * The indices must match the state IDs in serv_def.h exactly.
 * ====================================================================== */

typedef void (*servo_fn_t)(void);

static const servo_fn_t servo_functions_array[] = {
    init_dsic2_state,            /* INIT_DSIC2             = 0  */
    init_sledge_state,           /* INIT_SLEDGE            = 1  */
    check_in_sledge_state,       /* CHECK_IN_SLEDGE        = 2  */
    check_out_sledge_state,      /* CHECK_OUT_SLEDGE       = 3  */
    servo_idle_state,            /* SERVO_IDLE             = 4  */
    start_focus_state,           /* START_FOCUS            = 5  */
    check_focus_state,           /* CHECK_FOCUS            = 6  */
    double_check_focus_state,    /* DOUBLE_CHECK_FOCUS     = 7  */
    start_ttm_state,             /* START_TTM              = 8  */
    ttm_speedup_state,           /* TTM_SPEED_UP           = 9  */
    check_ttm_state,             /* CHECK_TTM              = 10 */
    start_radial_state,          /* START_RADIAL           = 11 */
    init_radial_state,           /* INIT_RADIAL            = 12 */
    servo_monitor_state,         /* SERVO_MONITOR          = 13 */
    radial_recover_state,        /* RADIAL_RECOVER         = 14 */
    focus_recover_state,         /* FOCUS_RECOVER          = 15 */
    sledge_inside_recover_state, /* SLEDGE_INSIDE_RECOVER  = 16 */
    sledge_outside_recover_state,/* SLEDGE_OUTSIDE_RECOVER = 17 */
    stop_servo_state,            /* STOP_SERVO             = 18 */
    check_stop_servo_state,      /* CHECK_STOP_SERVO       = 19 */
    jump_servo_state,            /* JUMP_SERVO             = 20 */
    check_jump_state,            /* CHECK_JUMP             = 21 */
    wait_subcode_state,          /* WAIT_SUBCODE           = 22 */
    upto_n2_state,               /* UPTO_N2                = 23 */
    downto_n1_state,             /* DOWNTO_N1              = 24 */
};

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise servo module state for the first time.
 *
 * Called from player_init().  Sets the physical starting point as the inner
 * sledge position and resets all flags.  The actual DSIC2 reset and sledge
 * homing happen in the state machine (INIT_DSIC2 → INIT_SLEDGE).
 */
void servo_init(void)
{
    servo_requested_state = SERVO_IDLE;
    servo_state           = INIT_DSIC2;
    servo_exec_state      = BUSY;
    n1_speed              = 1;
    disc_size             = DISC_12CM;          /* assume 12 cm until detected */
    n2_speed_req          = (N == 2) ? 1u : 0u; /* honour compile-time N setting */
}

/**
 * @brief  Request a full disc spin-up sequence.
 *
 * Called by strtstop.c when an SS_START_UP command is received.
 * Clears all jump-condition flags, sets the target state to SERVO_MONITOR,
 * and signals BUSY to the caller via servo_exec_state.
 */
void servo_start(void)
{
    servo_requested_state = START_FOCUS;
    servo_exec_state      = BUSY;
    rad_recover_in_jump   = 0;
    foc_recover_in_jump   = 0;
    no_efm_in_jump        = 0;
}

/**
 * @brief  Re-home the sledge without a full power-cycle.
 *
 * Called from service.c (ENTER_NORMAL_MODE_OPC) to return the sledge to a
 * known position after service operations.
 */
void servo_reinit_sledge(void)
{
    servo_requested_state = SERVO_IDLE;
    servo_state           = INIT_SLEDGE;
    servo_exec_state      = BUSY;
}

/** Request 1× speed (used by strtstop.c for SS_SPEED_N1). */
void servo_n1(void) { n2_speed_req = 0; servo_exec_state = BUSY; }

/** Request 2× speed (used by strtstop.c for SS_SPEED_N2). */
void servo_n2(void) { n2_speed_req = 1; servo_exec_state = BUSY; }

/**
 * @brief  Request a controlled disc stop.
 *
 * Transitions to STOP_SERVO only if not already stopping.  Safe to call
 * multiple times.
 */
void servo_stop(void)
{
    if (servo_requested_state != STOP_SERVO &&
        servo_requested_state != SERVO_IDLE)
    {
        servo_requested_state = STOP_SERVO;
        servo_state           = STOP_SERVO;
    }
    servo_exec_state = BUSY;
}

/**
 * @brief  Request a groove jump.
 *
 * @param jump_size  Number of grooves to jump.  Positive = outward (toward
 *                   lead-out); negative = inward (toward lead-in).
 *
 * The jump direction is inverted internally (grooves.val = -jump_size)
 * because the DSIC2 command sign convention is opposite to the logical
 * "disc position" sign convention used by the higher layers.
 */
void servo_jump(int jump_size)
{
    grooves.val           = -jump_size;
    servo_retries         = MAX_RETRIES;
    servo_requested_state = JUMP_SERVO;
    servo_exec_state      = BUSY;
    rad_recover_in_jump   = 0;
    foc_recover_in_jump   = 0;
    no_efm_in_jump        = 0;
}

/** @return Current servo execution state: BUSY / READY / CD_ERROR_STATE. */
uint8_t get_servo_process_state(void) { return servo_exec_state; }

/**
 * @brief  Return flags describing what happened during the last jump.
 *
 * Bits are set if the corresponding condition occurred:
 *   NO_HF_ON_TARGET      — HF absent at target (wrong position or bad disc)
 *   RADIAL_RECOVERY_DONE — radial recovery was needed
 *   FOCUS_RECOVERY_DONE  — focus recovery was needed
 */
uint8_t get_jump_status(void)
{
    uint8_t s = 0;
    if (no_efm_in_jump)      s |= NO_HF_ON_TARGET;
    if (rad_recover_in_jump) s |= RADIAL_RECOVERY_DONE;
    if (foc_recover_in_jump) s |= FOCUS_RECOVERY_DONE;
    return s;
}

/**
 * @brief  Return 1 if the servo is in a state where jumping is valid.
 *
 * Used by play.c to verify that a jump command can be accepted.  Jumping
 * is only valid while the servo is tracking (SERVO_MONITOR) or already
 * executing a jump (JUMP_SERVO).
 */
uint8_t servo_tracking(void)
{
    return (servo_requested_state == SERVO_MONITOR ||
            servo_requested_state == JUMP_SERVO) ? 1u : 0u;
}

/**
 * @brief  Return 1 if the servo is in a state compatible with service mode.
 *
 * Service mode requires the servo to be either idle (servo not spinning)
 * or tracking (laser on disc).  During transitions (spin-up, stop, recovery)
 * it is not safe to switch to service mode.
 */
uint8_t servo_to_service(void)
{
    return (servo_state == SERVO_IDLE ||
            servo_state == SERVO_MONITOR) ? 1u : 0u;
}

/**
 * @brief  Main servo tick — called once per main-loop iteration.
 *
 * Dispatches to the current state function via the function pointer table.
 * The state function may change servo_state before returning; the new state
 * takes effect on the next call.
 */
void servo(void)
{
    servo_functions_array[servo_state]();
}
