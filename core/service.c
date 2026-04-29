/**
 * @file  service.c
 * @brief Service-mode command executor.
 *
 * Service mode is entered via the ENTER_SERVICE_MODE_OPC opcode and
 * provides direct, layer-by-layer control of the drive mechanics.  It is
 * used during factory alignment, diagnostics, and manual calibration.
 *
 * ── Mechanical power-up hierarchy ────────────────────────────────────────
 *
 * The drive mechanics have a strict power dependency:
 *
 *   Laser  →  Focus  →  Motor (spindle)  →  Radial (tracking)
 *
 * You cannot activate radial servo without an in-focus beam and a spinning
 * disc, and you cannot focus without the laser on.  The service command
 * table enforces this by always including lower-level start-up steps before
 * the requested capability.
 *
 *   LASER_ON           : laser power on only (lowest level)
 *   FOCUS_ON           : laser on + wait for focus lock
 *   SPINDLE_MOTOR_ON   : laser + focus + motor spin-up
 *   RADIAL_ON          : laser + focus + motor + radial lock
 *
 * ── The 14 service commands ───────────────────────────────────────────────
 *
 *  Index  Opcode               Description
 *    0    ENTER_NORMAL_MODE    Teardown all, return to normal playback mode
 *    1    LASER_ON             Enable laser diode
 *    2    LASER_OFF            Disable radial, motor, focus, and laser
 *    3    FOCUS_ON             Laser on + wait for optical focus lock
 *    4    FOCUS_OFF            Disable radial, motor, and focus
 *    5    SPINDLE_MOTOR_ON     Laser + focus + spin-up to playback speed
 *    6    SPINDLE_MOTOR_OFF    Disable radial and motor
 *    7    RADIAL_ON            Full lock sequence (laser+focus+motor+radial)
 *    8    RADIAL_OFF           Disable radial servo only
 *    9    MOVE_SLEDGE          Move laser sled by param1 speed for param2 ticks
 *   10    JUMP_GROOVES         Perform a groove jump of param1:param2 grooves
 *   11    WRITE_CD6            Send raw byte param1 to the CXD2500BQ
 *   12    WRITE_DSIC2          Send raw byte param1 to the DSIC2
 *   13    READ_DSIC2           Read one byte from DSIC2 into param1
 *
 * ── Step dispatch ─────────────────────────────────────────────────────────
 *
 * Each service command is represented as a row of up to MAX_SVC_STEPS (5)
 * function pointers in service_functions[][].
 *
 * execute_service_functions() runs one step per main-loop tick.  A step
 * returning READY advances to the next step; PROCESS_READY ends the
 * command; BUSY waits; CD_ERROR_STATE aborts with cleanup.
 *
 * The high nibble of service_process holds the command index (0–13) and
 * the low nibble the current step index (0–4), matching the packed
 * encoding used in play.c and strtstop.c.
 *
 * ── Phase encoding within step functions ──────────────────────────────────
 *
 * Multi-phase step functions use two flags (service_phase0, service_phase1)
 * as a 2-bit state variable.  These are reset to 0 between steps by
 * execute_service_functions() when a step returns READY.
 *
 *   phase1=0, phase0=0 → initial / phase 0
 *   phase1=0, phase0=1 → phase 1
 *   phase1=1, phase0=0 → phase 2
 *   phase1=1, phase0=1 → phase 3 (not used here, but available)
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 */

#include <stdint.h>

#include "defs.h"
#include "serv_def.h"
#include "dsic2.h"
#include "driver.h"
#include "timer.h"
#include <stddef.h>

/* =========================================================================
 * External references
 * ====================================================================== */

extern volatile uint8_t timers[];
#define servo_timer timers[TIMER_SERVO]

extern uint8_t player_error;
extern int_hl_t grooves;      /**< Groove-count target for servo jump (negative = outward) */
extern uint8_t  initialized;  /**< Non-zero if radial calibration has been done this session */
extern uint8_t  motor_started;
extern uint8_t  motor_on_speed;
extern uint8_t  reload_servo_timer;

/* From player.c — parameters for WRITE_CD6, WRITE_DSIC2, MOVE_SLEDGE, JUMP */
extern interface_field_t player_interface;

/* From servo.c */
extern uint8_t dsic_in_focus_pub(void);
extern uint8_t dsic_on_track_pub(void);
extern void    jump_servo_state(void);
extern void    servo_reinit_sledge(void);
extern uint8_t switch_laser_on(void);
extern uint8_t switch_focus_off(void);
extern uint8_t turn_radial_off(void);
extern uint8_t turn_focus_laser_off(void);
extern uint8_t turn_laser_focus_on(void);
extern void    rad_hold(void);
extern void    rad_start(void);
extern void    sledge_off(void);
extern int status_cd6(uint8_t);

/* ── Convenience shims ───────────────────────────────────────────────────
 * servo.c uses identical inline checks internally but marks them static.
 * We replicate the raw DSIC2 reads here rather than expose them from servo.c.
 *
 * DSIC2 status byte bit layout (active-low):
 *   bit 0 — FE  focus error   (0 = in focus)
 *   bit 1 — TE  tracking error (0 = on track)
 *   bit 2 — SW  sledge switch  (0 = at home / CLOSED)
 */
static uint8_t dsic_in_focus(void)  { return rd_dsic2() & 0x01u ? 0u : 1u; }
static uint8_t dsic_on_track(void)  { return rd_dsic2() & 0x02u ? 0u : 1u; }
static uint8_t sledge_switch(void)  { return rd_dsic2() & 0x04u ? OPEN : CLOSED; }
static uint8_t mute_on(void)        { cd6_wr(MUTE); return READY; }

/* =========================================================================
 * Constants
 * ====================================================================== */

/** Maximum time allowed for a single groove-jump attempt: 250 × 8 ms = 2 s.
 *  service_jump allows one timer reload, giving 4 s total before RADIAL_ERROR. */
#define HALF_SERVICE_JUMP_TIME   250u

/** Maximum time to wait for radial (tracking) lock after issuing SRCOMM3:
 *  ~62 × 8 ms = 496 ms, rounded up to ~500 ms. */
#define RAD_ON_TIMEOUT           (500u / 8u)

/* =========================================================================
 * Module state
 * ====================================================================== */

static uint8_t service_active;   /**< Non-zero while a command is being executed */
static uint8_t service_phase0;   /**< Phase bit 0 — reset between steps          */
static uint8_t service_phase1;   /**< Phase bit 1 — reset between steps          */

/**
 * Combined command + step register.
 *   High nibble — command index (0–13, matching the opcode offset)
 *   Low  nibble — step index within service_functions[cmd][step]
 */
static uint8_t service_process;

/** High nibble carries BUSY / READY / CD_ERROR_STATE / PROCESS_READY. */
static uint8_t service_status;

/** Non-zero while service() is executing; used by player.c to detect re-entry. */
uint8_t service_command_busy = 0;

/* =========================================================================
 * Step functions — each returns BUSY / READY / CD_ERROR_STATE / PROCESS_READY
 *
 * READY       → execute_service_functions() advances to the next step,
 *               clears phase0 and phase1.
 * PROCESS_READY → command sequence complete; returns PROCESS_READY to caller.
 * CD_ERROR_STATE → abort; execute_service_functions() runs cleanup.
 * BUSY        → no state change; will be called again next tick.
 * ====================================================================== */

/**
 * @brief  Write raw byte player_interface.param1 to the CXD2500BQ.
 *
 * Used by WRITE_CD6_OPC for direct register access during diagnostics.
 * Unlike cd6_wr() which maps symbolic constants, this sends the raw byte.
 */
static uint8_t write_cd6(void)
{
    cd6_wr(player_interface.param1);
    return READY;
}

/**
 * @brief  Write raw byte player_interface.param1 to the DSIC2.
 *
 * Used by WRITE_DSIC2_OPC for direct servo IC register access.
 */
static uint8_t write_dsic2(void)
{
    wr_dsic2(player_interface.param1);
    return READY;
}

/**
 * @brief  Read one byte from the DSIC2 into player_interface.param1.
 *
 * The caller receives the DSIC2 status byte (FE/TE/SW in bits 0–2).
 * Used by READ_DSIC2_OPC to allow the host to inspect servo IC state.
 */
static uint8_t read_dsic2_fn(void)
{
    player_interface.param1 = rd_dsic2();
    return READY;
}

/**
 * @brief  Enable the radial (tracking) servo with optional calibration.
 *
 * Three internal phases:
 *
 *   Phase 0 (phase1=0, phase0=0):
 *     Turn the radial servo off.  If this is the first use since motor start
 *     (initialized == 0), issue RAD_INITIALIZE_TIME ticks then call rad_start()
 *     to run the calibration sequence.  Otherwise skip straight to phase 1.
 *
 *   Phase 1 (phase1=0, phase0=1):
 *     Wait for the calibration timer.  Then call rad_hold() to enter hold mode,
 *     write the SRCOMM3 tracking command packet (0xFF, 0xF6) and RAD_STAT3 to
 *     close the tracking loop.  Load the RAD_ON_TIMEOUT countdown and move to
 *     phase 2.
 *
 *   Phase 2 (phase1=1, phase0=0):
 *     Wait for the DSIC2 TE bit to clear (dsic_on_track() == 1).  On success
 *     activate the CXD2500's motor playback mode and return READY.  On timeout
 *     set RADIAL_ERROR and return CD_ERROR_STATE.
 */
static uint8_t radial_on(void)
{
    if (!service_phase1 && !service_phase0) {
        turn_radial_off();
        if (!initialized) {
            servo_timer = RAD_INITIALIZE_TIME;
            rad_start();
        } else {
            servo_timer = 0;
        }
        service_phase0 = 1;
    }

    if (!service_phase1 && service_phase0) {
        if (servo_timer == 0) {
            rad_hold();
            initialized = 1;
            /* SRCOMM3 3-word packet: command = 0xFF 0xF6, status mode = RAD_STAT3 */
            wr_dsic2(SRCOMM3);
            wr_dsic2(0xFF);
            wr_dsic2(0xF6);
            wr_dsic2(RAD_STAT3);
            servo_timer    = RAD_ON_TIMEOUT;
            service_phase1 = 1;
            service_phase0 = 0;
        }
    }

    if (service_phase1 && !service_phase0) {
        if (servo_timer == 0) {
            player_error = RADIAL_ERROR;
            return CD_ERROR_STATE;
        }
        if (dsic_on_track()) {
            /* Tracking locked — activate playback motor mode in CXD2500 */
            cd6_wr(MOT_PLAYM_ACTIVE);
            return READY;
        }
    }

    return BUSY;
}

/**
 * @brief  Spin up the disc to playback speed.
 *
 * Four internal phases, matching the servo.c START_FOCUS → START_STRTM1 →
 * START_STRTM2 → WAIT_NOMINAL sequence:
 *
 *   Phase 0 (phase1=0, phase0=0):
 *     Load FOCUS_TIME_OUT and advance to phase 1.  (Laser/focus is already
 *     on from a prior step in the table.)
 *
 *   Phase 1 (phase1=0, phase0=1):
 *     Wait for the DSIC2 FE bit to clear (in-focus).  Then load SPEEDUP_TIME,
 *     issue MOT_STRTM1_ACTIVE to the CXD2500 (initial spin-up phase 1), set
 *     motor_started, and move to phase 2.
 *
 *   Phase 2 (phase1=1, phase0=0):
 *     Wait SPEEDUP_TIME.  Then issue MOT_STRTM2_ACTIVE (spin-up phase 2,
 *     higher target speed) and load NOMINAL_SPEED_TIME.  Move to phase 3.
 *
 *   Phase 3 (phase1=1, phase0=1):
 *     Poll status_cd6(MOT_STRT_1) until the CXD2500 reports nominal CLV
 *     speed.  On success set motor_on_speed = 1 and return READY.
 *     On timeout return MOTOR_ERROR.
 *
 * @note If motor_on_speed is already set this function returns READY
 *       immediately — idempotent so SPINDLE_MOTOR_ON can be called while
 *       already spinning.
 */
static uint8_t start_motor(void)
{
    if (motor_on_speed) return READY;

    if (!service_phase1 && !service_phase0) {
        servo_timer    = FOCUS_TIME_OUT;
        service_phase0 = 1;
    }

    if (!service_phase1 && service_phase0) {
        if (servo_timer == 0) { player_error = FOCUS_ERROR; return CD_ERROR_STATE; }
        if (dsic_in_focus()) {
            servo_timer    = SPEEDUP_TIME;
            cd6_wr(MOT_STRTM1_ACTIVE);
            motor_started  = 1;
            service_phase1 = 1;
            service_phase0 = 0;
        }
    }

    if (service_phase1 && !service_phase0) {
        if (servo_timer == 0) {
            servo_timer    = NOMINAL_SPEED_TIME;
            cd6_wr(MOT_STRTM2_ACTIVE);
            service_phase0 = 1;
        }
    }

    if (service_phase1 && service_phase0) {
        if (servo_timer == 0) { player_error = MOTOR_ERROR; return CD_ERROR_STATE; }
        if (status_cd6(MOT_STRT_1)) { motor_on_speed = 1; return READY; }
    }

    return BUSY;
}

/**
 * @brief  Stop the spindle motor and clear all motion state.
 *
 * Sends MOT_OFF_ACTIVE to the CXD2500 (kills CLV loop), and clears
 * motor_started, motor_on_speed, and initialized so the next start_motor()
 * call will run the full spin-up sequence from scratch.
 */
static uint8_t stop_motor(void)
{
    cd6_wr(MOT_OFF_ACTIVE);
    motor_started  = 0;
    motor_on_speed = 0;
    initialized    = 0;
    return READY;
}

/**
 * @brief  Exit service mode and return to normal servo tracking.
 *
 * Calls servo_reinit_sledge() which re-arms the servo state machine for
 * normal operation.  Returns PROCESS_READY (not READY) to signal the
 * command sequence is entirely complete and service mode should end.
 */
static uint8_t set_normal_mode(void)
{
    servo_reinit_sledge();
    return PROCESS_READY;
}

/**
 * @brief  Gate step: wait until the optical assembly is in focus.
 *
 * Used after turn_laser_focus_on() as a synchronisation barrier before
 * steps that require a focused beam (motor spin-up, track reading, etc.).
 * Returns READY the instant the DSIC2 FE bit clears; BUSY otherwise.
 */
static uint8_t monitor_focussing(void)
{
    return dsic_in_focus() ? READY : BUSY;
}

/**
 * @brief  Mark the service command sequence as complete from within a step.
 *
 * Terminal step in most command sequences.  Always returns PROCESS_READY
 * so execute_service_functions() sets service_status = PROCESS_READY and
 * clears service_active.
 */
static uint8_t service_process_ready(void)
{
    return PROCESS_READY;
}

/**
 * @brief  Move the laser sled under direct host control.
 *
 * Parameters (from player_interface):
 *   param1 — DSIC2 sledge drive byte:
 *               bits [6:0] = speed/direction value
 *               bit  7     = direction: 1 = move inward (toward centre)
 *   param2 — timeout in 8 ms ticks before the move is stopped
 *
 * Stops automatically when:
 *   - The timeout expires, OR
 *   - Moving inward (param1 bit 7 set) and the sledge hits the inner limit
 *     switch (sledge_switch() == CLOSED, meaning SW bit 2 in DSIC2 = 0).
 *
 * The SRSLEDGE packet is a 2-word DSIC2 command (opcode + 1 data byte).
 */
static uint8_t move_sledge(void)
{
    if (!service_phase0) {
        servo_timer    = player_interface.param2;
        wr_dsic2(SRSLEDGE);
        wr_dsic2(player_interface.param1);
        service_phase0 = 1;
    }

    if (servo_timer == 0 ||
        (sledge_switch() == CLOSED && (player_interface.param1 & 0x80u)))
    {
        sledge_off();
        return READY;
    }
    return BUSY;
}

/**
 * @brief  Perform a direct groove jump under host control.
 *
 * Parameters (from player_interface):
 *   param1 — high byte of signed groove count
 *   param2 — low  byte of signed groove count
 *
 * The groove count is reassembled into grooves.val and negated before
 * calling jump_servo_state().  The servo convention is that a negative
 * groove count means "jump outward" (increasing radius), so this negation
 * maps the host's natural "positive = outward" encoding to the servo's
 * internal convention.
 *
 * Phase 0:
 *   Verify focus, motor speed, and radial lock are all present.
 *   Load the jump target and start jump_servo_state().
 *   Load HALF_SERVICE_JUMP_TIME and set reload_servo_timer = 1 (allows one
 *   timer reload → up to 2 × HALF_SERVICE_JUMP_TIME = 4 s total).
 *
 * Phase 1:
 *   Poll dsic_on_track().  If true the jump completed — restore MOT_PLAYM_ACTIVE
 *   and return READY.  On final timeout report RADIAL_ERROR.
 */
static uint8_t service_jump(void)
{
    if (!service_phase0) {
        if (!dsic_in_focus()) { player_error = FOCUS_ERROR;  return CD_ERROR_STATE; }
        if (!status_cd6(MOT_STRT_1)) { player_error = MOTOR_ERROR; return CD_ERROR_STATE; }
        if (!dsic_on_track()) { player_error = RADIAL_ERROR; return CD_ERROR_STATE; }

        grooves.b.high = player_interface.param1;
        grooves.b.low  = player_interface.param2;
        grooves.val    = -grooves.val;   /* negate: servo convention is negative = outward */

        jump_servo_state();

        service_phase0     = 1;
        servo_timer        = HALF_SERVICE_JUMP_TIME;
        reload_servo_timer = 1;
        return BUSY;
    }

    if (dsic_on_track()) {
        cd6_wr(MOT_PLAYM_ACTIVE);
        return READY;
    }
    if (servo_timer == 0) {
        if (reload_servo_timer) {
            /* Allow one timeout extension: total timeout = 4 s */
            servo_timer        = HALF_SERVICE_JUMP_TIME;
            reload_servo_timer = 0;
            return BUSY;
        }
        player_error = RADIAL_ERROR;
        return CD_ERROR_STATE;
    }
    return BUSY;
}

/* =========================================================================
 * Service command dispatch table
 *
 * Rows are indexed by command ID (0–13), matching opcode offset.
 * Columns are step functions executed in sequence.
 * NULL terminates a short sequence; service_process_ready() signals done.
 *
 * The power-up hierarchy is enforced here:
 *   Radial commands always begin with turn_laser_focus_on() + start_motor()
 *   to guarantee the disc is spinning before the tracking loop is closed.
 *
 * Index alignment with opcodes in defs.h:
 *   0  = ENTER_NORMAL_MODE_OPC  offset
 *   1  = LASER_ON_OPC           offset
 *   …
 *  13  = READ_DSIC2_OPC         offset
 * ====================================================================== */

#define MAX_SVC_STEPS 5

typedef uint8_t (*svc_fn_t)(void);

static const svc_fn_t service_functions[14][MAX_SVC_STEPS] = {
/* 0  ENTER_NORMAL_MODE  */ { mute_on,            turn_radial_off, stop_motor, turn_focus_laser_off, set_normal_mode  },
/* 1  LASER_ON           */ { switch_laser_on,    service_process_ready, NULL, NULL, NULL                            },
/* 2  LASER_OFF          */ { turn_radial_off,    stop_motor, turn_focus_laser_off, service_process_ready, NULL       },
/* 3  FOCUS_ON           */ { turn_laser_focus_on,monitor_focussing, service_process_ready, NULL, NULL               },
/* 4  FOCUS_OFF          */ { turn_radial_off,    stop_motor, switch_focus_off, service_process_ready, NULL          },
/* 5  SPINDLE_MOTOR_ON   */ { turn_laser_focus_on,start_motor, service_process_ready, NULL, NULL                     },
/* 6  SPINDLE_MOTOR_OFF  */ { turn_radial_off,    stop_motor, service_process_ready, NULL, NULL                      },
/* 7  RADIAL_ON          */ { turn_laser_focus_on,start_motor, radial_on, service_process_ready, NULL                },
/* 8  RADIAL_OFF         */ { turn_radial_off,    service_process_ready, NULL, NULL, NULL                            },
/* 9  MOVE_SLEDGE        */ { turn_radial_off,    move_sledge, service_process_ready, NULL, NULL                     },
/*10  JUMP_GROOVES       */ { service_jump,       service_process_ready, NULL, NULL, NULL                            },
/*11  WRITE_CD6          */ { write_cd6,          service_process_ready, NULL, NULL, NULL                            },
/*12  WRITE_DSIC2        */ { write_dsic2,        service_process_ready, NULL, NULL, NULL                            },
/*13  READ_DSIC2         */ { read_dsic2_fn,      service_process_ready, NULL, NULL, NULL                            },
};

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Entry point for service commands (called from player dispatch table).
 *
 * Uses the same command-latching pattern as play() and start_stop():
 *   - If !service_command_busy or the command index changed, latch the new
 *     command, reset phases, set status = BUSY, and mark busy.
 *   - Once service_status != BUSY the command is done; clear the busy flag.
 *   - Always return service_status so caller can poll.
 *
 * @param  service_cmd  Command index (0–13).
 * @return BUSY / READY / CD_ERROR_STATE / PROCESS_READY.
 */
uint8_t service(uint8_t service_cmd)
{
    if ((service_process >> 4) != service_cmd || !service_command_busy) {
        service_process      = (uint8_t)(service_cmd << 4);
        service_phase0       = 0;
        service_phase1       = 0;
        service_active       = 1;
        service_status       = BUSY;
        service_command_busy = 1;
    }
    if (service_status != BUSY)
        service_command_busy = 0;
    return service_status;
}

/**
 * @brief  Background executor — advance the active service command.
 *
 * Called unconditionally once per main-loop iteration (from player()).
 * Returns without doing anything if no command is active (service_active == 0).
 *
 * Step advancement:
 *   PROCESS_READY → set service_status, clear service_active (done).
 *   READY         → increment low nibble of service_process (next step),
 *                   reset service_phase0 and service_phase1 to 0.
 *   CD_ERROR_STATE→ set service_status, clear service_active, run cleanup:
 *                     RADIAL_ERROR  → turn_radial_off()
 *                     MOTOR_ERROR   → turn_radial_off() + stop_motor()
 *                     FOCUS_ERROR   → above + switch_focus_off()
 *   BUSY          → no change; same step will execute on the next tick.
 *
 * Cleanup is layered to match the power hierarchy: we always tear down
 * from the highest active layer downward.
 */
void execute_service_functions(void)
{
    if (!service_active) return;

    uint8_t cmd_idx  = service_process >> 4;
    uint8_t step_idx = service_process & 0x0Fu;

    if (cmd_idx >= 14u || step_idx >= MAX_SVC_STEPS) {
        service_status = PROCESS_READY;
        service_active = 0;
        return;
    }

    svc_fn_t fn = service_functions[cmd_idx][step_idx];
    if (fn == NULL) {
        service_status = PROCESS_READY;
        service_active = 0;
        return;
    }

    switch (fn()) {
    case PROCESS_READY:
        service_status = PROCESS_READY;
        service_active = 0;
        break;

    case READY:
        service_process++;   /* advance step index (low nibble) */
        service_phase0 = 0;
        service_phase1 = 0;
        break;

    case CD_ERROR_STATE:
        service_status = CD_ERROR_STATE;
        service_active = 0;
        /* Layered cleanup — tear down from highest active subsystem */
        if      (player_error == RADIAL_ERROR) { turn_radial_off(); }
        else if (player_error == MOTOR_ERROR)  { turn_radial_off(); stop_motor(); }
        else if (player_error == FOCUS_ERROR)  { turn_radial_off(); stop_motor(); switch_focus_off(); }
        break;

    case BUSY:
    default:
        break;
    }
}
