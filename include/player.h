/**
 * @file  player.h
 * @brief Top-level player module API.
 *
 * The player is the central coordinator of all disc-control activity.
 * main() calls player() once per main-loop iteration; player() dispatches
 * the active process step and runs the background servo/subcode/shock tasks.
 *
 * ── Command interface ─────────────────────────────────────────────────────
 *
 * Commands arrive via player_interface, written by command_handler():
 *   player_interface.a_command = opcode   (one of the *_OPC constants)
 *   player_interface.param1–3  = arguments
 *
 * The player detects a_command != IDLE_OPC and advances player_interface
 * to the corresponding process sequence.  On completion it writes:
 *   player_interface.p_status = READY or CD_ERROR_STATE
 *   player_interface.a_command = IDLE_OPC
 *
 * command_handler() monitors p_status to trigger the STATUS_UPDATE reply.
 *
 * ── Process model ─────────────────────────────────────────────────────────
 *
 * Processes are expressed as 2D arrays of (function_ptr, param) pairs.
 * process_id indexes the row (which sequence is active); function_id
 * indexes the column (current step within that sequence).
 * A step returning READY advances function_id; PROCESS_READY ends the
 * entire sequence; BUSY waits; CD_ERROR_STATE aborts.
 *
 * ── Service-mode guard ────────────────────────────────────────────────────
 *
 * While in service mode (process_id's high nibble == service range),
 * player() bypasses the servo/subcode/shock background tasks to prevent
 * interference with direct hardware access by service commands.
 */

#pragma once
#include "defs.h"

/** Shared interface structure between command_handler and the player tick. */
extern interface_field_t player_interface;

/** Current player error code (set on CD_ERROR_STATE; cleared on new command). */
extern byte player_error;

/** Current process index (0xFF = idle). */
extern byte process_id;

/** Current function step within the active process. */
extern byte function_id;

/**
 * @brief  Initialise all sub-modules.
 *
 * Calls in order:
 *   timer_init()       — start 8 ms repeating timer + SCOR interrupt
 *   driver_init()      — configure all GPIO pins
 *   reset_dsic2_cd6()  — hardware reset sequence for CXD2500 and DSIC2
 *   servo_init()       — set servo state machine to its initial state
 *   cd6_init()         — send CXD2500 startup register configuration
 *
 * Also sets player_interface.a_command = IDLE_OPC and
 * player_interface.p_status = READY so command_handler() can begin
 * dispatching immediately.
 */
void player_init(void);

/**
 * @brief  Main player tick — call once per main-loop iteration.
 *
 * Performs two actions:
 *
 *   1. Command dispatch: if player_interface.a_command != IDLE_OPC,
 *      look up the corresponding process in the dispatch table, set
 *      process_id, and start executing its steps.
 *
 *   2. Process step execution: call the current step function.  On READY,
 *      advance to the next step.  On PROCESS_READY, mark the command done.
 *      On CD_ERROR_STATE, record the error and mark done.  On BUSY, wait.
 *
 *   3. Background tasks (when not in service mode):
 *        servo()           — advance the servo state machine
 *        subcode_module()  — decode a new Q-channel frame if SCOR fired
 *        shock_recover()   — check for disc stalls and seek to recover
 */
void player(void);
