/**
 * @file  cmd_hndl.c
 * @brief Command handler — translates COMMO opcodes to player_interface commands.
 *
 * The command handler sits between the Dispatcher and the player module.
 * Its role is to hold a received opcode + parameters until the player
 * interface becomes free, then write them atomically into player_interface
 * so player() picks them up on its next tick.
 *
 * ── Data flow ─────────────────────────────────────────────────────────────
 *
 *   Host → COMMO → Dispatcher → New_command() → s_pending_cmd
 *                                                    ↓ (when player is idle)
 *                               command_handler() → player_interface
 *                                                    ↓
 *                               player() → executes command → p_status=READY
 *                                                    ↓
 *                               command_handler() → Store_update_status(STATUS_UPDATE)
 *                                                    ↓
 *                               Dispatcher → SEND_STRING → COMMO → Host
 *
 * ── Double-buffering ──────────────────────────────────────────────────────
 *
 * s_pending_cmd holds the next command while player_interface.a_command
 * holds the current (or last) one.  command_handler() dispatches the
 * pending command as soon as:
 *   - player_interface.a_command == IDLE_OPC  (no active command)
 *   - player_interface.p_status  == READY      (player is not busy)
 *
 * This means the Dispatcher can accept the next COMMO packet while the
 * player is still finishing its previous command, as long as the pending
 * slot is empty.
 *
 * ── Status feedback ───────────────────────────────────────────────────────
 *
 * When command_handler() detects that the player has completed
 * (p_status == READY or CD_ERROR_STATE), it calls Store_update_status()
 * to notify the Dispatcher that a STATUS_UPDATE should be sent to the host.
 * This is the sole mechanism by which the host learns that a command finished.
 */

#include <stdint.h>
#include <string.h>

#include "defs.h"
#include "cmd_hndl.h"
#include "commo.h"
#include "player.h"
#include "sts_q_id.h"

/* =========================================================================
 * Internal state
 * ====================================================================== */

static uint8_t s_state          = IDLE;
static uint8_t s_pending_cmd    = NO_CMD;    /**< Opcode waiting for dispatch   */
static uint8_t s_pending_p1     = 0;
static uint8_t s_pending_p2     = 0;
static uint8_t s_pending_p3     = 0;
static uint8_t s_cmd_accepted   = 0;   /**< 1 = command has been written to player_interface */
static uint8_t s_handler_ready  = 1;   /**< 1 = pending slot is free; 0 = occupied or executing */

/* =========================================================================
 * Init
 * ====================================================================== */

void Init_command_handler(void)
{
    s_state         = IDLE;
    s_pending_cmd   = NO_CMD;
    s_cmd_accepted  = 0;
    s_handler_ready = 1;
}

/* =========================================================================
 * command_handler — called once per main-loop iteration
 *
 * Two responsibilities per tick:
 *
 *   1. If a command is pending and the player interface is free, write it
 *      into player_interface.  This is the only place player_interface is
 *      written — centralised to prevent races.
 *
 *   2. If we previously dispatched a command, monitor p_status.  When the
 *      player finishes (READY or CD_ERROR_STATE), mark the handler free and
 *      schedule a STATUS_UPDATE so the Dispatcher will send it to the host.
 * ====================================================================== */

/**
 * @brief  Advance the command handler by one tick.
 *
 * Called from main() before COMMO_INTERFACE() so that the player has a full
 * tick to begin executing before the next COMMO packet might arrive.
 */
void command_handler(void)
{
    /* ── Dispatch pending command if player is idle ── */
    if (s_pending_cmd != NO_CMD &&
        player_interface.a_command == IDLE_OPC &&
        player_interface.p_status  == READY)
    {
        player_interface.a_command = s_pending_cmd;
        player_interface.param1    = s_pending_p1;
        player_interface.param2    = s_pending_p2;
        player_interface.param3    = s_pending_p3;

        s_pending_cmd   = NO_CMD;
        s_cmd_accepted  = 1;
        s_handler_ready = 0;   /* slot is now occupied by the executing command */
    }

    /* ── Detect command completion ── */
    if (!s_handler_ready) {
        if (player_interface.p_status == READY ||
            player_interface.p_status == CD_ERROR_STATE)
        {
            s_handler_ready = 1;   /* ready to accept the next command */
            s_cmd_accepted  = 0;
            /* Notify Dispatcher: the host needs a status reply */
            Store_update_status(STATUS_UPDATE);
        }
    }
}

/* =========================================================================
 * New_command — called by the Dispatcher when COMMO_NEW_COMMAND is received
 *
 * Copies the opcode and parameters from the COMMO receive buffer into the
 * pending slot.  The Dispatcher must not call FREE_CMD_BUFFER() until
 * Cmd_acception_status() returns COMMO_TRUE, ensuring the COMMO buffer
 * stays valid long enough to be copied here.
 *
 * Returns COMMO_FALSE (busy) if a command is already pending, so the
 * Dispatcher will retry next tick.
 * ====================================================================== */

/**
 * @brief  Accept a new opcode from the Dispatcher.
 *
 * @return COMMO_TRUE  if the pending slot was free and the command was stored.
 *         COMMO_FALSE if the handler is still busy with the previous command.
 */
uint8_t New_command(void)
{
    if (!s_handler_ready) return COMMO_FALSE;   /* busy — try again next tick */

    s_pending_cmd = GET_BUFFER(0);
    s_pending_p1  = GET_BUFFER(1);
    s_pending_p2  = GET_BUFFER(2);
    s_pending_p3  = GET_BUFFER(3);

    s_handler_ready = 0;   /* mark busy until command_handler dispatches it */
    return COMMO_TRUE;
}

/**
 * @brief  Query whether the command handler can accept another command.
 *
 * Used by the Dispatcher to know when it is safe to call FREE_CMD_BUFFER().
 * Returns COMMO_TRUE once the handler has written the previous command into
 * player_interface and the pending slot is available again.
 *
 * @return COMMO_TRUE if ready, COMMO_FALSE if busy.
 */
uint8_t Cmd_acception_status(void)
{
    return s_handler_ready ? COMMO_TRUE : COMMO_FALSE;
}

/**
 * @brief  Query whether the command handler is completely idle.
 *
 * Similar to Cmd_acception_status() but named to emphasise that nothing
 * is executing or pending.
 *
 * @return COMMO_TRUE if idle, COMMO_FALSE otherwise.
 */
uint8_t Command_handler_ready(void)
{
    return s_handler_ready ? COMMO_TRUE : COMMO_FALSE;
}
