/**
 * @file  dispatcher.c
 * @brief Dispatcher — arbitrates COMMO transmits and routes received commands.
 *
 * The Dispatcher sits between the COMMO serial layer and the command handler.
 * It runs once per main-loop iteration and performs four tasks in order:
 *
 *   1. Complete pending transmit: if a previous SEND_STRING has finished,
 *      acknowledge the corresponding status/Q/ID update or retry a suspended
 *      transmit.
 *
 *   2. Capture pending update: if sts_q_id has a new packet ready and no
 *      transmit is active, snapshot the update type so it can be sent.
 *
 *   3. Release receive buffer: once the command handler confirms acceptance
 *      (Cmd_acception_status == COMMO_TRUE), call FREE_CMD_BUFFER() to allow
 *      the COMMO layer to accept the next incoming packet.
 *
 *   4. Route received command: poll NEW_CMD_RECEIVED() and dispatch:
 *      - COMMO_NEW_COMMAND  → pass to command handler via New_command()
 *      - COMMO_SAME_COMMAND → echo current status back to host (no re-execute)
 *      - COMMO_CMD_ERROR    → report CHECKSUM_ERROR status to host
 *
 * ── Suspend / retry mechanism ─────────────────────────────────────────────
 *
 * If the COMMO bus is busy when a status update needs to be sent, the update
 * type is saved in s_suspend_transmit.  After the current transmit completes,
 * Request_packet_transmit(s_suspend_transmit) is called to retry.
 *
 * ── COMMO_SAME_COMMAND handling ───────────────────────────────────────────
 *
 * When the host resends the same opcode it is asking for the current status
 * without re-executing the command.  We store the opcode and schedule a
 * STATUS_UPDATE so the host gets the most recent status packet.  This is
 * the host's polling mechanism for long-running commands.
 *
 * Ported from the original Philips/Commodore 1993 firmware with all 8051
 * constructs removed.
 */

#include <stdint.h>

#include "defs.h"
#include "commo.h"
#include "sts_q_id.h"
#include "cmd_hndl.h"

/* =========================================================================
 * Module state
 * ====================================================================== */

/** Type of packet that is pending or was last sent (NO_UPDATE / STATUS_UPDATE
 *  / Q_READY / ID_READY).  Mirrors the update-status code from sts_q_id. */
static uint8_t s_suspend_transmit      = NO_UPDATE;

/** Non-zero once a command has been forwarded to the command handler and we
 *  are waiting for the handler to finish so we can release the receive buffer. */
static uint8_t s_report_for_free_buf   = 0;

/** Saved receive result for a command that could not be routed immediately
 *  because the status update slot was already occupied. */
static uint8_t s_cmd_still_to_report   = COMMO_NO_COMMAND;

/** Non-zero when the current in-progress COMMO transmit originated from the
 *  status/Q/ID module (and therefore we must call Clear_update() on completion). */
static uint8_t s_acknowledge_update    = 0;

/* =========================================================================
 * Request_packet_transmit
 *
 * Attempt to hand a packet to the COMMO TX layer.
 * The packet contents are determined by the sts_q_id module and are ready
 * in its internal buffer (Get_sts_q_id_ptr()).
 *
 * Returns COMMO_TRUE if the transmit was accepted, COMMO_FALSE if the COMMO
 * interface was already busy.
 *
 * Packet lengths:
 *   STATUS_PACKET_LENGTH  — 15 bytes: status + Q + ID fields (all three)
 *   Q_PACKET_LENGTH       — 15 bytes
 *   ID_PACKET_LENGTH      — 15 bytes
 *
 * All three are 15 bytes in this implementation; the distinction exists for
 * future use where different packet subsets might be sent.
 * ====================================================================== */

static uint8_t Request_packet_transmit(uint8_t mode)
{
    switch (mode) {

    case NO_UPDATE:
        return COMMO_TRUE;

    case STATUS_UPDATE:
        if (SEND_STRING(SEND_STRING_COMPLETE,
                        Get_sts_q_id_ptr(),
                        STATUS_PACKET_LENGTH) == COMMO_TRUE) {
            s_acknowledge_update = 1;
            return COMMO_TRUE;
        }
        return COMMO_FALSE;

    case Q_READY:
        if (SEND_STRING(SEND_STRING_COMPLETE,
                        Get_sts_q_id_ptr(),
                        Q_PACKET_LENGTH) == COMMO_TRUE) {
            s_acknowledge_update = 1;
            return COMMO_TRUE;
        }
        return COMMO_FALSE;

    case ID_READY:
        if (SEND_STRING(SEND_STRING_COMPLETE,
                        Get_sts_q_id_ptr(),
                        ID_PACKET_LENGTH) == COMMO_TRUE) {
            s_acknowledge_update = 1;
            return COMMO_TRUE;
        }
        return COMMO_FALSE;

    default:
        return COMMO_FALSE;
    }
}

/* =========================================================================
 * Dispatcher — called once per main-loop iteration
 * ====================================================================== */

/**
 * @brief  Advance the dispatcher by one tick.
 *
 * Must be called from the main loop after COMMO_INTERFACE() and before
 * player() so that status updates are queued promptly after command
 * completion.
 */
void Dispatcher(void)
{
    /* ── 1. Check if a previous COMMO transmit has completed ─────────── */
    if (SEND_STRING_READY() <= COMMO_READY_WITH_ERROR) {

        if (s_acknowledge_update) {
            /* This transmit delivered a status/Q/ID packet — clear the source */
            Clear_update();
            s_suspend_transmit   = NO_UPDATE;
            s_acknowledge_update = 0;
        } else {
            if (s_suspend_transmit != NO_UPDATE) {
                /* A packet was suspended because COMMO was busy; retry now */
                Request_packet_transmit(s_suspend_transmit);
            }
        }
    }

    /* ── 2. Capture new pending status update ────────────────────────── */
    if (s_suspend_transmit == NO_UPDATE &&
        Get_update_status() != NO_UPDATE)
    {
        /* Snapshot the update type; attempt to transmit it */
        s_suspend_transmit = Get_update_status();
        Request_packet_transmit(s_suspend_transmit);
    }

    /* ── 3. Release the COMMO receive buffer when the handler is ready ──
     * FREE_CMD_BUFFER() must not be called until the command handler has
     * finished processing the current packet, otherwise the buffer could
     * be overwritten while the handler still needs it. */
    if (s_report_for_free_buf && Cmd_acception_status() == COMMO_TRUE) {
        if (FREE_CMD_BUFFER() == COMMO_TRUE) {
            s_report_for_free_buf = 0;
        }
    }

    /* ── 4. Poll for newly received commands ─────────────────────────── */
    if (s_cmd_still_to_report == COMMO_NO_COMMAND) {
        s_cmd_still_to_report = NEW_CMD_RECEIVED();
    }

    switch (s_cmd_still_to_report) {

    case COMMO_NO_COMMAND:
        break;

    case COMMO_NEW_COMMAND:
        /* Forward to command handler; if busy it will retry next tick */
        if (New_command() == COMMO_TRUE) {
            s_cmd_still_to_report = COMMO_NO_COMMAND;
            s_report_for_free_buf = 1;
        }
        break;

    case COMMO_SAME_COMMAND:
        /* Host is polling status for the last command — re-send current status.
         * Wait until the update slot is free before claiming it. */
        if (Get_update_status() == NO_UPDATE) {
            Store_command(GET_BUFFER(0));
            Store_update_status(STATUS_UPDATE);
            s_suspend_transmit    = STATUS_UPDATE;
            Request_packet_transmit(STATUS_UPDATE);
            s_report_for_free_buf = 1;
            s_cmd_still_to_report = COMMO_NO_COMMAND;
        }
        break;

    case COMMO_CMD_ERROR:
        /* Checksum or protocol error — report error status to host */
        if (Get_update_status() == NO_UPDATE) {
            Store_command(GET_BUFFER(0));
            Store_error_condition(CHECKSUM_ERROR);
            Store_update_status(STATUS_UPDATE);
            s_suspend_transmit    = STATUS_UPDATE;
            Request_packet_transmit(STATUS_UPDATE);
            s_report_for_free_buf = 1;
            s_cmd_still_to_report = COMMO_NO_COMMAND;
        }
        break;

    default:
        s_cmd_still_to_report = COMMO_NO_COMMAND;
        break;
    }
}
