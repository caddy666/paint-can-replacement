/**
 * @file  sts_q_id.c
 * @brief Status / Q-channel / ID packet buffer module.
 *
 * Maintains a 15-byte packet buffer and an update-status flag.  The
 * Dispatcher polls Get_update_status() each tick; when a packet is ready
 * it calls Get_sts_q_id_ptr() to obtain the buffer address and queues it
 * for COMMO transmission via SEND_STRING().
 *
 * ── Packet layout (15 bytes) ──────────────────────────────────────────────
 *
 *   Byte 0: Echoed command opcode — tells the host which command this
 *           status response belongs to.
 *   Byte 1: Error/condition byte — bit 4 (ERROR_DETECT_MASK = 0x10) is
 *           set whenever an error is stored here to distinguish error
 *           responses from normal status bytes.
 *   Bytes 2–14: Additional fields filled in by play.c (Q-channel data,
 *               disc ID, track position, etc.) depending on the update type.
 *
 * ── Update status codes ───────────────────────────────────────────────────
 *
 *   NO_UPDATE     (0x00) — nothing pending; Dispatcher does nothing.
 *   STATUS_UPDATE (0x01) — bytes 0–1 are valid; send STATUS_PACKET_LENGTH bytes.
 *   Q_READY       (0x02) — bytes 0–14 contain Q-channel data.
 *   ID_READY      (0x03) — bytes 0–14 contain disc ID.
 *   DISPATCH_STATUS(0x04)— internal; dispatcher is actively sending.
 *
 * ── Thread safety ─────────────────────────────────────────────────────────
 *
 * All accesses are from the cooperative main loop — no ISR writes here —
 * so no atomic protection is needed.  The Dispatcher calls Clear_update()
 * after the transmit completes to prevent the same packet being sent twice.
 */

#include <stdint.h>
#include <string.h>

#include "sts_q_id.h"

/* =========================================================================
 * Packet buffer
 * ====================================================================== */

#define PACKET_BUF_SIZE  15

/** Outgoing packet buffer; single instance shared by all packet types. */
static uint8_t  s_packet[PACKET_BUF_SIZE];

/** Current update pending for the Dispatcher (NO_UPDATE when idle). */
static uint8_t  s_update_status = NO_UPDATE;

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Return a pointer to the packet buffer.
 *
 * The Dispatcher passes this pointer directly to SEND_STRING().  Callers
 * that need to fill the buffer (play.c, cmd_hndl.c) should write into
 * this buffer before calling Store_update_status().
 */
uint8_t *Get_sts_q_id_ptr(void)
{
    return s_packet;
}

/** @return Current update-status code (NO_UPDATE / STATUS_UPDATE / Q_READY / ID_READY). */
uint8_t Get_update_status(void)
{
    return s_update_status;
}

/**
 * @brief  Schedule a packet of the given type for transmission.
 *
 * The Dispatcher will send the packet on its next tick.  Calling this
 * while an update is already pending overwrites the pending type — the
 * caller is responsible for only writing when Get_update_status()==NO_UPDATE.
 */
void Store_update_status(uint8_t status)
{
    s_update_status = status;
}

/**
 * @brief  Clear the pending update flag.
 *
 * Called by the Dispatcher after the COMMO transmit completes, preventing
 * the same packet from being retransmitted on the next Dispatcher tick.
 */
void Clear_update(void)
{
    s_update_status = NO_UPDATE;
}

/**
 * @brief  Store the echoed command opcode in byte 0 of the packet.
 *
 * The host identifies which command a status reply belongs to by reading
 * byte 0.  Always call this before Store_update_status().
 *
 * @param  cmd  Opcode to echo.
 */
void Store_command(uint8_t cmd)
{
    s_packet[0] = cmd;
}

/**
 * @brief  Store an error code in byte 1 of the packet.
 *
 * Sets ERROR_DETECT_MASK (bit 4) so the host can distinguish an error
 * response from a normal status byte.  The low nibble of byte 1 carries
 * the error code itself (e.g. CHECKSUM_ERROR = 0x01).
 *
 * @param  err  Error code (without the mask; mask is OR'd in here).
 */
void Store_error_condition(uint8_t err)
{
    s_packet[1] = err | ERROR_DETECT_MASK;
}
