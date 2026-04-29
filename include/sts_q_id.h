/**
 * @file  sts_q_id.h
 * @brief Status / Q-channel / ID packet buffer module.
 *
 * Maintains a 15-byte outgoing packet buffer and a one-byte update-status
 * flag.  The Dispatcher polls Get_update_status() each tick and, when
 * non-zero, hands the buffer to COMMO for transmission.
 *
 * ── Packet layout ─────────────────────────────────────────────────────────
 *
 *   Byte 0: Echoed command opcode (set by Store_command())
 *   Byte 1: Error/condition byte (set by Store_error_condition())
 *           Bit 4 (ERROR_DETECT_MASK) is set to distinguish error responses
 *           from normal status values.
 *   Bytes 2–14: Filled by play.c before calling Store_update_status().
 *               Content depends on the packet type:
 *               STATUS_UPDATE — drive status flags
 *               Q_READY       — decoded Q-channel subcode fields
 *               ID_READY      — disc identification data
 *
 * ── Update-status codes ───────────────────────────────────────────────────
 *
 *   NO_UPDATE (0)     — Dispatcher does nothing; no packet pending.
 *   STATUS_UPDATE (1) — bytes 0–1 valid; send STATUS_PACKET_LENGTH bytes.
 *   Q_READY (2)       — bytes 0–14 contain a Q-channel packet.
 *   ID_READY (3)      — bytes 0–14 contain a disc ID packet.
 *   DISPATCH_STATUS(4)— in use by Dispatcher during active send.
 *
 * ── Lifecycle ─────────────────────────────────────────────────────────────
 *
 *   1. Producer (play.c or cmd_hndl.c) writes packet bytes, calls
 *      Store_command(), optionally Store_error_condition(), then
 *      Store_update_status(TYPE).
 *
 *   2. Dispatcher() calls Get_update_status(), Request_packet_transmit(),
 *      SEND_STRING(Get_sts_q_id_ptr(), length).
 *
 *   3. After SEND_STRING_READY() returns non-BUSY, Dispatcher calls
 *      Clear_update() to reset to NO_UPDATE.
 */

#pragma once
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────
 * Update status codes
 * ──────────────────────────────────────────────────────────────────────── */
#define NO_UPDATE        0x00   /**< No packet pending                       */
#define STATUS_UPDATE    0x01   /**< Status packet ready to transmit         */
#define Q_READY          0x02   /**< Q-channel packet ready                  */
#define ID_READY         0x03   /**< Disc ID packet ready                    */
#define DISPATCH_STATUS  0x04   /**< Dispatcher is actively sending          */

/* ─────────────────────────────────────────────────────────────────────────
 * Error condition codes (stored in byte 1 of the status packet)
 * ──────────────────────────────────────────────────────────────────────── */
#define CHECKSUM_ERROR   0x01   /**< COMMO packet checksum mismatch          */

/** Bit 4 of byte 1 is set whenever an error code is stored there.
 *  The host checks this bit to distinguish an error from a normal status. */
#define ERROR_DETECT_MASK  0x10

/* ─────────────────────────────────────────────────────────────────────────
 * API
 * ──────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Return a pointer to the 15-byte outgoing packet buffer.
 *
 * The Dispatcher passes this pointer directly to SEND_STRING().
 * Producers that need to fill additional packet bytes beyond byte 1
 * should index this pointer directly before calling Store_update_status().
 */
uint8_t *Get_sts_q_id_ptr(void);

/**
 * @brief  Return the current update-status code.
 * @return NO_UPDATE / STATUS_UPDATE / Q_READY / ID_READY / DISPATCH_STATUS.
 */
uint8_t Get_update_status(void);

/**
 * @brief  Schedule a packet for transmission.
 *
 * Sets the update-status code that the Dispatcher will act on during its
 * next tick.  Should only be called when Get_update_status() == NO_UPDATE
 * to avoid overwriting a pending packet.
 *
 * @param  status  One of STATUS_UPDATE / Q_READY / ID_READY.
 */
void Store_update_status(uint8_t status);

/**
 * @brief  Clear the pending update flag (called by Dispatcher after TX done).
 *
 * Resets update-status to NO_UPDATE so the same packet is not retransmitted.
 */
void Clear_update(void);

/**
 * @brief  Store the echoed command opcode in byte 0 of the packet.
 *
 * The host identifies which command a response belongs to by reading byte 0.
 * Call this before Store_update_status().
 *
 * @param  cmd  Opcode to echo.
 */
void Store_command(uint8_t cmd);

/**
 * @brief  Store an error code in byte 1 of the packet.
 *
 * OR's @p err with ERROR_DETECT_MASK (bit 4) so the host can detect errors.
 * The lower nibble carries the specific error code.
 *
 * @param  err  Error code (CHECKSUM_ERROR or similar).
 */
void Store_error_condition(uint8_t err);
