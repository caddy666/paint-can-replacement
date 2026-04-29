/**
 * @file  commo.h
 * @brief COMMO serial interface — command receive and status transmit API.
 *
 * The CD32 host (Amiga chipset) communicates with the drive firmware over a
 * 3-wire serial bus:
 *   DATA  — bidirectional data line (we control direction via DIR)
 *   CLK   — driven by the host during receive; driven by us during transmit
 *   DIR   — direction output from this firmware (1=we transmit, 0=we receive)
 *
 * ── Packet format ─────────────────────────────────────────────────────────
 *
 * All bytes are transferred LSB-first.
 *
 * Receive:  [opcode] [param...] [checksum]
 *   Checksum = ~(sum of opcode + all params)  (one's complement)
 *
 * Transmit: [data...] [checksum]  (if SEND_STRING_COMPLETE mode)
 *           [data...]             (if SEND_STRING_APPEND mode)
 *
 * ── Receive acknowledgement ───────────────────────────────────────────────
 *
 * After each byte is clocked in, the firmware briefly asserts DIR=0 as a
 * per-byte acknowledgement.  The host waits for this before sending the
 * next byte.
 *
 * ── COMMO state machine states ───────────────────────────────────────────
 *
 *   IDLE → RXD_OPCODE → RXD_PARM* → RXD_CHECKSUM → IDLE
 *   IDLE → TXD_DATA → TXD_CHECKSUM → IDLE
 *   IDLE → TXD_DATA → IDLE  (no checksum mode)
 *   (on zero opcode) → ERR_SEND (128-tick delay) → IDLE
 *
 * Each call to COMMO_INTERFACE() advances the state machine by one step.
 */

#pragma once
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────
 * Boolean return values
 * ──────────────────────────────────────────────────────────────────────── */
#define COMMO_FALSE               0x00   /**< Operation not possible / rejected */
#define COMMO_TRUE                0x01   /**< Operation accepted / succeeded    */
#define COMMO_ERROR               0x02   /**< Operation completed with error    */

/* ─────────────────────────────────────────────────────────────────────────
 * Transmit readiness codes (returned by SEND_STRING_READY)
 * ──────────────────────────────────────────────────────────────────────── */
#define COMMO_READY_WITHOUT_ERROR 0x00   /**< Idle — previous TX succeeded     */
#define COMMO_READY_WITH_ERROR    0x01   /**< Idle — previous TX had an error  */
#define COMMO_BUSY                0x03   /**< TX in progress                   */
#define COMMO_PENDING             0x04   /**< TX queued but not yet started    */

/* ─────────────────────────────────────────────────────────────────────────
 * Receive result codes (returned by NEW_CMD_RECEIVED)
 * ──────────────────────────────────────────────────────────────────────── */
#define COMMO_NO_COMMAND          0x00   /**< No complete packet received yet  */
#define COMMO_NEW_COMMAND         0x01   /**< New opcode decoded successfully  */
#define COMMO_SAME_COMMAND        0x02   /**< Same opcode as last; host polling */
#define COMMO_CMD_ERROR           0x03   /**< Checksum mismatch or bad opcode  */

/* ─────────────────────────────────────────────────────────────────────────
 * Opcode field masks
 * ──────────────────────────────────────────────────────────────────────── */
#define COMMO_OPCODE_MASK   0x0F   /**< Low nibble: identifies the command class */
#define COMMO_INDEX_MASK    0xF0   /**< High nibble: command instance or flags   */

/* ─────────────────────────────────────────────────────────────────────────
 * SEND_STRING mode selectors
 * ──────────────────────────────────────────────────────────────────────── */
#define SEND_STRING_COMPLETE  1   /**< Append one's-complement checksum byte     */
#define SEND_STRING_APPEND    0   /**< Send data bytes only (no checksum)        */

/* ─────────────────────────────────────────────────────────────────────────
 * Outgoing packet lengths
 *
 * All three packet types use a 15-byte buffer in sts_q_id.c.  The same
 * buffer holds whichever packet type was most recently prepared.
 * ──────────────────────────────────────────────────────────────────────── */
#define STATUS_PACKET_LENGTH  15   /**< Drive status + echoed command           */
#define Q_PACKET_LENGTH       15   /**< Q-channel subcode data                  */
#define ID_PACKET_LENGTH      15   /**< Disc ID / ISRC                          */

/* ─────────────────────────────────────────────────────────────────────────
 * API
 * ──────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise COMMO GPIO pins and reset the state machine.
 *
 * PIN_COMMO_DIR → output, driven high (receive state)
 * PIN_COMMO_CLK → input, pull-up
 * PIN_COMMO_DATA→ input, pull-up
 *
 * Must be called once from main() before the main loop begins.
 */
void COMMO_INIT(void);

/**
 * @brief  Advance the COMMO state machine by one step.
 *
 * Must be called once per main-loop iteration.  Each call handles at most
 * one complete byte transfer (8 bits clocked in or out) plus one state
 * transition, ensuring the cooperative main loop is not starved.
 */
void COMMO_INTERFACE(void);

/**
 * @brief  Poll for a newly decoded receive packet.
 *
 * Returns a non-zero result only once per received packet.  The result
 * persists until FREE_CMD_BUFFER() is called.
 *
 * @return COMMO_NEW_COMMAND  — valid packet with a new opcode.
 *         COMMO_SAME_COMMAND — valid packet repeating the previous opcode.
 *         COMMO_CMD_ERROR    — checksum failure or zero opcode received.
 *         COMMO_NO_COMMAND   — no complete packet available yet.
 */
uint8_t NEW_CMD_RECEIVED(void);

/**
 * @brief  Return one byte from the receive buffer.
 *
 * Buffer layout:
 *   index 0 = opcode
 *   index 1 = param1 (first parameter, if present)
 *   …
 *   index N = paramN (up to 11 parameter bytes for the largest opcode)
 *
 * @param  idx  Buffer index (0–11).
 * @return Byte value at that index; 0 if idx is out of range.
 */
uint8_t GET_BUFFER(uint8_t idx);

/**
 * @brief  Queue a packet for COMMO transmission.
 *
 * Copies @p length bytes from @p data into the internal TX buffer.
 * The actual transmission begins on a subsequent COMMO_INTERFACE() call.
 *
 * Only one transmit can be queued at a time.  The caller should check
 * SEND_STRING_READY() before calling this function.
 *
 * @param  mode    SEND_STRING_COMPLETE to append a checksum byte;
 *                 SEND_STRING_APPEND to send data only.
 * @param  data    Source byte array.
 * @param  length  Number of bytes to send (maximum 16).
 * @return COMMO_TRUE if accepted; COMMO_FALSE if busy or packet too long.
 */
uint8_t SEND_STRING(uint8_t mode, uint8_t *data, uint8_t length);

/**
 * @brief  Poll transmit completion.
 *
 * @return COMMO_BUSY if transmit is in progress.
 *         COMMO_READY_WITHOUT_ERROR (0) when idle (no active TX).
 *
 * Note: the Dispatcher uses the comparison "<= COMMO_READY_WITH_ERROR"
 * (i.e., value <= 1) as the "done" test, covering both success and error.
 */
uint8_t SEND_STRING_READY(void);

/**
 * @brief  Release the COMMO receive buffer.
 *
 * Clears the report_cmd flag so NEW_CMD_RECEIVED() returns COMMO_NO_COMMAND
 * again, allowing the COMMO layer to accept the next incoming packet.
 *
 * Must be called by the Dispatcher after the command handler has
 * accepted the current packet (Cmd_acception_status() == COMMO_TRUE).
 *
 * @return COMMO_TRUE always.
 */
uint8_t FREE_CMD_BUFFER(void);
