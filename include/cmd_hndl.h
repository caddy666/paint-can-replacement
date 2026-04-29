/**
 * @file  cmd_hndl.h
 * @brief Command handler — bridges COMMO/Dispatcher to the player module.
 *
 * The command handler acts as a single-slot queue between the Dispatcher
 * (which receives opcodes from the COMMO bus) and the player module (which
 * executes them).
 *
 * ── Command flow ──────────────────────────────────────────────────────────
 *
 *   Dispatcher   → New_command()           — copy opcode+params to pending slot
 *   command_handler() tick                 — when player is idle, move pending
 *                                            command to player_interface
 *   player()     → executes, sets p_status — command_handler() detects READY
 *                                            and queues STATUS_UPDATE
 *
 * ── Handler states ────────────────────────────────────────────────────────
 *
 *   s_handler_ready = 1:
 *     Pending slot is empty; New_command() will be accepted.
 *     Dispatcher may call FREE_CMD_BUFFER() to release the COMMO buffer.
 *
 *   s_handler_ready = 0:
 *     Either a command is pending in the slot, or has been dispatched to the
 *     player and has not yet completed.  New_command() returns COMMO_FALSE.
 */

#pragma once
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────
 * Playback state values
 * These match the p_status / disc state encoding used in status packets.
 * ──────────────────────────────────────────────────────────────────────── */
#define OPEN_S    0x00   /**< Disc door open / no disc        */
#define STOP_S    0x01   /**< Disc loaded, motor stopped      */
#define PLAY_S    0x02   /**< Playing                         */
#define PAUSE_S   0x03   /**< Paused                          */

/* ─────────────────────────────────────────────────────────────────────────
 * Internal command handler opcodes
 * These are generated internally (not received from the COMMO bus) and are
 * used to trigger housekeeping actions within the player or status module.
 * ──────────────────────────────────────────────────────────────────────── */
#define RESEND           0x80   /**< Re-send the last status packet          */
#define S_STAT           0x81   /**< Request a STATUS_UPDATE packet          */
#define S_CMD_ERR        0x82   /**< Report command error to host            */
#define S_ID             0x83   /**< Request an ID_READY packet              */
#define LED_CNTRL        0x84   /**< Control the status LED                  */
#define SET_ERROR_STATUS 0x85   /**< Force an error status packet            */
#define SEND_AUTO_Q      0x86   /**< Auto-send Q-channel data                */
#define SEND_Q           0x87   /**< Single Q-channel data packet            */
#define S_DISK_ERR       0x88   /**< Report a disc error to host             */
#define S_CLOSED         0x89   /**< Confirm door-closed state               */

/* ─────────────────────────────────────────────────────────────────────────
 * Play state sub-commands (param1 value for PLAY_OPC)
 * ──────────────────────────────────────────────────────────────────────── */
#define START_PAUSE  0x00   /**< Enter pause at current position             */
#define START_PLAY   0x01   /**< Begin playback from param2:param3 time      */
#define MODIFY_PLAY  0x02   /**< Adjust playback parameters while playing    */
#define STOP_C       0x03   /**< Stop and return to idle                     */
#define PLAY_PAUSE   0x04   /**< Toggle between play and pause               */
#define NEW_PLAY     0x05   /**< Start play from a new TOC position          */
#define SEEK_PLAY    0x06   /**< Seek then play                              */
#define SEEK_STOP    0x07   /**< Seek then stop                              */
#define PAUSE_PLAY   0x08   /**< Resume play from pause                      */
#define ENT_DIA      0x09   /**< Enter diagnostic / service access           */
#define DIA          0x0A   /**< Execute diagnostic command                  */
#define SEEK_PAUSE   0x0B   /**< Seek then pause                             */
#define OPEN_C       0x0C   /**< Open tray / eject                           */
#define AREA_ERROR   0x0D   /**< Report disc area error                      */

/* ─────────────────────────────────────────────────────────────────────────
 * Miscellaneous constants
 * ──────────────────────────────────────────────────────────────────────── */
#define RESEND_REQ     0x80   /**< Resend request flag in status byte        */
#define AUTO_Q         0x06   /**< Auto-Q channel mode identifier            */
#define MS_80          10     /**< 80 ms in 8 ms timer ticks                 */
#define IDLE           0xFF   /**< Handler idle / no command pending         */
#define NO_CMD         0x00   /**< No pending command                        */
#define STOPPED_CLOSED 0x01   /**< Stopped with door closed                  */
#define STOPPED_OPEN   0x00   /**< Stopped with door open                    */

/* ─────────────────────────────────────────────────────────────────────────
 * API
 * ──────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the command handler module.
 *
 * Clears the pending command slot and marks the handler ready.
 * Called once from main() before the main loop.
 */
void Init_command_handler(void);

/**
 * @brief  Advance the command handler by one tick.
 *
 * Called from main() once per loop iteration BEFORE COMMO_INTERFACE() and
 * player().
 *
 * Dispatches a pending command to player_interface when the player is idle,
 * and detects player completion to trigger a STATUS_UPDATE.
 */
void command_handler(void);

/**
 * @brief  Accept a new opcode from the Dispatcher.
 *
 * Copies opcode + up to 3 parameters from GET_BUFFER() into the pending
 * command slot.  Returns COMMO_FALSE if the handler is already busy.
 *
 * Called by the Dispatcher when NEW_CMD_RECEIVED() == COMMO_NEW_COMMAND.
 *
 * @return COMMO_TRUE if accepted; COMMO_FALSE if busy.
 */
uint8_t New_command(void);

/**
 * @brief  Query whether the command handler can accept another command.
 *
 * Returns COMMO_TRUE once the previous command has been dispatched to
 * player_interface and the pending slot is free again.  The Dispatcher
 * uses this to decide when to call FREE_CMD_BUFFER().
 *
 * @return COMMO_TRUE if ready; COMMO_FALSE if busy.
 */
uint8_t Cmd_acception_status(void);

/**
 * @brief  Query whether the command handler is completely idle.
 *
 * Same semantics as Cmd_acception_status() but used in contexts where the
 * caller wants to confirm there is no in-flight command at all.
 *
 * @return COMMO_TRUE if idle; COMMO_FALSE otherwise.
 */
uint8_t Command_handler_ready(void);
