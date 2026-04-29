/**
 * @file  defs.h
 * @brief Master type definitions and command opcodes for the CD32 Pico firmware.
 *
 * Ported from the original Commodore/Philips 8051 firmware (1992-1993).
 * Adapted for the RP2350 (Raspberry Pi Pico 2).
 *
 * The Commodore CD32 uses a proprietary serial protocol (COMMO) over three
 * wires to the drive MCU.  The host sends one-byte opcodes (with optional
 * parameters) and the drive responds with status packets.  This header
 * defines all opcodes, error codes, and shared data structures that the
 * host protocol and internal sub-modules agree on.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Basic types
 *
 * 'byte' / 'Byte' mirror the 8051 CMOS byte type.  'rom' was the 8051
 * keyword for data held in code memory; we map it to const.
 * ---------------------------------------------------------------------- */
typedef uint8_t  byte;
typedef uint8_t  Byte;

#undef  rom
#define rom  const

/* -------------------------------------------------------------------------
 * TOC limits
 *
 * The CD standard allows up to 99 tracks; we only buffer 20 in RAM to
 * keep the TOC store small enough for the original 8051 data memory.
 * ---------------------------------------------------------------------- */
#define MAX_TRACK_STORED_IN_TOC   20

/* -------------------------------------------------------------------------
 * Interface field
 *
 * This structure is the handshake register between the host-comms layer
 * (commo.c / dispatcher.c / cmd_hndl.c) and the player engine (player.c).
 *
 * Writing a_command ≠ IDLE_OPC tells the player to execute that command.
 * The player clears a_command to IDLE_OPC as soon as it has latched it,
 * and sets p_status to BUSY for the duration.  When done it sets p_status
 * to READY (success) or CD_ERROR_STATE (failure, error code in param1).
 * ---------------------------------------------------------------------- */
typedef struct {
    byte p_status;   /**< BUSY / READY / CD_ERROR_STATE — current player state */
    byte a_command;  /**< Opcode to execute; IDLE_OPC when no command pending   */
    byte param1;     /**< Parameter 1 / error code on CD_ERROR_STATE            */
    byte param2;     /**< Parameter 2                                           */
    byte param3;     /**< Parameter 3                                           */
} interface_field_t;

/* -------------------------------------------------------------------------
 * Normal-mode command opcodes (a_command / COMMO opcode values)
 *
 * These are the commands the CD32 host may issue during normal playback.
 * The player.c dispatch table maps each opcode to a sequence of sub-module
 * step calls; see processes[][] in player.c.
 * ---------------------------------------------------------------------- */
#define TRAY_OUT_OPC              0x00  /**< Eject tray (not fitted on CD32)          */
#define TRAY_IN_OPC               0x01  /**< Load tray  (not fitted on CD32)          */
#define START_UP_OPC              0x02  /**< Spin up the disc from rest               */
#define STOP_OPC                  0x03  /**< Stop playback and park the laser          */
#define PLAY_TRACK_OPC            0x04  /**< Play from a given track number            */
#define PAUSE_ON_OPC              0x05  /**< Freeze the laser at the current position  */
#define PAUSE_OFF_OPC             0x06  /**< Resume from the paused position           */
#define SEEK_OPC                  0x07  /**< Seek to an absolute disc time             */
#define READ_TOC_OPC              0x08  /**< Read Table of Contents from lead-in area  */
#define READ_SUBCODE_OPC          0x09  /**< Return the current Q-channel subcode frame*/
#define SINGLE_SPEED_OPC          0x0A  /**< Switch to 1× disc speed                  */
#define DOUBLE_SPEED_OPC          0x0B  /**< Switch to 2× disc speed                  */
#define SET_VOLUME_OPC            0x0C  /**< Set DAC output volume level               */
#define JUMP_TRACKS_OPC           0x0D  /**< Jump ±N tracks relative to current pos.  */
#define ENTER_SERVICE_MODE_OPC    0x0E  /**< Enter diagnostic service mode             */

/* -------------------------------------------------------------------------
 * Service-mode command opcodes
 *
 * Only valid after ENTER_SERVICE_MODE_OPC; the player rejects them in
 * normal mode with ILLEGAL_COMMAND.
 * ---------------------------------------------------------------------- */
#define ENTER_NORMAL_MODE_OPC     0x0F  /**< Leave service mode and re-home sledge     */
#define LASER_ON_OPC              0x10  /**< Enable laser diode power                  */
#define LASER_OFF_OPC             0x11  /**< Disable laser diode power                 */
#define FOCUS_ON_OPC              0x12  /**< Enable focus servo loop                   */
#define FOCUS_OFF_OPC             0x13  /**< Disable focus servo loop                  */
#define SPINDLE_MOTOR_ON_OPC      0x14  /**< Start the spindle motor                   */
#define SPINDLE_MOTOR_OFF_OPC     0x15  /**< Stop the spindle motor                    */
#define RADIAL_ON_OPC             0x16  /**< Enable radial (tracking) servo loop       */
#define RADIAL_OFF_OPC            0x17  /**< Disable radial servo loop                 */
#define MOVE_SLEDGE_OPC           0x18  /**< Drive sledge motor in/out by timed pulse  */
#define JUMP_GROOVES_OPC          0x19  /**< Jump a fixed number of grooves            */
#define WRITE_CD6_OPC             0x1A  /**< Write raw byte to CXD2500BQ               */
#define WRITE_DSIC2_OPC           0x1B  /**< Write raw byte to DSIC2 servo IC          */
#define READ_DSIC2_OPC            0x1C  /**< Read a status byte from DSIC2             */

/* -------------------------------------------------------------------------
 * Internal opcodes and range limits
 * ---------------------------------------------------------------------- */
#define IDLE_OPC                  0xFF  /**< No command pending (interface idle)        */
#define ERROR_HANDLING_ID         0x00  /**< process_id reserved for error recovery     */
#define MAX_LEGAL_NORMAL_ID       0x0F  /**< Highest valid normal-mode process_id       */
#define MAX_LEGAL_SERVICE_ID      0x1D  /**< Highest valid service-mode process_id      */

/* -------------------------------------------------------------------------
 * Tray sub-commands (param1 for tray sequences)
 * ---------------------------------------------------------------------- */
#define TRAY_IDLE    0x00
#define TRAY_OUT     0x01
#define TRAY_IN      0x02

/* -------------------------------------------------------------------------
 * Start/stop sub-commands
 *
 * These are the values passed as 'cmd' to start_stop() from the player
 * dispatch table.  SS_IDLE means "hold current state and return READY".
 * ---------------------------------------------------------------------- */
#define SS_IDLE       0x00  /**< No action — return READY immediately         */
#define SS_STOP       0x01  /**< Stop the disc: servo_stop() → wait           */
#define SS_START_UP   0x02  /**< Spin up: servo_start() → disc-type detection */
#define SS_SPEED_N1   0x03  /**< Switch to 1× speed while running             */
#define SS_SPEED_N2   0x04  /**< Switch to 2× speed while running             */
#define SS_MOTOR_OFF  0x05  /**< Cut motor power immediately (error recovery) */

/* -------------------------------------------------------------------------
 * Play sub-commands
 *
 * Passed as 'cmd' to play() from the player dispatch table.  Each maps
 * to a row in play_processes[][] inside play.c.
 * ---------------------------------------------------------------------- */
#define PLAY_IDLE                   0x00  /**< Mute, stop subcode, return PROCESS_READY */
#define PLAY_STARTUP                0x01  /**< No-op (returns PROCESS_READY)             */
#define PAUSE_ON                    0x02  /**< Record pause position; enter PAUSE_MODE   */
#define PAUSE_OFF                   0x03  /**< Restore to pause position; track again    */
#define JUMP_TO_ADDRESS             0x04  /**< Seek to param1:param2:param3 (mm:ss:ff)   */
#define PLAY_TRACK                  0x05  /**< No-op (handled by higher layer)           */
#define PLAY_READ_SUBCODE           0x06  /**< Return current Q-channel frame to host    */
#define PLAY_READ_TOC               0x07  /**< Enter TOC-reading mode                    */
#define PLAY_PREPARE_SPEED_CHANGE   0x08  /**< Mute + record position before speed change*/
#define PLAY_RESTORE_SPEED_CHANGE   0x09  /**< Seek back to recorded pos after speed chg */
#define PLAY_SET_VOLUME             0x0A  /**< No-op (volume set in cmd_hndl.c)          */
#define PLAY_JUMP_TRACKS            0x0B  /**< Relative track jump (param1:param2 signed)*/

/* -------------------------------------------------------------------------
 * Player-level sub-commands (passed to player_module_fn inside player.c)
 * ---------------------------------------------------------------------- */
#define PLAYER_IDLE           0x00  /**< Mark interface READY and become idle  */
#define PLAYER_HANDLE_ERROR   0x01  /**< Decide whether to stop disc on error  */
#define SET_SERVICE_MODE      0x02  /**< Switch to diagnostic service mode     */

/* -------------------------------------------------------------------------
 * Process/function execution states
 *
 * Returned by every step-function in the player dispatch table.
 *
 *  BUSY          — still executing; retry next tick
 *  READY         — this step is done; advance to the next step
 *  CD_ERROR_STATE— this step failed; trigger the error-handling sequence
 *  PROCESS_READY — entire command sequence is finished
 * ---------------------------------------------------------------------- */
#define BUSY           0
#define READY          1
#define CD_ERROR_STATE 2    /**< Renamed from ERROR to avoid clash with system headers */
#define PROCESS_READY  3

/* -------------------------------------------------------------------------
 * Time comparison results (returned by compare_time() in maths.c)
 * ---------------------------------------------------------------------- */
#define SMALLER  0
#define EQUAL    1
#define BIGGER   2

/* -------------------------------------------------------------------------
 * Subcode area identifiers (used with is_subcode() in subcode.c)
 *
 * A CD disc is divided into three areas, each carrying different Q-channel
 * content:
 *   Lead-in  — TOC data (track start times, disc geometry)
 *   Program  — actual audio/data, tno = 01..99
 *   Lead-out — silence after the last track, tno = 0xAA
 * ---------------------------------------------------------------------- */
#define ALL_SUBCODES       0  /**< Match any valid Q-channel frame           */
#define ABS_TIME           1  /**< Mode-1 frame in the program area (tno≠0)  */
#define CATALOG_NR         2  /**< Mode-2 frame (UPC/EAN catalogue number)   */
#define ISRC_NR            3  /**< Mode-3 frame (ISRC code for current track)*/
#define FIRST_LEADIN_AREA  4  /**< Mode-1 frame in the lead-in (tno = 0)     */
#define LEADIN_AREA        5  /**< Any mode-1 or mode-5 frame in lead-in     */
#define PROGRAM_AREA       6  /**< Mode-1 frame, tno 01–99 (not lead-out)    */
#define LEADOUT_AREA       7  /**< Mode-1 frame, tno = 0xAA                  */

/* -------------------------------------------------------------------------
 * Lid / door states
 * ---------------------------------------------------------------------- */
#define CLOSED      0
#define OPEN        1
#define LID_OPEN    0
#define LID_CLOSED  1

/* -------------------------------------------------------------------------
 * Boolean convenience
 * ---------------------------------------------------------------------- */
#ifndef TRUE
#define TRUE   1
#endif
#ifndef FALSE
#define FALSE  0
#endif

/* -------------------------------------------------------------------------
 * Error codes
 *
 * Stored in player_error and reported back to the host in p_status.param1
 * when p_status == CD_ERROR_STATE.
 * ---------------------------------------------------------------------- */
#define NO_ERROR               0x00
#define ILLEGAL_COMMAND        0x01  /**< Opcode invalid in current mode              */
#define ILLEGAL_PARAMETER      0x02  /**< Parameter value out of range                */
#define SLEDGE_ERROR           0x03  /**< Sledge motor failed to reach target          */
#define FOCUS_ERROR            0x04  /**< Focus servo failed to lock                  */
#define MOTOR_ERROR            0x05  /**< Spindle motor failed to reach speed         */
#define RADIAL_ERROR           0x06  /**< Radial (tracking) servo failed to lock      */
#define PLL_LOCK_ERROR         0x07  /**< CXD2500 PLL failed to lock to disc clock    */
#define SUBCODE_TIMEOUT_ERROR  0x08  /**< Q-channel frame not received within timeout */
#define SUBCODE_NOT_FOUND      0x09  /**< Requested subcode area / type not found     */
#define TRAY_ERROR             0x0A  /**< Tray mechanism error                        */
#define TOC_READ_ERROR         0x0B  /**< TOC could not be read (retries exhausted)   */
#define JUMP_ERROR             0x0C  /**< Seek landed outside disc data area          */
#define HF_DETECTOR_ERROR      0x0D  /**< HF (high-frequency) signal absent after jump*/

/* -------------------------------------------------------------------------
 * CD time structure
 *
 * All three fields are in HEX (not BCD) inside the firmware.  BCD
 * conversion is performed only at the COMMO layer boundary.
 *
 * Standard CD frame rate is 75 frames per second, so:
 *   total_frames = (min * 60 + sec) * 75 + frm
 * ---------------------------------------------------------------------- */
typedef struct {
    byte min;  /**< Minutes  (0–74 for a standard 80-min disc) */
    byte sec;  /**< Seconds  (0–59)                            */
    byte frm;  /**< Frames   (0–74)                            */
} cd_time_t;

/* -------------------------------------------------------------------------
 * Q-channel subcode frame
 *
 * Reflects the standard CD Q-channel structure for Mode 1 (address=1)
 * frames.  The original firmware called this 'subcode_frame'.
 *
 *   conad  — Control nibble (high) + Address nibble (low); address=1 for time
 *   tno    — Track number (0 = lead-in, 0xAA = lead-out, 01–99 = program)
 *   index  — Index within track (0 = pre-gap, 1–99 = normal; 0xA0/A1/A2 in TOC)
 *   r_time — Relative time since start of current index
 *   zero   — Reserved byte (always 0)
 *   a_time — Absolute disc time from disc start
 * ---------------------------------------------------------------------- */
typedef struct {
    byte      conad;
    byte      tno;
    byte      index;
    cd_time_t r_time;
    byte      zero;
    cd_time_t a_time;
} subcode_frame_t;

/* -------------------------------------------------------------------------
 * 16-bit big/little split helper
 *
 * Used in play.c and servo.c to pass signed groove counts through the
 * player_interface (which uses byte-wide fields).
 * ---------------------------------------------------------------------- */
typedef struct { byte high; byte low; } byte_hl_t;
typedef union  { int val; byte_hl_t b; } int_hl_t;
