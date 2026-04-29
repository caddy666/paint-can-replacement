/**
 * @file  commo.c
 * @brief COMMO serial interface state machine.
 *
 * Implements the 3-wire bidirectional serial protocol between this firmware
 * and the Amiga CD32 host chipset (Paula / Akiko).
 *
 * ── Physical bus ──────────────────────────────────────────────────────────
 *
 *   PIN_COMMO_CLK  — clock line, always driven by the HOST (input to us
 *                    during receive; we drive it during transmit)
 *   PIN_COMMO_DATA — data line, bidirectional; we select direction via DIR
 *   PIN_COMMO_DIR  — direction control output from this firmware:
 *                      1 = drive transmitting (we own DATA)
 *                      0 = host transmitting (we sample DATA)
 *
 * ── Receive protocol (host → drive) ──────────────────────────────────────
 *
 *   The host signals intent to send by pulling DATA low while CLK is low.
 *   We detect this in the IDLE state via commo_get_data() == 0.
 *
 *   Byte reception (get_rxd_data):
 *     1. Drive CLK low.
 *     2. For each of 8 bits:
 *          - Raise CLK (sample point).
 *          - Sample DATA into the MSB of the accumulator, then shift right.
 *          - Lower CLK.
 *     3. Pull DIR low briefly as a byte-level acknowledgement.
 *
 *   Note: bits arrive LSB-first on the wire, but we shift right from MSB
 *   after sampling, so the first bit received ends up at bit 7 and shifts
 *   down — effectively reconstructing the byte MSB-first in software.
 *
 * ── Transmit protocol (drive → host) ─────────────────────────────────────
 *
 *   We wait for the host to release DATA (DATA == 1) before each byte.
 *   We drive DATA with the current bit, toggle CLK ourselves, then shift
 *   the byte right and repeat for all 8 bits.
 *
 *   After the last bit we release the bus: DIR=1, DATA=1 (idle state).
 *   Bits are sent LSB-first to match the host's receive logic.
 *
 * ── Packet structure ──────────────────────────────────────────────────────
 *
 *   Receive:  [opcode] [param_0] … [param_N-1] [checksum]
 *   Transmit: [data_0] … [data_M-1] [checksum]          (if SEND_STRING_COMPLETE)
 *
 *   The number of parameter bytes for each opcode is looked up from
 *   command_length_table[opcode & 0x0F], which includes the opcode itself
 *   but not the final checksum byte.
 *
 * ── Checksum ──────────────────────────────────────────────────────────────
 *
 *   One's complement of the arithmetic sum of all data bytes.
 *
 *   Receive validation:  ~received_checksum == accumulated_sum
 *   Transmit generation: send ~accumulated_sum after all data bytes.
 *
 *   This is the same checksum scheme used in IPv4 headers.
 *
 * ── command_length_table ──────────────────────────────────────────────────
 *
 *   Indexed by (opcode & 0x0F), the table gives the total number of bytes
 *   in the command body (opcode + parameters, NOT counting the checksum):
 *
 *     index:  0  1  2  3   4  5  6  7  8  9 10 11 12 13 14 15
 *     length: 1  2  1  1  12  2  1  1  4  1  1  1  1  2  1  1
 *
 *   The maximum is 12 bytes (index 4), matching the largest opcode payload
 *   (e.g. a full TOC request or bulk subcode read with parameters).
 *
 * Ported from the original Philips/Commodore 8051 firmware (1992-1993).
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <string.h>

#include "commo.h"
#include "gpio_map.h"

/* =========================================================================
 * Internal state machine
 * ====================================================================== */

/**
 * State enum for the COMMO serial state machine.
 *
 *   IDLE         — wait for host start-of-byte (DATA low) or queue TX
 *   RXD_OPCODE   — receive first byte of incoming packet (the opcode)
 *   RXD_PARM     — receive one parameter byte at a time
 *   RXD_CHECKSUM — receive and validate the final checksum byte
 *   TXD_DATA     — transmit one data byte per call
 *   TXD_CHECKSUM — transmit the one's-complement checksum
 *   ERR_SEND     — count-down delay before reporting a receive error
 */
typedef enum {
    COMMO_SM_IDLE = 0,
    COMMO_SM_RXD_OPCODE,
    COMMO_SM_RXD_PARM,
    COMMO_SM_RXD_CHECKSUM,
    COMMO_SM_TXD_DATA,
    COMMO_SM_TXD_CHECKSUM,
    COMMO_SM_ERR_SEND
} commo_sm_state_t;

typedef struct {
    commo_sm_state_t state;

    uint8_t cmd_length;       /**< Total bytes expected in body (from table)   */
    uint8_t checksum;         /**< Running byte sum (one's complement on TX)   */
    uint8_t byte_counter;     /**< Bytes received so far / bytes left to send  */
    uint8_t byte_pointer;     /**< TX buffer read index                        */
    uint8_t rx_status;        /**< Outcome: COMMO_NEW/SAME/CMD_ERROR           */
    uint8_t last_command;     /**< Opcode of the last successfully decoded cmd */

    uint8_t rx_buffer[12];    /**< Holds opcode + up to 11 parameter bytes     */
    uint8_t tx_buffer[16];    /**< Outgoing packet bytes                       */
    uint8_t tx_length;        /**< Number of bytes loaded into tx_buffer       */

    uint8_t tx_req;           /**< 1 = a transmit has been queued              */
    uint8_t tx_chk_req;       /**< 1 = append checksum byte after data         */
    uint8_t report_cmd;       /**< 1 = RX result is ready for NEW_CMD_RECEIVED */
    uint8_t cmd_buf_free;     /**< 1 = caller has called FREE_CMD_BUFFER       */
} commo_ctx_t;

static commo_ctx_t s_commo;

/**
 * Number of body bytes (including opcode, excluding checksum) for each
 * opcode family.  Indexed by (opcode & 0x0F).
 *
 * A value of 1 means the opcode carries no parameters — the byte after the
 * opcode is already the checksum.  Values > 1 mean that many bytes must be
 * received before the checksum state.
 */
static const uint8_t command_length_table[16] = {
    1, 2, 1, 1, 12, 2, 1, 1, 4, 1, 1, 1, 1, 2, 1, 1
};

/* =========================================================================
 * GPIO helpers
 * ====================================================================== */

static inline void commo_set_dir(int v)
{
    gpio_put(PIN_COMMO_DIR, (uint32_t)v);
}

/** Switch DATA to output mode and set its level. */
static inline void commo_set_data_out(int v)
{
    gpio_set_dir(PIN_COMMO_DATA, GPIO_OUT);
    gpio_put(PIN_COMMO_DATA, (uint32_t)v);
}

/** Switch DATA to input mode and return its current level. */
static inline int commo_get_data(void)
{
    gpio_set_dir(PIN_COMMO_DATA, GPIO_IN);
    return (int)gpio_get(PIN_COMMO_DATA);
}

static inline void commo_set_clk(int v)
{
    gpio_put(PIN_COMMO_CLK, (uint32_t)v);
}

/* =========================================================================
 * Low-level byte I/O
 * ====================================================================== */

/**
 * @brief  Receive one byte from the host, host-clocked.
 *
 * Drives CLK and samples DATA on each rising edge.  The host provides
 * data on DATA; we sample on CLK rising edge.  After all 8 bits, pull
 * DIR low briefly to acknowledge receipt to the host.
 *
 * Bit order: first bit received is placed at bit 7 of the accumulator and
 * shifted right on each subsequent bit, so bits arrive in LSB-first order
 * on the wire and are reconstructed correctly in the 8-bit result.
 *
 * @return Received byte value.
 */
static uint8_t get_rxd_data(void)
{
    uint8_t a = 0;

    commo_set_clk(0);
    for (int i = 0; i < 8; i++) {
        commo_set_clk(1);
        a >>= 1;
        if (commo_get_data()) a |= 0x80u;
        commo_set_clk(0);
    }
    commo_set_dir(0);   /* acknowledgement pulse */
    return a;
}

/**
 * @brief  Transmit one byte to the host, drive-clocked.
 *
 * We own CLK during transmit.  Data is placed on DATA before each CLK
 * rising edge.  Bits are sent LSB-first.
 *
 * After the last bit: assert DIR=1 and DATA=1 to return the bus to idle.
 *
 * @param  a  Byte to send.
 */
static void transmit_txd(uint8_t a)
{
    for (int i = 0; i < 8; i++) {
        commo_set_data_out(a & 1);
        commo_set_clk(0);
        commo_set_clk(1);
        a >>= 1;
    }
    commo_set_dir(1);
    commo_set_data_out(1);
}

/* =========================================================================
 * State machine — one step per main-loop call
 *
 * Each call to commo_step() advances the state machine by exactly one
 * step.  Blocking I/O is performed only inside get_rxd_data() and
 * transmit_txd() which loop for 8 bits — those are fast enough that they
 * do not violate the cooperative multitasking contract.
 * ====================================================================== */

static void commo_step(commo_ctx_t *c)
{
    switch (c->state) {

    case COMMO_SM_IDLE:
        if (c->tx_req) {
            /* Transmit has priority: start sending when DATA is released */
            c->byte_pointer = 0;
            c->checksum     = 0;
            c->state        = COMMO_SM_TXD_DATA;
        } else if (!commo_get_data()) {
            /* Host pulled DATA low — start-of-byte detected */
            c->state        = COMMO_SM_RXD_OPCODE;
            c->byte_counter = 0;
            c->checksum     = 0;
        }
        break;

    case COMMO_SM_RXD_OPCODE: {
        uint8_t b = get_rxd_data();
        if (b == 0) {
            /* A zero opcode is invalid; count down 128 ticks then report error */
            c->state        = COMMO_SM_ERR_SEND;
            c->byte_counter = 128;
            break;
        }
        c->rx_buffer[0] = b;
        c->checksum     = b;
        c->byte_counter = 1;
        /* Look up how many bytes this opcode family needs */
        c->cmd_length   = command_length_table[b & 0x0Fu];
        /* If cmd_length == 1 there are no parameters — go straight to checksum */
        c->state = (c->cmd_length == 1) ? COMMO_SM_RXD_CHECKSUM
                                        : COMMO_SM_RXD_PARM;
        break;
    }

    case COMMO_SM_RXD_PARM:
        /* Wait for host to signal next byte (DATA must go low first) */
        if (commo_get_data()) break;
        {
            uint8_t b = get_rxd_data();
            c->rx_buffer[c->byte_counter++] = b;
            c->checksum += b;
            if (c->byte_counter >= c->cmd_length)
                c->state = COMMO_SM_RXD_CHECKSUM;
        }
        break;

    case COMMO_SM_RXD_CHECKSUM:
        if (commo_get_data()) break;
        {
            uint8_t rx = get_rxd_data();
            /* One's complement: ~received == accumulated_sum means valid */
            if ((uint8_t)~rx == c->checksum) {
                /* Determine if the opcode is the same as the last accepted one */
                c->rx_status = (c->rx_buffer[0] == c->last_command)
                               ? COMMO_SAME_COMMAND
                               : COMMO_NEW_COMMAND;
                if (c->rx_status == COMMO_NEW_COMMAND)
                    c->last_command = c->rx_buffer[0];
            } else {
                c->rx_status = COMMO_CMD_ERROR;
            }
            c->report_cmd = 1;
            c->checksum   = 0;
            c->state      = COMMO_SM_IDLE;
        }
        break;

    case COMMO_SM_TXD_DATA:
        /* Wait for host to release bus before sending each byte */
        if (!commo_get_data()) break;
        transmit_txd(c->tx_buffer[c->byte_pointer]);
        c->checksum += c->tx_buffer[c->byte_pointer];
        c->byte_pointer++;
        if (--c->byte_counter == 0) {
            c->tx_req = 0;
            c->state  = c->tx_chk_req ? COMMO_SM_TXD_CHECKSUM
                                      : COMMO_SM_IDLE;
        }
        break;

    case COMMO_SM_TXD_CHECKSUM:
        if (!commo_get_data()) break;
        transmit_txd((uint8_t)~c->checksum);   /* one's complement of sum */
        c->tx_req     = 0;
        c->tx_chk_req = 0;
        c->checksum   = 0;
        c->state      = COMMO_SM_IDLE;
        break;

    case COMMO_SM_ERR_SEND:
        /* Count down before reporting the error to the Dispatcher.
         * This gives the host time to release the bus after a bad byte. */
        if (c->byte_counter > 0) {
            c->byte_counter--;
        } else {
            c->rx_status  = COMMO_CMD_ERROR;
            c->report_cmd = 1;
            c->state      = COMMO_SM_IDLE;
        }
        break;

    default:
        c->state = COMMO_SM_IDLE;
        break;
    }
}

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the COMMO serial interface hardware and state machine.
 *
 * Pin configuration:
 *   DIR  → output, driven high (bus in receive state at startup)
 *   CLK  → input, pull-up (host drives clock during receive; high = idle)
 *   DATA → input, pull-up (bidir; high = bus released / idle)
 */
void COMMO_INIT(void)
{
    memset(&s_commo, 0, sizeof(s_commo));
    s_commo.state = COMMO_SM_IDLE;

    gpio_init(PIN_COMMO_DIR);
    gpio_set_dir(PIN_COMMO_DIR, GPIO_OUT);
    gpio_put(PIN_COMMO_DIR, 1);

    gpio_init(PIN_COMMO_CLK);
    gpio_set_dir(PIN_COMMO_CLK, GPIO_IN);
    gpio_pull_up(PIN_COMMO_CLK);

    gpio_init(PIN_COMMO_DATA);
    gpio_set_dir(PIN_COMMO_DATA, GPIO_IN);
    gpio_pull_up(PIN_COMMO_DATA);
}

/**
 * @brief  Service the COMMO state machine — one step per main-loop iteration.
 *
 * Must be called once per loop from main().  Each call advances the
 * state machine by at most one state transition plus the I/O required
 * for that transition (get_rxd_data / transmit_txd are blocking but brief).
 */
void COMMO_INTERFACE(void)
{
    commo_step(&s_commo);
}

/**
 * @brief  Poll for a newly received command.
 *
 * Returns COMMO_NO_COMMAND until a complete packet has been received and
 * either validated or rejected.  After returning a non-zero value the
 * result persists until FREE_CMD_BUFFER() is called.
 *
 * @return COMMO_NEW_COMMAND  — new valid opcode received.
 *         COMMO_SAME_COMMAND — same opcode as last command repeated.
 *         COMMO_CMD_ERROR    — checksum or protocol error.
 *         COMMO_NO_COMMAND   — nothing received yet.
 */
uint8_t NEW_CMD_RECEIVED(void)
{
    if (!s_commo.report_cmd) return COMMO_NO_COMMAND;
    return s_commo.rx_status;
}

/**
 * @brief  Return byte @p idx from the receive buffer.
 *
 * Buffer layout:
 *   [0] = opcode
 *   [1] = param1 (if present)
 *   [2] = param2 (if present)
 *   …
 *
 * @param  idx  Buffer index (0–11).
 * @return Byte value; 0 if idx is out of range.
 */
uint8_t GET_BUFFER(uint8_t idx)
{
    if (idx >= (uint8_t)sizeof(s_commo.rx_buffer)) return 0;
    return s_commo.rx_buffer[idx];
}

/**
 * @brief  Queue a packet for transmission over the COMMO bus.
 *
 * The packet is copied into the internal TX buffer.  Transmission begins
 * on the next COMMO_INTERFACE() call when the state machine reaches
 * TXD_DATA and the bus is free.
 *
 * @param  mode    SEND_STRING_COMPLETE to append a checksum byte after the
 *                 data; SEND_STRING_APPEND to send data only.
 * @param  data    Pointer to the bytes to transmit.
 * @param  length  Number of bytes (max 16).
 * @return COMMO_TRUE if accepted; COMMO_FALSE if a transmit is already
 *         pending or the packet is too long.
 */
uint8_t SEND_STRING(uint8_t mode, uint8_t *data, uint8_t length)
{
    if (s_commo.tx_req)                               return COMMO_FALSE;
    if (length > (uint8_t)sizeof(s_commo.tx_buffer))  return COMMO_FALSE;

    memcpy(s_commo.tx_buffer, data, length);
    s_commo.byte_counter = length;
    s_commo.tx_length    = length;
    s_commo.tx_chk_req   = (mode == SEND_STRING_COMPLETE) ? 1u : 0u;
    s_commo.tx_req       = 1;

    return COMMO_TRUE;
}

/**
 * @brief  Poll whether the current transmit has completed.
 *
 * @return COMMO_BUSY if still transmitting (TXD_DATA or TXD_CHECKSUM state).
 *         COMMO_READY_WITHOUT_ERROR when idle.
 */
uint8_t SEND_STRING_READY(void)
{
    if (s_commo.state == COMMO_SM_TXD_DATA ||
        s_commo.state == COMMO_SM_TXD_CHECKSUM)
        return COMMO_BUSY;

    return COMMO_READY_WITHOUT_ERROR;
}

/**
 * @brief  Release the receive buffer after the command has been processed.
 *
 * Clears the report_cmd flag so NEW_CMD_RECEIVED() returns COMMO_NO_COMMAND
 * again.  The Dispatcher calls this once the command handler has accepted
 * the current packet.
 *
 * @return COMMO_TRUE always.
 */
uint8_t FREE_CMD_BUFFER(void)
{
    s_commo.report_cmd   = 0;
    s_commo.cmd_buf_free = 1;
    return COMMO_TRUE;
}
