/**
 * @file  gpio_map.h
 * @brief GPIO pin assignments for the CD32 Pico2 hardware.
 *
 * Adjust these defines to match your PCB layout.
 *
 * ── IC connections ────────────────────────────────────────────────────────
 *
 *   CXD2500BQ (Sony, referred to as "CD6"):
 *     CD signal processor — EFM decoder, CLV servo, audio DAC, subcode output.
 *     Write-only 3-wire SPI-like bus: UCL (clock), UDAT (data), ULAT (latch).
 *     Protocol: 3 dummy clocks → 8 bits LSB-first → latch pulse.
 *
 *   DSIC2 (Sony CXA1372 servo IC):
 *     Controls focus, spindle, radial (tracking), and sledge actuators.
 *     Bidirectional 3-wire bus: SICL (clock), SIDA (data, bidir), SILD (latch).
 *     GPIO direction toggled at runtime for read vs write.
 *
 *   QDA / QCL (CXD2500BQ subcode output):
 *     Q-channel serial data/clock pair driven by the CXD2500.  10 bytes
 *     (80 bits) are clocked out after each SCOR falling edge.
 *
 *   SCOR (subcode clock output, CXD2500BQ):
 *     Falling edge fires once per CD frame (75 Hz ≈ every 13.3 ms).
 *     Triggers the GPIO ISR in timer.c which sets scor_edge and decrements
 *     scor_counter.
 *
 *   COMMO bus (bidirectional serial to Amiga CD32 host chipset):
 *     DATA — bidir data; DIR controls which side drives it.
 *     CLK  — driven by the host during receive; by us during transmit.
 *     DIR  — direction output from this firmware (1=transmit, 0=receive).
 *
 * ── Pin allocation summary ────────────────────────────────────────────────
 *
 *   GPIO  2  CXD2500 UCL  (clock)
 *   GPIO  3  CXD2500 UDAT (data, output only)
 *   GPIO  4  CXD2500 ULAT (latch)
 *   GPIO  5  DSIC2 SICL   (clock)
 *   GPIO  6  DSIC2 SIDA   (data, bidir — direction toggled at runtime)
 *   GPIO  7  DSIC2 SILD   (latch)
 *   GPIO  8  QDA          (Q-channel data from CXD2500, input)
 *   GPIO  9  QCL          (Q-channel clock from CXD2500, input)
 *   GPIO 10  HF detector  (disc presence, active-high, input)
 *   GPIO 11  Door switch  (disc lid closed, input)
 *   GPIO 12  SCOR         (subcode frame clock, falling-edge interrupt)
 *   GPIO 13  COMMO DIR    (direction output: 1=we transmit)
 *   GPIO 14  COMMO DATA   (bidir, switched between input and output)
 *   GPIO 15  COMMO CLK    (clock, input-only from host)
 *   GPIO 25  Status LED   (onboard Pico LED, active-high)
 */

#pragma once

/* ── CXD2500BQ control (UCL / UDAT / ULAT) ─────────────────────────────── */
#define PIN_CXD_CLK    2   /**< UCL  — write-only serial clock              */
#define PIN_CXD_DATA   3   /**< UDAT — write-only serial data (LSB-first)   */
#define PIN_CXD_LAT    4   /**< ULAT — latch strobe (active pulse after 8b) */

/* ── DSIC2 servo IC (SICL / SIDA / SILD) ──────────────────────────────── */
#define PIN_DSIC_CLK   5   /**< SICL — serial clock                         */
#define PIN_DSIC_DATA  6   /**< SIDA — serial data (bidir, MSB-first)       */
#define PIN_DSIC_LAT   7   /**< SILD — latch strobe                         */

/* ── CXD2500BQ Q-channel subcode serial output ─────────────────────────── */
#define PIN_QDA        8   /**< Q-channel data output from CXD2500 (input)  */
#define PIN_QCL        9   /**< Q-channel clock output from CXD2500 (input) */

/* ── Disc / door sense ─────────────────────────────────────────────────── */
#define PIN_HF_DET    10   /**< HF detector: high = RF signal present = disc */
#define PIN_DOOR      11   /**< Door / lid switch: state TBD by hw design    */

/* ── SCOR subcode frame clock interrupt ─────────────────────────────────── */
#define PIN_SCOR      12   /**< Falling edge = new Q-channel frame ready (75 Hz) */

/* ── COMMO bus to Amiga CD32 host chipset ──────────────────────────────── */
#define PIN_COMMO_DIR  13   /**< Direction control output (1=we drive DATA)  */
#define PIN_COMMO_DATA 14   /**< Bidir data; driven by host or us per DIR    */
#define PIN_COMMO_CLK  15   /**< Clock input from host (we drive during TX)  */

/* ── Status LED ────────────────────────────────────────────────────────── */
#define PIN_LED_STATUS 25   /**< Onboard LED, active-high; set on boot done  */
