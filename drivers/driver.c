/**
 * @file  driver.c
 * @brief Hardware driver for the CD32 Pico2 firmware.
 *
 * Implements bit-banged serial communication to three hardware ICs:
 *
 *   CXD2500BQ — Sony CD signal processor (DSP).  Controls the spindle
 *               motor CLV servo, the audio output DAC, and the EFM decoder.
 *               3-wire write-only bus: UCL (clock), UDAT (data), ULAT (latch).
 *               Protocol: 3 dummy clock pulses, then 8 bits LSB-first,
 *               then a latch pulse on ULAT.
 *
 *   DSIC2     — Sony CXA1372 focus/radial/sledge servo controller.
 *               3-wire bidirectional bus: SICL (clock), SIDA (data, bidir),
 *               SILD (latch).  Write: 8 bits MSB-first, latch on SILD.
 *               Read: master releases SIDA to GPIO_IN, clocks 8 bits.
 *
 *   Q-channel — Serial subcode data clocked from the CXD2500BQ output pins.
 *               QCL (clock, driven by us), QDA (data, input from CXD2500).
 *               Triggered by the SCOR falling-edge interrupt in timer.c.
 *
 * All GPIO pin assignments are in gpio_map.h.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <string.h>

#include "driver.h"
#include "gpio_map.h"
#include "serv_def.h"
#include "timer.h"

/* =========================================================================
 * Shared data (also exposed via driver.h externs to other modules)
 * ====================================================================== */

uint8_t Q_buffer[10]    = {0};  /**< 10-byte Q-channel subcode frame (raw from disc) */
uint8_t audio_cntrl     = 0;    /**< Shadow of CXD2500 audio-control register        */
uint8_t peak_level_low  = 0;    /**< Lowest peak level sampled since last read       */
uint8_t peak_level_high = 0;    /**< Highest peak level sampled since last read      */
uint8_t hex_abs_min     = 0;    /**< Absolute minute position (hex); drives brake table */
uint8_t simulation_timer = 0;   /**< Down-counter for simulated motor-start events   */

/* n1_speed: 1 = single speed (N=1), 0 = double speed (N=2).
 * Shared with servo.c which also needs to know the current speed for gain
 * and brake-table calculations. */
int n1_speed = 1;

/* mute_pin: local shadow of the mute output state (not wired to a real pin
 * on this hardware revision; retained for compatibility). */
static int mute_pin = 0;

/* =========================================================================
 * GPIO helpers — inline wrappers keep call sites readable
 *
 * For the CXD2500BQ 'U' prefix stands for the 'User' control bus (UCL,
 * UDAT, ULAT) as named in the Sony datasheet.  For the DSIC2, 'SI' means
 * 'Serial Interface'.
 * ====================================================================== */

/* CXD2500BQ (UCL / UDAT / ULAT) */
static inline void UCL(int v)  { gpio_put(PIN_CXD_CLK,  (uint32_t)v); }
static inline void UDAT(int v) { gpio_put(PIN_CXD_DATA, (uint32_t)v); }
static inline void ULAT(int v) { gpio_put(PIN_CXD_LAT,  (uint32_t)v); }

/* DSIC2 (SICL / SIDA / SILD) */
static inline void SICL(int v) { gpio_put(PIN_DSIC_CLK,  (uint32_t)v); }
static inline void SILD(int v) { gpio_put(PIN_DSIC_LAT,  (uint32_t)v); }
static inline void SIDA_OUT(int v) { gpio_put(PIN_DSIC_DATA, (uint32_t)v); }
static inline int  SIDA_IN(void)   { return (int)gpio_get(PIN_DSIC_DATA); }

/* Q-channel */
static inline void QCL(int v) { gpio_put(PIN_QCL, (uint32_t)v); }
static inline int  QDA(void)  { return (int)gpio_get(PIN_QDA); }

/* Sense inputs */
static inline int HF_PRESENT(void)   { return (int)gpio_get(PIN_HF_DET); }
static inline int DOOR_SWITCH(void)  { return (int)gpio_get(PIN_DOOR);   }

/* =========================================================================
 * driver_init — configure all GPIO pins
 * ====================================================================== */

void driver_init(void)
{
    /* CXD2500BQ: all three control lines are outputs driven by us */
    gpio_init(PIN_CXD_CLK);  gpio_set_dir(PIN_CXD_CLK,  GPIO_OUT);
    gpio_init(PIN_CXD_DATA); gpio_set_dir(PIN_CXD_DATA, GPIO_OUT);
    gpio_init(PIN_CXD_LAT);  gpio_set_dir(PIN_CXD_LAT,  GPIO_OUT);

    /* DSIC2: clock and latch are always outputs; SIDA starts as output
     * (write mode) and is switched to input by rd_dsic2() as needed. */
    gpio_init(PIN_DSIC_CLK);  gpio_set_dir(PIN_DSIC_CLK,  GPIO_OUT);
    gpio_init(PIN_DSIC_DATA); gpio_set_dir(PIN_DSIC_DATA, GPIO_OUT);
    gpio_init(PIN_DSIC_LAT);  gpio_set_dir(PIN_DSIC_LAT,  GPIO_OUT);

    /* Q-channel: we drive QCL; QDA comes from the CXD2500 output */
    gpio_init(PIN_QCL); gpio_set_dir(PIN_QCL, GPIO_OUT);
    gpio_init(PIN_QDA); gpio_set_dir(PIN_QDA, GPIO_IN);
    gpio_pull_up(PIN_QDA);

    /* Disc/door sense: both are open-collector switches pulled high */
    gpio_init(PIN_HF_DET); gpio_set_dir(PIN_HF_DET, GPIO_IN); gpio_pull_up(PIN_HF_DET);
    gpio_init(PIN_DOOR);   gpio_set_dir(PIN_DOOR,   GPIO_IN); gpio_pull_up(PIN_DOOR);

    /* SCOR interrupt input: configured as input here; the ISR is installed
     * later by timer_init() after this function returns. */
    gpio_init(PIN_SCOR); gpio_set_dir(PIN_SCOR, GPIO_IN); gpio_pull_up(PIN_SCOR);

    /* Status LED: driven low (off) until boot completes */
    gpio_init(PIN_LED_STATUS); gpio_set_dir(PIN_LED_STATUS, GPIO_OUT);
    gpio_put(PIN_LED_STATUS, 0);

    /* Leave all serial buses in their idle (de-asserted) state so that
     * the ICs do not latch spurious data during initialisation. */
    SILD(1); SIDA_OUT(1); SICL(1);
    QCL(1); UDAT(1); UCL(1); ULAT(1);
}

/* =========================================================================
 * reset_dsic2_cd6 — hardware power-on reset sequence
 *
 * Reproduces the original firmware's timed reset sequence for both ICs:
 *   80 ms  — hold reset (DSIC2 requires this before accepting commands)
 *   20 ms  — settle (allow internal oscillators to stabilise)
 *  500 µs  — strobe
 *   10 ms  — stabilise (DSIC2 datasheet setup time)
 *
 * After the sequence all bus lines are returned to idle and the SCOR edge
 * flag is cleared to discard any glitch edges that occurred during reset.
 * ====================================================================== */

void reset_dsic2_cd6(void)
{
    delay_byte = 160; delay();   /* 160 × 500 µs = 80 ms  */
    delay_byte = 40;  delay();   /*  40 × 500 µs = 20 ms  */
    delay_byte = 1;   delay();   /*   1 × 500 µs = 500 µs */
    delay_byte = 20;  delay();   /*  20 × 500 µs = 10 ms  */

    SILD(1); SIDA_OUT(1); SICL(1);
    QCL(1); UDAT(1); UCL(1); ULAT(1);

    extern volatile uint8_t scor_edge;
    scor_edge = 0;   /* discard SCOR glitches that may have fired during reset */
}

/* =========================================================================
 * cxd2500_wr — send an 8-bit command word to the CXD2500BQ
 *
 * The CXD2500BQ uses an unusual protocol: three dummy clock pulses precede
 * the data to synchronise the internal shift register.  Data is sent LSB
 * first (opposite to the DSIC2).  After all 8 bits a latch pulse on ULAT
 * transfers the shift register to the internal command register.
 *
 * Timing is implicit (GPIO operations are fast enough on the Pico).
 * ====================================================================== */

void cxd2500_wr(uint8_t data)
{
    UDAT(0);   /* assert data low before clocking to signal start */

    /* 3 dummy clock pulses — required by the CXD2500 internal protocol */
    for (int i = 0; i < 3; i++) { UCL(0); UCL(1); }

    /* 8 data bits, LSB first (bit 0 sent first) */
    for (int i = 0; i < 8; i++) {
        UCL(0);
        UDAT(data & 1);    /* place bit on data line while clock is low  */
        UCL(1);            /* rising edge — CXD2500 latches bit          */
        data >>= 1;
    }

    /* Latch pulse — moves the 8-bit shift register into the command register */
    ULAT(0); ULAT(1);
    UDAT(1);   /* release data line to idle-high */
}

/* =========================================================================
 * audio_cxd2500 — send audio control word with opcode 0x0A
 *
 * This encodes a 6-bit audio-control field followed by a fixed 4-bit
 * opcode (0x0A) that tells the CXD2500 to update its audio register.
 *
 * The original 8051 source used two RRC (rotate-right-through-carry)
 * instructions to pre-rotate the byte before shifting.  We replicate that
 * rotation arithmetically: RRC RRC ≡ >>2 with the two displaced LSBs
 * wrapped into the two MSBs (circular right-shift by 2 positions).
 * ====================================================================== */

void audio_cxd2500(uint8_t data)
{
    UDAT(0);
    for (int i = 0; i < 3; i++) { UCL(0); UCL(1); }   /* 3 dummy pulses */

    /* Circular right-shift by 2: replicate 8051 RRC RRC */
    data = (uint8_t)((data >> 2) | (data << 6));

    /* 6 data bits (the audio-control field) */
    for (int i = 0; i < 6; i++) {
        UCL(0);
        UDAT(data & 1);
        UCL(1);
        data >>= 1;
    }

    /* Fixed 4-bit opcode 0x0A: identifies this as an audio-control write */
    uint8_t cmd = 0x0A;
    for (int i = 0; i < 4; i++) {
        UCL(0);
        UDAT(cmd & 1);
        UCL(1);
        cmd >>= 1;
    }

    ULAT(0); ULAT(1);
    UDAT(1);
}

/* =========================================================================
 * wr_dsic2 — shift 8 bits MSB-first into the DSIC2 IC
 *
 * The DSIC2 latches its command register on the rising edge of SILD.
 * Data is presented on SIDA while SICL is low and clocked on the rising
 * edge (opposite bit order to CXD2500).  A 150 µs settle time after the
 * latch pulse is required by the DSIC2 datasheet for the servo loops to
 * respond before the next command.
 * ====================================================================== */

void wr_dsic2(uint8_t data)
{
    gpio_set_dir(PIN_DSIC_DATA, GPIO_OUT);   /* ensure we own the data line */

    for (int i = 0; i < 8; i++) {
        SICL(0);
        SIDA_OUT((data >> 7) & 1);   /* MSB first */
        SICL(1);                     /* rising edge — DSIC2 shifts in the bit */
        data <<= 1;
    }

    SIDA_OUT(1);          /* release data line before latch */
    SILD(0); SILD(1);     /* latch pulse: transfers shift register to command reg */

    sleep_us(150);   /* ~150 µs settle time from original firmware specification */
}

/* =========================================================================
 * rd_dsic2 — read 8 bits MSB-first from the DSIC2 IC
 *
 * To initiate a read the master releases SIDA to GPIO_IN and then asserts
 * both SICL and SILD low to signal a read cycle to the DSIC2.  The DSIC2
 * then drives SIDA with the 8 status bits, MSB first.
 *
 * After reading, SIDA is returned to GPIO_OUT (write mode) ready for the
 * next wr_dsic2() call.
 * ====================================================================== */

uint8_t rd_dsic2(void)
{
    uint8_t value = 0;

    gpio_set_dir(PIN_DSIC_DATA, GPIO_IN);
    gpio_pull_up(PIN_DSIC_DATA);   /* avoid floating input during read */

    /* Assert both SICL and SILD low to signal read cycle start */
    SICL(0);
    SILD(0);

    /* Clock in 8 bits, MSB first */
    for (int i = 0; i < 8; i++) {
        SICL(0);
        value <<= 1;
        value |= (uint8_t)(SIDA_IN() ? 1u : 0u);   /* sample on SICL low */
        SICL(1);                                     /* rising edge advances DSIC2 pointer */
    }

    SILD(1);   /* de-assert latch */

    /* Return data pin to output mode so wr_dsic2() can drive it immediately */
    gpio_set_dir(PIN_DSIC_DATA, GPIO_OUT);

    sleep_us(150);
    return value;
}

/* =========================================================================
 * cd6_read_subcode — capture one 10-byte Q-channel frame
 *
 * The CXD2500BQ signals a new subcode frame by asserting the SCOR output
 * (falling edge on PIN_SCOR).  The ISR in timer.c sets scor_edge = 1.
 *
 * We clock 10 bytes (80 bits) out of QDA by toggling QCL.  The CXD2500
 * shifts out bits MSB-first on each rising edge of QCL.
 *
 * The function returns 0 immediately if SCOR has not fired, or if QDA is
 * not asserted (QDA low means the CXD2500 has no valid frame ready).
 * ====================================================================== */

int cd6_read_subcode(void)
{
    extern volatile uint8_t scor_edge;

    if (!scor_edge) return 0;    /* no new subcode clock edge yet */
    scor_edge = 0;               /* consume the edge */

    if (!QDA()) return 0;        /* QDA not asserted — frame not valid */

    peak_level_low  = 0;
    peak_level_high = 0;

    /* Shift in 10 bytes (80 bits) into Q_buffer[0..9] */
    for (int b = 0; b < 10; b++) {
        uint8_t v = 0;
        for (int bit = 0; bit < 8; bit++) {
            QCL(0); QCL(1);     /* clock pulse — CXD2500 advances to next bit */
            /* Accumulate MSB-first: shift received bit into position 7 */
            v = (uint8_t)((v >> 1) | ((uint8_t)QDA() << 7));
        }
        Q_buffer[b] = v;
    }

    return 1;
}

/* =========================================================================
 * hf_present — test whether the HF (high-frequency) signal is present
 *
 * The HF detector output is active-low: it goes low when the laser is
 * reading valid EFM data from the disc surface.  We sample it five times
 * to debounce the signal — if any sample is active we report disc present.
 * ====================================================================== */

int hf_present(void)
{
    for (int i = 0; i < 5; i++) {
        if (!HF_PRESENT()) return 1;   /* active low: 0 means HF present */
    }
    return 0;
}

int door_closed(void)
{
    return DOOR_SWITCH();   /* switch pulls high when door is closed */
}

/* =========================================================================
 * SCOR counter helpers
 *
 * The SCOR counter provides frame-accurate synchronisation during seeks.
 * init_scor_counter(n) loads n-1 (because the ISR decrements on the first
 * edge and we want to count n more edges).  zero_scor_counter() tests for
 * expiry.
 * ====================================================================== */

void init_scor_counter(uint8_t count)
{
    extern volatile uint8_t scor_counter;
    scor_counter = count;
    if (scor_counter > 0) scor_counter--;   /* pre-decrement: first edge is "free" */
}

int zero_scor_counter(void)
{
    extern volatile uint8_t scor_counter;
    return scor_counter == 0;
}

void increment_scor_counter(void)
{
    extern volatile uint8_t scor_counter;
    scor_counter++;
}

void enable_scor_counter(void)
{
    /* On the 8051 this function set EA=1 and EX0=1 to unmask external
     * interrupt 0.  On the Pico the GPIO interrupt is enabled permanently
     * inside timer_init(); this function is a no-op retained for the
     * original call-site in main.c. */
}

/* =========================================================================
 * status_cd6 — poll a CXD2500 status flag by type
 *
 * On the original hardware several CXD2500BQ status bits were available on
 * dedicated output pins.  This Pico port has no such pins wired, so we
 * simulate the motor-speed flags using simulation_timer (a down-counter
 * that is loaded when motor-start commands are issued).
 *
 * SUBCODE_READY is handled via the real SCOR interrupt flag.
 * ====================================================================== */

int status_cd6(uint8_t status_type)
{
    switch (status_type) {
    case MOTOR_OVERFLOW:
        return 0;               /* never overflows in simulation */

    case MOT_STRT_2:
        return 0;               /* second speed check not modelled */

    case MOT_STRT_1:
    case MOT_STOP:
        /* simulation_timer is loaded by cd6_wr(MOT_STRTM2_ACTIVE) with a
         * count proportional to the expected spin-up time.  When it reaches
         * zero we report motor-at-speed (MOT_STRT_1) or motor-stopped
         * (MOT_STOP). */
        return (simulation_timer == 0);

    case SUBCODE_READY:
        /* Real SCOR flag from the ISR */
        return (int)scor_edge;

    default:
        return 0;
    }
}

/* =========================================================================
 * Brake-table helper
 *
 * When the drive stops after CLV playback it needs to apply an active brake
 * of controlled duration to prevent the disc from coasting and stressing the
 * sledge.  The required brake time depends on disc radius (hex_abs_min) and
 * speed.
 *
 * hex_abs_min is updated by subcode.c each time a new Q-channel frame is
 * decoded.  It holds the absolute minute field (in hex), which increases
 * linearly with disc radius.  We divide it by 4 to index a 20-entry table.
 *
 * At double speed the brake time is doubled to compensate for higher
 * angular momentum.
 * ====================================================================== */

static uint8_t time_to_brake(void)
{
    /* Empirically derived brake-time table from the original firmware.
     * Entry 0 = innermost position (lead-in), entry 19 = near lead-out. */
    static const uint8_t brake_table[20] = {
        19,18,18,17,17,16,16,15,14,14,13,13,12,11,11,10,10,9,9,8
    };
    uint8_t idx = hex_abs_min >> 2;    /* divide by 4 to map minute→index */
    if (idx >= 20) idx = 19;           /* clamp to table bounds            */
    return n1_speed ? brake_table[idx] : (uint8_t)(brake_table[idx] * 2u);
}

/**
 * @brief  Return the radial zone (1=inner, 2=middle, 3=outer) for jump calc.
 *
 * The disc is divided into three zones based on the current absolute minute
 * position.  The zones are used in servo.c to select kick/brake durations
 * appropriate for the local disc geometry.
 */
uint8_t get_area(void)
{
    if (hex_abs_min < 16) return 1;   /* lead-in and first ~16 minutes   */
    if (hex_abs_min > 32) return 3;   /* outer 2/3 of the programme area */
    return 2;                          /* mid-disc                        */
}

/* =========================================================================
 * set_level_meter_mode — select audio level-meter display mode
 *
 * Updates the lower nibble of audio_cntrl (mode bits) without disturbing
 * the upper nibble (mute / attenuate bits).  The updated value is sent to
 * the CXD2500 by the next call to audio_cxd2500().
 * ====================================================================== */

void set_level_meter_mode(uint8_t mode)
{
    audio_cntrl &= 0xF0u;    /* clear existing mode bits */
    switch (mode) {
    case 0: audio_cntrl |= NORMAL_MODE; break;
    case 1: audio_cntrl |= LEVEL_MODE;  break;
    case 2: audio_cntrl |= PEAK_MODE;   break;
    default: break;
    }
}

/* =========================================================================
 * cd6_wr — high-level motor / audio command dispatcher
 *
 * Translates the symbolic constants from serv_def.h into actual CXD2500BQ
 * register writes.  This abstraction layer means the servo and play modules
 * do not need to know the raw register values.
 *
 * Motor commands (MOT_*):
 *   The CXD2500 motor-control register uses bits [7:5] for mode selection:
 *     0xE0 — off / free run
 *     0xE8 — start-mode 1 kick
 *     0xEA — brake (active or gentle depending on configuration)
 *     0xEE — rough servo (used during spin-up and jumps)
 *     0xE6 — play mode (closed-loop CLV)
 *
 * Speed / gain commands (0x9x, 0xCx):
 *   Written directly as raw CXD2500 register bytes.
 *
 * Audio commands (MUTE / FULL_SCALE / ATTENUATE):
 *   Modify the audio_cntrl shadow register and call audio_cxd2500().
 * ====================================================================== */

void cd6_wr(uint8_t mode)
{
    switch (mode) {

    case MOT_OFF_ACTIVE:
        cxd2500_wr(0xE0);          /* motor off / free-run */
        break;

    case MOT_BRM1_ACTIVE:
        cxd2500_wr(0xEA);          /* gentle brake */
        break;

    case MOT_BRM2_ACTIVE:
        /* Active timed brake: load simulation_timer so status_cd6(MOT_STOP)
         * fires after the appropriate stopping distance. */
        simulation_timer = time_to_brake();
        cxd2500_wr(0xEA);
        break;

    case MOT_STRTM1_ACTIVE:
        cxd2500_wr(0xE8);          /* kick mode — high current pulse to start */
        hex_abs_min = 0;           /* at inner stop; reset brake-table index  */
        break;

    case MOT_STRTM2_ACTIVE:
        /* Rough servo mode: simulation_timer counts down the expected spin-up
         * time (~600 ms / 8 ms per tick = 75 ticks). */
        simulation_timer = 600u / 8u;
        cxd2500_wr(0xEE);
        break;

    case MOT_JMPM_ACTIVE:
    case MOT_JMPM1_ACTIVE:
        cxd2500_wr(0xEE);          /* rough servo during long jump */
        break;

    case MOT_PLAYM_ACTIVE:
        cxd2500_wr(0xE6);          /* closed-loop CLV play mode */
        break;

    case SPEED_CONTROL_N1:
        n1_speed = 1;
        cxd2500_wr(0x99);          /* PLL speed-divider for 1× */
        break;

    case SPEED_CONTROL_N2:
        n1_speed = 0;
        cxd2500_wr(0x9D);          /* PLL speed-divider for 2× */
        break;

    case MOT_GAIN_8CM_N1:
    case MOT_GAIN_12CM_N1:
        cxd2500_wr(0xC1);
        break;

    case MOT_GAIN_8CM_N2:
    case MOT_GAIN_12CM_N2:
        cxd2500_wr(0xC6);
        break;

    case DAC_OUTPUT_MODE:
        cxd2500_wr(0x89);          /* 0x89 = CD-ROM DAC output; 0x81 = audio */
        break;

    case MOT_OUTPUT_MODE:
        cxd2500_wr(0xD0);
        break;

    case EBU_OUTPUT_MODE:
        /* EBU digital output is not connected on this hardware revision */
        break;

    case MUTE:
        /* Set bit 5 of audio_cntrl (mute bit) and send updated word */
        audio_cntrl |= 0x20u;
        audio_cxd2500(audio_cntrl);
        mute_pin = 1;
        break;

    case FULL_SCALE:
        /* Unmute: clear the mute bit only if it was set */
        if (audio_cntrl & 0x20u) {
            audio_cntrl &= 0xCFu;    /* clear bits 4 and 5 (mute + attenuate) */
            audio_cxd2500(audio_cntrl);
            mute_pin = 0;
        }
        break;

    case ATTENUATE:
        /* Set bit 4 of audio_cntrl (–6 dB attenuation), clear mute bit */
        audio_cntrl &= 0xCFu;
        audio_cntrl |= 0x10u;
        mute_pin = 0;
        audio_cxd2500(audio_cntrl);
        break;

    default:
        /* Pass through any raw register value not covered above */
        cxd2500_wr(mode);
        break;
    }
}
