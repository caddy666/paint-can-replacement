/**
 * @file  maths.c
 * @brief BCD ↔ hex conversion, CD time arithmetic, and track-count estimation.
 *
 * All functions are pure (no side-effects beyond their output arguments)
 * and operate on cd_time_t structs or plain uint8_t values.
 *
 * ── CD time base ──────────────────────────────────────────────────────────
 *
 * CD-DA absolute disc time is encoded in a mixed-radix base:
 *   minutes : seconds : frames   (MM:SS:FF)
 *   range   : 00–99  : 00–59  : 00–74
 *
 * This is NOT standard clock time — a "frame" is 1/75 of a second (one
 * CD sector = 2352 bytes at 150 kB/s × 1× speed).  The time_base carries
 * at FF=74 → SS+1 (not at 99 like decimal), and at SS=59 → MM+1.
 *
 * ── Track-count estimation ─────────────────────────────────────────────────
 *
 * tracks_calc() estimates the absolute radial track number for a given disc
 * time using the spiral geometry of a standard 120 mm CD:
 *
 *   Inner radius ≈ 25 mm, outer radius ≈ 58 mm, track pitch ≈ 1.6 µm.
 *   Total tracks ≈ (58-25)mm / 0.0016mm ≈ 20,625 turns.
 *
 * The relationship between playing time T (absolute frame count) and radius r
 * follows: r² ≈ r_inner² + (track_pitch / π) · T
 *
 * Translated to integer arithmetic using empirically derived constants
 * A = 0xBB3D (47933) and B = 0x671F>>3 (3299):
 *
 *   track_number ≈ sqrt(A · T + B)
 *
 * The result is scaled by 16 (i.e. the raw integer sqrt is 16× the actual
 * groove count).  calc_tracks() divides by 16 (>>4) before returning.
 */

#include <stdint.h>
#include "defs.h"

/* =========================================================================
 * BCD helpers
 * ====================================================================== */

/**
 * @brief  Convert a single BCD-encoded byte to its decimal (hex) value.
 *
 * BCD format: upper nibble = tens digit, lower nibble = units digit.
 * Example: 0x59 → (5 × 10) + 9 = 59.
 *
 * @param  bcd  BCD byte (e.g. 0x59 for 59 seconds).
 * @return Equivalent decimal value as a plain integer (e.g. 59).
 */
uint8_t bcd_to_hex(uint8_t bcd)
{
    return ((bcd >> 4) * 10u) + (bcd & 0x0Fu);
}

/**
 * @brief  Convert a decimal value to BCD encoding.
 *
 * Example: 59 → 0x59.  Only valid for inputs 0–99.
 *
 * @param  hex  Decimal value 0–99.
 * @return BCD-encoded byte.
 */
uint8_t hex_to_bcd(uint8_t hex)
{
    uint8_t tens = hex / 10u;
    return (uint8_t)((tens << 4) | (hex - tens * 10u));
}

/**
 * @brief  Convert a cd_time_t in BCD to a cd_time_t in hex in-place.
 *
 * Converts all three fields (min, sec, frm) from BCD to decimal.
 * Used when re-encoding times from the COMMO buffer (which arrives in BCD)
 * into the internal hex representation.
 *
 * @param  src  BCD-encoded time.
 * @param  dst  Output hex-encoded time (may alias src).
 */
void bcd_to_hex_time(const cd_time_t *src, cd_time_t *dst)
{
    dst->min = bcd_to_hex(src->min);
    dst->sec = bcd_to_hex(src->sec);
    dst->frm = bcd_to_hex(src->frm);
}

/* =========================================================================
 * Time comparison
 *
 * Compares two cd_time_t values lexicographically: min first, then sec,
 * then frm.  Returns the relationship of a relative to b.
 * ====================================================================== */

/**
 * @brief  Compare two CD absolute times.
 * @return SMALLER (0) if a < b, EQUAL (1) if a == b, BIGGER (2) if a > b.
 */
uint8_t compare_time(const cd_time_t *a, const cd_time_t *b)
{
    if (a->min < b->min) return SMALLER;
    if (a->min > b->min) return BIGGER;
    if (a->sec < b->sec) return SMALLER;
    if (a->sec > b->sec) return BIGGER;
    if (a->frm < b->frm) return SMALLER;
    if (a->frm > b->frm) return BIGGER;
    return EQUAL;
}

/* =========================================================================
 * Time arithmetic
 *
 * add_time and subtract_time implement mixed-radix arithmetic using the
 * CD time base: 75 frames per second, 60 seconds per minute.
 *
 * The carry / borrow propagation is explicit because the frame radix (75)
 * is not a power of two — the compiler cannot generate it from shifts.
 * ====================================================================== */

/**
 * @brief  Add two CD times: r = a + b.
 *
 * Frame carry at 75, second carry at 60.  Result stored in r (may alias
 * a or b provided the computation uses temporaries — which it does).
 *
 * @param  a, b  Operands (hex, not BCD).
 * @param  r     Result (may alias a or b).
 */
void add_time(const cd_time_t *a, const cd_time_t *b, cd_time_t *r)
{
    uint32_t frm = (uint32_t)a->frm + b->frm;
    uint32_t sec = (uint32_t)a->sec + b->sec;
    uint32_t min = (uint32_t)a->min + b->min;

    sec += frm / 75u;
    frm %= 75u;
    min += sec / 60u;
    sec %= 60u;

    r->min = (uint8_t)min;
    r->sec = (uint8_t)sec;
    r->frm = (uint8_t)frm;
}

/**
 * @brief  Subtract two CD times: r = a - b.
 *
 * Assumes a >= b; result is undefined if a < b (caller must check with
 * compare_time first).  Borrow propagates downward: frame borrows from
 * seconds at 75, seconds borrow from minutes at 60.
 *
 * @param  a, b  Operands (hex, not BCD); a must be >= b.
 * @param  r     Result.
 */
void subtract_time(const cd_time_t *a, const cd_time_t *b, cd_time_t *r)
{
    int frm = (int)a->frm - (int)b->frm;
    int sec = (int)a->sec - (int)b->sec;
    int min = (int)a->min - (int)b->min;

    if (frm < 0) { frm += 75; sec--; }
    if (sec < 0) { sec += 60; min--; }

    r->min = (uint8_t)min;
    r->sec = (uint8_t)sec;
    r->frm = (uint8_t)frm;
}

/**
 * @brief  Convert a cd_time_t to a total frame count from disc start.
 *
 * Used internally for disc geometry calculations.
 * Formula: T = (min × 60 + sec) × 75 + frm
 *
 * @param  t  Disc time (hex).
 * @return Total number of CD frames from disc start.
 */
static uint32_t convert_time(const cd_time_t *t)
{
    return ((uint32_t)t->min * 60u + t->sec) * 75u + t->frm;
}

/* =========================================================================
 * Integer square root — binary digit-by-digit algorithm
 *
 * Computes floor(sqrt(x)) using a top-down bit-scan.  Each iteration tests
 * whether the current bit position contributes to the result by checking if
 * (partial_result + bit) <= remaining_radicand.
 *
 * This is O(16) iterations for a 32-bit input (one per even bit position
 * from bit 30 down to bit 0) with no division or floating-point.
 * ====================================================================== */

static uint32_t isqrt(uint32_t x)
{
    if (x == 0) return 0;

    uint32_t res = 0;
    uint32_t bit = 1u << 30;   /* highest even power of 2 not exceeding 2^31 */

    while (bit > x)  bit >>= 2;

    while (bit) {
        uint32_t tmp = res + bit;
        res >>= 1;
        if (x >= tmp) { x -= tmp; res += bit; }
        bit >>= 2;
    }
    return res;
}

/**
 * @brief  Estimate the absolute track (groove) number for a disc time.
 *
 * Uses the empirically-derived spiral geometry formula:
 *   track ≈ sqrt(A × T + B)
 *
 * where A = 47933 (0xBB3D) and B = 3299 (0x671F >> 3) are constants
 * derived from standard 120 mm CD geometry.  The result is scaled ×16
 * relative to actual groove counts; the caller divides by 16 (>>4).
 *
 * @param  t  Disc time (hex).
 * @return Scaled track-position estimate.
 */
static uint32_t tracks_calc(const cd_time_t *t)
{
    const uint32_t A = 0xBB3Du;           /* spiral geometry constant */
    const uint32_t B = (0x671Fu >> 3u);   /* inner-radius offset term */
    uint32_t T = convert_time(t);
    return isqrt(A * T + B);
}

/**
 * @brief  Estimate the number of grooves (tracks) between two disc times.
 *
 * Computes the radial distance between t1 and t2 in groove units, using
 * the spiral geometry model.  The sign encodes direction:
 *   positive — t2 is further out than t1 (laser must move outward)
 *   negative — t2 is inside t1 (laser must move inward)
 *   zero     — same position
 *
 * The raw sqrt difference is divided by 16 (>>4) to convert from the
 * over-scaled integer units to actual groove counts.
 *
 * Used by play.c's jump_time() to calculate the size of each step in the
 * binary-convergence seek algorithm.
 *
 * @param  t1  Starting disc time.
 * @param  t2  Target disc time.
 * @return Estimated groove count (positive = outward from t1 to t2).
 */
int calc_tracks(const cd_time_t *t1, const cd_time_t *t2)
{
    uint8_t cmp = compare_time(t1, t2);
    if (cmp == EQUAL) return 0;

    const cd_time_t *big   = (cmp == BIGGER) ? t1 : t2;
    const cd_time_t *small = (cmp == BIGGER) ? t2 : t1;

    uint32_t bv = tracks_calc(big);
    uint32_t sv = tracks_calc(small);
    int result  = (int)((bv - sv) >> 4);   /* divide by 16 to get groove count */

    /* If t1 > t2 then t2 is inside t1: return negative (inward) */
    return (cmp == BIGGER) ? -result : result;
}
