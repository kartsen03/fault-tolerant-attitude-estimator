/* Telemetry formatting without stdio: fixed buffer, bounded loops, integer
 * arithmetic only (printf-family functions pull in large, stack-hungry code
 * and float formatting on a CPU without an FPU). */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app.h"

#define TLM_RAD_TO_CENTIDEG (5729.57795130823f) /* 18000 / pi */
#define TLM_CENTIDEG_LIMIT (100000.0f)
#define TLM_MAX_TOKEN (16u)

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    bool overflow;
} tlm_writer_t;

/* Always leaves room for the NUL terminator. */
static void put_char(tlm_writer_t *w, char c)
{
    if ((w->len + 1u) < w->cap) {
        w->buf[w->len] = c;
        w->len += 1u;
    } else {
        w->overflow = true;
    }
}

static void put_str(tlm_writer_t *w, const char *s)
{
    for (size_t i = 0u; (i < TLM_MAX_TOKEN) && (s[i] != '\0'); ++i) {
        put_char(w, s[i]);
    }
}

static void put_u32(tlm_writer_t *w, uint32_t value)
{
    char digits[10];
    uint32_t v = value;
    uint32_t n = 0u;
    do {
        digits[n] = (char)('0' + (int32_t)(v % 10u));
        v /= 10u;
        n += 1u;
    } while ((v != 0u) && (n < 10u));
    while (n > 0u) {
        n -= 1u;
        put_char(w, digits[n]);
    }
}

/* Centi-units as a decimal with two places: -512 -> "-5.12". */
static void put_centi(tlm_writer_t *w, int32_t centi)
{
    const uint32_t mag = (centi < 0) ? (uint32_t)(-(int64_t)centi) : (uint32_t)centi;
    if (centi < 0) {
        put_char(w, '-');
    }
    put_u32(w, mag / 100u);
    put_char(w, '.');
    put_char(w, (char)('0' + (int32_t)((mag / 10u) % 10u)));
    put_char(w, (char)('0' + (int32_t)(mag % 10u)));
}

static char hex_digit(uint32_t nibble)
{
    static const char k_hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                   '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    return k_hex[nibble & 0xFu];
}

static void put_hex(tlm_writer_t *w, uint32_t value, uint32_t digits)
{
    for (uint32_t i = digits; i > 0u; --i) {
        put_char(w, hex_digit(value >> ((i - 1u) * 4u)));
    }
}

static int32_t rad_to_centideg(float rad)
{
    float cd = rad * TLM_RAD_TO_CENTIDEG;
    /* Written so NaN also fails the range test and maps to 0. */
    if (!((cd > -TLM_CENTIDEG_LIMIT) && (cd < TLM_CENTIDEG_LIMIT))) {
        cd = 0.0f;
    }
    return (int32_t)((cd >= 0.0f) ? (cd + 0.5f) : (cd - 0.5f));
}

const char *app_mode_name(att_mode_t mode)
{
    const char *name;
    switch (mode) {
    case ATT_MODE_INIT:
        name = "INIT";
        break;
    case ATT_MODE_NOMINAL:
        name = "NOM";
        break;
    case ATT_MODE_GYRO_ONLY:
        name = "GYRO";
        break;
    case ATT_MODE_ACCEL_ONLY:
        name = "ACC";
        break;
    case ATT_MODE_HOLD:
        name = "HOLD";
        break;
    case ATT_MODE_FAILED:
        name = "FAIL";
        break;
    default:
        name = "?";
        break;
    }
    return name;
}

const char *app_health_name(att_health_t health)
{
    const char *name;
    switch (health) {
    case ATT_HEALTH_OK:
        name = "OK";
        break;
    case ATT_HEALTH_DEGRADED:
        name = "DEG";
        break;
    case ATT_HEALTH_FAILED:
        name = "FAIL";
        break;
    default:
        name = "?";
        break;
    }
    return name;
}

size_t app_format_telemetry(const app_output_t *out, char *buf, size_t len)
{
    size_t result = 0u;

    if ((out != NULL) && (buf != NULL) && (len > 0u)) {
        tlm_writer_t w = {buf, len, 0u, false};
        uint32_t checksum = 0u;

        put_str(&w, "$ATT,");
        put_u32(&w, out->seq);
        put_char(&w, ',');
        put_u32(&w, out->t_us / 1000u);
        put_char(&w, ',');
        put_centi(&w, rad_to_centideg(out->est.attitude.roll_rad));
        put_char(&w, ',');
        put_centi(&w, rad_to_centideg(out->est.attitude.pitch_rad));
        put_char(&w, ',');
        put_char(&w, out->est.valid ? '1' : '0');
        put_char(&w, ',');
        put_str(&w, app_mode_name(out->est.mode));
        put_char(&w, ',');
        put_str(&w, app_health_name(out->est.health));
        put_char(&w, ',');
        put_hex(&w, out->est.faults, 4u);
        put_char(&w, ',');
        put_u32(&w, out->overruns);
        put_char(&w, ',');
        put_u32(&w, out->read_errors);

        for (size_t i = 1u; i < w.len; ++i) { /* skip '$' */
            checksum ^= (uint32_t)(uint8_t)w.buf[i];
        }
        put_char(&w, '*');
        put_hex(&w, checksum, 2u);
        put_char(&w, '\r');
        put_char(&w, '\n');

        buf[w.len] = '\0'; /* put_char always leaves room for it */
        result = w.overflow ? 0u : w.len;
    }
    return result;
}
