#include "drm_viterbi.h"
#include <string.h>

static const uint8_t taps[6] = {                     /* Bit d von w = a_{i-d} */
    (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<6),             /* b0 = 133 */
    (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<6),             /* b1 = 171 */
    (1<<0)|(1<<1)|(1<<4)|(1<<6),                    /* b2 = 145 */
    (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<6),
    (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<6),
    (1<<0)|(1<<1)|(1<<4)|(1<<6)};

static uint8_t outbits[128];        /* [w] -> 6 Ausgangsbits (b0 = Bit0) */
static int ok;

static void init(void)
{
    for (int w = 0; w < 128; w++) {
        uint8_t o = 0;
        for (int j = 0; j < 6; j++) o |= (uint8_t)(__builtin_parity(w & taps[j]) << j);
        outbits[w] = o;
    }
    ok = 1;
}

void drm_viterbi(const int8_t *soft, int n, uint64_t *dec, uint8_t *out)
{
    if (!ok) init();
    int32_t pm[64], nm[64];
    for (int s = 0; s < 64; s++) pm[s] = -1000000;
    pm[0] = 0;
    for (int i = 0; i < n; i++) {
        const int8_t *sf = soft + 6*i;
        int32_t bm[64];                              /* Zweig-Metrik je Ausgangsmuster */
        for (int o = 0; o < 64; o++) {
            int32_t m = 0;
            for (int j = 0; j < 6; j++) m += (o >> j & 1) ? -sf[j] : sf[j];
            bm[o] = m;
        }
        uint64_t d = 0;
        int32_t best = -0x7fffffff;
        for (int ns = 0; ns < 64; ns++) {
            int a = ns & 1;
            int32_t m0, m1;
            int s0 = (ns >> 1), s1 = (ns >> 1) | 32;  /* Vorgaenger, t = 0/1 */
            m0 = pm[s0] + bm[outbits[a | (s0 << 1)]];
            m1 = pm[s1] + bm[outbits[a | (s1 << 1)]];
            if (m1 > m0) { nm[ns] = m1; d |= 1ull << ns; } else nm[ns] = m0;
            if (nm[ns] > best) best = nm[ns];
        }
        for (int s = 0; s < 64; s++) pm[s] = nm[s] - best;   /* Normierung */
        dec[i] = d;
    }
    int st = 0;                                      /* terminiert */
    for (int i = n-1; i >= 0; i--) {
        out[i] = (uint8_t)(st & 1);
        st = (st >> 1) | (int)(((dec[i] >> st) & 1) << 5);
    }
}

/* Mutter-Code-Encoder: n Infobits (+6 Tailbits, intern Nullen) -> n+6 Ausgangsworte (Bit j = b_j) */
void drm_conv_encode(const uint8_t *a, int n, uint8_t *b)
{
    if (!ok) init();
    int st = 0;
    for (int i = 0; i < n + 6; i++) {
        int in = (i < n) ? (a[i] & 1) : 0;
        b[i] = outbits[in | (st << 1)];
        st = ((st << 1) | in) & 63;
    }
}
