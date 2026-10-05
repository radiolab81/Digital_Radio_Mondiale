#include "drm_fac.h"
#include "drm_viterbi.h"
#include <string.h>

#define XIN 130
static int16_t perm[XIN];
static uint8_t prbs[128];
static int inited;

static void init(void)
{
    /* Bit-Interleaver Kap. 7.3.3: 4-QAM/FAC t0=21, s=256, q=63 */
    const int s = 256, q = s/4 - 1, t0 = 21;
    perm[0] = 0;
    for (int i = 1; i < XIN; i++) {
        int v = (t0*perm[i-1] + q) % s;
        while (v >= XIN) v = (t0*v + q) % s;
        perm[i] = (int16_t)v;
    }
    /* PRBS X^9+X^5+1, Init 1..1; erste Bits: 0000011110111110 (Tab. 26) */
    unsigned r = 0x1ff;
    for (int i = 0; i < 128; i++) {
        unsigned fb = ((r >> 8) ^ (r >> 4)) & 1;       /* Stufe 9 xor Stufe 5 */
        prbs[i] = (uint8_t)fb;
        r = ((r << 1) | fb) & 0x1ff;
    }
    inited = 1;
}

static uint8_t crc8(const uint8_t *bits, int n)
{
    uint8_t r = 0xff;
    for (int i = 0; i < n; i++) {
        uint8_t fb = (uint8_t)(((r >> 7) & 1) ^ bits[i]);
        r <<= 1; if (fb) r ^= 0x1d;
    }
    return (uint8_t)~r;
}
static unsigned getb(const uint8_t *b, int pos, int n) { unsigned v = 0; while (n--) v = (v<<1) | b[pos++]; return v; }

int drm_fac_decode(const int8_t *y, drm_fac_t *f)
{
    if (!inited) init();
    int8_t v[XIN];
    for (int i = 0; i < XIN; i++) v[perm[i]] = y[i];          /* y_i = v_perm(i) */
    /* Entpunktieren 3/5 (B0:111, B1:101), 78 Schritte */
    int8_t soft[6*78]; memset(soft, 0, sizeof soft);
    static const uint8_t pat[3][2] = {{1,1},{1,0},{1,1}};
    int pos = 0;
    for (int st = 0; st < 78; st++)
        for (int j = 0; j < 2; j++) if (pat[st%3][j]) soft[6*st+j] = v[pos++];
    uint64_t dec[78]; uint8_t out[78];
    drm_viterbi(soft, 78, dec, out);
    memset(f, 0, sizeof *f);
    for (int i = 0; i < 72; i++) f->bits[i] = out[i] ^ prbs[i];
    f->crc_ok = (crc8(f->bits, 64) == getb(f->bits, 64, 8));
    const uint8_t *b = f->bits;
    f->identity = getb(b,1,2); f->rm_flag = b[3]; f->spectrum_occ = getb(b,4,3);
    f->interleaver_depth = b[7]; f->msc_mode = getb(b,8,2); f->sdc_mode = b[10];
    f->nservices = getb(b,11,4); f->reconf = getb(b,15,3); f->toggle = b[18];
    f->service_id = getb(b,20,24); f->short_id = getb(b,44,2); f->audio_ca = b[46];
    f->language = getb(b,47,4); f->audio_data = b[51]; f->descriptor = getb(b,52,5);
    return f->crc_ok;
}
