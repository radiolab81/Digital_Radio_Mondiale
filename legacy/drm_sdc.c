#include "drm_sdc.h"
#include "drm_viterbi.h"
#include <string.h>

#define XIN (2*SDC_NCELL)          /* 644 */
#define LINFO 316
static int16_t perm[XIN];
static int inited;

static void init(void)
{
    const int s = 1024, q = s/4 - 1, t0 = 21;     /* 4-QAM: t0=21; s=2^ceil(log2 644) */
    perm[0] = 0;
    for (int i = 1; i < XIN; i++) {
        int v = (t0*perm[i-1] + q) % s;
        while (v >= XIN) v = (t0*v + q) % s;
        perm[i] = (int16_t)v;
    }
    inited = 1;
}
static unsigned getb(const uint8_t *b, int pos, int n) { unsigned v = 0; while (n--) v = (v<<1) | b[pos++]; return v; }

static uint16_t crc16(const uint8_t *bits, int n)
{
    uint16_t r = 0xffff;
    for (int i = 0; i < n; i++) {
        uint16_t fb = (uint16_t)(((r >> 15) & 1) ^ bits[i]);
        r <<= 1; if (fb) r ^= 0x1021;
    }
    return (uint16_t)~r;
}

int drm_sdc_decode(const int8_t *y, drm_sdc_t *sd)
{
    if (!inited) init();
    int8_t v[XIN];
    for (int i = 0; i < XIN; i++) v[perm[i]] = y[i];
    static int8_t soft[6*(LINFO+6)];
    memset(soft, 0, sizeof soft);
    for (int st = 0; st < LINFO+6; st++) { soft[6*st] = v[2*st]; soft[6*st+1] = v[2*st+1]; }   /* R=1/2 */
    static uint64_t dec[LINFO+6]; uint8_t out[LINFO+6];
    drm_viterbi(soft, LINFO+6, dec, out);
    /* Energy dispersal: PRBS X^9+X^5+1, Init 1 */
    uint8_t bits[LINFO]; int r[9]; for (int i = 0; i < 9; i++) r[i] = 1;
    for (int i = 0; i < LINFO; i++) {
        int fb = r[4] ^ r[8];
        for (int j = 8; j > 0; j--) r[j] = r[j-1];
        r[0] = fb; bits[i] = out[i] ^ (uint8_t)fb;
    }
    memset(sd, 0, sizeof *sd);
    /* CRC ueber AFS-Index als 8 Bit (4 MSBs=0) + Datenfeld (37 Byte) */
    uint8_t buf[8 + 296];
    memset(buf, 0, 4); memcpy(buf+4, bits, 4); memcpy(buf+8, bits+4, 296);
    sd->crc_ok = (crc16(buf, 304) == getb(bits, 300, 16));
    sd->afs_index = (int)getb(bits, 0, 4);
    if (!sd->crc_ok) return 0;
    /* Datenentitaeten */
    int pos = 4, end = 4 + 296;
    while (pos + 12 <= end) {
        int len = (int)getb(bits, pos, 7), type = (int)getb(bits, pos+8, 4);
        if (len == 0 && type == 0 && getb(bits, pos, 12) == 0) break;     /* Padding */
        int b0 = pos + 12, bodybits = 4 + 8*len;
        if (b0 + bodybits > end) break;
        if (type == 0) {
            sd->prot_a = (int)getb(bits, b0, 2); sd->prot_b = (int)getb(bits, b0+2, 2);
            sd->nstreams = len/3; if (sd->nstreams > 4) sd->nstreams = 4;
            for (int i = 0; i < sd->nstreams; i++) {
                sd->len_a[i] = (int)getb(bits, b0+4+24*i, 12); sd->len_b[i] = (int)getb(bits, b0+16+24*i, 12);
            }
        } else if (type == 1) {
            int id = (int)getb(bits, b0, 2), n = len < 64 ? len : 64;
            for (int i = 0; i < n; i++) sd->label[id][i] = (char)getb(bits, b0+4+8*i, 8);
            sd->label[id][n] = 0;
        } else if (type == 9) {
            sd->have_audio = 1;
            sd->a_short = (int)getb(bits, b0, 2); sd->a_stream = (int)getb(bits, b0+2, 2);
            sd->a_coding = (int)getb(bits, b0+4, 2); sd->a_sbr = (int)getb(bits, b0+6, 1);
            sd->a_mode = (int)getb(bits, b0+7, 2); sd->a_rate = (int)getb(bits, b0+9, 3);
            sd->a_text = (int)getb(bits, b0+12, 1); sd->a_enh = (int)getb(bits, b0+13, 1);
            sd->a_cfglen = len - 2; if (sd->a_cfglen > 24) sd->a_cfglen = 24; if (sd->a_cfglen < 0) sd->a_cfglen = 0;
            for (int i = 0; i < sd->a_cfglen; i++) sd->a_cfg[i] = (uint8_t)getb(bits, b0+20+8*i, 8);
        }
        pos = b0 + bodybits;
    }
    return 1;
}
