/* ==========================================================================================================
 * drm_sdc.c  --  Service Description Channel (SDC)
 * ==========================================================================================================
 * Der SDC steht in den ersten 2 (Mode A/B) bzw. 3 (C/D) OFDM-Symbolen jedes 1,2-s-Superframes [8.5.3] und
 * beschreibt, WAS im MSC steckt: Anzahl und Laenge der Datenstroeme, Schutzstufen, Audiocodec, Abtastrate,
 * Programmname usw.  Seine Bits werden wie der FAC kodiert, aber mit 4-QAM (Rate 0,5) oder 16-QAM [7.5.2].
 *
 * Blockaufbau [6.4.2, Tab. 21]:  AFS-Index (4 Bit) | Datenfeld (n Byte) | Auffuellung | CRC-16 (16 Bit).
 * Das Datenfeld ist eine Folge von Datenentitaeten: Kopf 12 Bit (Laenge 7 Bit | Version 1 Bit | Typ 4 Bit),
 * danach der Koerper [6.4.3.0].  Die Laenge zaehlt in Byte OHNE die ersten 4 Bit des Koerpers.
 * CRC-16: G16(x) = X^16 + X^12 + X^5 + 1, Initialwert 1..1, Ergebnis invertiert; berechnet ueber den AFS-Index
 * (zu 8 Bit aufgefuellt) und das Datenfeld [Annex D, 6.4.2].
 * ========================================================================================================== */
#include "drm_sdc.h"
#include "drm_viterbi.h"
#include <math.h>
#include <string.h>

#define MAXX 1024                                 /* groesste Bitzahl des Bit-Interleavers */
static int16_t perm[MAXX]; static int perm_n = -1;

/* Laenge des Datenfelds in Byte [6.4.2, Tab. 21], [Modus A..D][SDC-Modus 0 = 16-QAM, 1 = 4-QAM][Belegung 0..3] */
static const int dflen[4][2][4] = {
    {{37,43,85,97}, {17,20,41,47}}, {{28,33,66,76}, {13,15,32,37}}, {{0,0,0,68}, {0,0,0,32}}, {{0,0,0,33}, {0,0,0,15}} };

/* Bit-Interleaver des 4-QAM-SDC [7.3.3.2]: t0 = 21, xin = 2*N_SDC (Formel wie beim FAC, siehe drm_fac.c). */
static void init_perm(int xin)
{
    int s = 1; while (s < xin) s <<= 1;
    const int q = s / 4 - 1, t0 = 21;
    perm[0] = 0;
    for (int i = 1; i < xin; i++) { int v = (t0 * perm[i-1] + q) % s; while (v >= xin) v = (t0 * v + q) % s; perm[i] = (int16_t)v; }
    perm_n = xin;
}
static unsigned getb(const uint8_t *b, int pos, int n) { unsigned v = 0; while (n--) v = (v << 1) | b[pos++]; return v; }

/* CRC-16 (siehe oben), bitweise, MSB zuerst. */
static uint16_t crc16(const uint8_t *bits, int n)
{
    uint16_t r = 0xffff;
    for (int i = 0; i < n; i++) { uint16_t fb = (uint16_t)(((r >> 15) & 1) ^ bits[i]); r <<= 1; if (fb) r ^= 0x1021; }
    return (uint16_t)~r;
}

int drm_sdc_decode(const drm_mode_t *m, int sdc_mode, const msc_cell_t *cells, drm_sdc_t *sd)
{
    if (m->nsdc <= 0 || (sdc_mode != 0 && sdc_mode != 1)) return -1;
    const int N = m->nsdc, nbytes = dflen[m->id][sdc_mode][m->occ];
    static uint8_t bits[MSC_MAXBITS];

    if (sdc_mode == 1) {
        /* ---- 4-QAM, Rate 1/2: Soft-Bits -> Entschachteln -> Viterbi -> Entwuerfeln [7.5.2] ---- */
        const int XIN = 2 * N, LINFO = N - 6;                 /* N Trellisschritte = LINFO Infobits + 6 Tail */
        if (perm_n != XIN) init_perm(XIN);
        static int8_t v[MAXX], y[MAXX]; float sum = 0;
        for (int n = 0; n < N; n++) sum += fabsf(cells[n].zr * cells[n].w) + fabsf(cells[n].zi * cells[n].w);
        const float sc = 40.0f / (sum / (2 * N) + 1e-9f);
        for (int n = 0; n < N; n++) for (int ax = 0; ax < 2; ax++) {          /* 4-QAM-Bits: i0 = Re, q0 = Im  [7.4, Bild 30] */
            float x = (ax ? cells[n].zi : cells[n].zr) * cells[n].w * sc;     /* Gewichtung mit |H|^2 */
            if (x > 127) x = 127;
            if (x < -127) x = -127;
            y[2 * n + ax] = (int8_t)lrintf(x);
        }
        for (int i = 0; i < XIN; i++) v[perm[i]] = y[i];
        static int8_t soft[6 * (MAXX / 2 + 8)]; for (int i = 0; i < 6 * N; i++) soft[i] = 0;
        for (int st = 0; st < N; st++) { soft[6 * st] = v[2 * st]; soft[6 * st + 1] = v[2 * st + 1]; }   /* Rate 1/2: b0, b1 */
        static uint64_t dec[MAXX / 2 + 8]; static uint8_t out[MAXX / 2 + 8];
        drm_viterbi(soft, N, dec, out);
        int r[9]; for (int i = 0; i < 9; i++) r[i] = 1;                       /* PRBS [7.2.2] */
        for (int i = 0; i < LINFO; i++) { int fb = r[4] ^ r[8]; for (int j = 8; j > 0; j--) r[j] = r[j-1]; r[0] = fb; bits[i] = out[i] ^ (uint8_t)fb; }
    } else {
        /* ---- 16-QAM: zweistufiger Mehrebenencode wie der MSC, EEP, ohne Zellverschachtelung [7.5.2] ---- */
        drm_msc_cfg_t c; if (drm_msc_setup(&c, 16, N, 1, 0, 0, 0) < 0) return -1;
        drm_msc_decode(&c, cells, bits);
    }

    memset(sd, 0, sizeof *sd);
    /* CRC: ueber 4 Nullbits + AFS-Index + Datenfeld; CRC-Feld direkt danach */
    static uint8_t buf[8 + 8 * 256];
    memset(buf, 0, 4); memcpy(buf + 4, bits, 4); memcpy(buf + 8, bits + 4, 8 * nbytes);
    sd->crc_ok = (crc16(buf, 8 + 8 * nbytes) == getb(bits, 4 + 8 * nbytes, 16));
    sd->afs_index = (int)getb(bits, 0, 4);
    if (!sd->crc_ok) return 0;

    /* Datenentitaeten der Reihe nach auswerten [6.4.3.0]: Kopf (Laenge 7 | Version 1 | Typ 4), dann Koerper */
    int pos = 4; const int end = 4 + 8 * nbytes;
    while (pos + 12 <= end) {
        int len = (int)getb(bits, pos, 7), type = (int)getb(bits, pos + 8, 4);
        if (len == 0 && type == 0 && getb(bits, pos, 12) == 0) break;           /* Auffuellung (Nullen) */
        int b0 = pos + 12, bodybits = 4 + 8 * len;
        if (b0 + bodybits > end) break;
        if (type == 0) {                                                        /* Multiplex description [6.4.3.1] */
            sd->prot_a = (int)getb(bits, b0, 2); sd->prot_b = (int)getb(bits, b0 + 2, 2);
            sd->nstreams = len / 3; if (sd->nstreams > 4) sd->nstreams = 4;     /* je Strom 24 Bit: Laenge A (12) + Laenge B (12) */
            for (int i = 0; i < sd->nstreams; i++) { sd->len_a[i] = (int)getb(bits, b0 + 4 + 24 * i, 12); sd->len_b[i] = (int)getb(bits, b0 + 16 + 24 * i, 12); }
        } else if (type == 1) {                                                 /* Label [6.4.3.2] */
            int id = (int)getb(bits, b0, 2), n = len < 64 ? len : 64;
            for (int i = 0; i < n; i++) sd->label[id][i] = (char)getb(bits, b0 + 4 + 8 * i, 8);
            sd->label[id][n] = 0;
        } else if (type == 9) {                                                 /* Audio information [6.4.3.10] */
            sd->have_audio = 1;
            sd->a_short = (int)getb(bits, b0, 2);      sd->a_stream = (int)getb(bits, b0 + 2, 2);
            sd->a_coding = (int)getb(bits, b0 + 4, 2); /* 0 AAC, 1 reserviert, 2 reserviert, 3 xHE-AAC */
            sd->a_sbr = (int)getb(bits, b0 + 6, 1);    sd->a_mode = (int)getb(bits, b0 + 7, 2);
            sd->a_rate = (int)getb(bits, b0 + 9, 3);   sd->a_text = (int)getb(bits, b0 + 12, 1); sd->a_enh = (int)getb(bits, b0 + 13, 1);
            sd->a_cfglen = len - 2; if (sd->a_cfglen > 24) sd->a_cfglen = 24; if (sd->a_cfglen < 0) sd->a_cfglen = 0;
            for (int i = 0; i < sd->a_cfglen; i++) sd->a_cfg[i] = (uint8_t)getb(bits, b0 + 20 + 8 * i, 8);
        }
        pos = b0 + bodybits;
    }
    return 0;
}

void drm_sdc_merge(drm_sdc_t *g, const drm_sdc_t *sd)
{
    if (!sd->crc_ok) return;
    g->crc_ok = 1;
    if (sd->nstreams > 0) { g->nstreams = sd->nstreams; g->prot_a = sd->prot_a; g->prot_b = sd->prot_b; memcpy(g->len_a, sd->len_a, sizeof g->len_a); memcpy(g->len_b, sd->len_b, sizeof g->len_b); }
    if (sd->have_audio) { g->have_audio = 1; g->a_short = sd->a_short; g->a_stream = sd->a_stream; g->a_coding = sd->a_coding; g->a_sbr = sd->a_sbr; g->a_mode = sd->a_mode;
                          g->a_rate = sd->a_rate; g->a_text = sd->a_text; g->a_enh = sd->a_enh; g->a_cfglen = sd->a_cfglen; memcpy(g->a_cfg, sd->a_cfg, sizeof g->a_cfg); }
    for (int q = 0; q < 4; q++) if (sd->label[q][0]) memcpy(g->label[q], sd->label[q], 65);
}

/* ---- Sender-Seite (nur fuer Selbsttests) ---------------------------------------------------------------- */
int drm_sdc_nbytes(const drm_mode_t *m, int sdc_mode) { return dflen[m->id][sdc_mode][m->occ]; }

/* bits[0..3] AFS-Index, bits[4 .. 4+8*nbytes) Datenfeld; hier wird die CRC-16 dahinter gesetzt, der Rest bleibt 0. */
void drm_sdc_finish(const drm_mode_t *m, int sdc_mode, uint8_t *bits)
{
    const int nbytes = dflen[m->id][sdc_mode][m->occ];
    static uint8_t buf[8 + 8 * 256];
    memset(buf, 0, 4); memcpy(buf + 4, bits, 4); memcpy(buf + 8, bits + 4, 8 * nbytes);
    uint16_t c = crc16(buf, 8 + 8 * nbytes);
    for (int i = 0; i < 16; i++) bits[4 + 8 * nbytes + i] = (uint8_t)((c >> (15 - i)) & 1);
}

void drm_sdc_encode4(const drm_mode_t *m, const uint8_t *info, float *re, float *im)
{
    const int N = m->nsdc, XIN = 2 * N, LINFO = N - 6;
    if (perm_n != XIN) init_perm(XIN);
    static uint8_t a[MAXX], enc[MAXX / 2 + 8], v[MAXX], y[MAXX];
    int r[9]; for (int i = 0; i < 9; i++) r[i] = 1;
    for (int i = 0; i < LINFO; i++) { int fb = r[4] ^ r[8]; for (int j = 8; j > 0; j--) r[j] = r[j-1]; r[0] = fb; a[i] = info[i] ^ (uint8_t)fb; }
    drm_conv_encode(a, LINFO, enc);
    for (int st = 0; st < N; st++) { v[2 * st] = enc[st] & 1; v[2 * st + 1] = (enc[st] >> 1) & 1; }            /* Rate 1/2: b0, b1 */
    for (int i = 0; i < XIN; i++) y[i] = v[perm[i]];
    const float a4 = 0.70710678f;
    for (int n = 0; n < N; n++) { re[n] = y[2 * n] ? -a4 : a4; im[n] = y[2 * n + 1] ? -a4 : a4; }
}
