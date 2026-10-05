#include "drm_msc.h"
#include "drm_viterbi.h"
#include <math.h>
#include <string.h>

#define N MSC_NMUX
#define XB (2*N)                       /* 4674 Bit je Ebene */
#define M0 1554
#define M1 3108
static int16_t Pc[N], Pci[N];          /* Zell-Interleaver und Inverse */
static int16_t Pb0[XB], Pb1[XB];       /* Bit-Interleaver Ebene 0 (t=13), Ebene 1 (t=21) */
static int inited;

static void mkperm(int16_t *P, int n, int t0)
{
    int s = 1; while (s < n) s <<= 1;
    int q = s/4 - 1;
    P[0] = 0;
    for (int i = 1; i < n; i++) {
        int v = (t0*P[i-1] + q) % s;
        while (v >= n) v = (t0*v + q) % s;
        P[i] = (int16_t)v;
    }
}
static void init(void)
{
    mkperm(Pc, N, 5);
    for (int i = 0; i < N; i++) Pci[Pc[i]] = (int16_t)i;
    mkperm(Pb0, XB, 13);
    mkperm(Pb1, XB, 21);
    inited = 1;
}

void drm_msc_deinterleave(const msc_cell_t *frames, msc_cell_t *out)
{
    if (!inited) init();
    for (int j = 0; j < N; j++) {
        int i = Pci[j];                              /* transmittiert wurde z_{n-G(i),P(i)} an Position i */
        out[j] = frames[(i % MSC_D)*N + i];
    }
}

/* Konstellationspegel (in Einheiten von a): (i0,i1) -> Re : 11:-3  01:-1  10:+1  00:+3 */
static inline float lvl(int b0, int b1) { static const float t[4] = {3.f, -1.f, 1.f, -3.f}; return t[(b0<<1)|b1]; }

static inline int8_t q8(float v, float sc)
{
    v *= sc; if (v > 127) v = 127; if (v < -127) v = -127; return (int8_t)lrintf(v);
}

/* Kleinster Abstand^2 ueber Punkte mit Bit0 = 0 bzw. 1 (Ebene 0) */
static void llr_l0(float x, float w, float *l)
{
    float d[4], m0 = 1e30f, m1 = 1e30f;
    for (int b0 = 0; b0 < 2; b0++) for (int b1 = 0; b1 < 2; b1++) {
        float e = x - lvl(b0, b1); e *= e;
        if (b0) { if (e < m1) m1 = e; } else { if (e < m0) m0 = e; }
    }
    (void)d; *l = w * (m1 - m0);
}
static void llr_l1(float x, float w, int b0, float *l)
{
    float e0 = x - lvl(b0, 0), e1 = x - lvl(b0, 1);
    *l = w * (e1*e1 - e0*e0);
}

static uint8_t bit_prbs(int i)               /* Energy dispersal PRBS (X^9+X^5+1, Init 1) */
{
    static uint8_t tab[MSC_LBITS]; static int ok;
    if (!ok) { int r[9]; for (int k = 0; k < 9; k++) r[k] = 1;
        for (int k = 0; k < MSC_LBITS; k++) { int fb = r[4]^r[8]; for (int j = 8; j > 0; j--) r[j] = r[j-1]; r[0] = fb; tab[k] = (uint8_t)fb; }
        ok = 1; }
    return tab[i];
}

int drm_msc_decode(const msc_cell_t *c, uint8_t *bits)
{
    if (!inited) init();
    const float k10 = sqrtf(10.0f);
    static float l0[XB], l1[XB]; static int8_t soft[6*(M1+6)], vs[XB];
    static uint64_t dec[M1+6]; static uint8_t out0[M0+6], out1[M1+6], enc[M1+6], hard0[XB];
    /* ---- Ebene 0: Bits (i0,q0) je Zelle ---- */
    float sum = 0;
    for (int n = 0; n < N; n++) {
        float xr = c[n].zr*k10, xi = c[n].zi*k10;
        llr_l0(xr, c[n].w, &l0[2*n]); llr_l0(xi, c[n].w, &l0[2*n+1]);
        sum += fabsf(l0[2*n]) + fabsf(l0[2*n+1]);
    }
    float sc = 24.0f / (sum/XB + 1e-12f);
    for (int i = 0; i < XB; i++) vs[Pb0[i]] = q8(l0[i], sc);                 /* v[P(i)] = y[i] */
    memset(soft, 0, sizeof soft);
    for (int st = 0; st < M0; st++) for (int j = 0; j < 3; j++) soft[6*st+j] = vs[3*st+j];
    for (int t = 0; t < 6; t++) for (int j = 0; j < 2; j++) soft[6*(M0+t)+j] = vs[3*M0 + 2*t + j];
    drm_viterbi(soft, M0+6, dec, out0);
    /* ---- Reencode Ebene 0 -> harte Bits y_{0,i} ---- */
    drm_conv_encode(out0, M0, enc);
    static uint8_t vh[XB];
    for (int st = 0; st < M0; st++) for (int j = 0; j < 3; j++) vh[3*st+j] = (enc[st] >> j) & 1;
    for (int t = 0; t < 6; t++) for (int j = 0; j < 2; j++) vh[3*M0 + 2*t + j] = (enc[M0+t] >> j) & 1;
    for (int i = 0; i < XB; i++) hard0[i] = vh[Pb0[i]];
    int agree = 0;
    for (int i = 0; i < XB; i++) agree += ((l0[i] > 0) == (hard0[i] == 0));
    /* ---- Ebene 1 mit bekannter Ebene 0 ---- */
    sum = 0;
    for (int n = 0; n < N; n++) {
        float xr = c[n].zr*k10, xi = c[n].zi*k10;
        llr_l1(xr, c[n].w, hard0[2*n],   &l1[2*n]);
        llr_l1(xi, c[n].w, hard0[2*n+1], &l1[2*n+1]);
        sum += fabsf(l1[2*n]) + fabsf(l1[2*n+1]);
    }
    sc = 24.0f / (sum/XB + 1e-12f);
    for (int i = 0; i < XB; i++) vs[Pb1[i]] = q8(l1[i], sc);
    memset(soft, 0, sizeof soft);
    int p = 0;                                                /* 2/3: B0=11, B1=10 */
    for (int st = 0; st < M1; st++) { soft[6*st] = vs[p++]; if ((st & 1) == 0) soft[6*st+1] = vs[p++]; }
    for (int t = 0; t < 6; t++) for (int j = 0; j < 2; j++) soft[6*(M1+t)+j] = vs[p++];
    drm_viterbi(soft, M1+6, dec, out1);
    /* ---- zusammenfuegen + Energy Dispersal ---- */
    for (int i = 0; i < M0; i++) bits[i] = out0[i] ^ bit_prbs(i);
    for (int i = 0; i < M1; i++) bits[M0+i] = out1[i] ^ bit_prbs(M0+i);
    return (int)(1000L*agree/XB);
}
