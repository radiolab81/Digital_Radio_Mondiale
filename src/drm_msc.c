/* ==========================================================================================================
 * drm_msc.c  --  Kanalcodierung des MSC (Main Service Channel): Mehrebenencode, UEP, Interleaver, Dekoder
 * ==========================================================================================================
 * Die Nutzdaten (Audio, Daten) gehen als Bitstrom u ueber den MSC.  Der Sender macht (ES 201 980, Kap. 7):
 *
 *   u --Energieverwischung PRBS [7.2.2]--> Aufteilen auf Ebenen [7.3.1.1] --> je Ebene p: Faltungscode R=1/6
 *   [7.3.2] + Punktierung [Tab. 27/28] --> Bitverschachtelung [7.3.3.3] --> QAM-Abbildung [7.4]
 *   --> Zellverschachtelung ueber D Rahmen [7.6] --> OFDM-Zellen [7.7].
 *
 * Mehrebenencode (Multilevel Coding, "MLC", [7.3.1]):  Eine 16-QAM-Zelle traegt 4 Bit, eine 64-QAM-Zelle 6 Bit.
 * Real- und Imaginaerteil bilden je eine ASK-Achse mit 4 bzw. 8 Pegeln (2 bzw. 3 Bit je Achse).  Die Bits werden
 * so auf die Pegel abgebildet ("Set Partitioning"), dass Bit 0 die grobe Hälfte der Achse waehlt (robust), die
 * folgenden Bits immer feiner unterteilen (empfindlicher).  Jede Ebene p bekommt daher ihren eigenen Faltungs-
 * code mit eigener Rate R_p: die empfindlichsten Ebenen werden staerker geschuetzt [Tab. 30, 32].
 *
 * Empfangsprinzip (Mehrstufendekodierung mit Rueckkopplung, [7.3.1.0] "decoding ... iterative process"):
 *   1) Ebene 0 aus den Zellen schaetzen (min. Distanz ueber alle Punkte, deren Bit 0 = 0 bzw. 1);
 *   2) Viterbi, Re-Encoding -> "harte" Bits der Ebene 0;
 *   3) Ebene 1 mit Kenntnis von Ebene 0 schaetzen, usw. fuer alle Ebenen;
 *   4) Iteration: jede Ebene erneut, jetzt mit Kenntnis ALLER anderen Ebenen -> bessere Entscheidungen.
 *      Im Simulator (drm_sim) bringt das etwa 1-3 dB.
 *
 * UEP (Unequal Error Protection, [7.2.1.1, 7.3.1.1, 7.3.3.3, 7.5.1]): Ein Multiplexrahmen darf aus zwei Teilen
 * mit verschiedener Schutzstufe bestehen.  Teil A (hoeher geschuetzt, X Byte laut SDC) belegt die ersten N1 Zellen,
 * Teil B die restlichen N2 = N_MUX - N1 Zellen.  Jede Ebene p bekommt Infobits M_p1 (Teil A) + M_p2 (Teil B),
 * wird in EINEM Faltungskodierer-Durchlauf codiert, aber im Teil A mit der Punktierung von PL_A, im Teil B mit
 * der von PL_B; die Tailbits stehen am Ende von Teil B.  Bitverschachtelung und Entschachtelung arbeiten je
 * Teil getrennt (Laengen 2*N1 bzw. 2*N2).  Die Teile ueberlappen sich also nirgends.
 * ========================================================================================================== */
#include "drm_msc.h"
#include "drm_viterbi.h"
#include <math.h>
#include <string.h>

/* ----------------------------------------------------------------------------------------------------------
 * Punktierung [7.3.2, Tab. 27]:  Das Muster B[j][st % P] sagt, ob Codebit b_j im Trellisschritt st uebertragen
 * wird (1) oder nicht (0).  Rate RX/RY heisst: je RX Eingangsbits werden RY Bits uebertragen.
 * ---------------------------------------------------------------------------------------------------------- */
typedef struct { int rx, ry, P; uint8_t B[6][8]; } punct_t;
static const punct_t pt[] = {
    {1,4,1,{{1},{1},{1},{1},{0},{0}}},                         /* R = 1/4 */
    {1,3,1,{{1},{1},{1},{0},{0},{0}}},                         /* 1/3 */
    {1,2,1,{{1},{1},{0},{0},{0},{0}}},                         /* 1/2 */
    {2,3,2,{{1,1},{1,0}}},                                     /* 2/3 */
    {3,4,3,{{1,1,1},{1,0,0}}},                                 /* 3/4 */
    {4,5,4,{{1,1,1,1},{1,0,0,0}}},                             /* 4/5 */
    {7,8,7,{{1,1,1,1,1,1,1},{1,0,0,0,0,0,0}}},                 /* 7/8 */
    {8,9,8,{{1,1,1,1,1,1,1,1},{1,0,0,0,0,0,0,0}}},             /* 8/9 */
};
static const punct_t *find_p(int rx, int ry)
{
    for (unsigned i = 0; i < sizeof pt / sizeof *pt; i++) if (pt[i].rx == rx && pt[i].ry == ry) return &pt[i];
    return NULL;
}

/* Tailbits [7.3.2, Tab. 28]: Nach den Infobits folgen 6 Schritte mit Eingang 0, damit der Kodierer im Zustand 0
 * endet.  Wie viele ihrer 6*6 Codebits uebertragen werden, haengt vom Rest rp = (2N2-12) mod RY ab.  B0 = B1 = 1
 * immer; B2 und B3 nach Tabelle; B4/B5 nie (fuer die hier vorkommenden Raten). */
static const uint8_t tailB2[10][6] = {{0,0,0,0,0,0},{1,0,0,0,0,0},{1,0,0,1,0,0},{1,1,0,1,0,0},{1,1,0,1,1,0},{1,1,1,1,1,0},{1,1,1,1,1,1},{1,1,1,1,1,1},{1,1,1,1,1,1},{1,1,1,1,1,1}};
static const uint8_t tailB3[10][6] = {{0},{0},{0},{0},{0},{0},{0},{1,0,0,0,0,0},{1,0,0,1,0,0},{1,1,0,1,0,0}};
static inline int tail_sel(int rp, int st, int j) { return j < 2 ? 1 : j == 2 ? tailB2[rp][st] : j == 3 ? tailB3[rp][st] : 0; }

/* ----------------------------------------------------------------------------------------------------------
 * Schutzstufen [7.5.1.1, Tab. 30 (16-QAM) und Tab. 32 (64-QAM)]: Coderaten je Ebene.
 * ---------------------------------------------------------------------------------------------------------- */
static const int rates16[2][2][2] = {{{1,3},{2,3}}, {{1,2},{3,4}}};
static const int rates64[4][3][2] = {{{1,4},{1,2},{3,4}}, {{1,3},{2,3},{4,5}}, {{1,2},{3,4},{7,8}}, {{2,3},{4,5},{8,9}}};
static int gcd_(int a, int b) { while (b) { int t = a % b; a = b; b = t; } return a; }

int drm_msc_setup(drm_msc_cfg_t *c, int qam, int n, int depth, int plA, int plB, int bytes_a)
{
    memset(c, 0, sizeof *c);
    if (n <= 0 || n > MSC_MAXN) return -1;
    c->n = n; c->depth = depth; c->qam = qam;
    if (qam == 16)      { c->levels = 2; if (plA < 0 || plA > 1 || plB < 0 || plB > 1) return -1; }
    else if (qam == 64) { c->levels = 3; if (plA < 0 || plA > 3 || plB < 0 || plB > 3) return -1; }
    else return -1;
    int lcm = 1, sumRA_num = 0;                          /* RY_lcm und Summe der Raten R_p von Teil A [Tab. 30/32] */
    double sumRA = 0;
    for (int p = 0; p < c->levels; p++) {
        c->rxA[p] = qam == 16 ? rates16[plA][p][0] : rates64[plA][p][0];  c->ryA[p] = qam == 16 ? rates16[plA][p][1] : rates64[plA][p][1];
        c->rxB[p] = qam == 16 ? rates16[plB][p][0] : rates64[plB][p][0];  c->ryB[p] = qam == 16 ? rates16[plB][p][1] : rates64[plB][p][1];
        if (!find_p(c->rxA[p], c->ryA[p]) || !find_p(c->rxB[p], c->ryB[p])) return -1;
        lcm = lcm / gcd_(lcm, c->ryA[p]) * c->ryA[p];
        sumRA += (double)c->rxA[p] / c->ryA[p];
    }
    (void)sumRA_num;
    /* N1 = ceil( 8X / (2 * RYlcm * sum R_p) ) * RYlcm   [7.2.1.1] */
    if (bytes_a > 0) {
        double t = 8.0 * bytes_a / (2.0 * lcm * sumRA);
        c->n1 = (int)ceil(t - 1e-9) * lcm;
        if (c->n1 > n - 20) return -1;                   /* Einschraenkung N1 <= N_MUX - 20 */
    }
    c->n2 = n - c->n1;
    for (int p = 0; p < c->levels; p++) {
        c->M1[p] = 2 * c->n1 * c->rxA[p] / c->ryA[p];                         /* M_p,1 = 2 N1 R_p   [7.3.1.1] */
        c->M2[p] = c->rxB[p] * ((2 * c->n2 - 12) / c->ryB[p]);                /* M_p,2 = RX_p floor((2N2-12)/RY_p) */
        c->M[p] = c->M1[p] + c->M2[p];
        c->L1 += c->M1[p]; c->L2 += c->M2[p];
    }
    c->lbits = c->L1 + c->L2;
    return c->lbits <= MSC_MAXBITS ? 0 : -1;
}

int drm_msc_config(drm_msc_cfg_t *c, const drm_mode_t *m, int msc_mode, int depth_flag, int plA, int plB, int bytes_a)
{
    /* FAC-Feld MSC mode [6.3.3]: 00 = 64-QAM, 11 = 16-QAM (Hierarchical Mapping ist in V4.3.1 entfallen).  Die
     * Zeitverschachtelung ist lang (D = 5) oder kurz (D = 1) [7.6]. */
    int qam = (msc_mode == 3) ? 16 : (msc_mode == 0) ? 64 : 0;
    return drm_msc_setup(c, qam, m->nmux, depth_flag ? 1 : 5, plA, plB, bytes_a);
}

/* ----------------------------------------------------------------------------------------------------------
 * Verschachtelungs-Permutationen [7.3.3.1/7.6]:  P(0) = 0,  P(i) = (t0 * P(i-1) + q) mod s,  s = 2^ceil(log2 n),
 * q = s/4 - 1; Werte >= n werden uebersprungen (nochmal iteriert).  t0 = 5 (Zellen), 13 / 21 (Bitebenen).
 * ---------------------------------------------------------------------------------------------------------- */
static void mkperm(int16_t *P, int n, int t0)
{
    int s = 1; while (s < n) s <<= 1;
    int q = s / 4 - 1; P[0] = 0;
    for (int i = 1; i < n; i++) { int v = (t0 * P[i-1] + q) % s; while (v >= n) v = (t0 * v + q) % s; P[i] = (int16_t)v; }
}

/* Cache: Zellpermutation (n) und Bitpermutationen je Teil (2*n1, 2*n2) fuer t0 = 13 und 21. */
static int16_t Pc[MSC_MAXN], Pci[MSC_MAXN];
static int16_t Pb1[2][2 * MSC_MAXN], Pb2[2][2 * MSC_MAXN];
static int cache_n = -1, cache_n1 = -1;
static void init_perm(int n, int n1)
{
    if (cache_n == n && cache_n1 == n1) return;
    mkperm(Pc, n, 5);
    for (int i = 0; i < n; i++) Pci[Pc[i]] = (int16_t)i;
    if (n1 > 0) { mkperm(Pb1[0], 2 * n1, 13); mkperm(Pb1[1], 2 * n1, 21); }
    mkperm(Pb2[0], 2 * (n - n1), 13); mkperm(Pb2[1], 2 * (n - n1), 21);
    cache_n = n; cache_n1 = n1;
}

/* Bit-Interleaver-Wahl je Ebene [7.3.3.3, Tab. bei Bild 23/24]: 16-QAM: Ebene 0 -> t0 = 13, Ebene 1 -> 21.
 * 64-QAM: Ebene 0 ohne Interleaver, Ebene 1 -> 13, Ebene 2 -> 21.  Rueckgabe: Index 0/1 (t0 = 13/21) oder -1. */
static int level_t(const drm_msc_cfg_t *c, int p) { return c->levels == 3 ? p - 1 : p; }

/* Zell-Deinterleaver [7.6]:  fuer jede Ausgangsposition j wird der Eingangsrahmen n-G(i) mit G(i) = i mod D und
 * die Eingangsposition P(i) berechnet; wir kehren das um (i = P^-1(j)). */
void drm_msc_deinterleave(const drm_msc_cfg_t *c, const msc_cell_t *const *frames, msc_cell_t *out)
{
    init_perm(c->n, c->n1);
    for (int j = 0; j < c->n; j++) { int i = Pci[j]; out[j] = frames[i % c->depth][i]; }
}

/* ----------------------------------------------------------------------------------------------------------
 * QAM-Pegel [7.4, Bild 26 (64-QAM) und 29 (16-QAM)]:  idx = (b0 b1 [b2]) als Binaerzahl, b0 = MSB.
 * 16-QAM: 00 -> +3, 01 -> -1, 10 -> +1, 11 -> -3  (Einheit a, a = 1/sqrt(10));
 * 64-QAM: 000 +7, 001 -1, 010 +3, 011 -5, 100 +5, 101 -3, 110 +1, 111 -7  (a = 1/sqrt(42)).
 * ---------------------------------------------------------------------------------------------------------- */
static inline float amp(int levels, int idx)
{
    static const float t2[4] = {3, -1, 1, -3}, t3[8] = {7, -1, 3, -5, 5, -3, 1, -7};
    return levels == 2 ? t2[idx] : t3[idx];
}

/* LLR (>0 => Bit 0) von Ebene p einer Achse, wenn die oberen Ebenen kb (p Stueck, MSB zuerst) bekannt sind. */
static float llr_axis(int levels, int p, int kb, float x, float w)
{
    float m0 = 1e30f, m1 = 1e30f;
    for (int idx = 0; idx < (1 << levels); idx++) {
        if ((idx >> (levels - p)) != kb) continue;            /* nur Punkte, die zu den bekannten Bits passen */
        int bit = (idx >> (levels - 1 - p)) & 1;
        float e = x - amp(levels, idx); e *= e;               /* quadratische Distanz zum Punkt */
        if (bit) { if (e < m1) m1 = e; } else { if (e < m0) m0 = e; }
    }
    return w * (m1 - m0);                                     /* max-log-LLR, gewichtet mit |H|^2 */
}
/* Wie oben, aber alle anderen Ebenen q != p mit Bits kb[q] bekannt (Iteration). */
static float llr_axis_all(int levels, int p, const int *kb, float x, float w)
{
    float m0 = 1e30f, m1 = 1e30f;
    for (int idx = 0; idx < (1 << levels); idx++) {
        int ok = 1;
        for (int q = 0; q < levels && ok; q++) if (q != p && ((idx >> (levels - 1 - q)) & 1) != kb[q]) ok = 0;
        if (!ok) continue;
        int bit = (idx >> (levels - 1 - p)) & 1;
        float e = x - amp(levels, idx); e *= e;
        if (bit) { if (e < m1) m1 = e; } else { if (e < m0) m0 = e; }
    }
    return w * (m1 - m0);
}

int drm_msc_iterations = 2;        /* Rueckkopplungsdurchlaeufe; 0 = einfacher Mehrstufendekoder */

/* Energieverwischung [7.2.2]: PRBS 1 + X^5 + X^9, Start 1..1; Bit i der Folge wird mit u_i XOR-verknuepft. */
static uint8_t prbs_bit(int i)
{
    static uint8_t tab[MSC_MAXBITS]; static int ok;
    if (!ok) {
        int r[9]; for (int k = 0; k < 9; k++) r[k] = 1;
        for (int k = 0; k < MSC_MAXBITS; k++) { int fb = r[4] ^ r[8]; for (int j = 8; j > 0; j--) r[j] = r[j-1]; r[0] = fb; tab[k] = (uint8_t)fb; }
        ok = 1;
    }
    return tab[i];
}

/* Zusammenstellung der uebertragenen Bits einer Ebene.  Reihenfolge im Vektor v: [Teil A | Teil B]; Teil B enthaelt
 * hinten die Tailbits.  Die Funktion legt fest, WELCHE Codebits uebertragen werden (Punktierung) und ruft cb je
 * uebertragenem Bit mit (Schritt, Codebit-Nr) -- ein Rahmen fuer Entpunktieren UND Punktieren. */
typedef void (*punct_cb)(void *ctx, int pos, int step, int bit);
static void walk_level(const drm_msc_cfg_t *c, int p, punct_cb cb, void *ctx)
{
    const punct_t *pa = find_p(c->rxA[p], c->ryA[p]), *pb = find_p(c->rxB[p], c->ryB[p]);
    const int n2b = 2 * c->n2, ryB = c->ryB[p];
    const int rp = (n2b - 12) - ryB * ((n2b - 12) / ryB);       /* Rest fuer die Tailbit-Auswahl [Tab. 28] */
    int pos = 0, st = 0;
    for (; st < c->M1[p]; st++)                                 /* Teil A: Muster von PL_A, Schrittzaehler ab 0 */
        for (int j = 0; j < 6; j++) if (pa->B[j][st % pa->P]) cb(ctx, pos++, st, j);
    for (int s2 = 0; s2 < c->M2[p]; s2++, st++)                 /* Teil B: Muster von PL_B, Zaehler neu ab 0 */
        for (int j = 0; j < 6; j++) if (pb->B[j][s2 % pb->P]) cb(ctx, pos++, st, j);
    for (int t = 0; t < 6; t++, st++)                           /* Tailbits */
        for (int j = 0; j < 6; j++) if (tail_sel(rp, t, j)) cb(ctx, pos++, st, j);
}

/* Entpunktieren: Soft-Wert v[pos] an die Trellisposition (Schritt, Bit) setzen. */
typedef struct { const int8_t *v; int8_t *soft; } depunct_t;
static void depunct_cb(void *ctx, int pos, int step, int bit) { depunct_t *d = (depunct_t *)ctx; d->soft[6 * step + bit] = d->v[pos]; }
/* Punktieren: Codebit des Re-Encodings in den Vektor schreiben. */
typedef struct { const uint8_t *enc; uint8_t *vh; } punct_t2;
static void punct_cb_fn(void *ctx, int pos, int step, int bit) { punct_t2 *d = (punct_t2 *)ctx; d->vh[pos] = (d->enc[step] >> bit) & 1; }

/* Eine Ebene dekodieren: llr in Senderreihenfolge y (ueber alle 2N Bits) -> info (M_p Bits), hard_y = Re-Encoding
 * in derselben Reihenfolge y (fuer die Rueckkopplung an die anderen Ebenen). */
static void level_decode(const drm_msc_cfg_t *c, int p, const float *llr, uint8_t *info, uint8_t *hard_y)
{
    const int n2b = 2 * c->n2, n1b = 2 * c->n1, nb = 2 * c->n, M = c->M[p], steps = M + 6;
    const int t = level_t(c, p);
    static int8_t v[2 * MSC_MAXN], soft[6 * (MSC_MAXBITS / 2)]; static uint64_t dec[MSC_MAXBITS / 2];
    static uint8_t vh[2 * MSC_MAXN], enc[MSC_MAXBITS / 2];
    /* 1) Skalieren auf 8 Bit (Viterbi arbeitet mit int8) */
    float sum = 0; for (int i = 0; i < nb; i++) sum += fabsf(llr[i]);
    float sc = 24.0f / (sum / nb + 1e-12f);
    /* 2) Bit-Entschachtelung je Teil: y[i] = v[P(i)]  =>  v[P(i)] = y[i]   [7.3.3.3] */
    for (int i = 0; i < nb; i++) {
        float x = llr[i] * sc; if (x > 127) x = 127; if (x < -127) x = -127; int8_t q = (int8_t)lrintf(x);
        if (t < 0)            v[i] = q;                                  /* Ebene 0 bei 64-QAM: kein Interleaver */
        else if (i < n1b)     v[Pb1[t][i]] = q;                          /* Teil A */
        else                  v[n1b + Pb2[t][i - n1b]] = q;              /* Teil B */
    }
    /* 3) Entpunktieren -> Trellis, 4) Viterbi */
    memset(soft, 0, (size_t)6 * steps);
    depunct_t d1 = { v, soft }; walk_level(c, p, depunct_cb, &d1);
    drm_viterbi(soft, steps, dec, info);
    /* 5) Re-Encoding, Punktieren, Verschachteln -> harte Bits in y-Reihenfolge */
    drm_conv_encode(info, M, enc);
    punct_t2 d2 = { enc, vh }; walk_level(c, p, punct_cb_fn, &d2);
    for (int i = 0; i < nb; i++) {
        if (t < 0)        hard_y[i] = vh[i];
        else if (i < n1b) hard_y[i] = vh[Pb1[t][i]];
        else              hard_y[i] = vh[n1b + Pb2[t][i - n1b]];
    }
    (void)n2b;
}

/* Position von Infobit i der Ebene p im Sendebitstrom u: erst alle Teil-A-Bits (Ebene fuer Ebene), dann Teil B. */
static int u_index(const drm_msc_cfg_t *c, int p, int i)
{
    if (i < c->M1[p]) { int off = 0; for (int q = 0; q < p; q++) off += c->M1[q]; return off + i; }
    int off = c->L1; for (int q = 0; q < p; q++) off += c->M2[q];
    return off + (i - c->M1[p]);
}

int drm_msc_decode(const drm_msc_cfg_t *c, const msc_cell_t *cell, uint8_t *bits)
{
    init_perm(c->n, c->n1);
    const int N = c->n, L = c->levels; const float norm = (L == 2) ? sqrtf(10.0f) : sqrtf(42.0f);   /* a = 1/sqrt(10|42) */
    static float llr[2 * MSC_MAXN]; static uint8_t hard[3][2 * MSC_MAXN]; static uint8_t info[MSC_MAXBITS / 2];
    int agree = 0;

    /* Erster Durchlauf: Ebenen nacheinander, jede mit den bereits entschiedenen oberen Ebenen */
    for (int p = 0; p < L; p++) {
        for (int n = 0; n < N; n++) for (int ax = 0; ax < 2; ax++) {         /* ax 0 = Realteil (i-Bits), 1 = Imaginaerteil (q-Bits) */
            float x = (ax ? cell[n].zi : cell[n].zr) * norm; int kb = 0;
            for (int q = 0; q < p; q++) kb = (kb << 1) | hard[q][2 * n + ax];
            llr[2 * n + ax] = llr_axis(L, p, kb, x, cell[n].w);
        }
        level_decode(c, p, llr, info, hard[p]);
        if (p == 0) for (int i = 0; i < 2 * N; i++) agree += ((llr[i] > 0) == (hard[0][i] == 0));
        for (int i = 0; i < c->M[p]; i++) { int u = u_index(c, p, i); bits[u] = info[i] ^ prbs_bit(u); }
    }
    /* Rueckkopplung: jede Ebene erneut, mit Kenntnis aller anderen Ebenen */
    for (int it = 0; it < drm_msc_iterations && L > 1; it++)
        for (int p = 0; p < L; p++) {
            for (int n = 0; n < N; n++) for (int ax = 0; ax < 2; ax++) {
                float x = (ax ? cell[n].zi : cell[n].zr) * norm; int kb[3] = {0, 0, 0};
                for (int q = 0; q < L; q++) if (q != p) kb[q] = hard[q][2 * n + ax];
                llr[2 * n + ax] = llr_axis_all(L, p, kb, x, cell[n].w);
            }
            level_decode(c, p, llr, info, hard[p]);
            for (int i = 0; i < c->M[p]; i++) { int u = u_index(c, p, i); bits[u] = info[i] ^ prbs_bit(u); }
        }
    return (int)(1000L * agree / (2 * N));
}

/* ---- Sender-Seite (nur fuer Selbsttests/Simulation) -------------------------------------------------------- */
int drm_msc_encode(const drm_msc_cfg_t *c, const uint8_t *bits, msc_cell_t *cells)
{
    init_perm(c->n, c->n1);
    const int N = c->n, L = c->levels, nb = 2 * N, n1b = 2 * c->n1;
    static uint8_t y[3][2 * MSC_MAXN], vh[2 * MSC_MAXN], info[MSC_MAXBITS / 2], enc[MSC_MAXBITS / 2];
    for (int p = 0; p < L; p++) {
        const int t = level_t(c, p);
        for (int i = 0; i < c->M[p]; i++) { int u = u_index(c, p, i); info[i] = bits[u] ^ prbs_bit(u); }   /* Verwischen + Aufteilen */
        drm_conv_encode(info, c->M[p], enc);
        punct_t2 d = { enc, vh }; walk_level(c, p, punct_cb_fn, &d);
        for (int i = 0; i < nb; i++) {
            if (t < 0)        y[p][i] = vh[i];
            else if (i < n1b) y[p][i] = vh[Pb1[t][i]];
            else              y[p][i] = vh[n1b + Pb2[t][i - n1b]];
        }
    }
    const float norm = (L == 2) ? sqrtf(10.0f) : sqrtf(42.0f);
    for (int n = 0; n < N; n++) {
        float a[2];
        for (int ax = 0; ax < 2; ax++) { int idx = 0; for (int p = 0; p < L; p++) idx = (idx << 1) | y[p][2 * n + ax]; a[ax] = amp(L, idx) / norm; }
        cells[n].zr = a[0]; cells[n].zi = a[1]; cells[n].w = 1.0f;
    }
    return 0;
}

void drm_msc_interleave(const drm_msc_cfg_t *c, const msc_cell_t *const *hist, msc_cell_t *out)
{
    init_perm(c->n, c->n1);
    for (int i = 0; i < c->n; i++) out[i] = hist[i % c->depth][Pc[i]];      /* Gegenstueck zu drm_msc_deinterleave */
}
