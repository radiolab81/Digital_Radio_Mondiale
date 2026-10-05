/* ==========================================================================================================
 * drm_mode.c  --  Robustheitsmodi A-D: OFDM-Parameter, Pilot- und Zellraster
 * ==========================================================================================================
 * Ein DRM-Signal besteht aus OFDM-Symbolen.  Jedes Symbol traegt auf K Unterträgern je eine komplexe "Zelle".
 * Welche Zelle was ist, legt die Norm als Raster aus (Symbolindex s) x (Traegerindex k) fest [8.4 - 8.6]:
 *
 *      Frequenzreferenzen (3 feste Traeger, in jedem Symbol)   [8.4.2, Tab. 51-52]  -> Frequenz-/Taktnachfuehrung
 *      Zeitreferenzen     (nur Symbol 0 jedes Rahmens)         [8.4.3, Tab. 53-56] -> Rahmensynchronisation
 *      Verstaerkungsreferenzen "Gain" (Gitter x*y)             [8.4.4, Tab. 58-60] -> Kanalschaetzung
 *      FAC-Zellen (65 je Rahmen)                               [8.5.2.1, Tab. 62 (A), 63 (B), 64 (C), 65 (D)]
 *      SDC-Zellen (Anfang jedes Superframes)                   [8.5.3; Anzahl 7.5.2 Tab. 25]
 *      Datenzellen (MSC)                                       [8.6]
 *      ungenutzte Traeger (DC k = 0; bei Mode A auch k = +-1)  [8.3.1, Tab. 50]
 *
 * Alle Pilotzellen tragen Referenzwerte  a * exp(j*2*pi*phi/1024)  mit bekannter Phase phi (in 1/1024 Umlauf)
 * und Amplitude a = sqrt(2) (Leistungsgewinn 2), an den Bandkanten a = 2 (Leistungsgewinn 4) [8.4.4.2, Tab. 59].
 * Der Empfaenger kennt alle diese Werte und kann daher aus dem Verhaeltnis (empfangen / bekannt) den Kanal
 * (Daempfung + Phasendrehung) je Pilotposition messen und dazwischen interpolieren.
 *
 * Die Gain-Pilotphasen folgen der Formel  theta = 4*Z[n][m] + p*W[n][m] + p^2*(1+s)*Q   (mod 1024)
 * [8.4.4.3.1], mit n = s mod y, m = s div y, p = Gitterindex; W, Z, Q stehen je Modus in [8.4.4.3.2 - 8.4.4.3.5].
 * ========================================================================================================== */
#include "drm_mode.h"
#include <math.h>
#include <string.h>

/* Eintrag einer Referenzzelle: Traegerindex k und Phase in Einheiten von 2*pi/1024 */
typedef struct { int16_t k; uint16_t ph; } kp_t;
/* Frequenzreferenzen [8.4.2.1 Tab. 51 Traegernummern, 8.4.2.2 Tab. 52 Phasen]: Traeger bei 750, 2250 und 3000 Hz.  Zeilen: Mode A, B, C, D. */
static const kp_t fr_tab[4][3] = {
 {{18,205},{54,836},{72,215}}, {{16,331},{48,651},{64,555}}, {{11,214},{33,392},{44,242}}, {{7,788},{21,1014},{28,332}} };
/* Zeitreferenzen im ersten Symbol jedes Rahmens [8.4.3.1, Tab. 53 (A), 54 (B), 55 (C), 56 (D)]; Mode A, B, C, D */
static const kp_t tr_A[] = {{17,973},{18,205},{19,717},{21,264},{28,357},{29,357},{32,952},{33,440},{39,856},{40,88},{41,88},{53,68},{54,836},{55,836},{56,836},{60,1008},{61,1008},{63,752},{71,215},{72,215},{73,727}};
static const kp_t tr_B[] = {{14,304},{16,331},{18,108},{20,620},{24,192},{26,704},{32,44},{36,432},{42,588},{44,844},{48,651},{49,651},{50,651},{54,460},{56,460},{62,944},{64,555},{66,940},{68,428}};
static const kp_t tr_C[] = {{8,722},{10,466},{11,214},{12,214},{14,479},{16,516},{18,260},{22,577},{24,662},{28,3},{30,771},{32,392},{33,392},{36,37},{38,37},{42,474},{44,242},{45,242},{46,754}};
static const kp_t tr_D[] = {{5,636},{6,124},{7,788},{8,788},{9,200},{11,688},{12,152},{14,920},{15,920},{17,644},{18,388},{20,652},{21,1014},{23,176},{24,176},{26,752},{27,496},{28,332},{29,432},{30,964},{32,452}};
static const struct { const kp_t *t; int n; } tr_tab[4] = {{tr_A, sizeof tr_A/sizeof *tr_A}, {tr_B, sizeof tr_B/sizeof *tr_B}, {tr_C, sizeof tr_C/sizeof *tr_C}, {tr_D, sizeof tr_D/sizeof *tr_D}};

/* Verstaerkungsreferenzen [8.4.4.1, Tab. 60]: Gitter k = k0 + x*n + x*y*p  im Symbol s (n = s mod y).
 * x = Traegerabstand zwischen den Zeilen, y = Symbolperiode des Musters, Q = quadratischer Phasenanteil [8.4.4.3]. */
static const int gx[4] = {4,2,2,1}, gy[4] = {5,3,2,3}, gk0[4] = {2,1,1,1}, gQ[4] = {36,12,12,14}, gnm[4] = {3,5,10,8};
/* Phasentabellen W (in 1/1024), Z (in 1/256) je Modus [8.4.4.3.2 (A), .3 (B), .4 (C), .5 (D)] */
static const uint16_t W_A[5][3] = {{228,341,455},{455,569,683},{683,796,910},{910,0,114},{114,228,341}};
static const uint8_t  Z_A[5][3] = {{0,81,248},{18,106,106},{122,116,31},{129,129,39},{33,32,111}};
static const uint16_t W_B[3][5] = {{512,0,512,0,512},{0,512,0,512,0},{512,0,512,0,512}};
static const uint8_t  Z_B[3][5] = {{0,57,164,64,12},{168,255,161,106,118},{25,232,132,233,38}};
static const uint16_t W_C[2][10] = {{465,372,279,186,93,0,931,838,745,652},{931,838,745,652,559,465,372,279,186,93}};
static const uint8_t  Z_C[2][10] = {{0,76,29,76,9,190,161,248,33,108},{179,178,83,253,127,105,101,198,250,145}};
static const uint16_t W_D[3][8] = {{366,439,512,585,658,731,805,878},{731,805,878,951,0,73,146,219},{73,146,219,293,366,439,512,585}};
static const uint8_t  Z_D[3][8] = {{0,240,17,60,220,38,151,101},{110,7,78,82,175,150,106,25},{165,7,252,124,253,177,197,142}};
static int Wf(int id, int n, int m) { switch (id) { case 0: return W_A[n][m]; case 1: return W_B[n][m]; case 2: return W_C[n][m]; default: return W_D[n][m]; } }
static int Zf(int id, int n, int m) { switch (id) { case 0: return Z_A[n][m]; case 1: return Z_B[n][m]; case 2: return Z_C[n][m]; default: return Z_D[n][m]; } }

/* Traegerbereich kmin..kmax je Modus und Spektrumbelegung 0..3 [8.3.1, Tab. 49; Belegungsnummern Tab. 48]; (0,0) = in dieser Edition nicht definiert */
static const int kr[4][4][2] = {   /* [Modus][Belegung] = {kmin,kmax}; Belegung 0..3 */
 {{2,102},{2,114},{-102,102},{-114,114}}, {{1,91},{1,103},{-91,91},{-103,103}},
 {{0,0},{0,0},{0,0},{-69,69}}, {{0,0},{0,0},{0,0},{-44,44}} };
/* Randtraeger mit erhoehter Pilotamplitude (Leistungsgewinn 4 statt 2) [8.4.4.2, Tab. 59] */
static const int boost_tab[4][4][4] = {
 {{2,6,98,102},{2,6,110,114},{-102,-98,98,102},{-114,-110,110,114}},
 {{1,3,89,91},{1,3,101,103},{-91,-89,89,91},{-103,-101,101,103}},
 {{0},{0},{0},{-69,-67,67,69}}, {{0},{0},{0},{-44,-43,43,44}} };
/* Zellzahlen: N_SDC je SDC-Block [7.5.2, Tab. 25] und N_MUX je MSC-Multiplexrahmen [7.7, Tab. 41-45] */
static const int nsdc_tab[4][4] = {{167,190,359,405},{130,150,282,322},{0,0,0,288},{0,0,0,152}};
static const int nmux_tab[4][4] = {{1259,1422,2632,2959},{966,1110,2051,2337},{0,0,0,1844},{0,0,0,1226}};
static const int nsym_tab[4] = {15,15,20,24};   /* Symbole je 400-ms-Uebertragungsrahmen [8.2, Tab. 47] */
/* Nutzsymbol Tu und Guardintervall Tg in Abtastwerten bei 12 kHz [8.2, Tab. 47]:
 * A 24 ms + 2,67 ms, B 21,33 ms + 5,33 ms, C 14,67 ms + 5,33 ms, D 9,33 ms + 7,33 ms. */
static const int TU[4] = {288,256,176,112}, TG[4] = {32,64,64,88};

/* FAC-Zellpositionen je Symbol [8.5.2, Tab. 62-65]: je Symbol bis zu 8 Traeger, 0 = Ende; Symbole ab erstem FAC-Symbol */
static const int8_t facA[15][9] = {{0},{0},{26,46,66,86},{10,30,50,70,90},{14,22,34,62,74,94},{26,38,58,66,78},{22,30,42,62,70,82},{26,34,46,66,74,86},{10,30,38,50,58,70,78,90},{14,22,34,42,62,74,82,94},{26,38,46,66,86},{10,30,50,70,90},{14,34,74,94},{38,58,78},{0}};
static const int8_t facB[15][9] = {{0},{0},{13,25,43,55,67},{15,27,45,57,69},{17,29,47,59,71},{19,31,49,61,73},{9,21,33,51,63,75},{11,23,35,53,65,77},{13,25,37,55,67,79},{15,27,39,57,69,81},{17,29,41,59,71,83},{19,31,43,61,73},{21,33,45,63,75},{23,35,47,65,77},{0}};
static const int8_t facC[20][9] = {{0},{0},{0},{9,21,45,57},{23,35,47},{13,25,37,49},{15,27,39,51},{5,17,29,41,53},{7,19,31,43,55},{9,21,45,57},{23,35,47},{13,25,37,49},{15,27,39,51},{5,17,29,41,53},{7,19,31,43,55},{9,21,45,57},{23,35,47},{13,25,37,49},{15,27,39,51},{0}};
static const int8_t facD[24][9] = {{0},{0},{0},{9,18,27},{10,19},{11,20,29},{12,30},{13,22,31},{5,14,23,32},{6,15,24,33},{16,25,34},{8,17,26,35},{9,18,27,36},{10,19,37},{11,20,29},{12,30},{13,22,31},{5,14,23,32},{6,15,24,33},{16,25,34},{8,17,26,35},{9,18,27,36},{10,19,37},{0}};

const char *drm_mode_name(int id) { static const char *n[4] = {"A","B","C","D"}; return (id >= 0 && id < 4) ? n[id] : "?"; }
#define AT(m,s,k) ((m)->map[s][(k)-(m)->kmin])

int drm_mode_setup(drm_mode_t *m, int id, int occ)
{
    /* Baut das komplette Zellraster map[s][k] auf.  Reihenfolge ist wichtig: erst alles als Daten markieren, dann
     * Pilotarten aufpraegen; bei Ueberschneidung haben Frequenz- und Zeitreferenzen Vorrang vor Gain [8.4.4.1]. */
    if (id < 0 || id > 3 || occ < 0 || occ > 3) return -1;
    if (kr[id][occ][1] == 0) return -1;
    memset(m, 0, sizeof *m);
    m->id = id; m->occ = occ; m->Tu = TU[id]; m->Tg = TG[id]; m->Ts = m->Tu + m->Tg; m->nsym = nsym_tab[id];
    m->kmin = kr[id][occ][0]; m->kmax = kr[id][occ][1]; m->ncar = m->kmax - m->kmin + 1;
    m->spacing_hz = (float)DRM_FS / m->Tu; m->nsdc_sym = (id <= 1) ? 2 : 3;
    m->nsdc = nsdc_tab[id][occ]; m->nmux = nmux_tab[id][occ];
    for (int s = 0; s < m->nsym; s++) for (int k = m->kmin; k <= m->kmax; k++) {
        int unused = (k == 0) || (id == 0 && (k == -1 || k == 1));
        AT(m,s,k).type = unused ? CT_UNUSED : CT_DATA;
    }
    for (int s = 0; s < m->nsym; s++) for (int i = 0; i < 3; i++) {
        int k = fr_tab[id][i].k; if (k < m->kmin || k > m->kmax) continue;
        AT(m,s,k).type = CT_FREQ; AT(m,s,k).ph1024 = fr_tab[id][i].ph;
    }
    for (int i = 0; i < tr_tab[id].n; i++) {
        int k = tr_tab[id].t[i].k; if (k < m->kmin || k > m->kmax) continue;
        AT(m,0,k).type = CT_TIME; AT(m,0,k).ph1024 = tr_tab[id].t[i].ph;
    }
    int x = gx[id], y = gy[id], k0 = gk0[id];
    for (int s = 0; s < m->nsym; s++) {
        int n = s % y, mm = s / y; if (mm >= gnm[id]) mm = gnm[id]-1;
        for (int k = m->kmin; k <= m->kmax; k++) {
            int d = k - k0 - n*x; if (d % (x*y) != 0) continue;
            drm_cellinfo_t *c = &AT(m,s,k);
            if (c->type == CT_FREQ || c->type == CT_TIME) continue;   /* Vorrang (8.4.4.3.0) */
            int p = d / (x*y);
            long th = 4L*Zf(id,n,mm) + (long)p*Wf(id,n,mm) + (long)p*p*(1+s)*gQ[id];
            th %= 1024; if (th < 0) th += 1024;
            c->type = CT_GAIN; c->ph1024 = (uint16_t)th;
            for (int b = 0; b < 4; b++) if (boost_tab[id][occ][b] == k && k != 0) c->boost = 1;
        }
    }
    for (int s = 0; s < m->nsym; s++) {
        const int8_t *f = (id == 0) ? facA[s] : (id == 1) ? facB[s] : (id == 2) ? facC[s] : facD[s];
        for (int i = 0; f[i] && i < 9; i++) {
            if (f[i] < m->kmin || f[i] > m->kmax) continue;
            drm_cellinfo_t *c = &AT(m,s,f[i]); if (c->type == CT_DATA) c->type = CT_FAC;
        }
    }
    return 0;
}

/* Referenzwert einer Pilotzelle als komplexe Zahl a*exp(j*phi) [8.4.2.2, 8.4.3.2, 8.4.4.2]. */
void drm_ref_value(const drm_mode_t *m, int s, int k, float *re, float *im)
{
    const drm_cellinfo_t *c = &m->map[s][k - m->kmin];
    float a = c->boost ? 2.0f : 1.41421356f, ph = 2.0f*(float)M_PI*c->ph1024/1024.0f;
    *re = a*cosf(ph); *im = a*sinf(ph);
}
