/* ==========================================================================================================
 * drm_fft.c  --  Mixed-Radix-FFT (Cooley-Tukey, "decimation in time", rekursiv)
 * ==========================================================================================================
 * Warum Mixed-Radix?  Die Nutzsymbolinhalte der Robustheitsmodi haben die Laengen
 *      Mode A: 288 = 2^5 * 3^2     Mode B: 256 = 2^8     Mode C: 176 = 2^4 * 11     Mode D: 112 = 2^4 * 7
 * [ES 201 980, 8.2, Tab. 44], also keine reinen Zweierpotenzen.  Der Algorithmus zerlegt N = p * m (p = kleinster
 * Primfaktor), berechnet p Teil-DFTs der Laenge m und setzt sie mit Drehfaktoren ("Twiddle") zusammen:
 *
 *      X[k + q*m] = sum_{r=0}^{p-1} W_N^{r*k} * W_p^{r*q} * X_r[k]      (k = 0..m-1, q = 0..p-1)
 *
 * X_r ist die DFT der Teilfolge x[r], x[r+p], x[r+2p], ...  W_N^i = exp(-j*2*pi*i/N).
 * Fuer einen MCU-Port ist dies (einmal je Symbol, 26 ms) unkritisch: Mode B braucht ~ 20 kFLOP je Symbol.
 * Eine feste Radix-2/4-Variante in Q15 liegt in legacy/ (Stufe 1) und ist dort gegen diese Fassung geprueft.
 * ========================================================================================================== */
#include "drm_fft.h"
#include <math.h>

int drm_fft_plan(drm_fft_plan_t *p, int n)
{
    if (n < 2 || n > DRM_MAXTU) return -1;
    int r = n;                                       /* nur Primfaktoren 2,3,5,7,11 zulassen */
    for (int f = 2; f <= 11; f++) while (r % f == 0) r /= f;
    if (r != 1) return -1;
    p->n = n;
    for (int i = 0; i < n; i++) {                    /* W_N^i, einmal vorberechnet (Tabelle statt sin/cos im Betrieb) */
        double a = -2.0 * M_PI * i / n;
        p->wr[i] = (float)cos(a);
        p->wi[i] = (float)sin(a);
    }
    return 0;
}

/* Rekursionsschritt: DFT der n Werte in[0], in[stride], in[2*stride], ... nach out[0..n-1]. */
static void rec(const drm_fft_plan_t *P, const drm_cell_t *in, drm_cell_t *out, int n, int stride)
{
    if (n == 1) { out[0] = in[0]; return; }
    int p = 2; while (n % p) p++;                    /* kleinster Primfaktor = Radix dieses Schritts */
    int m = n / p;
    for (int r = 0; r < p; r++)                      /* p Teil-DFTs der Laenge m (Eingang jeweils um p*stride gespreizt) */
        rec(P, in + r * stride, out + r * m, m, stride * p);

    const int N = P->n, ws = N / n;                  /* W_n^j = W_N^(j * N/n) */
    drm_cell_t t[11];
    for (int k = 0; k < m; k++) {
        for (int r = 0; r < p; r++) {                /* Twiddle: t[r] = W_n^(r*k) * X_r[k] */
            int idx = (r * k * ws) % N;
            drm_cell_t v = out[r * m + k];
            t[r].re = v.re * P->wr[idx] - v.im * P->wi[idx];
            t[r].im = v.re * P->wi[idx] + v.im * P->wr[idx];
        }
        for (int q = 0; q < p; q++) {                /* kleine p-Punkt-DFT ueber die Twiddle-Werte (Schmetterling) */
            float sr = 0, si = 0;
            for (int r = 0; r < p; r++) {
                int idx = (r * q * m * ws) % N;      /* W_p^(r*q) = W_N^(r*q*N/p) */
                sr += t[r].re * P->wr[idx] - t[r].im * P->wi[idx];
                si += t[r].re * P->wi[idx] + t[r].im * P->wr[idx];
            }
            out[k + q * m].re = sr;
            out[k + q * m].im = si;
        }
    }
}

void drm_fft(const drm_fft_plan_t *p, const drm_cell_t *in, drm_cell_t *out) { rec(p, in, out, p->n, 1); }
