/* drm_fft.h -- Diskrete Fourier-Transformation fuer die OFDM-Demodulation (vgl. drm_fft.c). */
#ifndef DRM_FFT_H
#define DRM_FFT_H
#include "drm_cfg.h"

typedef struct { int n; float wr[DRM_MAXTU], wi[DRM_MAXTU]; } drm_fft_plan_t;   /* Drehfaktoren W_N^i */

/* Plant eine FFT der Laenge n (n = 2^a * 3^b * 7^c * 11^d, hier 288/256/176/112).  Rueckgabe 0 = ok. */
int  drm_fft_plan(drm_fft_plan_t *p, int n);
/* out[k] = sum_n in[n] * exp(-j*2*pi*n*k/N), unskaliert.  in und out duerfen nicht ueberlappen. */
void drm_fft(const drm_fft_plan_t *p, const drm_cell_t *in, drm_cell_t *out);
#endif
