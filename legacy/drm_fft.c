#include "drm_fft.h"
#include <math.h>

#define N DRM_TU
#define LOG2N 8
static int16_t tw_re[N/2], tw_im[N/2];   /* exp(-j2*pi*i/N) in Q15 */
static uint16_t brev[N];
static float twf_re[N/2], twf_im[N/2];

void drm_fft_init(void)
{
    for (int i = 0; i < N/2; i++) {
        double a = -2.0*M_PI*i/N;
        double c = cos(a), s = sin(a);
        twf_re[i] = (float)c; twf_im[i] = (float)s;
        long cr = lround(c*32767.0), ci = lround(s*32767.0);
        tw_re[i] = (int16_t)cr; tw_im[i] = (int16_t)ci;
    }
    for (int i = 0; i < N; i++) {
        unsigned r = 0;
        for (int b = 0; b < LOG2N; b++) if (i & (1<<b)) r |= 1u << (LOG2N-1-b);
        brev[i] = (uint16_t)r;
    }
}

void drm_fft_q15(drm_cs16_t *x)
{
    for (int i = 0; i < N; i++) {
        int j = brev[i];
        if (j > i) { drm_cs16_t t = x[i]; x[i] = x[j]; x[j] = t; }
    }
    for (int len = 2; len <= N; len <<= 1) {
        int half = len >> 1, step = N/len;
        for (int i = 0; i < N; i += len) {
            for (int j = 0; j < half; j++) {
                int32_t wr = tw_re[j*step], wi = tw_im[j*step];
                drm_cs16_t *a = &x[i+j], *b = &x[i+j+half];
                int32_t tr = ((int32_t)b->re*wr - (int32_t)b->im*wi + 16384) >> 15;
                int32_t ti = ((int32_t)b->re*wi + (int32_t)b->im*wr + 16384) >> 15;
                int32_t ar = a->re, ai = a->im;
                a->re = (int16_t)((ar + tr) >> 1); a->im = (int16_t)((ai + ti) >> 1);
                b->re = (int16_t)((ar - tr) >> 1); b->im = (int16_t)((ai - ti) >> 1);
            }
        }
    }
}

void drm_fft_f32(const drm_cs16_t *in, drm_cell_t *o)
{
#ifndef DRM_FIXED
    for (int i = 0; i < N; i++) { int j = brev[i]; o[j].re = in[i].re; o[j].im = in[i].im; }
    for (int len = 2; len <= N; len <<= 1) {
        int half = len >> 1, step = N/len;
        for (int i = 0; i < N; i += len)
            for (int j = 0; j < half; j++) {
                float wr = twf_re[j*step], wi = twf_im[j*step];
                drm_cell_t *a = &o[i+j], *b = &o[i+j+half];
                float tr = b->re*wr - b->im*wi, ti = b->re*wi + b->im*wr;
                float ar = a->re, ai = a->im;
                a->re = ar+tr; a->im = ai+ti; b->re = ar-tr; b->im = ai-ti;
            }
    }
#else
    (void)in; (void)o;
#endif
}
