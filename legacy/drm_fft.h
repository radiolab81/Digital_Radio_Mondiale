#ifndef DRM_FFT_H
#define DRM_FFT_H
#include "drm_cfg.h"
/* In-place FFT, N = DRM_TU (Zweierpotenz).  Ausgabe = X[k]/N im Fixed-Pfad
 * (Skalierung >>1 je Stufe), unskaliert im Float-Pfad.
 * Mode A/C/D (N=288/176/112) brauchen Mixed-Radix (3,7,11) -> Stufe 2.   */
void drm_fft_init(void);
void drm_fft_q15(drm_cs16_t *x);                       /* Ganzzahl  */
void drm_fft_f32(const drm_cs16_t *in, drm_cell_t *out_f32); /* nur !DRM_FIXED */
#endif
