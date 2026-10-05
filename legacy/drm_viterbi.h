#ifndef DRM_VITERBI_H
#define DRM_VITERBI_H
#include "drm_cfg.h"
/* Soft-Decision-Viterbi fuer den DRM-Mutter-Code (R=1/6, K=7, Polynome 133,171,145,133,171,145 oktal).
 * soft[6*i+j]: Eingangswert fuer Bit b_j des Schritts i; >0 = Bit 0 wahrscheinlicher, 0 = punktiert.
 * nsteps Schritte inkl. 6 Tail-Bits. dec: Puffer mit nsteps Eintraegen (uint64). Terminiert in Zustand 0.
 * out[i] = a_i (nsteps Bits, eines je Byte).                                                   */
void drm_viterbi(const int8_t *soft, int nsteps, uint64_t *dec, uint8_t *out);
void drm_conv_encode(const uint8_t *a, int n, uint8_t *b);
#endif
