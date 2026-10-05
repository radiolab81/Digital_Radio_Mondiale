#ifndef DRM_RX_H
#define DRM_RX_H
#include "drm_cfg.h"
#include "drm_modeb.h"
/* Rahmensynchronisation ueber Zeitreferenzzellen. acc[15] wird pro Symbol mit
 * der Kohaerenz der Zeitreferenzzellen fuer die Hypothese "dieses Symbol = s0" gefuettert. */
float drm_time_ref_metric(const drm_cell_t *cells, int adv, int shift);
/* Kanalschaetzung aus Piloten des Symbols s (Interpolation in Polarform ueber k).
 * Ausgabe: H[k-KMIN] (re,im) und Zellen mit Timing-Vorlauf-Kompensation (xr). */
void drm_chan_est(const drm_cell_t *cells, int s, int fft_adv, float *xr, float *xi, float *hr, float *hi);
/* Pilot-Statistik eines Symbols (nach drm_chan_est): mittlere Phasensteigung ueber k (rad/Traeger)
 * aus Gain-Piloten im Abstand 6, und E=X/U der drei Frequenzpiloten (6 floats: re,im je Pilot). */
void drm_pilot_stats(const float *xr, const float *xi, int s, float *slope, float *fe);
/* Zeit-Frequenz-Kanalschaetzung: Fenster aus DRM_TW Symbolen (Mitte = Index DRM_TW/2), xr/xi bereits
 * timing-kompensiert (aus drm_chan_est), sph[i] = Rahmenphase s des Symbols i. */
#define DRM_TW 5   /* Zeitfenster +-5 Symbole */
void drm_chan_est_tf(float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], float *hr, float *hi);
void drm_rotate_cells(const drm_cell_t *cells, int adv, float *xr, float *xi);
#endif
