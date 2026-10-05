/* drm_chan.h -- Pilotbasierte Kanalschaetzung, Rahmensynchronisation und Nachfuehrungsgroessen. */
#ifndef DRM_CHAN_H
#define DRM_CHAN_H
#include "drm_cfg.h"
#include "drm_mode.h"

#define DRM_TW 5                 /* Zeitfenster der Kanalschaetzung in Symbolen (Mitte = DRM_TW/2) */

/* Fensterversatz herausrechnen: x = cells * exp(+j*2*pi*k*adv/Tu)  (xr/xi: m->ncar Werte). */
void drm_rotate_cells(const drm_mode_t *m, const drm_cell_t *cells, int adv, float *xr, float *xi);

/* Rahmensynchronisation [8.4.3]: Kohaerenz der Zeitreferenzzellen eines Symbols, wenn dieses das erste Symbol
 * eines Rahmens waere (shift = angenommener ganzzahliger Traegerversatz).  Nahe 1 = Treffer. */
float drm_time_ref_metric(const drm_mode_t *m, const drm_cell_t *cells, int adv, int shift);

/* Nachfuehrungsgroessen aus den Piloten eines Symbols s (nach drm_rotate_cells):
 *   slope : mittlere Phasensteigung der Kanalschaetzung ueber k in rad/Traeger  (-> Zeitfehler)
 *   fe[6] : E = X/U der drei Frequenzpiloten (Re, Im je Pilot)                   (-> Frequenzfehler) */
void drm_pilot_stats(const drm_mode_t *m, const float *xr, const float *xi, int s, float *slope, float *fe);

/* Zeit-Frequenz-Kanalschaetzung fuer das mittlere von DRM_TW Symbolen.  sph[w] = Rahmenphase (Symbol im Rahmen) des
 * Fensterindex w.  smooth != 0 glaettet die Pilotschaetzungen zusaetzlich ueber Nachbartraeger (bei niedrigem SNR
 * nuetzlich).  hr/hi: geschaetzter Kanal H je Traeger. */
void drm_chan_est_tf(const drm_mode_t *m, float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], int smooth, float *hr, float *hi);
#endif
