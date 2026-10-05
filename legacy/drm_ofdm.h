#ifndef DRM_OFDM_H
#define DRM_OFDM_H
#include "drm_cfg.h"

typedef struct {
    /* Zustand, komplett statisch -> kein malloc */
    uint32_t nco_phase;      /* 32-Bit-Phasenakku                     */
    int32_t  nco_inc;        /* Phasenschritt je Sample (Frequenzoffs) */
    int      sym_pos;        /* Sample-Index Beginn Guard des naechsten Symbols */
    int      fft_adv;        /* FFT-Fenster liegt fft_adv Samples vor Guard-Ende */
    float    foff_hz;        /* aktuell geschaetzter Frequenzoffset   */
} drm_ofdm_t;

void drm_ofdm_init(drm_ofdm_t *o);

/* Akquisition: findet Symboltakt (0..TS-1) und Fraktional-Offset aus der
 * Guard-Intervall-Korrelation ueber nsym Symbole. Rein Ganzzahl (int64 Akku). */
int  drm_ofdm_acquire(drm_ofdm_t *o, const drm_cs16_t *iq, int n, int nsym,
                      float *foff_hz, float *metric);

void drm_ofdm_set_freq(drm_ofdm_t *o, float hz);

/* Ein Symbol demodulieren: iq zeigt auf Guard-Beginn (relativ zum globalen
 * Index base fuer NCO-Phase ist nicht noetig, NCO laeuft mit).  Schreibt
 * DRM_NCAR Zellen k=KMIN..KMAX nach cells. */
void drm_ofdm_symbol(drm_ofdm_t *o, const drm_cs16_t *iq, drm_cell_t *cells);

/* Nachfuehrung: Guard-Korrelation um die aktuelle Lage (+-2 Samples) */
void drm_ofdm_track(drm_ofdm_t *o, const drm_cs16_t *iq_sym0, int nsym);
#endif
