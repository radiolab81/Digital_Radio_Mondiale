/* drm_ofdm.h -- OFDM-Demodulator: Moduserkennung/Erstsynchronisation, Mischer, Fensterung, FFT. */
#ifndef DRM_OFDM_H
#define DRM_OFDM_H
#include "drm_cfg.h"
#include "drm_mode.h"
#include "drm_fft.h"

typedef struct {
    const drm_mode_t *m;           /* aktueller Robustheitsmodus / Belegung */
    drm_fft_plan_t plan;           /* FFT-Plan der Laenge Tu */
    uint32_t nco_phase;            /* Phase des Mischers (32 Bit = ein Umlauf), laeuft ueber die Symbole weiter */
    int32_t  nco_inc;              /* Phasenschritt je Abtastwert = Frequenzoffset * 2^32 / fs */
    int      fft_adv;              /* FFT-Fenster liegt fft_adv Abtastwerte vor dem Ende des Guardintervalls */
    float    foff_hz;              /* aktuell eingestellter Frequenzoffset */
} drm_ofdm_t;

/* Initialisiert Mischer-Tabelle, FFT-Plan und Standardwerte. */
void drm_ofdm_init(drm_ofdm_t *o, const drm_mode_t *m);
/* Stellt den Mischer ein: ein Signal bei +hz wird nach 0 Hz verschoben. */
void drm_ofdm_set_freq(drm_ofdm_t *o, float hz);

/* Ergebnis der Erstsynchronisation ueber die Guard-Korrelation */
typedef struct { int mode, t0; float foff_hz, metric, score[4]; } drm_acq_t;
/* Erkennt Robustheitsmodus (0..3 = A..D), Symboltakt t0 (0..Ts-1) und Bruchteil-Frequenzoffset aus nblocks
 * Symbollaengen Signal.  Rueckgabe: Modus oder -1 ("kein DRM-Signal"). Benoetigt (nblocks+1)*Ts+Tu Abtastwerte. */
int drm_acquire(const drm_cs16_t *iq, int n, int nblocks, drm_acq_t *r);

/* Ein OFDM-Symbol demodulieren: iq zeigt auf den Beginn des Guardintervalls, frac (0..1) ist ein Bruchteil-
 * Abtastwert, um den das FFT-Fenster spaeter liegt (bandbegrenzte Interpolation, 24 Taps).  Es werden 11 Abtast-
 * werte Vorlauf und 12 Nachlauf vor/nach dem Symbol benoetigt.  cells erhaelt die m->ncar Spektralwerte der
 * Traeger kmin..kmax (noch nicht entzerrt). */
void drm_ofdm_symbol(drm_ofdm_t *o, const drm_cs16_t *iq, float frac, drm_cell_t *cells);
#define DRM_OFDM_PRE   11
#define DRM_OFDM_POST  12
#endif
