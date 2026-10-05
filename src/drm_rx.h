/* drm_rx.h -- Streaming-Empfaenger: I/Q-Abtastwerte hinein, Ereignisse (FAC, SDC, Audio-Rahmen) heraus.
 *
 * Zustandsautomat:  ACQ (Modus, Symboltakt, Feinfrequenz)  ->  SYNC (Rahmenphase, ganzzahliger Traegerversatz)
 *                   ->  TRACK (Nachfuehrung von Zeit und Frequenz; FAC, SDC, MSC, Audio).
 * Bei anhaltendem FAC-Verlust faellt er nach ACQ zurueck.  Aller Speicher liegt in drm_rx_t (statisch anlegen!);
 * es gibt kein malloc.  Typische Groesse ca. 600 kB (Mode A, 10 kHz); der Hauptanteil sind die MSC-Zellen fuer die
 * 2 s lange Zeitverschachtelung (5 Rahmen x 2959 Zellen x 12 Byte).  Fuer MCU-Ports laesst sich dieser Speicher durch
 * Ablegen der Zellen als int16 halbieren (siehe README, Abschnitt Portierung). */
#ifndef DRM_RX_H
#define DRM_RX_H
#include "drm_cfg.h"
#include "drm_fac.h"
#include "drm_sdc.h"

enum { DRM_ST_ACQ = 0, DRM_ST_SYNC, DRM_ST_TRACK };
enum { DRM_EV_STATE = 0,   /* data = NULL, len = neuer Zustand                                              */
       DRM_EV_FAC,         /* data = drm_fac_t*   (nur CRC-gueltige)                                        */
       DRM_EV_SDC,         /* data = drm_sdc_t*   (zusammengefuehrte Entitaeten aller bisher gueltigen Bloecke) */
       DRM_EV_AUDIO };     /* data = Rahmen, len > 0 gueltig | 0 CRC-Fehler | -1 Superframe verloren          */

typedef void (*drm_event_cb)(void *user, int type, const void *data, int len);

typedef struct drm_rx drm_rx_t;
size_t    drm_rx_size(void);                                         /* benoetigte Speichergroesse in Byte */
drm_rx_t *drm_rx_init(void *mem, size_t memsize);                    /* mem >= drm_rx_size(), 16-Byte-ausgerichtet */
void      drm_rx_set_callback(drm_rx_t *rx, drm_event_cb cb, void *user);
void      drm_rx_push(drm_rx_t *rx, const drm_cs16_t *iq, int n);    /* beliebig lange Bloecke 12 kHz komplex */
int       drm_rx_state(const drm_rx_t *rx);

typedef struct {
    int mode, occ, ncar;                       /* erkannter Modus (0..3 = A..D), Belegung (aus FAC), Traegerzahl */
    int fac_ok, fac_n, sdc_ok, sdc_n, hdr_ok, hdr_n, frames_ok, frames_bad, frames_lost, reacq;
    float foff_hz, pilot_snr_db;               /* Frequenzoffset; Daten-SNR aus den Frequenzpiloten */
    int msc_qam, msc_levels, prot_a, prot_b, depth;
} drm_stats_t;
void drm_rx_stats(const drm_rx_t *rx, drm_stats_t *s);

/* Zugriff auf Audio-Konfiguration (nur gueltig, wenn ein SDC mit Audioentitaet gesehen wurde) */
const drm_sdc_t *drm_rx_sdc(const drm_rx_t *rx);
const drm_fac_t *drm_rx_fac(const drm_rx_t *rx);
#endif
