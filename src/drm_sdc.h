/* drm_sdc.h -- Service Description Channel: Dekodierung und Auswertung der wichtigsten Datenentitaeten. */
#ifndef DRM_SDC_H
#define DRM_SDC_H
#include "drm_cfg.h"
#include "drm_mode.h"
#include "drm_msc.h"

typedef struct {
    int crc_ok, afs_index;
    /* Entitaet Typ 0 "Multiplex description" [6.4.3.1]: Schutzstufen und Laenge je Datenstrom (Byte) */
    int nstreams, prot_a, prot_b, len_a[4], len_b[4];
    /* Entitaet Typ 1 "Label" [6.4.3.2]: Programmname je Short-Id (UTF-8) */
    char label[4][65];
    /* Entitaet Typ 9 "Audio information" [6.4.3.10] */
    int have_audio, a_short, a_stream, a_coding, a_sbr, a_mode, a_rate, a_text, a_enh, a_cfglen;
    uint8_t a_cfg[24];                    /* xHE-AAC: Static Config [5.3.2] (a_coding == 3) */
} drm_sdc_t;

/* cells: m->nsdc entzerrte QAM-Zellen (je SDC-Symbol Traeger aufsteigend); sdc_mode = FAC-Feld (1 = 4-QAM, 0 = 16-QAM).
 * Rueckgabe 0 = dekodiert (crc_ok pruefen), -1 = Modus nicht unterstuetzt. */
int drm_sdc_decode(const drm_mode_t *m, int sdc_mode, const msc_cell_t *cells, drm_sdc_t *s);
/* Fuehrt die Entitaeten mehrerer SDC-Bloecke zusammen (Typ 0/1/9 koennen auf verschiedene Bloecke verteilt sein). */
void drm_sdc_merge(drm_sdc_t *dst, const drm_sdc_t *src);

/* Sender-Seite fuer Selbsttests: SDC-Datenfeld (AFS-Index + Daten + CRC bereits enthalten, N_SDC-6 Bits) -> N_SDC 4-QAM-Zellen. */
void drm_sdc_encode4(const drm_mode_t *m, const uint8_t *info_bits, float *cells_re, float *cells_im);
/* Baut aus Datenfeldbits (4 Bit AFS-Index + 8*nbytes Datenbit) die CRC-16 und die Gesamtbits (nur 4-QAM, EEP). Liefert nbytes. */
int  drm_sdc_nbytes(const drm_mode_t *m, int sdc_mode);
void drm_sdc_finish(const drm_mode_t *m, int sdc_mode, uint8_t *bits);
#endif
