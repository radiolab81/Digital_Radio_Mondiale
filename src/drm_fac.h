/* drm_fac.h -- Fast Access Channel (FAC): Dekodierung der 65 Zellen eines Uebertragungsrahmens
 *              [ES 201 980, 6.3 Inhalt, 7.5.3 Codierung, 8.5.2 Zellpositionen]. */
#ifndef DRM_FAC_H
#define DRM_FAC_H
#include "drm_cfg.h"

typedef struct {
    int crc_ok;                          /* 1 = CRC-8 stimmt */
    int identity;                        /* 0..3: Position des Rahmens im Superframe */
    int rm_flag;                         /* 0: Modi A-D, 1: Mode E */
    int spectrum_occ;                    /* Spektrumbelegung 0..5 (4,5 / 5 / 9 / 10 / 18 / 20 kHz) */
    int interleaver_depth;               /* 0: lang (2 s), 1: kurz (400 ms) */
    int msc_mode;                        /* 0: 64-QAM, 3: 16-QAM (1, 2 reserviert) */
    int sdc_mode;                        /* 0: 16-QAM, 1: 4-QAM */
    int nservices, reconf, toggle;       /* Dienstzahl, Rekonfigurationsanzeige, Toggle-Bit */
    uint32_t service_id;                 /* Kennung des ersten Dienstes */
    int short_id, audio_ca, language, audio_data, descriptor;   /* Dienstparameter (audio_data: 0 Audio / 1 Daten) */
    uint8_t bits[72];                    /* alle 72 entwuerfelten Bits (64 Daten + 8 CRC) */
} drm_fac_t;

/* soft: 130 Werte (65 Zellen x {Re, Im}, > 0 => Bit 0) in Uebertragungsreihenfolge, entzerrt und mit |H|^2 gewichtet.
 * Rueckgabe: crc_ok. */
int drm_fac_decode(const int8_t *soft130, drm_fac_t *f);

/* Sender-Seite fuer Selbsttests (drm_gen): 64 Datenbits -> CRC-8 anhaengen, verwischen, kodieren, verschachteln,
 * auf 65 4-QAM-Zellen (Einheitsleistung) abbilden. */
void drm_fac_encode(const uint8_t *data64, float *cells_re, float *cells_im);
#endif
