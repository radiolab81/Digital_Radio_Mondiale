/* drm_msc.h -- Kanaldecodierung des MSC (und des 16-QAM-SDC): Mehrebenencode, UEP, Zell-Deinterleaver. */
#ifndef DRM_MSC_H
#define DRM_MSC_H
#include "drm_cfg.h"
#include "drm_mode.h"

#define MSC_MAXN    3000      /* groesstes N_MUX (Mode A, 10 kHz: 2959)                                */
#define MSC_MAXBITS 20000     /* groesstes L_MUX (64-QAM, Mode A 10 kHz, PL3: ca. 13,9 kBit)            */

/* Entzerrte QAM-Zelle:  z = X / H (Sollwert hat Einheitsleistung), w = |H|^2 als Zuverlaessigkeit der Zelle. */
typedef struct { float zr, zi, w; } msc_cell_t;

/* Beschreibung der Kanalcodierung eines Multiplexrahmens bzw. SDC-Blocks (ES 201 980 [7.2.1.1, 7.3]). */
typedef struct {
    int n;                      /* Zellen insgesamt: N_MUX (MSC) oder N_SDC                               */
    int n1, n2;                 /* Zellen des hoeher (Teil A) / niedriger (Teil B) geschuetzten Teils; n1 = 0: EEP */
    int depth;                  /* Zeitverschachtelungstiefe D: 5 (lang, 2 s) oder 1 (kurz, 400 ms)       [7.6] */
    int qam, levels;            /* 16 oder 64; Codierebenen Pmax = 2 bzw. 3                                [7.3.1] */
    int rxA[3], ryA[3];         /* Coderate je Ebene fuer Teil A (aus Schutzstufe PL_A, Tab. 30/32)       */
    int rxB[3], ryB[3];         /* dito Teil B                                                            */
    int M1[3], M2[3], M[3];     /* Infobits je Ebene: Teil A, Teil B, gesamt                             [7.3.1.1] */
    int L1, L2, lbits;          /* Infobits: Teil A, Teil B, gesamt (= L_MUX)                            [7.2.1.1] */
} drm_msc_cfg_t;

/* Allgemeine Konfiguration.  qam = 16/64, n = Zellzahl, plA/plB = Schutzstufen (SDC Typ 0), bytes_a = Laenge des
 * Teils A in Byte (X, 0 = EEP).  Rueckgabe 0 = ok, -1 = unzulaessig. */
int  drm_msc_setup(drm_msc_cfg_t *c, int qam, int n, int depth, int plA, int plB, int bytes_a);
/* Bequemfunktion fuer den MSC: Modus/FAC-Felder -> drm_msc_setup. msc_mode: FAC-Feld (0 = 64-QAM, 3 = 16-QAM). */
int  drm_msc_config(drm_msc_cfg_t *c, const drm_mode_t *m, int msc_mode, int depth_flag, int plA, int plB, int bytes_a);

/* Zell-Deinterleaver ueber D Multiplexrahmen [7.6]: frames[0..D-1] = Zeiger auf D aufeinanderfolgende Rahmen (je n
 * Zellen), AELTESTER ZUERST; out = der aelteste Rahmen in Originalreihenfolge. */
void drm_msc_deinterleave(const drm_msc_cfg_t *c, const msc_cell_t *const *frames, msc_cell_t *out);

/* Mehrstufige Dekodierung (Ebene fuer Ebene, danach drm_msc_iterations Rueckkopplungsdurchlaeufe).  bits erhaelt
 * die c->lbits entwuerfelten Bits in Sendereihenfolge u (Teil A vor Teil B).  Rueckgabe: Anteil der Ebene-0-
 * Entscheidungen, die mit dem Re-Encoding uebereinstimmen, in Promille (grobes Qualitaetsmass). */
extern int drm_msc_iterations;
int  drm_msc_decode(const drm_msc_cfg_t *c, const msc_cell_t *cells, uint8_t *bits);

/* Sender-Seite fuer Selbsttests (drm_sim): bits -> n Zellen mit Einheitsleistung, ohne Zell-Interleaver. */
int  drm_msc_encode(const drm_msc_cfg_t *c, const uint8_t *bits, msc_cell_t *cells);
/* Zell-Interleaver Senderseite [7.6]: hist[g] = Eingangsrahmen (aktueller - g), g = 0..D-1; out = zu sendender Rahmen. */
void drm_msc_interleave(const drm_msc_cfg_t *c, const msc_cell_t *const *hist, msc_cell_t *out);
#endif
