#ifndef DRM_MSC_H
#define DRM_MSC_H
#include "drm_cfg.h"
/* MSC Mode B / 10 kHz / 16-QAM SM / EEP, Schutzstufe 0 (R0=1/3, R1=2/3)  (Tab. 30, 42) */
#define MSC_NMUX   2337
#define MSC_LBITS  4662                 /* 2*NMUX-12 = 1554 (Ebene 0) + 3108 (Ebene 1) */
#define MSC_D      5                    /* lange Verschachtelung */
typedef struct { float zr, zi, w; } msc_cell_t;      /* entzerrte Zelle, w=|H|^2 */

/* Zell-Deinterleaver (lang, D=5): frames zeigt auf 5 aufeinanderfolgende empfangene
 * Multiplexrahmen [q .. q+4] (je MSC_NMUX Zellen) -> out = Rahmen q in Originalreihenfolge. */
void drm_msc_deinterleave(const msc_cell_t *frames, msc_cell_t *out);
/* Multilevel-Dekodierung (Ebene 0 -> Hard-Reencode -> Ebene 1), liefert MSC_LBITS Bits
 * nach Energy-Dispersal. Rueckgabe: Anteil der uebereinstimmenden Ebene-0-Bits
 * (Reencode vs. Empfang) in Promille als grobe Guete. */
int  drm_msc_decode(const msc_cell_t *cells, uint8_t *bits);
#endif
