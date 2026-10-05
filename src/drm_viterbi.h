/* drm_viterbi.h -- Faltungscode der DRM-Kanalcodierung: Dekoder (Viterbi) und Kodierer (fuer Selbsttests). */
#ifndef DRM_VITERBI_H
#define DRM_VITERBI_H
#include "drm_cfg.h"

/* Soft-Decision-Viterbi fuer den Mutter-Code (Rate 1/6, K = 7, Generatoren 133 171 145 133 171 145 oktal).
 *   soft[6*i+j] : Eingangswert fuer Codebit b_j des Schritts i;  > 0: Bit 0 wahrscheinlicher, < 0: Bit 1,
 *                 0 = nicht uebertragen (punktiert) -> traegt nichts zur Entscheidung bei.
 *   n           : Anzahl Trellisschritte inklusive der 6 Tailbit-Schritte.
 *   dec         : Arbeitsfeld mit n Eintraegen (Entscheidungsbits je Zustand).
 *   out[i]      : decodiertes Informationsbit a_i (n Werte; die letzten 6 sind die Tailbits = 0).
 * Der Trellis ist terminiert (Start- und Endzustand 0). */
void drm_viterbi(const int8_t *soft, int n, uint64_t *dec, uint8_t *out);

/* Mutter-Code-Kodierer: n Informationsbits a[] -> n+6 Codeworte b[] (Bit j des Bytes = Codebit b_j). */
void drm_conv_encode(const uint8_t *a, int n, uint8_t *b);
#endif
