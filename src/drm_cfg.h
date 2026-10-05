/* ==========================================================================================================
 * drm_cfg.h  --  Gemeinsame Typen und Obergrenzen des DRM-Empfaengers
 * ==========================================================================================================
 * Referenz: ETSI ES 201 980 V4.3.1 (2023-11), "Digital Radio Mondiale (DRM); System Specification".
 *           Im Folgenden kurz "ES 201 980" genannt; Verweise wie [8.4.2] meinen Kapitel dieser Norm.
 *
 * Portabilitaet:  Der Kern (alle drm_*.c ausser den Werkzeugen drm_test, drm_sim, iq_prep, xhe_dec, aac_dec)
 * benoetigt nur <stdint.h>, <stddef.h>, <string.h> und <math.h>.  Es wird nirgends malloc() benutzt: aller
 * Speicher liegt in Strukturen, die der Aufrufer bereitstellt (statisch, im Stack oder im PSRAM eines ESP32).
 * ========================================================================================================== */
#ifndef DRM_CFG_H
#define DRM_CFG_H
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef M_PI                                 /* streng ISO-C-Umgebungen (z. B. manche Embedded-Toolchains) */
#define M_PI 3.14159265358979323846
#endif

/* Abtastrate des komplexen Basisbandes, das alle Module erwarten.  Die DRM-Normen (Modi A-D) definieren die
 * OFDM-Zeitparameter in Vielfachen von T = 1/12000 s [8.2, Tab. 47]; bei 12 kHz sind alle Symbollaengen ganz-
 * zahlig (Nutzsymbol 288, 256, 176 bzw. 112 Abtastwerte).  Andere Eingangsraten werden von iq_prep umgerechnet. */
#define DRM_FS       12000

/* Obergrenzen fuer statische Puffer (Modi A-D, alle Belegungen). */
#define DRM_MAXTU    288     /* groesste FFT-Laenge:  Mode A, Tu = 24 ms                               [8.2] */
#define DRM_MAXCAR   240     /* groesste Traegerzahl: Mode A, 10 kHz = 229 Traeger                      [8.3.1] */
#define DRM_MAXSYM   24      /* groesste Symbolzahl je Rahmen: Mode D = 24                              [8.1] */

/* Eingangsabtastwert: komplexes Basisband, 16 Bit je Komponente (I = Realteil, Q = Imaginaerteil). */
typedef struct { int16_t re, im; } drm_cs16_t;

/* Spektralwert nach der FFT (eine OFDM-"Zelle", noch nicht entzerrt). */
typedef struct { float re, im; } drm_cell_t;

#endif
