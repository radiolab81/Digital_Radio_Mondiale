/* drm_cfg.h - Konfiguration / Portabilitaet
 *
 * Nur <stdint.h>/<stddef.h> im Kern. Keine dynamische Speicherallokation im Kern
 * (alle Puffer werden vom Aufrufer bereitgestellt) -> direkt auf ESP-IDF nutzbar.
 *
 *  -DDRM_FIXED   OFDM-Pfad in Q15-Ganzzahl (int16 Zellen, int32 Akkus)
 *  (default)     OFDM-Pfad in float32
 */
#ifndef DRM_CFG_H
#define DRM_CFG_H
#include <stdint.h>
#include <stddef.h>

/* ---- Robustheitsmodus / Bandbreite (Stufe 1: Mode B, 10 kHz) ---- */
#define DRM_FS          12000          /* Abtastrate I/Q (komplex)            */
#define DRM_TU          256            /* Nutzsymbol  [Samples]  Mode B        */
#define DRM_TG          64             /* Guard       [Samples]  Mode B        */
#define DRM_TS          (DRM_TU+DRM_TG)
#define DRM_NSYM_FRAME  15             /* OFDM-Symbole pro Uebertragungsrahmen  */
#define DRM_KMIN        (-103)         /* Traeger Mode B, 10 kHz (ES 201 980 Tab.49) */
#define DRM_KMAX        ( 103)
#define DRM_NCAR        (DRM_KMAX-DRM_KMIN+1)

typedef struct { int16_t re, im; } drm_cs16_t;

#ifdef DRM_FIXED
  typedef drm_cs16_t drm_cell_t;
  #define CELL_RE(c) ((float)(c).re)
  #define CELL_IM(c) ((float)(c).im)
#else
  typedef struct { float re, im; } drm_cell_t;
  #define CELL_RE(c) ((c).re)
  #define CELL_IM(c) ((c).im)
#endif

#endif
