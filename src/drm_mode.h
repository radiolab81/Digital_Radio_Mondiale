/* drm_mode.h -- Robustheitsmodi A-D (ES 201 980 V4.3.1, Kap. 8): Parameter, Pilot- und Zellraster.
 * Zellarten: CT_DATA = QAM-Datenzelle (MSC; im ersten Rahmen eines Superframes SDC), CT_FREQ/CT_TIME/CT_GAIN =
 * Pilotarten, CT_FAC = FAC-Zelle, CT_UNUSED = ungenutzter Traeger. */
#ifndef DRM_MODE_H
#define DRM_MODE_H
#include "drm_cfg.h"
enum { CT_UNUSED = 0, CT_DATA, CT_FREQ, CT_TIME, CT_GAIN, CT_FAC };
enum { DRM_MODE_A = 0, DRM_MODE_B, DRM_MODE_C, DRM_MODE_D };
typedef struct { uint8_t type, boost; uint16_t ph1024; } drm_cellinfo_t;

typedef struct {
    int id, occ;                         /* Modus 0..3 = A..D, Spektrumbelegung 0..3 */
    int Tu, Tg, Ts, nsym;                /* Samples bei 12 kHz, Symbole je Rahmen   */
    int kmin, kmax, ncar;
    float spacing_hz;
    int nsdc_sym;                        /* Symbole mit SDC (A/B: 2) */
    int nsdc, nmux;                      /* QAM-Zellen SDC / MSC je Multiplexrahmen */
    drm_cellinfo_t map[DRM_MAXSYM][DRM_MAXCAR];   /* [s][k-kmin] */
} drm_mode_t;

/* 0 bei Erfolg, -1: Kombination (noch) nicht unterstuetzt */
int  drm_mode_setup(drm_mode_t *m, int id, int occ);
void drm_ref_value(const drm_mode_t *m, int s, int k, float *re, float *im);
const char *drm_mode_name(int id);
#endif
