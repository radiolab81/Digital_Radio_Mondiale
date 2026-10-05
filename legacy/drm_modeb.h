/* drm_modeb.h - Zellraster Mode B / 10 kHz (ES 201 980 V4.3.1, Kap. 8.4-8.6) */
#ifndef DRM_MODEB_H
#define DRM_MODEB_H
#include "drm_cfg.h"
enum { CT_UNUSED = 0, CT_DATA, CT_FREQ, CT_TIME, CT_GAIN, CT_FAC };
typedef struct { uint8_t type; uint8_t boost; uint16_t ph1024; } drm_cellinfo_t;
extern drm_cellinfo_t drm_map[DRM_NSYM_FRAME][DRM_NCAR];   /* [s][k-KMIN] */
void drm_modeb_init(void);
/* Referenzwert U(s,k) als float */
void drm_ref_value(int s, int k, float *re, float *im);
#endif
