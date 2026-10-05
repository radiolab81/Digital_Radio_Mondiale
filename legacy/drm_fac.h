#ifndef DRM_FAC_H
#define DRM_FAC_H
#include "drm_cfg.h"
typedef struct {
    int crc_ok;
    int identity, rm_flag, spectrum_occ, interleaver_depth, msc_mode, sdc_mode, nservices, reconf, toggle;
    uint32_t service_id; int short_id, audio_ca, language, audio_data, descriptor;
    uint8_t bits[72];
} drm_fac_t;
/* soft: 130 Werte (65 Zellen x {Re,Im}, >0 => Bit 0) in Uebertragungsreihenfolge */
int drm_fac_decode(const int8_t *soft130, drm_fac_t *f);
#endif
