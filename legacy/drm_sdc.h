#ifndef DRM_SDC_H
#define DRM_SDC_H
#include "drm_cfg.h"
#define SDC_NCELL 322            /* Mode B, 10 kHz (Tab. 25) */
typedef struct {
    int crc_ok, afs_index;
    int nstreams, prot_a, prot_b, len_a[4], len_b[4];       /* Typ 0 (Bytes) */
    char label[4][65];                                      /* Typ 1, je Short-Id */
    int  have_audio;                                        /* Typ 9 */
    int  a_short, a_stream, a_coding, a_sbr, a_mode, a_rate, a_text, a_enh, a_cfglen;
    uint8_t a_cfg[24];
} drm_sdc_t;
/* soft: 2*SDC_NCELL Werte (Re,Im je Zelle, >0 => Bit 0), Reihenfolge: k aufsteigend, dann Symbol 0, Symbol 1 */
int drm_sdc_decode(const int8_t *soft, drm_sdc_t *s);
#endif
