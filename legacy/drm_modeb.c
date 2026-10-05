#include "drm_modeb.h"
#include <math.h>
#include <string.h>

drm_cellinfo_t drm_map[DRM_NSYM_FRAME][DRM_NCAR];

/* Tab. 52 / 54 (Mode B). */
static const struct { int8_t k; uint16_t ph; } freqref[3] = {{16,331},{48,651},{64,555}};
static const struct { int8_t k; uint16_t ph; } timeref[] = {
 {14,304},{16,331},{18,108},{20,620},{24,192},{26,704},{32,44},{36,432},{42,588},
 {44,844},{48,651},{49,651},{50,651},{54,460},{56,460},{62,944},{64,555},{66,940},{68,428}};
/* Kap. 8.4.4.3.3 */
static const uint16_t W1024[3][5] = {{512,0,512,0,512},{0,512,0,512,0},{512,0,512,0,512}};
static const uint8_t  Z256 [3][5] = {{0,57,164,64,12},{168,255,161,106,118},{25,232,132,233,38}};
#define Q1024 12
/* Tab. 63: FAC-Zellen, Symbole 2..13 */
static const int8_t fac_pos[DRM_NSYM_FRAME][7] = {
 {0},{0},
 {13,25,43,55,67,0},{15,27,45,57,69,0},{17,29,47,59,71,0},{19,31,49,61,73,0},
 {9,21,33,51,63,75,0},{11,23,35,53,65,77,0},{13,25,37,55,67,79,0},{15,27,39,57,69,81,0},
 {17,29,41,59,71,83,0},{19,31,43,61,73,0},{21,33,45,63,75,0},{23,35,47,65,77,0},{0}};

#define AT(s,k) drm_map[s][(k)-DRM_KMIN]

void drm_modeb_init(void)
{
    memset(drm_map, 0, sizeof drm_map);
    for (int s = 0; s < DRM_NSYM_FRAME; s++)
        for (int k = DRM_KMIN; k <= DRM_KMAX; k++)
            AT(s,k).type = (k == 0) ? CT_UNUSED : CT_DATA;
    for (int s = 0; s < DRM_NSYM_FRAME; s++)
        for (int i = 0; i < 3; i++) { AT(s,freqref[i].k).type = CT_FREQ; AT(s,freqref[i].k).ph1024 = freqref[i].ph; }
    for (unsigned i = 0; i < sizeof timeref/sizeof timeref[0]; i++) {
        AT(0,timeref[i].k).type = CT_TIME; AT(0,timeref[i].k).ph1024 = timeref[i].ph;
    }
    for (int s = 0; s < DRM_NSYM_FRAME; s++) {
        int n = s % 3, m = s / 3;
        for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
            if (((k - 1 - 2*n) % 6) != 0) continue;           /* k = 1+2n+6p */
            if (AT(s,k).type == CT_FREQ || AT(s,k).type == CT_TIME) continue; /* Vorrang */
            int p = (k - 1 - 2*n) / 6;
            long th = 4L*Z256[n][m] + (long)p*W1024[n][m] + (long)p*p*(1+s)*Q1024;
            th %= 1024; if (th < 0) th += 1024;
            AT(s,k).type = CT_GAIN; AT(s,k).ph1024 = (uint16_t)th;
            if (k==-103||k==-101||k==101||k==103) AT(s,k).boost = 1;   /* Tab. 59 */
        }
    }
    for (int s = 0; s < DRM_NSYM_FRAME; s++)
        for (int i = 0; fac_pos[s][i]; i++) {
            drm_cellinfo_t *c = &AT(s, fac_pos[s][i]);
            if (c->type == CT_DATA) c->type = CT_FAC;          /* Konflikt mit Piloten -> bleibt Pilot */
        }
}

void drm_ref_value(int s, int k, float *re, float *im)
{
    const drm_cellinfo_t *c = &AT(s,k);
    float a = c->boost ? 2.0f : 1.41421356f;
    float ph = 2.0f*(float)M_PI*c->ph1024/1024.0f;
    *re = a*cosf(ph); *im = a*sinf(ph);
}
