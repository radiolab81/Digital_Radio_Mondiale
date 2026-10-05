/* probe.c - Analysewerkzeug (nur PC): WAV-I/Q lesen, synchronisieren, demodulieren,
 * Pilot-Kohaerenz je Traeger ausgeben. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "drm_ofdm.h"

static drm_cs16_t *load_wav(const char *fn, int *n)
{
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); exit(1); }
    uint8_t h[12]; fread(h, 1, 12, f);
    uint32_t sz; uint8_t id[4]; long dpos = 0; uint32_t dlen = 0;
    while (fread(id, 1, 4, f) == 4 && fread(&sz, 4, 1, f) == 1) {
        if (!memcmp(id, "data", 4)) { dpos = ftell(f); dlen = sz; break; }
        fseek(f, sz + (sz & 1), SEEK_CUR);
    }
    *n = dlen / 4;
    drm_cs16_t *b = malloc(dlen); fseek(f, dpos, SEEK_SET);
    fread(b, 4, *n, f); fclose(f); return b;
}

int main(int argc, char **argv)
{
    int n; drm_cs16_t *iq = load_wav(argc > 1 ? argv[1] : "DRM_6030_000_iq.wav", &n);
    drm_ofdm_t o; drm_ofdm_init(&o);
    float foff, met;
    int t0 = drm_ofdm_acquire(&o, iq, n, 60, &foff, &met);
    printf("Akquisition: Symbolstart-Offset %d, Frac-Freq %.2f Hz, Metrik %.3f\n", t0, foff, met);
    drm_ofdm_set_freq(&o, foff);

    int nsym = (n - t0 - DRM_TS) / DRM_TS; if (nsym > 2000) nsym = 2000;
    static double cr[DRM_NCAR], ci[DRM_NCAR], pw[DRM_NCAR];
    drm_cell_t prev[DRM_NCAR], cur[DRM_NCAR];
    clock_t c0 = clock();
    for (int l = 0; l < nsym; l++) {
        drm_ofdm_symbol(&o, iq + t0 + l*DRM_TS, cur);
        if (l) for (int k = 0; k < DRM_NCAR; k++) {
            double ar = CELL_RE(cur[k]), ai = CELL_IM(cur[k]);
            double br = CELL_RE(prev[k]), bi = CELL_IM(prev[k]);
            cr[k] += ar*br + ai*bi; ci[k] += ai*br - ar*bi;
            pw[k] += sqrt((ar*ar+ai*ai)*(br*br+bi*bi));
        }
        memcpy(prev, cur, sizeof cur);
    }
    double dt = (double)(clock()-c0)/CLOCKS_PER_SEC;
    printf("%d Symbole demoduliert in %.1f ms (%.1f us/Symbol)\n", nsym, dt*1e3, dt*1e6/nsym);
    printf("Traeger mit hoher Symbol-zu-Symbol-Kohaerenz (Frequenzpilots):\n");
    for (int k = 0; k < DRM_NCAR; k++) {
        double r = sqrt(cr[k]*cr[k]+ci[k]*ci[k]) / (pw[k]+1e-9);
        if (r > 0.5) printf("  k=%4d  coh=%.3f  (%.1f Hz)\n", k+DRM_KMIN, r, (k+DRM_KMIN)*46.875);
    }
    return 0;
}
