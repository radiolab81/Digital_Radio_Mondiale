#include "drm_ofdm.h"
#include "drm_fft.h"
#include <math.h>

/* ---- Sinus-LUT fuer NCO (Q15, 1024 Eintraege, Viertelwelle nicht genutzt fuer Klarheit) ---- */
#define LUTN 1024
static int16_t lut_c[LUTN], lut_s[LUTN];
static int lut_ok;

void drm_ofdm_init(drm_ofdm_t *o)
{
    if (!lut_ok) {
        for (int i = 0; i < LUTN; i++) {
            lut_c[i] = (int16_t)lround(32767.0*cos(2.0*M_PI*i/LUTN));
            lut_s[i] = (int16_t)lround(32767.0*sin(2.0*M_PI*i/LUTN));
        }
        lut_ok = 1;
    }
    drm_fft_init();
    o->nco_phase = 0; o->nco_inc = 0; o->sym_pos = 0; o->fft_adv = 8; o->foff_hz = 0;
}

void drm_ofdm_set_freq(drm_ofdm_t *o, float hz)
{
    /* Mischer multipliziert mit exp(-j*2*pi*hz*t): Signal bei +hz -> DC */
    o->foff_hz = hz;
    o->nco_inc = (int32_t)llround((double)hz / DRM_FS * 4294967296.0);
}

int drm_ofdm_acquire(drm_ofdm_t *o, const drm_cs16_t *iq, int n, int nsym,
                     float *foff_hz, float *metric)
{
    static int64_t cr[DRM_TS], ci[DRM_TS];
    static int64_t en[DRM_TS];
    if (n < (nsym+1)*DRM_TS + DRM_TU) return -1;
    for (int t = 0; t < DRM_TS; t++) { cr[t] = ci[t] = 0; en[t] = 0; }
    for (int m = 0; m < nsym; m++) {
        for (int t = 0; t < DRM_TS; t++) {
            const drm_cs16_t *a = iq + m*DRM_TS + t;
            const drm_cs16_t *b = a + DRM_TU;
            int64_t sr = 0, si = 0, e = 0;
            for (int i = 0; i < DRM_TG; i++) {
                int32_t ar = a[i].re >> 3, ai = a[i].im >> 3;
                int32_t br = b[i].re >> 3, bi = b[i].im >> 3;
                sr += (int32_t)ar*br + (int32_t)ai*bi;     /* a * conj(b) */
                si += (int32_t)ai*br - (int32_t)ar*bi;
                e  += (int32_t)ar*ar + (int32_t)ai*ai + (int32_t)br*br + (int32_t)bi*bi;
            }
            cr[t] += sr; ci[t] += si; en[t] += e;
        }
    }
    int best = 0; double bm = -1;
    for (int t = 0; t < DRM_TS; t++) {
        double m = sqrt((double)cr[t]*cr[t] + (double)ci[t]*ci[t]);
        if (m > bm) { bm = m; best = t; }
    }
    /* a*conj(b) mit b = a verschoben um +TU: Phase = -2*pi*f*TU/fs  ->  f = -arg*fs/(2*pi*TU) */
    double arg = atan2((double)ci[best], (double)cr[best]);
    *foff_hz = (float)(-arg * DRM_FS / (2.0*M_PI*DRM_TU));
    *metric  = (float)(bm / (0.5*(double)en[best] + 1.0));   /* ~1 = perfekte Korrelation */
    o->sym_pos = best;
    return best;
}

/* Mischen + Fensterung + FFT eines Symbols */
void drm_ofdm_symbol(drm_ofdm_t *o, const drm_cs16_t *iq, drm_cell_t *cells)
{
    drm_cs16_t buf[DRM_TU];
    const drm_cs16_t *p = iq + DRM_TG - o->fft_adv;
    /* Phase am Fensteranfang: NCO laeuft pro Symbol um TS Samples weiter, das
     * Fenster beginnt (TG-fft_adv) Samples nach Symbolanfang.               */
    uint32_t ph = o->nco_phase + (uint32_t)((int64_t)o->nco_inc * (DRM_TG - o->fft_adv));
    for (int i = 0; i < DRM_TU; i++) {
        unsigned idx = (ph >> 22) & (LUTN-1);         /* 32 -> 10 Bit */
        int32_t c = lut_c[idx], s = lut_s[idx];       /* exp(-j...) = c - j s */
        int32_t xr = p[i].re, xi = p[i].im;
        buf[i].re = (int16_t)((xr*c + xi*s + 16384) >> 15);
        buf[i].im = (int16_t)((xi*c - xr*s + 16384) >> 15);
        ph += (uint32_t)o->nco_inc;
    }
    o->nco_phase += (uint32_t)((int64_t)o->nco_inc * DRM_TS);
#ifdef DRM_FIXED
    drm_fft_q15(buf);
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) cells[k-DRM_KMIN] = buf[(k+DRM_TU)%DRM_TU];
#else
    drm_cell_t f[DRM_TU];
    drm_fft_f32(buf, f);
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) cells[k-DRM_KMIN] = f[(k+DRM_TU)%DRM_TU];
#endif
}

void drm_ofdm_track(drm_ofdm_t *o, const drm_cs16_t *s0, int nsym)
{
    /* Feintiming: Guard-Korrelation fuer Verschiebung -2..+2 */
    int64_t mag[5]; int64_t cr_best = 0, ci_best = 0; int bi = 2;
    for (int d = -2; d <= 2; d++) {
        int64_t cr = 0, ci = 0;
        for (int m = 0; m < nsym; m++) {
            const drm_cs16_t *a = s0 + m*DRM_TS + d, *b = a + DRM_TU;
            for (int i = 0; i < DRM_TG; i++) {
                int32_t ar=a[i].re>>3, ai=a[i].im>>3, br=b[i].re>>3, bq=b[i].im>>3;
                cr += (int32_t)ar*br + (int32_t)ai*bq;
                ci += (int32_t)ai*br - (int32_t)ar*bq;
            }
        }
        mag[d+2] = (int64_t)sqrt((double)cr*cr + (double)ci*ci);
        if (d == 0) { cr_best = cr; ci_best = ci; }
    }
    for (int i = 0; i < 5; i++) if (mag[i] > mag[bi]) bi = i;
    o->sym_pos += bi - 2;
    (void)cr_best; (void)ci_best;
}
