/* ==========================================================================================================
 * drm_ofdm.c  --  OFDM-Demodulation
 * ==========================================================================================================
 * OFDM (Lehrbuch): Ein Symbol der Dauer Ts = Tu + Tg besteht aus dem Nutzteil Tu und einem vorangestellten
 * Guardintervall Tg, das eine Kopie des Symbolendes ist (zyklisches Praefix).  Mehrwegeausbreitung mit
 * Verzoegerungen < Tg stoert dann nur das Guardintervall; schneidet man das FFT-Fenster der Laenge Tu aus dem
 * ungestoerten Rest, ist jeder Unterträger nach der FFT nur mit dem komplexen Kanalfaktor H(k) multipliziert
 * (Kanal = einfache Multiplikation je Traeger) [ES 201 980, 8.1, 8.2].
 *
 * Synchronisation ueber das Guardintervall: x(n) im Guard gleicht x(n+Tu).  Die Korrelation
 *      C(t) = sum_{i<Tg} x(t+i) * conj(x(t+i+Tu))
 * hat ihr Maximum am Symbolbeginn.  Der Betrag zeigt die Symbolgrenze, das Argument den Frequenzoffset:
 * ein Offset f dreht das Signal in der Zeit Tu um 2*pi*f*Tu, also  f = -arg(C) * fs / (2*pi*Tu)  (Bereich
 * +-0,5 Traegerabstand; den ganzzahligen Anteil findet drm_test/drm_rx ueber die Pilote).  Weil die vier
 * Modi verschiedene (Tu, Tg) haben, liefert die Metrik zugleich die Erkennung des Robustheitsmodus.
 * ========================================================================================================== */
#include "drm_ofdm.h"
#include <math.h>
#include <string.h>

/* ---- Mischer (NCO): exp(-j*phase) aus einer 1024-Punkt-Tabelle, Phase als 32-Bit-Akkumulator -------------- */
#define LUTN 1024
static int16_t lut_c[LUTN], lut_s[LUTN]; static int lut_ok;

/* ---- Interpolationsfilter fuer das Bruchteil-Fenster: gefenstertes sinc (Kaiser, beta = 6), 24 Taps, 128 Phasen.
 * Fehler gegenueber idealem Verzoegerer < -55 dB bis 0,83 * Nyquist.  Warum?  Die Abtasttakte von Sender und
 * Empfaenger weichen um einige 10..100 ppm ab, das Symbolfenster wandert also langsam.  Ganzzahlige Fensterschritte
 * wuerden Phasensprunge 2*pi*k/N je Schritt erzeugen und die Kanalschaetzung stoeren; die stetige Bruchteil-
 * verschiebung vermeidet das. ---- */
#define FR_TAPS 24
#define FR_PH   128
static float ftab[FR_PH + 1][FR_TAPS]; static int ftab_ok;
static double bessel_i0(double x) { double s = 1, t = 1; for (int k = 1; k < 30; k++) { t *= (x / (2.0 * k)) * (x / (2.0 * k)); s += t; } return s; }
static void ftab_init(void)
{
    const double beta = 6.0;
    for (int p = 0; p <= FR_PH; p++) {
        double fr = (double)p / FR_PH, sum = 0, h[FR_TAPS];
        for (int i = 0; i < FR_TAPS; i++) {
            int n = i - (FR_TAPS / 2 - 1); double t = n - fr, x = t / (FR_TAPS / 2.0);
            double w = fabs(x) < 1 ? bessel_i0(beta * sqrt(1 - x * x)) / bessel_i0(beta) : 0;     /* Kaiser-Fenster */
            h[i] = (fabs(t) < 1e-12 ? 1.0 : sin(M_PI * t) / (M_PI * t)) * w; sum += h[i];
        }
        for (int i = 0; i < FR_TAPS; i++) ftab[p][i] = (float)(h[i] / sum);                   /* Gleichanteil = 1 */
    }
    ftab_ok = 1;
}

void drm_ofdm_init(drm_ofdm_t *o, const drm_mode_t *m)
{
    if (!lut_ok) {
        for (int i = 0; i < LUTN; i++) { lut_c[i] = (int16_t)lround(32767.0 * cos(2.0 * M_PI * i / LUTN)); lut_s[i] = (int16_t)lround(32767.0 * sin(2.0 * M_PI * i / LUTN)); }
        lut_ok = 1;
    }
    if (!ftab_ok) ftab_init();
    memset(o, 0, sizeof *o);
    o->m = m; drm_fft_plan(&o->plan, m->Tu);
    o->fft_adv = 8;       /* Fenster etwas vor das Guardende legen: Reserve gegen Echos, die in den Nutzteil hineinreichen */
}

void drm_ofdm_set_freq(drm_ofdm_t *o, float hz)
{
    o->foff_hz = hz;
    o->nco_inc = (int32_t)llround((double)hz / DRM_FS * 4294967296.0);
}

/* ---- Erstsynchronisation (Guard-Korrelation) in reiner Ganzzahlarithmetik (Akkus 64 Bit, Eingang >>3) ---- */
int drm_acquire(const drm_cs16_t *iq, int n, int nb, drm_acq_t *r)
{
    static int64_t cr[512], ci[512], en[512];
    static const int TU[4] = {288, 256, 176, 112}, TG[4] = {32, 64, 64, 88};      /* Modi A-D [8.2, Tab. 47] */
    int bestm = -1; double bestscore = 0;
    for (int md = 0; md < 4; md++) {
        const int Tu = TU[md], Tg = TG[md], Ts = Tu + Tg;
        if (n < (nb + 1) * Ts + Tu) { r->score[md] = 0; continue; }
        for (int t = 0; t < Ts; t++) cr[t] = ci[t] = en[t] = 0;
        for (int b = 0; b < nb; b++)                          /* ueber nb Symbole mitteln -> Rauschen sinkt */
            for (int t = 0; t < Ts; t++) {                    /* Kandidat t fuer den Symbolbeginn */
                const drm_cs16_t *a = iq + b * Ts + t, *c = a + Tu;
                int64_t sr = 0, si = 0, e = 0;
                for (int i = 0; i < Tg; i++) {
                    int32_t ar = a[i].re >> 3, ai = a[i].im >> 3, br = c[i].re >> 3, bi = c[i].im >> 3;
                    sr += ar * br + ai * bi;                  /* Re{ a * conj(c) } */
                    si += ai * br - ar * bi;                  /* Im{ a * conj(c) } */
                    e  += ar * ar + ai * ai + br * br + bi * bi;
                }
                cr[t] += sr; ci[t] += si; en[t] += e;
            }
        int best = 0; double bm = -1;
        for (int t = 0; t < Ts; t++) { double mg = sqrt((double)cr[t] * cr[t] + (double)ci[t] * ci[t]); if (mg > bm) { bm = mg; best = t; } }
        double met = bm / (0.5 * (double)en[best] + 1.0);     /* normierter Korrelationskoeffizient (1 = Kopie) */
        r->score[md] = (float)met;
        if (met > bestscore) {
            bestscore = met; bestm = md; r->t0 = best; r->metric = (float)met;
            r->foff_hz = (float)(-atan2((double)ci[best], (double)cr[best]) * DRM_FS / (2.0 * M_PI * Tu));
        }
    }
    r->mode = (bestscore > 0.15) ? bestm : -1;               /* Schwelle gegen reines Rauschen */
    return r->mode;
}

/* ---- Ein Symbol: (Bruchteil-)Verzoegerung -> Mischen auf 0 Hz -> FFT -> Traeger kmin..kmax herausschneiden ---- */
void drm_ofdm_symbol(drm_ofdm_t *o, const drm_cs16_t *iq, float frac, drm_cell_t *cells)
{
    const drm_mode_t *m = o->m; drm_cell_t buf[DRM_MAXTU], f[DRM_MAXTU];
    /* Interpolationskoeffizienten fuer den Bruchteil frac aus der 128-Phasen-Tabelle (linear zwischen Phasen) */
    float fp = frac * FR_PH; int pi = (int)fp; float pf = fp - pi;
    if (pi >= FR_PH) { pi = FR_PH - 1; pf = 1; }
    float tap[FR_TAPS];
    for (int i = 0; i < FR_TAPS; i++) tap[i] = ftab[pi][i] + pf * (ftab[pi + 1][i] - ftab[pi][i]);

    /* Fensterbeginn: Ende Guardintervall minus fft_adv; die Filterspanne ragt 11 Werte davor hinein */
    const drm_cs16_t *p = iq + m->Tg - o->fft_adv - (FR_TAPS / 2 - 1);
    uint32_t ph = o->nco_phase + (uint32_t)((int64_t)o->nco_inc * (m->Tg - o->fft_adv));
    for (int i = 0; i < m->Tu; i++) {
        float xr = 0, xi = 0; const drm_cs16_t *q = p + i;
        for (int k = 0; k < FR_TAPS; k++) { xr += tap[k] * q[k].re; xi += tap[k] * q[k].im; }   /* FIR-Interpolator */
        unsigned idx = (ph >> 22) & (LUTN - 1);               /* oberste 10 Bit der Phase = Tabellenindex */
        float c = lut_c[idx] * (1.0f / 32768), s = lut_s[idx] * (1.0f / 32768);
        buf[i].re = xr * c + xi * s;                          /* x * exp(-j*phase) */
        buf[i].im = xi * c - xr * s;
        ph += (uint32_t)o->nco_inc;
    }
    o->nco_phase += (uint32_t)((int64_t)o->nco_inc * m->Ts);  /* Mischerphase laeuft ueber das ganze Symbol weiter */
    drm_fft(&o->plan, buf, f);
    /* Traeger k liegt im FFT-Bin k (negative k am oberen Ende): kmin..kmax in aufsteigender Folge entnehmen */
    for (int k = m->kmin; k <= m->kmax; k++) cells[k - m->kmin] = f[(k + m->Tu) % m->Tu];
}
