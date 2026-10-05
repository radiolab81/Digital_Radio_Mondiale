/* ==========================================================================================================
 * iq_prep.c  --  Eingangsaufbereitung (nur PC-Werkzeug): beliebige WAV-Aufnahme -> komplexes 12-kHz-Basisband
 * ==========================================================================================================
 * Der Empfaenger (drm_rx) erwartet komplexes Basisband mit 12 kHz Abtastrate, das DRM-Band ungefaehr um 0 Hz.
 * Aufnahmen kommen aber meist anders:
 *   (a) 2 Kanaele, I != Q            : echtes I/Q-Signal (z. B. SDR). Es wird nur umgetastet.
 *   (b) 1 Kanal oder L == R          : reelles ZF-Signal, z. B. die Soundkartenaufnahme eines Empfaengers mit DRM-
 *       Signal bei 12 kHz Zwischenfrequenz.  Ein reelles Signal enthaelt das Band doppelt (positive und negative
 *       Frequenz).  Man macht es "analytisch": mit exp(-j*2*pi*fc*t) herunter mischen (Band nach 0 Hz, Spiegelband
 *       nach -2*fc) und tiefpassfiltern; uebrig bleibt das komplexe Basisband.
 * Schritte: Bandmitte und -breite aus dem Leistungsspektrum schaetzen (Welch-Mittelung, laengster zusammen-
 * haengender Lauf ueber der Schwelle) -> mischen -> Tiefpass (Blackman-Sinc, Grenze 5,8 kHz) -> bandbegrenzte
 * Interpolation auf 12 kHz (32 Taps).  Die Mittenfrequenz muss nur auf einige 100 Hz stimmen: den Rest findet die
 * Synchronisation des Empfaengers (ganzzahliger Traegerversatz +-12 Traeger in drm_rx, Feinwert ueber die Guard-
 * Korrelation, siehe drm_ofdm.c).
 * Aufruf: iq_prep in.wav out.wav [-f center_hz] -- wird -f angegeben, entfaellt die automatische Bandsuche.
 * ========================================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define OUT_FS 12000
static int rd_wav(const char *fn, int16_t **d, int *n, int *ch, int *fs)
{
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); return 0; }
    uint8_t h[12]; if (fread(h, 1, 12, f) != 12) return 0;
    uint32_t sz; uint8_t id[4]; long dpos = 0; uint32_t dlen = 0; uint16_t fmt[8] = {0};
    while (fread(id, 1, 4, f) == 4 && fread(&sz, 4, 1, f) == 1) {
        if (!memcmp(id, "fmt ", 4)) { uint8_t b[16]; if (fread(b, 1, 16, f) != 16) return 0; memcpy(fmt, b, 16); if (sz > 16) fseek(f, sz - 16 + (sz & 1), SEEK_CUR); continue; }
        if (!memcmp(id, "data", 4)) { dpos = ftell(f); dlen = sz; break; }
        fseek(f, sz + (sz & 1), SEEK_CUR);
    }
    if (!dpos || fmt[0] != 1 || fmt[7] != 16) { fprintf(stderr, "nur PCM16\n"); return 0; }
    *ch = fmt[1]; *fs = (int)(fmt[2] | (fmt[3] << 16)); *n = (int)(dlen / (2 * (*ch)));
    *d = malloc(dlen); fseek(f, dpos, SEEK_SET); if (fread(*d, 2, (size_t)(*n) * (*ch), f) != (size_t)(*n) * (*ch)) return 0;
    fclose(f); return 1;
}
static void wr_wav(const char *fn, const int16_t *iq, int n, int fs)
{
    FILE *f = fopen(fn, "wb"); uint32_t nb = (uint32_t)n * 4, v; uint16_t s;
    fwrite("RIFF", 1, 4, f); v = 36 + nb; fwrite(&v, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    v = 16; fwrite(&v, 4, 1, f); s = 1; fwrite(&s, 2, 1, f); s = 2; fwrite(&s, 2, 1, f);
    v = (uint32_t)fs; fwrite(&v, 4, 1, f); v = (uint32_t)fs * 4; fwrite(&v, 4, 1, f); s = 4; fwrite(&s, 2, 1, f); s = 16; fwrite(&s, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&nb, 4, 1, f); fwrite(iq, 4, (size_t)n, f); fclose(f);
}
/* kleine Radix-2-FFT (double) nur fuer die Spektrumschaetzung */
static void fft2(double *re, double *im, int n)
{
    for (int i = 1, j = 0; i < n; i++) { int b = n >> 1; for (; j & b; b >>= 1) j ^= b; j ^= b; if (i < j) { double t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; } }
    for (int len = 2; len <= n; len <<= 1) {
        double a = -2*M_PI/len, wr = cos(a), wi = sin(a);
        for (int i = 0; i < n; i += len) { double cr = 1, ci = 0;
            for (int k = 0; k < len/2; k++) {
                double xr = re[i+k+len/2]*cr - im[i+k+len/2]*ci, xi = re[i+k+len/2]*ci + im[i+k+len/2]*cr;
                re[i+k+len/2] = re[i+k] - xr; im[i+k+len/2] = im[i+k] - xi; re[i+k] += xr; im[i+k] += xi;
                double t = cr*wr - ci*wi; ci = cr*wi + ci*wr; cr = t; } }
    }
}
static int cmpd(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }

/* Mitte und Breite des belegten Bandes aus der mittleren Leistungsdichte */
static void find_band(const double *xr, const double *xi, int n, int fs, int real, double *fc, double *bw)
{
    const int N = 4096; static double P[4096], re[4096], im[4096], w[4096];
    for (int i = 0; i < N; i++) w[i] = 0.5 - 0.5*cos(2*M_PI*i/N);
    int nb = n / N; if (nb > 400) nb = 400;
    for (int b = 0; b < nb; b++) {
        for (int i = 0; i < N; i++) { re[i] = xr[b*N+i]*w[i]; im[i] = real ? 0 : xi[b*N+i]*w[i]; }
        fft2(re, im, N); for (int i = 0; i < N; i++) P[i] += re[i]*re[i] + im[i]*im[i];
    }
    /* Frequenzachse -fs/2..fs/2 (komplex) bzw. 0..fs/2 (reell), 8-Bin-Glaettung */
    int lo_i = real ? N/64 : 0, hi_i = real ? N/2 - N/64 : N;
    static double S[4096]; double db[4096]; int m = 0;
    for (int i = 0; i < N; i++) { int k = real ? i : (i + N/2) % N; double s = 0; for (int j = -4; j <= 3; j++) s += P[(k + j + N) % N]; S[i] = 10*log10(s/8 + 1e-9); }
    double tmp[4096]; for (int i = lo_i; i < hi_i; i++) tmp[m++] = S[i]; memcpy(db, tmp, m*sizeof(double));
    qsort(tmp, m, sizeof(double), cmpd);
    double thr = 0.5*(tmp[(int)(0.1*m)] + tmp[(int)(0.9*m)]);
    int a = -1, b = -1, ra = -1, best = 0;                 /* laengster zusammenhaengender Lauf ueber der Schwelle */
    for (int i = lo_i; i <= hi_i; i++) {
        int on = (i < hi_i) && S[i] > thr;
        if (on && ra < 0) ra = i;
        if (!on && ra >= 0) { if (i - ra > best) { best = i - ra; a = ra; b = i - 1; } ra = -1; }
    }
    double df = (double)fs / N;
    if (real) { *fc = 0.5*(a + b) * df; } else { *fc = (0.5*(a + b) - N/2) * df; }
    *bw = (b - a) * df;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "Aufruf: %s in.wav out.wav [-f center_hz]\n", argv[0]); return 1; }
    double fc_user = 1e30; for (int i = 3; i + 1 < argc; i += 2) if (!strcmp(argv[i], "-f")) fc_user = atof(argv[i+1]);
    int16_t *d; int n, ch, fs; if (!rd_wav(argv[1], &d, &n, &ch, &fs)) return 1;
    double *xr = malloc(n*sizeof(double)), *xi = malloc(n*sizeof(double));
    int real = (ch == 1);
    if (ch >= 2) { int same = 1; for (int i = 0; i < n && same; i++) if (d[ch*i] != d[ch*i+1]) same = 0; real = same; }
    for (int i = 0; i < n; i++) { xr[i] = d[ch*i]; xi[i] = (ch >= 2) ? d[ch*i+1] : 0; }
    double fc, bw; find_band(xr, xi, n, fs, real, &fc, &bw);
    if (fc_user < 1e29) fc = fc_user;
    fprintf(stderr, "%s: %d Hz, %d Kanal(e), %s, %.1f s; Band: Mitte %.0f Hz, Breite ca. %.0f Hz\n", argv[1], fs, ch, real ? "REELLES ZF-Signal" : "komplex I/Q", (double)n/fs, fc, bw);
    /* 1) in Basisband mischen (reell: analytisches Signal ueber Mischung + Tiefpass) */
    double ph = 0, dph = -2*M_PI*fc/fs;
    for (int i = 0; i < n; i++) {
        double c = cos(ph), s = sin(ph); ph += dph; if (ph < -M_PI) ph += 2*M_PI;
        double a = xr[i], b = xi[i]; xr[i] = a*c - b*s; xi[i] = a*s + b*c;
        if (real) { xr[i] *= 2; xi[i] *= 2; }          /* Amplitude des positiven Seitenbandes erhalten */
    }
    /* 2) Tiefpass: Blackman-Sinc, Grenze 5.8 kHz, Uebergang ~1 kHz -> 255 Taps bei 44.1 kHz */
    int taps = (int)(5.5 * fs / 1000.0) | 1; if (taps < 63) taps = 63; if (taps > 1201) taps = 1201;
    double *h = malloc(taps*sizeof(double)), cut = 5800.0/fs, hs = 0;
    for (int i = 0; i < taps; i++) { double t = i - (taps-1)/2.0; double sc = (t == 0) ? 2*cut : sin(2*M_PI*cut*t)/(M_PI*t);
        h[i] = sc*(0.42 - 0.5*cos(2*M_PI*i/(taps-1)) + 0.08*cos(4*M_PI*i/(taps-1))); hs += h[i]; }
    for (int i = 0; i < taps; i++) h[i] /= hs;
    double *yr = malloc(n*sizeof(double)), *yi = malloc(n*sizeof(double)); int dl = (taps-1)/2;
    for (int i = 0; i < n; i++) { double sr = 0, si = 0; for (int k = 0; k < taps; k++) { int j = i + dl - k; if (j >= 0 && j < n) { sr += h[k]*xr[j]; si += h[k]*xi[j]; } } yr[i] = sr; yi[i] = si; }
    /* 3) bandbegrenzte Interpolation auf 12 kHz (32 Taps, Blackman) */
    int nout = (int)((double)(n - 64) * OUT_FS / fs); int16_t *o = malloc((size_t)nout*4);
    double rmax = 0;
    for (int m = 0; m < nout; m++) {
        double t = 32 + (double)m * fs / OUT_FS; int i0 = (int)floor(t); double fr = t - i0, sr = 0, si = 0;
        for (int k = -15; k <= 16; k++) { double u = fr - k; double sc = (fabs(u) < 1e-12) ? 1 : sin(M_PI*u)/(M_PI*u);
            double wv = 0.42 + 0.5*cos(M_PI*u/16.5) + 0.08*cos(2*M_PI*u/16.5); sr += yr[i0+k]*sc*wv; si += yi[i0+k]*sc*wv; }
        if (fabs(sr) > rmax) rmax = fabs(sr); if (fabs(si) > rmax) rmax = fabs(si);
        o[2*m] = (int16_t)lrint(fmax(-32768, fmin(32767, sr))); o[2*m+1] = (int16_t)lrint(fmax(-32768, fmin(32767, si)));
    }
    wr_wav(argv[2], o, nout, OUT_FS);
    fprintf(stderr, "-> %s: %d Samples, %.1f s, Spitze %.0f%s\n", argv[2], nout, (double)nout/OUT_FS, rmax, rmax > 30000 ? " (Uebersteuerung!)" : "");
    return 0;
}
