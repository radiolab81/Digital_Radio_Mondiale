/* ==========================================================================================================
 * drm_gen.c  --  Synthetischer DRM-Sender fuer Ende-zu-Ende-Tests (nur PC, kein Teil des Empfaengers)
 * ==========================================================================================================
 * Erzeugt ein vollstaendiges DRM-Basisbandsignal (12 kHz komplex) mit FAC, SDC und MSC fuer die Modi A-D, beliebige
 * Belegung, 16-/64-QAM, EEP oder UEP, xHE-AAC- oder AAC-artige Audio-Superframes (Zufallsnutzdaten mit korrekten
 * CRCs/Kopfdaten), optional Frequenzoffset und Rauschen.  Der Sender nutzt dieselben Tabellen wie der Empfaenger
 * (drm_mode.c); er prueft also die Konsistenz der Kette und die Algorithmen, nicht die Tabellenwerte selbst.
 *
 * Aufbau eines Senderrahmens (400 ms):  Zellenraster map[s][k] aus drm_mode.c
 *   Pilotzellen        = Referenzwert (drm_ref_value)
 *   FAC-Zellen         = drm_fac_encode (65 Zellen)
 *   SDC-Zellen         = drm_sdc_encode4 (nur im ersten Rahmen jedes Superframes, in den ersten Symbolen)
 *   Datenzellen (MSC)  = drm_msc_encode + Zellverschachtelung
 *   OFDM: IFFT je Symbol, Guardintervall = Kopie der letzten Tg Werte [ES 201 980, 8.1/8.2].
 * Aufruf: drm_gen out.wav mode(A-D) occ(0-3) msc(16|64) plA plB bytesA codec(xhe|aac) superframes snr_db foff_hz [seed]
 * ========================================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "drm_fac.h"
#include "drm_sdc.h"
#include "drm_msc.h"
#include "drm_audio.h"
#include "drm_fft.h"

static double gauss(void)
{ double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }
static void setbits(uint8_t *b, int pos, int n, unsigned v) { for (int i = 0; i < n; i++) b[pos + i] = (uint8_t)((v >> (n - 1 - i)) & 1); }

static uint16_t crc16(const uint8_t *d, int n) { uint16_t c = 0xffff; for (int i = 0; i < n; i++) for (int b = 7; b >= 0; b--) { uint16_t fb = ((c >> 15) & 1) ^ ((d[i] >> b) & 1); c <<= 1; if (fb) c ^= 0x1021; } return (uint16_t)~c; }
static uint8_t crc8b(const uint8_t *bits, int n) { uint8_t r = 0xff; for (int i = 0; i < n; i++) { uint8_t fb = ((r >> 7) & 1) ^ bits[i]; r <<= 1; if (fb) r ^= 0x1d; } return (uint8_t)~r; }

/* ---- Audio-Superframe (xHE-AAC) aus einem fortlaufenden Strom zufaelliger Rahmen mit CRC-16 bauen ------------ */
static uint8_t stream[1 << 18]; static int stream_len, stream_pos;
static int frame_start[4096], nstart;
static void ensure_stream(int need)
{
    while (stream_len - stream_pos < need) {
        int len = 60 + rand() % 90;                                  /* Rahmenlaenge inkl. CRC-16 */
        for (int i = 0; i < len - 2; i++) stream[stream_len + i] = (uint8_t)rand();
        uint16_t c = crc16(stream + stream_len, len - 2); stream[stream_len + len - 2] = (uint8_t)(c >> 8); stream[stream_len + len - 1] = (uint8_t)c;
        frame_start[nstart++] = stream_len; stream_len += len;
    }
}
static void build_xhe_asf(uint8_t *out, int ASF)
{
    /* Gesucht: Zahl b der Rahmengrenzen im Nutzteil, sodass b Grenzen genau in die Nutzteillaenge ASF-2-2b fallen.  Passt
     * keine, werden die noch nicht gesendeten Rahmen mit anderen Zufallslaengen neu gewuerfelt. */
    for (int attempt = 0; attempt < 200; attempt++) {
        for (int b = 0; b <= 15; b++) {
            int plen = ASF - 2 - 2 * b; ensure_stream(plen + 200);
            int pos[16], nb = 0;
            for (int i = 0; i < nstart; i++) if (frame_start[i] >= stream_pos && frame_start[i] < stream_pos + plen && nb < 16) pos[nb++] = frame_start[i] - stream_pos;
            if (nb != b) continue;
            memset(out, 0, (size_t)ASF);
            out[0] = (uint8_t)(b << 4 | 5);                                   /* Zahl der Grenzen | Bit-Reservoir (fest 5) */
            uint8_t bits[8]; for (int i = 0; i < 8; i++) bits[i] = (out[0] >> (7 - i)) & 1; out[1] = crc8b(bits, 8);
            memcpy(out + 2, stream + stream_pos, (size_t)plen);
            for (int i = 0; i < b; i++) { int off = ASF - 2 * (i + 1); out[off] = (uint8_t)(pos[i] >> 4); out[off + 1] = (uint8_t)((pos[i] & 15) << 4 | (b & 15)); }
            stream_pos += plen; return;
        }
        int f0 = 0; while (f0 < nstart && frame_start[f0] < stream_pos) f0++;     /* erster noch nicht begonnener Rahmen */
        if (f0 < nstart) { stream_len = frame_start[f0]; nstart = f0; }            /* ab dort neu wuerfeln */
    }
    fprintf(stderr, "ASF-Bau fehlgeschlagen\n"); exit(1);
}
/* AAC-artiger Superframe [5.4.1]: Kopf (n-1 Grenzen zu 12 Bit), n CRC-Bytes, n Rahmen mit Zufallsinhalt */
static void build_aac_asf(uint8_t *out, int ASF, int n)
{
    const int hbits = (n - 1) * 12 + (n == 10 ? 4 : 0), hb = hbits / 8, off = hb + n, plen = ASF - off;
    int bord[11]; bord[0] = 0; for (int f = 1; f < n; f++) bord[f] = bord[f-1] + plen / n; bord[n] = plen;
    memset(out, 0, ASF);
    for (int i = 0; i < n - 1; i++) { int v = bord[i+1] & 0xfff; for (int k = 0; k < 12; k++) { int bit = 12 * i + k; if ((v >> (11 - k)) & 1) out[bit >> 3] |= (uint8_t)(1 << (7 - (bit & 7))); } }
    for (int f = 0; f < n; f++) out[hb + f] = (uint8_t)(0xA0 + f);
    for (int i = 0; i < plen; i++) out[off + i] = (uint8_t)rand();
}

int main(int argc, char **argv)
{
    if (argc < 11) { fprintf(stderr, "Aufruf: %s out.wav mode(A-D) occ msc(16|64) plA plB bytesA codec(xhe|aac) superframes snr_db foff_hz [seed]\n", argv[0]); return 1; }
    const char *outfn = argv[1]; const int mode = argv[2][0] - 'A', occ = atoi(argv[3]), qam = atoi(argv[4]), plA = atoi(argv[5]), plB = atoi(argv[6]), bytesA = atoi(argv[7]);
    const int xhe = !strcmp(argv[8], "xhe"), NSF = atoi(argv[9]); const double snr = atof(argv[10]), foff = argc > 11 ? atof(argv[11]) : 0; srand(argc > 12 ? atoi(argv[12]) : 1);
    static drm_mode_t M; if (drm_mode_setup(&M, mode, occ)) { fprintf(stderr, "Modus/Belegung nicht unterstuetzt\n"); return 1; }
    drm_msc_cfg_t cfg; const int depth = 5;
    if (drm_msc_setup(&cfg, qam, M.nmux, depth, plA, plB, bytesA) < 0) { fprintf(stderr, "MSC-Konfiguration unzulaessig\n"); return 1; }
    const int NS = M.nsym, NC = M.ncar, Tu = M.Tu, Ts = M.Ts;
    const int mux_bytes = cfg.lbits / 8, text = 0, asf_len = mux_bytes - text;       /* ein Audiostrom fuellt den Multiplexrahmen */

    /* --- SDC-Datenfeld: Typ 0 (ein Strom: A = bytesA, B = Rest) + Typ 9 (Audio) -------------------------------- */
    static uint8_t sdcbits[2048]; memset(sdcbits, 0, sizeof sdcbits);
    const int sdc_mode = 1; int p = 4;
    setbits(sdcbits, 0, 4, 0);                                                         /* AFS-Index */
    setbits(sdcbits, p, 7, 3); setbits(sdcbits, p + 7, 1, 0); setbits(sdcbits, p + 8, 4, 0); p += 12;               /* Typ 0: Laenge 3 Byte */
    setbits(sdcbits, p, 2, plA); setbits(sdcbits, p + 2, 2, plB); setbits(sdcbits, p + 4, 12, bytesA); setbits(sdcbits, p + 16, 12, asf_len - bytesA); p += 4 + 24;
    const int cfglen = xhe ? 5 : 0; static const uint8_t xcfg[5] = {0xe3, 0x26, 0xea, 0, 0};
    setbits(sdcbits, p, 7, 2 + cfglen); setbits(sdcbits, p + 7, 1, 0); setbits(sdcbits, p + 8, 4, 9); p += 12;       /* Typ 9: Audio */
    setbits(sdcbits, p, 2, 0); setbits(sdcbits, p + 2, 2, 0); setbits(sdcbits, p + 4, 2, xhe ? 3 : 0); setbits(sdcbits, p + 6, 1, 0); setbits(sdcbits, p + 7, 2, 0);
    setbits(sdcbits, p + 9, 3, xhe ? 6 : 3); setbits(sdcbits, p + 12, 1, 0); setbits(sdcbits, p + 13, 1, 0); setbits(sdcbits, p + 14, 5, 0); setbits(sdcbits, p + 19, 1, 0);
    for (int i = 0; i < cfglen; i++) setbits(sdcbits, p + 20 + 8 * i, 8, xcfg[i]);
    drm_sdc_finish(&M, sdc_mode, sdcbits);

    /* --- Ausgabe: NSF Superframes = 3*NSF Rahmen; Rahmen beginnen bei Symbol 0 -------------------------------------- */
    const long nsamp = (long)NSF * 3 * NS * Ts;
    float *sig = calloc((size_t)nsamp * 2, sizeof(float));
    drm_fft_plan_t plan; drm_fft_plan(&plan, Tu);
    static msc_cell_t hist[5][MSC_MAXN], txs[3][MSC_MAXN]; const msc_cell_t *hp[5];
    static uint8_t bits[MSC_MAXBITS], mux[MSC_MAXBITS / 8]; static float sdc_re[1024], sdc_im[1024];
    drm_sdc_encode4(&M, sdcbits, sdc_re, sdc_im);
    static float Xr[DRM_MAXSYM][DRM_MAXCAR], Xi[DRM_MAXSYM][DRM_MAXCAR];
    long pos = 0;
    for (int sf = 0; sf < NSF; sf++) {
        /* 3 Multiplexrahmen dieses Superframes: Bits -> QAM-Zellen -> Zellverschachtelung ueber die letzten D Rahmen [7.6] */
        for (int k = 0; k < 3; k++) {
            memset(mux, 0, sizeof mux);
            if (xhe) build_xhe_asf(mux, asf_len); else build_aac_asf(mux, asf_len, 10);
            for (int i = 0; i < mux_bytes; i++) for (int b = 0; b < 8; b++) bits[8 * i + b] = (mux[i] >> (7 - b)) & 1;
            for (int g = 4; g > 0; g--) memcpy(hist[g], hist[g - 1], sizeof hist[0]);           /* Historie: hist[g] = Rahmen vor g Schritten */
            drm_msc_encode(&cfg, bits, hist[0]);
            for (int g = 0; g < cfg.depth; g++) hp[g] = hist[g];
            drm_msc_interleave(&cfg, hp, txs[k]);
        }
        int mcnt = 0;                                                                              /* laufende MSC-Zelle im Superframe */
        for (int fr = 0; fr < 3; fr++) {
            uint8_t fac[64]; memset(fac, 0, sizeof fac);                                            /* FAC dieses Rahmens [6.3] */
            setbits(fac, 1, 2, (unsigned)fr); setbits(fac, 4, 3, occ); setbits(fac, 7, 1, 0); setbits(fac, 8, 2, qam == 16 ? 3 : 0);
            setbits(fac, 10, 1, sdc_mode); setbits(fac, 11, 4, 1); setbits(fac, 20, 24, 0x123456);
            float fre[65], fim[65]; drm_fac_encode(fac, fre, fim);
            memset(Xr, 0, sizeof Xr); memset(Xi, 0, sizeof Xi);
            int nf = 0, ns_sdc = 0;
            for (int s = 0; s < NS; s++) for (int i = 0; i < NC; i++) {
                const int k = i + M.kmin, t = M.map[s][i].type;
                if (t == CT_FREQ || t == CT_TIME || t == CT_GAIN) { float a, b; drm_ref_value(&M, s, k, &a, &b); Xr[s][i] = a; Xi[s][i] = b; }
                else if (t == CT_FAC) { Xr[s][i] = fre[nf]; Xi[s][i] = fim[nf]; nf++; }
                else if (t == CT_DATA) {
                    if (fr == 0 && s < M.nsdc_sym) { if (ns_sdc < M.nsdc) { Xr[s][i] = sdc_re[ns_sdc]; Xi[s][i] = sdc_im[ns_sdc]; ns_sdc++; } }
                    else { int c = mcnt++; if (c < 3 * cfg.n) { const msc_cell_t *q = &txs[c / cfg.n][c % cfg.n]; Xr[s][i] = q->zr; Xi[s][i] = q->zi; } }
                }
            }
            /* OFDM-Modulation: IFFT ueber conj-Trick  x = conj(FFT(conj(X))), Guard = letzte Tg Werte vorangestellt [8.1] */
            for (int s = 0; s < NS; s++) {
                drm_cell_t in[DRM_MAXTU], out[DRM_MAXTU]; memset(in, 0, sizeof in);
                for (int i = 0; i < NC; i++) { int k = i + M.kmin; in[(k + Tu) % Tu].re = Xr[s][i]; in[(k + Tu) % Tu].im = -Xi[s][i]; }
                drm_fft(&plan, in, out);
                for (int n = 0; n < Ts; n++) { int src = (n < M.Tg) ? Tu - M.Tg + n : n - M.Tg; sig[2 * (pos + n)] = out[src].re; sig[2 * (pos + n) + 1] = -out[src].im; }
                pos += Ts;
            }
        }
    }
    /* Rauschen: Es/N0 je Zelle = Tu / sigma^2 bei Einheitszellen (FFT summiert Tu Werte, Signal Amplitude Tu) */
    const double sig2 = Tu / pow(10.0, snr / 10.0);
    double pw = 0; for (long n = 0; n < nsamp; n++) pw += (double)sig[2*n]*sig[2*n] + (double)sig[2*n+1]*sig[2*n+1];
    const double g = 2500.0 / sqrt(pw / nsamp + sig2);                                         /* Normierung auf ca. 2500 Effektivwert */
    int16_t *o = malloc((size_t)nsamp * 4); double ph = 0;
    for (long n = 0; n < nsamp; n++) {
        double xr = sig[2*n] + sqrt(sig2 / 2) * gauss(), xi = sig[2*n+1] + sqrt(sig2 / 2) * gauss();
        double c = cos(ph), sn = sin(ph); ph += 2 * M_PI * foff / DRM_FS;
        double yr = xr * c - xi * sn, yi = xr * sn + xi * c;                                    /* Frequenzversatz */
        o[2*n] = (int16_t)lrint(fmax(-32768, fmin(32767, yr * g))); o[2*n+1] = (int16_t)lrint(fmax(-32768, fmin(32767, yi * g)));
    }
    FILE *f = fopen(outfn, "wb"); uint32_t nb2 = (uint32_t)nsamp * 4, v; uint16_t sh;
    fwrite("RIFF", 1, 4, f); v = 36 + nb2; fwrite(&v, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f); v = 16; fwrite(&v, 4, 1, f); sh = 1; fwrite(&sh, 2, 1, f); sh = 2; fwrite(&sh, 2, 1, f);
    v = DRM_FS; fwrite(&v, 4, 1, f); v = DRM_FS * 4; fwrite(&v, 4, 1, f); sh = 4; fwrite(&sh, 2, 1, f); sh = 16; fwrite(&sh, 2, 1, f); fwrite("data", 1, 4, f); fwrite(&nb2, 4, 1, f); fwrite(o, 4, (size_t)nsamp, f); fclose(f);
    printf("%s: Modus %c, Belegung %d, %d Traeger, %d-QAM, PL_A=%d PL_B=%d X=%d, N_MUX=%d (N1=%d), L_MUX=%d Bit, %d Superframes (%.1f s), SNR %.1f dB, Offset %.1f Hz\n",
           outfn, 'A' + mode, occ, NC, qam, plA, plB, bytesA, cfg.n, cfg.n1, cfg.lbits, NSF, (double)nsamp / DRM_FS, snr, foff);
    return 0;
}
