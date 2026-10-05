/* fac_test.c - Stufe 2: Rahmensync, Kanalschaetzung, FAC-Dekodierung (nur PC-Testtreiber) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "drm_ofdm.h"
#include "drm_rx.h"
#include "drm_fac.h"
#include "drm_sdc.h"
#include "drm_msc.h"

static drm_cs16_t *load_wav(const char *fn, int *n)
{
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); exit(1); }
    uint8_t h[12]; if (fread(h, 1, 12, f) != 12) exit(1);
    uint32_t sz; uint8_t id[4]; long dpos = 0; uint32_t dlen = 0;
    while (fread(id, 1, 4, f) == 4 && fread(&sz, 4, 1, f) == 1) {
        if (!memcmp(id, "data", 4)) { dpos = ftell(f); dlen = sz; break; }
        fseek(f, sz + (sz & 1), SEEK_CUR);
    }
    *n = dlen / 4; drm_cs16_t *b = malloc(dlen); fseek(f, dpos, SEEK_SET);
    if (fread(b, 4, *n, f) != (size_t)*n) exit(1); fclose(f); return b;
}

int main(int argc, char **argv)
{
    int n; drm_cs16_t *iq = load_wav(argc > 1 ? argv[1] : "DRM_6030_000_iq.wav", &n);
    drm_modeb_init();
    drm_ofdm_t o; drm_ofdm_init(&o);
    float foff, met; int t0 = drm_ofdm_acquire(&o, iq, n, 60, &foff, &met);
    drm_ofdm_set_freq(&o, foff);
    printf("Sync: t0=%d foff=%.2f Hz metric=%.3f\n", t0, foff, met);
    int nsym = (n - t0 - DRM_TS) / DRM_TS;

    /* 1) Rahmensync ueber Zeitreferenzen */
    static drm_cell_t cell[DRM_NCAR];
    double acc[DRM_NSYM_FRAME] = {0};
    for (int l = 0; l < 600; l++) {
        drm_ofdm_symbol(&o, iq + t0 + l*DRM_TS, cell);
        acc[l % DRM_NSYM_FRAME] += drm_time_ref_metric(cell, 8, 0);
    }
    int best = 0; for (int i = 1; i < DRM_NSYM_FRAME; i++) if (acc[i] > acc[best]) best = i;
    printf("Zeitreferenz-Metrik je Symbolphase (mean):"); for (int i = 0; i < 15; i++) printf(" %.2f", acc[i]/40);
    printf("\n=> Rahmenstart bei l mod 15 = %d\n", best);

    /* 2) Pass 1: alle Symbole demodulieren (mit Timing-/Frequenznachfuehrung), rotierte Zellen speichern */
    drm_ofdm_init(&o); drm_ofdm_set_freq(&o, foff);
    o.fft_adv = getenv("DRM_ADV") ? atoi(getenv("DRM_ADV")) : 12;
    float TG_ = getenv("TGAIN") ? (float)atof(getenv("TGAIN")) : 0.003f, FG_ = getenv("FGAIN") ? (float)atof(getenv("FGAIN")) : 0.01f;
    { extern int drm_ce_mode; if (getenv("CE_MODE")) drm_ce_mode = atoi(getenv("CE_MODE")); }
    int L = nsym - best; static float (*XR)[DRM_NCAR], (*XI)[DRM_NCAR];
    XR = malloc((size_t)L*sizeof *XR); XI = malloc((size_t)L*sizeof *XI);
    static float hr[DRM_NCAR], hi[DRM_NCAR];
    for (int l = 0; l < best; l++) drm_ofdm_symbol(&o, iq + t0 + l*DRM_TS, cell);
    float tcorr = 0, fcorr = foff, pfe[6] = {0}; int have_prev = 0;
    float trate = getenv("TRATE0") ? (float)atof(getenv("TRATE0")) : 0.0f;
    float tsign = getenv("TSIGN") ? (float)atof(getenv("TSIGN")) : 0.0f;   /* 0 = altes Verfahren */
    for (int l = 0; l < L; l++) {
        int s = l % DRM_NSYM_FRAME;
        int dint = (int)lrintf(tcorr);
        int pos = t0 + (best+l)*DRM_TS + dint;
        drm_ofdm_symbol(&o, iq + pos, cell);
        int adv_eff = (tsign != 0.0f) ? o.fft_adv - dint : o.fft_adv;   /* Fensterversatz exakt herausrechnen */
        drm_chan_est(cell, s, adv_eff, XR[l], XI[l], hr, hi);
        float sl, fe[6]; drm_pilot_stats(XR[l], XI[l], s, &sl, fe);
        float e = sl * DRM_TU / (2.0f*(float)M_PI);
        if (tsign != 0.0f) tcorr += TG_ * (tsign * e - tcorr);        /* offener Schaetzer, kein Regelkreis */
        else { float ki = getenv("TKI") ? (float)atof(getenv("TKI")) : 0.0f; trate -= ki * e; tcorr += trate - TG_ * e; }
        if (have_prev) { float pr=0,pi=0; for (int q=0;q<3;q++){ pr += fe[2*q]*pfe[2*q]+fe[2*q+1]*pfe[2*q+1]; pi += fe[2*q+1]*pfe[2*q]-fe[2*q]*pfe[2*q+1]; }
            fcorr += FG_ * atan2f(pi,pr) * DRM_FS / (2.0f*(float)M_PI*DRM_TS); drm_ofdm_set_freq(&o, fcorr); }
        memcpy(pfe, fe, sizeof fe); have_prev = 1;
    }
    printf("Nachfuehrung: Drift %.4f Samples/Symbol, tcorr=%.1f Samples, fcorr=%.2f Hz nach %.1f s\n", trate, tcorr, fcorr, (double)L*DRM_TS/DRM_FS);
    /* Pass 2: Frame fuer Frame FAC dekodieren, Kanalschaetzung ueber Zeit und Frequenz */
    int nfr = 0, nok = 0; int idcount[4] = {0}; int firstok = 1;
    int phvote[3] = {0,0,0}; static int frid[400];
    for (int base = DRM_NSYM_FRAME; base + DRM_NSYM_FRAME + 2 <= L; base += DRM_NSYM_FRAME) {
        int8_t soft[130]; int ns = 0; float scale = 0; int cnt = 0; float sr[130];
        for (int s = 0; s < DRM_NSYM_FRAME; s++) {
            int l = base + s;
            float *wr[DRM_TW], *wi[DRM_TW]; int sph[DRM_TW];
            for (int w = 0; w < DRM_TW; w++) { wr[w] = XR[l-2+w]; wi[w] = XI[l-2+w]; sph[w] = (l-2+w) % DRM_NSYM_FRAME; }
            drm_chan_est_tf(wr, wi, sph, hr, hi);
            for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
                int i = k - DRM_KMIN;
                if (drm_map[s][i].type != CT_FAC) continue;
                float re = XR[l][i]*hr[i] + XI[l][i]*hi[i];
                float im = XI[l][i]*hr[i] - XR[l][i]*hi[i];
                sr[ns++] = re; sr[ns++] = im; scale += fabsf(re) + fabsf(im); cnt += 2;
            }
        }
        scale = 40.0f / (scale/cnt + 1e-9f);
        for (int i = 0; i < 130; i++) { float v = sr[i]*scale; if (v > 127) v = 127; if (v < -127) v = -127; soft[i] = (int8_t)lrintf(v); }
        drm_fac_t f; drm_fac_decode(soft, &f);
        frid[nfr] = f.crc_ok ? f.identity : -1;
        if (f.crc_ok) phvote[((nfr - (f.identity==3?0:f.identity)) % 3 + 3) % 3]++;
        nfr++; if (f.crc_ok) { nok++; idcount[f.identity]++; }
        if (f.crc_ok && firstok) { firstok = 0; printf("Erster gueltiger FAC (Frame %d): id=%d RM=%d occ=%d il=%d msc=%d sdc=%d nserv=%d sid=0x%06x lang=%d audio/data=%d\n",
             nfr, f.identity, f.rm_flag, f.spectrum_occ, f.interleaver_depth, f.msc_mode, f.sdc_mode, f.nservices, f.service_id, f.language, f.audio_data); }
        printf("%c", f.crc_ok ? '#' : '.'); if (nfr % 75 == 0) printf("\n");
    }
    printf("\n");
    int sfph = 0; for (int i = 1; i < 3; i++) if (phvote[i] > phvote[sfph]) sfph = i;
    printf("Superframe-Phase (Frame mod 3) = %d  (Stimmen %d/%d/%d)\n", sfph, phvote[0], phvote[1], phvote[2]);
    /* Pass 3: SDC */
    { int nsf = 0, nsdc_ok = 0, shown = 0;
      for (int fr = sfph; fr < nfr; fr += 3) {
        int base = DRM_NSYM_FRAME + fr*DRM_NSYM_FRAME;               /* wie in Pass 2 */
        int8_t soft[2*SDC_NCELL]; float sr[2*SDC_NCELL]; int ns = 0; float scale = 0;
        static float hr2[DRM_NCAR], hi2[DRM_NCAR];
        for (int s = 0; s < 2; s++) {
            int l = base + s; float *wr[DRM_TW], *wi[DRM_TW]; int sph[DRM_TW];
            for (int w = 0; w < DRM_TW; w++) { wr[w] = XR[l-2+w]; wi[w] = XI[l-2+w]; sph[w] = (l-2+w) % DRM_NSYM_FRAME; }
            drm_chan_est_tf(wr, wi, sph, hr2, hi2);
            for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
                int i = k - DRM_KMIN; if (drm_map[s][i].type != CT_DATA) continue;
                if (ns >= 2*SDC_NCELL) { ns += 2; continue; }
                float re = XR[l][i]*hr2[i] + XI[l][i]*hi2[i], im = XI[l][i]*hr2[i] - XR[l][i]*hi2[i];
                sr[ns++] = re; sr[ns++] = im; scale += fabsf(re) + fabsf(im);
            }
        }
        if (ns != 2*SDC_NCELL) { printf("SDC-Zellzahl %d (erwartet %d)\n", ns/2, SDC_NCELL); break; }
        scale = 40.0f / (scale/ns + 1e-9f);
        for (int i = 0; i < ns; i++) { float v = sr[i]*scale; if (v>127) v=127; if (v<-127) v=-127; soft[i] = (int8_t)lrintf(v); }
        drm_sdc_t sd; drm_sdc_decode(soft, &sd); nsf++;
        if (sd.crc_ok) nsdc_ok++;
        if (sd.crc_ok && shown < 1) { shown++;
            printf("SDC (Superframe %d): CRC OK, AFS-Index %d\n", nsf, sd.afs_index);
            printf("  Multiplex: %d Stream(s), Schutz A=%d B=%d\n", sd.nstreams, sd.prot_a, sd.prot_b);
            for (int i = 0; i < sd.nstreams; i++) printf("    Stream %d: Teil A %d Byte, Teil B %d Byte\n", i, sd.len_a[i], sd.len_b[i]);
            for (int i = 0; i < 4; i++) if (sd.label[i][0]) printf("  Label Short-Id %d: \"%s\"\n", i, sd.label[i]);
            if (sd.have_audio) { printf("  Audio: short=%d stream=%d coding=%d(0=AAC,3=xHE) sbr=%d mode=%d rate=%d text=%d enh=%d cfg=%d Byte:",
                sd.a_short, sd.a_stream, sd.a_coding, sd.a_sbr, sd.a_mode, sd.a_rate, sd.a_text, sd.a_enh, sd.a_cfglen);
                for (int i = 0; i < sd.a_cfglen; i++) printf(" %02x", sd.a_cfg[i]); printf("\n"); }
        }
      }
      printf("SDC-CRC ok: %d / %d Superframes\n", nsdc_ok, nsf); }
    /* ---- Pass 4: MSC ---- */
    { int nsf = 0; for (int fr = sfph; fr + 3 <= nfr; fr += 3) nsf++;
      int NF = nsf*3; msc_cell_t *mf = calloc((size_t)NF*MSC_NMUX, sizeof *mf);
      static float hrm[DRM_NCAR], him[DRM_NCAR]; int sf = 0;
      for (int fr = sfph; fr + 3 <= nfr; fr += 3, sf++) {
        int cnt = 0;
        for (int f = 0; f < 3; f++) for (int s = 0; s < DRM_NSYM_FRAME; s++) {
            if (f == 0 && s < 2) continue;                       /* SDC */
            int l = DRM_NSYM_FRAME + (fr+f)*DRM_NSYM_FRAME + s; float *wr[DRM_TW], *wi[DRM_TW]; int sph[DRM_TW];
            for (int w = 0; w < DRM_TW; w++) { wr[w] = XR[l-2+w]; wi[w] = XI[l-2+w]; sph[w] = (l-2+w) % DRM_NSYM_FRAME; }
            drm_chan_est_tf(wr, wi, sph, hrm, him);
            for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
                int i = k - DRM_KMIN; if (drm_map[s][i].type != CT_DATA) continue;
                if (cnt >= 3*MSC_NMUX) { cnt++; continue; }
                float h2 = hrm[i]*hrm[i] + him[i]*him[i] + 1e-12f;
                msc_cell_t *m = &mf[(size_t)sf*3*MSC_NMUX + cnt];
                m->zr = (XR[l][i]*hrm[i] + XI[l][i]*him[i]) / h2;
                m->zi = (XI[l][i]*hrm[i] - XR[l][i]*him[i]) / h2;
                m->w = h2; cnt++;
            }
        }
        if (sf == 0) printf("MSC-Zellen je Superframe: %d (erwartet %d + 2 unbenutzt)\n", cnt, 3*MSC_NMUX);
      }
      /* Multiplexrahmen dekodieren, Audio-Superframe (Stream 0: 467 Byte) pruefen */
      static msc_cell_t dq[MSC_NMUX]; static uint8_t bits[MSC_LBITS];
      int ndec = 0, nhdr = 0, ngood = 0, nfrm = 0, nfcrc = 0; double qsum = 0;
      uint8_t openf[8192]; int openlen = 0, have_open = 0;
      FILE *fo = fopen("drm_audio_frames.bin", "wb");
      for (int q = 0; q + MSC_D <= NF; q++) {
        drm_msc_deinterleave(mf + (size_t)q*MSC_NMUX, dq);
        int qual = drm_msc_decode(dq, bits); qsum += qual; ndec++;
        uint8_t by[MSC_LBITS/8]; for (int i = 0; i < MSC_LBITS/8; i++) { unsigned v = 0; for (int b = 0; b < 8; b++) v = (v<<1) | bits[8*i+b]; by[i] = (uint8_t)v; }
        const int ASF = 467 - 4;   /* letzte 4 Byte = Textmeldung */ const uint8_t *a = by;
        uint8_t hb[8]; for (int b = 0; b < 8; b++) hb[b] = (uint8_t)((a[0] >> (7-b)) & 1);
        uint8_t r = 0xff; for (int b = 0; b < 8; b++) { uint8_t fb = ((r>>7)&1) ^ hb[b]; r <<= 1; if (fb) r ^= 0x1d; }
        int hdr_ok = ((uint8_t)~r == a[1]);
        if (q < 6) printf("  MSC-Frame %d: Ebene-0-Uebereinstimmung %d/1000, Header %02x %02x CRC %s\n", q, qual, a[0], a[1], hdr_ok ? "OK" : "BAD");
        if (!hdr_ok) { have_open = 0; { uint16_t Z = 0xFFFF; fwrite(&Z, 2, 1, fo); } continue; }
        nhdr++;
        int b = a[0] >> 4, plen = ASF - 2 - 2*b;
        if (plen < 0) { have_open = 0; { uint16_t Z = 0xFFFF; fwrite(&Z, 2, 1, fo); } continue; }
        const uint8_t *pay = a + 2; int bord[16], nb = 0;
        for (int i = 0; i < b; i++) { int off = ASF - 2*(i+1); int idx = (a[off] << 4) | (a[off+1] >> 4); bord[i] = idx; }   /* letztes Verzeichniselement = erste Grenze */
        for (int i = 0; i < b; i++) if (bord[i] < plen) bord[nb++] = bord[i];
        if (q < 40 && hdr_ok) { printf("   q=%d b=%d plen=%d borders:", q, b, plen); for (int i = 0; i < b; i++) printf(" %d", (a[ASF-2*(i+1)]<<4)|(a[ASF-2*(i+1)+1]>>4)); printf("  cnt-nibbles:"); for (int i = 0; i < b; i++) printf(" %d", a[ASF-2*(i+1)+1]&15); printf("\n"); }
        int start = (nb ? bord[0] : plen);
        if (have_open) { if (openlen + start < (int)sizeof openf) { memcpy(openf+openlen, pay, start); openlen += start;
            nfrm++; if (openlen >= 3) { /* CRC16 ueber AU */
              uint16_t c16 = 0xffff; for (int i = 0; i < openlen-2; i++) for (int bb = 7; bb >= 0; bb--) { uint16_t fb = ((c16>>15)&1) ^ ((openf[i]>>bb)&1); c16 <<= 1; if (fb) c16 ^= 0x1021; }
              if ((uint16_t)~c16 == (uint16_t)((openf[openlen-2]<<8) | openf[openlen-1])) { nfcrc++; uint16_t L = (uint16_t)openlen; fwrite(&L, 2, 1, fo); fwrite(openf, 1, openlen, fo); } else { uint16_t Z = 0; fwrite(&Z, 2, 1, fo); } } } }
        for (int i = 0; i < nb; i++) {
            int e = (i+1 < nb) ? bord[i+1] : plen; int len = e - bord[i];
            if (i+1 < nb) { nfrm++;
              uint16_t c16 = 0xffff; for (int k = 0; k < len-2; k++) for (int bb = 7; bb >= 0; bb--) { uint16_t fb = ((c16>>15)&1) ^ ((pay[bord[i]+k]>>bb)&1); c16 <<= 1; if (fb) c16 ^= 0x1021; }
              if (len >= 3 && (uint16_t)~c16 == (uint16_t)((pay[e-2]<<8) | pay[e-1])) { nfcrc++; uint16_t L = (uint16_t)len; fwrite(&L, 2, 1, fo); fwrite(pay+bord[i], 1, len, fo); } else { uint16_t Z = 0; fwrite(&Z, 2, 1, fo); } }
            else { openlen = len; memcpy(openf, pay+bord[i], len); have_open = 1; }
        }
        if (nb == 0) { if (have_open && openlen + plen < (int)sizeof openf) { memcpy(openf+openlen, pay, plen); openlen += plen; } }
        ngood++;
      }
      fclose(fo);
      printf("MSC: %d Multiplexrahmen dekodiert, mittlere Ebene-0-Uebereinstimmung %.0f/1000\n", ndec, ndec ? qsum/ndec : 0.0);
      printf("     Audio-Superframe-Header-CRC ok: %d / %d,  Audio-Frames mit gueltiger CRC16: %d / %d\n", nhdr, ndec, nfcrc, nfrm);
      free(mf); }
    printf("FAC-CRC ok: %d / %d Frames  (Identity 0/1/2/3: %d %d %d %d)\n", nok, nfr, idcount[0], idcount[1], idcount[2], idcount[3]);
    return 0;
}
