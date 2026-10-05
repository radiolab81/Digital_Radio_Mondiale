/* ==========================================================================================================
 * drm_test.c  --  Kommandozeilenwerkzeug: 12-kHz-I/Q-WAV -> Empfaenger (drm_rx) -> Status + Audio-Rahmen-Datei
 * ==========================================================================================================
 * Aufruf:  drm_test eingabe_12k.wav [rahmen.bin]
 * Eingabe: komplexes Basisband, 12 kHz, 16 Bit stereo (I links, Q rechts); andere Quellen mit iq_prep umrechnen.
 * Ausgabe: Statuszeilen auf stdout; rahmen.bin (Standard drm_audio_frames.bin) mit allen Audiorahmen:
 *            uint16 L, L Byte     L = Rahmenlaenge (xHE-AAC: inkl. CRC-16; AAC: Rohrahmen)
 *            L = 0                Rahmen mit CRC-Fehler           L = 0xFFFF   Superframe verloren
 *          rahmen.bin.meta: "<Konfig-Hex> <Rate Hz> <Modus> <Codec> <SBR> <Rahmen je Superframe>" fuer xhe_dec/aac_dec.
 * Das Werkzeug speist den Empfaenger in Bloeckchen zu 1000 Abtastwerten, wie es auch ein Audio-Callback taete. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drm_rx.h"
#include "drm_audio.h"

static drm_cs16_t *load_wav(const char *fn, int *n, int *fs)
{
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); exit(1); }
    uint8_t h[12]; if (fread(h, 1, 12, f) != 12) exit(1);
    uint32_t sz; uint8_t id[4]; long dpos = 0; uint32_t dlen = 0; uint16_t fmt[8] = {0};
    while (fread(id, 1, 4, f) == 4 && fread(&sz, 4, 1, f) == 1) {                  /* RIFF-Chunks durchlaufen */
        if (!memcmp(id, "fmt ", 4)) { uint8_t b[16]; if (fread(b, 1, 16, f) != 16) exit(1); memcpy(fmt, b, 16); if (sz > 16) fseek(f, sz - 16 + (sz & 1), SEEK_CUR); continue; }
        if (!memcmp(id, "data", 4)) { dpos = ftell(f); dlen = sz; break; }
        fseek(f, sz + (sz & 1), SEEK_CUR);
    }
    if (fmt[1] != 2 || fmt[7] != 16) { fprintf(stderr, "%s: erwartet 16 Bit Stereo (I/Q)\n", fn); exit(1); }
    *fs = (int)(fmt[2] | (fmt[3] << 16)); *n = (int)(dlen / 4);
    drm_cs16_t *b = malloc(dlen); fseek(f, dpos, SEEK_SET); if (fread(b, 4, (size_t)*n, f) != (size_t)*n) exit(1); fclose(f); return b;
}

static FILE *fo;
static void on_event(void *user, int type, const void *data, int len)
{
    (void)user;
    if (type == DRM_EV_STATE) { static const char *nm[3] = {"ACQ (Suche)", "SYNC (Rahmentakt)", "TRACK (Empfang)"}; printf("  [Zustand] %s\n", nm[len]); }
    else if (type == DRM_EV_AUDIO && fo) {
        uint16_t Z = (len > 0) ? (uint16_t)len : (len == 0 ? 0 : 0xFFFF);
        fwrite(&Z, 2, 1, fo); if (len > 0) fwrite(data, 1, (size_t)len, fo);
    }
}

int main(int argc, char **argv)
{
    int n, fs; const char *fn = argc > 1 ? argv[1] : "DRM_6030_000_iq.wav", *out = argc > 2 ? argv[2] : "drm_audio_frames.bin";
    drm_cs16_t *iq = load_wav(fn, &n, &fs);
    if (fs != DRM_FS) { fprintf(stderr, "Abtastrate %d != 12000: erst mit iq_prep umrechnen\n", fs); return 1; }
    static char mem[1 << 21] __attribute__((aligned(16)));                                /* Platz fuer den Empfaenger (statisch) */
    if (drm_rx_size() > sizeof mem) { fprintf(stderr, "Speicher zu klein: %zu\n", drm_rx_size()); return 1; }
    drm_rx_t *rx = drm_rx_init(mem, sizeof mem);
    fo = fopen(out, "wb"); drm_rx_set_callback(rx, on_event, NULL);
    printf("%s: %.1f s, Empfaengerspeicher %.0f kB\n", fn, (double)n / fs, drm_rx_size() / 1024.0);
    for (int i = 0; i < n; i += 1000) drm_rx_push(rx, iq + i, (n - i < 1000) ? n - i : 1000);
    fclose(fo);

    drm_stats_t st; drm_rx_stats(rx, &st); const drm_fac_t *f = drm_rx_fac(rx); const drm_sdc_t *s = drm_rx_sdc(rx);
    printf("Modus %c, Belegung %d (%d Traeger), Frequenzversatz %.1f Hz, Re-Akquisitionen %d\n", 'A' + st.mode, st.occ, st.ncar, st.foff_hz, st.reacq);
    printf("Pilot-SNR (Datenzellen) %.1f dB;  FAC-CRC ok %d/%d, SDC-CRC ok %d/%d\n", st.pilot_snr_db, st.fac_ok, st.fac_n, st.sdc_ok, st.sdc_n);
    if (!f) { printf("kein gueltiger FAC\n"); return 1; }
    printf("FAC: MSC %s, SDC %s, Verschachtelung %s, %d Dienste, Sender-ID 0x%06x\n", f->msc_mode == 3 ? "16-QAM" : "64-QAM", f->sdc_mode ? "4-QAM" : "16-QAM", f->interleaver_depth ? "kurz" : "lang", f->nservices, f->service_id);
    if (s) {
        printf("SDC: Schutz A=%d B=%d, %d Strom/Stroeme", s->prot_a, s->prot_b, s->nstreams);
        for (int i = 0; i < s->nstreams; i++) printf(", [%d] A=%d B=%d Byte", i, s->len_a[i], s->len_b[i]);
        printf("\n"); for (int i = 0; i < 4; i++) if (s->label[i][0]) printf("  Label %d: \"%s\"\n", i, s->label[i]);
        if (s->have_audio) {
            static const char *cd[4] = {"AAC", "reserviert", "reserviert", "xHE-AAC"};
            printf("  Audio: %s, Modus %d, SBR %d, Rate-Code %d, Text %d\n", cd[s->a_coding], s->a_mode, s->a_sbr, s->a_rate, s->a_text);
            static const int xr[8] = {0, 0, 16000, 19200, 24000, 32000, 38400, 48000};
            char mn[512]; snprintf(mn, sizeof mn, "%s.meta", out); FILE *fm = fopen(mn, "w");
            if (fm) { if (!s->a_cfglen) fprintf(fm, "-"); for (int i = 0; i < s->a_cfglen; i++) fprintf(fm, "%02x", s->a_cfg[i]);
                      int rate = s->a_coding == 0 ? (s->a_rate == 1 ? 12000 : s->a_rate == 3 ? 24000 : 48000) : xr[s->a_rate & 7];
                      fprintf(fm, " %d %d %d %d %d\n", rate, s->a_mode, s->a_coding, s->a_sbr, s->a_coding == 0 ? drm_aac_nframes(s->a_rate) : 0); fclose(fm); }
        }
    }
    printf("MSC: Audio-Superframe-Kopf ok %d/%d; Rahmen: %d gueltig, %d CRC-Fehler, %d Superframes verloren\n", st.hdr_ok, st.hdr_n, st.frames_ok, st.frames_bad, st.frames_lost);
    return 0;
}
