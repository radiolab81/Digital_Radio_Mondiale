/* ==========================================================================================================
 * aac_dec.c  --  DRM-AAC-Rahmen (aus drm_test) mit FAAD2 im DRM-Modus nach WAV dekodieren
 * ==========================================================================================================
 * Das klassische DRM-Audio ist ER-AAC (Object Type 20 "scalable", HCR/VCB11/RVLC, 960er Transformation) mit DRM-
 * eigener SBR-/PS-Syntax [ES 201 980, 5.3.x; ISO/IEC 14496-3].  Standard-ADTS-Dekoder koennen das nicht; FAAD2
 * bietet dafuer NeAACDecInitDRM() (Bibliothek mit DRM-Option uebersetzen!).  Die Rahmen kommen aus
 * drm_aac_asf_parse() (drm_audio.c); Konfiguration steht in rahmen.bin.meta.
 * Getestet: mit einer von Dream erzeugten synthetischen Datei bestaetigt, dass die Kette laeuft.
 * ========================================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <neaacdec.h>

static void wav_hdr(FILE *f, uint32_t rate, int ch, uint32_t nbytes)
{
    uint8_t h[44]; uint32_t v; uint16_t s;
    memcpy(h, "RIFF", 4); v = 36 + nbytes; memcpy(h+4, &v, 4); memcpy(h+8, "WAVEfmt ", 8); v = 16; memcpy(h+16, &v, 4);
    s = 1; memcpy(h+20, &s, 2); s = (uint16_t)ch; memcpy(h+22, &s, 2); memcpy(h+24, &rate, 4); v = rate*ch*2; memcpy(h+28, &v, 4);
    s = (uint16_t)(ch*2); memcpy(h+32, &s, 2); s = 16; memcpy(h+34, &s, 2); memcpy(h+36, "data", 4); memcpy(h+40, &nbytes, 4);
    fseek(f, 0, SEEK_SET); fwrite(h, 1, 44, f);
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "Aufruf: %s in.bin out.wav\n", argv[0]); return 1; }
    char hex[128] = ""; int rate = 24000, mode = 0, codec = 0, sbr = 0, nframes = 10;
    { char mn[512]; snprintf(mn, sizeof mn, "%s.meta", argv[1]); FILE *fm = fopen(mn, "r");
      if (!fm || fscanf(fm, "%127s %d %d %d %d %d", hex, &rate, &mode, &codec, &sbr, &nframes) < 6) { fprintf(stderr, "in.bin.meta fehlt/unvollstaendig\n"); return 1; } fclose(fm); }
    if (codec != 0) { fprintf(stderr, "Codec %d ist nicht AAC (xHE-AAC: xhe_dec benutzen)\n", codec); return 1; }
    /* DRM-Kanalmodus von FAAD2: mono 1, stereo 2, SBR mono 3, SBR-LC stereo 4, SBR stereo 5 (PS: mode == 1) */
    int drmch = (mode == 2) ? (sbr ? DRMCH_SBR_STEREO : DRMCH_STEREO) : (mode == 1) ? DRMCH_SBR_STEREO : (sbr ? DRMCH_SBR_MONO : DRMCH_MONO);
    NeAACDecHandle h = NeAACDecOpen();
    NeAACDecConfigurationPtr c = NeAACDecGetCurrentConfiguration(h); c->outputFormat = FAAD_FMT_16BIT; NeAACDecSetConfiguration(h, c);
    char r = NeAACDecInitDRM(&h, (unsigned long)rate, (unsigned char)drmch);
    if (r != 0) { fprintf(stderr, "NeAACDecInitDRM fehlgeschlagen (%d): FAAD2 ohne DRM-Unterstuetzung?\n", (int)r); return 2; }
    FILE *fi = fopen(argv[1], "rb"), *fo = fopen(argv[2], "wb"); if (!fi || !fo) { perror("open"); return 1; }
    wav_hdr(fo, (uint32_t)rate * (sbr || mode == 1 ? 2 : 1), 1, 0);
    static uint8_t buf[65536]; uint16_t L; uint32_t total = 0; int ok = 0, err = 0, ch = 1; uint32_t orate = (uint32_t)rate * (sbr || mode == 1 ? 2 : 1);
    static int16_t last[16384]; int lastn = 0;
    while (fread(&L, 2, 1, fi) == 1) {
        if (L == 0xFFFF) { for (int i = 0; i < nframes && lastn; i++) { memset(last, 0, (size_t)lastn*2); fwrite(last, 2, (size_t)lastn, fo); total += (uint32_t)lastn*2; } continue; }
        if (fread(buf, 1, L, fi) != L) break;
        NeAACDecFrameInfo fi_; void *pcm = NeAACDecDecode(h, &fi_, buf, L);
        if (fi_.error || !pcm || !fi_.samples) { err++; if (lastn) { memset(last, 0, (size_t)lastn*2); fwrite(last, 2, (size_t)lastn, fo); total += (uint32_t)lastn*2; } continue; }
        ch = fi_.channels; orate = (uint32_t)fi_.samplerate; lastn = (int)fi_.samples < 16384 ? (int)fi_.samples : 16384; memcpy(last, pcm, (size_t)lastn*2);
        fwrite(pcm, 2, fi_.samples, fo); total += (uint32_t)fi_.samples*2; ok++;
    }
    wav_hdr(fo, orate, ch, total); fclose(fo); fclose(fi); NeAACDecClose(h);
    fprintf(stderr, "AAC: %d Rahmen dekodiert, %d Fehler, %.1f s Audio (%u Hz, %d Kanal)\n", ok, err, (double)total/(2.0*ch*orate), orate, ch);
    return 0;
}
