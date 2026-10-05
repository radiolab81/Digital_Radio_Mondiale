/* ==========================================================================================================
 * xhe_dec.c  --  xHE-AAC-Audio-Rahmen (aus drm_test) mit FDK-AAC nach WAV dekodieren
 * ==========================================================================================================
 * Eingang: rahmen.bin (uint16 L + L Byte; L = 0xFFFF verlorener Superframe, L = 0 CRC-Fehler) plus rahmen.bin.meta
 * (Static Config, Ausgaberate, Mono/Stereo).  Jeder Rahmen = USAC-Access-Unit + CRC-16 (die 2 CRC-Byte werden hier
 * abgeschnitten).  Fehlende Rahmen ersetzt FDK durch Fehlerverdeckung (AACDEC_CONCEAL).
 * Ablauf: ASC aus der DRM-Konfiguration bauen (xheaac_asc.c) -> aacDecoder_ConfigRaw -> je Rahmen Fill + DecodeFrame
 * -> PCM an die WAV-Datei.  FDK-AAC muss mit USAC-Unterstuetzung uebersetzt sein (Version 2.0 oder neuer).
 * ========================================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fdk-aac/aacdecoder_lib.h>
#include "xheaac_asc.h"

static void wav_hdr(FILE *f, uint32_t rate, int ch, uint32_t nbytes)
{
    uint8_t h[44]; uint32_t v;
    memcpy(h, "RIFF", 4); v = 36 + nbytes; memcpy(h+4, &v, 4); memcpy(h+8, "WAVEfmt ", 8);
    v = 16; memcpy(h+16, &v, 4); uint16_t s = 1; memcpy(h+20, &s, 2); s = (uint16_t)ch; memcpy(h+22, &s, 2);
    memcpy(h+24, &rate, 4); v = rate*ch*2; memcpy(h+28, &v, 4); s = (uint16_t)(ch*2); memcpy(h+32, &s, 2);
    s = 16; memcpy(h+34, &s, 2); memcpy(h+36, "data", 4); memcpy(h+40, &nbytes, 4);
    fseek(f, 0, SEEK_SET); fwrite(h, 1, 44, f);
}

static HANDLE_AACDECODER open_dec(const uint8_t *cfg, int cfglen, int mode, int rate, int *variant_used)
{
    for (int variant = 0; variant < 2; variant++) {
        uint8_t asc[64]; int n = xheaac_build_asc(cfg, cfglen, mode, rate, variant, asc, sizeof asc);
        if (n < 0) continue;
        HANDLE_AACDECODER h = aacDecoder_Open(TT_MP4_RAW, 1);
        UCHAR *p[1] = { asc }; UINT sz[1] = { (UINT)n };
        AAC_DECODER_ERROR e = aacDecoder_ConfigRaw(h, p, sz);
        fprintf(stderr, "ASC-Variante %d (%d Byte): ConfigRaw -> 0x%04x %s\n", variant, n, (unsigned)e, e == AAC_DEC_OK ? "OK" : "Fehler");
        if (e == AAC_DEC_OK) { *variant_used = variant; return h; }
        aacDecoder_Close(h);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "Aufruf: %s in.bin out.wav [hexcfg] [rate] [mode]\n", argv[0]); return 1; }
    char hexbuf[128] = "e326ea0000"; const char *hex = hexbuf; int rate = 38400, mode = 0, coding = 3;
    { char mn[512]; snprintf(mn, sizeof mn, "%s.meta", argv[1]); FILE *fm = fopen(mn, "r");
      if (fm) { if (fscanf(fm, "%127s %d %d %d", hexbuf, &rate, &mode, &coding) >= 3 /* weitere Felder (sbr, nframes) werden ignoriert */) fprintf(stderr, "Meta: cfg=%s rate=%d mode=%d\n", hexbuf, rate, mode); fclose(fm); } }
    if (argc > 3) hex = argv[3];
    if (argc > 4) rate = atoi(argv[4]);
    if (argc > 5) mode = atoi(argv[5]);
    uint8_t cfg[32]; int cl = 0;
    for (const char *q = hex; q[0] && q[1] && cl < 32; q += 2) { unsigned v; sscanf(q, "%2x", &v); cfg[cl++] = (uint8_t)v; }
    int var = -1; HANDLE_AACDECODER h = open_dec(cfg, cl, mode, rate, &var);
    if (!h) { fprintf(stderr, "Kein ASC wurde akzeptiert.\n"); return 2; }

    FILE *fi = fopen(argv[1], "rb"), *fo = fopen(argv[2], "wb");
    if (!fi || !fo) { perror("open"); return 1; }
    wav_hdr(fo, (uint32_t)rate, 1, 0);
    static INT_PCM pcm[8*2048]; uint32_t total = 0; int ok = 0, bad = 0, lost = 0, concealed = 0, ch = 1; uint32_t orate = (uint32_t)rate;
    double owe = 0; uint16_t L; static uint8_t buf[65536];
    while (fread(&L, 2, 1, fi) == 1) {
        int nconceal = 0;
        if (L == 0xFFFF) { lost++; owe += 3.75; nconceal = (int)owe; owe -= nconceal; }
        else if (L == 0)  { bad++; nconceal = 1; }
        else {
            if (fread(buf, 1, L, fi) != L) break;
            UCHAR *p[1] = { buf }; UINT sz[1] = { (UINT)(L - 2) }, valid = (UINT)(L - 2);
            aacDecoder_Fill(h, p, sz, &valid);
            AAC_DECODER_ERROR e = aacDecoder_DecodeFrame(h, pcm, (INT)(sizeof pcm/sizeof pcm[0]), 0);
            if (e == AAC_DEC_OK) {
                CStreamInfo *si = aacDecoder_GetStreamInfo(h);
                ch = si->numChannels; orate = (uint32_t)si->sampleRate;
                fwrite(pcm, sizeof(INT_PCM), (size_t)(si->frameSize*ch), fo); total += (uint32_t)(si->frameSize*ch*2); ok++;
            } else { fprintf(stderr, "Decode-Fehler 0x%04x\n", (unsigned)e); nconceal = 1; }
        }
        for (int i = 0; i < nconceal; i++) {                    /* Fehlerverdeckung des Dekoders */
            AAC_DECODER_ERROR e = aacDecoder_DecodeFrame(h, pcm, (INT)(sizeof pcm/sizeof pcm[0]), AACDEC_CONCEAL);
            CStreamInfo *si = aacDecoder_GetStreamInfo(h);
            if ((e == AAC_DEC_OK || e == AAC_DEC_TRANSPORT_SYNC_ERROR) && si && si->frameSize > 0) {
                fwrite(pcm, sizeof(INT_PCM), (size_t)(si->frameSize*ch), fo); total += (uint32_t)(si->frameSize*ch*2); concealed++;
            }
        }
    }
    wav_hdr(fo, orate, ch, total); fclose(fo); fclose(fi); aacDecoder_Close(h);
    fprintf(stderr, "ASC-Variante %d: %d Frames dekodiert, %d CRC-Fehler, %d Superframes verloren, %d Frames verdeckt, %.1f s Audio (%u Hz, %d Kanal)\n",
            var, ok, bad, lost, concealed, (double)total/(2.0*ch*orate), orate, ch);
    return 0;
}
