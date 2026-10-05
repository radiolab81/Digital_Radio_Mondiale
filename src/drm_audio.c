/* ==========================================================================================================
 * drm_audio.c  --  Audio-Superframes
 * ==========================================================================================================
 * Audio wird im DRM-System in "Superframes" von 400 ms uebertragen [5.2]: mehrere Audiorahmen des Codecs werden
 * zu einem Block gebuendelt, der genau einem Stream-Anteil eines Multiplexrahmens entspricht.  Jeder Superframe hat
 * Kopfinformation, damit der Empfaenger die Rahmen wiederfindet, und Pruefsummen, damit er defekte erkennt.
 * ========================================================================================================== */
#include "drm_audio.h"
#include <string.h>

/* ---- Strom aus dem Multiplexrahmen [6.2.3] ----
 * Der Multiplexrahmen ist: [A-Teil Strom 0][A-Teil Strom 1]...[B-Teil Strom 0][B-Teil Strom 1]...  (jeweils die
 * "Laengen" aus der Multiplexbeschreibung des SDC, Typ 0).  Bei gleichem Schutz (EEP) ist der A-Teil leer. */
int drm_mux_stream(const uint8_t *mux, int mux_len, const drm_sdc_t *sd, int stream, uint8_t *out, int outmax)
{
    if (stream < 0 || stream >= sd->nstreams) return 0;
    int totA = 0, offA = 0, offB = 0;
    for (int i = 0; i < sd->nstreams; i++) totA += sd->len_a[i];
    for (int i = 0; i < stream; i++) { offA += sd->len_a[i]; offB += sd->len_b[i]; }
    const int la = sd->len_a[stream], lb = sd->len_b[stream];
    if (la + lb > outmax || totA + offB + lb > mux_len) return 0;
    memcpy(out, mux + offA, (size_t)la);                         /* hoeher geschuetzter Anteil */
    memcpy(out + la, mux + totA + offB, (size_t)lb);             /* niedriger geschuetzter Anteil */
    return la + lb;
}

/* CRC-16 der xHE-AAC-Rahmen [Annex D]: G16(x) = X^16 + X^12 + X^5 + 1, Init 0xFFFF, Ergebnis invertiert. */
static uint16_t crc16(const uint8_t *d, int n)
{
    uint16_t c = 0xffff;
    for (int i = 0; i < n; i++) for (int b = 7; b >= 0; b--) { uint16_t fb = ((c >> 15) & 1) ^ ((d[i] >> b) & 1); c <<= 1; if (fb) c ^= 0x1021; }
    return (uint16_t)~c;
}
void drm_asf_init(drm_asf_t *a) { memset(a, 0, sizeof *a); }

static void emit(drm_asf_t *a, const uint8_t *d, int len, drm_frame_cb cb, void *u)
{
    if (len >= 3 && crc16(d, len - 2) == (uint16_t)((d[len - 2] << 8) | d[len - 1])) { a->af_ok++; cb(u, d, len); }
    else { a->af_bad++; cb(u, NULL, 0); }
}

/* ---- xHE-AAC-Superframe [5.3.1.1-5.3.1.3] -----------------------------------------------------------------
 *   Byte 0 : obere 4 Bit = Zahl b der Rahmengrenzen im Verzeichnis, untere 4 Bit = Fuellstand des Bit-Reservoirs
 *   Byte 1 : CRC-8 ueber Byte 0 (G8 = X^8+X^4+X^3+X^2+1, Init 0xFF, invertiert)  [Annex D]
 *   Nutzteil: die Audiorahmen laufend aneinandergereiht; ein Rahmen kann ueber die Superframegrenze hinausreichen
 *   Ende   : b Grenzen zu je 2 Byte (12 Bit Position im Nutzteil | 4 Bit Zaehler); das LETZTE Verzeichniselement
 *            enthaelt die ERSTE Grenze.
 * Jeder Audiorahmen endet mit einer CRC-16 ueber seine Daten.  Rahmen, die in den naechsten Superframe laufen,
 * werden in a->openf zwischengespeichert ("offener Rahmen"). */
void drm_asf_parse(drm_asf_t *a, const uint8_t *s, int ASF, drm_frame_cb cb, void *u)
{
    uint8_t r = 0xff;                                            /* CRC-8 ueber Byte 0, bitweise */
    for (int b = 0; b < 8; b++) { uint8_t fb = ((r >> 7) & 1) ^ ((s[0] >> (7 - b)) & 1); r <<= 1; if (fb) r ^= 0x1d; }
    a->hdr_n++;
    int bad = ((uint8_t)~r != s[1]);
    int nbord = s[0] >> 4, plen = ASF - 2 - 2 * nbord;           /* Nutzteillaenge */
    if (bad || plen < 0) { a->have_open = 0; a->af_lost++; cb(u, NULL, -1); return; }
    a->hdr_ok++;
    const uint8_t *pay = s + 2; int bord[16], nb = 0;
    for (int i = 0; i < nbord; i++) {
        int off = ASF - 2 * (i + 1); int idx = (s[off] << 4) | (s[off + 1] >> 4);
        if (idx < plen) bord[nb++] = idx;                        /* Grenze im Nutzteil */
    }
    int start = nb ? bord[0] : plen;                             /* Bytes bis zur ersten Grenze gehoeren noch zum Vorgaengerrahmen */
    if (a->have_open && a->openlen + start < (int)sizeof a->openf) {
        memcpy(a->openf + a->openlen, pay, (size_t)start); a->openlen += start;
        emit(a, a->openf, a->openlen, cb, u);                    /* Rahmen aus dem Vorgaenger-Superframe komplettiert */
    }
    for (int i = 0; i < nb; i++) {
        int e = (i + 1 < nb) ? bord[i + 1] : plen, len = e - bord[i];
        if (i + 1 < nb) emit(a, pay + bord[i], len, cb, u);
        else if (len < (int)sizeof a->openf) { a->openlen = len; memcpy(a->openf, pay + bord[i], (size_t)len); a->have_open = 1; }
    }
    if (nb == 0 && a->have_open && a->openlen + plen < (int)sizeof a->openf) { memcpy(a->openf + a->openlen, pay, (size_t)plen); a->openlen += plen; }
}

int drm_aac_nframes(int rc) { return rc == 1 ? 5 : rc == 3 ? 10 : 0; }

/* ---- AAC-Superframe [5.4.1, Tab. 10/11] ----------------------------------------------------------------
 *   Kopf   : (n-1) Rahmengrenzen zu je 12 Bit (MSB zuerst); bei n = 10 folgen 4 Fuellbits zur Byte-Ausrichtung
 *   CRC    : n Bytes, je ein CRC-8 pro Rahmen ueber den empfindlichsten Teil des Rahmens (mono1+mono2 bzw.
 *            stereo1..5).  Diese CRC koennen erst nach teilweisem Parsen des AAC-Bitstroms geprueft werden und
 *            werden hier nur durchgereicht.
 *   Nutzteil: n Rahmen hintereinander.
 * Nur die 12 niederwertigen Bit der Grenzen werden gesendet; da die Grenzen steigen, wird jeder Rueckgang
 * als Ueberlauf um 4096 gedeutet (Hinweis 2 zu Tab. 11). */
int drm_aac_asf_parse(const uint8_t *s, int ASF, int n, drm_aac_cb cb, void *u)
{
    if (n != 5 && n != 10) { cb(u, NULL, -1, 0); return 0; }
    const int hbits = (n - 1) * 12 + (n == 10 ? 4 : 0), hbytes = hbits / 8, payload_off = hbytes + n, plen = ASF - payload_off;
    if (plen <= 0) { cb(u, NULL, -1, 0); return 0; }
    int bord[11], prev = 0; bord[0] = 0;
    for (int i = 0; i < n - 1; i++) {
        int bit = 12 * i, v = 0;
        for (int b = 0; b < 12; b++) { int p = bit + b; v = (v << 1) | ((s[p >> 3] >> (7 - (p & 7))) & 1); }
        while (v < prev) v += 4096;                              /* Ueberlauf der 12-Bit-Angabe */
        if (v > plen) { cb(u, NULL, -1, 0); return 0; }
        bord[i + 1] = v; prev = v;
    }
    bord[n] = plen;
    for (int f = 0; f < n; f++) if (bord[f + 1] < bord[f]) { cb(u, NULL, -1, 0); return 0; }
    for (int f = 0; f < n; f++) cb(u, s + payload_off + bord[f], bord[f + 1] - bord[f], s[hbytes + f]);
    return 1;
}
