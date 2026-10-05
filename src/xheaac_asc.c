/* ==========================================================================================================
 * xheaac_asc.c  --  DRM xHE-AAC Static Config -> UsacConfig/AudioSpecificConfig (Hintergrund siehe xheaac_asc.h)
 * ==========================================================================================================
 * Bitleser/-schreiber (rd_t/wr_t) arbeiten MSB-zuerst wie alle MPEG-Syntaxen.  escapedValue(n1,n2,n3) ist die
 * MPEG-D-Schreibweise fuer ganze Zahlen mit Fortsetzungsfeldern: n1 Bit; sind alle 1, folgen n2 Bit (Summe), und
 * wieder alle 1 -> n3 Bit [ISO/IEC 23003-3, 5.2].  Die DRM-Syntax [ES 201 980, 5.3.2.1ff] entspricht der USAC-
 * Syntax ohne die in DRM festen Felder (tw_mdct = 0, usacConfigExtension leer, channelConfiguration aus dem SDC).
 * ========================================================================================================== */
#include "xheaac_asc.h"
#include <string.h>

typedef struct { const uint8_t *b; int n, pos; int err; } rd_t;
typedef struct { uint8_t *b; int max, pos; } wr_t;

static unsigned rbits(rd_t *r, int n)
{
    unsigned v = 0;
    while (n--) {
        if (r->pos >= r->n*8) { r->err = 1; v <<= 1; continue; }
        v = (v << 1) | ((r->b[r->pos >> 3] >> (7 - (r->pos & 7))) & 1); r->pos++;
    }
    return v;
}
static void wbits(wr_t *w, unsigned v, int n)
{
    for (int i = n-1; i >= 0; i--) {
        if (w->pos < w->max*8) {
            int bit = (v >> i) & 1, by = w->pos >> 3, sh = 7 - (w->pos & 7);
            if (!(w->pos & 7)) w->b[by] = 0;
            w->b[by] |= (uint8_t)(bit << sh);
        }
        w->pos++;
    }
}
/* escapedValue(n1,n2,n3) lesen bzw. schreiben (ISO/IEC 23003-3) */
static unsigned rescaled(rd_t *r, int n1, int n2, int n3)
{
    unsigned v = rbits(r, n1);
    if (v == (1u << n1) - 1) { unsigned a = rbits(r, n2); v += a;
        if (a == (1u << n2) - 1) v += rbits(r, n3); }
    return v;
}
static void wescaped(wr_t *w, unsigned v, int n1, int n2, int n3)
{
    unsigned m1 = (1u << n1) - 1, m2 = (1u << n2) - 1;
    if (v < m1) { wbits(w, v, n1); return; }
    wbits(w, m1, n1); v -= m1;
    if (v < m2) { wbits(w, v, n2); return; }
    wbits(w, m2, n2); v -= m2; wbits(w, v, n3);
}

/* SbrConfig() [ISO/IEC 23003-3, 5.2]: harmonicSBR, bs_interTes, bs_pvc, SbrDfltHeader() -- unveraendert in beiden Syntaxen,
 * wird deshalb Bit fuer Bit vom DRM-Strom uebernommen. */
static void copy_sbrconfig(rd_t *r, wr_t *w)
{
    wbits(w, rbits(r,1), 1); wbits(w, rbits(r,1), 1); wbits(w, rbits(r,1), 1);
    wbits(w, rbits(r,4), 4); wbits(w, rbits(r,4), 4);
    unsigned e1 = rbits(r,1), e2 = rbits(r,1); wbits(w, e1, 1); wbits(w, e2, 1);
    if (e1) { wbits(w, rbits(r,2), 2); wbits(w, rbits(r,1), 1); wbits(w, rbits(r,2), 2); }
    if (e2) { wbits(w, rbits(r,2), 2); wbits(w, rbits(r,2), 2); wbits(w, rbits(r,1), 1); wbits(w, rbits(r,1), 1); }
}

/* Hauptfunktion: liest die DRM-Konfiguration und schreibt den ASC (Reihenfolge: AOT, [Rate, Kanal], UsacConfig). */
int xheaac_build_asc(const uint8_t *cfg, int len, int mode, int rate, int variant, uint8_t *out, int outmax)
{
    rd_t r = { cfg, len, 0, 0 }; wr_t w = { out, outmax, 0 };
    unsigned cidx = rbits(&r, 2);                 /* coreSbrFrameLengthIndexDrm */
    unsigned csl = cidx + 1;                      /* coreSbrFrameLengthIndex nach ISO/IEC 23003-3 */
    int sbr_ratio = (csl >= 2);                   /* Indizes 2..4: SBR */
    int chcfg = (mode == 2) ? 2 : 1;
    wbits(&w, 31, 5); wbits(&w, 42 - 32, 6);      /* audioObjectType 42 (USAC) */
    if (variant == 0) {
        wbits(&w, 15, 4); wbits(&w, (unsigned)rate, 24); wbits(&w, (unsigned)chcfg, 4);
    }
    /* UsacConfig() */
    wbits(&w, 31, 5); wbits(&w, (unsigned)rate, 24);
    wbits(&w, csl, 3);
    wbits(&w, (unsigned)chcfg, 5);
    /* UsacDecoderConfig(): 1 Kanalelement + numExtElements Erweiterungen */
    /* DRM: xHEAACDecoderConfig() */
    unsigned nf = rbits(&r, 1);                   /* noiseFilling (SCE/CPE) */
    /* Kanalelement puffern, damit numElements vorab bekannt ist: erst DRM-Teil lesen */
    uint8_t tmp[24]; wr_t t = { tmp, 24, 0 };
    if (mode == 2) {
        unsigned sci = 0;
        if (sbr_ratio) { copy_sbrconfig(&r, &t); sci = rbits(&r, 2); wbits(&t, sci, 2); }
        if (sci > 0) {                                /* xHEAACMps212Config -> ISO Mps212Config (bsDecorrConfig = 0 ergaenzt) */
            wbits(&t, rbits(&r,3), 3);                /* bsFreqRes */
            wbits(&t, rbits(&r,3), 3);                /* bsFixedGainDMX */
            unsigned ts = rbits(&r,1); wbits(&t, ts ? 3 : 0, 2);   /* bsTempShapeConfig (DRM: 1 Bit -> 3) */
            wbits(&t, 0, 2);                          /* bsDecorrConfig */
            wbits(&t, rbits(&r,1), 1);                /* bsHighRateMode */
            wbits(&t, rbits(&r,1), 1);                /* bsPhaseCoding */
            unsigned op = rbits(&r,1); wbits(&t, op, 1);
            if (op) wbits(&t, rbits(&r,5), 5);        /* bsOttBandsPhase */
            if (sci >= 2) { wbits(&t, rbits(&r,5), 5); wbits(&t, rbits(&r,1), 1); }   /* bsResidualBands, bsPseudoLr */
        }
    } else if (sbr_ratio) copy_sbrconfig(&r, &t);
    unsigned next = rescaled(&r, 2, 4, 8);        /* numExtElements */
    if (r.err) return -1;
    wescaped(&w, next, 4, 8, 16);                 /* numElements-1 = numExtElements */
    /* Element 0 */
    wbits(&w, mode == 2 ? 1 : 0, 2);              /* usacElementType: SCE=0, CPE=1 */
    wbits(&w, 0, 1);                              /* tw_mdct */
    wbits(&w, nf, 1);                             /* noiseFilling */
    for (int i = 0; i < t.pos; i++) wbits(&w, (tmp[i>>3] >> (7-(i&7))) & 1, 1);
    /* Erweiterungselemente: UsacExtElementConfig() */
    for (unsigned i = 0; i < next; i++) {
        unsigned et  = rescaled(&r, 4, 8, 16);
        unsigned el  = rescaled(&r, 4, 8, 16);
        unsigned dlp = rbits(&r, 1); unsigned dl = 0; if (dlp) dl = rescaled(&r, 8, 16, 0);
        unsigned frag = rbits(&r, 1);
        if (el) return -1;                        /* Config-Payload der Ext-Elemente nicht unterstuetzt */
        wbits(&w, 3, 2);                          /* ID_USAC_EXT */
        wescaped(&w, et, 4, 8, 16); wescaped(&w, el, 4, 8, 16);
        wbits(&w, dlp, 1); if (dlp) wescaped(&w, dl, 8, 16, 0);
        wbits(&w, frag, 1);
    }
    unsigned cext = rbits(&r, 1);                 /* usacConfigExtensionPresent (DRM: UsacConfigExtensionDrm) */
    if (r.err) return -1;
    wbits(&w, 0, 1);                              /* keine Standard-Config-Extension (Loudness wird ignoriert) */
    (void)cext;
    return (w.pos + 7) / 8;
}
