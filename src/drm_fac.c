/* ==========================================================================================================
 * drm_fac.c  --  Fast Access Channel (FAC): Dekodierung und Auswertung
 * ==========================================================================================================
 * Der FAC ist der "Personalausweis" der Aussendung: er meldet Robustheitsmodus-Zusatzinfos, Spektrumbelegung,
 * Verschachtelungstiefe, QAM-Art von MSC und SDC sowie Grunddaten der Dienste.  Er wird in jedem 400-ms-Rahmen
 * mit 65 QAM-Zellen (4-QAM) gesendet [6.3, 7.2.1.2, 7.5.3, Tab. 62-65 fuer die Zellpositionen; [8.5.2.1]].
 *
 * Kette (Empfangsrichtung, jeweils Umkehrung des Senders):
 *   65 Zellen -> 130 Soft-Bits (Re, Im)                             [7.4, Bild 30: 4-QAM, Bitpaar (i0,q0)]
 *   -> Bit-Entschachtelung (Permutation mit t0 = 21, xin = 130)     [7.3.3.1]
 *   -> Entpunktieren Rate 0,6 = 3/5, Muster (11)(10)(11)            [7.5.3, Tab. 27]
 *   -> Viterbi, 72 Informationsbits (64 Daten + 8 CRC) + 6 Tail     [7.3.2]
 *   -> Entwuerfeln (Energy dispersal) mit PRBS X^9 + X^5 + 1        [7.2.2, Tab. 26]
 *   -> CRC-8 pruefen: Polynom G8(x) = X^8+X^4+X^3+X^2+1, Init 1..1, Ergebnis invertiert   [Annex D / 6.3.5]
 * ========================================================================================================== */
#include "drm_fac.h"
#include "drm_viterbi.h"
#include <string.h>

#define XIN 130                               /* Eingangsbits des Bit-Interleavers = 2 * 65 Zellen */
static int16_t perm[XIN];                     /* Interleaver-Permutation  */
static uint8_t prbs[128];                     /* Entwuerfelungsfolge */
static int tables_ok;

static void init_tables(void)
{
    /* Bit-Interleaver [7.3.3.1]: Pi(0) = 0, Pi(i) = (t0 * Pi(i-1) + s/4 - 1) mod s, Werte >= xin werden uebersprungen
     * (erneut iteriert).  Fuer den FAC: t0 = 21, s = 2^ceil(log2(xin)) = 256. */
    const int s = 256, q = s / 4 - 1, t0 = 21;
    perm[0] = 0;
    for (int i = 1; i < XIN; i++) {
        int v = (t0 * perm[i-1] + q) % s;
        while (v >= XIN) v = (t0 * v + q) % s;
        perm[i] = (int16_t)v;
    }
    /* PRBS [7.2.2]: 9-stufiges Schieberegister, Rueckkopplung Stufe 9 XOR Stufe 5, alle Stufen anfangs 1.
     * Kontrolle: die ersten Bits lauten 0000011110111110 (Tab. 26). */
    unsigned r = 0x1ff;
    for (int i = 0; i < 128; i++) {
        unsigned fb = ((r >> 8) ^ (r >> 4)) & 1;
        prbs[i] = (uint8_t)fb;
        r = ((r << 1) | fb) & 0x1ff;
    }
    tables_ok = 1;
}

/* CRC-8 ueber n Bits (Bitwise-Schieberegister, MSB zuerst), Init 0xFF, Polynom 0x1D, Ergebnis invertiert. */
static uint8_t crc8(const uint8_t *bits, int n)
{
    uint8_t r = 0xff;
    for (int i = 0; i < n; i++) {
        uint8_t fb = (uint8_t)(((r >> 7) & 1) ^ bits[i]);
        r <<= 1;
        if (fb) r ^= 0x1d;
    }
    return (uint8_t)~r;
}
static unsigned getb(const uint8_t *b, int pos, int n) { unsigned v = 0; while (n--) v = (v << 1) | b[pos++]; return v; }

int drm_fac_decode(const int8_t *y, drm_fac_t *f)
{
    if (!tables_ok) init_tables();
    int8_t v[XIN];
    for (int i = 0; i < XIN; i++) v[perm[i]] = y[i];       /* Senderseitig gilt y_i = v_perm(i): Entschachteln */

    /* Entpunktieren: Rate 3/5 bedeutet je 3 Schritte 5 uebertragene Bits aus dem Muster B0 = 111, B1 = 101.
     * Die nicht uebertragenen Positionen (und alle Bits b2..b5) bleiben 0 = "keine Information". */
    static const uint8_t pat[3][2] = {{1,1},{1,0},{1,1}};
    int8_t soft[6 * 78]; memset(soft, 0, sizeof soft);
    int pos = 0;
    for (int st = 0; st < 78; st++)                         /* 72 Info- + 6 Tailbit-Schritte */
        for (int j = 0; j < 2; j++)
            if (pat[st % 3][j]) soft[6 * st + j] = v[pos++];

    uint64_t dec[78]; uint8_t out[78];
    drm_viterbi(soft, 78, dec, out);

    memset(f, 0, sizeof *f);
    for (int i = 0; i < 72; i++) f->bits[i] = out[i] ^ prbs[i];            /* Entwuerfeln */
    f->crc_ok = (crc8(f->bits, 64) == getb(f->bits, 64, 8));

    /* Felder des FAC [6.3.3 Kanalparameter, 6.3.4 Dienstparameter]; Bit 0 = Base/Enhancement-Flag */
    const uint8_t *b = f->bits;
    f->identity = getb(b, 1, 2);       /* 0..3: Position des Rahmens im Superframe (0 und 3 koennen den Beginn markieren) */
    f->rm_flag = b[3];                 /* 0: Modi A-D, 1: Mode E */
    f->spectrum_occ = getb(b, 4, 3);   /* Spektrumbelegung 0..5 (4,5 / 5 / 9 / 10 / 18 / 20 kHz) */
    f->interleaver_depth = b[7];       /* 0: lang (2 s, D = 5), 1: kurz (400 ms, D = 1) */
    f->msc_mode = getb(b, 8, 2);       /* 0: 64-QAM SM, 1: HMsym, 2: HMmix, 3: 16-QAM SM */
    f->sdc_mode = b[10];               /* 0: 16-QAM, 1: 4-QAM */
    f->nservices = getb(b, 11, 4);
    f->reconf = getb(b, 15, 3);
    f->toggle = b[18];
    f->service_id = getb(b, 20, 24);
    f->short_id = getb(b, 44, 2);
    f->audio_ca = b[46];
    f->language = getb(b, 47, 4);
    f->audio_data = b[51];             /* 0: Audio, 1: Daten */
    f->descriptor = getb(b, 52, 5);
    return f->crc_ok;
}

/* ---- Sender-Seite (nur fuer Selbsttests) ---------------------------------------------------------------- */
void drm_fac_encode(const uint8_t *data64, float *re, float *im)
{
    if (!tables_ok) init_tables();
    uint8_t a[72], enc[78], y[XIN], v[XIN];
    memcpy(a, data64, 64);
    unsigned crc = crc8(a, 64); for (int i = 0; i < 8; i++) a[64 + i] = (uint8_t)((crc >> (7 - i)) & 1);      /* CRC-8 anhaengen */
    for (int i = 0; i < 72; i++) a[i] ^= prbs[i];                                                              /* Energieverwischung */
    drm_conv_encode(a, 72, enc);
    static const uint8_t pat[3][2] = {{1,1},{1,0},{1,1}};                                                      /* Punktierung Rate 3/5 */
    int pos = 0;
    for (int st = 0; st < 78; st++) for (int j = 0; j < 2; j++) if (pat[st % 3][j]) v[pos++] = (enc[st] >> j) & 1;
    for (int i = 0; i < XIN; i++) y[i] = v[perm[i]];                                                           /* Bit-Interleaver */
    const float a4 = 0.70710678f;                                                                              /* 4-QAM: Bit 0 -> +a, 1 -> -a */
    for (int n = 0; n < 65; n++) { re[n] = y[2 * n] ? -a4 : a4; im[n] = y[2 * n + 1] ? -a4 : a4; }
}
