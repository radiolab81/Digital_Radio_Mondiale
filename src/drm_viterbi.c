/* ==========================================================================================================
 * drm_viterbi.c  --  Faltungscode (Mutter-Code R = 1/6, K = 7) und Soft-Decision-Viterbi-Dekoder
 * ==========================================================================================================
 * Norm: ES 201 980 [7.3.2] "Component code", Bild 25 und Formeln fuer b_0,i ... b_5,i:
 *
 *      b0,i = a_i ^ a_{i-2} ^ a_{i-3} ^ a_{i-5} ^ a_{i-6}      (Generator 133 oktal)
 *      b1,i = a_i ^ a_{i-1} ^ a_{i-2} ^ a_{i-3} ^ a_{i-6}      (171)
 *      b2,i = a_i ^ a_{i-1} ^ a_{i-4} ^ a_{i-6}                (145)
 *      b3..b5 wiederholen b0..b2.
 *
 * Alle uebertragenen Raten entstehen durch Punktierung dieses einen Codes (Tab. 27, siehe drm_msc.c).
 * Kodierer und Decoder teilen sich die Tabelle outbits[]: Zustand+Eingang -> 6 Codebits.
 *
 * Viterbi-Prinzip (Lehrbuch): Der Kodierer ist ein Schieberegister mit 6 Speicherzellen = 64 Zustaende.  Zu jedem
 * Zeitpunkt wird fuer jeden Zustand der "Pfad" gespeichert, der die kleinste Distanz zu den empfangenen Werten
 * hat (Add-Compare-Select, ACS).  Am Ende wird vom bekannten Endzustand 0 rueckwaerts den gespeicherten
 * Entscheidungen gefolgt (Traceback).  Hier wird mit "Korrelation" statt Distanz gerechnet: je groesser die
 * Metrik, desto besser.  Bei 6 Codebits je Schritt entspricht die Zweigmetrik der Summe +-soft[j] (Vorzeichen
 * nach erwartetem Bit); punktierte Werte sind 0 und fallen heraus.
 * Rechenaufwand: 64 ACS je Schritt, ~ 12-14 kBit/s Nutzrate -> wenige Millionen ACS/s (MCU-tauglich).
 * ========================================================================================================== */
#include "drm_viterbi.h"
#include <string.h>

/* Abgriffe: Bit d des Index w = a_{i-d} (d = 0 aktuelles Bit, 1..6 Speicher).  Reihenfolge b0..b5. */
static const uint8_t taps[6] = {
    (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<6),     /* 133 oktal: b0 */
    (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<6),     /* 171 oktal: b1 */
    (1<<0)|(1<<1)|(1<<4)|(1<<6),            /* 145 oktal: b2 */
    (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<6),     /* b3 = b0 */
    (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<6),     /* b4 = b1 */
    (1<<0)|(1<<1)|(1<<4)|(1<<6)             /* b5 = b2 */
};

/* Paritaet (XOR aller Bits) eines Bytes: GCC/Clang-Builtin, sonst portable Schleife. */
#if defined(__GNUC__)
#define PARITY8(x) __builtin_parity((unsigned)(x))
#else
static int parity_fallback(unsigned x) { int p = 0; while (x) { p ^= (int)(x & 1); x >>= 1; } return p; }
#define PARITY8(x) parity_fallback((unsigned)(x))
#endif

static uint8_t outbits[128];                 /* w (7 Bit: Eingang + 6 Speicher) -> 6 Codebits */
static int tables_ok;

static void init_tables(void)
{
    for (int w = 0; w < 128; w++) {
        uint8_t o = 0;
        for (int j = 0; j < 6; j++) o |= (uint8_t)(PARITY8(w & taps[j]) << j);   /* XOR der Abgriffe */
        outbits[w] = o;
    }
    tables_ok = 1;
}

/* Vorab berechnete Zuordnung: fuer jeden Zielzustand ns die Ausgangsmuster der beiden Vorgaenger-Zweige. */
static uint8_t ob0[64], ob1[64];
static int acs_ok;
static void init_acs(void)
{
    if (!tables_ok) init_tables();
    for (int ns = 0; ns < 64; ns++) {
        int a = ns & 1, s0 = ns >> 1, s1 = (ns >> 1) | 32;
        ob0[ns] = outbits[a | (s0 << 1)];
        ob1[ns] = outbits[a | (s1 << 1)];
    }
    acs_ok = 1;
}

void drm_viterbi(const int8_t *soft, int n, uint64_t *dec, uint8_t *out)
{
    if (!acs_ok) init_acs();
    int32_t pm[64], nm[64];                  /* Pfadmetriken alt / neu (groesser = besser) */
    for (int s = 0; s < 64; s++) pm[s] = -1000000;
    pm[0] = 0;                               /* Start im Zustand 0 */

    for (int i = 0; i < n; i++) {
        const int8_t *sf = soft + 6 * i;
        /* Zweigmetriken fuer alle 64 Ausgangsmuster o (Bit j von o = erwartetes Codebit b_j):
         *   bm[o] = sum_j (o_j ? -sf[j] : +sf[j]).
         * Statt 64*6 Additionen: bm[0] = sum sf[j], dann je Bit j: bm[o | 1<<j] = bm[o] - 2*sf[j] (63 Additionen). */
        int32_t bm[64];
        bm[0] = sf[0] + sf[1] + sf[2] + sf[3] + sf[4] + sf[5];
        for (int j = 0; j < 6; j++) { const int32_t d2 = 2 * sf[j]; const int hi = 1 << j; for (int o = 0; o < hi; o++) bm[o | hi] = bm[o] - d2; }
        uint64_t d = 0;                      /* Entscheidungsbits dieses Schritts (ein Bit je Zielzustand) */
        int32_t best = -0x7fffffff;
        for (int ns = 0; ns < 64; ns++) {    /* Zielzustand ns hat zwei Vorgaenger: s0 = ns>>1 und s1 = s0 | 32 (Add-Compare-Select) */
            const int32_t m0 = pm[ns >> 1] + bm[ob0[ns]];
            const int32_t m1 = pm[(ns >> 1) | 32] + bm[ob1[ns]];
            const int sel = m1 > m0;
            nm[ns] = sel ? m1 : m0;
            d |= (uint64_t)sel << ns;
            if (nm[ns] > best) best = nm[ns];
        }
        for (int s = 0; s < 64; s++) pm[s] = nm[s] - best;   /* Normierung gegen Ueberlauf */
        dec[i] = d;
    }
    /* Traceback ab Endzustand 0 (Tailbits erzwingen ihn) */
    int st = 0;
    for (int i = n - 1; i >= 0; i--) {
        out[i] = (uint8_t)(st & 1);
        st = (st >> 1) | (int)(((dec[i] >> st) & 1) << 5);
    }
}

void drm_conv_encode(const uint8_t *a, int n, uint8_t *b)
{
    if (!tables_ok) init_tables();
    int st = 0;                              /* Zustand = die letzten 6 Eingangsbits */
    for (int i = 0; i < n + 6; i++) {
        int in = (i < n) ? (a[i] & 1) : 0;   /* nach n Bits werden 6 Nullen (Tailbits) angehaengt: Endzustand 0 */
        b[i] = outbits[in | (st << 1)];
        st = ((st << 1) | in) & 63;
    }
}
