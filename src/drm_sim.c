/* ==========================================================================================================
 * drm_sim.c  --  Selbsttest und Leistungsmessung der MSC-Dekodierung (nur PC, kein Teil des Empfaengers)
 * ==========================================================================================================
 * Aufbau: Zufallsbits -> Sender (drm_msc_encode) -> AWGN-Kanal (perfekt bekannt) -> Empfaenger (drm_msc_decode).
 * Das prueft, dass Kodierer, Punktierung, Verschachtelung, Abbildung und Dekoder zueinander passen -- auch fuer
 * Konfigurationen, zu denen keine Aufnahme vorliegt (64-QAM mit hohen Raten, UEP).  Es prueft NICHT, ob die
 * Tabellen mit der Norm uebereinstimmen; das geschieht ueber echte Aufnahmen (Bouquet-Datei: 14,6 kbit/s = PL1).
 *
 * SNR-Definition: Es/N0 je QAM-Zelle bei Einheitsleistung der Zelle (gleiche Definition wie die Diagnose von
 * drm_test).  Ausgegeben wird die kleinste SNR, bei der 30 von 30 Rahmen fehlerfrei dekodiert werden.
 * Aufruf: drm_sim [rahmen_je_snr] [rueckkopplungsdurchlaeufe]
 * ========================================================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "drm_msc.h"

static double gauss(void)                       /* normalverteilte Zufallszahl (Box-Muller) */
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

/* Fuehrt ntr Rahmen bei snr_db aus; Rueckgabe: Zahl fehlerhafter Rahmen.  errA/errB (optional): Bitfehler in Teil A / B. */
static int trial(const drm_msc_cfg_t *c, double snr_db, int ntr, long *errA, long *errB)
{
    static msc_cell_t cell[MSC_MAXN]; static uint8_t bits[MSC_MAXBITS], dec[MSC_MAXBITS]; int bad = 0;
    const double sg = sqrt(0.5 / pow(10.0, snr_db / 10.0));        /* Rauschstandardabweichung je Achse */
    for (int t = 0; t < ntr; t++) {
        for (int i = 0; i < c->lbits; i++) bits[i] = (uint8_t)(rand() & 1);
        if (drm_msc_encode(c, bits, cell) < 0) return -1;
        for (int n = 0; n < c->n; n++) { cell[n].zr += (float)(sg * gauss()); cell[n].zi += (float)(sg * gauss()); }
        drm_msc_decode(c, cell, dec);
        int frame_bad = 0;
        for (int i = 0; i < c->lbits; i++) if (bits[i] != dec[i]) { frame_bad = 1; if (i < c->L1) { if (errA) (*errA)++; } else if (errB) (*errB)++; }
        bad += frame_bad;
    }
    return bad;
}

int main(int argc, char **argv)
{
    const int ntr = argc > 1 ? atoi(argv[1]) : 30;
    srand(12345);
    if (argc > 2) drm_msc_iterations = atoi(argv[2]);
    static drm_mode_t m; drm_mode_setup(&m, DRM_MODE_B, 3);          /* Mode B, 10 kHz: N_MUX = 2337 */

    printf("MSC, Mode B/10 kHz (N_MUX %d), AWGN, %d Rahmen je SNR-Schritt, %d Rueckkopplungs-Durchlaeufe\n", m.nmux, ntr, drm_msc_iterations);
    printf("--- EEP (gleicher Schutz): kleinste SNR mit 30/30 fehlerfreien Rahmen ---\n");
    static const struct { int msc_mode, pl; } eep[] = { {3,0}, {3,1}, {0,0}, {0,1}, {0,2}, {0,3} };
    for (unsigned k = 0; k < sizeof eep / sizeof *eep; k++) {
        drm_msc_cfg_t c; if (drm_msc_config(&c, &m, eep[k].msc_mode, 0, eep[k].pl, eep[k].pl, 0) < 0) continue;
        double thr = -1; int bad_hi = trial(&c, 40.0, 5, NULL, NULL);
        for (double snr = 0; snr <= 34; snr += 1.0) if (trial(&c, snr, ntr, NULL, NULL) == 0) { thr = snr; break; }
        printf("  %2d-QAM PL%d  Raten", c.qam, eep[k].pl); for (int p = 0; p < c.levels; p++) printf(" %d/%d", c.rxA[p], c.ryA[p]);
        printf("  %5d Bit (%.1f kbit/s): ", c.lbits, c.lbits / 0.4 / 1000.0);
        if (bad_hi) printf("auch bei 40 dB fehlerhaft -> BUG\n"); else if (thr < 0) printf("> 34 dB\n"); else printf("fehlerfrei ab %.0f dB\n", thr);
    }

    printf("--- UEP (Teil A hoeher geschuetzt als Teil B) ---\n");
    static const struct { int msc_mode, plA, plB, bytesA; } uep[] = { {3,0,1,200}, {0,1,3,500}, {0,0,2,600} };
    for (unsigned k = 0; k < sizeof uep / sizeof *uep; k++) {
        drm_msc_cfg_t c; if (drm_msc_config(&c, &m, uep[k].msc_mode, 0, uep[k].plA, uep[k].plB, uep[k].bytesA) < 0) { printf("  UEP %u: unzulaessig\n", k); continue; }
        int bad_hi = trial(&c, 40.0, 5, NULL, NULL);
        printf("  %2d-QAM  PL_A=%d PL_B=%d  X=%d Byte -> N1=%d N2=%d Zellen, L1=%d L2=%d Bit: ", c.qam, uep[k].plA, uep[k].plB, uep[k].bytesA, c.n1, c.n2, c.L1, c.L2);
        if (bad_hi) { printf("fehlerhaft bei 40 dB -> BUG\n"); continue; }
        /* Mittlere SNR waehlen, bei der Teil B leidet: dort muss Teil A deutlich weniger Bitfehler haben */
        double snr_test = (c.qam == 16) ? 8.0 : 13.0; long ea = 0, eb = 0; trial(&c, snr_test, 20, &ea, &eb);
        printf("bei %.0f dB: Bitfehler Teil A %ld (von %d), Teil B %ld (von %d)\n", snr_test, ea, 20 * c.L1, eb, 20 * c.L2);
    }
    return 0;
}
