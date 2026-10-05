/* ==========================================================================================================
 * drm_rx.c  --  Streaming-Empfaenger (Zustandsautomat ueber alle Verarbeitungsstufen)
 * ==========================================================================================================
 * Signalkette (Details in README.md):
 *
 *   I/Q 12 kHz -> [ACQ] Guard-Korrelation: Modus, Symboltakt, Feinfrequenz            (drm_ofdm.c)
 *              -> [SYNC] Rahmenphase und ganzzahliger Traegerversatz ueber Zeitreferenzen   (drm_chan.c)
 *              -> [TRACK] je Symbol: Bruchteilfenster+Mischer+FFT, Fensterlage entdrehen, Pilotstatistik,
 *                         PI-Regler Zeit (Abtasttakt), P-Regler Frequenz
 *                         je Rahmen (400 ms): Kanalschaetzung -> FAC dekodieren (drm_fac.c)
 *                         je Superframe (1,2 s): SDC (drm_sdc.c); MSC-Zellen sammeln, ueber D Rahmen entschachteln,
 *                         Mehrebenen-Viterbi (drm_msc.c), Strom herausloesen, Audio-Superframe zerlegen (drm_audio.c)
 *
 * Zeitliche Gliederung [ES 201 980, 8.1]:  Symbol (Ts) < Uebertragungsrahmen (400 ms, Ns Symbole) < Superframe
 * (3 Rahmen = 1,2 s).  Ein Rahmen wird erst verarbeitet, wenn DRM_TW/2 = 2 weitere Symbole eingetroffen sind,
 * weil die Kanalschaetzung auch spaetere Piloten benutzt.
 * ========================================================================================================== */
#include "drm_rx.h"
#include "drm_ofdm.h"
#include "drm_chan.h"
#include "drm_msc.h"
#include "drm_audio.h"
#include <math.h>
#include <string.h>

#define BUFN        4096                 /* Eingangs-Ringpuffer in Abtastwerten (Zweierpotenz) */
#define RS          32                   /* Zellen-Ring: Symbole (>= Ns + 2*(DRM_TW/2), Mode D: 24+4) */
#define H           (DRM_TW / 2)
#define MAXD        5                    /* maximale Zeitverschachtelungstiefe */
#define ACQ_BLOCKS  60                   /* Laenge der Erstsynchronisation in Symbollaengen */
#define MAXSHIFT    12                   /* ganzzahliger Traegerversatz, der in SYNC gesucht wird */
#define MAXFRAMES_SYNC 40
#define FAC_LOSS    15                   /* so viele aufeinanderfolgende FAC-Fehler -> neu einrasten */
#define KP_T 0.01f                       /* Zeitregler (Symbolfenster, Abtasttakt): P- und I-Anteil */
#define KI_T 0.0002f
#define KP_F 0.01f                       /* Frequenzregler */
#define SNR_SMOOTH 15.0f                 /* unterhalb dieses Pilot-SNR wird die Kanalschaetzung zusaetzlich geglaettet */

typedef struct { int Tu, Tg, Ts; int64_t fr[352], fi[352], fe[352]; int32_t dr[88], di[88], de[88]; int64_t sr, si, se; } acq_t;

struct drm_rx {
    /* --- bleibt bei einer Neu-Akquisition erhalten --- */
    drm_event_cb cb; void *user;
    drm_cs16_t buf[BUFN]; int64_t nin;                      /* Eingangsring, Zahl der bisher gelesenen Werte */
    drm_stats_t st;                                         /* Zaehler (auch ueber Neu-Akquisitionen) */
    int occ, occ_known, mode, have_mode;                    /* einmal erkannte Belegung / Modus */
    int state;
    /* --- ab hier wird bei go_acq() alles auf 0 gesetzt --- */
    acq_t acq[4]; int64_t acq_n;                            /* ACQ: Korrelationsakkus je Modus */
    drm_mode_t M; drm_ofdm_t o;                             /* Modus und Demodulator */
    int64_t p0, l;                                          /* Position des Symbols 0; laufender Symbolindex */
    float tcorr, trate, fcorr; float pfe[3][6]; int nfe;    /* Regler, Pilothistorie */
    float ps_acc, pn_acc, psnr;                             /* Pilot-SNR-Schaetzer */
    double sacc[2 * MAXSHIFT + 1][DRM_MAXSYM]; int sync_syms; int fphase;   /* SYNC */
    drm_cell_t cell[DRM_MAXCAR];
    float XR[RS][DRM_MAXCAR], XI[RS][DRM_MAXCAR]; int64_t l_track0;          /* TRACK: Zellenring */
    int64_t fr; int phvote[3]; int fac_bad;
    drm_fac_t fac; int have_fac; drm_sdc_t sdc; int have_sdc;
    int cnt; int64_t mux_count; msc_cell_t cur[MSC_MAXN], mf[MAXD][MSC_MAXN], dq[MSC_MAXN];     /* MSC */
    drm_asf_t asf; uint8_t bits[MSC_MAXBITS], mux[MSC_MAXBITS / 8], stream[4096];
};

size_t drm_rx_size(void) { return sizeof(struct drm_rx); }
static void emit(drm_rx_t *rx, int type, const void *d, int len) { if (rx->cb) rx->cb(rx->user, type, d, len); }
static void set_state(drm_rx_t *rx, int s) { rx->state = s; emit(rx, DRM_EV_STATE, NULL, s); }
void drm_rx_set_callback(drm_rx_t *rx, drm_event_cb cb, void *user) { rx->cb = cb; rx->user = user; }
int  drm_rx_state(const drm_rx_t *rx) { return rx->state; }
const drm_sdc_t *drm_rx_sdc(const drm_rx_t *rx) { return rx->have_sdc ? &rx->sdc : NULL; }
const drm_fac_t *drm_rx_fac(const drm_rx_t *rx) { return rx->have_fac ? &rx->fac : NULL; }
void drm_rx_stats(const drm_rx_t *rx, drm_stats_t *s)
{
    *s = rx->st; s->mode = rx->mode; s->occ = rx->occ; s->ncar = rx->have_mode ? rx->M.ncar : 0;
    s->foff_hz = rx->fcorr; s->pilot_snr_db = rx->psnr; s->hdr_ok = rx->asf.hdr_ok; s->hdr_n = rx->asf.hdr_n;
}

/* ---- Rueckfall nach ACQ (Zaehler bleiben erhalten) ---- */
static void acq_restart(drm_rx_t *rx)
{
    static const int TU[4] = {288, 256, 176, 112}, TG[4] = {32, 64, 64, 88};
    for (int m = 0; m < 4; m++) { memset(&rx->acq[m], 0, sizeof rx->acq[m]); rx->acq[m].Tu = TU[m]; rx->acq[m].Tg = TG[m]; rx->acq[m].Ts = TU[m] + TG[m]; }
    rx->acq_n = 0;
}
/* Zurueck zur Suche: Eingangsring, Zaehler und erkannte Belegung bleiben, alle Verfolgungszustaende werden geloescht. */
static void go_acq(drm_rx_t *rx, int count_reacq)
{
    const int old_state = rx->state;
    memset(&rx->acq, 0, sizeof *rx - offsetof(struct drm_rx, acq));
    if (count_reacq) rx->st.reacq++;
    drm_asf_init(&rx->asf);
    acq_restart(rx);
    rx->state = DRM_ST_ACQ;
    if (old_state != DRM_ST_ACQ) emit(rx, DRM_EV_STATE, NULL, DRM_ST_ACQ);
}

drm_rx_t *drm_rx_init(void *mem, size_t sz)
{
    if (sz < sizeof(struct drm_rx)) return NULL;
    drm_rx_t *rx = (drm_rx_t *)mem; memset(rx, 0, sizeof *rx); acq_restart(rx); drm_asf_init(&rx->asf); rx->state = DRM_ST_ACQ;
    return rx;
}

/* ============================== ACQ ============================== */
/* Mitlaufende Guard-Korrelation fuer alle vier Modi gleichzeitig (siehe drm_ofdm.c):
 *   d(j) = x[j] * conj(x[j+Tu]);  W(m) = sum_{i<Tg} d(m+i);  fold[m mod Ts] += W(m)
 * W wird als gleitende Summe in O(1) je Abtastwert fortgeschrieben (neuen Wert dazu, den vor Tg Werten abziehen). */
static void acq_sample(drm_rx_t *rx)
{
    const int64_t n = rx->nin - 1; const drm_cs16_t b = rx->buf[n & (BUFN - 1)];
    for (int m = 0; m < 4; m++) {
        acq_t *a = &rx->acq[m]; if (n < a->Tu) continue;
        const drm_cs16_t x = rx->buf[(n - a->Tu) & (BUFN - 1)];
        int32_t ar = x.re >> 3, ai = x.im >> 3, br = b.re >> 3, bi = b.im >> 3;
        int32_t dr = ar * br + ai * bi, di = ai * br - ar * bi, de = ar * ar + ai * ai + br * br + bi * bi;
        const int64_t j = n - a->Tu; const int slot = (int)(j % a->Tg);
        a->sr += dr - a->dr[slot]; a->si += di - a->di[slot]; a->se += de - a->de[slot];
        a->dr[slot] = dr; a->di[slot] = di; a->de[slot] = de;
        if (j >= a->Tg - 1) { int t = (int)((j - (a->Tg - 1)) % a->Ts); a->fr[t] += a->sr; a->fi[t] += a->si; a->fe[t] += a->se; }
    }
    rx->acq_n++;
}
static int acq_decide(drm_rx_t *rx)
{
    int bestm = -1, bestt = 0; double bests = 0; float foff = 0;
    for (int m = 0; m < 4; m++) {
        acq_t *a = &rx->acq[m]; int bt = 0; double bm = -1;
        for (int t = 0; t < a->Ts; t++) { double mg = sqrt((double)a->fr[t] * a->fr[t] + (double)a->fi[t] * a->fi[t]); if (mg > bm) { bm = mg; bt = t; } }
        double met = bm / (0.5 * (double)a->fe[bt] + 1.0);
        if (met > bests) { bests = met; bestm = m; bestt = bt; foff = (float)(-atan2((double)a->fi[bt], (double)a->fr[bt]) * DRM_FS / (2.0 * M_PI * a->Tu)); }
    }
    if (bests < 0.15) { acq_restart(rx); return 0; }               /* kein DRM-Signal: weiter suchen */
    rx->mode = bestm; rx->have_mode = 1;
    if (!rx->occ_known) rx->occ = 3;                                 /* erst die breiteste Belegung annehmen, FAC verraet die echte */
    drm_mode_setup(&rx->M, rx->mode, rx->occ);
    drm_ofdm_init(&rx->o, &rx->M); rx->fcorr = foff; drm_ofdm_set_freq(&rx->o, foff);
    /* erster Symbolbeginn: Raster t0 + k*Ts, mit genug Vorlauf im Ring */
    const int Ts = rx->M.Ts; int64_t k = (rx->nin - 1800 - bestt + Ts - 1) / Ts; if (k < 1) k = 1;
    rx->p0 = bestt + k * Ts; rx->l = 0; rx->sync_syms = 0; memset(rx->sacc, 0, sizeof rx->sacc);
    set_state(rx, DRM_ST_SYNC);
    return 1;
}

/* ============================== Symbolverarbeitung ============================== */
static int symbol_ready(const drm_rx_t *rx, int di)
{
    int64_t pos = rx->p0 + rx->l * rx->M.Ts + di;
    return rx->nin >= pos + rx->M.Ts + DRM_OFDM_POST + 1;
}
/* FIR-Zugriff auf einen zusammenhaengenden Block aus dem Ring */
static void fetch(const drm_rx_t *rx, int64_t start, int n, drm_cs16_t *out) { for (int i = 0; i < n; i++) out[i] = rx->buf[(start + i) & (BUFN - 1)]; }

/* ============================== SYNC ============================== */
static void sync_step(drm_rx_t *rx)
{
    const drm_mode_t *m = &rx->M; const int NS = m->nsym;
    drm_cs16_t x[DRM_MAXTU + 200];
    int64_t pos = rx->p0 + rx->l * m->Ts;
    if (rx->nin - (pos - DRM_OFDM_PRE) > BUFN - 8) { go_acq(rx, 1); return; }          /* zu spaet dran */
    fetch(rx, pos - DRM_OFDM_PRE, m->Ts + DRM_OFDM_PRE + DRM_OFDM_POST + 1, x);
    drm_ofdm_symbol(&rx->o, x + DRM_OFDM_PRE, 0.0f, rx->cell);
    for (int d = -MAXSHIFT; d <= MAXSHIFT; d++) rx->sacc[d + MAXSHIFT][rx->l % NS] += drm_time_ref_metric(m, rx->cell, rx->o.fft_adv, d);
    rx->l++; rx->sync_syms++;
    if (rx->sync_syms % NS != 0 || rx->sync_syms < 3 * NS) return;                      /* Entscheidung alle Rahmen, fruehestens nach 3 */
    int bd = 0, bp = 0; double bv = -1, second = 0;
    for (int d = 0; d <= 2 * MAXSHIFT; d++) for (int p = 0; p < NS; p++) if (rx->sacc[d][p] > bv) { bv = rx->sacc[d][p]; bd = d; bp = p; }
    for (int d = 0; d <= 2 * MAXSHIFT; d++) for (int p = 0; p < NS; p++) if (!(d == bd && p == bp) && rx->sacc[d][p] > second) second = rx->sacc[d][p];
    double mean = bv / (rx->sync_syms / NS);                                            /* mittlere Kohaerenz je Rahmen */
    if (mean < 0.35 || bv < 1.6 * second) {                                             /* noch nicht eindeutig */
        if (rx->sync_syms >= MAXFRAMES_SYNC * NS) go_acq(rx, 1);
        return;
    }
    int shift = bd - MAXSHIFT;                       /* Signal erscheint bei k + shift: um shift Traegerabstaende herunter mischen */
    rx->fcorr += shift * m->spacing_hz; drm_ofdm_set_freq(&rx->o, rx->fcorr);
    rx->fphase = bp;                                 /* Symbole l mit l % NS == bp sind Rahmenanfaenge */
    rx->o.fft_adv = (m->Tg > 24) ? 12 : m->Tg / 3;   /* im Betrieb: Fenster etwas weiter in den Guard hinein */
    rx->tcorr = 0; rx->trate = 0; rx->nfe = 0; rx->ps_acc = rx->pn_acc = 0;
    rx->l_track0 = rx->l; rx->fr = 0; memset(rx->phvote, 0, sizeof rx->phvote); rx->fac_bad = 0; rx->cnt = 0; rx->mux_count = 0;
    set_state(rx, DRM_ST_TRACK);
}

/* ============================== TRACK ============================== */
/* Zellen eines Symbols entzerren: z = X/H (Einheitsleistung), w = |H|^2; arbeitet auf dem Zellenring. */
static void equalize(drm_rx_t *rx, int64_t l, int smooth, float *zr, float *zi, float *w)
{
    static float hr[DRM_MAXCAR], hi[DRM_MAXCAR]; float *wr[DRM_TW], *wi[DRM_TW]; int sph[DRM_TW];
    const drm_mode_t *m = &rx->M; const int NS = m->nsym, NC = m->ncar;
    for (int k = 0; k < DRM_TW; k++) {
        int64_t ll = l - H + k; int slot = (int)(((ll % RS) + RS) % RS);
        wr[k] = rx->XR[slot]; wi[k] = rx->XI[slot];
        sph[k] = (int)((((ll - rx->fphase) % NS) + NS) % NS);          /* Symbol im Rahmen */
    }
    drm_chan_est_tf(m, wr, wi, sph, smooth, hr, hi);
    const float *xr = rx->XR[(int)(l % RS)], *xi = rx->XI[(int)(l % RS)];
    for (int i = 0; i < NC; i++) {
        float h2 = hr[i] * hr[i] + hi[i] * hi[i] + 1e-12f;
        zr[i] = (xr[i] * hr[i] + xi[i] * hi[i]) / h2; zi[i] = (xi[i] * hr[i] - xr[i] * hi[i]) / h2; w[i] = h2;
    }
}
static int8_t q8(float v, float sc) { v *= sc; if (v > 127) v = 127; if (v < -127) v = -127; return (int8_t)lrintf(v); }

static void audio_frame_cb(void *u, const uint8_t *f, int len)
{
    drm_rx_t *rx = (drm_rx_t *)u;
    if (len > 0) rx->st.frames_ok++; else if (len == 0) rx->st.frames_bad++; else rx->st.frames_lost++;
    emit(rx, DRM_EV_AUDIO, f, len);
}
static void aac_cb(void *u, const uint8_t *f, int len, int crc8)
{
    (void)crc8; drm_rx_t *rx = (drm_rx_t *)u;
    if (len > 0) rx->st.frames_ok++; else rx->st.frames_lost++;
    emit(rx, DRM_EV_AUDIO, f, len);
}

/* Einen vollstaendig entschachtelten Multiplexrahmen auswerten (Audio-Strom herausholen und zerlegen). */
static void mux_frame_done(drm_rx_t *rx)
{
    const msc_cell_t *w[MAXD]; const int D = rx->have_fac && rx->fac.interleaver_depth ? 1 : 5;
    if (!rx->have_sdc || !rx->sdc.have_audio || !rx->have_fac) return;
    const drm_sdc_t *sd = &rx->sdc; int bytesA = 0; for (int i = 0; i < sd->nstreams; i++) bytesA += sd->len_a[i];
    drm_msc_cfg_t cfg;
    if (drm_msc_config(&cfg, &rx->M, rx->fac.msc_mode, rx->fac.interleaver_depth, sd->prot_a, sd->prot_b, bytesA) < 0) return;
    if (rx->mux_count < D) return;                                                 /* noch nicht genug Rahmen fuer die Entschachtelung */
    for (int i = 0; i < D; i++) w[i] = rx->mf[(rx->mux_count - D + i) % MAXD];     /* aeltester zuerst */
    drm_msc_deinterleave(&cfg, w, rx->dq);
    drm_msc_decode(&cfg, rx->dq, rx->bits);
    for (int i = 0; i < cfg.lbits / 8; i++) { unsigned v = 0; for (int b = 0; b < 8; b++) v = (v << 1) | rx->bits[8 * i + b]; rx->mux[i] = (uint8_t)v; }
    int len = drm_mux_stream(rx->mux, cfg.lbits / 8, sd, sd->a_stream, rx->stream, sizeof rx->stream);
    if (len <= 0) return;
    if (sd->a_text) len -= 4;                                                      /* letzte 4 Byte: Textmeldung [6.5] */
    if (sd->a_coding == 3) drm_asf_parse(&rx->asf, rx->stream, len, audio_frame_cb, rx);
    else if (sd->a_coding == 0) {
        int nf = drm_aac_nframes(sd->a_rate); rx->asf.hdr_n++;
        if (drm_aac_asf_parse(rx->stream, len, nf, aac_cb, rx)) rx->asf.hdr_ok++; else { rx->st.frames_lost++; emit(rx, DRM_EV_AUDIO, NULL, -1); }
    }
}

/* Alle Verarbeitung, die einen vollstaendigen Rahmen (und 2 Symbole Nachlauf) braucht. */
static void process_frame(drm_rx_t *rx, int64_t lf)
{
    const drm_mode_t *m = &rx->M; const int NS = m->nsym, NC = m->ncar;
    static float zr[DRM_MAXCAR], zi[DRM_MAXCAR], w[DRM_MAXCAR];
    const int smooth = 1;            /* FAC/SDC: Glaettung der Pilotschaetzung (kurze Bloecke, empfindlich gegen Schaetzrauschen) */
    /* ---- FAC: 65 Zellen -> 130 Soft-Werte [8.5.2, 7.5.3] ---- */
    float sr[130]; int ns = 0;
    for (int s = 0; s < NS; s++) {
        int need = 0; for (int i = 0; i < NC; i++) if (m->map[s][i].type == CT_FAC) { need = 1; break; }
        if (!need) continue;
        equalize(rx, lf + s, smooth, zr, zi, w);
        for (int i = 0; i < NC; i++) if (m->map[s][i].type == CT_FAC && ns < 130) { sr[ns++] = zr[i] * w[i]; sr[ns++] = zi[i] * w[i]; }
    }
    float sc = 0; for (int i = 0; i < ns; i++) sc += fabsf(sr[i]); sc = 40.0f / (sc / (ns ? ns : 1) + 1e-9f);
    int8_t soft[130]; for (int i = 0; i < 130; i++) soft[i] = q8(i < ns ? sr[i] : 0, sc);
    drm_fac_t f; drm_fac_decode(soft, &f); rx->st.fac_n++;
    if (f.crc_ok) {
        rx->st.fac_ok++; rx->fac_bad = 0; rx->fac = f; rx->have_fac = 1; emit(rx, DRM_EV_FAC, &f, (int)sizeof f);
        if (!rx->occ_known && f.spectrum_occ != rx->occ) {              /* wahre Belegung weicht von der Annahme ab: neu einrasten */
            rx->occ = f.spectrum_occ; rx->occ_known = 1; go_acq(rx, 0); return;
        }
        rx->occ_known = 1; rx->occ = f.spectrum_occ;
        rx->phvote[(int)((((rx->fr - (f.identity == 3 ? 0 : f.identity)) % 3) + 3) % 3)]++;
    } else if (++rx->fac_bad >= FAC_LOSS) { go_acq(rx, 1); return; }
    int best = 0; for (int i = 1; i < 3; i++) if (rx->phvote[i] > rx->phvote[best]) best = i;
    if (rx->phvote[0] + rx->phvote[1] + rx->phvote[2] < 2) { rx->fr++; return; }   /* Superframe-Phase noch unbekannt */
    const int first = ((((rx->fr - best) % 3) + 3) % 3) == 0;                  /* erster Rahmen eines Superframes? */

    /* ---- SDC: erste 2 (A/B) bzw. 3 (C/D) Symbole des ersten Rahmens [8.5.3] ---- */
    if (first) {
        static msc_cell_t sc_[MSC_MAXN]; int nn = 0;
        for (int s = 0; s < m->nsdc_sym; s++) {
            equalize(rx, lf + s, smooth, zr, zi, w);
            for (int i = 0; i < NC; i++) if (m->map[s][i].type == CT_DATA && nn < m->nsdc) { sc_[nn].zr = zr[i]; sc_[nn].zi = zi[i]; sc_[nn].w = w[i]; nn++; }
        }
        if (nn == m->nsdc) {
            drm_sdc_t sd; rx->st.sdc_n++;
            if (drm_sdc_decode(m, rx->fac.sdc_mode, sc_, &sd) == 0 && sd.crc_ok) {
                rx->st.sdc_ok++; drm_sdc_merge(&rx->sdc, &sd); rx->have_sdc = 1; emit(rx, DRM_EV_SDC, &rx->sdc, (int)sizeof rx->sdc);
            }
        }
        rx->cnt = 0;                                                            /* MSC-Zaehler je Superframe neu */
    }
    /* ---- MSC-Zellen dieses Rahmens sammeln [8.6, 7.7]: alle Datenzellen ausser SDC ---- */
    for (int s = 0; s < NS; s++) {
        if (first && s < m->nsdc_sym) continue;
        equalize(rx, lf + s, 0, zr, zi, w);
        for (int i = 0; i < NC; i++) {
            if (m->map[s][i].type != CT_DATA) continue;
            int c = rx->cnt++;
            if (c >= 3 * m->nmux) continue;                                     /* die letzten 0..2 Zellen sind ungenutzt */
            msc_cell_t *cell = &rx->cur[c % m->nmux]; cell->zr = zr[i]; cell->zi = zi[i]; cell->w = w[i];
            if ((c + 1) % m->nmux == 0) {                                       /* Multiplexrahmen komplett */
                memcpy(rx->mf[rx->mux_count % MAXD], rx->cur, (size_t)m->nmux * sizeof(msc_cell_t));
                rx->mux_count++; mux_frame_done(rx);
            }
        }
    }
    rx->fr++;
}

static void track_step(drm_rx_t *rx)
{
    const drm_mode_t *m = &rx->M; const int NS = m->nsym, NC = m->ncar;
    int di = (int)floorf(rx->tcorr); float fr = rx->tcorr - di;
    drm_cs16_t x[DRM_MAXTU + 200];
    int64_t pos = rx->p0 + rx->l * m->Ts + di;
    if (rx->nin - (pos - DRM_OFDM_PRE) > BUFN - 8) { go_acq(rx, 1); return; }
    fetch(rx, pos - DRM_OFDM_PRE, m->Ts + DRM_OFDM_PRE + DRM_OFDM_POST + 1, x);
    drm_ofdm_symbol(&rx->o, x + DRM_OFDM_PRE, fr, rx->cell);
    const int slot = (int)(rx->l % RS); const int s = (int)((((rx->l - rx->fphase) % NS) + NS) % NS);
    drm_rotate_cells(m, rx->cell, rx->o.fft_adv, rx->XR[slot], rx->XI[slot]);
    /* Pilotstatistik -> Zeit- und Frequenzregelung */
    float sl, fe[6]; drm_pilot_stats(m, rx->XR[slot], rx->XI[slot], s, &sl, fe);
    float terr = -(sl * m->Tu / (2.0f * (float)M_PI));                           /* Zeitfehler in Abtastwerten */
    rx->trate += KI_T * terr; rx->tcorr += KP_T * terr + rx->trate;               /* PI-Regler: trate = Abtasttaktfehler je Symbol */
    if (rx->nfe > 0) {
        const float *pf = rx->pfe[(rx->nfe - 1) % 3]; float pr = 0, pi = 0;
        for (int q = 0; q < 3; q++) { pr += fe[2 * q] * pf[2 * q] + fe[2 * q + 1] * pf[2 * q + 1]; pi += fe[2 * q + 1] * pf[2 * q] - fe[2 * q] * pf[2 * q + 1]; }
        rx->fcorr += KP_F * atan2f(pi, pr) * DRM_FS / (2.0f * (float)M_PI * m->Ts); drm_ofdm_set_freq(&rx->o, rx->fcorr);
    }
    memcpy(rx->pfe[rx->nfe % 3], fe, sizeof fe); rx->nfe++;
    if (rx->nfe >= 3) {                              /* Pilot-SNR: Rauschen aus der 2. Differenz der konstanten Frequenzpiloten */
        const float *a0 = rx->pfe[(rx->nfe - 3) % 3], *a1 = rx->pfe[(rx->nfe - 2) % 3], *a2 = rx->pfe[(rx->nfe - 1) % 3];
        float pn = 0, ps = 0;
        for (int q = 0; q < 3; q++) { float dr = a2[2*q] - 2*a1[2*q] + a0[2*q], dim = a2[2*q+1] - 2*a1[2*q+1] + a0[2*q+1]; pn += (dr*dr + dim*dim) / 6.0f; ps += a1[2*q]*a1[2*q] + a1[2*q+1]*a1[2*q+1]; }
        rx->ps_acc = 0.995f * rx->ps_acc + ps; rx->pn_acc = 0.995f * rx->pn_acc + pn;
        if (rx->pn_acc > 0) rx->psnr = 10.0f * log10f(rx->ps_acc / rx->pn_acc) - 3.0f;     /* -3 dB: Pilotueberhoehung gegen Datenzellen */
    }
    (void)NC;
    /* Rahmen-Verarbeitung: sobald Rahmen lf komplett und H Symbole Nachlauf vorliegen */
    int64_t lf = rx->l - (NS - 1) - H;
    if (lf >= rx->l_track0 + H && (((lf - rx->fphase) % NS) + NS) % NS == 0) process_frame(rx, lf);
    rx->l++;
}

/* ============================== Eingang ============================== */
void drm_rx_push(drm_rx_t *rx, const drm_cs16_t *iq, int n)
{
    for (int i = 0; i < n; i++) {
        rx->buf[rx->nin & (BUFN - 1)] = iq[i]; rx->nin++;
        switch (rx->state) {
        case DRM_ST_ACQ:
            acq_sample(rx);
            if (rx->acq_n >= (int64_t)ACQ_BLOCKS * 352) acq_decide(rx);
            break;
        case DRM_ST_SYNC:  while (rx->state == DRM_ST_SYNC  && symbol_ready(rx, 0)) sync_step(rx);  break;
        case DRM_ST_TRACK: while (rx->state == DRM_ST_TRACK && symbol_ready(rx, (int)floorf(rx->tcorr))) track_step(rx); break;
        }
    }
}
