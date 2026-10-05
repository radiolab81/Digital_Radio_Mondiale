/* ==========================================================================================================
 * drm_chan.c  --  Kanalschaetzung, Rahmensynchronisation und Pilotstatistik
 * ==========================================================================================================
 * Pilotgestuetzte Kanalschaetzung (Lehrbuch): Der Funkkanal wirkt auf jeden Traeger k in jedem Symbol s wie ein
 * komplexer Faktor H(s,k) (Daempfung + Phasendrehung).  An Pilotzellen kennt der Empfaenger den gesendeten Wert
 * U(s,k) (Betrag und Phase aus der Norm, siehe drm_mode.c).  Dort gilt  X = H * U + Rauschen, also
 *      H_est(s,k) = X(s,k) / U(s,k).
 * Zwischen den Piloten wird interpoliert (zweidimensional, Zeit x Frequenz), danach werden die Datenzellen
 * entzerrt:  z = X / H_est  (Einheitsleistung wie gesendet).  Das Pilotraster der DRM-Modi (x*y) ist so
 * dicht, dass die Abtasttheoreme fuer typische Kurzwellenkanaele erfuellt sind [8.4.4, Tab. 60]: Verzoegerung
 * (Frequenzrichtung) bis ca. Tu/(2x), Doppler (Zeitrichtung) bis ca. 1/(2*y*Ts).
 *
 * Vorgehen hier:  (1) je Traeger, an dem im Zeitfenster Piloten liegen, wird H ueber die Zeit durch eine Aus-
 * gleichsgerade (Betrag und entfaltete Phase) geschaetzt; (2) fehlende Traeger werden aus den Nachbarn linear
 * interpoliert; (3) optional wird vorher in Frequenzrichtung ueber Nachbarpiloten gemittelt (senkt das
 * Schaetzrauschen).
 *
 * Nachfuehrung:  Ein Zeitfehler tau (Symbolfenster verschoben) erzeugt ueber k eine lineare Phase
 * exp(-j*2*pi*k*tau/Tu): aus der mittleren Phasensteigung der Pilote jedes Symbols folgt tau.  Ein Frequenz-
 * fehler dreht alle Traeger von Symbol zu Symbol gleichartig: aus der Phasenaenderung der drei Frequenzreferenzen
 * [8.4.2] zwischen Symbolen folgt df = arg(.)/(2*pi*Ts).
 * ========================================================================================================== */
#include "drm_chan.h"
#include <math.h>

/* ---- Drehfaktoren exp(+j*2*pi*k*adv/Tu) je Traeger, zwischengespeichert fuer (Modus, Belegung, adv) ---- */
static float rot_c[DRM_MAXCAR], rot_s[DRM_MAXCAR];
static int rot_adv = -1000, rot_mode = -1, rot_occ = -1;
static void rot_chk(const drm_mode_t *m, int adv)
{
    if (rot_adv == adv && rot_mode == m->id && rot_occ == m->occ) return;
    for (int k = m->kmin; k <= m->kmax; k++) { float a = 2.0f * (float)M_PI * k * adv / m->Tu; rot_c[k - m->kmin] = cosf(a); rot_s[k - m->kmin] = sinf(a); }
    rot_adv = adv; rot_mode = m->id; rot_occ = m->occ;
}

/* Das FFT-Fenster liegt adv Werte VOR dem Guardende: Zeitverschiebung um -adv  <=>  Faktor exp(-j*2*pi*k*adv/Tu)
 * auf Traeger k; hier wird er rueckgaengig gemacht, damit Pilotphasen "bei Fensterlage 0" erscheinen. */
void drm_rotate_cells(const drm_mode_t *m, const drm_cell_t *cells, int adv, float *xr, float *xi)
{
    rot_chk(m, adv);
    for (int i = 0; i < m->ncar; i++) {
        float a = cells[i].re, b = cells[i].im;
        xr[i] = a * rot_c[i] - b * rot_s[i];
        xi[i] = a * rot_s[i] + b * rot_c[i];
    }
}

/* ---- Rahmensynchronisation [8.4.3] ----------------------------------------------------------------------
 * Im ersten Symbol jedes Rahmens liegen Zeitreferenzzellen mit bekannten Phasen U(0,k).  Bildet man y_k = X_k *
 * conj(U_k), so haben alle y_k bei richtigem Symbol dieselbe Phase (= Kanalphase, die sich zwischen benachbarten
 * Traegern kaum aendert).  Die Kohaerenz  |sum y_k*conj(y_{k-1})| / sum|y_k||y_{k-1}|  ist dann nahe 1, sonst nahe 0.
 * shift testet einen ganzzahligen Frequenzversatz in Traegern (Signal erscheint bei k+shift). */
float drm_time_ref_metric(const drm_mode_t *m, const drm_cell_t *cells, int adv, int shift)
{
    rot_chk(m, adv);
    float pr = 0, pi = 0, pw = 1e-9f, lr = 0, li = 0; int have = 0;
    for (int k = m->kmin; k <= m->kmax; k++) {
        if (m->map[0][k - m->kmin].type != CT_TIME) continue;
        int j = k - m->kmin + shift; if (j < 0 || j >= m->ncar) continue;
        float ur, ui; drm_ref_value(m, 0, k, &ur, &ui);
        float xr = cells[j].re, xi = cells[j].im, c = rot_c[k - m->kmin], s = rot_s[k - m->kmin];
        float rr = xr * c - xi * s, ri = xr * s + xi * c;                 /* Fensterlage entfernt */
        float yr = rr * ur + ri * ui, yi = ri * ur - rr * ui;             /* y = X * conj(U) */
        if (have) { pr += yr * lr + yi * li; pi += yi * lr - yr * li; pw += sqrtf((yr * yr + yi * yi) * (lr * lr + li * li)); }
        lr = yr; li = yi; have = 1;
    }
    return sqrtf(pr * pr + pi * pi) / pw;
}

/* ---- Nachfuehrungsgroessen -------------------------------------------------------------------------------- */
void drm_pilot_stats(const drm_mode_t *m, const float *xr, const float *xi, int s, float *slope, float *fe)
{
    static const int dk[4] = {20, 6, 4, 3};         /* Abstand benachbarter Gain-Piloten im selben Symbol = x*y [Tab. 60] */
    float pr = 0, pi = 0, lr = 0, li = 0; int lk = 0, have = 0, nf = 0;
    for (int k = m->kmin; k <= m->kmax; k++) {
        int i = k - m->kmin; uint8_t t = m->map[s][i].type;
        if (t != CT_FREQ && t != CT_TIME && t != CT_GAIN) continue;
        float ur, ui; drm_ref_value(m, s, k, &ur, &ui);
        float er = xr[i] * ur + xi[i] * ui, ei = xi[i] * ur - xr[i] * ui;      /* E = X * conj(U) (Betrag ~ |H| * |U|^2) */
        if (t == CT_FREQ && nf < 3) { fe[2 * nf] = er; fe[2 * nf + 1] = ei; nf++; }
        if (have && k - lk == dk[m->id]) { pr += er * lr + ei * li; pi += ei * lr - er * li; }   /* Phasenschritt zwischen Nachbarpiloten */
        lr = er; li = ei; lk = k; have = 1;
    }
    *slope = atan2f(pi, pr) / (float)dk[m->id];                                /* rad je Traegerabstand */
}

/* ---- Kanalschaetzung -------------------------------------------------------------------------------------- */
static float wrap_pi(float d) { while (d > (float)M_PI) d -= 2 * (float)M_PI; while (d < -(float)M_PI) d += 2 * (float)M_PI; return d; }

/* E = X/U (mit |U|^2 normiert) am Fensterindex w, Traegerindex i, falls dort ein Pilot liegt. */
static int pilot_at(const drm_mode_t *m, float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], int w, int i, float *er, float *ei)
{
    uint8_t t = m->map[sph[w]][i].type;
    if (t != CT_FREQ && t != CT_TIME && t != CT_GAIN) return 0;
    float ur, ui; drm_ref_value(m, sph[w], i + m->kmin, &ur, &ui); float u2 = ur * ur + ui * ui;
    *er = (xr[w][i] * ur + xi[w][i] * ui) / u2;
    *ei = (xi[w][i] * ur - xr[w][i] * ui) / u2;
    return 1;
}

void drm_chan_est_tf(const drm_mode_t *m, float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], int smooth, float *hr, float *hi)
{
    static float mg_[DRM_MAXCAR], ph_[DRM_MAXCAR], kr[DRM_MAXCAR], ki[DRM_MAXCAR], sr[DRM_MAXCAR], si[DRM_MAXCAR];
    static uint8_t known[DRM_MAXCAR];
    const int C = DRM_TW / 2, NC = m->ncar;

    /* (1) Zeitrichtung: je Traeger aus allen Pilotbeobachtungen im Fenster eine Gerade (Betrag, entfaltete Phase) bei t = 0 */
    for (int i = 0; i < NC; i++) {
        known[i] = 0; float tt[DRM_TW], mg[DRM_TW], ph[DRM_TW]; int n = 0;
        for (int w = 0; w < DRM_TW; w++) {
            float er, ei; if (!pilot_at(m, xr, xi, sph, w, i, &er, &ei)) continue;
            float pp = atan2f(ei, er); if (n) pp = ph[n - 1] + wrap_pi(pp - ph[n - 1]);        /* Phase entfalten */
            tt[n] = (float)(w - C); mg[n] = sqrtf(er * er + ei * ei); ph[n] = pp; n++;
        }
        if (!n) continue;
        float st = 0, sm = 0, sp = 0, stt = 0, stm = 0, stp = 0;
        for (int j = 0; j < n; j++) { st += tt[j]; sm += mg[j]; sp += ph[j]; stt += tt[j] * tt[j]; stm += tt[j] * mg[j]; stp += tt[j] * ph[j]; }
        float det = n * stt - st * st;                                                       /* kleinste Quadrate */
        if (n >= 2 && det > 1e-6f) { float bm = (n * stm - st * sm) / det, bp = (n * stp - st * sp) / det; mg_[i] = (sm - bm * st) / n; ph_[i] = (sp - bp * st) / n; }
        else { mg_[i] = sm / n; ph_[i] = sp / n; }
        known[i] = 1;
        kr[i] = mg_[i] * cosf(ph_[i]); ki[i] = mg_[i] * sinf(ph_[i]);
    }
    /* (2) optional: Glaettung ueber Nachbarpiloten im Abstand 2 (Gewichte 2,1,1): Phase aus dem komplexen Mittel,
     * Betrag aus dem Betragsmittel (so schrumpft der Betrag bei Phasenstreuung nicht) */
    if (smooth) {
        for (int i = 0; i < NC; i++) { if (!known[i]) continue;
            float cw = 2.0f, ar = kr[i] * cw, ai = ki[i] * cw, am = mg_[i] * cw, ws = cw;
            for (int d = -2; d <= 2; d += 4) { int j = i + d; if (j >= 0 && j < NC && known[j]) { ar += kr[j]; ai += ki[j]; am += mg_[j]; ws += 1; } }
            float pm = sqrtf(ar * ar + ai * ai) + 1e-12f, mm = am / ws;
            sr[i] = ar / pm * mm; si[i] = ai / pm * mm; }
        for (int i = 0; i < NC; i++) if (known[i]) { kr[i] = sr[i]; ki[i] = si[i]; }
    }
    /* (3) Frequenzrichtung: Traeger ohne Pilot im Fenster aus den naechsten bekannten Nachbarn linear interpolieren */
    int left = -1;
    for (int i = 0; i < NC; i++) {
        if (known[i]) { hr[i] = kr[i]; hi[i] = ki[i]; left = i; continue; }
        int right = -1; for (int j = i + 1; j < NC; j++) if (known[j]) { right = j; break; }
        if (left < 0 && right < 0) { hr[i] = 1; hi[i] = 0; continue; }
        if (left < 0)  { hr[i] = kr[right]; hi[i] = ki[right]; continue; }
        if (right < 0) { hr[i] = kr[left];  hi[i] = ki[left];  continue; }
        float t = (float)(i - left) / (float)(right - left);
        hr[i] = kr[left] + t * (kr[right] - kr[left]); hi[i] = ki[left] + t * (ki[right] - ki[left]);
    }
}
