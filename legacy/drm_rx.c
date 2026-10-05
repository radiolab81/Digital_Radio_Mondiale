#include "drm_rx.h"
#include <math.h>

static float rot_c[DRM_NCAR], rot_s[DRM_NCAR];
static int rot_adv = -1000;
static void rot_init(int adv)
{
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
        float a = 2.0f*(float)M_PI*k*adv/DRM_TU;      /* Fenster liegt adv Samples frueher */
        rot_c[k-DRM_KMIN] = cosf(a); rot_s[k-DRM_KMIN] = sinf(a);
    }
    rot_adv = adv;
}

float drm_time_ref_metric(const drm_cell_t *cells, int adv, int shift)
{
    if (rot_adv != adv) rot_init(adv);
    float pr = 0, pi = 0, pw = 1e-9f; int have = 0; float lr = 0, li = 0;
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
        const drm_cellinfo_t *c = &drm_map[0][k-DRM_KMIN];
        if (c->type != CT_TIME) continue;
        int j = k - DRM_KMIN + shift;                 /* Zellindex bei Frequenzversatz 'shift' Traeger */
        if (j < 0 || j >= DRM_NCAR) continue;
        float ur, ui; drm_ref_value(0, k, &ur, &ui);
        float xr = CELL_RE(cells[j]), xi = CELL_IM(cells[j]);
        float c_ = rot_c[k-DRM_KMIN], s_ = rot_s[k-DRM_KMIN];
        float rr = xr*c_ - xi*s_, ri = xr*s_ + xi*c_;
        float yr = rr*ur + ri*ui, yi = ri*ur - rr*ui;
        if (have) { pr += yr*lr + yi*li; pi += yi*lr - yr*li; pw += sqrtf((yr*yr+yi*yi)*(lr*lr+li*li)); }
        lr = yr; li = yi; have = 1;
    }
    return sqrtf(pr*pr + pi*pi) / pw;
}

void drm_chan_est(const drm_cell_t *cells, int s, int adv, float *xr, float *xi, float *hr, float *hi)
{
    if (rot_adv != adv) rot_init(adv);
    int kp[DRM_NCAR], np = 0; float ph[DRM_NCAR], mg[DRM_NCAR];
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
        int i = k - DRM_KMIN;
        float a = CELL_RE(cells[i]), b = CELL_IM(cells[i]);
        xr[i] = a*rot_c[i] - b*rot_s[i]; xi[i] = a*rot_s[i] + b*rot_c[i];
        uint8_t t = drm_map[s][i].type;
        if (t == CT_FREQ || t == CT_TIME || t == CT_GAIN) {
            float ur, ui; drm_ref_value(s, k, &ur, &ui);
            float er = xr[i]*ur + xi[i]*ui, ei = xi[i]*ur - xr[i]*ui;   /* X/U*|U|^2 */
            float u2 = ur*ur + ui*ui;
            kp[np] = k; mg[np] = sqrtf(er*er+ei*ei)/u2; ph[np] = atan2f(ei, er); np++;
        }
    }
    /* Phase entfalten und polar interpolieren */
    for (int j = 1; j < np; j++) {
        float d = ph[j] - ph[j-1];
        while (d >  (float)M_PI) { ph[j] -= 2*(float)M_PI; d -= 2*(float)M_PI; }
        while (d < -(float)M_PI) { ph[j] += 2*(float)M_PI; d += 2*(float)M_PI; }
    }
    int j = 0;
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
        while (j < np-2 && k > kp[j+1]) j++;
        float t = (float)(k - kp[j]) / (float)(kp[j+1] - kp[j]);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        float m = mg[j] + t*(mg[j+1]-mg[j]), p = ph[j] + t*(ph[j+1]-ph[j]);
        hr[k-DRM_KMIN] = m*cosf(p); hi[k-DRM_KMIN] = m*sinf(p);
    }
}

void drm_pilot_stats(const float *xr, const float *xi, int s, float *slope, float *fe)
{
    float pr = 0, pi = 0, lr = 0, li = 0; int lk = 0, have = 0, nf = 0;
    for (int k = DRM_KMIN; k <= DRM_KMAX; k++) {
        int i = k - DRM_KMIN; uint8_t t = drm_map[s][i].type;
        if (t != CT_FREQ && t != CT_TIME && t != CT_GAIN) continue;
        float ur, ui; drm_ref_value(s, k, &ur, &ui);
        float er = xr[i]*ur + xi[i]*ui, ei = xi[i]*ur - xr[i]*ui;
        if (t == CT_FREQ && nf < 3) { fe[2*nf] = er; fe[2*nf+1] = ei; nf++; }
        if (have && k - lk == 6) { pr += er*lr + ei*li; pi += ei*lr - er*li; }
        lr = er; li = ei; lk = k; have = 1;
    }
    *slope = atan2f(pi, pr) / 6.0f;
}

static float wrap_pi(float d)
{
    while (d >  (float)M_PI) d -= 2*(float)M_PI;
    while (d < -(float)M_PI) d += 2*(float)M_PI;
    return d;
}

/* Pilotwert E = X/U an (Fensterindex w, Index i); 0 wenn dort kein Pilot */
static int pilot_at(float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], int w, int i, float *er, float *ei)
{
    uint8_t t = drm_map[sph[w]][i].type;
    if (t != CT_FREQ && t != CT_TIME && t != CT_GAIN) return 0;
    float ur, ui; drm_ref_value(sph[w], i + DRM_KMIN, &ur, &ui);
    float u2 = ur*ur + ui*ui;
    *er = (xr[w][i]*ur + xi[w][i]*ui) / u2; *ei = (xi[w][i]*ur - xr[w][i]*ui) / u2;
    return 1;
}

int drm_ce_mode = 1;   /* Standard: komplexe Interpolation in Frequenzrichtung */
void drm_chan_est_tf(float *const xr[DRM_TW], float *const xi[DRM_TW], const int sph[DRM_TW], float *hr, float *hi)
{
    static float m[DRM_NCAR], p[DRM_NCAR]; static uint8_t known[DRM_NCAR];
    const int C = DRM_TW/2;
    for (int i = 0; i < DRM_NCAR; i++) {
        known[i] = 0;
        float tt[DRM_TW], mg[DRM_TW], ph[DRM_TW]; int n = 0;
        for (int w = 0; w < DRM_TW; w++) {
            float er, ei; if (!pilot_at(xr, xi, sph, w, i, &er, &ei)) continue;
            float pp = atan2f(ei, er);
            if (n) pp = ph[n-1] + wrap_pi(pp - ph[n-1]);
            tt[n] = (float)(w - C); mg[n] = sqrtf(er*er+ei*ei); ph[n] = pp; n++;
        }
        if (!n) continue;
        /* Ausgleichsgerade ueber Beobachtungen, ausgewertet bei t=0 */
        float st=0, sm=0, sp=0, stt=0, stm=0, stp=0;
        for (int j = 0; j < n; j++) { st += tt[j]; sm += mg[j]; sp += ph[j]; stt += tt[j]*tt[j]; stm += tt[j]*mg[j]; stp += tt[j]*ph[j]; }
        float det = n*stt - st*st;
        if (n >= 2 && det > 1e-6f) {
            float bm = (n*stm - st*sm)/det, bp = (n*stp - st*sp)/det;
            m[i] = (sm - bm*st)/n; p[i] = (sp - bp*st)/n;
        } else { m[i] = sm/n; p[i] = sp/n; }
        known[i] = 1;
    }
    int left = -1;
    for (int i = 0; i < DRM_NCAR; i++) {
        if (known[i]) { hr[i] = m[i]*cosf(p[i]); hi[i] = m[i]*sinf(p[i]); left = i; continue; }
        int right = -1; for (int j = i+1; j < DRM_NCAR; j++) if (known[j]) { right = j; break; }
        if (left < 0 && right < 0) { hr[i] = 1; hi[i] = 0; continue; }
        float mm, pp;
        if (left < 0)       { mm = m[right]; pp = p[right]; }
        else if (right < 0) { mm = m[left];  pp = p[left]; }
        else {
            float t = (float)(i-left)/(float)(right-left);
            mm = m[left] + t*(m[right]-m[left]);
            pp = p[left] + t*wrap_pi(p[right]-p[left]);
            if (drm_ce_mode == 1) {
                float ar = m[left]*cosf(p[left]), ai = m[left]*sinf(p[left]);
                float br = m[right]*cosf(p[right]), bi = m[right]*sinf(p[right]);
                hr[i] = ar + t*(br-ar); hi[i] = ai + t*(bi-ai); continue;
            }
        }
        hr[i] = mm*cosf(pp); hi[i] = mm*sinf(pp);
    }
}

void drm_rotate_cells(const drm_cell_t *cells, int adv, float *xr, float *xi)
{
    if (rot_adv != adv) rot_init(adv);
    for (int i = 0; i < DRM_NCAR; i++) {
        float a = CELL_RE(cells[i]), b = CELL_IM(cells[i]);
        xr[i] = a*rot_c[i] - b*rot_s[i]; xi[i] = a*rot_s[i] + b*rot_c[i];
    }
}
