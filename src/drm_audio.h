/* drm_audio.h -- Audio-Superframes (xHE-AAC und AAC) aus dem MSC herausloesen. */
#ifndef DRM_AUDIO_H
#define DRM_AUDIO_H
#include "drm_cfg.h"
#include "drm_sdc.h"

/* Datenstrom i aus den entschluesselten MSC-Bytes eines Multiplexrahmens holen [6.2.3]: Teil A aller Stroeme steht
 * vorn, dann Teil B aller Stroeme; Strom i = A_i gefolgt von B_i.  Rueckgabe: Laenge in Byte (0 bei Fehler). */
int drm_mux_stream(const uint8_t *mux, int mux_len, const drm_sdc_t *sd, int stream, uint8_t *out, int outmax);

/* Callback fuer Audiorahmen: len > 0 = gueltiger Rahmen (xHE-AAC: Access Unit inkl. CRC-16; AAC: Rohrahmen),
 * len = 0 = Rahmen mit CRC-Fehler, len = -1 = ganzer Superframe unbrauchbar. */
typedef void (*drm_frame_cb)(void *user, const uint8_t *frame, int len);

/* ---- xHE-AAC [5.3.1]: Kopf 2 Byte (Rahmenzahl/Bit-Reservoir, CRC-8), Nutzteil, am Ende Rahmengrenzen-Verzeichnis ---- */
typedef struct { uint8_t openf[16384]; int openlen, have_open; int hdr_ok, hdr_n, af_ok, af_bad, af_lost; } drm_asf_t;
void drm_asf_init(drm_asf_t *a);
void drm_asf_parse(drm_asf_t *a, const uint8_t *data, int asf_len, drm_frame_cb cb, void *user);

/* ---- AAC [5.4.1, Tab. 10/11]: Kopf mit (n-1) 12-Bit-Rahmengrenzen, n CRC-8-Bytes, dann die Rahmen ---- */
typedef void (*drm_aac_cb)(void *user, const uint8_t *frame, int len, int crc8);
/* Rueckgabe 1 = Kopf plausibel (Grenzen steigend, im Nutzteil), sonst 0 (cb mit len = -1). */
int drm_aac_asf_parse(const uint8_t *s, int asf_len, int nframes, drm_aac_cb cb, void *user);
/* Rahmenzahl je Superframe aus dem SDC-Abtastratencode (001 = 12 kHz -> 5, 011 = 24 kHz -> 10), sonst 0 [5.4.1]. */
int drm_aac_nframes(int sdc_rate_code);
#endif
