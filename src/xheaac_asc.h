/* ==========================================================================================================
 * xheaac_asc.h  --  DRM-xHE-AAC-"Static Config" (SDC, ES 201 980 [5.3.2]) -> MPEG-D-USAC-AudioSpecificConfig
 * ==========================================================================================================
 * DRM uebertraegt die Decoderkonfiguration nicht im Audiostrom, sondern verkuerzt im SDC (Audio-Entitaet,
 * Feld "xHE-AAC Static Config", Syntax [5.3.2.x]).  Ein USAC-Dekoder (hier FDK-AAC) erwartet dagegen den normalen
 * AudioSpecificConfig nach ISO/IEC 14496-3 mit UsacConfig() nach ISO/IEC 23003-3.  Die Funktion baut diese
 * Konfiguration aus den DRM-Feldern (coreSbrFrameLengthIndexDrm -> coreSbrFrameLengthIndex, noiseFilling,
 * SbrConfig, Stereo mit MPS212, Erweiterungselemente) und fuegt die bei DRM weggelassenen festen Felder ein.
 * variant 0: AOT 42 + samplingFrequencyIndex + channelConfiguration + UsacConfig();  variant 1: AOT 42 + UsacConfig().
 * ========================================================================================================== */
#ifndef XHEAAC_ASC_H
#define XHEAAC_ASC_H
#include <stdint.h>
/* Baut aus der DRM-xHE-AAC-Static-Config (SDC Typ 9, ES 201 980 Kap. 5.3.2) einen
 * MPEG-D-USAC-AudioSpecificConfig.  variant 0: AOT 42 + samplingFrequencyIndex +
 * channelConfiguration + UsacConfig();  variant 1: AOT 42 + UsacConfig() direkt.
 * Rueckgabe: Laenge in Bytes, oder -1 bei Fehler.  mono_stereo: SDC-Audiomodus (0=mono, 2=stereo). */
int xheaac_build_asc(const uint8_t *drmcfg, int drmlen, int audio_mode, int out_rate_hz,
                     int variant, uint8_t *out, int outmax);
#endif
