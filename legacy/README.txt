Stufen 1-5 des Projekts (nur Mode B / 10 kHz, fest verdrahtet), enthalten u. a. die Q15-Festkomma-FFT (drm_fft.c) und die
Q15-Variante des OFDM-Frontends (-DDRM_FIXED).  Sie dienen als Ausgangspunkt fuer einen Ganzzahlpfad auf MCUs ohne FPU und
sind nicht mehr Teil des aktuellen Empfaengers (src/).  Bauen: make (in diesem Ordner).
