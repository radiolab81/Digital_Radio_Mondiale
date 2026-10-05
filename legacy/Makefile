# Stufe 1-5 (nur Mode B / 10 kHz, fest verdrahtet, mit Q15-Variante des OFDM-Frontends)
CORE = drm_msc.c drm_sdc.c drm_ofdm.c drm_fft.c drm_modeb.c drm_rx.c drm_fac.c drm_viterbi.c
all: probe probe_fx fac_test fac_test_fx
probe:       probe.c    $(CORE) ; $(CC) -O2 -Wall           -o $@ $^ -lm
probe_fx:    probe.c    $(CORE) ; $(CC) -O2 -Wall -DDRM_FIXED -o $@ $^ -lm
fac_test:    fac_test.c $(CORE) ; $(CC) -O2 -Wall           -o $@ $^ -lm
fac_test_fx: fac_test.c $(CORE) ; $(CC) -O2 -Wall -DDRM_FIXED -o $@ $^ -lm
