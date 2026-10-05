CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra
CORE = src/drm_mode.c src/drm_fft.c src/drm_ofdm.c src/drm_chan.c src/drm_fac.c src/drm_sdc.c src/drm_msc.c src/drm_audio.c src/drm_viterbi.c src/drm_rx.c
all: drm_test iq_prep drm_sim drm_gen
drm_test: src/drm_test.c $(CORE) ; $(CC) $(CFLAGS) -o $@ $^ -lm
iq_prep:  src/iq_prep.c ; $(CC) -O2 -o $@ $^ -lm
drm_sim:  src/drm_sim.c src/drm_msc.c src/drm_mode.c src/drm_viterbi.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
drm_gen:  src/drm_gen.c src/drm_mode.c src/drm_fft.c src/drm_fac.c src/drm_sdc.c src/drm_msc.c src/drm_audio.c src/drm_viterbi.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
# Audio-Dekoder-Wrapper (optional): FDK-AAC (xHE-AAC) bzw. FAAD2 mit DRM-Unterstuetzung (AAC)
FDK ?= $(HOME)/fdk
xhe_dec: src/xhe_dec.c src/xheaac_asc.c ; $(CC) $(CFLAGS) -I$(FDK)/include -o $@ $^ -L$(FDK)/lib -lfdk-aac -Wl,-rpath,$(FDK)/lib
FAAD ?= $(HOME)/faad
aac_dec: src/aac_dec.c ; $(CC) $(CFLAGS) -I$(FAAD)/include -o $@ $^ -L$(FAAD)/lib -lfaad -Wl,-rpath,$(FAAD)/lib
clean: ; rm -f drm_test iq_prep drm_sim drm_gen xhe_dec aac_dec
