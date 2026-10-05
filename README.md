# DRM-Decoder in portablem C

Ein Decoder für **Digital Radio Mondiale** (DRM, Kurzwelle/Mittelwelle/Langwelle) vom komplexen
Basisband bis zu den Audiorahmen, geschrieben in portablem C (C99, nur `<stdint.h>`, `<string.h>`, `<math.h>`, **kein malloc im Kern**).
Er läuft auf dem PC (Debian 13 getestet) und ist für eine Portierung auf MCUs (ESP32‑P4 u. ä.) aufgebaut.
Die Audiodekodierung selbst (xHE‑AAC, AAC) übernehmen vorhandene Bibliotheken (FDK‑AAC, FAAD2); die Anbindung liegt bei.

Norm: **ETSI ES 201 980 V4.3.1 (2023‑11)**, „DRM System Specification“. Verweise wie **[8.4.2]** im Quelltext und in dieser Datei
meinen die Kapitel dieser Norm.

---

## 1. Das DRM-System in Kürze

DRM überträgt digitales Audio (plus Daten) in den AM‑Bändern unter 30 MHz in Kanälen von 4,5 bis 20 kHz. Das Signal besteht aus
**OFDM** (Orthogonal Frequency Division Multiplex): viele schmale, gleichzeitig gesendete Unterträger, jeder mit einer
**QAM**‑Zelle (4‑, 16‑ oder 64‑stufig) moduliert [8]. Weil die Symbole lang sind und ein **Guardintervall** (zyklisches Präfix)
tragen, verkraftet das Signal die Mehrwegeausbreitung der Raumwelle.

**Robustheitsmodi** [8.2]: A (Grundwelle/gutmütig), B (Raumwelle, Standard Kurzwelle), C und D (starke Dopplerverschiebung,
lange Echos). Sie unterscheiden sich in Trägerabstand und Guardintervall:

| Modus | Nutzsymbol Tu | Guard Tg | Trägerabstand | Symbole/Rahmen | Einsatz |
|---|---|---|---|---|---|
| A | 24,00 ms | 2,67 ms | 41,7 Hz | 15 | geringe Echos |
| B | 21,33 ms | 5,33 ms | 46,9 Hz | 15 | typische Kurzwelle |
| C | 14,67 ms | 5,33 ms | 68,2 Hz | 20 | starker Doppler |
| D | 9,33 ms | 7,33 ms | 107 Hz | 24 | Doppler + lange Echos |

**Zeitstruktur** [8.1]: Symbol → **Übertragungsrahmen** (400 ms) → **Superframe** (3 Rahmen = 1,2 s).

**Drei logische Kanäle** teilen sich das Zellraster:

* **FAC** (Fast Access Channel, 65 Zellen je Rahmen): Modus‑Zusatzinfos, Spektrumbelegung, QAM‑Art, Verschachtelungstiefe [6.3, 7.5.3].
* **SDC** (Service Description Channel, Beginn jedes Superframes): beschreibt die Datenströme, Schutzstufen, Audiocodec, Programmname [6.4, 7.5.2].
* **MSC** (Main Service Channel): die Nutzdaten (Audio) in 16‑ oder 64‑QAM mit **Mehrebenencode** und optional
  **ungleichem Fehlerschutz (UEP)** [7.3, 7.5.1]; Zeitverschachtelung über 5 Rahmen (2 s) oder 1 Rahmen [7.6].

Dazu kommen **Pilotzellen** (Frequenz‑, Zeit‑ und Verstärkungsreferenzen) mit bekannter Amplitude und Phase [8.4], mit denen der
Empfänger den Kanal schätzt, den Rahmenanfang findet und Zeit/Frequenz nachführt.

**Audiocodecs** [5]: **xHE‑AAC** (USAC, neu) und **AAC** (ER‑AAC, DRM‑spezifisch, klassisch). CELP/HVXC sind in dieser Ausgabe der Norm
nicht mehr enthalten.

## 2. Signalkette dieses Empfängers/Dekoders

```
 Eingang: komplexes Basisband, 12 kHz, DRM-Band um 0 Hz      (iq_prep rechnet WAV-Aufnahmen darauf um)
   │
   ▼
 [ACQ]  Guard-Korrelation  C(t)=Σ x(t+i)·x*(t+i+Tu)                    drm_ofdm.c / drm_rx.c   [8.1, 8.2]
   │      → Robustheitsmodus A–D, Symboltakt, Bruchteil-Frequenzoffset
   ▼
 [SYNC] Kohärenz der Zeitreferenzzellen über alle Rahmenphasen und ±12 Träger      drm_chan.c    [8.4.3]
   │      → Rahmenanfang, ganzzahliger Trägerversatz
   ▼
 [TRACK] je OFDM-Symbol (alle ~26 ms):
   │   Bruchteil-Fenster (24-Tap-Interpolator) → Mischer (NCO) → FFT (Mixed-Radix 2/3/7/11)   drm_ofdm.c, drm_fft.c
   │   Fensterlage entdrehen → Pilotstatistik → PI-Regler Zeit (Abtasttakt), P-Regler Frequenz   drm_chan.c, drm_rx.c
   ▼
 je Rahmen (400 ms), mit 2 Symbolen Nachlauf:
   │   Kanalschätzung H(t,f): Piloten → Gerade über Zeit → Interpolation über Frequenz          drm_chan.c   [8.4.4]
   │   Entzerrung z = X/H, Zuverlässigkeit w = |H|²
   ├─► FAC:  Bit-Entschachtelung → Entpunktieren (3/5) → Viterbi → Entwürfeln → CRC-8      drm_fac.c   [7.5.3]
   ├─► SDC:  (4-QAM oder 16-QAM) → Viterbi → Entwürfeln → CRC-16 → Entitäten (Typ 0/1/9)     drm_sdc.c   [7.5.2, 6.4]
   └─► MSC:  Zellen sammeln → Zell-Deinterleaver (D=5 oder 1) → Mehrebenen-Dekodierung        drm_msc.c   [7.6, 7.3]
              (Soft-QAM-Demapper je Ebene, Bit-Deinterleaver, Entpunktieren, Viterbi, Re-Encoding,
               Rückkopplung aller Ebenen, UEP-Teile A/B, Entwürfeln)
              → Datenstrom i = Teil A ∥ Teil B [6.2.3] → Audio-Superframe zerlegen                drm_audio.c [5.3.1, 5.4.1]
              → Audiorahmen (xHE-AAC: Access Unit + CRC-16 | AAC: Rohrahmen)  ──►  Ereignis DRM_EV_AUDIO
```

Ausgabe sind **Ereignisse** (`drm_rx.h`): Zustandswechsel, gültige FAC, zusammengeführte SDC‑Information, Audiorahmen. Die
Dekodierung der Audiorahmen zu PCM erledigen `xhe_dec` (FDK‑AAC) und `aac_dec` (FAAD2).

## 3. Dateien

| Datei | Inhalt | Norm |
|---|---|---|
| `src/drm_cfg.h` | Typen, Obergrenzen | |
| `src/drm_mode.[ch]` | Modi A–D: Parameter, Pilot-/Zellraster, Phasentabellen | [8.2–8.6] |
| `src/drm_fft.[ch]` | Mixed‑Radix‑FFT 288/256/176/112 | [8.2] |
| `src/drm_ofdm.[ch]` | Modus‑/Takt‑Erkennung, NCO, Bruchteilfenster, FFT | [8.1, 8.2] |
| `src/drm_chan.[ch]` | Kanalschätzung, Zeitreferenz‑Sync, Pilotstatistik | [8.4] |
| `src/drm_viterbi.[ch]` | Faltungscode R=1/6 K=7, Soft‑Viterbi | [7.3.2] |
| `src/drm_fac.[ch]` | FAC | [6.3, 7.5.3] |
| `src/drm_sdc.[ch]` | SDC | [6.4, 7.5.2] |
| `src/drm_msc.[ch]` | MSC: Mehrebenencode, UEP, Interleaver, Sender‑Seite für Tests | [7.2–7.7] |
| `src/drm_audio.[ch]` | Strom‑Extraktion, xHE‑AAC‑ und AAC‑Superframes | [5.3, 5.4, 6.2.3] |
| `src/drm_rx.[ch]` | Streaming‑Empfänger (Zustandsautomat) | |
| `src/drm_test.c` | Kommandozeilenwerkzeug | |
| `src/iq_prep.c` | WAV‑Aufbereitung (reelle ZF/IQ, beliebige Rate → 12 kHz I/Q) | |
| `src/xhe_dec.c`, `src/xheaac_asc.[ch]` | xHE‑AAC → WAV (FDK‑AAC) | [5.3.2] |
| `src/aac_dec.c` | AAC → WAV (FAAD2 mit DRM) | |
| `src/drm_sim.c` | Kanaldecoder‑Selbsttest mit AWGN | |
| `src/drm_gen.c` | synthetischer DRM‑Sender für Ende‑zu‑Ende‑Tests | |
| `tests/run_loopback.sh` | Testmatrix Sender → Empfänger | |
| `legacy/` | frühe Stufen mit Q15‑Festkomma‑FFT | |

## 4. Bauen und Bedienen

```sh
make                                   # drm_test, iq_prep, drm_sim, drm_gen
sh tests/run_loopback.sh               # Selbsttest ohne Aufnahmen (alle Modi, UEP, xHE-AAC/AAC)
./drm_sim 30                           # Kanaldecoder: Mindest-SNR je Konstellation (AWGN)

./iq_prep aufnahme.wav aufnahme_12k.wav        # reelle ZF / beliebige Rate -> komplexes 12-kHz-I/Q  (-f hz: Bandmitte vorgeben)
./drm_test aufnahme_12k.wav rahmen.bin         # Empfang; schreibt rahmen.bin und rahmen.bin.meta
```

**xHE‑AAC‑Ton** (FDK‑AAC ≥ 2.0 mit USAC; Debian 13 hat kein Paket, Bau aus dem Quelltext):

```sh
sudo apt install git build-essential autoconf automake libtool
git clone https://github.com/mstorsjo/fdk-aac && cd fdk-aac && autoreconf -fiv && ./configure --prefix=$HOME/fdk && make -j4 && make install
cd .. && make xhe_dec FDK=$HOME/fdk
./xhe_dec rahmen.bin ton.wav && aplay ton.wav
```

**AAC‑Ton** (FAAD2 mit DRM‑Unterstützung; das Distributionspaket hat sie meist nicht): `make aac_dec FAAD=$HOME/faad`, dann `./aac_dec rahmen.bin ton.wav`.

**Eigener Aufruf der Bibliothek:**

```c
static char mem[1<<21] __attribute__((aligned(16)));          /* drm_rx_size() Byte */
drm_rx_t *rx = drm_rx_init(mem, sizeof mem);
drm_rx_set_callback(rx, on_event, user);                        /* FAC, SDC, Audiorahmen */
for (;;) drm_rx_push(rx, iq_block, n);                          /* 12 kHz komplex, beliebige Blocklaengen */
```

## 5. Getestet an

Echte Aufnahmen (Dream‑Beispieldateien und eigene), Referenz: Dream‑Konsole (aus dem Quelltext gebaut, gleiche Dateien):

| Aufnahme | Modus, MSC / SDC | Pilot‑SNR | Ergebnis |
|---|---|---|---|
| DRM_6030 (12 kHz I/Q) | B 10 kHz, 16‑QAM / 4‑QAM | 19 dB | 674 gültige xHE‑AAC‑Rahmen, Kopf‑CRC 208/211 |
| FM Gold | A 9 kHz, 16‑QAM / 4‑QAM, Stereo | 20 dB (Dream 20,6) | 411 Rahmen, Kopf 134/135, Ton ✔ |
| BBC World Service 3955 kHz | B 10 kHz, 16‑QAM / 4‑QAM | 19 dB | 1773 Rahmen, Kopf 494/495 |
| RTL Slideshow, RTL Test2, Voice of Russia | B, 64‑QAM / 16‑QAM | 8–9 dB | FAC, teils SDC; MSC nicht (Dream ebenso) |
| BouquetFlevoNL | B, 16‑QAM / 16‑QAM | 9 dB | FAC, vereinzelt SDC |
| Deutschlandradio | A 9 kHz, 64‑QAM / 16‑QAM | 11 dB | FAC; SDC/MSC nicht (Dream rastet gar nicht ein) |

Synthetisch (`tests/run_loopback.sh`, 30–35 dB): Modi **A, B, C, D**, Belegungen 9/10 kHz, 16‑ und 64‑QAM, EEP und **UEP**, xHE‑AAC und AAC —
alle fehlerfrei. Mode C/D und UEP sind damit nur gegen den eigenen Sender geprüft (gleiche Tabellen), nicht gegen echte Signale.

**Mindest‑SNR des Kanaldecoders** (`drm_sim`, AWGN, perfekter Kanal, 30/30 Rahmen fehlerfrei, je Zelle):
16‑QAM PL0/PL1 10/11 dB, 64‑QAM PL0–PL3 14/15/18/19 dB.

## 6. Leistung und Portierung

* PC: Echtzeitfaktor **0,01–0,02** (≈ 1–2 % eines Kerns). Das meiste kostet der Viterbi‑Dekoder (Profil), er ist bereits optimiert (Zweigmetriken in 63 Additionen, bitgenau zur Referenzfassung).
* RAM: ca. 0,6 MB für Mode A 10 kHz (Hauptanteil: Zellen der 5 Multiplexrahmen der Zeitverschachtelung, 5 × 2959 × 12 Byte). Halbierbar durch Ablage der Zellen als `int16`.
* Der Kern braucht keinen `malloc`; alle Puffer liegen in `drm_rx_t`.
* ESP32‑P4 (FPU, 400 MHz): Mode‑B‑Empfang ohne Audiodekoder auf einem Kern unter ca. 30 % Last. xHE‑AAC‑Dekodierung kommt dazu.
* Ganzzahl: Viterbi arbeitet bereits mit `int8`/`int32`. Die Q15‑FFT und das Q15‑Frontend liegen in `legacy/`; Kanalschätzung und MSC‑Demapper rechnen in `float`.

## 7. Grenzen und Offenes

Nicht enthalten:

* **Hierarchical Mapping** (HMsym/HMmix): in ES 201 980 V4.3.1 nicht mehr definiert (FAC‑Feld „MSC mode“ 01/10 reserviert).
* **CELP/HVXC**: in dieser Normausgabe nicht mehr enthalten (Audio‑Codec‑Feld 01/10 reserviert).
* **Spektrumbelegung 4/5 (18/20 kHz) und Mode E (DRM+)**: brauchen mehr als 12 kHz Abtastrate; die Modi C/D kennen nur die 10‑kHz‑Belegung.
* **Datendienste** (Paketmodus, MOT/Slideshow, Journaline) und Dienstauswahl: es wird der erste Audiostrom des SDC dekodiert. Textmeldungen (letzte 4 Byte) werden verworfen.
* **AAC**: der Rahmen‑CRC‑8 (nur über Teilbereiche, braucht einen AAC‑Parser) wird nicht geprüft; AAC wurde nur mit einer von Dream erzeugten synthetischen Datei geprüft.
* **Mode C/D und UEP** nur im Loopback getestet.
* Start: bis zum ersten Audiorahmen vergehen ca. 6–7 s (Suche, Rahmentakt, FAC‑Phase, 2 s Verschachtelung).
* 64‑QAM braucht in der Praxis deutlich mehr als die Mindest‑SNR des Simulators (Fading, Kanalschätzverlust).

## 8. Normverweise nach Thema

Zeit/Rahmen [8.1], OFDM‑Parameter [8.2, Tab. 47], Träger [8.3.1, Tab. 49/50], Piloten [8.4: Frequenz 8.4.2, Zeit 8.4.3, Gain 8.4.4], FAC‑/SDC‑Zellpositionen [8.5], Datenzellen [8.6];
Energieverwischung [7.2.2], Mehrebenencode [7.3.1], Faltungscode [7.3.2, Tab. 27/28], Bit‑Interleaver [7.3.3], QAM‑Abbildung [7.4, Bild 26/29/30], FAC/SDC/MSC‑Codierung [7.5], Zellinterleaver [7.6], MSC‑Zellzahlen [7.7, Tab. 41–45];
FAC [6.3], SDC [6.4], Multiplex [6.2.3], Audio [5.3 xHE‑AAC, 5.4 AAC], CRC [Annex D].

## 9. Wiedereinrasten

Bei Signalausfall (15 FAC‑Fehler in Folge, also ca. 6 s) fällt der Empfänger nach ACQ zurück und sucht Modus, Takt und Feinfrequenz neu; Zähler und die einmal erkannte
Spektrumbelegung bleiben erhalten. Geprüft mit einem synthetischen Signal: 8 s Rauschen mitten im Empfang und danach 20 Hz Frequenzsprung → Rückkehr nach TRACK, danach wieder
gültige Audiorahmen (Zustandsfolge ACQ → SYNC → TRACK → ACQ → SYNC → TRACK).
