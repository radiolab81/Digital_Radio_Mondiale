#!/bin/sh
# Ende-zu-Ende-Selbsttest ohne Aufnahmen: synthetischer Sender (drm_gen) -> Empfaenger (drm_test).
# Prueft Robustheitsmodi A-D, Belegungen, 16-/64-QAM, EEP/UEP und beide Audioformate.  Aufruf aus dem Projektordner: sh tests/run_loopback.sh
set -e
make -s drm_test drm_gen 2>/dev/null || true
CFG="
A 3 16 0 0 0 xhe 14 30 -17 2
A 2 16 1 1 0 aac 14 30 0 3
B 2 64 0 0 0 xhe 14 35 8 4
B 3 64 1 3 500 xhe 14 35 0 5
C 3 16 0 0 0 xhe 14 30 0 6
D 3 16 0 0 0 aac 14 30 0 7
A 3 64 0 2 600 aac 14 35 0 8
"
echo "$CFG" | while read m o q a b x c n s f seed; do
  [ -z "$m" ] && continue
  ./drm_gen /tmp/loop.wav $m $o $q $a $b $x $c $n $s $f $seed > /tmp/loop.txt
  ./drm_test /tmp/loop.wav /tmp/loop.bin > /tmp/loop.out
  ok=$(grep -c "0 CRC-Fehler, 0 Superframes verloren" /tmp/loop.out || true)
  printf "Modus %s occ %s %s-QAM PL_A=%s PL_B=%s X=%-4s %-3s  " $m $o $q $a $b $x $c
  if [ "$ok" = "1" ]; then echo "OK"; else echo "FEHLER"; cat /tmp/loop.out; exit 1; fi
done
echo "alle Loopback-Tests bestanden"
