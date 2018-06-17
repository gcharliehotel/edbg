#!/bin/sh

set -o errexit
set -o xtrace

for n in 8 9 123; do
  if [ \! -e /sys/class/gpio/gpio${n} ]; then
      echo -n $n > /sys/class/gpio/export
  fi
done

make && ./edbg -b -s 8 -S 123 -n 9 -t atmel_cm0p "$@"
