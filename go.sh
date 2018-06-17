#!/bin/sh

set -o errexit
set -o xtrace

# DIO -s - Yellow - Pin 5 - GPIO 3
# CLK -S - Green - Pin 3 - GPIO 2
# nRESET -n - Red - Pin 7 - GPIO 4

for n in 2 3 4; do
  if [ \! -e /sys/class/gpio/gpio${n} ]; then
      echo -n $n > /sys/class/gpio/export
  fi
done

make && ./edbg -b -s 3 -S 2 -n 4 -t atmel_cm0p "$@"
