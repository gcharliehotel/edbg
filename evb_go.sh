#!/bin/sh

set -o errexit
set -o xtrace

# Prefix with ~ to invert.
SWDIO_GPIO=8
SWCLK_GPIO=123
NRESET_GPIO=9

for n in ${SWDIO_GPIO} ${SWCLK_GPIO} ${NRESET_GPIO}; do
  if [ \! -e /sys/class/gpio/gpio${n} ]; then
      echo -n $n > /sys/class/gpio/export
  fi
done

./edbg -b -s ${SWDIO_GPIO} -S ${SWCLK_GPIO} -n ${NRESET_GPIO} -t atmel_cm0p "$@"
