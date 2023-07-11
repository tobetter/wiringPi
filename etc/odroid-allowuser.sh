#!/bin/sh

# /etc/odroid-allowuser.sh
# written by Steve Jeong <steve@how2flow.net>
#
# allow access sys node (with odroid-wiringpi gpiomem).
# udev rules: 99-odroid-wiringpi-*.rules

cutoff=0

while [ ! -d "$1" ]; do
  cutoff=$(expr ${cutoff} + 1)
  [ ${cutoff} -gt 5 ] && break
  sleep .1
done

chmod -R ugo+rw $1
