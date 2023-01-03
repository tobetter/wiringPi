#!/bin/sh

# /etc/odroid-wpi-pwm.sh
# written by Steve Jeong <how2soft@gmail.com>
#
# allow access pwm sys node (with odroid-wiringpi gpiomem).
# udev rules: 99-odroid-wiringpi-pwm.rules
# param: "/sys/class/pwm/pwmchip*" default.

cutoff=0

while [ ! -d "$1" ]; do
  cutoff=$(expr ${cutoff} + 1)
  [ ${cutoff} -gt 5 ] && break
  sleep .1
done

chmod -R ugo+rw $1
