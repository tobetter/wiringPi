ACLOCAL_AMFLAGS = -I m4 ${ACLOCAL_FLAGS}

COMMON_CFLAGS = $(EXTRA_CFLAGS) $(LIBWIRINGPI_CFLAGS)
LIBS = $(LIBWIRINGPI_LIBS)

bin_PROGRAMS = blink blink12 blink12drcs blink8 clock delayTest ds1302 isr isr-osc lcd lcd-adafruit max31855 nes okLed pwm rht03 serialRead serialTest softPwm softTone speed wfi

blink_SOURCES = blink.c
blink_CFLAGS = $(COMMON_CFLAGS)

blink12_SOURCES = blink12.c
blink12_CFLAGS = $(COMMON_CFLAGS)

blink12drcs_SOURCES = blink12drcs.c
blink12drcs_CFLAGS = $(COMMON_CFLAGS)

blink8_SOURCES = blink8.c
blink8_CFLAGS = $(COMMON_CFLAGS)

clock_SOURCES = clock.c
clock_CFLAGS = $(COMMON_CFLAGS)
clock_LDADD = -lm

delayTest_SOURCES = delayTest.c
delayTest_CFLAGS = $(COMMON_CFLAGS)

ds1302_SOURCES = ds1302.c
ds1302_CFLAGS = $(COMMON_CFLAGS)

isr_SOURCES = isr.c
isr_CFLAGS = $(COMMON_CFLAGS)

isr_osc_SOURCES = isr-osc.c
isr_osc_CFLAGS = $(COMMON_CFLAGS)

lcd_SOURCES = lcd.c
lcd_CFLAGS = $(COMMON_CFLAGS)

lcd_adafruit_SOURCES = lcd-adafruit.c
lcd_adafruit_CFLAGS = $(COMMON_CFLAGS)

max31855_SOURCES = max31855.c
max31855_CFLAGS = $(COMMON_CFLAGS)

nes_SOURCES = nes.c
nes_CFLAGS = $(COMMON_CFLAGS)

okLed_SOURCES = okLed.c
okLed_CFLAGS = $(COMMON_CFLAGS)

pwm_SOURCES = pwm.c
pwm_CFLAGS = $(COMMON_CFLAGS)

rht03_SOURCES = rht03.c
rht03_CFLAGS = $(COMMON_CFLAGS)

serialRead_SOURCES = serialRead.c
serialRead_CFLAGS = $(COMMON_CFLAGS)

serialTest_SOURCES = serialTest.c
serialTest_CFLAGS = $(COMMON_CFLAGS)

softPwm_SOURCES = softPwm.c
softPwm_CFLAGS = $(COMMON_CFLAGS)

softTone_SOURCES = softTone.c
softTone_CFLAGS = $(COMMON_CFLAGS)

speed_SOURCES = speed.c
speed_CFLAGS = $(COMMON_CFLAGS)

wfi_SOURCES = wfi.c
wfi_CFLAGS = $(COMMON_CFLAGS)

SUBDIRS = Gertboard PiFace PiGlow q2w scrollPhat
