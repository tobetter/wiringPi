lib_LTLIBRARIES = libwiringPiDev.la

libwiringPiDev_la_SOURCES = \
	ds1302.c \
	gertboard.c \
	lcd.c \
	lcd128x64.c \
	maxdetect.c \
	piFace.c \
	piGlow.c \
	piNes.c \
	scrollPhat.c

libwiringPiDev_la_CFLAGS = \
	-I ../wiringPi \
	@EXTRA_CFLAGS@

libwiringPiDev_la_LDFLAGS = \
	-version-info @LIBWIRINGPI_VERSION@ \
	-lpthread $(LIBCRYPT_LIBS)
