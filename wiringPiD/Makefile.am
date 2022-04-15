sbin_PROGRAMS = wiringPiD

wiringPiD_SOURCES = \
	daemonise.c \
	network.c \
	runRemote.c \
	wiringpid.c

wiringPiD_CFLAGS = \
	-I ../wiringPi \
	@EXTRA_CFLAGS@ \
	$(LIBCRYPT_CFLAGS)

wiringPiD_LDFLAGS = \
	-L../wiringPi/.libs -lwiringPi -lgpiod \
	$(LIBCRYPT_LIBS)
