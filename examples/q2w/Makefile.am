COMMON_CFLAGS=$(EXTRA_CFLAGS) $(LIBWIRINGPI_CFLAGS)
LIBS=$(LIBWIRINGPI_LIBS)

bin_PROGRAMS = blink button blink-io volts bright

blink_SOURCES = blink.c
blink_CFLAGS = $(COMMON_CFLAGS)
blink_LDADD = -lm

button_SOURCES = button.c
button_CFLAGS = $(COMMON_CFLAGS)

blink_io_SOURCES = blink-io.c
blink_io_CFLAGS = $(COMMON_CFLAGS)

volts_SOURCES = volts.c
volts_CFLAGS = $(COMMON_CFLAGS)

bright_SOURCES = bright.c
bright_CFLAGS = $(COMMON_CFLAGS)
