COMMON_CFLAGS=$(EXTRA_CFLAGS) $(LIBWIRINGPI_CFLAGS)
LIBS=$(LIBWIRINGPI_LIBS)

bin_PROGRAMS = gertboard buttons 7segments voltmeter temperature vumeter record

gertboard_SOURCES = gertboard.c
gertboard_CFLAGS = $(COMMON_CFLAGS)
gertboard_LDADD = -lm

buttons_SOURCES = buttons.c
buttons_CFLAGS = $(COMMON_CFLAGS)

7segments_SOURCES = 7segments.c
7segments_CFLAGS = $(COMMON_CFLAGS)

voltmeter_SOURCES = voltmeter.c
voltmeter_CFLAGS = $(COMMON_CFLAGS)

temperature_SOURCES = temperature.c
temperature_CFLAGS = $(COMMON_CFLAGS)

vumeter_SOURCES = vumeter.c
vumeter_CFLAGS = $(COMMON_CFLAGS)

record_SOURCES = record.c
record_CFLAGS = $(COMMON_CFLAGS)
