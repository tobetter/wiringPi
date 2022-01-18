COMMON_CFLAGS = $(EXTRA_CFLAGS) $(LIBWIRINGPI_CFLAGS) -I.
LIBS = $(LIBWIRINGPI_LIBS)

bin_PROGRAMS = blink buttons ladder metro motor reaction

blink_SOURCES = blink.c
blink_CFLAGS = $(COMMON_CFLAGS)

buttons_SOURCES = buttons.c
buttons_CFLAGS = $(COMMON_CFLAGS)

ladder_SOURCES = ladder.c
ladder_CFLAGS = $(COMMON_CFLAGS)

metro_SOURCES = metro.c
metro_CFLAGS = $(COMMON_CFLAGS)

motor_SOURCES = motor.c
motor_CFLAGS = $(COMMON_CFLAGS)

reaction_SOURCES = reaction.c
reaction_CFLAGS = $(COMMON_CFLAGS)
