#
# Makefile:
#	wiringPi - Wiring Compatable library for the Raspberry Pi
#
#	Copyright (c) 2012-2015 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	https://projects.drogon.net/raspberry-pi/wiringpi/
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################

VERSION=$(shell cat ../VERSION)
DESTDIR?=/usr
PREFIX?=/local

MAJOR=$(shell cat ../VERSION | cut -d'.' -f1)

LDCONFIG?=ldconfig

ifneq ($V,1)
Q ?= @
endif

STATIC=libwiringPi.a
DYNAMIC=libwiringPi.so.$(VERSION)

#DEBUG	= -g -O0
DEBUG	= -O2
CC	= gcc
INCLUDE	= -I.
DEFS	= -D_GNU_SOURCE
CFLAGS	= $(DEBUG) $(DEFS) -Wformat=2 -Wall -Wextra -Winline $(INCLUDE) -pipe -fPIC

LIBS    = -lm -lpthread -lrt -lcrypt

###############################################################################

SRC	=	wiringPi.c						\
		wiringSerial.c wiringShift.c				\
		wiringPiSPI.c wiringPiI2C.c				\
		piHiPri.c piThread.c					\
		softPwm.c softTone.c softServo.c			\
		odroidc1.c						\
		odroidc2.c						\
		odroidxu3.c						\
		odroidn1.c						\
		odroidn2.c						\
		odroidc4.c

HEADERS =	$(shell ls *.h)

OBJ	=	$(SRC:.c=.o)

all:		$(DYNAMIC)

static:		$(STATIC)

$(STATIC):	$(OBJ)
	$Q echo "[Link (Static)]"
	$Q ar rcs $(STATIC) $(OBJ)
	$Q ranlib $(STATIC)
#	@size   $(STATIC)

$(DYNAMIC):	$(OBJ)
	$Q echo "[Link (Dynamic)]"
	$Q $(CC) -shared -Wl,-soname,libwiringPi.so$(WIRINGPI_SONAME_SUFFIX) -o libwiringPi.so.$(VERSION) $(LIBS) $(OBJ)

.c.o:
	$Q echo [Compile] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@


.PHONY:	clean
clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) $(OBJ_I2C) *~ core tags Makefile.bak libwiringPi.*

.PHONY:	tags
tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)


.PHONY:	install
install:	$(DYNAMIC)
	$Q echo "[Install Headers]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/include
	$Q install -m 0644 $(HEADERS)					$(DESTDIR)$(PREFIX)/include
	$Q echo "[Install Dynamic Lib]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/lib
	$Q install -m 0755 libwiringPi.so.$(VERSION)			$(DESTDIR)$(PREFIX)/lib/libwiringPi.so.$(VERSION)
	$Q ln -sf libwiringPi.so.$(VERSION)				$(DESTDIR)/lib/libwiringPi.so.$(MAJOR)
	$Q ln -sf libwiringPi.so.$(MAJOR)				$(DESTDIR)/lib/libwiringPi.so
	$Q $(LDCONFIG)

.PHONY:	install-static
install-static:	$(STATIC)
	$Q echo "[Install Headers]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/include
	$Q install -m 0644 $(HEADERS)					$(DESTDIR)$(PREFIX)/include
	$Q echo "[Install Static Lib]"
	$Q install -m 0755 -d						$(DESTDIR)$(PREFIX)/lib
	$Q install -m 0755 libwiringPi.a				$(DESTDIR)$(PREFIX)/lib

.PHONY:	uninstall
uninstall:
	$Q echo "[UnInstall]"
	$Q cd $(DESTDIR)$(PREFIX)/include/ && rm -f $(HEADERS) wiringOdroid*.h
	$Q cd $(DESTDIR)$(PREFIX)/lib/     && rm -f libwiringPi.*
	$Q $(LDCONFIG)


.PHONY:	depend
depend:
	makedepend -Y $(SRC) $(SRC_I2C)

# DO NOT DELETE
wiringPi.o: softPwm.h softTone.h wiringPi.h ../version.h
wiringSerial.o: wiringSerial.h
wiringShift.o: wiringPi.h wiringShift.h
wiringPiSPI.o: wiringPi.h wiringPiSPI.h
wiringPiI2C.o: wiringPi.h wiringPiI2C.h
piHiPri.o: wiringPi.h
piThread.o: wiringPi.h
softPwm.o: wiringPi.h softPwm.h
softTone.o: wiringPi.h softTone.h
softServo.o: wiringPi.h softServo.h

# ODROID Boards
odroidc1.o  : wiringPi.h odroidc1.h
odroidc2.o  : wiringPi.h odroidc2.h
odroidxu3.o : wiringPi.h odroidxu3.h
odroidn1.o  : wiringPi.h odroidn1.h
odroidn2.o  : wiringPi.h odroidn2.h
odroidc4.o  : wiringPi.h odroidc4.h
