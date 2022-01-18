#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([wiringpi-examples], [1.0], [tobetter@gmail.com])
AC_CONFIG_SRCDIR([serialRead.c])
AC_CONFIG_HEADERS([config.h])

AC_SUBST([EXTRA_CFLAGS], ["-Wformat=2 -Wall -Wextra -Winline -Wno-format-truncation -pipe -fPIC"])

# Checks for programs.
AC_PROG_CC

# Checks for libraries.
PKG_CHECK_MODULES([LIBCRYPT], [libcrypt])
PKG_CHECK_MODULES([LIBWIRINGPI], [libwiringpi2])

AM_INIT_AUTOMAKE([foreign])
AC_CONFIG_MACRO_DIRS([m4])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/ioctl.h sys/time.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_FUNC_MALLOC
AC_CHECK_FUNCS([floor gettimeofday rint strcasecmp strerror])

AC_CONFIG_FILES([
	Makefile
	Gertboard/Makefile
	PiFace/Makefile
	PiGlow/Makefile
	q2w/Makefile
	scrollPhat/Makefile
	])

AC_OUTPUT
