AC_PREREQ([2.69])
AC_INIT([odroid-wiringpi], [3.0], [tobetter@gmail.com])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])

CURRENT=2
REVISION=0
AGE=0
AC_SUBST([LIBWIRINGPI_VERSION], [$CURRENT:$REVISION:$AGE])

AC_SUBST([EXTRA_CFLAGS], "-Wformat=2 -Wall -Wextra -Winline -Wno-format-truncation -pipe -fPIC")

pkgconfigdir=${libdir}/pkgconfig
AC_SUBST(pkgconfigdir)

# Checks for programs.
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_RANLIB

# Checks for libraries.
PKG_CHECK_MODULES([LIBCRYPT], [libcrypt])

# Checks for header files.
AC_CHECK_HEADERS([arpa/inet.h fcntl.h malloc.h netdb.h netinet/in.h stdint.h stdlib.h string.h sys/ioctl.h sys/socket.h sys/time.h syslog.h termios.h unistd.h])

AM_INIT_AUTOMAKE([foreign])
#AM_MAINTAINER_MODE

AC_CONFIG_MACRO_DIRS([m4])
AC_PROG_LIBTOOL

# Checks for typedefs, structures, and compiler characteristics.
AC_CHECK_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT8_T
AC_TYPE_PID_T
AC_TYPE_UID_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_CHOWN
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_FUNC_MMAP
AC_CHECK_FUNCS([clock_gettime floor ftruncate gettimeofday memset pow rint socket strcasecmp strchr strerror strstr strtol strtoul uname])

AC_CONFIG_FILES([
		Makefile
		devLib/Makefile
		gpio/Makefile
		libwiringpi2.pc
		pins/Makefile
		wiringPi/Makefile
		wiringPiD/Makefile])
AC_OUTPUT
