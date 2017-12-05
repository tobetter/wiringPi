/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-??? Board Control file (??? Platform)
//
//
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>
#include <sys/utsname.h>

/*----------------------------------------------------------------------------*/
#include "wiringOdroid.h"
#include "odroid???.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	???, ???,	//  0 |  1 :
	???, -1,	//  2 |  3 :
	???, ???,	//  4 |  5 :
	???, ???,	//  6 |  7 :
	-1,  -1,	//  8 |  9 :
	???, ???,	// 10 | 11 :
	???, ???,	// 12 | 13 :
	???, -1,	// 14 | 15 :
	-1,  -1,	// 16 | 17 :
	-1,  -1,	// 18 | 19 :
	-1,  ???,	// 20 | 21 :
	???, ???,	// 22 | 23 :
	???, -1,	// 24 | 25 :
	???, ???,	// 26 | 27 :
	-1,  -1,	// 28 | 29 :
	-1,  -1,	// 30 | 31 :
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 :
	-1,  -1,	//  3 |  4 :
	-1,  -1,	//  5 |  6 :
	???, -1,	//  7 |  8 :
	-1,  -1,	//  9 | 10 :
	???,???,	// 11 | 12 :
	???, -1,	// 13 | 14 :
	-1, ???,	// 15 | 16 :
	-1, ???,	// 17 | 18 :
	???, -1,	// 19 | 20 :
	???,???,	// 21 | 22 :
	???,???,	// 23 | 24 :
	-1, ???,	// 25 | 26 :
	-1,  -1,	// 27 | 28 :
	???, -1,	// 29 | 30 :
	???,???,	// 31 | 32 :
	???, -1,	// 33 | 34 :
	???,???,	// 35 | 36 :
	-1,  -1,	// 37 | 38 :
	-1,  -1,	// 39 | 40 :
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
/* ADC file descriptor */
static char *adcFds[2];

/* GPIO mmap control */
static volatile uint32_t *gpio;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);

/*----------------------------------------------------------------------------*/
// wiringPi core function 
/*----------------------------------------------------------------------------*/
static int		getModeToGpio	(int mode, int pin);
static void		pinMode		(int pin, int mode);
static int		getAlt		(int pin);
static void		pullUpDnControl	(int pin, int pud);
static int		digitalRead	(int pin);
static void		digitalWrite	(int pin, int value);
static int		analogRead	(int pin);
static void		digitalWriteByte(const int value);
static unsigned int	digitalReadByte	(void);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);
static 	void init_adc_fds	(void);

	void init_odroid??? 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	return	-1;
}
/*----------------------------------------------------------------------------*/
static int getModeToGpio (int mode, int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
static void pinMode (int pin, int mode)
{
}

/*----------------------------------------------------------------------------*/
static int getAlt (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
static void pullUpDnControl (int pin, int pud)
{
}

/*----------------------------------------------------------------------------*/
static int digitalRead (int pin)
{
}

/*----------------------------------------------------------------------------*/
static void digitalWrite (int pin, int value)
{
}

/*----------------------------------------------------------------------------*/
static int analogRead (int pin)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
static void digitalWriteByte (const int value)
{
}

/*----------------------------------------------------------------------------*/
static unsigned int digitalReadByte (void)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
}

/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
}

/*----------------------------------------------------------------------------*/
void init_odroid??? (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= getModeToGpio;
	libwiring->pinMode		= pinMode;
	libwiring->getAlt		= getAlt;
	libwiring->pullUpDnControl	= pullUpDnControl;
	libwiring->digitalRead		= digitalRead;
	libwiring->digitalWrite		= digitalWrite;
	libwiring->analogRead		= analogRead;
	libwiring->digitalWriteByte	= digitalWriteByte;
	libwiring->digitalReadByte	= digitalReadByte;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
