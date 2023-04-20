/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-N2 Board Control file (AMLogic 64Bits Platform)
//
//
/*----------------------------------------------------------------------------*/
#include <dirent.h>
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
#include <sys/stat.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "odroidn2.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio_rev1[64] = {
			// wiringPi number to native gpio number
	479, 492,	// GPIOX.3			0	| 1	GPIOX.16(PWM_E)
	480, 483,	// GPIOX.4			2	| 3	GPIOX.7(PWM_F)
	476, 477,	// GPIOX.0			4	| 5	GPIOX.1
	478, 473,	// GPIOX.2			6	| 7	GPIOA.13
	493, 494,	// GPIOX.17(I2C-2_SDA)		8	| 9	GPIOX.18(I2C-2_SCL)
	486, 464,	// GPIOX.10			10	| 11	GPIOA.4
	484, 485,	// GPIOX.8			12	| 13	GPIOX.9
	487, 488,	// GPIOX.11			14	| 15	GPIOX.12
	489,  -1,	// GPIOX.13			16	| 17
	-1,  -1,	// 				18	| 19
	-1,  490,	// 				20	| 21	GPIOX.14
	491, 481,	// GPIOX.15			22	| 23	GPIOX.5(PWM_C)
	482, -1,	// GPIOX.6(PWM_D)		24	| 25	ADC.AIN3
	472, 495,	// GPIOA.12			26	| 27	GPIOX.19
	-1,  -1,	// REF1.8V OUT			28	| 29	ADC.AIN2
	474, 475,	// GPIOA.14(I2C-3_SDA)		30	| 31	GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio_rev1[64] = {
			// physical header pin number to native gpio number
	 -1,		// 			0
	 -1,  -1,	// 3.3V			1	| 2	5.0V
	493,  -1,	// GPIOX.17(I2C-2_SDA)	3	| 4	5.0V
	494,  -1,	// GPIOX.18(I2C-2_SCL)	5	| 6	GND
	473, 488,	// GPIOA.13		7	| 8	GPIOX.12(UART_TX_B)
	 -1, 489,	// GND			9	| 10	GPIOX.13(UART_RX_B)
	479, 492,	// GPIOX.3		11	| 12	GPIOX.16(PWM_E)
	480,  -1,	// GPIOX.4		13	| 14	GND
	483, 476,	// GPIOX.7(PWM_F)	15	| 16	GPIOX.0
	 -1, 477,	// 3.3V			17	| 18	GPIOX.1
	484,  -1,	// GPIOX.8(SPI_MOSI)	19	| 20	GND
	485, 478,	// GPIOX.9(SPI_MISO)	21	| 22	GPIOX.2
	487, 486,	// GPIOX.11(SPI_SCLK)	23	| 24	GPIOX.10(SPI_CE0)
	 -1, 464,	// GND			25	| 26	GPIOA.4(SPI_CE1)
	474, 475,	// GPIOA.14(I2C-3_SDA)	27	| 28	GPIOA.15(I2C-3_SCL)
	490,  -1,	// GPIOX.14		29	| 30	GND
	491, 472,	// GPIOX.15		31	| 32	GPIOA.12
	481,  -1,	// GPIOX.5(PWM_C)	33	| 34	GND
	482, 495,	// GPIOX.6(PWM_D)	35	| 36	GPIOX.19
	 -1,  -1,	// ADC.AIN3		37	| 38	1.8V REF OUT
	 -1,  -1,	// GND			39	| 40	ADC.AIN2
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const char *pinToPwm[64] = {
	// wiringPi number to pwm group number
		"None", "ffd19000", // GPIOX.3			0	| 1	GPIOX.16(PWM_E)
		"None", "ffd19000",		// GPIOX.4			2	| 3	GPIOX.7(PWM_F)
		"None", "None",			// GPIOX.0			4	| 5	GPIOX.1
		"None", "None",		// GPIOX.2			6	| 7	GPIOA.13
		"None", "None",			// GPIOX.17(I2C-2_SDA)		8	| 9	GPIOX.18(I2C-2_SCL)
		"None", "None",			// GPIOX.10			10	| 11	GPIOA.4
		"None", "None",			// GPIOX.8			12	| 13	GPIOX.9
		"None", "None",			// GPIOX.11			14	| 15	GPIOX.12
		"None", "None",			// GPIOX.13			16	| 17
		"None", "None",			// 				18	| 19
		"None", "None",			// 				20	| 21	GPIOX.14
		"None", "ffd1a000",     // GPIOX.15			22	| 23	GPIOX.5(PWM_C)
		"ffd1a000", "None",     // GPIOX.6(PWM_D)		24	| 25	ADC.AIN3
		"None", "None",         // GPIOA.12			26	| 27	GPIOX.19
		"None", "None",         // REF1.8V OUT			28	| 29	ADC.AIN2
		"None", "None",         // GPIOA.14(I2C-3_SDA)		30	| 31	GPIOA.15(I2C-3_SCL)
	// Padding:
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None", // 32...47
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None"  // 48...63
};

static const int pinToPwmNum[64] = {
	// wiringPi number to pwm pin number
	 -1,  2,	// GPIOX.3			0	| 1	GPIOX.16(PWM_E)
	 -1,  3,	// GPIOX.4			2	| 3	GPIOX.7(PWM_F)
	 -1, -1,	// GPIOX.0			4	| 5	GPIOX.1
	 -1, -1,	// GPIOX.2			6	| 7	GPIOA.13
	 -1, -1,	// GPIOX.17(I2C-2_SDA)		8	| 9	GPIOX.18(I2C-2_SCL)
	 -1, -1,	// GPIOX.10			10	| 11	GPIOA.4
	 -1, -1,	// GPIOX.8			12	| 13	GPIOX.9
	 -1, -1,	// GPIOX.11			14	| 15	GPIOX.12
	 -1, -1,	// GPIOX.13			16	| 17
	 -1, -1,	// 				18	| 19
	 -1, -1,	// 				20	| 21	GPIOX.14
	 -1,  0,	// GPIOX.15			22	| 23	GPIOX.5(PWM_C)
	  1, -1,	// GPIOX.6(PWM_D)		24	| 25	ADC.AIN3
	 -1, -1,	// GPIOA.12			26	| 27	GPIOX.19
	 -1, -1,	// REF1.8V OUT			28	| 29	ADC.AIN2
	 -1, -1,	// GPIOA.14(I2C-3_SDA)		30	| 31	GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1	// 48...63
};

static char pwmPinPath[10][(BLOCK_SIZE)] = {
	"","",
	"","",
	// Padding:
	"None","None",
	"None","None","None","None"
};

static char setupedPwmPinPath[10][BLOCK_SIZE] = {
	"None","None",
	"None","None",
	"None","None",
	"None","None",
	"None","None"
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
const int *pinToGpio, *phyToGpio;

/* ADC file descriptor */
static int adcFds[2];

/* GPIO mmap control */
static volatile uint32_t *gpio;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/* pwm sysnode */
static DIR *pwm;
static struct dirent *pwmchip;
/* pwm params */
static char sysPwmPath[(BLOCK_SIZE / 4)];
static char pwmExport[(BLOCK_SIZE / 16)];
static char pwmUnexport[(BLOCK_SIZE / 16)];
static char pwmPeriod[(BLOCK_SIZE / 16)];
static char pwmDuty[(BLOCK_SIZE / 16)];
static unsigned int pwmClock;
static unsigned int pwmRange;
/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);
static int	gpioToDSReg	(int pin);
static int	gpioToMuxReg	(int pin);
/*----------------------------------------------------------------------------*/
// Function of pwm define
/*----------------------------------------------------------------------------*/
static int	pinToSysPwmPath	(int pin);
static int	pwmSetup (int pin);
static int	pwmRelease (int pin);
/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_setDrive		(int pin, int value);
static int		_getDrive		(int pin);
static int		_pinMode		(int pin, int mode);
static int		_getAlt			(int pin);
static int		_getPUPD		(int pin);
static int		_pullUpDnControl	(int pin, int pud);
static int		_digitalRead		(int pin);
static int		_digitalWrite		(int pin, int value);
static int		_pwmWrite		(int pin, int value);
static int		_analogRead		(int pin);
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);
static void		_pwmSetRange		(unsigned int range);
static void		_pwmSetClock		(int divisor);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);
static 	void init_adc_fds	(void);

	void init_odroidn2 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPSETReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_OUTP_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_OUTP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_INP_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_PUEN_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return	N2_GPIOX_PUPD_REG_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  pin - N2_GPIOX_PIN_START;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  pin - N2_GPIOA_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if(pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_FSEL_REG_OFFSET;
	if(pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	if (pin >= N2_GPIOX_PIN_START && pin <= N2_GPIOX_PIN_MID)
		return  N2_GPIOX_DS_REG_2A_OFFSET;
	if (pin > N2_GPIOX_PIN_MID && pin <= N2_GPIOX_PIN_END)
		return  N2_GPIOX_DS_REG_2B_OFFSET;
	if (pin >= N2_GPIOA_PIN_START && pin <= N2_GPIOA_PIN_END)
		return  N2_GPIOA_DS_REG_5A_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pin Mux register
//
/*----------------------------------------------------------------------------*/
static int gpioToMuxReg (int pin)
{
	switch (pin) {
	case	N2_GPIOA_PIN_START	...N2_GPIOA_PIN_START + 7:
		return  N2_GPIOA_MUX_D_REG_OFFSET;
	case	N2_GPIOA_PIN_START + 8	...N2_GPIOA_PIN_END:
		return  N2_GPIOA_MUX_E_REG_OFFSET;
	case	N2_GPIOX_PIN_START	...N2_GPIOX_PIN_START + 7:
		return  N2_GPIOX_MUX_3_REG_OFFSET;
	case	N2_GPIOX_PIN_START + 8	...N2_GPIOX_PIN_START + 15:
		return  N2_GPIOX_MUX_4_REG_OFFSET;
	case	N2_GPIOX_PIN_START + 16	...N2_GPIOX_PIN_END:
		return  N2_GPIOX_MUX_5_REG_OFFSET;
	default:
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
//
// config pwm sys path. "/sys/class/pwm/pwmchip?"
//
/*----------------------------------------------------------------------------*/
static int pinToSysPwmPath (int pin)
{
	const char *pwmGroup;
	char pwmLinkSrc[(BLOCK_SIZE / 8)];
	char pwmPath[(BLOCK_SIZE / 8)];
	int sz_link;

	memset(pwmLinkSrc, 0, sizeof(pwmLinkSrc));
	memset(pwmPath, 0, sizeof(pwmPath));

	pwmGroup = pinToPwm[pin];
	pwm = opendir("/sys/class/pwm");
	if (pwm == NULL) {
		printf("need to set device: pwm\n");
		return -1;
	}

	while (1) {
		pwmchip = readdir(pwm);

		if (pwmchip == NULL) {
			break;
		}

		if (strlen(pwmchip->d_name) <= 2)
			continue;

		sprintf(pwmPath, "%s/%s", "/sys/class/pwm", pwmchip->d_name);
		sz_link = readlink(pwmPath, pwmLinkSrc, sizeof(pwmLinkSrc));
		if (sz_link < 0) {
			perror("Read symbolic link fail");
			return sz_link;
		}

		if (strstr(pwmLinkSrc, pwmGroup) != NULL) {
			strncpy(sysPwmPath, pwmPath, (sizeof(sysPwmPath) - 1));
			break;
		}
	}
	closedir(pwm);

	return 0;
}

static int pwmSetup (int pin) {
	char cmd[(BLOCK_SIZE * 2)];
	int pwmPin, ret;

	memset(cmd, 0, sizeof(cmd));
	memset(pwmExport, 0, sizeof(pwmExport));

	if ((ret = pinToSysPwmPath(pin)) < 0) {
		perror("set pwm dtb overlays");
		return ret;
	}

	if (strstr(sysPwmPath, "pwmchip") == NULL) {
		printf("config pwm dtb overlays\n");
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	pwmClock = N2_PWM_INTERNAL_CLK;
	sprintf(pwmExport, "%d", (pwmPin % 2));
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, (pwmPin % 2));
	strncpy(setupedPwmPinPath[pwmPin], pwmPinPath[pwmPin], (BLOCK_SIZE - 1));
#ifdef ANDROID
	sprintf(cmd, "su -s sh -c %s %s", SYS_ACCESS_SCRIPT, pwmPinPath[pwmPin]);
#else
	sprintf(cmd, "sudo sh %s %s", SYS_ACCESS_SCRIPT, pwmPinPath[pwmPin]);
#endif
	inputToSysNode(sysPwmPath, "export", pwmExport);
	system(cmd);
	printf("PWM/pin%d: Don't change to gpio mode with overlay registered.\n", pin);

	return 0;
}

static int pwmRelease (int pin) {
	int pwmPin, ret;

	if ((ret = pinToSysPwmPath(pin)) < 0) {
		return ret;
	}

	if (strstr(sysPwmPath, "pwmchip") == NULL) {
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	sprintf(pwmUnexport, "%d", (pwmPin % 2));
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, (pwmPin % 2));
	if ((pwm = opendir(pwmPinPath[pwmPin])) != NULL) {
		inputToSysNode(pwmPinPath[pwmPin], "enable", "0");
		inputToSysNode(sysPwmPath, "unexport", pwmUnexport);
		closedir(pwm);
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		retPin = pin;
		break;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		retPin = lib->sysFds[pin] != -1 ? pin : -1;
		break;
	/* wiringPi number */
	case	MODE_PINS:
		retPin = pin < 64 ? pinToGpio[pin] : -1;
		break;
	/* header pin number */
	case	MODE_PHYS:
		retPin = pin < 64 ? phyToGpio[pin] : -1;
		break;
	default	:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return retPin;
}

/*----------------------------------------------------------------------------*/
static int _setDrive (int pin, int value)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	shift = pin > N2_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2 ;

	*(gpio + ds) &= ~(0b11 << shift);
	*(gpio + ds) |= (value << shift);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	shift = pin > N2_GPIOX_PIN_MID ? (shift - 16) * 2 : shift * 2 ;

	return (*(gpio + ds) >> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	int fsel, mux, target, shift, origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	pwmRelease (origPin);
	softPwmStop  (origPin);
	softToneStop (origPin);

	fsel	= gpioToGPFSELReg(pin);
	mux	= gpioToMuxReg(pin);
	shift	= gpioToShiftReg (pin);
	target	= shift * 4;

	switch (mode) {
	case	INPUT:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case	OUTPUT:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) & ~(1 << shift));
		break;
	case 	INPUT_PULLUP:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case 	INPUT_PULLDOWN:
		*(gpio + mux) = (*(gpio + mux) & ~(0xF << target));
		*(gpio + fsel) = (*(gpio + fsel) |  (1 << shift));
		_pullUpDnControl(origPin, PUD_DOWN);
		break;
	case	SOFT_PWM_OUTPUT:
		softPwmCreate (origPin, 0, 100);
		break;
	case	SOFT_TONE_OUTPUT:
		softToneCreate (origPin);
		break;
	case	PWM_OUTPUT:
		pwmSetup(origPin);
		break;
	default:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getAlt (int pin)
{
	int fsel, mux, shift, target, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel   = gpioToGPFSELReg(pin);
	mux    = gpioToMuxReg(pin);
	target = shift = gpioToShiftReg(pin);

	while (target >= 8) {
		target -= 8;
	}

	mode = (*(gpio + mux) >> (target * 4)) & 0xF;
	return	mode ? mode + 1 : (*(gpio + fsel) & (1 << shift)) ? 0 : 1;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	puen  = gpioToPUENReg(pin);
	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin);

	if (*(gpio + puen) & (1 << shift))
		return *(gpio + pupd) & (1 << shift) ? 1 : 2;
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	int shift = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	shift = gpioToShiftReg(pin);

	if (pud) {
		// Enable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) | (1 << shift));

		if (pud == PUD_UP)
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) |  (1 << shift));
		else
			*(gpio + gpioToPUPDReg(pin)) =
				(*(gpio + gpioToPUPDReg(pin)) & ~(1 << shift));
	} else	// Disable Pull/Pull-down resister
		*(gpio + gpioToPUENReg(pin)) =
			(*(gpio + gpioToPUENReg(pin)) & ~(1 << shift));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalRead (int pin)
{
	char c ;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek	(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return	(c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	if ((*(gpio + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
		return HIGH ;
	else
		return LOW ;
}

/*----------------------------------------------------------------------------*/
static int _digitalWrite (int pin, int value)
{
	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write(lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			} else {
				if (write(lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value == LOW)
		*(gpio + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
	else
		*(gpio + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));

	return 0;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
/*----------------------------------------------------------------------------*/
static int _pwmWrite (int pin, int value)
{
	unsigned int duty;
	int pwmPin;

	memset(pwmDuty, 0, sizeof(pwmDuty));

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if (((unsigned int)value > pwmRange) || (pwmRange <= 0)) {
		printf("warn : pwm range value is greater than or equal pwmWrite's\n");
		return -1;
	}

	pwmPin = pinToPwmNum[pin];
	duty = ((value * 100) / pwmRange);
	sprintf(pwmDuty, "%d", ((atoi(pwmPeriod) * duty) / 100));

	inputToSysNode(pwmPinPath[pwmPin], "duty_cycle", pwmDuty);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int _analogRead (int pin)
{
	char value[5] = {0,};

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	/* wiringPi ADC number = pin 25, pin 29 */
	switch (pin) {
#if defined(ARDUINO)
	/* To work with physical analog channel numbering */
	case	3:	case	25:
		pin = 0;
	break;
	case	2:	case	29:
		pin = 1;
	break;
#else
	case	0:	case	25:
		pin = 0;
	break;
	case	1:	case	29:
		pin = 1;
	break;
#endif
	default:
		return	0;
	}
	if (adcFds [pin] == -1)
		return 0;

	lseek(adcFds [pin], 0L, SEEK_SET);
	if (read(adcFds [pin], &value[0], 4) < 0) {
		msg(MSG_WARN, "%s: Error occurs when it reads from ADC file descriptor. \n", __func__);
		return -1;
	}

	return	atoi(value);
}

/*----------------------------------------------------------------------------*/
static int _digitalWriteByte (const unsigned int value)
{
	union	reg_bitfield	gpiox;
	union	reg_bitfield	gpioa;

	gpiox.wvalue = *(gpio + N2_GPIOX_INP_REG_OFFSET);
	gpioa.wvalue = *(gpio + N2_GPIOA_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = N2 GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = N2 GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = N2 GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x04);
	/* Wiring PI GPIO3 = N2 GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x08);
	/* Wiring PI GPIO4 = N2 GPIOX.0 */
	gpiox.bits.bit0 = (value & 0x10);
	/* Wiring PI GPIO5 = N2 GPIOX.1 */
	gpiox.bits.bit1 = (value & 0x20);
	/* Wiring PI GPIO6 = N2 GPIOX.2 */
	gpiox.bits.bit2 = (value & 0x40);
	/* Wiring PI GPIO7 = N2 GPIOA.13 */
	gpioa.bits.bit13 = (value & 0x80);

	*(gpio + N2_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;
	*(gpio + N2_GPIOA_OUTP_REG_OFFSET) = gpioa.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
// PWM signal ___-----------___________---------------_______-----_
//               <--value-->           <----value---->
//               <-------range--------><-------range-------->
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetRange (unsigned int range)
{
	unsigned int freq, period;

	memset(pwmPeriod, 0, sizeof(pwmPeriod));

	if (lib->mode == MODE_GPIO_SYS)
		return;

	if (pwmClock < 2) {
		printf("error : pwm freq: %dMHz / (pwmSetClock's value) >= 2\n",
				(N2_PWM_INTERNAL_CLK / 1000000));
		return;
	}

	pwmRange = range;
	if ((pwmRange < 1) || (pwmRange >= pwmClock)) {
		printf("error : invalied value. ( < pwm freq)\n");
		return;
	}

	freq = (pwmClock / pwmRange);
	period = (1000000000 / freq); // period: s to ns.
	sprintf(pwmPeriod, "%d", period);

	for (int i = 0; i < 10; i++) {
		if (strstr(setupedPwmPinPath[i], "None") != NULL)
			continue;
		inputToSysNode(setupedPwmPinPath[i], "period", pwmPeriod);
		inputToSysNode(setupedPwmPinPath[i], "polarity", "normal");
		inputToSysNode(setupedPwmPinPath[i], "enable", "1");
	}
}

/*----------------------------------------------------------------------------*/
// Internal clock == 24MHz
// PWM clock == (Internal clock) / divisor
// PWM frequency == (PWM clock) / range
/*----------------------------------------------------------------------------*/
static void _pwmSetClock (int divisor)
{
	if (pwmClock > 0)
		pwmClock = (pwmClock / divisor);
	else {
		printf("error : pwm mode error\n");
		return;
	}
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	return	-1;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped;

	/* GPIO mmap setup */
	if (!getuid()) {
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg(MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg(MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));
			setUsingGpiomem(TRUE);
		} else
			msg(MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		//#define N2_GPIO_BASE	0xff634000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N2_GPIO_BASE);
#else
		mapped = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)N2_GPIO_BASE);
#endif
#else
		mapped = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, N2_GPIO_BASE);
#endif

		if (mapped == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped;
	}
}

/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
	const char *AIN0_NODE, *AIN1_NODE;

	/* ADC node setup */
	if (cmpKernelVersion(KERN_NUM_TO_MAJOR, 5)) {
		AIN0_NODE = "/sys/devices/platform/soc/ff800000.bus/ff809000.adc/iio:device0/in_voltage2_raw";
		AIN1_NODE = "/sys/devices/platform/soc/ff800000.bus/ff809000.adc/iio:device0/in_voltage3_raw";
	}

	else {
		AIN0_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage2_raw";
		AIN1_NODE = "/sys/devices/platform/ff809000.saradc/iio:device0/in_voltage3_raw";
	}

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}

/*----------------------------------------------------------------------------*/
void init_odroidn2 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();

	pinToGpio = pinToGpio_rev1;
	phyToGpio = phyToGpio_rev1;

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->setDrive		= _setDrive;
	libwiring->getDrive		= _getDrive;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->pwmWrite		= _pwmWrite;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;

	/* specify pin base number */
	libwiring->pinBase		= N2_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
