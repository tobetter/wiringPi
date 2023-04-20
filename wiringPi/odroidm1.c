/*----------------------------------------------------------------------------*/
//
//
//	WiringPi ODROID-M1 Board Control file (ROCKCHIP 64Bits Platform)
//
//
/*----------------------------------------------------------------------------*/
/*******************************************************************************
Copyright (C) 2022 Steve Jeong <how2soft@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/
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
#include "odroidm1.h"

#if !defined(DEVMEM)
#include <gpiod.h>
/*----------------------------------------------------------------------------*/
// libgpiod define
/*----------------------------------------------------------------------------*/
static struct gpiod_chip *chip;
static struct gpiod_line *gpiod;
#endif

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	16, 120,	//  0 |  1 : GPIO0_C0, GPIO3_D0
	17, 106,	//  2 |  3 : GPIO0_C1, GPIO3_B2
	118,119,	//  4 |  5 : GPIO3_C6, GPIO3_C7
	121,14, 	//  6 |  7 : GPIO3_D1, GPIO0_B6
	110,109,	//  8 |  9 : GPIO3_B6, GPIO3_B5
	90, 122,	// 10 | 11 : GPIO2_D2, GPIO3_D2
	89,  88,	// 12 | 13 : GPIO2_D1, GPIO2_D0
	91, 126,	// 14 | 15 : GPIO2_D3, GPIO3_D6
	127, -1,	// 16 | 17 : GPIO3_D7
	-1,  -1,	// 18 | 19 :
	-1, 145,	// 20 | 21 : , GPIO4_C1
	142, 13,	// 22 | 23 : GPIO4_B6, GPIO0_B5
	125, -1,	// 24 | 25 : GPIO3_D5,
	123,124,	// 26 | 27 : GPIO3_D3, GPIO3_D4
	-1,  -1,	// 28 | 29 :
	 12, 11,	// 30 | 31 : GPIO0_B4, GPIO0_B3

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	-1,		//  0
	-1,  -1,	//  1 |  2 : 3.3V, 5.0V
	110, -1,	//  3 |  4 : GPIO3_B6, 5.0V
	109, -1,	//  5 |  6 : GPIO3_B5, GND
	14, 126,	//  7 |  8 : GPIO0_B6, GPIO3_D6
	-1, 127,	//  9 | 10 : GND, GPIO3_D7
	16 ,120,	// 11 | 12 : GPIO0_C0, GPIO3_D0
	17 , -1,	// 13 | 14 : GPIO0_C1, GND
	106,118,	// 15 | 16 : GPIO3_B2, GPIO3_C6
	-1, 119,	// 17 | 18 : 3.3V, GPIO3_C7
	89,  -1,	// 19 | 20 : GPIO2_D1, GND
	88, 121,	// 21 | 22 : GPIO2_D0, GPIO3_D1
	91,  90,	// 23 | 24 : GPIO2_D3, GPIO2_D2
	-1, 122,	// 25 | 26 : GND, GPIO3_D2
	12,  11,	// 27 | 28 : GPIO0_B4, GPIO0_B3
	145, -1,	// 29 | 30 : GPIO4_C1, GND
	142,123,	// 31 | 32 : GPIO4_B6, GPIO3_D3
	13,  -1,	// 33 | 34 : GPIO0_B5, GND
	125,124,	// 35 | 36 : GPIO3_D5, GPIO3_D4
	-1,  -1,	// 37 | 38 : ADC.AIN1, 1.8V REF
	-1,  -1,	// 39 | 40 : GND, ADC.AIN0

	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

static const char *pinToPwm[64] = {
	// wiringPi number to pwm group number
		"None", "None",//  0 |  1 : GPIO0_C0, GPIO3_D0
		"None", "fe6f0010",	   //  2 |  3 : GPIO0_C1, GPIO3_B2(PWM9)
		"None", "None",		   //  4 |  5 : GPIO3_C6, GPIO3_C7
		"None", "fdd70020",	   //  6 |  7 : GPIO3_D1, GPIO0_B6(PWM2)
		"None", "None",        //  8 |  9 : GPIO3_B6, GPIO3_B5
		"None", "None",        // 10 | 11 : GPIO2_D2, GPIO3_D2
		"None", "None",        // 12 | 13 : GPIO2_D1, GPIO2_D0
		"None", "None",        // 14 | 15 : GPIO2_D3, GPIO3_D6
		"None", "None",		   // 16 | 17 : GPIO3_D7
		"None", "None",		   // 18 | 19 :
		"None", "None",		   // 20 | 21 : , GPIO4_C1
		"None", "fdd70010",	   // 22 | 23 : GPIO4_B6, GPIO0_B5(PWM1)
		"None", "None",	   // 24 | 25 : GPIO3_D5,
		"None", "None",        // 26 | 27 : GPIO3_D3, GPIO3_D4
		"None", "None",        // 28 | 29 :
		"None", "None",        // 30 | 31 : GPIO0_B4, GPIO0_B3
	// Padding:
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None", // 32...47
	"None","None","None","None","None","None","None","None","None","None","None","None","None","None","None","None"  // 48...63
};

static const int pinToPwmNum[64] = {
	// wiringPi number to pwm pin number
	 -1, -1,	//  0 |  1 : GPIO0_C0, GPIO3_D0
	 -1,  2,	//  2 |  3 : GPIO0_C1, GPIO3_B2(PWM9)
	 -1, -1,	//  4 |  5 : GPIO3_C6, GPIO3_C7
	 -1,  1,	//  6 |  7 : GPIO3_D1, GPIO0_B6(PWM2)
	 -1, -1,	//  8 |  9 : GPIO3_B6, GPIO3_B5
	 -1, -1,	// 10 | 11 : GPIO2_D2, GPIO3_D2
	 -1, -1,	// 12 | 13 : GPIO2_D1, GPIO2_D0
	 -1, -1,	// 14 | 15 : GPIO2_D3, GPIO3_D6
	 -1, -1,	// 16 | 17 : GPIO3_D7
	 -1, -1,	// 18 | 19 :
	 -1, -1,	// 20 | 21 : , GPIO4_C1
	 -1,  0,	// 22 | 23 : GPIO4_B6, GPIO0_B5(PWM1)
	 -1, -1,	// 24 | 25 : GPIO3_D5,
	 -1, -1,	// 26 | 27 : GPIO3_D3, GPIO3_D4
	 -1, -1,	// 28 | 29 :
	 -1, -1,	// 30 | 31 : GPIO0_B4, GPIO0_B3
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1	// 48...63
};

static char pwmPinPath[10][(BLOCK_SIZE)] = {
	"","",
	"",
	// Padding:
	"None","None","None",
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
/* ADC file descriptor */
static int adcFds[2];

/* GPIO mmap control. Actual GPIO bank number. */
static volatile uint32_t *gpio[5];

/* GRF(General Register Files) base addresses to control GPIO modes */
static volatile uint32_t *grf[2];

/* CRU(Clock & Reset Unit) base addresses to control CLK mode */
static volatile uint32_t *cru[2];

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
static int	gpioToShiftRegBy32	(int pin);
static int	gpioToShiftRegBy16	(int pin);
static void	setClkState	(int pin, int state);
static int	setIomuxMode 	(int pin, int mode);
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
static int		_pinMode		(int pin, int mode);
static int		_getDrive		(int pin);
static int		_setDrive		(int pin, int value);
static int		_getAlt			(int pin);
static int		_getPUPD		(int pin);
static int		_pullUpDnControl	(int pin, int pud);
static int		_digitalRead		(int pin);
static int		_digitalWrite		(int pin, int value);
static int		_pwmWrite		(int pin, int value);
static int		_analogRead		(int pin);
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);
static void		_pwmSetRange	(unsigned int range);
static void		_pwmSetClock	(int divisor);

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
// libgpiod core function
/*----------------------------------------------------------------------------*/
static int		_pinMode_gpiod		(int pin, int mode);
static int		_digitalRead_gpiod		(int pin);
static int		_digitalWrite_gpiod		(int pin, int value);
static int		_digitalWriteByte_gpiod	(const unsigned int value);
static unsigned int	_digitalReadByte_gpiod	(void);
#endif

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);
static 	void init_adc_fds	(void);
void init_odroidm1 	(struct libodroid *libwiring);
/*----------------------------------------------------------------------------*/
//
// for the offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftRegBy32 (int pin)
{
	return pin % 32;
}
/*----------------------------------------------------------------------------*/
//
// for the offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftRegBy16 (int pin)
{
	return pin % 16;
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
	if (pwmPin == 2)
		pwmClock = M1_PWM_INTERNAL_CLK;
	else
		pwmClock = (M1_PWM_INTERNAL_CLK / 2);
	sprintf(pwmExport, "%d", 0);
	sprintf(pwmPinPath[pwmPin], "%s/pwm%d", sysPwmPath, 0);
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
	if (pin > 255)
		return msg(MSG_ERR, "%s : Invalid pin number %d\n", __func__, pin);

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		return	pin;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		return	lib->sysFds[pin] != -1 ? pin : -1;
	/* wiringPi number */
	case	MODE_PINS:
		return	pin < 64 ? pinToGpio[pin] : -1;
	/* header pin number */
	case	MODE_PHYS:
		return	pin < 64 ? phyToGpio[pin] : -1;
	default	:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return	-1;
	}
}
/*----------------------------------------------------------------------------*/
//
// set GPIO clock state
//
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static void setClkState (int pin, int state)
{
	uint32_t target = 0;
	uint8_t bank = pin / GPIO_SIZE;
	uint8_t gpioPclkShift = (bank == 0 ? M1_PMU_CRU_GPIO_PCLK_BIT : (bank * M1_CRU_GPIO_PCLK_BIT));

	target |= (1 << (gpioPclkShift + 16));

	switch (state) {
	case M1_CLK_ENABLE:
		if (bank == 0) {
			target |= *(cru[0] + (M1_PMU_CRU_GPIO_CLK_OFFSET >> 2));
			target &= ~(1 << gpioPclkShift);
			*(cru[0] + (M1_PMU_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		} else {
			target |= *(cru[1] + (M1_CRU_GPIO_CLK_OFFSET >> 2));
			target &= ~(1 << gpioPclkShift);
			*(cru[1] + (M1_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		}
		break;
	case M1_CLK_DISABLE:
		if (bank == 0) {
			target |= *(cru[0] + (M1_PMU_CRU_GPIO_CLK_OFFSET >> 2));
			target |= (1 << gpioPclkShift);
			*(cru[0] + (M1_PMU_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		} else {
			target |= *(cru[1] + (M1_CRU_GPIO_CLK_OFFSET >> 2));
			target |=  (1 << gpioPclkShift);
			*(cru[1] + (M1_CRU_GPIO_CLK_OFFSET >> 2)) = target;
		}
		break;
	case M1_CLK_BYTE_ENABLE:
		setClkState(GPIO_SIZE * 0, M1_CLK_ENABLE);
		setClkState(GPIO_SIZE * 3, M1_CLK_ENABLE);
		break;
	case M1_CLK_BYTE_DISABLE:
		setClkState(GPIO_SIZE * 0, M1_CLK_DISABLE);
		setClkState(GPIO_SIZE * 3, M1_CLK_DISABLE);
		break;
	default:
		break;
	}
}
/*----------------------------------------------------------------------------*/
//
// set IOMUX mode
//
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int setIomuxMode (int pin, int mode)
{
	uint32_t offset, target;
	uint8_t	bank, group, bitNum, bitInByte;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;
	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	group = bitNum / 8; // A or B or C or D
	bitInByte = (pin - (bank * GPIO_SIZE)) % 8;

	offset = (bank == 0 ? 0 : bank-1) * 0x20 + group * 0x8;
	offset += (bitInByte / 4 == 0) ? 0x0 : 0x4;

	// Common IOMUX Funtion 1 : GPIO (3'h0)
	switch (mode) {
	case M1_FUNC_GPIO: // Common IOMUX Function 1_GPIO (3'h0)
		if (bank == 0) {
			offset += M1_PMU_GRF_IOMUX_OFFSET;
			target = *(grf[0] + (offset >> 2));
			target |= (0x7 << ((bitInByte % 4) * 4 + 16));
			target &= ~(0x7 << ((bitInByte % 4) * 4)); // ~0x07 = 3'h0
			*(grf[0] + (offset >> 2)) = target;
		}
		else {
			offset += M1_SYS_GRF_IOMUX_OFFSET;
			target = *(grf[1] + (offset >> 2));
			target |= (0x7 << ((bitInByte % 4) * 4 + 16));
			target &= ~(0x7 << ((bitInByte % 4) * 4));
			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	case M1_FUNC_PWM:
		if (bank == 0) {
			offset += M1_PMU_GRF_IOMUX_OFFSET;
			target = *(grf[0] + (offset >> 2));
			target |= (0x7 << ((bitInByte % 4) * 4 + 16));
			target |= (0x4 << ((bitInByte % 4) * 4)); // gpio0 b5/b6: 3'h100
			*(grf[0] + (offset >> 2)) = target;
		}
		else {
			offset += M1_SYS_GRF_IOMUX_OFFSET;
			target = *(grf[1] + (offset >> 2));
			target |= (0x7 << ((bitInByte % 4) * 4 + 16));
			target |= (0x5 << ((bitInByte % 4) * 4)); // gpio3 b2: 3'h101
			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	default:
		break;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _pinMode (int pin, int mode)
{
	uint32_t offset, target;
	uint8_t bank, bitNum;
	int origPin;

	origPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	offset = bitNum / 16 == 0 ? M1_GPIO_DIR_OFFSET : M1_GPIO_DIR_OFFSET + 0x4;

	pwmRelease (origPin);
	softPwmStop(pin);
	softToneStop(pin);

	target = *(gpio[bank] + (offset >> 2));
	target |= (1 << (gpioToShiftRegBy16(pin) + 16));

	switch (mode) {
	case INPUT:
		setIomuxMode(origPin, M1_FUNC_GPIO);
		target &= ~(1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) = target;
		_pullUpDnControl(origPin, PUD_OFF);
		break;
	case OUTPUT:
		setIomuxMode(origPin, M1_FUNC_GPIO);
		target |= (1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) = target;
		break;
	case INPUT_PULLUP:
		setIomuxMode(origPin, M1_FUNC_GPIO);
		target &= ~(1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) = target;
		_pullUpDnControl(origPin, PUD_UP);
		break;
	case INPUT_PULLDOWN:
		setIomuxMode(origPin, M1_FUNC_GPIO);
		target &= ~(1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) = target;
		_pullUpDnControl(origPin, PUD_DOWN);
		break;
	case SOFT_PWM_OUTPUT:
		softPwmCreate(origPin, 0, 100);
		break;
	case SOFT_TONE_OUTPUT:
		softToneCreate(origPin);
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

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _pinMode_gpiod (int pin, int mode)
{
	uint8_t bank, bitNum;
	int oriPin, ret;

	oriPin = pin;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);

	softPwmStop(pin);
	softToneStop(pin);

	chip = gpiod_chip_open_by_number(bank);
	if (!chip) {
		return -1;
	}

	gpiod = gpiod_chip_get_line(chip, bitNum);
	if (!gpiod) {
		printf("gpio get error\n");
		gpiod_chip_close(chip);
		return -1;
	}

	switch (mode) {
	case INPUT:
		ret = gpiod_line_request_input(gpiod, CONSUMER);
		if (ret < 0) {
			printf("gpiod request error\n");
			gpiod_line_release(gpiod);
		}
		_pullUpDnControl(oriPin, PUD_OFF);
		break;
	case OUTPUT:
		ret = gpiod_line_request_output(gpiod, CONSUMER, 0);
		if (ret < 0) {
			printf("gpiod request error\n");
			gpiod_line_release(gpiod);
		}
		break;
	case INPUT_PULLUP:
		ret = gpiod_line_request_input(gpiod, CONSUMER);
		if (ret < 0) {
			printf("gpiod request error\n");
			gpiod_line_release(gpiod);
		}
		_pullUpDnControl(oriPin, PUD_UP);
		break;
	case INPUT_PULLDOWN:
		ret = gpiod_line_request_input(gpiod, CONSUMER);
		if (ret < 0) {
			printf("gpiod request error\n");
			gpiod_line_release(gpiod);
		}
		_pullUpDnControl(oriPin, PUD_DOWN);
		break;
	case SOFT_PWM_OUTPUT:
		softPwmCreate(oriPin, 0, 100);
		break;
	case SOFT_TONE_OUTPUT:
		softToneCreate(oriPin);
		break;
	default:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	gpiod_line_release(gpiod);
	gpiod_chip_close(chip);

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _getDrive(int pin)
{
	uint32_t offset, target;
	uint8_t bank, group, bitNum, bitInByte;
	int value = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	bitInByte = bitNum % 8;
	group = bitNum / 8;

	if (bank == 0) {
		offset = M1_PMU_GRF_DS_OFFSET;
		offset += (group * 0x10);
		offset += ((bitInByte / 2) * 0x4);
		target = *(grf[0] + (offset >> 2));
	}
	else {
		offset = M1_SYS_GRF_DS_OFFSET;
		offset += ((bank - 1) * 0x40);
		offset += (group * 0x10);
		offset += ((bitInByte / 2) * 0x4);
		target = *(grf[1] + (offset >> 2));
	}

	target &= 0x3f3f; //reset reserved bits
	target = (bitInByte % 2 == 0 ? target & 0x3f : target >> 8);

	switch (target) {
		case DS_LEVEL_0:
			value = 0;
			break;
		case DS_LEVEL_1:
			value = 1;
			break;
		case DS_LEVEL_2:
			value = 2;
			break;
		case DS_LEVEL_3:
			value = 3;
			break;
		case DS_LEVEL_4:
			value = 4;
			break;
		case DS_LEVEL_5:
			value = 5;
			break;
		default:
			value = -1;
			break;
	}

	return value;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _setDrive(int pin, int value)
{
	uint32_t offset, target;
	uint8_t bank, group, bitNum, bitInByte;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	bitInByte = bitNum % 8;
	group = bitNum / 8;

	if (bank == 0) {
		offset = M1_PMU_GRF_DS_OFFSET;
		offset += (group * 0x10);
		offset += ((bitInByte / 2) * 0x4);
		target = *(grf[0] + (offset >> 2));
	}
	else {
		offset = M1_SYS_GRF_DS_OFFSET;
		offset += ((bank - 1) * 0x40);
		offset += (group * 0x10);
		offset += ((bitInByte / 2) * 0x4);
		target = *(grf[1] + (offset >> 2));
	}

	target |= (0x3f3f << 16);
	target &= ~(bitInByte % 2 == 0 ? 0x3f << 0 : 0x3f << 8);

	switch (value) {
		case 0:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_0 : (DS_LEVEL_0 << 8));
			break;
		case 1:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_1 : (DS_LEVEL_1 << 8));
			break;
		case 2:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_2 : (DS_LEVEL_2 << 8));
			break;
		case 3:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_3 : (DS_LEVEL_3 << 8));
			break;
		case 4:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_4 : (DS_LEVEL_4 << 8));
			break;
		case 5:
			target |= (bitInByte % 2 == 0 ? DS_LEVEL_5 : (DS_LEVEL_5 << 8));
			break;
		default:
			break;
	}

	if (bank == 0) {
		*(grf[0] + (offset >> 2)) = target;
	}
	else {
		*(grf[1] + (offset >> 2)) = target;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _getAlt (int pin)
{
	// TODO: Working confirmed
	uint32_t offset;
	uint16_t ret = 0;
	uint8_t	bank, group, bitNum, bitInByte, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / GPIO_SIZE; // GPIO0, GPIO1, ...
	bitNum = pin - (bank * GPIO_SIZE);
	bitInByte = bitNum % 8;
	group = bitNum / 8; // GPIO0_A, GPIO0_B, ...

	// Move to the proper IOMUX register regardless of whether it is L, H.
	offset = 0x20 * (bank == 0 ? 0x0 : bank - 1) + 0x8 * group;

	// Check where the register this pin located in
	offset += (bitInByte / 4 == 0) ? 0x0 : 0x4;

	// The shift to move to the target pin at the register
	shift = bitInByte % 4 * 4;

	// Check if the pin is GPIO mode on GRF register
	if (bank == 0) {
		offset += M1_PMU_GRF_IOMUX_OFFSET; //0x00
		ret = (*(grf[0] + (offset >> 2)) >> shift) & 0x7;
	} else {
		offset += M1_SYS_GRF_IOMUX_OFFSET; //0x00
		ret = (*(grf[1] + (offset >> 2)) >> shift) & 0x7;
	}

	// If it is ALT0 (GPIO mode), check it's direction
	// Add offset 0x4 to go to H register
	// when the bit group is in the high two-bytes of the word size
	if (ret == 0) {
		if (bitNum / 16 == 0)
			offset = (M1_GPIO_DIR_OFFSET >> 2);
		else
			offset = ((M1_GPIO_DIR_OFFSET + 0x4) >> 2);
		ret = !!(*(gpio[bank] + offset) & (1 << gpioToShiftRegBy16(bitNum)));
	}
	else {
		// If it is alternative mode, add number 2 to fit into
		// the alts[] array for "gpio readall" command
		// Because the read number directly indicates the number of alt function
		ret += 2;
	}

	return ret;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _getPUPD (int pin)
{
	uint32_t offset, pupd;
	uint8_t bank, group, bitNum, bitInByte;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode,pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	group = bitNum / 8;
	bitInByte = bitNum % 8;
	pupd = 0x00;
	pupd = (0x3 << (bitInByte * 2));
	offset = (bank == 0) ? M1_PMU_GRF_PUPD_OFFSET + (group * 0x4) :  M1_SYS_GRF_PUPD_OFFSET + (group * 0x4) + ((bank - 1) * 0x10);

	if (bank == 0)
		pupd &= *(grf[0] + (offset >> 2));
	else
		pupd &= *(grf[1] + (offset >> 2));

	pupd = (pupd >> bitInByte * 2);

	return pupd;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _pullUpDnControl (int pin, int pud)
{
	uint32_t offset, target;
	uint8_t	bank, group, bitNum, bitInByte;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0) //exit
		return	-1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);
	group = bitNum / 8;
	bitInByte = bitNum % 8;
	offset = (bank == 0) ? M1_PMU_GRF_PUPD_OFFSET + (group * 0x4) :  M1_SYS_GRF_PUPD_OFFSET + (group * 0x4) + ((bank - 1) * 0x10);

	switch (pud) {
	case PUD_UP:
		if (bank == 0)
		{
			target = *(grf[0] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			target |= (0x1 << (bitInByte * 2));
			*(grf[0] + (offset >> 2)) = target;
		}
		else
		{
			target = *(grf[1] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			target |= (0x1 << (bitInByte * 2));
			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	case PUD_DOWN:
		if (bank == 0)
		{
			target = *(grf[0] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			target |= (0x2 << (bitInByte * 2));
			*(grf[0] + (offset >> 2)) = target;
		}
		else
		{
			target = *(grf[1] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			target |= (0x2 << (bitInByte * 2));
			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	case PUD_OFF:
		if (bank == 0)
		{
			target = *(grf[0] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			*(grf[0] + (offset >> 2)) = target;
		}
		else
		{
			target = *(grf[1] + (offset >> 2));
			target |= (0x3 << ((bitInByte * 2) + 16));
			target &= ~(0x3 << (bitInByte * 2));
			*(grf[1] + (offset >> 2)) = target;
		}
		break;
	default:
		break;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _digitalRead (int pin)
{
	uint8_t bank;
	int ret;
	char c;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return (c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / GPIO_SIZE;

	ret = *(gpio[bank] + (M1_GPIO_GET_OFFSET >> 2)) & (1 << gpioToShiftRegBy32(pin)) ? HIGH : LOW;

	return ret;
}

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _digitalRead_gpiod (int pin)
{
	uint8_t bank, bitNum;
	int ret;
	char c;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return (c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);

	chip = gpiod_chip_open_by_number(bank);
	if (!chip) {
		printf("chip open error\n");
		return -1;
	}
	gpiod = gpiod_chip_get_line(chip,bitNum);
	if (!gpiod) {
		printf("gpiod get error\n");
		gpiod_chip_close(chip);
	}
	ret = gpiod_line_get_value(gpiod);
	if (ret < 0) {
		printf("gpiod get error\n");
		gpiod_line_release(gpiod);
	}

	gpiod_line_release(gpiod);
	gpiod_chip_close(chip);

	return ret;
}
#endif

/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _digitalWrite (int pin, int value)
{
	uint32_t offset, target;
	uint8_t bank, bitNum;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write (lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + M1_GPIO_PIN_BASE);
			} else {
				if (write (lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + M1_GPIO_PIN_BASE);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank*GPIO_SIZE);

	offset = (bitNum / 16 == 0 ? M1_GPIO_SET_OFFSET : M1_GPIO_SET_OFFSET + 0x04);

	target = *(gpio[bank] + (offset >> 2));
	target |= (1 << (gpioToShiftRegBy16(pin)+16));

	switch (value) {
	case LOW:
		target &= ~(1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) = target;
		break;
	case HIGH:
		target |= (1 << gpioToShiftRegBy16(pin));
		*(gpio[bank] + (offset >> 2)) =  target;
		break;
	default:
		break;
	}

	return 0;
}

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _digitalWrite_gpiod (int pin, int value)
{
	uint8_t bank,bitNum;
	int ret;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write (lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + M1_GPIO_PIN_BASE);
			} else {
				if (write (lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_ERR,
					"%s : %s\nEdit direction file to output mode for\n\t/sys/class/gpio/gpio%d/direction\n",
					__func__, strerror(errno), pin + M1_GPIO_PIN_BASE);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	bank = pin / GPIO_SIZE;
	bitNum = pin - (bank * GPIO_SIZE);

	chip = gpiod_chip_open_by_number(bank);
	if (!chip) {
		printf("chip open error\n");
		return -1;
	}

	gpiod = gpiod_chip_get_line(chip, bitNum);
	if (!gpiod) {
		printf("gpiod get error\n");
		gpiod_chip_close(chip);
	}

	switch (value) {
	case LOW:
		ret = gpiod_line_set_value(gpiod, 0);
		if (ret < 0) {
			printf("gpiod set error\n");
			gpiod_line_release(gpiod);
		}
		break;
	case HIGH:
		ret = gpiod_line_set_value(gpiod, 1);
		if (ret < 0) {
			printf("gpiod set error\n");
			gpiod_line_release(gpiod);
		}
		break;
	default:
		break;
	}

	gpiod_line_release(gpiod);
	gpiod_chip_close(chip);

	return 0;
}
#endif

__attribute__ ((unused))static int _pwmWrite (int pin, int value)
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
	char value[5] = {0, };

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	/* wiringPi ADC number = pin 25, pin 29 */
	switch (pin) {
#if defined(ARDUINO)
	/* To work with physical analog channel numbering */
	case	1:	case	25:
		pin = 0;
	break;
	case	0:	case	29:
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
__attribute__ ((unused))static int _digitalWriteByte (const unsigned int value)
{
	union reg_bitfield gpio0;
	union reg_bitfield gpio3;

	if (lib->mode == MODE_GPIO_SYS) {
		return -1;
	}

	/* Read data register */
	gpio0.wvalue = *(gpio[0] + (M1_GPIO_GET_OFFSET >> 2));
	gpio3.wvalue = *(gpio[3] + (M1_GPIO_GET_OFFSET >> 2));

	/* Wiring PI GPIO0 = M1 GPIO0_C.0 */
	gpio0.bits.bit16 = ((value & 0x01) >> 0);
	/* Wiring PI GPIO1 = M1 GPIO3_D.0 */
	gpio3.bits.bit24 = ((value & 0x02) >> 1);
	/* Wiring PI GPIO2 = M1 GPIO0_C.1 */
	gpio0.bits.bit17 = ((value & 0x04) >> 2);
	/* Wiring PI GPIO3 = M1 GPIO3_B.2 */
	gpio3.bits.bit10 = ((value & 0x08) >> 3);
	/* Wiring PI GPIO4 = M1 GPIO3_C.6 */
	gpio3.bits.bit22 = ((value & 0x10) >> 4);
	/* Wiring PI GPIO5 = M1 GPIO3_C.7 */
	gpio3.bits.bit23 = ((value & 0x20) >> 5);
	/* Wiring PI GPIO6 = M1 GPIO3_D.1 */
	gpio3.bits.bit25 = ((value & 0x40) >> 6);
	/* Wiring PI GPIO7 = M1 GPIO0_B.6 */
	gpio0.bits.bit14 = ((value & 0x80) >> 7);

	/* Update data register */
	*(gpio[0] + ((M1_GPIO_SET_OFFSET + 0x4) >> 2)) = (WRITE_BYTE_MASK_GPIO0_H | (gpio0.wvalue >> 16));
	*(gpio[0] + (M1_GPIO_SET_OFFSET >> 2)) = (WRITE_BYTE_MASK_GPIO0_L | (gpio0.wvalue & 0xffff));

	*(gpio[3] + ((M1_GPIO_SET_OFFSET + 0x4) >> 2)) = (WRITE_BYTE_MASK_GPIO3_H | (gpio3.wvalue >> 16));
	*(gpio[3] + (M1_GPIO_SET_OFFSET >> 2)) = (WRITE_BYTE_MASK_GPIO3_L | (gpio3.wvalue & 0xffff));

	return 0;
}

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static int _digitalWriteByte_gpiod (const unsigned int value)
{
	static struct gpiod_chip *chip[2];
	static struct gpiod_line *gpiodLines[8];

	uint32_t unit=0x0;
	int ret;

	if (lib->mode == MODE_GPIO_SYS) {
		return -1;
	}

	chip[0] = gpiod_chip_open_by_number(0);
	chip[1] = gpiod_chip_open_by_number(3);

	//gpiod get lines
	/* Wiring PI GPIO0 = M1 GPIO0_C.0 */
	gpiodLines[0] = gpiod_chip_get_line(chip[0], 16);
	/* Wiring PI GPIO1 = M1 GPIO3_D.0 */
	gpiodLines[1] = gpiod_chip_get_line(chip[1], 24);
	/* Wiring PI GPIO2 = M1 GPIO0_C.1 */
	gpiodLines[2] = gpiod_chip_get_line(chip[0], 17);
	/* Wiring PI GPIO3 = M1 GPIO3_B.2 */
	gpiodLines[3] = gpiod_chip_get_line(chip[1], 10);
	/* Wiring PI GPIO4 = M1 GPIO3_C.6 */
	gpiodLines[4] = gpiod_chip_get_line(chip[1], 22);
	/* Wiring PI GPIO5 = M1 GPIO3_C.7 */
	gpiodLines[5] = gpiod_chip_get_line(chip[1], 23);
	/* Wiring PI GPIO6 = M1 GPIO3_D.1 */
	gpiodLines[6] = gpiod_chip_get_line(chip[1], 25);
	/* Wiring PI GPIO7 = M1 GPIO0_B.6 */
	gpiodLines[7] = gpiod_chip_get_line(chip[0], 14);
	for (int i = 0; i < 8; i++) {
		if (!gpiodLines[i]) {
			printf("gpiod get error pin:%d\n",i);
			for (int j = 0; j < 2; j++)
				gpiod_chip_close(chip[j]);
		}
	}

	for (int i = 0; i < 8; i++) {
		unit = (1 << i);
		ret = gpiod_line_request_output(gpiodLines[i],CONSUMER,0);
		if (ret < 0) {
			printf("gpiod request error pin:%d\n" , i);
			gpiod_line_release(gpiodLines[i]);
		}
		ret = gpiod_line_set_value(gpiodLines[i],((value & unit) >> i));
		if (ret < 0) {
			printf("gpiod set error pin:%d\n" , i);
			gpiod_line_release(gpiodLines[i]);
		}
	}

	for (int i = 0; i < 8; i++)
		gpiod_line_release(gpiodLines[i]);
	for (int i = 0; i < 2; i++)
		gpiod_chip_close(chip[i]);

	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static unsigned int _digitalReadByte (void)
{
	union reg_bitfield gpio0;
	union reg_bitfield gpio3;

	unsigned int value = 0;

	if (lib->mode == MODE_GPIO_SYS) {
		return	-1;
	}

	/* Read data register */
	gpio0.wvalue = *(gpio[0] + (M1_GPIO_GET_OFFSET >> 2));
	gpio3.wvalue = *(gpio[3] + (M1_GPIO_GET_OFFSET >> 2));

	/* Wiring PI GPIO0 = M1 GPIO0_C.0 */
	if (gpio0.bits.bit16)
		value |= 0x01;
	/* Wiring PI GPIO1 = M1 GPIO3_D.0 */
	if (gpio3.bits.bit24)
		value |= 0x02;
	/* Wiring PI GPIO2 = M1 GPIO0_C.1 */
	if (gpio0.bits.bit17)
		value |= 0x04;
	/* Wiring PI GPIO3 = M1 GPIO3_B.2 */
	if (gpio3.bits.bit10)
		value |= 0x08;
	/* Wiring PI GPIO4 = M1 GPIO3_C.6 */
	if (gpio3.bits.bit22)
		value |= 0x10;
	/* Wiring PI GPIO5 = M1 GPIO3_C.7 */
	if (gpio3.bits.bit23)
		value |= 0x20;
	/* Wiring PI GPIO6 = M1 GPIO3_D.1 */
	if (gpio3.bits.bit25)
		value |= 0x40;
	/* Wiring PI GPIO7 = M1 GPIO0_B.6 */
	if (gpio0.bits.bit14)
		value |= 0x80;

	return value;
}

#if !defined(DEVMEM)
/*----------------------------------------------------------------------------*/
__attribute__ ((unused))static unsigned int _digitalReadByte_gpiod (void)
{
	static struct gpiod_chip *chip[2];
	static struct gpiod_line *gpiodLines[8];

	unsigned int value = 0;
	uint32_t unit = 0x0;
	int ret;

	if (lib->mode == MODE_GPIO_SYS) {
		return -1;
	}

	chip[0] = gpiod_chip_open_by_number(0);
	chip[1] = gpiod_chip_open_by_number(3);

	//gpiod get lines
	/* Wiring PI GPIO0 = M1 GPIO0_C.0 */
	gpiodLines[0] = gpiod_chip_get_line(chip[0], 16);
	/* Wiring PI GPIO1 = M1 GPIO3_D.0 */
	gpiodLines[1] = gpiod_chip_get_line(chip[1], 24);
	/* Wiring PI GPIO2 = M1 GPIO0_C.1 */
	gpiodLines[2] = gpiod_chip_get_line(chip[0], 17);
	/* Wiring PI GPIO3 = M1 GPIO3_B.2 */
	gpiodLines[3] = gpiod_chip_get_line(chip[1], 10);
	/* Wiring PI GPIO4 = M1 GPIO3_C.6 */
	gpiodLines[4] = gpiod_chip_get_line(chip[1], 22);
	/* Wiring PI GPIO5 = M1 GPIO3_C.7 */
	gpiodLines[5] = gpiod_chip_get_line(chip[1], 23);
	/* Wiring PI GPIO6 = M1 GPIO3_D.1 */
	gpiodLines[6] = gpiod_chip_get_line(chip[1], 25);
	/* Wiring PI GPIO7 = M1 GPIO0_B.6 */
	gpiodLines[7] = gpiod_chip_get_line(chip[0], 14);
	for (int i = 0; i < 8; i++) {
		if (!gpiodLines[i]) {
			printf("gpiod get error pin:%d\n",i);
			for (int j = 0; j < 2; j++)
				gpiod_chip_close(chip[j]);
		}
	}

	for (int i = 0; i < 8; i++) {
		unit = (1 << i);
		ret = gpiod_line_request_input(gpiodLines[i],CONSUMER);
		if (ret < 0) {
			printf("gpiod request error pin:%d\n",i);
			gpiod_line_release(gpiodLines[i]);
		}
		ret = gpiod_line_get_value(gpiodLines[i]);
		if (ret)
			value |= unit;
	}

	for (int i = 0; i < 8; i++)
		gpiod_line_release(gpiodLines[i]);
	for (int i = 0; i < 2; i++)
		gpiod_chip_close(chip[i]);

	return value;
}
#endif

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
		printf("error : pwm1 or pwm2 freq: %dMHz / (pwmSetClock's value) >= 2\n",
				(M1_PWM_INTERNAL_CLK / 2000000));
		printf("error : pwm9 freq: %dMHz / (pwmSetClock's value) >= 2\n",
				(M1_PWM_INTERNAL_CLK / 1000000));
		return;
	}

	pwmRange = range;
	if ((pwmRange < 1) || (pwmRange >= pwmClock)) {
		printf("error : invalid value. ( < pwm freq)\n");
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
// Internal clock: PWM0: 12MHz PWM2: 24MHz
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
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_cru[2], *mapped_grf[2], *mapped_gpio[5];

	/* GPIO mmap setup */
	if (!getuid()) {
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg (MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));
			setUsingGpiomem(TRUE);
		} else
			msg (MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		mapped_cru[0] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_PMU_CRU_BASE);
		mapped_cru[1] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_CRU_BASE);

		mapped_grf[0] = mmap(0, M1_GRF_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_PMU_GRF_BASE);
		mapped_grf[1] = mmap(0, M1_GRF_BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_SYS_GRF_BASE);

		mapped_gpio[0] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_GPIO_0_BASE);
		mapped_gpio[1] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_GPIO_1_BASE);
		mapped_gpio[2] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_GPIO_2_BASE);
		mapped_gpio[4] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_GPIO_4_BASE);
		mapped_gpio[3] = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M1_GPIO_3_BASE);

		if ((mapped_cru[0] == MAP_FAILED) || (mapped_cru[1] == MAP_FAILED)) {
			msg (MSG_ERR,"wiringPiSetup: mmap (CRU) failed: %s\n",strerror (errno));
		} else {
			cru[0] = (uint32_t *) mapped_cru[0];
			cru[1] = (uint32_t *) mapped_cru[1];
		}

		if ((mapped_grf[0] == MAP_FAILED) || (mapped_grf[1] == MAP_FAILED)) {
			msg (MSG_ERR,"wiringPiSetup: mmap (GRF) failed: %s\n",strerror (errno));
		} else {
			grf[0] = (uint32_t *) mapped_grf[0];
			grf[1] = (uint32_t *) mapped_grf[1];
		}

		if ((mapped_gpio[0] == MAP_FAILED) ||
			(mapped_gpio[1] == MAP_FAILED) ||
			(mapped_gpio[2] == MAP_FAILED) ||
			(mapped_gpio[3] == MAP_FAILED) ||
			(mapped_gpio[4] == MAP_FAILED)) {
			msg (MSG_ERR,
				"wiringPiSetup: mmap (GPIO) failed: %s\n",
				strerror (errno));
		} else {
			gpio[0] = (uint32_t *) mapped_gpio[0];
			gpio[1] = (uint32_t *) mapped_gpio[1];
			gpio[2] = (uint32_t *) mapped_gpio[2];
			gpio[3] = (uint32_t *) mapped_gpio[3];
			gpio[4] = (uint32_t *) mapped_gpio[4];
		}
	}
}
/*----------------------------------------------------------------------------*/
static void init_adc_fds (void)
{
	const char *AIN0_NODE, *AIN1_NODE;

	AIN0_NODE = "/sys/devices/platform/fe720000.saradc/iio:device0/in_voltage7_raw";
	AIN1_NODE = "/sys/devices/platform/fe720000.saradc/iio:device0/in_voltage6_raw";

	adcFds[0] = open(AIN0_NODE, O_RDONLY);
	adcFds[1] = open(AIN1_NODE, O_RDONLY);
}
/*----------------------------------------------------------------------------*/
void init_odroidm1 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	init_adc_fds();
#if defined(DEVMEM)
	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->getDrive			= _getDrive;
	libwiring->setDrive			= _setDrive;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;
	libwiring->pwmWrite			= _pwmWrite;
	libwiring->pwmSetRange		= _pwmSetRange;
	libwiring->pwmSetClock		= _pwmSetClock;
#else
	/* wiringPi-libgpiod Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->pinMode		= _pinMode_gpiod;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->getDrive			= _getDrive;
	libwiring->setDrive			= _setDrive;
	libwiring->digitalRead		= _digitalRead_gpiod;
	libwiring->digitalWrite		= _digitalWrite_gpiod;
	libwiring->analogRead		= _analogRead;
	libwiring->digitalWriteByte	= _digitalWriteByte_gpiod;
	libwiring->digitalReadByte	= _digitalReadByte_gpiod;
#endif
	/* specify pin base number */
	libwiring->pinBase		= M1_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}
/*----------------------------------------------------------------------------*/
