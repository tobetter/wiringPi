/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-N2 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_N2_H__
#define	__ODROID_N2_H__

/*----------------------------------------------------------------------------*/
#define ODROIDN2_GPIO_MASK		(0xFFFFFF00)
#define ODROIDN2_GPIO_BASE		0xff634000

#define GPIO_PIN_BASE			410

#define N2_GPIOA_PIN_START		(GPIO_PIN_BASE + 50)
#define N2_GPIOA_PIN_END		(GPIO_PIN_BASE + 65)
#define N2_GPIOX_PIN_START		(GPIO_PIN_BASE + 66)
#define N2_GPIOX_PIN_END		(GPIO_PIN_BASE + 85)

#define N2_GPIOX_FSEL_REG_OFFSET	0x116
#define N2_GPIOX_OUTP_REG_OFFSET	0x117
#define N2_GPIOX_INP_REG_OFFSET		0x118
#define N2_GPIOX_PUPD_REG_OFFSET	0x13C
#define N2_GPIOX_PUEN_REG_OFFSET	0x14A

#define N2_GPIOA_FSEL_REG_OFFSET	0x120
#define N2_GPIOA_OUTP_REG_OFFSET	0x121
#define N2_GPIOA_INP_REG_OFFSET		0x122
#define N2_GPIOA_PUPD_REG_OFFSET	0x13F
#define N2_GPIOA_PUEN_REG_OFFSET	0x14D

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidn2 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_N2_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
