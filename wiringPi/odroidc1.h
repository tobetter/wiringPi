/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-C0/C1/C1+ Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_C1_H__
#define	__ODROID_C1_H__

/*----------------------------------------------------------------------------*/
#define ODROIDC_GPIO_MASK	(0xFFFFFF80)
#define ODROIDC1_GPIO_BASE	0xC1108000

#define GPIO_PIN_BASE		0

#define GPIODV_PIN_START	50
#define GPIODV_PIN_END		79
#define GPIOY_PIN_START		80
#define GPIOY_PIN_END		96
#define GPIOX_PIN_START		97
#define GPIOX_PIN_END		118

#define GPIOX_FSEL_REG_OFFSET	0x0C
#define GPIOX_OUTP_REG_OFFSET	0x0D
#define GPIOX_INP_REG_OFFSET	0x0E
#define GPIOX_PUPD_REG_OFFSET	0x3E
#define GPIOX_PUEN_REG_OFFSET	0x4C

#define GPIOY_FSEL_REG_OFFSET	0x0F
#define GPIOY_OUTP_REG_OFFSET	0x10
#define GPIOY_INP_REG_OFFSET	0x11
#define GPIOY_PUPD_REG_OFFSET	0x3D
#define GPIOY_PUEN_REG_OFFSET	0x4B

#define GPIODV_FSEL_REG_OFFSET	0x12
#define GPIODV_OUTP_REG_OFFSET	0x13
#define GPIODV_INP_REG_OFFSET	0x14
#define GPIODV_PUPD_REG_OFFSET	0x3A
#define GPIODV_PUEN_REG_OFFSET	0x48

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidc1 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_C1_H__ */
/*----------------------------------------------------------------------------*/
