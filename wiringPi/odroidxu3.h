/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-XU3/XU4 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_XU3_H__
#define	__ODROID_XU3_H__

/*----------------------------------------------------------------------------*/
#define ODROIDXU3_GPIO_MASK	(0xFFFFFF00)

#define GPIO_PIN_BASE		0

// GPX0,1,2,3
#define ODROIDXU3_GPX_BASE	0x13400000

#define GPIO_X1_START		16
#define GPIO_X1_CON_OFFSET	0x0C20
#define GPIO_X1_DAT_OFFSET	0x0C24
#define GPIO_X1_PUD_OFFSET	0x0C28
#define GPIO_X1_END		23

#define GPIO_X2_START		24
#define GPIO_X2_CON_OFFSET	0x0C40
#define GPIO_X2_DAT_OFFSET	0x0C44
#define GPIO_X2_PUD_OFFSET	0x0C48
#define GPIO_X2_END		31

#define GPIO_X3_START		32
#define GPIO_X3_CON_OFFSET	0x0C60
#define GPIO_X3_DAT_OFFSET	0x0C64
#define GPIO_X3_PUD_OFFSET	0x0C68
#define GPIO_X3_END		39

// GPA0,1,2, GPB0,1,2,3,4
#define ODROIDXU3_GPA_BASE	0x14010000

#define GPIO_A0_START		171
#define GPIO_A0_CON_OFFSET	0x0000
#define GPIO_A0_DAT_OFFSET	0x0004
#define GPIO_A0_PUD_OFFSET	0x0008
#define GPIO_A0_END		178

#define GPIO_A2_START		185
#define GPIO_A2_CON_OFFSET	0x0040
#define GPIO_A2_DAT_OFFSET	0x0044
#define GPIO_A2_PUD_OFFSET	0x0048
#define GPIO_A2_END		192

#define GPIO_B3_START		207
#define GPIO_B3_CON_OFFSET	0x00C0
#define GPIO_B3_DAT_OFFSET	0x00C4
#define GPIO_B3_PUD_OFFSET	0x00C8
#define GPIO_B3_END		214

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidxu3 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_XU3_H__ */
/*----------------------------------------------------------------------------*/
