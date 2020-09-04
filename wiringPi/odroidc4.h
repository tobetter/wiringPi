/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-C4 Board Header file

 */
/*----------------------------------------------------------------------------*/
#ifndef	__ODROID_C4_H__
#define	__ODROID_C4_H__

/*----------------------------------------------------------------------------*/
#define C4_GPIO_BASE			0xFF634000

#define C4_GPIO_PIN_BASE		410

#define C4_GPIOH_PIN_START		(C4_GPIO_PIN_BASE + 17)		// GPIOH_0
#define C4_GPIOH_PIN_END		(C4_GPIO_PIN_BASE + 25)		// GPIOH_8
#define C4_GPIOA_PIN_START		(C4_GPIO_PIN_BASE + 50)		// GPIOA_0
#define C4_GPIOA_PIN_END		(C4_GPIO_PIN_BASE + 65)		// GPIOA_15
#define C4_GPIOX_PIN_START		(C4_GPIO_PIN_BASE + 66)		// GPIOX_0
#define C4_GPIOX_PIN_MID		(C4_GPIO_PIN_BASE + 81)		// GPIOX_15
#define C4_GPIOX_PIN_END		(C4_GPIO_PIN_BASE + 85)		// GPIOX_19

#define C4_GPIOH_FSEL_REG_OFFSET	0x119
#define C4_GPIOH_OUTP_REG_OFFSET	0x11A
#define C4_GPIOH_INP_REG_OFFSET		0x11B
#define C4_GPIOH_PUPD_REG_OFFSET	0x13D
#define C4_GPIOH_PUEN_REG_OFFSET	0x14B
#define C4_GPIOH_DS_REG_3A_OFFSET	0x1D4
#define C4_GPIOH_MUX_B_REG_OFFSET	0x1BB

#define C4_GPIOA_FSEL_REG_OFFSET	0x120
#define C4_GPIOA_OUTP_REG_OFFSET	0x121
#define C4_GPIOA_INP_REG_OFFSET		0x122
#define C4_GPIOA_PUPD_REG_OFFSET	0x13F
#define C4_GPIOA_PUEN_REG_OFFSET	0x14D
#define C4_GPIOA_DS_REG_5A_OFFSET	0x1D6
#define C4_GPIOA_MUX_D_REG_OFFSET	0x1BD
#define C4_GPIOA_MUX_E_REG_OFFSET	0x1BE

#define C4_GPIOX_FSEL_REG_OFFSET	0x116
#define C4_GPIOX_OUTP_REG_OFFSET	0x117
#define C4_GPIOX_INP_REG_OFFSET		0x118
#define C4_GPIOX_PUPD_REG_OFFSET	0x13C
#define C4_GPIOX_PUEN_REG_OFFSET	0x14A
#define C4_GPIOX_DS_REG_2A_OFFSET	0x1D2
#define C4_GPIOX_DS_REG_2B_OFFSET	0x1D3
#define C4_GPIOX_MUX_3_REG_OFFSET	0x1B3
#define C4_GPIOX_MUX_4_REG_OFFSET	0x1B4
#define C4_GPIOX_MUX_5_REG_OFFSET	0x1B5

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidc4 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_C4_H__ */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
