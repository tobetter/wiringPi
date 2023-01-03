/*----------------------------------------------------------------------------*/
/*

	WiringPi ODROID-M1 Board Header file

 */
/*----------------------------------------------------------------------------*/
/*******************************************************************************
Copyright (C) 2021 Steve Jeong <how2soft@gmail.com>

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

#ifndef	__ODROID_M1_H__
#define	__ODROID_M1_H__
// flag of Using "/dev/gpiomem" or "libgpiod"
#define DEVMEM

/*----------------------------------------------------------------------------*/
// Common mmap block size for ODROID-M1 GRF register
#define M1_GPIO_PIN_BASE	0
//setClkState mode
#define M1_CLK_ENABLE	0
#define M1_CLK_DISABLE	1
#define M1_CLK_BYTE_ENABLE	2
#define M1_CLK_BYTE_DISABLE	3

#define M1_GRF_BLOCK_SIZE 0xFFFF
#define GPIO_SIZE	32

#define M1_FUNC_GPIO 0
#define M1_FUNC_PWM 1

// GPIO[0]
#define M1_GPIO_0_BASE	0xFDD60000
// to control clock (PMU_CRU)
#define M1_PMU_CRU_BASE	0xFDD00000
#define M1_PMU_CRU_GPIO_CLK_OFFSET	0x0184
#define M1_PMU_CRU_GPIO_PCLK_BIT	9
// to control IOMUX
#define M1_PMU_GRF_BASE	0xFDC20000
#define M1_PMU_GRF_IOMUX_OFFSET	0x0000
#define M1_PMU_GRF_PUPD_OFFSET	0x0020
#define M1_PMU_GRF_DS_OFFSET	0x0070

// GPIO[1:4]
#define M1_GPIO_1_BASE	0xFE740000
#define M1_GPIO_2_BASE	0xFE750000
#define M1_GPIO_3_BASE	0xFE760000
#define M1_GPIO_4_BASE	0xFE770000
// to control clock (SYS_CRU)
#define M1_CRU_BASE	0xFDD20000
#define M1_CRU_GPIO_CLK_OFFSET	0x037C
#define M1_CRU_GPIO_PCLK_BIT	2
// to control IOMUX
#define M1_SYS_GRF_BASE	0xFDC60000
#define M1_SYS_GRF_IOMUX_OFFSET	0x0000
#define M1_SYS_GRF_PUPD_OFFSET	0x0080
#define M1_SYS_GRF_DS_OFFSET	0x0200

// Common offset for GPIO registers from each GPIO bank's base address
#define M1_GPIO_DIR_OFFSET	0x0008
#define M1_GPIO_SET_OFFSET	0x0000
#define M1_GPIO_GET_OFFSET	0x0070

// GPIO DS LEVELS
#define DS_LEVEL_0	0x01 //0b000001
#define DS_LEVEL_1	0x03 //0b000011
#define DS_LEVEL_2	0x07 //0b000111
#define DS_LEVEL_3	0x0f //0b001111
#define DS_LEVEL_4	0x1f //0b011111
#define DS_LEVEL_5	0x3f //0b111111

// GPIO write mask for WriteByte
#define WRITE_BYTE_MASK_GPIO0_H	0x00030000
#define WRITE_BYTE_MASK_GPIO0_L	0x40000000
#define WRITE_BYTE_MASK_GPIO3_H	0x03C00000
#define WRITE_BYTE_MASK_GPIO3_L	0x04000000

// PWM
#define M1_PWM_BASE 0xFDD70000
#define M1_PWM9_BASE 0xFE6F0000
#define M1_PWM_SCALE 0x0c
#define M1_PWM_READY 0x10
#define M1_PWM_EN 0x0b
#define M1_PWM_LOCK 0x08
// PWM offset
#define M1_PWM1_PERIOD_OFFSET 0x14
#define M1_PWM1_DUTY_OFFSET 0x18
#define M1_PWM1_CTRL_OFFSET 0x1c
#define M1_PWM2_PERIOD_OFFSET 0x24
#define M1_PWM2_DUTY_OFFSET 0x28
#define M1_PWM2_CTRL_OFFSET 0x2c
#define M1_PWM9_PERIOD_OFFSET 0x14
#define M1_PWM9_DUTY_OFFSET 0x18
#define M1_PWM9_CTRL_OFFSET 0x1c

#define CONSUMER "consumer"

#define M1_PWM_INTERNAL_CLK			24000000 // 24MHz

#ifdef __cplusplus
extern "C" {
#endif

extern void init_odroidm1 (struct libodroid *libwiring);

#ifdef __cplusplus
}
#endif
/*----------------------------------------------------------------------------*/
#endif	/* __ODROID_M1_H__ */
/*----------------------------------------------------------------------------*/

