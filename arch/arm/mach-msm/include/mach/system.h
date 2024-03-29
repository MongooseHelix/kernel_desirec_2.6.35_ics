/* arch/arm/mach-msm/include/mach/system.h
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_SYSTEM_H
#define __ASM_ARCH_MSM_SYSTEM_H

#include <mach/hardware.h>

void arch_idle(void);

#ifdef CONFIG_ARCH_MSM8X60
void arch_reset(char mode, const char *cmd);
#else
static inline void arch_reset(char mode, const char *cmd)
{
	for (;;) ;  /* depends on IPC w/ other core */
}
#endif

/* low level hardware reset hook -- for example, hitting the
 * PSHOLD line on the PMIC to hard reset the system
 */
extern void (*msm_hw_reset_hook)(void);

void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat, int clk_str, int dat_str);
void msm_i2c_gpio_init(void);

void set_melfas_reset_pin(int gpio);
void reset_melfas(void);
#endif

