/* linux/arch/arm/mach-msm/board-desirec-rfkill.c
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Control bluetooth power for desirec platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include "gpio_chip.h"
#include "proc_comm.h"
#include "board-desirec.h"

extern int desirec_bt_fastclock_power(int on);

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6350";

static int desirec_bt_status;

/*
 * Info on the following tables:
 * - DESIREC_GPIO_UART1 corresponds to the bluetooth device
 * - With DESIREC_GPIO_WB_SHUT_DOWN_N we can control the power-mode
 */
static uint32_t desirec_bt_init_table[] = {
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_CTS, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RX,  0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_TX,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(DESIREC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static uint32_t desirec_bt_on_table[] = {
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_CTS, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RX,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_TX,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(DESIREC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static uint32_t desirec_bt_off_table[] = {
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_CTS, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RX,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_TX,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	
	PCOM_GPIO_CFG(DESIREC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN,
			GPIO_8MA),
};

static uint32_t desirec_bt_disable_active_table[] = {
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_CTS, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RX,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_TX,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
};

static uint32_t desirec_bt_disable_sleep_table[] = {
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_CTS, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_RX,  0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(DESIREC_GPIO_UART1_TX,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void desirec_config_bt_init(void)
{
	desirec_bt_status = 0;
	config_bt_table(desirec_bt_init_table, ARRAY_SIZE(desirec_bt_init_table));
	mdelay(5);
	gpio_configure(DESIREC_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
}

static void desirec_config_bt_on(void)
{
	config_bt_table(desirec_bt_on_table, ARRAY_SIZE(desirec_bt_on_table));
	mdelay(2);

	gpio_configure(DESIREC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(15);
	gpio_configure(DESIREC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);
	gpio_configure(DESIREC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(1);

	desirec_bt_fastclock_power(1);
	mdelay(2);
	desirec_bt_status = 1;
}

static void desirec_config_bt_off(void)
{
	gpio_configure(DESIREC_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	desirec_bt_fastclock_power(0);
	config_bt_table(desirec_bt_off_table, ARRAY_SIZE(desirec_bt_off_table));
	mdelay(5);
	desirec_bt_status = 0;
}

void desirec_config_bt_disable_active(void)
{	
	config_bt_table(desirec_bt_disable_active_table, ARRAY_SIZE(desirec_bt_disable_active_table));
}

void desirec_config_bt_disable_sleep(void)
{
	config_bt_table(desirec_bt_disable_sleep_table, ARRAY_SIZE(desirec_bt_disable_sleep_table));
	mdelay(5);
	gpio_configure(DESIREC_GPIO_UART1_RTS, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	gpio_configure(DESIREC_GPIO_UART1_TX, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
}

int desirec_is_bluetooth_off(void)
{
	return !desirec_bt_status;	//ON:1, OFF:0
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		desirec_config_bt_on();
	} else {
		desirec_config_bt_off();
	}
	return 0;
}

static struct rfkill_ops desirec_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int desirec_rfkill_probe(struct platform_device *pdev)
{
	int rc;
	bool default_state = true;  /* off */

	desirec_config_bt_init();	/* bt gpio initial config */

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &desirec_rfkill_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	/* userspace cannot take exclusive control */
	rfkill_set_states(bt_rfk, default_state, false);

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_destroy(bt_rfk);
	return rc;
}

static int desirec_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);

	return 0;
}

static struct platform_driver desirec_rfkill_driver = {
	.probe = desirec_rfkill_probe,
	.remove = desirec_rfkill_remove,
	.driver = {
		.name = "desirec_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init desirec_rfkill_init(void)
{
	return platform_driver_register(&desirec_rfkill_driver);
}

static void __exit desirec_rfkill_exit(void)
{
	platform_driver_unregister(&desirec_rfkill_driver);
}

module_init(desirec_rfkill_init);
module_exit(desirec_rfkill_exit);
MODULE_DESCRIPTION("desirec rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
