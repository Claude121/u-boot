// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2014 Samsung Electronics
 * Przemyslaw Marczak <p.marczak@samsung.com>
 */

#include <common.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/power.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/arch/cpu.h>
#include <dm.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/s5m8767.h>
#include <errno.h>
#include <mmc.h>
#include <usb.h>
#include <usb/dwc2_udc.h>
#include <samsung/misc.h>

DECLARE_GLOBAL_DATA_PTR;

u32 get_board_rev(void)
{
	return 0;
}

static void board_clock_init(void)
{
	//remove
}

static void board_gpio_init(void)
{
#ifdef CONFIG_CMD_USB
	/* USB3503A Connect */
	gpio_request(EXYNOS4X12_GPIO_M33, "USB3503A Connect");

	/* USB3503A Reset */
	gpio_request(EXYNOS4X12_GPIO_M24, "USB3503A Reset");

    /* USB ETH */
    gpio_request(EXYNOS4X12_GPIO_C01, "DM9621");
#endif
}

int exynos_early_init_f(void)
{
	board_clock_init();

	return 0;
}

int exynos_init(void)
{
	board_gpio_init();

	return 0;
}

int exynos_power_init(void)
{
	return 0;
}

#ifdef CONFIG_USB_GADGET
static int s5pc210_phy_control(int on)
{
/*
	struct udevice *dev;
	int ret;

	ret = regulator_get_by_platname("VDD_UOTG_3.0V", &dev);
	if (ret) {
		pr_err("Regulator get error: %d\n", ret);
		return ret;
	}

	if (on)
		return regulator_set_mode(dev, OPMODE_ON);
	else
		return regulator_set_mode(dev, OPMODE_LPM);
*/
	return 0;
}

struct dwc2_plat_otg_data s5pc210_otg_data = {
	.phy_control	= s5pc210_phy_control,
	.regs_phy	= EXYNOS4X12_USBPHY_BASE,
	.regs_otg	= EXYNOS4X12_USBOTG_BASE,
	.usb_phy_ctrl	= EXYNOS4X12_USBPHY_CONTROL,
	.usb_flags	= PHY0_SLEEP,
};
#endif

#if defined(CONFIG_USB_GADGET) || defined(CONFIG_CMD_USB)

int board_usb_init(int index, enum usb_init_type init)
{
#ifdef CONFIG_CMD_USB
	struct udevice *dev;
	int ret;

	/* Itop4412 Us have it at 12MHz */
printf("setting gpio\n");
	/* Disconnect, Reset, Connect */
	gpio_direction_output(EXYNOS4X12_GPIO_M33, 0);
	gpio_direction_output(EXYNOS4X12_GPIO_M24, 0);
	gpio_direction_output(EXYNOS4X12_GPIO_M24, 1);
	gpio_direction_output(EXYNOS4X12_GPIO_M33, 1);

    gpio_direction_output(EXYNOS4X12_GPIO_C01, 0);
    udelay(7000);
    gpio_direction_output(EXYNOS4X12_GPIO_C01, 1);
#endif
	debug("USB_udc_probe\n");
	return dwc2_udc_probe(&s5pc210_otg_data);
}
#endif
