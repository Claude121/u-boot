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

static int exynos_set_regulator(const char *name, uint uv)
{
	struct udevice *dev;
	int ret;

	ret = regulator_get_by_platname(name, &dev);
	if (ret) {
		debug("%s: Cannot find regulator %s\n", __func__, name);
		return ret;
	}
	ret = regulator_set_value(dev, uv);
	if (ret) {
		debug("%s: Cannot set regulator %s\n", __func__, name);
		return ret;
	}

	return 0;
}

int exynos_power_init(void)
{
	struct udevice *dev;
	int ret;

	ret = pmic_get("s5m8767-pmic", &dev);
	if (!ret)
		s5m8767_enable_32khz_cp(dev);

	if (ret == -ENODEV)
		return 0;

	ret = regulators_enable_boot_on(false);
	if (ret)
		return ret;

	ret = exynos_set_regulator("vdd_mif", 1100000);
	if (ret)
		return ret;

	ret = exynos_set_regulator("vdd_arm", 1300000);
	if (ret)
		return ret;
	ret = exynos_set_regulator("vdd_int", 1012500);
	if (ret)
		return ret;
	ret = exynos_set_regulator("vdd_g3d", 1200000);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_USB_GADGET
static int s5pc210_phy_control(int on)
{
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
	if(init == USB_INIT_HOST)
	{
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
		return 0;
	} else if(init == USB_INIT_DEVICE)
	{
		debug("USB_udc_probe\n");
		return dwc2_udc_probe(&s5pc210_otg_data);
	} else
		return -EINVAL;
}
#endif
