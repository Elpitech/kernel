// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal Electronics BE-M1000 DesignWare HDMI 2.0 Tx PHY support driver
 *
 * Copyright (C) 2019-2022 Baikal Electronics, JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <drm/drm_modes.h>

#include <drm/bridge/dw_hdmi.h>

int fixed_clock = 0;
int max_clock = 0;

static const struct dw_hdmi_mpll_config baikal_hdmi_mpll_cfg[] = {
	/* pixelclk      opmode  gmp         */
	{  44900000, { { 0x00b3, 0x0000 }, }, },
	{  90000000, { { 0x0072, 0x0001 }, }, },
	{ 182750000, { { 0x0051, 0x0002 }, }, },
	{ 340000000, { { 0x0040, 0x0003 }, }, },
	{ 594000000, { { 0x1a40, 0x0003 }, }, },
	{ ~0UL,      { { 0x0000, 0x0000 }, }, }
};

static const struct dw_hdmi_curr_ctrl baikal_hdmi_cur_ctr[] = {
	/* pixelclk    current   */
	{  44900000, { 0x0000, }, },
	{  90000000, { 0x0008, }, },
	{ 182750000, { 0x001b, }, },
	{ 340000000, { 0x0036, }, },
	{ 594000000, { 0x003f, }, },
	{ ~0UL,      { 0x0000, }, }
};

static const struct dw_hdmi_phy_config baikal_hdmi_phy_cfg[] = {
	/* pixelclk  symbol  term    vlev */
	{ 148250000, 0x8009, 0x0004, 0x0232},
	{ 218250000, 0x8009, 0x0004, 0x0230},
	{ 288000000, 0x8009, 0x0004, 0x0273},
	{ 340000000, 0x8029, 0x0004, 0x0273},
	{ 594000000, 0x8039, 0x0004, 0x014a},
	{ ~0UL,      0x0000, 0x0000, 0x0000}
};

static enum drm_mode_status
baikal_hdmi_mode_valid(struct dw_hdmi *hdmi, void *data,
                const struct drm_display_info *info,
                const struct drm_display_mode *mode)
{
	if (mode->clock < 13500)
		return MODE_CLOCK_LOW;
	if (mode->clock >= 340000)
		return MODE_CLOCK_HIGH;
	if (fixed_clock && mode->clock != fixed_clock)
		return MODE_BAD;
	if (max_clock && mode->clock > max_clock)
		return MODE_BAD;

	return MODE_OK;
}

static struct dw_hdmi_plat_data baikal_dw_hdmi_plat_data = {
	.mpll_cfg   = baikal_hdmi_mpll_cfg,
	.cur_ctr    = baikal_hdmi_cur_ctr,
	.phy_config = baikal_hdmi_phy_cfg,
	.mode_valid = baikal_hdmi_mode_valid,
};

static int baikal_dw_hdmi_probe(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi;
	hdmi = dw_hdmi_probe(pdev, &baikal_dw_hdmi_plat_data);
	if (IS_ERR(hdmi)) {
		return PTR_ERR(hdmi);
	} else {
		return 0;
	}
}

static int baikal_dw_hdmi_remove(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi = platform_get_drvdata(pdev);
	dw_hdmi_remove(hdmi);
	return 0;
}

static const struct of_device_id baikal_dw_hdmi_of_table[] = {
	{ .compatible = "baikal,hdmi" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, baikal_dw_hdmi_of_table);

static struct platform_driver baikal_dw_hdmi_platform_driver = {
	.probe		= baikal_dw_hdmi_probe,
	.remove		= baikal_dw_hdmi_remove,
	.driver		= {
		.name	= "baikal-dw-hdmi",
		.of_match_table = baikal_dw_hdmi_of_table,
	},
};

module_param(fixed_clock, int, 0644);
module_param(max_clock, int, 0644);

module_platform_driver(baikal_dw_hdmi_platform_driver);

MODULE_AUTHOR("Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal BE-M1000 SoC DesignWare HDMI 2.0 Tx + Gen2 PHY Driver");
MODULE_LICENSE("GPL");
