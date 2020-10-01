// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal Electronics BE-M1000 DesignWare HDMI 2.0 Tx PHY support driver
 *
 * Copyright (C) 2019 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (C) 2016 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <drm/drm_modes.h>

#include <drm/bridge/dw_hdmi.h>

#define BAIKAL_HDMI_PHY_OPMODE_PLLCFG	0x06	/* Mode of operation and PLL dividers */
#define BAIKAL_HDMI_PHY_PLLCURRCTRL		0x10	/* PLL current */
#define BAIKAL_HDMI_PHY_PLLGMPCTRL		0x15	/* PLL Gmp (conductance) */
#define BAIKAL_HDMI_PHY_TXTERM			0x19	/* Rterm */
#define BAIKAL_HDMI_PHY_VLEVCTRL		0x0e	/* Voltage levels */
#define BAIKAL_HDMI_PHY_CKSYMTXCTRL		0x09	/* Tx symbols control and slope boost */

int fixed_clock;
int max_clock;

struct baikal_hdmi_phy_params {
	unsigned long mpixelclock;
	u16 opmode_div;
	u16 curr;
	u16 gmp;
	u16 txterm;
	u16 vlevctrl;
	u16 cksymtxctrl;
};

static const struct baikal_hdmi_phy_params baikal_hdmi_phy_params[] = {
	/* PCLK      opmode  current gmp     txter   vlevctrl cksymtxctrl */
	{ 44900000,  0x00b3, 0x0000, 0x0000, 0x0004, 0x0232,  0x8009 },
	{ 90000000,  0x0072, 0x0008, 0x0001, 0x0004, 0x0232,  0x8009 },
	{ 148250000, 0x0051, 0x001b, 0x0002, 0x0004, 0x0232,  0x8009 },
	{ 182750000, 0x0051, 0x001b, 0x0002, 0x0004, 0x0230,  0x8009 },
	{ 218250000, 0x0040, 0x0036, 0x0003, 0x0004, 0x0230,  0x8009 },
	{ 288000000, 0x0040, 0x0036, 0x0003, 0x0004, 0x0273,  0x8009 },
	{ 340000000, 0x0040, 0x0036, 0x0003, 0x0004, 0x0273,  0x8029 },
	{ 594000000, 0x1a40, 0x003f, 0x0003, 0x0004, 0x014a,  0x8039 },
	{ ~0UL },
};

static int baikal_hdmi_phy_configure(struct dw_hdmi *hdmi,
				   const struct dw_hdmi_plat_data *pdata,
				   unsigned long mpixelclock)
{
	const struct baikal_hdmi_phy_params *params = baikal_hdmi_phy_params;

	for (; params && params->mpixelclock != ~0UL; ++params) {
		if (mpixelclock <= params->mpixelclock)
			break;
	}

	if (params->mpixelclock == ~0UL)
		return -EINVAL;

	dw_hdmi_phy_i2c_write(hdmi, params->opmode_div,
				BAIKAL_HDMI_PHY_OPMODE_PLLCFG);
	dw_hdmi_phy_i2c_write(hdmi, params->curr,
				BAIKAL_HDMI_PHY_PLLCURRCTRL);
	dw_hdmi_phy_i2c_write(hdmi, params->gmp,
				BAIKAL_HDMI_PHY_PLLGMPCTRL);
	dw_hdmi_phy_i2c_write(hdmi, params->txterm,
				BAIKAL_HDMI_PHY_TXTERM);
	dw_hdmi_phy_i2c_write(hdmi, params->vlevctrl,
				BAIKAL_HDMI_PHY_VLEVCTRL);
	dw_hdmi_phy_i2c_write(hdmi, params->cksymtxctrl,
				BAIKAL_HDMI_PHY_CKSYMTXCTRL);

	return 0;
}

static enum drm_mode_status baikal_hdmi_mode_valid(struct drm_connector *con,
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

static const struct dw_hdmi_plat_data baikal_dw_hdmi_plat_data = {
	.configure_phy	= baikal_hdmi_phy_configure,
	.mode_valid	= baikal_hdmi_mode_valid,
};

static int baikal_dw_hdmi_bind(struct device *dev, struct device *master,
				void *data)
{
	struct dw_hdmi *hdmi;
	struct platform_device *pdev = to_platform_device(dev);

	hdmi = dw_hdmi_probe(pdev, &baikal_dw_hdmi_plat_data);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);
	return 0;
}

static void baikal_dw_hdmi_unbind(struct device *dev, struct device *master,
				  void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_hdmi *hdmi = platform_get_drvdata(pdev);

	dw_hdmi_unbind(hdmi);
}

static int baikal_dw_hdmi_probe(struct platform_device *pdev)
{
	struct dw_hdmi *hdmi;

	hdmi = dw_hdmi_probe(pdev, &baikal_dw_hdmi_plat_data);
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);
	else
		return 0;
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
