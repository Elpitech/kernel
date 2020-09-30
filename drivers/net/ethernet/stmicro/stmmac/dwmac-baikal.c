// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal Electronics SoCs DWMAC glue layer
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
 * Author:
 *   Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 * All bugs by Alexey Sheplyakov <asheplyakov@altlinux.org>
 */

#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include "stmmac.h"
#include "stmmac_platform.h"
#include "common.h"
#include "dwmac_dma.h"

#define MAC_GPIO	0x000000e0	/* GPIO register */
#define MAC_GPIO_GPO0	(1 << 8)	/* 0-output port */

struct baikal_dwmac {
	struct device	*dev;
	struct clk	*tx2_clk;
};

static struct stmmac_dma_ops baikal_dma_ops;

static int baikal_dwmac_dma_reset(void __iomem *ioaddr)
{
	int err;
	u32 value = readl(ioaddr + DMA_BUS_MODE);

	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);

	udelay(10);
	/* Clear PHY reset */
	value = readl(ioaddr + MAC_GPIO);
	value |= MAC_GPIO_GPO0;
	writel(value, ioaddr + MAC_GPIO);
	pr_info("PHY re-inited for Baikal DWMAC\n");

	err = readl_poll_timeout(ioaddr + DMA_BUS_MODE, value,
				 !(value & DMA_BUS_MODE_SFT_RESET),
				 10000, 1000000);
	if (err)
		return -EBUSY;

	return 0;
}

static void baikal_dwmac_fix_mac_speed(void *priv, unsigned int speed)
{
	struct baikal_dwmac *dwmac = priv;
	unsigned long tx2_clk_freq = 0;

	dev_dbg(dwmac->dev, "fix_mac_speed new speed %u\n", speed);
	switch (speed) {
	case SPEED_1000:
		tx2_clk_freq = 250000000;
		break;
	case SPEED_100:
		tx2_clk_freq = 50000000;
		break;
	case SPEED_10:
		tx2_clk_freq = 5000000;
		break;
	}

	if (dwmac->tx2_clk && tx2_clk_freq != 0) {
		dev_dbg(dwmac->dev, "setting TX2 clock frequency to %lu\n",
			tx2_clk_freq);
		clk_set_rate(dwmac->tx2_clk, tx2_clk_freq);
	}
}

static int dwmac_baikal_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct baikal_dwmac *dwmac;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_warn(&pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	dwmac->dev = &pdev->dev;
	dwmac->tx2_clk = devm_clk_get(dwmac->dev, "tx2_clk");
	if (IS_ERR(dwmac->tx2_clk)) {
		dev_warn(&pdev->dev, "coldn't get TX2 clock\n");
		dwmac->tx2_clk = NULL;
	}
	plat_dat->fix_mac_speed = baikal_dwmac_fix_mac_speed;
	plat_dat->bsp_priv = dwmac;

	plat_dat->has_gmac = 1;
	plat_dat->enh_desc = 1;
	plat_dat->tx_coe = 1;
	plat_dat->rx_coe = 1;
	// TODO: set CSR correct clock in dts!
	plat_dat->clk_csr = 3;

	dev_info(&pdev->dev, "Baikal Electronics DWMAC glue driver\n");

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_remove_config_dt;

	/* change dma_reset method to baikal specific */
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (priv && priv->hw && priv->hw->dma) {
		dev_info(&pdev->dev, "Updating dma_reset (was %ps)\n",
			 priv->hw->dma->reset);
		memcpy(&baikal_dma_ops, priv->hw->dma, sizeof(baikal_dma_ops));
		baikal_dma_ops.reset = baikal_dwmac_dma_reset;
		priv->hw->dma = &baikal_dma_ops;
	}

	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	return ret;
}

static const struct of_device_id dwmac_baikal_match[] = {
	{ .compatible = "be,dwmac-3.710"},
	{ .compatible = "be,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_baikal_match);

static struct platform_driver dwmac_baikal_driver = {
	.probe  = dwmac_baikal_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name		= "baikal-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_baikal_match),
	},
};
module_platform_driver(dwmac_baikal_driver);

MODULE_DESCRIPTION("Baikal dwmac glue driver");
MODULE_LICENSE("GPL v2");

