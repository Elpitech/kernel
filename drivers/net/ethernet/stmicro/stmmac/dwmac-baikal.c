// SPDX-License-Identifier: GPL-2.0
/*
 * Baikal Electronics SoCs DWMAC glue layer
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
 * Author:
 *   Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 * All bugs by Alexey Sheplyakov <asheplyakov@altlinux.org>
 */

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
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

#define BAIKAL_SMC_GMAC_DIV2_ENABLE	0x82000500
#define BAIKAL_SMC_GMAC_DIV2_DISABLE	0x82000501

struct baikal_gmac {
	struct device	*dev;
	uint64_t	base;
	struct clk	*tx2_clk;
	int		has_aux_div2;
};

static struct stmmac_dma_ops baikal_dma_ops;

static int baikal_dwmac_dma_reset(void __iomem *ioaddr)
{
	int err;
	u32 value;

	/* DMA SW reset */
	value = readl(ioaddr + DMA_BUS_MODE);
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

static void baikal_gmac_fix_mac_speed(void *priv, unsigned int speed)
{
	struct arm_smccc_res res;
	struct baikal_gmac *gmac = priv;
	unsigned long tx2_clk_freq = 0;

	dev_dbg(gmac->dev, "fix_mac_speed new speed %u\n", speed);
	switch (speed) {
	case SPEED_1000:
		tx2_clk_freq = 250000000;
		if (gmac->has_aux_div2) {
			arm_smccc_smc(BAIKAL_SMC_GMAC_DIV2_DISABLE,
				      gmac->base, 0, 0, 0, 0, 0, 0, &res);
		}
		break;
	case SPEED_100:
		tx2_clk_freq = 50000000;
		if (gmac->has_aux_div2) {
			arm_smccc_smc(BAIKAL_SMC_GMAC_DIV2_DISABLE,
				      gmac->base, 0, 0, 0, 0, 0, 0, &res);
		}
		break;
	case SPEED_10:
		tx2_clk_freq = 5000000;
		if (gmac->has_aux_div2) {
			tx2_clk_freq = 10000000;
			arm_smccc_smc(BAIKAL_SMC_GMAC_DIV2_ENABLE,
				      gmac->base, 0, 0, 0, 0, 0, 0, &res);
		}
		break;
	}

	if (gmac->tx2_clk && tx2_clk_freq != 0) {
		dev_dbg(gmac->dev, "setting TX2 clock frequency to %lu\n",
			tx2_clk_freq);
		clk_set_rate(gmac->tx2_clk, tx2_clk_freq);
	}
}

static int dwmac_baikal_init(struct platform_device *pdev, void *plat_priv)
{
	struct baikal_gmac *gmac = plat_priv;

	clk_prepare_enable(gmac->tx2_clk);

	return 0;
}

static void dwmac_baikal_exit(struct platform_device *pdev, void *plat_priv)
{
	struct baikal_gmac *gmac = plat_priv;

	clk_disable_unprepare(gmac->tx2_clk);
}

static int dwmac_baikal_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct resource *res;
	struct baikal_gmac *gmac;
	u32 value;
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

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	gmac->dev = &pdev->dev;
	gmac->tx2_clk = devm_clk_get(gmac->dev, "tx2_clk");
	if (IS_ERR(gmac->tx2_clk)) {
		dev_warn(&pdev->dev, "coldn't get TX2 clock\n");
		gmac->tx2_clk = NULL;
	}
	clk_prepare_enable(gmac->tx2_clk);

	if (gmac->dev->of_node &&
	     of_device_is_compatible(gmac->dev->of_node, "baikal,bs1000-gmac")) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		gmac->base = res->start;
		gmac->has_aux_div2 = 1;
	} else {
		gmac->has_aux_div2 = 0;
	}

	plat_dat->fix_mac_speed = baikal_gmac_fix_mac_speed;
	plat_dat->bsp_priv = gmac;

	plat_dat->has_gmac = 1;
	plat_dat->enh_desc = 1;
	plat_dat->tx_coe = 1;
	plat_dat->rx_coe = 1;
	plat_dat->exit = dwmac_baikal_exit;
	plat_dat->init = dwmac_baikal_init;
	// TODO: set CSR correct clock in dts!
	plat_dat->clk_csr = 3;

	dev_info(&pdev->dev, "Baikal Electronics DWMAC glue driver\n");

	/* Clear PHY reset now */
	value = readl(stmmac_res.addr + MAC_GPIO);
	value |= MAC_GPIO_GPO0;
	writel(value, stmmac_res.addr + MAC_GPIO);

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
	{ .compatible = "baikal,bm1000-gmac" },
	{ .compatible = "baikal,bs1000-gmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_baikal_match);

static struct platform_driver dwmac_baikal_driver = {
	.probe  = dwmac_baikal_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name		= "baikal-gmac-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_baikal_match),
	},
};
module_platform_driver(dwmac_baikal_driver);

MODULE_DESCRIPTION("Baikal dwmac glue driver");
MODULE_LICENSE("GPL v2");

