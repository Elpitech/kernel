// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 * All bugs by Alexey Sheplyakov <asheplyakov@altlinux.org>
 *
 */

#include <linux/arm-smccc.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_aperture.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define DRIVER_NAME                 "baikal-vdu"
#define DRIVER_DESC                 "DRM module for Baikal VDU"
#define DRIVER_DATE                 "20230404"

#define BAIKAL_SMC_SCP_LOG_DISABLE  0x82000200

static struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int vdu_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct arm_smccc_res res;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	drm_mode_config_init(dev);
	mode_config = &dev->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 4095;
	mode_config->min_height = 1;
	mode_config->max_height = 4095;

	ret = baikal_vdu_primary_plane_init(dev);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init primary plane\n");
		goto out_config;
	}

	ret = drm_of_find_panel_or_bridge(dev->dev->of_node, -1, -1,
					  &panel,
					  &bridge);
	if (ret == -EPROBE_DEFER) {
		dev_info(dev->dev, "Bridge probe deferred\n");
		goto out_config;
	}

	priv->clk = devm_clk_get(dev->dev, "pclk");
	if (IS_ERR(priv->clk)) {
		dev_err(dev->dev, "fatal: unable to get pclk, err %ld\n",
			PTR_ERR(priv->clk));
		ret = PTR_ERR(priv->clk);
		goto out_config;
	}

	ret = baikal_vdu_crtc_create(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create crtc\n");
		goto out_config;
	}

	encoder = drmm_plain_encoder_alloc(dev, NULL,
			(priv->type == VDU_TYPE_LVDS)?
			DRM_MODE_ENCODER_LVDS:
			DRM_MODE_ENCODER_TMDS,
			(priv->type == VDU_TYPE_LVDS)?"vdu-lvds":"vdu-hdmi");
	if (IS_ERR(encoder)) {
		dev_err(dev->dev, "Failed to create DRM encoder\n");
		ret = PTR_ERR(encoder);
		goto out_config;
	}
	encoder->crtc = &priv->crtc;
	encoder->possible_crtcs = BIT(drm_crtc_index(encoder->crtc));
	if (panel) {
		bridge = drm_panel_bridge_add(panel);
		if (IS_ERR(bridge)) {
			ret = PTR_ERR(bridge);
			dev_err(dev->dev, "Can't create panel bridge (%d)\n", ret);
			goto out_config;
		}
	}
	if (bridge) {
		priv->bridge = bridge;
		bridge->encoder = encoder;
		ret = drm_bridge_attach(encoder, bridge, NULL, 0);
		if (ret) {
			dev_err(dev->dev, "Failed to attach DRM bridge %d\n",
				ret);
			goto out_config;
		}
	}

	if (ret) {
		dev_err(dev->dev, "No bridge or panel attached!\n");
		goto out_config;
	}

	drm_aperture_remove_framebuffers(dev->driver);

	dev->vblank_disable_immediate = true;

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init vblank\n");
		goto out_config;
	}

	arm_smccc_smc(BAIKAL_SMC_SCP_LOG_DISABLE, 0, 0, 0, 0, 0, 0, 0, &res);

	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto out_config;

	drm_fbdev_generic_setup(dev, 32);
	goto finish;

out_config:
	drm_mode_config_cleanup(dev);
finish:
	return ret;
}

DEFINE_DRM_GEM_DMA_FOPS(drm_fops);

static struct drm_driver vdu_drm_driver = {
	.driver_features = DRIVER_HAVE_IRQ | DRIVER_GEM |
			DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &drm_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	DRM_GEM_DMA_DRIVER_OPS,
};

static int baikal_vdu_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_vdu_private *priv;
	struct drm_device *drm;
	struct resource *mem;
	const char *data_mapping;
	int irq;
	int ret;
	u32 reg;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = drm_dev_alloc(&vdu_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	platform_set_drvdata(pdev, drm);
	priv->drm = drm;
	drm->dev_private = priv;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "%s no MMIO resource specified\n", __func__);
		return -EINVAL;
	}

	priv->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(priv->regs)) {
		dev_err(dev, "%s MMIO allocation failed\n", __func__);
		return PTR_ERR(priv->regs);
	}

	/* turn off interrupts before requesting the irq */
	writel(0, priv->regs + IMR);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(dev, "%s no IRQ resource specified\n", __func__);
		return -EINVAL;
	}

	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->queue);

	ret = devm_request_irq(dev, irq, baikal_vdu_irq, 0, dev->driver->name, drm);
	if (ret != 0) {
		dev_err(dev, "%s IRQ %d allocation failed\n", __func__, irq);
		return ret;
	}

	priv->cr1_cfg = CR1_FDW_16_WORDS;

	if (pdev->dev.of_node &&
	    of_property_read_bool(pdev->dev.of_node, "lvds-out")) {
		priv->type = VDU_TYPE_LVDS;
		ret = of_property_read_u32(pdev->dev.of_node, "num-lanes",
					   &priv->num_lanes);
		if (ret)
			priv->num_lanes = 1;

		priv->lvds_gpior = GPIOR_UHD_ENB;
		if (priv->num_lanes == 4)
			priv->lvds_gpior |= GPIOR_UHD_QUAD_PORT;
		else if (priv->num_lanes == 2)
			priv->lvds_gpior |= GPIOR_UHD_DUAL_PORT;

		if (of_property_read_string(pdev->dev.of_node, "data-mapping",
					    &data_mapping)) {
			priv->cr1_cfg |= CR1_OPS_LCD24;
		} else if (!strncmp(data_mapping, "vesa-24", 8)) {
			priv->cr1_cfg |= CR1_OPS_LCD24;
		} else if (!strncmp(data_mapping, "jeida-18", 9)) {
			priv->cr1_cfg |= CR1_OPS_LCD18;
			priv->lvds_gpior |= GPIOR_UHD_FMT_JEIDA;
		} else {
			dev_warn(&pdev->dev,
				 "%s data mapping is not supported, vesa-24 is set\n",
				 data_mapping);
			priv->cr1_cfg |= CR1_OPS_LCD24;
		}
	} else {
		priv->cr1_cfg |= CR1_OPS_LCD24;
	}

	writel(0x3ffff, priv->regs + ISR);
	writel(INTR_LDD | INTR_FER, priv->regs + IMR);

	reg = readl(priv->regs + CR1);
	priv->state = !!(reg & CR1_LCE);

	ret = vdu_modeset_init(drm);
	if (ret != 0) {
		dev_err(dev, "Failed to init modeset\n");
		goto dev_unref;
	}

	return 0;

dev_unref:
	writel(0, priv->regs + IMR);
	writel(0x3ffff, priv->regs + ISR);
	drm->dev_private = NULL;
	drm_dev_put(drm);
	return ret;
}

static int baikal_vdu_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	drm_mode_config_cleanup(drm);
	drm->dev_private = NULL;
	drm_dev_put(drm);

	return 0;
}

static int baikal_vdu_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	dev_dbg(dev, "Suspend\n");
	drm_mode_config_helper_suspend(drm);

	return 0;
}

static int baikal_vdu_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	dev_dbg(dev, "Resume\n");

	drm_mode_config_helper_resume(drm);
	
	return 0;
}

static const struct dev_pm_ops baikal_vdu_pm_ops = {
	.suspend = baikal_vdu_suspend,
	.resume = baikal_vdu_resume,
};

static const struct of_device_id baikal_vdu_of_match[] = {
	{ .compatible = "baikal,vdu" },
	{ },
};

static struct platform_driver baikal_vdu_platform_driver = {
	.probe  = baikal_vdu_drm_probe,
	.remove = baikal_vdu_drm_remove,
	.driver = {
		.name   = DRIVER_NAME,
		.of_match_table = baikal_vdu_of_match,
		.pm = &baikal_vdu_pm_ops,
	},
};

module_platform_driver(baikal_vdu_platform_driver);

MODULE_AUTHOR("Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics BE-M1000 Video Display Unit (VDU) DRM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
