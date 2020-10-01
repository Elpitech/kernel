// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 * All bugs by Alexey Sheplyakov <asheplyakov@altlinux.org>
 *
 * This driver is based on ARM PL111 DRM driver
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
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
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_irq.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define DRIVER_NAME                 "baikal-vdu"
#define DRIVER_DESC                 "DRM module for Baikal VDU"
#define DRIVER_DATE                 "20200131"

#define BAIKAL_SMC_SCP_LOG_DISABLE  0x82000200

int mode_fixup;

static struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int vdu_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct arm_smccc_res res;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	drm_mode_config_init(dev);
	mode_config = &dev->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 4096;
	mode_config->min_height = 1;
	mode_config->max_height = 4096;

	ret = baikal_vdu_primary_plane_init(dev);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init primary plane\n");
		goto out_config;
	}

	ret = baikal_vdu_crtc_create(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create crtc\n");
		goto out_config;
	}

	ret = drm_of_find_panel_or_bridge(dev->dev->of_node, -1, -1,
					  &priv->connector.panel,
					  &priv->bridge);
	if (ret == -EPROBE_DEFER) {
		dev_info(dev->dev, "Bridge probe deferred\n");
		goto out_config;
	}

	ret = baikal_vdu_encoder_init(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create DRM encoder\n");
		goto out_config;
	}

	if (priv->bridge) {
		priv->bridge->encoder = &priv->encoder;
		ret = drm_bridge_attach(&priv->encoder, priv->bridge, NULL);
		if (ret) {
			dev_err(dev->dev, "Failed to attach DRM bridge %d\n",
				ret);
			goto out_config;
		}
	} else if (priv->connector.panel) {
		ret = baikal_vdu_connector_create(dev);
		if (ret) {
			dev_err(dev->dev, "Failed to create DRM connector\n");
			goto out_config;
		}
		ret = drm_connector_attach_encoder(&priv->connector.connector,
						&priv->encoder);
		if (ret != 0) {
			dev_err(dev->dev, "Failed to attach encoder\n");
			goto out_config;
		}
	} else
		ret = -EINVAL;

	if (ret) {
		dev_err(dev->dev, "No bridge or panel attached!\n");
		goto out_config;
	}

	priv->clk = clk_get(dev->dev, "pclk");
	if (IS_ERR(priv->clk)) {
		dev_err(dev->dev, "fatal: unable to get pclk, err %ld\n",
			PTR_ERR(priv->clk));
		ret = PTR_ERR(priv->clk);
		goto out_config;
	}

	priv->mode_fixup = mode_fixup;

	drm_fb_helper_remove_conflicting_framebuffers(NULL, "baikal-vdudrmfb",
						      false);

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init vblank\n");
		goto out_clk;
	}

	arm_smccc_smc(BAIKAL_SMC_SCP_LOG_DISABLE, 0, 0, 0, 0, 0, 0, 0, &res);
	INIT_DEFERRABLE_WORK(&priv->update_work,
			     baikal_vdu_update_work);

	drm_mode_config_reset(dev);

	drm_kms_helper_poll_init(dev);

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto out_clk;

	drm_fbdev_generic_setup(dev, 32);
	goto finish;

out_clk:
	clk_put(priv->clk);
out_config:
	drm_mode_config_cleanup(dev);
finish:
	return ret;
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.read = drm_read,
};

static struct drm_driver vdu_drm_driver = {
	.driver_features = DRIVER_HAVE_IRQ | DRIVER_GEM |
			DRIVER_MODESET | DRIVER_ATOMIC,
	.irq_handler = baikal_vdu_irq,
	.ioctls = NULL,
	.fops = &drm_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	.dumb_create = baikal_vdu_dumb_create,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.enable_vblank = baikal_vdu_enable_vblank,
	.disable_vblank = baikal_vdu_disable_vblank,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = baikal_vdu_debugfs_init,
#endif
};

static int baikal_vdu_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_vdu_private *priv;
	struct drm_device *drm;
	struct resource *mem;
	int irq;
	int ret;

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

	ret = drm_irq_install(drm, irq);
	if (ret != 0) {
		dev_err(dev, "%s IRQ %d allocation failed\n", __func__, irq);
		return ret;
	}

	if (pdev->dev.of_node &&
	    of_property_read_bool(pdev->dev.of_node, "lvds-out")) {
		priv->type = VDU_TYPE_LVDS;
		ret = of_property_read_u32(pdev->dev.of_node, "num-lanes",
					   &priv->num_lanes);
		if (ret)
			priv->num_lanes = 1;
	}

	ret = vdu_modeset_init(drm);
	if (ret != 0) {
		dev_err(dev, "Failed to init modeset\n");
		goto dev_unref;
	}

	return 0;

dev_unref:
	drm_irq_uninstall(drm);
	drm->dev_private = NULL;
	drm_dev_put(drm);
	return ret;
}

static int baikal_vdu_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	drm_mode_config_cleanup(drm);
	drm_irq_uninstall(drm);
	drm->dev_private = NULL;
	drm_dev_put(drm);

	return 0;
}

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
	},
};

module_param(mode_fixup, int, 0644);

module_platform_driver(baikal_vdu_platform_driver);

MODULE_AUTHOR("Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics BE-M1000 Video Display Unit (VDU) DRM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
