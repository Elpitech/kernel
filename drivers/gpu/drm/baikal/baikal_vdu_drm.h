/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#ifndef __BAIKAL_VDU_DRM_H__
#define __BAIKAL_VDU_DRM_H__

#include <drm/drm_gem.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/workqueue.h>

struct clk;
struct drm_device;
struct drm_fbdev_cma;
struct drm_panel;

struct baikal_vdu_private {
	struct drm_device *drm;

	struct drm_crtc crtc;
	struct drm_encoder *encoder; //not used
	struct drm_bridge *bridge;   //not used
	struct drm_plane primary;

	void *regs;
	struct clk *clk;
	spinlock_t lock;
	u32 counters[20];

	int type;
#define VDU_TYPE_HDMI	0
#define VDU_TYPE_LVDS	1
	int num_lanes; /* connected lanes (1, 2 or 4) - used for vdu_lvds */
	int state; /* 0 - disabled, 1 - running */

	u32 cr1_cfg; /* persistent bits for CR1 */
	u32 lvds_gpior;	/* GPIOR config */
	u32 fb_addr;
	u32 fb_end;

	struct wait_queue_head queue;
};

/* CRTC Functions */
int baikal_vdu_crtc_create(struct drm_device *dev);
irqreturn_t baikal_vdu_irq(int irq, void *data);

int baikal_vdu_primary_plane_init(struct drm_device *dev);

void baikal_vdu_wait_off(struct baikal_vdu_private *priv);

#endif /* __BAIKAL_VDU_DRM_H__ */
