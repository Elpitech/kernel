/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
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

struct baikal_vdu_drm_connector {
	struct drm_connector connector;
	struct drm_panel *panel;
};

struct baikal_vdu_private {
	struct drm_device *drm;

	struct baikal_vdu_drm_connector connector;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_plane primary;

	void *regs;
	struct clk *clk;
	spinlock_t lock;
	u32 counters[20];
	int mode_fixup;

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

	struct delayed_work update_work;
};

#define to_baikal_vdu_drm_connector(x) \
	container_of(x, struct baikal_vdu_drm_connector, connector)

extern const struct drm_encoder_funcs baikal_vdu_encoder_funcs;

/* CRTC Functions */
int baikal_vdu_crtc_create(struct drm_device *dev);
irqreturn_t baikal_vdu_irq(int irq, void *data);

int baikal_vdu_primary_plane_init(struct drm_device *dev);

void baikal_vdu_wait_off(struct baikal_vdu_private *priv);

/* Connector Functions */
int baikal_vdu_connector_create(struct drm_device *dev);

/* Encoder Functions */
int baikal_vdu_encoder_init(struct drm_device *dev);

/* GEM Functions */
int baikal_vdu_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev,
		      struct drm_mode_create_dumb *args);

/* Worker functions */
void baikal_vdu_update_work(struct work_struct *work);

#endif /* __BAIKAL_VDU_DRM_H__ */
