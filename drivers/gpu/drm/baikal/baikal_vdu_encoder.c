// SPDX-License-Identifier: GPL-2.0
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

/**
 * baikal_vdu_encoder.c
 * Implementation of the encoder functions for Baikal Electronics
 * BE-M1000 VDU driver
 */
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>

#include <drm/drm_crtc_helper.h>

#include "baikal_vdu_drm.h"

const struct drm_encoder_funcs baikal_vdu_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

int baikal_vdu_encoder_init(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_encoder *encoder = &priv->encoder;
	int ret;

	ret = drm_encoder_init(dev, encoder, &baikal_vdu_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;

	encoder->crtc = &priv->crtc;
	encoder->possible_crtcs = BIT(drm_crtc_index(encoder->crtc));

	return 0;
}
