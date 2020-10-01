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
 * baikal_vdu_gem.c
 * Implementation of the GEM functions for Baikal Electronics
 * BE-M1000 VDU driver
 */
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include "baikal_vdu_drm.h"

int baikal_vdu_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	return drm_gem_cma_dumb_create_internal(file_priv, dev, args);
}
