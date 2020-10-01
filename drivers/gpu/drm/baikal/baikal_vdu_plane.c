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

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define BAIKAL_SMC_VDU_UPDATE_HDMI	0x82000100

void baikal_vdu_update_work(struct work_struct *work)
{
	struct arm_smccc_res res;
	unsigned long flags;
	struct baikal_vdu_private *priv =
		container_of(work, struct baikal_vdu_private,
			     update_work.work);
	int count = 0;
	u64 t1, t2;

	t1 = read_sysreg(CNTVCT_EL0);
	spin_lock_irqsave(&priv->lock, flags);
	arm_smccc_smc(BAIKAL_SMC_VDU_UPDATE_HDMI, priv->fb_addr, priv->fb_end,
		      0, 0, 0, 0, 0, &res);
	spin_unlock_irqrestore(&priv->lock, flags);
	if (res.a0 == -EBUSY)
		priv->counters[15]++;
	else
		priv->counters[16]++;
	while (res.a0 == -EBUSY && count < 10) {
		count++;
		usleep_range(10000, 20000);
		res.a0 = 0;
		spin_lock_irqsave(&priv->lock, flags);
		arm_smccc_smc(BAIKAL_SMC_VDU_UPDATE_HDMI, priv->fb_addr,
			      priv->fb_end, 0, 0, 0, 0, 0, &res);
		spin_unlock_irqrestore(&priv->lock, flags);
		if (res.a0 == -EBUSY)
			priv->counters[15]++;
		else
			priv->counters[16]++;
	}
	t2 = read_sysreg(CNTVCT_EL0);
	priv->counters[17] = t2 - t1;
	priv->counters[18] = count;
	priv->counters[19]++;
}

static int baikal_vdu_primary_plane_atomic_check(struct drm_plane *plane,
					    struct drm_plane_state *state)
{
	struct drm_device *dev = plane->dev;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_crtc_state *crtc_state;
	struct drm_display_mode *mode;
	int rate, ret;

	if (!state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(state->state, state->crtc);
	mode = &crtc_state->adjusted_mode;
	rate = mode->clock * 1000;
	if (rate == clk_get_rate(priv->clk))
		return 0;

	if (__clk_is_enabled(priv->clk))
		clk_disable_unprepare(priv->clk);
	ret = clk_set_rate(priv->clk, rate);
	DRM_DEV_DEBUG_DRIVER(dev->dev, "Requested pixel clock is %d Hz\n",
			     rate);

	if (ret < 0) {
		DRM_ERROR("Cannot set desired pixel clock (%d Hz)\n",
			  rate);
		return -EINVAL;
	}
	clk_prepare_enable(priv->clk);
	if (!__clk_is_enabled(priv->clk)) {
		DRM_ERROR("PLL could not lock at desired frequency (%d Hz)\n",
			  rate);
		return -EINVAL;
	}
	return 0;
}

static void baikal_vdu_primary_plane_atomic_update(struct drm_plane *plane,
					      struct drm_plane_state *old_state)
{
	struct drm_device *dev = plane->dev;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	struct arm_smccc_res res;
	u32 cntl, addr, end;
	unsigned long flags;

	if (!fb)
		return;

	addr = drm_fb_cma_get_gem_addr(fb, state, 0);
	end = ((addr + fb->height * fb->pitches[0] - 1) & MRR_DEAR_MRR_MASK) |
		MRR_OUTSTND_RQ(4);

	if (priv->type == VDU_TYPE_HDMI) {
		spin_lock_irqsave(&priv->lock, flags);
		arm_smccc_smc(BAIKAL_SMC_VDU_UPDATE_HDMI, addr, end,
			      0, 0, 0, 0, 0, &res);
		spin_unlock_irqrestore(&priv->lock, flags);

		if (res.a0 == -EBUSY) {
			priv->counters[15]++;
			priv->fb_addr = addr;
			priv->fb_end = end;
			smp_wmb();
			schedule_delayed_work(&priv->update_work,
					      usecs_to_jiffies(250));
		} else
			priv->counters[16]++;
	} else { /* the above isn't supported for vdu_lvds */
		spin_lock_irqsave(&priv->lock, flags);
		priv->fb_addr = addr;
		priv->fb_end = end;
		writel(addr, priv->regs + DBAR);
		/* enable vblank (or call baikal_vdu_enable_vblank()?) */
		/* clear interrupt status */
		writel(0x3ffff, priv->regs + ISR);
		writel(INTR_VCT + INTR_FER, priv->regs + IMR);
		spin_unlock_irqrestore(&priv->lock, flags);
	}

	cntl = readl(priv->regs + CR1);
	cntl &= ~CR1_BPP_MASK;

	/* Note that the the hardware's format reader takes 'r' from
	 * the low bit, while DRM formats list channels from high bit
	 * to low bit as you read left to right.
	 */
	switch (fb->format->format) {
	case DRM_FORMAT_BGR888:
		cntl |= CR1_BPP24 | CR1_FBP | CR1_BGR;
		break;
	case DRM_FORMAT_RGB888:
		cntl |= CR1_BPP24 | CR1_FBP;
		break;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		cntl |= CR1_BPP24 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		cntl |= CR1_BPP24;
		break;
	case DRM_FORMAT_BGR565:
		cntl |= CR1_BPP16_565 | CR1_BGR;
		break;
	case DRM_FORMAT_RGB565:
		cntl |= CR1_BPP16_565;
		break;
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		cntl |= CR1_BPP16_555 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		cntl |= CR1_BPP16_555;
		break;
	default:
		WARN_ONCE(true, "Unknown FB format 0x%08x, set XRGB8888 instead\n",
				fb->format->format);
		cntl |= CR1_BPP24;
		break;
	}

	writel(cntl, priv->regs + CR1);
}

static const struct drm_plane_helper_funcs
baikal_vdu_primary_plane_helper_funcs = {
	.atomic_check = baikal_vdu_primary_plane_atomic_check,
	.atomic_update = baikal_vdu_primary_plane_atomic_update,
};

static const struct drm_plane_funcs baikal_vdu_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int baikal_vdu_primary_plane_init(struct drm_device *drm)
{
	struct baikal_vdu_private *priv = drm->dev_private;
	struct drm_plane *plane = &priv->primary;
	static const u32 formats[] = {
		DRM_FORMAT_BGR888,
		DRM_FORMAT_RGB888,
		DRM_FORMAT_ABGR8888,
		DRM_FORMAT_XBGR8888,
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_BGR565,
		DRM_FORMAT_RGB565,
		DRM_FORMAT_ABGR1555,
		DRM_FORMAT_XBGR1555,
		DRM_FORMAT_ARGB1555,
		DRM_FORMAT_XRGB1555,
	};
	int ret;

	ret = drm_universal_plane_init(drm, plane, 0,
				       &baikal_vdu_primary_plane_funcs,
				       formats,
				       ARRAY_SIZE(formats),
				       NULL,
				       DRM_PLANE_TYPE_PRIMARY,
				       NULL);
	if (ret)
		return ret;

	drm_plane_helper_add(plane, &baikal_vdu_primary_plane_helper_funcs);

	return 0;
}


