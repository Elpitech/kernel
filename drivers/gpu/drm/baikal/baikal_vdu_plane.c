// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

static void baikal_vdu_primary_plane_atomic_update(struct drm_plane *plane,
					      struct drm_atomic_state *old_state)
{
	struct drm_device *dev = plane->dev;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	u32 cntl, addr, end, reg;
	u32 fmt_ctl = 0;
	unsigned long flags;

	if (!fb)
		return;

	/*
	 * Prepare format mask and check if it differs from active one.
	 * In latter case we would better disable VDU to avoid scenario
	 * when bpp is changed on the fly.
	 */
	/* Note that the the hardware's format reader takes 'r' from
	 * the low bit, while DRM formats list channels from high bit
	 * to low bit as you read left to right.
	 */
	switch (fb->format->format) {
	case DRM_FORMAT_BGR888:
		fmt_ctl |= CR1_BPP24 | CR1_FBP | CR1_BGR;
		break;
	case DRM_FORMAT_RGB888:
		fmt_ctl |= CR1_BPP24 | CR1_FBP;
		break;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		fmt_ctl |= CR1_BPP24 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		fmt_ctl |= CR1_BPP24;
		break;
	case DRM_FORMAT_BGR565:
		fmt_ctl |= CR1_BPP16_565 | CR1_BGR;
		break;
	case DRM_FORMAT_RGB565:
		fmt_ctl |= CR1_BPP16_565;
		break;
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		fmt_ctl |= CR1_BPP16_555 | CR1_BGR;
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		fmt_ctl |= CR1_BPP16_555;
		break;
	default:
		WARN_ONCE(true, "Unknown FB format 0x%08x, set XRGB8888 instead\n",
			fb->format->format);
		fmt_ctl |= CR1_BPP24;
		break;
	}

	cntl = readl(priv->regs + CR1);
	if ((cntl & CR1_FMT_MASK) != fmt_ctl) {
		if (priv->state) {
			dev_warn(dev->dev,
				"Can't change format while VDU is running! (%x -> %x)\n",
				(u32)(cntl & CR1_FMT_MASK), fmt_ctl);
			baikal_vdu_wait_off(priv);
		}

		cntl &= ~CR1_FMT_MASK;
		cntl |= fmt_ctl;
		/* CR1 is updated on exit */
	}

	addr = drm_fb_dma_get_gem_addr(fb, state, 0);
	end = ((addr + fb->height * fb->pitches[0] - 1) & MRR_DEAR_MRR_MASK) |
		MRR_OUTSTND_RQ(4);

	spin_lock_irqsave(&priv->lock, flags);
	priv->fb_addr = addr;
	priv->fb_end = end;
	reg = readl(priv->regs + ISR);
	writel(INTR_BAU, priv->regs + ISR);
	writel(addr, priv->regs + DBAR);
	if (priv->state == 0) {
		dev_dbg(dev->dev, "update MRR immediately (state %d)\n",
			priv->crtc.state->active);
		writel(end, priv->regs + MRR);
		priv->fb_end = 0;
	} else if ((reg & (INTR_BAU | INTR_VCT)) == 0) {
		dev_dbg(dev->dev, "update MRR immediately (vblank)\n");
		writel(end, priv->regs + MRR);
		priv->fb_end = 0;
	} else {
		/* enable vblank and INTR_BAU - MRR will be set on
		 * first of these interrupts */
		/* clear interrupt status */
		writel(INTR_VCT, priv->regs + ISR);
		reg = readl(priv->regs + IMR);
		writel(reg | INTR_VCT | INTR_BAU, priv->regs + IMR);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	writel(cntl, priv->regs + CR1);
	dev_dbg(dev->dev, "%s: start %x, end %x\n", __func__,
		priv->fb_addr, end);
}

static const struct drm_plane_helper_funcs
baikal_vdu_primary_plane_helper_funcs = {
	.atomic_update = baikal_vdu_primary_plane_atomic_update,
};

static const struct drm_plane_funcs baikal_vdu_primary_plane_funcs = {
	.reset = drm_atomic_helper_plane_reset,
	.update_plane = drm_atomic_helper_update_plane,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int baikal_vdu_primary_plane_init(struct drm_device *drm)
{
	struct baikal_vdu_private *priv = drm->dev_private;
	struct drm_plane *plane = &priv->primary;
	static const u32 formats[] = {
		DRM_FORMAT_RGB888,
		DRM_FORMAT_ARGB8888,
		DRM_FORMAT_XRGB8888,
		DRM_FORMAT_RGB565,
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
