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
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

/**
 * baikal_vdu_crtc.c
 * Implementation of the CRTC functions for Baikal Electronics BE-M1000 VDU driver
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/delay.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_vblank.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

struct baikal_vdu_crtc_mode_fixup {
	int vdisplay;
	int vfp_add;
};

static const struct baikal_vdu_crtc_mode_fixup mode_fixups[] = {
	{ 480, 38 },
	{ 600, 8 },
	{ 720, 43 },
	{ 768, 43 },
	{ 800, 71 },
	{ 864, 71 },
	{ 900, 71 },
	{ 960, 71 },
	{ 1024, 25 },
	{ 1050, 25 },
	{ 1080, 8 },
	{ 1200, 32 },
	{ 1440, 27 },
	{ ~0U },
};

irqreturn_t baikal_vdu_irq(int irq, void *data)
{
	struct drm_device *drm = data;
	struct baikal_vdu_private *priv = drm->dev_private;
	irqreturn_t status = IRQ_NONE;
	u32 raw_stat;
	u32 irq_stat;
	u32 reg;

	irq_stat = readl(priv->regs + IVR);
	raw_stat = readl(priv->regs + ISR);

	if (irq_stat & INTR_LDD) {
		dev_dbg(drm->dev, "INTR_LDD\n");
		priv->state = 0;
		wake_up(&priv->queue);
		status = IRQ_HANDLED;
	}

	if (irq_stat & INTR_VCT) {
		priv->counters[10]++;
		if (priv->fb_end) {
			writel(priv->fb_end, priv->regs + MRR);
			priv->fb_end = 0;
		}
		if (drm->vblank[0].enabled) {
			drm_crtc_handle_vblank(&priv->crtc);
		} else {
			reg = readl(priv->regs + IMR);
			reg &= ~INTR_VCT;
			writel(reg, priv->regs + IMR);
		}
		/*
		 * We also clear INTR_BAU. This allows us to detect vblank
		 * state by reading raw irq status:
		 * if INTR_VCT is not set and INTR_BAU is not set
		 *   then we are in vblank;
		 * if INTR_VCT is not set and INTR_BAU is set
		 *   then new frame has started (but not yet completed);
		 * if INTR_VCT is set
		 *   then we can't detect vblank.
		 */
		irq_stat |= INTR_BAU;
		status = IRQ_HANDLED;
	}

	if (irq_stat & INTR_BAU) {
		if (priv->fb_end) {
			writel(priv->fb_end, priv->regs + MRR);
			priv->fb_end = 0;
		}
		/*
		 * Don't clear INTR_BAU (see comment above)
		 * but just disable (mask) it instead.
		 */
		reg = readl(priv->regs + IMR);
		reg &= ~INTR_BAU;
		writel(reg, priv->regs + IMR);
		irq_stat &= ~INTR_BAU;
	}
	
	if (irq_stat & INTR_FER) {
		priv->counters[11]++;
		priv->counters[12] = readl(priv->regs + DBAR);
		priv->counters[13] = readl(priv->regs + DCAR);
		priv->counters[14] = readl(priv->regs + MRR);
		status = IRQ_HANDLED;
	}

	priv->counters[3] |= raw_stat;

	/* Clear all interrupts */
	writel(irq_stat, priv->regs + ISR);

	return status;
}

void baikal_vdu_wait_off(struct baikal_vdu_private *priv)
{
	u32 reg;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->lock, flags);
	reg = readl(priv->regs + CR1);
	if (reg & CR1_LCE) {
		writel(INTR_LDD, priv->regs + ISR);
		writel(reg & ~CR1_LCE, priv->regs + CR1);
		reg = readl(priv->regs + IMR);
		writel (reg | INTR_LDD, priv->regs + IMR);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	ret = wait_event_timeout(priv->queue, priv->state == 0,
				 msecs_to_jiffies(100));
	if (!ret)
		dev_err(priv->drm->dev, "Timeout waiting VDU off!\n");
}

static int baikal_vdu_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	struct drm_display_mode *mode = &state->adjusted_mode;
	struct baikal_vdu_private *priv = crtc->dev->dev_private;
	long rate, available_rate;

	if (!mode->clock)
		return 0;
	rate = mode->clock * 1000;
	available_rate = clk_round_rate(priv->clk, rate);
	if (available_rate > (rate - rate / 128) &&
	    available_rate < (rate + rate / 128))
		return 0;
	DRM_ERROR("Requested pixel clock is %lu Hz, available - %ld\n",
		  rate, available_rate);
	return -EINVAL;
}

bool baikal_vdu_crtc_mode_fixup(struct drm_crtc *crtc,
					const struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;

	memcpy(adjusted_mode, mode, sizeof(*mode));

	if (!priv->mode_fixup)
		return true;

	if (priv->mode_fixup == -1) {
		const struct baikal_vdu_crtc_mode_fixup *fixups = mode_fixups;
		for (; fixups && fixups->vdisplay != ~0U; ++fixups) {
			if (mode->vdisplay <= fixups->vdisplay)
				break;
		}
		if (fixups->vdisplay == ~0U)
			return true;
		else
			priv->mode_fixup = fixups->vfp_add;
	}

	adjusted_mode->vtotal += priv->mode_fixup;
	adjusted_mode->vsync_start += priv->mode_fixup;
	adjusted_mode->vsync_end += priv->mode_fixup;
	adjusted_mode->clock = mode->clock * adjusted_mode->vtotal / mode->vtotal;

	return true;
}

static void baikal_vdu_crtc_helper_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct baikal_vdu_private *priv = dev->dev_private;
	const struct drm_display_mode *orig_mode = &crtc->state->mode;
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	unsigned long rate;
	unsigned int ppl, hsw, hfp, hbp;
	unsigned int lpp, vsw, vfp, vbp;
	unsigned int reg;
	int ret;

	drm_mode_debug_printmodeline(orig_mode);
	drm_mode_debug_printmodeline(mode);

	if (priv->state)
		baikal_vdu_wait_off(priv);

	rate = mode->clock * 1000;

	ret = clk_set_rate(priv->clk, rate);
	DRM_DEV_DEBUG_DRIVER(dev->dev, "Requested pixel clock is %ld Hz\n",
			     rate);

	if (ret < 0) {
		DRM_ERROR("Cannot set desired pixel clock (%ld Hz)\n",
			  rate);
	}

	ppl = mode->hdisplay / 16;
	if (priv->type == VDU_TYPE_LVDS) {
		hsw = mode->hsync_end - mode->hsync_start;
		hfp = mode->hsync_start - mode->hdisplay - 1;
		hbp = mode->htotal - mode->hsync_end;
	} else {
		hsw = mode->hsync_end - mode->hsync_start - 1;
		hfp = mode->hsync_start - mode->hdisplay - 1;
		hbp = mode->htotal - mode->hsync_end - 1;
	}

	lpp = mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	writel((HTR_HFP(hfp) & HTR_HFP_MASK) |
			(HTR_PPL(ppl) & HTR_PPL_MASK) |
			(HTR_HBP(hbp) & HTR_HBP_MASK) |
			(HTR_HSW(hsw) & HTR_HSW_MASK),
			priv->regs + HTR);

	if (mode->hdisplay > 4080 || ppl * 16 != mode->hdisplay)
		writel((HPPLOR_HPPLO(mode->hdisplay) & HPPLOR_HPPLO_MASK) | HPPLOR_HPOE,
				priv->regs + HPPLOR);

	writel((VTR1_VSW(vsw) & VTR1_VSW_MASK) |
			(VTR1_VFP(vfp) & VTR1_VFP_MASK) |
			(VTR1_VBP(vbp) & VTR1_VBP_MASK),
			priv->regs + VTR1);

	writel(lpp & VTR2_LPP_MASK, priv->regs + VTR2);

	writel((HVTER_VSWE(vsw >> VTR1_VSW_LSB_WIDTH) & HVTER_VSWE_MASK) |
			(HVTER_HSWE(hsw >> HTR_HSW_LSB_WIDTH) & HVTER_HSWE_MASK) |
			(HVTER_VBPE(vbp >> VTR1_VBP_LSB_WIDTH) & HVTER_VBPE_MASK) |
			(HVTER_VFPE(vfp >> VTR1_VFP_LSB_WIDTH) & HVTER_VFPE_MASK) |
			(HVTER_HBPE(hbp >> HTR_HBP_LSB_WIDTH) & HVTER_HBPE_MASK) |
			(HVTER_HFPE(hfp >> HTR_HFP_LSB_WIDTH) & HVTER_HFPE_MASK),
			priv->regs + HVTER);

	/* Set polarities */
	reg = priv->cr1_cfg;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		reg |= CR1_HSP;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		reg |= CR1_VSP;
	reg |= CR1_DEP; // set DE to active high;
	writel(reg, priv->regs + CR1);

	crtc->hwmode = crtc->state->adjusted_mode;
}

static void baikal_vdu_crtc_helper_enable(struct drm_crtc *crtc,
					  struct drm_crtc_state *old_state)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;
	u32 cntl;

	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "enabling pixel clock\n");
	clk_prepare_enable(priv->clk);

	drm_panel_prepare(priv->connector.panel);

	writel(ISCR_VSC_VFP, priv->regs + ISCR);

	/* release clock reset; enable clocking */
	cntl = readl(priv->regs + PCTR);
	cntl |= PCTR_PCR + PCTR_PCI;
	writel(cntl, priv->regs + PCTR);

	/* Enable and Power Up */
	cntl = readl(priv->regs + CR1);
	cntl |= CR1_LCE;
	writel(cntl, priv->regs + CR1);
	priv->state = 1;

	if (priv->type == VDU_TYPE_LVDS)
		writel(priv->lvds_gpior, priv->regs + GPIOR);

	drm_panel_enable(priv->connector.panel);
	drm_crtc_vblank_on(crtc);
}

void baikal_vdu_crtc_helper_disable(struct drm_crtc *crtc)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;

	drm_panel_disable(priv->connector.panel);

	drm_panel_unprepare(priv->connector.panel);
	drm_crtc_vblank_off(crtc);

	baikal_vdu_wait_off(priv);
	clk_disable_unprepare(priv->clk);
	priv->state = 0;
}

static void baikal_vdu_crtc_helper_atomic_flush(struct drm_crtc *crtc,
					   struct drm_crtc_state *old_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static int baikal_vdu_enable_vblank(struct drm_crtc *crtc)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;
	u32 reg;

	if (!crtc->state->active)
		return -EINVAL;

	/* clear interrupt status */
	writel(INTR_VCT, priv->regs + ISR);

	reg = readl(priv->regs + IMR);
	writel(reg | INTR_VCT, priv->regs + IMR);

	return 0;
}

static void baikal_vdu_disable_vblank(struct drm_crtc *crtc)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;
	u32 reg;

	reg = readl(priv->regs + IMR);
	writel(reg & ~INTR_VCT, priv->regs + IMR);
}

static bool baikal_vdu_get_scanpos(struct drm_crtc *crtc,
				   bool in_vblank, int *vpos, int *hpos,
				   ktime_t *stime, ktime_t *etime,
				   const struct drm_display_mode *mode)
{
	struct baikal_vdu_private *priv = crtc->dev->dev_private;
	u32 dcar, dbar, cr, p;
	bool ret;
	int bpp, hsize;

	if (stime)
		*stime = ktime_get();
	cr = readl(priv->regs + CR1);
	if (!(cr & CR1_LCE))
		return false;

	switch (cr & CR1_BPP_MASK) {
	case CR1_BPP24:
	case CR1_BPP18:
		bpp = 4;
		break;
	case CR1_BPP16:
		bpp = 2;
		break;
	default:
		bpp = 1;
	}
	hsize = mode->hdisplay;
	dbar = readl(priv->regs + DBAR);
	dcar = readl(priv->regs + DCAR);
	p = (dcar - dbar) / bpp;
	*vpos = p / hsize;
	if (*vpos >= mode->vdisplay || in_vblank) {
		dev_dbg(crtc->dev->dev, "vpos = %d, vdisplay = %d\n", *vpos, mode->vdisplay);
		dev_dbg(crtc->dev->dev, "DBAR %x, DCAR %x, MRR %x, ISR %x, bpp %d\n",
			dbar, dcar, readl(priv->regs + MRR), readl(priv->regs + ISR), bpp);
		*vpos = (mode->vdisplay - mode->vtotal) / 2; // negative value!
		*hpos = 0;
		ret = true;
	} else {
		*hpos = p % hsize;
		ret = true;
	}
	if (etime)
		*etime = ktime_get();

	return ret;
}

const struct drm_crtc_funcs crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = baikal_vdu_enable_vblank,
	.disable_vblank = baikal_vdu_disable_vblank,
	.get_vblank_timestamp = drm_crtc_vblank_helper_get_vblank_timestamp,
};

const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_fixup = baikal_vdu_crtc_mode_fixup,
	.mode_set_nofb = baikal_vdu_crtc_helper_mode_set_nofb,
	.atomic_check = baikal_vdu_crtc_atomic_check,
	.atomic_flush = baikal_vdu_crtc_helper_atomic_flush,
	.disable = baikal_vdu_crtc_helper_disable,
	.atomic_enable = baikal_vdu_crtc_helper_enable,
	.get_scanout_position = baikal_vdu_get_scanpos,
};

int baikal_vdu_crtc_create(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;
	struct drm_crtc *crtc = &priv->crtc;

	drm_crtc_init_with_planes(dev, crtc,
				  &priv->primary, NULL,
				  &crtc_funcs, "primary");
	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	/* XXX: The runtime clock disabling still results in
	 * occasional system hangs, and needs debugging.
	 */

	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "enabling pixel clock\n");
	clk_prepare_enable(priv->clk);

	return 0;
}
