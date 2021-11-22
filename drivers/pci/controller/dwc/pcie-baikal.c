// SPDX-License-Identifier: GPL-2.0
/*
 * pcie-baikal - PCIe controller driver for Baikal SoCs
 *
 * Copyright (C) 2019 Baikal Electronics JSC
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 */

#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/baikal/lcru-pcie.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>

#include "pcie-designware.h"

struct baikal_pcie {
	struct dw_pcie *pci;
	unsigned bus_nr;
	struct regmap *lcru;
	struct gpio_desc *reset_gpio;
	char reset_name[32];
	u32 cfg_busdev;
	u32 max_bus;
	bool ecam;
};

#define to_baikal_pcie(x)	dev_get_drvdata((x)->dev)

#define LINK_RETRAIN_TIMEOUT HZ

/* Baikal (DesignWare) specific registers. */
#define PCIE_COHERENCE_CONTROL_3_OFF	(0x8e8) /* to set cache coherence register. */

/* Error registers in capabilities config space block */
#define PCI_DEV_CTRL_STAT		0x78
#define PCI_ROOT_CTRL_CAP		0x8c
#define PCI_UNCORR_ERR_STAT		0x104
#define PCI_UNCORR_ERR_MASK		0x10c
#define PCI_CORR_ERR_STAT		0x110
#define PCI_CORR_ERR_MASK		0x114
#define PCI_ROOT_ERR_CMD		0x12c
#define PCI_ROOT_ERR_STAT		0x130
#define PCI_ROOT_ERR_SRC_ID		0x134

#define PCIE_LINK_CAPABILITIES_REG		(0x7c)	/* Link Capabilities Register. */

#define PCIE_LINK_CONTROL_LINK_STATUS_REG	(0x80)	/* Link Control and Status Register. */
/* LINK_CONTROL_LINK_STATUS_REG */
#define PCIE_CAP_LINK_SPEED_SHIFT		16
#define PCIE_CAP_LINK_SPEED_MASK		0xF0000
#define PCIE_CAP_LINK_SPEED_GEN1		0x1
#define PCIE_CAP_LINK_SPEED_GEN2		0x2
#define PCIE_CAP_LINK_SPEED_GEN3		0x3
#define PCIE_STA_LINK_TRAINING			0x8000000
#define PCIE_STA_LINK_WIDTH_MASK		0x3f00000
#define PCIE_STA_LINK_WIDTH_SHIFT		(20)

#define PCIE_LINK_CONTROL2_LINK_STATUS2_REG	(0xa0)	/* Link Control 2 and Status 2 Register. */
/* PCIE_LINK_CONTROL2_LINK_STATUS2 */
#define PCIE_LINK_CONTROL2_GEN_MASK		(0xF)

#define PCIE_ATU_MIN_SIZE	0x10000		/* 64K */
#define PCIE_ATU_REGION_INDEX3	0x3
#define PCIE_ATU_CR2_CFG_SHIFT	BIT(28)
#define PCIE_ECAM_SIZE		0x10000000	/* 256M */
#define PCIE_ECAM_MASK		0x0fffffffULL
#define PCIE_ECAM_BUS_SHIFT	20
#define PCIE_ECAM_BUS_MASK	GENMASK(27,20)
#define PCIE_ECAM_DEVFN_SHIFT	12

static int baikal_pcie_link_up(struct dw_pcie *pci);

static inline int baikal_pcie_link_is_training(struct dw_pcie *pci)
{
	int reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);

	return reg & PCIE_STA_LINK_TRAINING;
}

static bool baikal_wait_pcie_link_training_done(struct dw_pcie *pci)
{
	unsigned long start_jiffies = jiffies;

	while (baikal_pcie_link_is_training(pci)) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			return false;
		}
		usleep_range(800, 1000);
	}
	return true;
}

static inline void baikal_pcie_link_retrain(struct dw_pcie *pci, int target_speed)
{
	int reg;
	unsigned long start_jiffies;

	/* In case link is already training wait for training to complete */
	baikal_wait_pcie_link_training_done(pci);

	/* Set desired speed */
	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
	reg |= target_speed;
	dw_pcie_writel_dbi(pci, PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	/* Set Retrain Link bit */
	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	reg |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, reg);

	/* Wait for link training end */
	if (!baikal_wait_pcie_link_training_done(pci))
		return;

	start_jiffies = jiffies;
	while (baikal_pcie_link_up(pci) == 0) {
		if (time_after(jiffies, start_jiffies + LINK_RETRAIN_TIMEOUT)) {
			pr_err("%s: link retrained for too long, timeout occured\n", __func__);
			break;
		}
		usleep_range(8000, 10000);
	}
}

static int baikal_pcie_link_up(struct dw_pcie *pci)
{
	struct baikal_pcie *rc = to_baikal_pcie(pci);
	u32 reg;

	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	if (!(reg & BAIKAL_PCIE_LTSSM_ENABLE))
		return 0;

	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr));
	reg &= BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
	       BAIKAL_PCIE_LTSSM_STATE_MASK;
	if (reg == (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
		    BAIKAL_PCIE_LTSSM_STATE_L0))
		return 1; /* Link is up */
	if (reg == (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
		    BAIKAL_PCIE_LTSSM_STATE_L0S))
		return 1; /* Link is also up */

	return 0;
}

static int baikal_pcie_establish_link(struct dw_pcie *pci)
{
	u32 reg;
	int speed, target_speed;
	int ret;

	ret = dw_pcie_wait_for_link(pci);
	if (ret != 0)
		return ret;

	reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	while (speed < PCIE_CAP_LINK_SPEED_GEN3) {
		target_speed = speed + 1;
		dev_info(pci->dev, "Retrain link to Gen%d...\n", target_speed);
		baikal_pcie_link_retrain(pci, target_speed);
		reg = dw_pcie_readl_dbi(pci, PCIE_LINK_CONTROL_LINK_STATUS_REG);
		speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >>
				PCIE_CAP_LINK_SPEED_SHIFT;
		if (speed < target_speed)
			break; /* give up */
	}
	dev_info(pci->dev, "Link Status is Gen%d, x%d\n", speed,
		 (reg & PCIE_STA_LINK_WIDTH_MASK) >>
			PCIE_STA_LINK_WIDTH_SHIFT);

	return ret;
}

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *baikal_pcie = to_baikal_pcie(pci);
	u32 lcru_reg, class_reg, reg;
	phys_addr_t cfg_base, cfg_end;
	u32 cfg_size;

	/* Configure MEM and I/O ATU regions */
	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX0,
			PCIE_ATU_TYPE_MEM, pp->mem_base,
			pp->mem_bus_addr, pp->mem_size);
	dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX1,
			PCIE_ATU_TYPE_IO, pp->io_base,
			pp->io_bus_addr, pp->io_size);

	/* Check, if we can set up ECAM */
	cfg_base = pp->cfg0_base;
	cfg_size = pp->cfg0_size << 1;
	cfg_end = cfg_base + cfg_size;
	cfg_base = (cfg_base & ~PCIE_ECAM_MASK) + (0x1 << PCIE_ECAM_BUS_SHIFT);
	if (cfg_base < pp->cfg0_base)
		cfg_base += PCIE_ECAM_SIZE;
	if (cfg_base + (4 << PCIE_ECAM_BUS_SHIFT) > cfg_end) {
		dev_info(pci->dev, "Not enough space for ECAM\n");
		baikal_pcie->max_bus = 255;
		dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX2,
			PCIE_ATU_TYPE_CFG0, pp->cfg0_base,
			PCIE_ATU_BUS(1), PCIE_ATU_MIN_SIZE);
	} else {
		if (cfg_end >= cfg_base + 0x0ff00000)
			cfg_size = 0x0ff00000;
		else
			cfg_size = (cfg_end - cfg_base) & PCIE_ECAM_BUS_MASK;
		if (pp->va_cfg0_base)
			iounmap(pp->va_cfg0_base);
		pp->va_cfg0_base = ioremap_nocache(cfg_base, cfg_size);
		if (pp->va_cfg1_base)
			iounmap(pp->va_cfg1_base);
		/* set up region 2 for bus 1 */
		dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT,
				PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX2);
		dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_BASE,
				lower_32_bits(cfg_base));
		dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_BASE,
				upper_32_bits(cfg_base));
		dw_pcie_writel_dbi(pci, PCIE_ATU_LIMIT,
				lower_32_bits(cfg_base) + PCIE_ATU_MIN_SIZE - 1);
		dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET, 0);
		dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET, 0);
		dw_pcie_writel_dbi(pci, PCIE_ATU_CR1, PCIE_ATU_TYPE_CFG0);
		dw_pcie_writel_dbi(pci, PCIE_ATU_CR2,
				PCIE_ATU_ENABLE | PCIE_ATU_CR2_CFG_SHIFT);
		/* set up region 3 for higher busses */
		dw_pcie_writel_dbi(pci, PCIE_ATU_VIEWPORT,
				PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX3);
		dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_BASE,
				lower_32_bits(cfg_base + (1 << PCIE_ECAM_BUS_SHIFT)));
		dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_BASE,
				upper_32_bits(cfg_base));
		dw_pcie_writel_dbi(pci, PCIE_ATU_LIMIT,
				lower_32_bits(cfg_base) + cfg_size - 1);
		dw_pcie_writel_dbi(pci, PCIE_ATU_LOWER_TARGET, 0);
		dw_pcie_writel_dbi(pci, PCIE_ATU_UPPER_TARGET, 0);
		dw_pcie_writel_dbi(pci, PCIE_ATU_CR1, PCIE_ATU_TYPE_CFG1);
		dw_pcie_writel_dbi(pci, PCIE_ATU_CR2,
				PCIE_ATU_ENABLE | PCIE_ATU_CR2_CFG_SHIFT);
		baikal_pcie->ecam = true;
		baikal_pcie->max_bus = (cfg_size >> PCIE_ECAM_BUS_SHIFT) + 1;
		dev_info(pci->dev, "ECAM configured. Max bus %u\n",
			 baikal_pcie->max_bus);
	}

	// Set class
	lcru_reg = baikal_pcie_lcru_readl(baikal_pcie->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(baikal_pcie->bus_nr));
	baikal_pcie_lcru_writel(baikal_pcie->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(baikal_pcie->bus_nr),
				lcru_reg & (~BAIKAL_PCIE_DBI2_MODE));

	dw_pcie_dbi_ro_wr_en(pci);
	class_reg = dw_pcie_readl_dbi(pci, PCI_CLASS_REVISION);

	class_reg = (PCI_CLASS_BRIDGE_PCI << 16) | (1 << 8) | (class_reg & 0xff);
	// class PCI_PCI_BRIDGE=0x604, prog-if=1
	dw_pcie_writel_dbi(pci, PCI_CLASS_REVISION, class_reg);

	dw_pcie_dbi_ro_wr_dis(pci);

	dw_pcie_setup_rc(pp);
	baikal_pcie_establish_link(pci); // This call waits for training completion

	dw_pcie_writel_dbi(pci, PCI_ROOT_ERR_CMD, 7); // enable AER
	reg = dw_pcie_readl_dbi(pci, PCI_DEV_CTRL_STAT);
	reg |= 0xf; // enable error reporting
	dw_pcie_writel_dbi(pci, PCI_DEV_CTRL_STAT, reg);
	reg = dw_pcie_readl_dbi(pci, PCI_ROOT_CTRL_CAP);
	reg |= 0xf; // enable error reporting
	dw_pcie_writel_dbi(pci, PCI_ROOT_CTRL_CAP, reg);

	return 0;
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct device_node *msi_node;

	/*
	 * The MSI domain is set by the generic of_msi_configure().  This
	 * .msi_host_init() function keeps us from doing the default MSI
	 * domain setup in dw_pcie_host_init() and also enforces the
	 * requirement that "msi-parent" exists.
	 */
	msi_node = of_parse_phandle(np, "msi-parent", 0);
	if (!msi_node) {
		dev_warn(dev, "failed to find msi-parent\n");
		return -EINVAL;
	}

	return 0;
}

static int baikal_pcie_access_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val, bool write)
{
	int ret;
	u32 busdev;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *rc = to_baikal_pcie(pci);

	if (bus->number > rc->max_bus) {
		if (!write)
			*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	
	if (rc->ecam) {
		va_cfg_base = pp->va_cfg0_base +
			((bus->number - 1) << PCIE_ECAM_BUS_SHIFT) +
			(devfn << PCIE_ECAM_DEVFN_SHIFT);
	} else {
		busdev = PCIE_ATU_BUS(bus->number) |
			 PCIE_ATU_DEV(PCI_SLOT(devfn)) |
			 PCIE_ATU_FUNC(PCI_FUNC(devfn));
		if (busdev != rc->cfg_busdev) {
			dw_pcie_prog_outbound_atu(pci, PCIE_ATU_REGION_INDEX3,
						  PCIE_ATU_TYPE_CFG1, pp->cfg1_base,
						  busdev, PCIE_ATU_MIN_SIZE);
			rc->cfg_busdev = busdev;
		}
		va_cfg_base = pp->va_cfg1_base;
	}

	if (write)
		ret = dw_pcie_write(va_cfg_base + where, size, *val);
	else
		ret = dw_pcie_read(va_cfg_base + where, size, val);

	return ret;
}

static int baikal_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	return baikal_pcie_access_other_conf(pp, bus, devfn, where, size, val,
			false);
}

static int baikal_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	return baikal_pcie_access_other_conf(pp, bus, devfn, where, size, &val,
			true);
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.rd_other_conf = baikal_pcie_rd_other_conf,
	.wr_other_conf = baikal_pcie_wr_other_conf,
	.host_init = baikal_pcie_host_init,
	.msi_host_init = baikal_pcie_msi_host_init,
};

static const struct dw_pcie_ops baikal_pcie_ops = {
	.link_up = baikal_pcie_link_up,
};

DEFINE_RATELIMIT_STATE(pcie_err_printk_ratelimit, 300 * HZ, 10);

static irqreturn_t baikal_pcie_err_irq_handler(int irq, void *priv)
{
	struct baikal_pcie *rc = priv;
	struct dw_pcie *pci = rc->pci;
	struct device *dev = pci->dev;
	u32 ue_st, ce_st, r_st, dev_st;

	ue_st = dw_pcie_readl_dbi(pci, PCI_UNCORR_ERR_STAT);
	ce_st = dw_pcie_readl_dbi(pci, PCI_CORR_ERR_STAT);
	r_st = dw_pcie_readl_dbi(pci, PCI_ROOT_ERR_STAT);
	dev_st = dw_pcie_readl_dbi(pci, PCI_DEV_CTRL_STAT);

	if (__ratelimit(&pcie_err_printk_ratelimit)) {
		if (r_st & 0x7c) {
			if (r_st & 0x58)
				dev_err(dev, "%sFatal Error: %x\n",
					(r_st & 0x8) ? "Multiple " : "", ue_st);
			else
				dev_err(dev, "%sNon-Fatal Error: %x\n",
					(r_st & 8) ? "Multiple " : "", ue_st);
		}

		if (r_st & 3)
			dev_err(dev, "%sCorrectable Error: %x\n",
				(r_st & 2) ? "Multiple " : "", ce_st);

		if (dev_st & 0xf0000)
			dev_err(dev, "Device Status Errors: %x\n", dev_st >> 16);
	}

	dw_pcie_writel_dbi(pci, PCI_UNCORR_ERR_STAT, ue_st);
	dw_pcie_writel_dbi(pci, PCI_CORR_ERR_STAT, ce_st);
	dw_pcie_writel_dbi(pci, PCI_ROOT_ERR_STAT, r_st);
	dw_pcie_writel_dbi(pci, PCI_DEV_CTRL_STAT, dev_st);

	return IRQ_HANDLED;
}

static int baikal_add_pcie_port(struct baikal_pcie *rc,
				       struct platform_device *pdev)
{
	struct dw_pcie *pci = rc->pci;
	struct pcie_port *pp = &pci->pp;
	struct resource *res;
	int irq;
	int ret;

	pci->dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		pci->dbi_base = devm_ioremap_resource(pci->dev, res);
		if (IS_ERR(pci->dbi_base)) {
			dev_err(pci->dev, "error with ioremap\n");
			return PTR_ERR(pci->dbi_base);
		}
	} else {
		dev_err(pci->dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(pci->dev, "missing IRQ resource: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(pci->dev, irq, baikal_pcie_err_irq_handler, IRQF_SHARED,
			"baikal-pcie-error-irq", rc);
	if (ret < 0) {
		dev_err(pci->dev, "failed to request error IRQ %d\n",
			irq);
		return ret;
	}

	pp->root_bus_nr = -1;
	pp->ops = &baikal_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pci->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int baikal_pcie_hw_init_m(struct baikal_pcie *rc)
{
	u32 reg;

	/* Cease link */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	/* Force controller reset */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg |= BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
	       BAIKAL_PCIE_PIPE_RESET | BAIKAL_PCIE_PHY_RESET;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Wait */
	usleep_range(80000, 100000);

	/* Deassert PHY reset */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_PHY_RESET;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Enable access to the PHY registers */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= BAIKAL_PCIE_PHY_MGMT_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	/* Clear all software controlled resets of the controller */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= ~(BAIKAL_PCIE_ADB_PWRDWN | BAIKAL_PCIE_HOT_RESET |
		 BAIKAL_PCIE_NONSTICKY_RST | BAIKAL_PCIE_STICKY_RST |
		 BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
		 BAIKAL_PCIE_PIPE_RESET | BAIKAL_PCIE_PIPE1_RESET);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Start LTSSM */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= BAIKAL_PCIE_LTSSM_ENABLE;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	return 0;
}

static const struct of_device_id of_baikal_pcie_match[] = {
	{ .compatible = "baikal,pcie-m", },
	{},
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct dw_pcie *pci;
	struct baikal_pcie *pcie;
	struct device *dev = &pdev->dev;
	int err;
	u32 index[2];
	enum of_gpio_flags flags;
	int reset_gpio;
	u32 reg;
	int link_up = 0;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci) {
		dev_err(dev, "failed to allocate memory [2]\n");
		return -ENOMEM;
	}
	pci->dev = dev;
	pci->ops = &baikal_pcie_ops;

	pcie->pci = pci;

	pcie->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,
					"baikal,pcie-lcru");
	if (IS_ERR(pcie->lcru)) {
		dev_err(dev, "No LCRU phandle specified\n");
		pcie->lcru = NULL;
		return -EINVAL;
	}

	if (of_property_read_u32_array(dev->of_node,
			"baikal,pcie-lcru", index, 2)) {
		dev_err(dev, "failed to read LCRU\n");
		pcie->lcru = NULL;
		return -EINVAL;
	}
	pcie->bus_nr = index[1];

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}

	reg = baikal_pcie_lcru_readl(pcie->lcru, BAIKAL_LCRU_PCIE_STATUS(pcie->bus_nr));
	dev_dbg(dev, "%s: bus %d - link state %x, reset %x\n",
		__func__, pcie->bus_nr, reg,
		baikal_pcie_lcru_readl(pcie->lcru, BAIKAL_LCRU_PCIE_RESET(pcie->bus_nr)));
	if ((reg & (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) ==
	    (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) {
		link_up = 1;
	}
	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		snprintf(pcie->reset_name, 32, "pcie%d-reset", pcie->bus_nr);
		/* gpio_flags correspond to "reset" state */
		if (flags & OF_GPIO_ACTIVE_LOW)
			gpio_flags = GPIOF_ACTIVE_LOW | GPIOF_OUT_INIT_LOW;
		else
			gpio_flags = GPIOF_OUT_INIT_HIGH;

		/*
		 * if link was up then we do not perform reset and request gpio
		 * in "non-reset" state
		 */
		if (link_up)
			gpio_flags ^= GPIOF_OUT_INIT_HIGH ^ GPIOF_OUT_INIT_LOW;
		err = devm_gpio_request_one(dev, reset_gpio, gpio_flags,
					    pcie->reset_name);
		if (err) {
			dev_err(dev, "request GPIO failed (%d)\n", err);
			goto err_pm_disable;
		}
		pcie->reset_gpio = gpio_to_desc(reset_gpio);

	} else if (reset_gpio == -EPROBE_DEFER) {
		err = reset_gpio;
		goto err_pm_disable;
	}

	if (!link_up) {
		baikal_pcie_hw_init_m(pcie);
		if (pcie->reset_gpio)
			gpiod_set_value_cansleep(pcie->reset_gpio, 0);
	}

	platform_set_drvdata(pdev, pcie);

	err = baikal_add_pcie_port(pcie, pdev);
	if (!err)
		return 0;

	pm_runtime_put(dev);

err_pm_disable:
	pm_runtime_disable(dev);

	return err;
}

#ifdef CONFIG_PM_SLEEP
static int baikal_pcie_suspend(struct device *dev)
{
	struct baikal_pcie *rc = dev_get_drvdata(dev);
	struct dw_pcie *pci = rc->pci;
	u32 val;

	/* clear MSE */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	return 0;
}

static int baikal_pcie_resume(struct device *dev)
{
	struct baikal_pcie *rc = dev_get_drvdata(dev);
	struct dw_pcie *pci = rc->pci;
	u32 val;

	/* set MSE */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	return 0;
}

static int baikal_pcie_suspend_noirq(struct device *dev)
{
	return 0;
}

static int baikal_pcie_resume_noirq(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops baikal_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(baikal_pcie_suspend, baikal_pcie_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(baikal_pcie_suspend_noirq,
				      baikal_pcie_resume_noirq)
};

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name	= "baikal-pcie",
		.of_match_table = of_baikal_pcie_match,
		.suppress_bind_attrs = true,
		.pm	= &baikal_pcie_pm_ops,
	},
	.probe = baikal_pcie_probe,
};

builtin_platform_driver(baikal_pcie_driver);
