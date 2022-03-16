// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Baikal SoC
 *
 * Copyright (C) 2019-2021 Baikal Electronics, JSC
 * Authors: Pavel Parkhomenko <pavel.parkhomenko@baikalelectronics.ru>
 *          Mikhail Ivanov <michail.ivanov@baikalelectronics.ru>
 *          Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

struct baikal_pcie_rc {
	struct dw_pcie *pcie;
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

#define BAIKAL_LCRU_PCIE_RESET_BASE		0x50000
#define BAIKAL_LCRU_PCIE_RESET(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_RESET_BASE)
#define BAIKAL_PCIE_PHY_RESET			BIT(0)
#define BAIKAL_PCIE_PIPE0_RESET			BIT(4) /* x8 & x4 controllers */
#define BAIKAL_PCIE_PIPE1_RESET			BIT(5) /* x8 controller only */
#define BAIKAL_PCIE_STICKY_RST			BIT(10)
#define BAIKAL_PCIE_CORE_RST			BIT(8)
#define BAIKAL_PCIE_PWR_RST			BIT(9)
#define BAIKAL_PCIE_NONSTICKY_RST		BIT(11)
#define BAIKAL_PCIE_HOT_RESET			BIT(12)
#define BAIKAL_PCIE_ADB_PWRDWN			BIT(13)

#define BAIKAL_LCRU_PCIE_STATUS_BASE		0x50004
#define BAIKAL_LCRU_PCIE_STATUS(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_STATUS_BASE)
#define BAIKAL_PCIE_RDLH_LINKUP			BIT(7)
#define BAIKAL_PCIE_SMLH_LINKUP			BIT(6)
#define BAIKAL_PCIE_LTSSM_MASK			0x3F
#define BAIKAL_PCIE_LTSSM_STATE_L0		0x11
#define BAIKAL_PCIE_LTSSM_STATE_L0S		0x12

#define BAIKAL_LCRU_PCIE_GEN_CTL_BASE		0x50008
#define BAIKAL_LCRU_PCIE_GEN_CTL(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_GEN_CTL_BASE)
#define BAIKAL_PCIE_LTSSM_ENABLE		BIT(1)
#define BAIKAL_PCIE_DBI2_MODE			BIT(2)
#define BAIKAL_PCIE_PHY_MGMT_ENABLE		BIT(3)

#define BAIKAL_LCRU_PCIE_MSI_TRANS_CTL0		0x500E8
#define BAIKAL_PCIE_MSI_AWUSER_SHIFT		0
#define BAIKAL_PCIE_MSI_AWUSER_MASK		0xF

#define BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2		0x500F8
#define BAIKAL_PCIE_MSI_TRANS_EN(x)		BIT((9 + (x)))
#define BAIKAL_PCIE_MSI_RCNUM(x)		((x) << (2 * (x)))
#define BAIKAL_PCIE_MSI_RCNUM_MASK(x)		(0x3 << (2 * (x)))

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

static int baikal_pcie_link_up(struct dw_pcie *pcie)
{
	struct baikal_pcie_rc *rc = to_baikal_pcie(pcie);
	u32 reg;

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	if (!(reg & BAIKAL_PCIE_LTSSM_ENABLE))
		return 0;

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr), &reg);
	reg &= BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
	       BAIKAL_PCIE_LTSSM_MASK;
	if (reg == (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
		    BAIKAL_PCIE_LTSSM_STATE_L0))
		return 1; /* Link is up */
	if (reg == (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP |
		    BAIKAL_PCIE_LTSSM_STATE_L0S))
		return 1; /* Link is also up */

	return 0;
}

static int baikal_pcie_establish_link(struct dw_pcie *pcie)
{
	u32 reg;
	int speed, target_speed;
	int ret;

	ret = dw_pcie_wait_for_link(pcie);
	if (ret != 0)
		return ret;

	reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	while (speed < PCIE_CAP_LINK_SPEED_GEN3) {
		target_speed = speed + 1;
		dev_info(pcie->dev, "Retrain link to Gen%d...\n", target_speed);
		baikal_pcie_link_retrain(pcie, target_speed);
		reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL_LINK_STATUS_REG);
		speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >>
				PCIE_CAP_LINK_SPEED_SHIFT;
		if (speed < target_speed) {
			if (speed < target_speed - 1) {
				dev_info(pcie->dev,
					"Retrain resulted in link downgrade...\n");
				baikal_pcie_link_retrain(pcie, target_speed - 1);
				reg = dw_pcie_readl_dbi(pcie,
					PCIE_LINK_CONTROL_LINK_STATUS_REG);
				speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >>
					PCIE_CAP_LINK_SPEED_SHIFT;
			}
			break; /* give up */
		}
	}
	dev_info(pcie->dev, "Link Status is Gen%d, x%d\n", speed,
		 (reg & PCIE_STA_LINK_WIDTH_MASK) >>
			PCIE_STA_LINK_WIDTH_SHIFT);

	return ret;
}

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie(pcie);
	u32 lcru_reg, class_reg, reg;
	phys_addr_t cfg_base, cfg_end;
	u32 cfg_size;

	/* Configure MEM and I/O ATU regions */
	dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX0,
			PCIE_ATU_TYPE_MEM, pp->mem_base,
			pp->mem_bus_addr, pp->mem_size);
	dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX1,
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
		dev_info(pcie->dev, "Not enough space for ECAM\n");
		rc->max_bus = 255;
		dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX2,
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
		dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
				PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX2);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_BASE,
				lower_32_bits(cfg_base));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_BASE,
				upper_32_bits(cfg_base));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LIMIT,
				lower_32_bits(cfg_base) + PCIE_ATU_MIN_SIZE - 1);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_TARGET, 0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_TARGET, 0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR1, PCIE_ATU_TYPE_CFG0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR2,
				PCIE_ATU_ENABLE | PCIE_ATU_CR2_CFG_SHIFT);
		/* set up region 3 for higher busses */
		dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
				PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX3);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_BASE,
				lower_32_bits(cfg_base + (1 << PCIE_ECAM_BUS_SHIFT)));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_BASE,
				upper_32_bits(cfg_base));
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LIMIT,
				lower_32_bits(cfg_base) + cfg_size - 1);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_LOWER_TARGET, 0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_UPPER_TARGET, 0);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR1, PCIE_ATU_TYPE_CFG1);
		dw_pcie_writel_dbi(pcie, PCIE_ATU_CR2,
				PCIE_ATU_ENABLE | PCIE_ATU_CR2_CFG_SHIFT);
		rc->ecam = true;
		rc->max_bus = (cfg_size >> PCIE_ECAM_BUS_SHIFT) + 1;
		dev_info(pcie->dev, "ECAM configured. Max bus %u\n",
			 rc->max_bus);
	}

	// Set class
	regmap_read(rc->lcru,
		    BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &lcru_reg);
	regmap_write(rc->lcru,
		     BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr),
		     lcru_reg & (~BAIKAL_PCIE_DBI2_MODE));

	dw_pcie_dbi_ro_wr_en(pcie);
	class_reg = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION);

	class_reg = (PCI_CLASS_BRIDGE_PCI << 16) | (1 << 8) | (class_reg & 0xff);
	// class PCI_PCI_BRIDGE=0x604, prog-if=1
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, class_reg);

	dw_pcie_dbi_ro_wr_dis(pcie);

	dw_pcie_setup_rc(pp);
	baikal_pcie_establish_link(pcie); // This call waits for training completion

	dw_pcie_writel_dbi(pcie, PCI_ROOT_ERR_CMD, 7); // enable AER
	reg = dw_pcie_readl_dbi(pcie, PCI_DEV_CTRL_STAT);
	reg |= 0xf; // enable error reporting
	dw_pcie_writel_dbi(pcie, PCI_DEV_CTRL_STAT, reg);
	reg = dw_pcie_readl_dbi(pcie, PCI_ROOT_CTRL_CAP);
	reg |= 0xf; // enable error reporting
	dw_pcie_writel_dbi(pcie, PCI_ROOT_CTRL_CAP, reg);

	return 0;
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct device *dev = pcie->dev;
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
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie(pcie);

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
			dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX3,
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
	struct baikal_pcie_rc *rc = priv;
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	u32 ue_st, ce_st, r_st, dev_st;

	ue_st = dw_pcie_readl_dbi(pcie, PCI_UNCORR_ERR_STAT);
	ce_st = dw_pcie_readl_dbi(pcie, PCI_CORR_ERR_STAT);
	r_st = dw_pcie_readl_dbi(pcie, PCI_ROOT_ERR_STAT);
	dev_st = dw_pcie_readl_dbi(pcie, PCI_DEV_CTRL_STAT);

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

	dw_pcie_writel_dbi(pcie, PCI_UNCORR_ERR_STAT, ue_st);
	dw_pcie_writel_dbi(pcie, PCI_CORR_ERR_STAT, ce_st);
	dw_pcie_writel_dbi(pcie, PCI_ROOT_ERR_STAT, r_st);
	dw_pcie_writel_dbi(pcie, PCI_DEV_CTRL_STAT, dev_st);

	return IRQ_HANDLED;
}

static int baikal_add_pcie_port(struct baikal_pcie_rc *rc,
				struct platform_device *pdev)
{
	struct dw_pcie *pcie = rc->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct resource *res;
	int irq;
	int ret;

	pcie->dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		pcie->dbi_base = devm_ioremap_resource(pcie->dev, res);
		if (IS_ERR(pcie->dbi_base)) {
			dev_err(pcie->dev, "error with ioremap\n");
			return PTR_ERR(pcie->dbi_base);
		}
	} else {
		dev_err(pcie->dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(pcie->dev, "missing IRQ resource: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(pcie->dev, irq, baikal_pcie_err_irq_handler, IRQF_SHARED,
			"baikal-pcie-error-irq", rc);
	if (ret < 0) {
		dev_err(pcie->dev, "failed to request error IRQ %d\n",
			irq);
		return ret;
	}

	pp->root_bus_nr = -1;
	pp->ops = &baikal_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(pcie->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int baikal_pcie_hw_init_m(struct baikal_pcie_rc *rc)
{
	u32 reg;

	/* Cease link */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	/* Force controller reset */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
	reg |= BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
	       BAIKAL_PCIE_PIPE0_RESET | BAIKAL_PCIE_PHY_RESET;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Wait */
	usleep_range(80000, 100000);

	/* Deassert PHY reset */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
	reg &= ~BAIKAL_PCIE_PHY_RESET;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Enable access to the PHY registers */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	reg |= BAIKAL_PCIE_PHY_MGMT_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	/* Clear all software controlled resets of the controller */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
	reg &= ~(BAIKAL_PCIE_ADB_PWRDWN | BAIKAL_PCIE_HOT_RESET |
		 BAIKAL_PCIE_NONSTICKY_RST | BAIKAL_PCIE_STICKY_RST |
		 BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
		 BAIKAL_PCIE_PIPE0_RESET | BAIKAL_PCIE_PIPE1_RESET);
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

	/* Start LTSSM */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	reg |= BAIKAL_PCIE_LTSSM_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	return 0;
}

static const struct of_device_id of_baikal_pcie_match[] = {
	{ .compatible = "baikal,bm1000-pcie" },

	/*
	 * TODO: it is pretty common to use "vendor,chip-*" prefixes
	 *	 to all SoC specific devices. So "pcie-m" is legacy.
	 */
	{ .compatible = "baikal,pcie-m", },
	{},
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct dw_pcie *pcie;
	struct baikal_pcie_rc *rc;
	struct device *dev = &pdev->dev;
	int err;
	u32 index[2];
	enum of_gpio_flags flags;
	int reset_gpio;
	u32 reg;
	int link_up = 0;

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		dev_err(dev, "failed to allocate memory [2]\n");
		return -ENOMEM;
	}
	pcie->dev = dev;
	pcie->ops = &baikal_pcie_ops;

	rc->pcie = pcie;

	rc->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,
					"baikal,pcie-lcru");
	if (IS_ERR(rc->lcru)) {
		dev_err(dev, "No LCRU phandle specified\n");
		rc->lcru = NULL;
		return -EINVAL;
	}

	if (of_property_read_u32_array(dev->of_node,
			"baikal,pcie-lcru", index, 2)) {
		dev_err(dev, "failed to read LCRU\n");
		rc->lcru = NULL;
		return -EINVAL;
	}
	rc->bus_nr = index[1];

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr), &reg);
	dev_dbg(dev, "%s: bus %d - link state %x, reset %x\n",
		__func__, rc->bus_nr, reg,
		(regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg), reg));
	if ((reg & (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) ==
	    (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) {
		link_up = 1;
	}
	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		snprintf(rc->reset_name, 32, "pcie%d-reset", rc->bus_nr);
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
					    rc->reset_name);
		if (err) {
			dev_err(dev, "request GPIO failed (%d)\n", err);
			goto err_pm_disable;
		}
		rc->reset_gpio = gpio_to_desc(reset_gpio);

	} else if (reset_gpio == -EPROBE_DEFER) {
		err = reset_gpio;
		goto err_pm_disable;
	}

	if (!link_up) {
		baikal_pcie_hw_init_m(rc);
		if (rc->reset_gpio)
			gpiod_set_value_cansleep(rc->reset_gpio, 0);
	}

	platform_set_drvdata(pdev, rc);

	err = baikal_add_pcie_port(rc, pdev);
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
	struct baikal_pcie_rc *rc = dev_get_drvdata(dev);
	struct dw_pcie *pcie = rc->pcie;
	u32 val;

	/* clear MSE */
	val = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	val &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, val);

	return 0;
}

static int baikal_pcie_resume(struct device *dev)
{
	struct baikal_pcie_rc *rc = dev_get_drvdata(dev);
	struct dw_pcie *pcie = rc->pcie;
	u32 val;

	/* set MSE */
	val = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	val |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, val);

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

#ifdef CONFIG_ACPI
static int baikal_pcie_get_res_acpi(struct acpi_device *adev, struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct resource_entry *entry;
	struct list_head list, *pos;
	int ret;
	unsigned long flags = IORESOURCE_MEM;

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 3) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 3)\n", ret);
		return -EINVAL;
	}

	/* ECAM (CONFIG) */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->cfg0_size = resource_size(entry->res);
	pp->cfg1_size = pp->cfg0_size;
	pp->cfg0_base = entry->res->start;
	pp->cfg1_base = pp->cfg0_base;

	/* DBI */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	to_dw_pcie_from_pp(pp)->dbi_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(to_dw_pcie_from_pp(pp)->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(to_dw_pcie_from_pp(pp)->dbi_base);
		return ret;
	}

	/* 32bit non-prefetchable memory */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->mem_base = entry->res->start;
	pp->mem_size = resource_size(entry->res);
	pp->mem_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);

	/* I/O */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_IO;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of IO resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->io_base = entry->res->start;
	pp->io_size = resource_size(entry->res);
	pp->io_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);
	return 0;
}

static int baikal_pcie_get_irq_acpi(struct acpi_device *adev, struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct resource res;
	int ret;

	memset(&res, 0, sizeof(res));

	ret = acpi_irq_get(adev->handle, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get irq %d\n", 0);
		return ret;
	}

	if (res.flags & IORESOURCE_BITS) {
		struct irq_data *irqd;

		irqd = irq_get_irq_data(res.start);
		if (!irqd)
			return -ENXIO;
		irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
	}

	pp->irq = res.start;

	ret = devm_request_irq(dev, pp->irq, baikal_pcie_err_irq_handler,
			       IRQF_SHARED, "baikal-pcie-error-irq", to_dw_pcie_from_pp(pp));
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", pp->irq);
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		memset(&res, 0, sizeof(res));

		ret = acpi_irq_get(adev->handle, 1, &res);
		if (ret) {
			dev_err(dev, "failed to get irq %d\n", 1);
			return ret;
		}

		if (res.flags & IORESOURCE_BITS) {
			struct irq_data *irqd;

			irqd = irq_get_irq_data(res.start);
			if (!irqd)
				return -ENXIO;
			irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
		}

		pp->msi_irq = res.start;
	}

	return 0;
}

static struct acpi_device *baikal_lcru;
static struct regmap *baikal_regmap;

static const struct regmap_config baikal_pcie_syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4
};

static struct regmap *baikal_get_regmap(struct acpi_device *adev)
{
	struct device *dev = &adev->dev;
	struct list_head list, *pos;
	struct resource *res;
	void __iomem *base;
	int ret;
	struct regmap *regmap = NULL;
	struct regmap_config config = baikal_pcie_syscon_regmap_config;
	unsigned long flags = IORESOURCE_MEM;

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return NULL;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		goto ret;
	}

	pos = list.next;
	res = list_entry(pos, struct resource_entry, node)->res;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base) {
		dev_err(dev, "error with ioremap\n");
		goto ret;
	}

	config.max_register = resource_size(res) - 4;

	regmap = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "regmap init failed\n");
		devm_iounmap(dev, base);
		goto ret;
	}

	dev_dbg(dev, "regmap %pR registered\n", res);

	baikal_lcru = adev;
	baikal_regmap = regmap;

ret:
	acpi_dev_free_resource_list(&list);
	return regmap;
}

static struct regmap *baikal_pcie_get_lcru_acpi(struct acpi_device *adev, struct baikal_pcie_rc *rc)
{
	struct device *dev = &adev->dev;
	struct acpi_device *lcru;
	struct regmap *regmap = NULL;
	union acpi_object *package = NULL;
	union acpi_object *element = NULL;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status = AE_OK;
	int ret;

	status = acpi_evaluate_object_typed(adev->handle, "LCRU", NULL, &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get LCRU data\n");
		return ERR_PTR(-ENODEV);
	}

	package = buffer.pointer;

	if (package->package.count != 2) {
		dev_err(dev, "invalid LCRU data\n");
		goto ret;
	}

	element = &(package->package.elements[0]);

	if (element->type != ACPI_TYPE_LOCAL_REFERENCE || !element->reference.handle) {
		dev_err(dev, "invalid LCRU reference\n");
		goto ret;
	}

	ret = acpi_bus_get_device(element->reference.handle, &lcru);
	if (ret) {
		dev_err(dev, "failed to process LCRU reference\n");
		goto ret;
	}

	element = &(package->package.elements[1]);

	if (element->type != ACPI_TYPE_INTEGER) {
		dev_err(dev, "failed to get LCRU index\n");
		goto ret;
	}

	rc->bus_nr = element->integer.value;
	if (baikal_regmap && lcru == baikal_lcru) {
		regmap = baikal_regmap;
	} else {
		regmap = baikal_get_regmap(lcru);
	}

ret:
	acpi_os_free(buffer.pointer);
	return regmap;
}

static void baikal_dw_msi_ack_irq(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void baikal_dw_msi_mask_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void baikal_dw_msi_unmask_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip baikal_dw_pcie_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_ack = baikal_dw_msi_ack_irq,
	.irq_mask = baikal_dw_msi_mask_irq,
	.irq_unmask = baikal_dw_msi_unmask_irq
};

static struct msi_domain_info baikal_dw_pcie_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.chip	= &baikal_dw_pcie_msi_irq_chip
};

static irqreturn_t baikal_dw_handle_msi_irq(struct pcie_port *pp)
{
	int i, pos, irq;
	unsigned long val;
	u32 status, num_ctrls;
	irqreturn_t ret = IRQ_NONE;

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	for (i = 0; i < num_ctrls; i++) {
		dw_pcie_read(to_dw_pcie_from_pp(pp)->dbi_base +
				PCIE_MSI_INTR0_STATUS +
				(i * MSI_REG_CTRL_BLOCK_SIZE),
			     4, &status);
		if (!status) {
			continue;
		}

		ret = IRQ_HANDLED;
		val = status;
		pos = 0;
		while ((pos = find_next_bit(&val, MAX_MSI_IRQS_PER_CTRL,
					    pos)) != MAX_MSI_IRQS_PER_CTRL) {
			irq = irq_find_mapping(pp->irq_domain,
					       (i * MAX_MSI_IRQS_PER_CTRL) +
					       pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

static void baikal_dw_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	baikal_dw_handle_msi_irq(pp);

	chained_irq_exit(chip, desc);
}

static void baikal_dw_pci_setup_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u64 msi_target;

	msi_target = (u64)pp->msi_data;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);

	msg->data = d->hwirq;

	dev_dbg(pci->dev, "msi#%d address_hi %#x address_lo %#x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo);
}

static int baikal_dw_pci_msi_set_affinity(struct irq_data *d,
					  const struct cpumask *mask,
					  bool force)
{
	return -EINVAL;
}

static void baikal_dw_pci_bottom_mask(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] |= BIT(bit);
	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_MASK + res,
		      4, pp->irq_mask[ctrl]);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void baikal_dw_pci_bottom_unmask(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] &= ~BIT(bit);
	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_MASK + res,
		      4, pp->irq_mask[ctrl]);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void baikal_dw_pci_bottom_ack(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_STATUS + res,
		      4, BIT(bit));
}

static struct irq_chip baikal_dw_pci_msi_bottom_irq_chip = {
	.name = "BAIKALPCI-MSI",
	.irq_ack = baikal_dw_pci_bottom_ack,
	.irq_compose_msi_msg = baikal_dw_pci_setup_msi_msg,
	.irq_set_affinity = baikal_dw_pci_msi_set_affinity,
	.irq_mask = baikal_dw_pci_bottom_mask,
	.irq_unmask = baikal_dw_pci_bottom_unmask
};

static int baikal_dw_pcie_irq_domain_alloc(struct irq_domain *domain,
					   unsigned int virq, unsigned int nr_irqs,
					   void *args)
{
	struct pcie_port *pp = domain->host_data;
	unsigned long flags;
	u32 i;
	int bit;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bit = bitmap_find_free_region(pp->msi_irq_in_use, pp->num_vectors,
				      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);

	if (bit < 0) {
		return -ENOSPC;
	}

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i,
				    pp->msi_irq_chip,
				    pp, handle_edge_irq,
				    NULL, NULL);
	}

	return 0;
}

static void baikal_dw_pcie_irq_domain_free(struct irq_domain *domain,
					   unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bitmap_release_region(pp->msi_irq_in_use, d->hwirq,
			      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static const struct irq_domain_ops baikal_dw_pcie_msi_domain_ops = {
	.alloc	= baikal_dw_pcie_irq_domain_alloc,
	.free	= baikal_dw_pcie_irq_domain_free
};

static int baikal_dw_pcie_allocate_domains(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct fwnode_handle *fwnode = pci->dev->fwnode;

	pp->irq_domain = irq_domain_create_linear(fwnode, pp->num_vectors,
						  &baikal_dw_pcie_msi_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(pci->dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	irq_domain_update_bus_token(pp->irq_domain, DOMAIN_BUS_NEXUS);

	pp->msi_domain = pci_msi_create_irq_domain(fwnode,
						   &baikal_dw_pcie_msi_domain_info,
						   pp->irq_domain);
	if (!pp->msi_domain) {
		dev_err(pci->dev, "Failed to create MSI domain\n");
		irq_domain_remove(pp->irq_domain);
		return -ENOMEM;
	}

	return 0;
}

static void baikal_dw_pcie_free_msi(struct pcie_port *pp)
{
	if (pp->msi_irq) {
		irq_set_chained_handler(pp->msi_irq, NULL);
		irq_set_handler_data(pp->msi_irq, NULL);
	}

	irq_domain_remove(pp->msi_domain);
	irq_domain_remove(pp->irq_domain);

	if (pp->msi_page) {
		__free_page(pp->msi_page);
	}
}

static int baikal_pcie_msi_init(struct pcie_port *pp)
{
	int ret;

	if (pci_msi_enabled()) {
		pp->num_vectors = MSI_DEF_NUM_VECTORS;

		pp->msi_irq_chip = &baikal_dw_pci_msi_bottom_irq_chip;

		ret = baikal_dw_pcie_allocate_domains(pp);
		if (ret) {
			return ret;
		}

		if (pp->msi_irq) {
			irq_set_chained_handler_and_data(pp->msi_irq,
							 baikal_dw_chained_msi_isr,
							 pp);
		}
	}

	return 0;
}

static int baikal_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct acpi_device *adev = to_acpi_device(dev);
	struct baikal_pcie_rc *rc;
	struct dw_pcie *pcie;
	struct pcie_port *pp;
	acpi_status status = AE_OK;
	u64 num_viewport;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		return -ENOMEM;
	}

	pcie->dev = dev;
	pcie->ops = &baikal_pcie_ops;

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc) {
		return -ENOMEM;
	}

	rc->pcie = pcie;
	cfg->priv = rc;
	pp = &pcie->pp;
	dev_set_drvdata(dev, rc);

	rc->lcru = baikal_pcie_get_lcru_acpi(adev, rc);

	if (IS_ERR_OR_NULL(rc->lcru)) {
		dev_err(dev, "No LCRU specified\n");
		return -EINVAL;
	}

	if (rc->bus_nr > 2) {
		dev_err(dev, "incorrect LCRU index\n");
		return -EINVAL;
        }

	/* TODO: gpio */

	ret = baikal_pcie_get_res_acpi(adev, pp);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	ret = baikal_pcie_get_irq_acpi(adev, pp);
	if (ret) {
		dev_err(dev, "failed to get irq info\n");
		return ret;
	}

	pp->ops = &baikal_pcie_host_ops;
	pp->root_bus_nr = cfg->busr.start;
	pp->busn = &cfg->busr;

	raw_spin_lock_init(&pp->lock);
	pp->va_cfg0_base = devm_pci_remap_cfgspace(dev, pp->cfg0_base, pp->cfg0_size);
	if (!pp->va_cfg0_base) {
		dev_err(dev, "error with ioremap\n");
		return -ENOMEM;
	}
	pp->va_cfg1_base = pp->va_cfg0_base;

	status = acpi_evaluate_integer(adev->handle, "NUMV", NULL, &num_viewport);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get num-viewport\n");
		return -EINVAL;
	}
	pcie->num_viewport = num_viewport;

	ret = baikal_pcie_msi_init(pp);
	if (ret) {
		dev_err(dev, "failed to init MSI\n");
		return ret;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "failed to enable DMA\n");
		return ret;
	}

	ret = baikal_pcie_host_init(pp);

	if (ret && pci_msi_enabled()) {
		baikal_dw_pcie_free_msi(pp);
	}

	return ret;
}

static void __iomem *baikal_pci_ecam_map_bus(struct pci_bus *bus, unsigned int devfn,
				     int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie_rc *priv = cfg->priv;
	u64 cpu_addr = priv->pcie->pp.cfg0_base;
	unsigned int devfn_shift = cfg->ops->bus_shift - 8;
	unsigned int busn = bus->number;
	void __iomem *base;
	int type;
	u32 retries, val;

	if (bus->parent->number == priv->pcie->pp.root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
	}

	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_VIEWPORT, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX1);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LOWER_BASE, lower_32_bits(cpu_addr));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_UPPER_BASE, upper_32_bits(cpu_addr));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LIMIT, lower_32_bits(cpu_addr + resource_size(&cfg->res) - 1));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LOWER_TARGET, 0);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_UPPER_TARGET, 0);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_CR1, type);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_CR2, PCIE_ATU_ENABLE | (0x1 << 28));

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++) {
		val = dw_pcie_readl_dbi(priv->pcie, PCIE_ATU_CR2);
		if (val & PCIE_ATU_ENABLE) {
			break;
		}

		mdelay(LINK_WAIT_IATU);
	}

	if (!(val & PCIE_ATU_ENABLE)) {
		dev_err(priv->pcie->dev, "Outbound iATU is not being enabled\n");
	}

	if (busn < cfg->busr.start || busn > cfg->busr.end) {
		return NULL;
	}

	busn -= cfg->busr.start;
	base = cfg->win + (busn << cfg->ops->bus_shift);
	return base + (devfn << devfn_shift) + where;
}

static void __iomem *baikal_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
					 int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie_rc *priv = cfg->priv;

	if (priv->pcie->pp.root_bus == NULL) {
		priv->pcie->pp.root_bus = bus;
		baikal_pcie_establish_link(priv->pcie);
	}

	if (bus->number != cfg->busr.start && !baikal_pcie_link_up(priv->pcie)) {
		return NULL;
	}

	if (bus->number == cfg->busr.start) {
		/*
		 * The DW PCIe core doesn't filter out transactions to other
		 * devices/functions on the root bus num, so we do this here.
		 */
		if (PCI_SLOT(devfn) > 0) {
			return NULL;
		} else {
			return priv->pcie->dbi_base + where;
		}
	}

	return baikal_pci_ecam_map_bus(bus, devfn, where);
}

struct pci_ecam_ops baikal_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= baikal_pcie_init,
	.pci_ops	= {
		.map_bus	= baikal_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif
