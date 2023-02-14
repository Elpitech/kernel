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
	unsigned rc_num;
	struct regmap *lcru;
	struct gpio_desc *reset_gpio;
	char reset_name[32];
	u32 cfg_busdev;
	u32 max_bus;
	bool ecam;
};

#define to_baikal_pcie_rc(x)	dev_get_drvdata((x)->dev)

#define LINK_RETRAIN_TIMEOUT HZ

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
#define BAIKAL_PCIE_PHY_RST			BIT(0)
#define BAIKAL_PCIE_PIPE0_RST			BIT(4) /* x8 & x4 controllers */
#define BAIKAL_PCIE_PIPE1_RST			BIT(5) /* x8 controller only */
#define BAIKAL_PCIE_STICKY_RST			BIT(10)
#define BAIKAL_PCIE_CORE_RST			BIT(8)
#define BAIKAL_PCIE_PWR_RST			BIT(9)
#define BAIKAL_PCIE_NONSTICKY_RST		BIT(11)
#define BAIKAL_PCIE_HOT_RST			BIT(12)
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

static int baikal_pcie_link_up(struct dw_pcie *pcie)
{
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	u32 reg;

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), &reg);
	if (!(reg & BAIKAL_PCIE_LTSSM_ENABLE))
		return 0;

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->rc_num), &reg);
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

static int baikal_pcie_access_other_conf(struct pcie_port *pp, struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val, bool write)
{
	int ret;
	u32 busdev;
	void __iomem *va_cfg_base;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pci);

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
						  PCIE_ATU_TYPE_CFG1, pp->cfg0_base,
						  busdev, PCIE_ATU_MIN_SIZE);
			rc->cfg_busdev = busdev;
		}
		va_cfg_base = pp->va_cfg0_base;
	}

	if (write)
		ret = dw_pcie_write(va_cfg_base + where, size, *val);
	else
		ret = dw_pcie_read(va_cfg_base + where, size, val);

	return ret;
}

static int baikal_pcie_rd_other_conf(struct pci_bus *bus,
		u32 devfn, int where, int size, u32 *val)
{
	struct pcie_port *pp = bus->sysdata;
	return baikal_pcie_access_other_conf(pp, bus, devfn, where, size, val,
			false);
}

static int baikal_pcie_wr_other_conf(struct pci_bus *bus,
		u32 devfn, int where, int size, u32 val)
{
	struct pcie_port *pp = bus->sysdata;
	return baikal_pcie_access_other_conf(pp, bus, devfn, where, size, &val,
			true);
}

static struct pci_ops baikal_child_pcie_ops = {
	.read = baikal_pcie_rd_other_conf,
	.write = baikal_pcie_wr_other_conf,
};

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	struct resource_entry *tmp, *entry = NULL;
	u32 lcru_reg, class_reg;
	phys_addr_t cfg_base, cfg_end;
	u32 cfg_size;

	/* Configure MEM and I/O ATU regions */
	/* Get last memory resource entry */
	resource_list_for_each_entry(tmp, &pp->bridge->windows)
		if (resource_type(tmp->res) == IORESOURCE_MEM)
			entry = tmp;

	dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX0,
			PCIE_ATU_TYPE_MEM, entry->res->start,
			entry->res->start - entry->offset,
			resource_size(entry->res));
	dw_pcie_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX1,
			PCIE_ATU_TYPE_IO, pp->io_base,
			pp->io_bus_addr, pp->io_size);

	/* Check, if we can set up ECAM */
	cfg_base = pp->cfg0_base;
	cfg_size = pp->cfg0_size;
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
		pp->va_cfg0_base = ioremap(cfg_base, cfg_size);
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
		rc->max_bus = cfg_size >> PCIE_ECAM_BUS_SHIFT;
		dev_info(pcie->dev, "ECAM configured. Max bus %u\n",
			 rc->max_bus);
	}

	// Set class
	regmap_read(rc->lcru,
		    BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), &lcru_reg);
	regmap_write(rc->lcru,
		     BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num),
		     lcru_reg & (~BAIKAL_PCIE_DBI2_MODE));

	dw_pcie_dbi_ro_wr_en(pcie);
	class_reg = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION);

	class_reg = (PCI_CLASS_BRIDGE_PCI << 16) | (1 << 8) | (class_reg & 0xff);
	// class PCI_PCI_BRIDGE=0x604, prog-if=1
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, class_reg);

	/* Hide MSI and MSI-X capabilities */
	dw_pcie_writeb_dbi(pcie, 0x41, 0x70); /* point to next cap - skip MSI */
	dw_pcie_writeb_dbi(pcie, 0x71, 0x00); /* end of caps - skip MSI-X */

	dw_pcie_dbi_ro_wr_dis(pcie);

	pp->bridge->child_ops = &baikal_child_pcie_ops;

	dw_pcie_setup_rc(pp);
	baikal_pcie_establish_link(pcie); // This call waits for training completion

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

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.host_init = baikal_pcie_host_init,
	.msi_host_init = baikal_pcie_msi_host_init,
};

static const struct dw_pcie_ops baikal_pcie_ops = {
	.link_up = baikal_pcie_link_up,
};

static int baikal_add_pcie_port(struct baikal_pcie_rc *rc,
				struct platform_device *pdev)
{
	struct dw_pcie *pcie = rc->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	pcie->dev = &pdev->dev;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		pcie->dbi_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->dbi_base)) {
			dev_err(dev, "error with ioremap\n");
			return PTR_ERR(pcie->dbi_base);
		}
	} else {
		dev_err(dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	pp->ops = &baikal_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int baikal_pcie_hw_init_m(struct baikal_pcie_rc *rc)
{
	u32 reg;

	/* Cease link */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), &reg);
	reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), reg);

	/* Force controller reset */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), &reg);
	reg |= BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
	       BAIKAL_PCIE_PIPE0_RST | BAIKAL_PCIE_PHY_RST;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), reg);

	/* Wait */
	usleep_range(80000, 100000);

	/* Deassert PHY reset */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), &reg);
	reg &= ~BAIKAL_PCIE_PHY_RST;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), reg);

	/* Enable access to the PHY registers */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), &reg);
	reg |= BAIKAL_PCIE_PHY_MGMT_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), reg);

	/* Clear all software controlled resets of the controller */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), &reg);
	reg &= ~(BAIKAL_PCIE_ADB_PWRDWN | BAIKAL_PCIE_HOT_RST |
		 BAIKAL_PCIE_NONSTICKY_RST | BAIKAL_PCIE_STICKY_RST |
		 BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST |
		 BAIKAL_PCIE_PIPE0_RST | BAIKAL_PCIE_PIPE1_RST);
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), reg);

	/* Start LTSSM */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), &reg);
	reg |= BAIKAL_PCIE_LTSSM_ENABLE;
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->rc_num), reg);

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
	rc->rc_num = index[1];

	pm_runtime_enable(dev);
	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->rc_num), &reg);
	if ((reg & (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) ==
	    (BAIKAL_PCIE_RDLH_LINKUP | BAIKAL_PCIE_SMLH_LINKUP)) {
		link_up = 1;
	}
	dev_dbg(dev, "%s: bus %d - link state %x, reset %x\n",
		__func__, rc->rc_num, reg,
		(regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->rc_num), &reg), reg));
	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		snprintf(rc->reset_name, 32, "pcie%d-reset", rc->rc_num);
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
static int baikal_pcie_get_res_acpi(struct acpi_device *adev,
				    struct acpi_device **child,
				    struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct resource_entry *entry;
	struct list_head list, *pos;
	struct fwnode_handle *fwnode;
	int ret;
	unsigned long flags = IORESOURCE_MEM;

	fwnode = fwnode_get_named_child_node(&adev->fwnode, "RES0");
	if (!fwnode) {
		dev_err(dev, "failed to get RES0 subdevice\n");
		return -EINVAL;
	}

	*child = to_acpi_device_node(fwnode);
	if (!*child) {
		dev_err(dev, "RES0 is not an acpi device node\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(*child, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse RES0._CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret > 0) { //vvv: debug
		pos = list.next;
		entry = list_entry(pos, struct resource_entry, node);
		dev_info(dev, "Res0: %pR\n", entry->res);
		if(ret > 1) {
			pos = pos->next;
			entry = list_entry(pos, struct resource_entry, node);
			dev_info(dev, "Res1: %pR\n", entry->res);
		}
	}

	if (ret != 2) {
		dev_err(dev, "invalid number of MEM resources present in RES0._CRS (%i, need 2)\n", ret);
		return -EINVAL;
	}

	/* ECAM (not used) */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);

	/* DBI */
	if (pp->cfg0_base == entry->res->start) {
		pos = pos->next;
		entry = list_entry(pos, struct resource_entry, node);
	}

	to_dw_pcie_from_pp(pp)->dbi_base = devm_ioremap_resource(dev,
								 entry->res);
	if (IS_ERR(to_dw_pcie_from_pp(pp)->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(to_dw_pcie_from_pp(pp)->dbi_base);
		return ret;
	}

	acpi_dev_free_resource_list(&list);

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

static struct regmap *baikal_pcie_get_lcru_acpi(struct acpi_device *adev,
						struct baikal_pcie_rc *rc)
{
	struct device *dev = &adev->dev;
	struct acpi_device *lcru;
	struct regmap *regmap = NULL;
	union acpi_object *package = NULL;
	union acpi_object *element = NULL;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status = AE_OK;
	int ret;

	status = acpi_evaluate_object_typed(adev->handle, "LCRU", NULL,
					    &buffer, ACPI_TYPE_PACKAGE);
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

	if (element->type != ACPI_TYPE_LOCAL_REFERENCE ||
	    !element->reference.handle) {
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

	rc->rc_num = element->integer.value;
	if (baikal_regmap && lcru == baikal_lcru) {
		regmap = baikal_regmap;
	} else {
		regmap = baikal_get_regmap(lcru);
	}

ret:
	acpi_os_free(buffer.pointer);
	return regmap;
}

static int baikal_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct acpi_device *adev = to_acpi_device(dev), *child;
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

	dev_dbg(dev, "baikal_pcie_init: window %pR\n", &cfg->res); //vvv

	rc->pcie = pcie;
	cfg->priv = rc;
	pp = &pcie->pp;
	pp->cfg0_base = cfg->res.start;
	pp->cfg0_size = resource_size(&cfg->res);
	dev_set_drvdata(dev, rc);

	rc->lcru = baikal_pcie_get_lcru_acpi(adev, rc);

	if (IS_ERR_OR_NULL(rc->lcru)) {
		dev_err(dev, "No LCRU specified\n");
		return -EINVAL;
	}

	if (rc->rc_num > 2) {
		dev_err(dev, "incorrect LCRU index\n");
		return -EINVAL;
        }

	/* TODO: gpio */

	ret = baikal_pcie_get_res_acpi(adev, &child, pp);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	pp->ops = &baikal_pcie_host_ops;

	raw_spin_lock_init(&pp->lock);
	pp->va_cfg0_base = cfg->win;
	if (!pp->va_cfg0_base) {
		dev_err(dev, "error with ioremap\n");
		return -ENOMEM;
	}

	status = acpi_evaluate_integer(adev->handle, "NUMV", NULL,
				       &num_viewport);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get num-viewport\n");
		return -EINVAL;
	}
	pcie->num_viewport = num_viewport;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "failed to enable DMA\n");
		return ret;
	}

	return ret;
}

static void __iomem *baikal_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
					 int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie_rc *priv = cfg->priv;

	if (bus->number != cfg->busr.start &&
	    !baikal_pcie_link_up(priv->pcie)) {
		return NULL;
	}

	if (bus->number == cfg->busr.start &&
	    PCI_SLOT(devfn) > 0) {
		/*
		 * The DW PCIe core doesn't filter out transactions to other
		 * devices/functions on the root bus num, so we do this here.
		 */
		return NULL;
	}

	return pci_ecam_map_bus(bus, devfn, where);
}

const struct pci_ecam_ops baikal_m_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= baikal_pcie_init,
	.pci_ops	= {
		.map_bus	= baikal_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif
