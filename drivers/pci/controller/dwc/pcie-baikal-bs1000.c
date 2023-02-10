// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Baikal BE-S1000
 *
 * Copyright (C) 2022 Baikal Electronics, JSC
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pci-ecam.h>
#include <linux/pm_runtime.h>

#include "pcie-designware.h"

struct baikal_pcie {
	struct dw_pcie	*pci;
	void __iomem	*apb_base;
	u64		cpu_addr_mask;
	bool		its_msi;
};

struct baikal_pcie_of_data {
	enum dw_pcie_device_mode	mode;
};

#define to_baikal_pcie(x)	dev_get_drvdata((x)->dev)

#define PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG		0x78
#define PCIE_CAP_CORR_ERR_REPORT_EN			BIT(0)
#define PCIE_CAP_NON_FATAL_ERR_REPORT_EN		BIT(1)
#define PCIE_CAP_FATAL_ERR_REPORT_EN			BIT(2)
#define PCIE_CAP_UNSUPPORT_REQ_REP_EN			BIT(3)

#define PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG		0x8c
#define PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN			BIT(0)
#define PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN		BIT(1)
#define PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN		BIT(2)
#define PCIE_CAP_PME_INT_EN				BIT(3)

#define PCIE_UNCORR_ERR_STATUS_REG			0x104
#define PCIE_CORR_ERR_STATUS_REG			0x110

#define PCIE_ROOT_ERR_CMD_REG				0x12c
#define PCIE_CORR_ERR_REPORTING_EN			BIT(0)
#define PCIE_NON_FATAL_ERR_REPORTING_EN			BIT(1)
#define PCIE_FATAL_ERR_REPORTING_EN			BIT(2)

#define PCIE_ROOT_ERR_STATUS_REG			0x130

#define PCIE_ATU_MIN_SIZE				0x10000		/* 64K */
#define PCIE_ATU_REGION_INDEX3				0x3
#define PCIE_ATU_CR2_CFG_SHIFT	BIT(28)
#define PCIE_ECAM_SIZE					0x10000000	/* 256M */
#define PCIE_ECAM_MASK					0x0fffffffULL
#define PCIE_ECAM_BUS_SHIFT				20
#define PCIE_ECAM_DEVFN_SHIFT				12

#define PCIE_IATU_REGION_CTRL_2_REG_SHIFT_MODE		BIT(28)

#define BS1000_PCIE_APB_PE_GEN_CTRL3			0x58
#define BS1000_PCIE_APB_PE_GEN_CTRL3_LTTSM_EN		BIT(0)

#define BS1000_PCIE_APB_PE_LINK_DBG2			0xb4
#define BS1000_PCIE_APB_PE_LINK_DBG2_LTSSM_STATE_L0	0x11
#define BS1000_PCIE_APB_PE_LINK_DBG2_LTSSM_STATE_MASK	0x3f
#define BS1000_PCIE_APB_PE_LINK_DBG2_SMLH_LINK_UP	BIT(6)
#define BS1000_PCIE_APB_PE_LINK_DBG2_RDLH_LINK_UP	BIT(7)

#define BS1000_PCIE_APB_PE_ERR_STS			0xe0
#define BS1000_PCIE_APB_PE_INT_STS			0xe8

#define BS1000_PCIE0_P0_DBI_BASE			0x39000000
#define BS1000_PCIE0_P1_DBI_BASE			0x39400000
#define BS1000_PCIE1_P0_DBI_BASE			0x3d000000
#define BS1000_PCIE1_P1_DBI_BASE			0x3d400000
#define BS1000_PCIE2_P0_DBI_BASE			0x45000000
#define BS1000_PCIE2_P1_DBI_BASE			0x45400000

static u64 baikal_pcie_cpu_addr_fixup(struct dw_pcie *pcie, u64 cpu_addr)
{
	struct baikal_pcie *bp = to_baikal_pcie(pcie);

	return cpu_addr & bp->cpu_addr_mask;
}

static int baikal_pcie_establish_link(struct dw_pcie *pci)
{
	struct baikal_pcie *bp = to_baikal_pcie(pci);
	u32 reg;

	reg = readl(bp->apb_base + BS1000_PCIE_APB_PE_GEN_CTRL3);
	reg |= BS1000_PCIE_APB_PE_GEN_CTRL3_LTTSM_EN;
	writel(reg, bp->apb_base + BS1000_PCIE_APB_PE_GEN_CTRL3);

	return 0;
}

static int baikal_pcie_link_up(struct dw_pcie *pcie)
{
	struct baikal_pcie *bp = to_baikal_pcie(pcie);
	u32 reg;

	reg = readl(bp->apb_base + BS1000_PCIE_APB_PE_LINK_DBG2);
	return (reg & BS1000_PCIE_APB_PE_LINK_DBG2_SMLH_LINK_UP) &&
		(reg & BS1000_PCIE_APB_PE_LINK_DBG2_RDLH_LINK_UP);
}

static void __iomem *bs_pcie_map_bus(struct pci_bus *bus,
				     unsigned int devfn, int where)
{
	struct pcie_port *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (!baikal_pcie_link_up(pci)) {
		return NULL;
	}

	if (bus->number == 1 && PCI_SLOT(devfn) > 0) {
		return NULL;
	} else {
		return pp->va_cfg0_base +
			(bus->number << PCIE_ECAM_BUS_SHIFT) +
			(devfn << PCIE_ECAM_DEVFN_SHIFT) +
			where;
	}
}

static struct pci_ops baikal_s_child_pcie_ops = {
	.map_bus	= bs_pcie_map_bus,
	.read		= pci_generic_config_read,
	.write		= pci_generic_config_write
};

static void bs_writel_ob_atu(struct dw_pcie *pcie, u32 index,
			  u32 reg, u32 val)
{
	u32 offset = index << 9;
	writel(val, pcie->atu_base + offset + reg);
}

static void bs_prog_outbound_atu(struct dw_pcie *pcie, int index, int type,
				 u64 cpu_addr, u64 pci_addr, u32 size, u32 cr2)
{
	u64 limit_addr = cpu_addr + size - 1;

	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_LOWER_BASE,
			 lower_32_bits(cpu_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_UPPER_BASE,
			 upper_32_bits(cpu_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_LOWER_LIMIT,
			 lower_32_bits(limit_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_UPPER_LIMIT,
			 upper_32_bits(limit_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_LOWER_TARGET,
			 lower_32_bits(pci_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_UPPER_TARGET,
			 upper_32_bits(pci_addr));
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_REGION_CTRL1, type);
	bs_writel_ob_atu(pcie, index, PCIE_ATU_UNR_REGION_CTRL2,
			 PCIE_ATU_ENABLE | cr2);

}

static void baikal_pcie_setup_rc_acpi(struct pcie_port *pp);

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	u32 reg;
	struct resource_entry *tmp, *entry = NULL;

	if (pp->cfg0_base & PCIE_ECAM_MASK ||
	    pp->cfg0_size < PCIE_ECAM_SIZE) {
		dev_warn(pcie->dev, "No ECAM due to config region size/alignment!\n");
		goto skip_atu;
	}

	/* Initialize all outbound iATU regions */
	pcie->atu_base = pcie->dbi_base + DEFAULT_DBI_ATU_OFFSET;
	/* CFG0 for bus 1 */
	bs_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX0,
			     PCIE_ATU_TYPE_CFG0,
			     baikal_pcie_cpu_addr_fixup(pcie,
				 pp->cfg0_base + (1 << PCIE_ECAM_BUS_SHIFT)),
			     0, PCIE_ATU_MIN_SIZE, PCIE_ATU_CR2_CFG_SHIFT);
	/* CFG1 for bus > 1 */
	bs_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX1,
			     PCIE_ATU_TYPE_CFG1,
			     baikal_pcie_cpu_addr_fixup(pcie,
				 pp->cfg0_base + (2 << PCIE_ECAM_BUS_SHIFT)),
			     0, PCIE_ECAM_SIZE - (2 << PCIE_ECAM_BUS_SHIFT),
			     PCIE_ATU_CR2_CFG_SHIFT);

	/* Get last memory resource entry */
	resource_list_for_each_entry(tmp, &pp->bridge->windows)
		if (resource_type(tmp->res) == IORESOURCE_MEM)
			entry = tmp;

	bs_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX2,
			     PCIE_ATU_TYPE_MEM,
			     baikal_pcie_cpu_addr_fixup(pcie, entry->res->start),
			     entry->res->start - entry->offset,
			     resource_size(entry->res), 0);

	bs_prog_outbound_atu(pcie, PCIE_ATU_REGION_INDEX3,
			     PCIE_ATU_TYPE_IO,
			     baikal_pcie_cpu_addr_fixup(pcie, pp->io_base),
			     pp->io_bus_addr, pp->io_size, 0);

	pp->bridge->child_ops = &baikal_s_child_pcie_ops;

skip_atu:
	if (acpi_disabled) {
		dw_pcie_setup_rc(pp);
	} else {
		baikal_pcie_setup_rc_acpi(pp);
	}

	/* Set prog-if 01 [subtractive decode] */
	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION);
	reg = (reg & 0xffff00ff) | (1 << 8);
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Enable error reporting */
	reg = dw_pcie_readl_dbi(pcie, PCIE_ROOT_ERR_CMD_REG);
	reg |= PCIE_CORR_ERR_REPORTING_EN      |
	       PCIE_NON_FATAL_ERR_REPORTING_EN |
	       PCIE_FATAL_ERR_REPORTING_EN;
	dw_pcie_writel_dbi(pcie, PCIE_ROOT_ERR_CMD_REG, reg);

	reg = dw_pcie_readl_dbi(pcie, PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG);
	reg |= PCIE_CAP_CORR_ERR_REPORT_EN	|
	       PCIE_CAP_NON_FATAL_ERR_REPORT_EN	|
	       PCIE_CAP_FATAL_ERR_REPORT_EN	|
	       PCIE_CAP_UNSUPPORT_REQ_REP_EN;
	dw_pcie_writel_dbi(pcie, PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG, reg);

	reg = dw_pcie_readl_dbi(pcie, PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG);
	reg |= PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN	    |
	       PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN |
	       PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN	    |
	       PCIE_CAP_PME_INT_EN;
	dw_pcie_writel_dbi(pcie, PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG, reg);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		struct baikal_pcie *bp = to_baikal_pcie(pcie);
		if (!bp->its_msi)
			dw_pcie_msi_init(pp);
	}

	return 0;
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp)
{
	return 0;
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.host_init = baikal_pcie_host_init
};

static const struct dw_pcie_host_ops baikal_pcie_host_its_msi_ops = {
	.host_init = baikal_pcie_host_init,
	.msi_host_init = baikal_pcie_msi_host_init,
};

static irqreturn_t baikal_pcie_intr_irq_handler(int irq, void *arg)
{
	struct baikal_pcie *bp = arg;
	struct dw_pcie *pcie = bp->pci;
	struct device *dev = pcie->dev;

	u32 apb_pe_err_status;
	u32 apb_pe_int_status;
	u32 corr_err_status;
	u32 dev_ctrl_dev_status;
	u32 root_err_status;
	u32 uncorr_err_status;

	apb_pe_err_status   = readl(bp->apb_base + BS1000_PCIE_APB_PE_ERR_STS);
	apb_pe_int_status   = readl(bp->apb_base + BS1000_PCIE_APB_PE_INT_STS);
	uncorr_err_status   = dw_pcie_readl_dbi(pcie,
					 PCIE_UNCORR_ERR_STATUS_REG);
	corr_err_status	    = dw_pcie_readl_dbi(pcie,
					 PCIE_CORR_ERR_STATUS_REG);
	root_err_status	    = dw_pcie_readl_dbi(pcie,
					 PCIE_ROOT_ERR_STATUS_REG);
	dev_ctrl_dev_status = dw_pcie_readl_dbi(pcie,
					 PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG);

	writel(apb_pe_err_status, bp->apb_base + BS1000_PCIE_APB_PE_ERR_STS);
	writel(apb_pe_int_status, bp->apb_base + BS1000_PCIE_APB_PE_INT_STS);
	dw_pcie_writel_dbi(pcie,
		    PCIE_UNCORR_ERR_STATUS_REG, uncorr_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_CORR_ERR_STATUS_REG, corr_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_ROOT_ERR_STATUS_REG, root_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG, dev_ctrl_dev_status);

	dev_err(dev,
		"DevErr:0x%x RootErr:0x%x UncorrErr:0x%x CorrErr:0x%x ApbErr:0x%x ApbInt:0x%x\n",
		(dev_ctrl_dev_status & 0xf0000) >> 16,
		root_err_status, uncorr_err_status, corr_err_status,
		apb_pe_err_status, apb_pe_int_status);

	return IRQ_HANDLED;
}

static int baikal_pcie_add_pcie_port(struct baikal_pcie *bp,
				     struct platform_device *pdev)
{
	struct dw_pcie *pcie = bp->pci;
	struct pcie_port *pp = &pcie->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq = platform_get_irq_byname(pdev, "intr");
	if (pp->irq < 0) {
		dev_err(dev, "failed to get \"intr\" IRQ\n");
		return pp->irq;
	}

	ret = devm_request_irq(dev, pp->irq, baikal_pcie_intr_irq_handler,
			       IRQF_SHARED, "bs1000-pcie-intr", bp);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d\n", pp->irq);
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq < 0) {
			dev_err(dev, "failed to get \"msi\" IRQ\n");
			return pp->msi_irq;
		}
	}

	if (bp->its_msi) {
		dev_dbg(dev, "its_msi_ops\n");
		pp->ops = &baikal_pcie_host_its_msi_ops;
	} else {
		pp->ops = &baikal_pcie_host_ops;
		dev_dbg(dev, "dw_msi_ops\n");
	}
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void baikal_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	for (bar = BAR_0; bar <= BAR_5; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
}

static int baikal_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				    enum pci_epc_irq_type type,
				    u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
		return -EINVAL;
	}
}

static const struct pci_epc_features baikal_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	.reserved_bar = BIT(BAR_3) | BIT(BAR_5),
};

static const struct pci_epc_features*
baikal_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	return &baikal_pcie_epc_features;
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = baikal_pcie_ep_init,
	.raise_irq = baikal_pcie_ep_raise_irq,
	.get_features = baikal_pcie_ep_get_features,
};

static int baikal_pcie_add_pcie_ep(struct baikal_pcie *bp,
				   struct platform_device *pdev)
{
	int ret;
	struct dw_pcie_ep *ep;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = bp->pci;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	pci->dbi_base2 = pci->dbi_base + 0x100000;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int baikal_pcie_pm_resume(struct device *dev)
{
	struct baikal_pcie *bp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = bp->pci;
	u32 reg;

	/* Set Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	reg |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, reg);
	return 0;
}

static int baikal_pcie_pm_resume_noirq(struct device *dev)
{
	return 0;
}

static int baikal_pcie_pm_suspend(struct device *dev)
{
	struct baikal_pcie *bp = dev_get_drvdata(dev);
	struct dw_pcie *pcie = bp->pci;
	u32 reg;

	/* Clear Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	reg &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, reg);
	return 0;
}

static int baikal_pcie_pm_suspend_noirq(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops baikal_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(baikal_pcie_pm_suspend,
				baikal_pcie_pm_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(baikal_pcie_pm_suspend_noirq,
				      baikal_pcie_pm_resume_noirq)
};

static const struct baikal_pcie_of_data baikal_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct baikal_pcie_of_data baikal_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id baikal_pcie_of_match[] = {
	{
		.compatible = "baikal,bs1000-pcie",
		.data = &baikal_pcie_rc_of_data,
	},
	{
		.compatible = "baikal,bs1000-pcie-ep",
		.data = &baikal_pcie_ep_of_data,
	},
	{}
};
MODULE_DEVICE_TABLE(of, baikal_pcie_of_match);

static const struct dw_pcie_ops baikal_pcie_ops = {
	.cpu_addr_fixup = baikal_pcie_cpu_addr_fixup,
	.link_up = baikal_pcie_link_up,
	.start_link = baikal_pcie_establish_link,
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_pcie *bp;
	struct dw_pcie *pcie;
	int ret;
	struct resource *res;
	const struct baikal_pcie_of_data *data;
	const struct of_device_id *match;
	enum dw_pcie_device_mode mode;
	const __be32 *msi_map;
	int len;
	u32 phandle;

	match = of_match_device(baikal_pcie_of_match, dev);
	if (!match) {
		return -EINVAL;
	}

	data = (struct baikal_pcie_of_data *)match->data;
	mode = (enum dw_pcie_device_mode)data->mode;


	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		return -ENOMEM;
	}

	pcie->dev = dev;
	pcie->ops = &baikal_pcie_ops;

	bp = devm_kzalloc(dev, sizeof(*bp), GFP_KERNEL);
	if (!bp) {
		return -ENOMEM;
	}

	bp->pci = pcie;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}

	platform_set_drvdata(pdev, bp);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		devm_request_resource(dev, &iomem_resource, res);
		pcie->dbi_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->dbi_base)) {
			dev_err(dev, "error with ioremap\n");
			ret = PTR_ERR(pcie->dbi_base);
			goto err_pm_put;
		}

		if (res->start == BS1000_PCIE0_P0_DBI_BASE ||
		    res->start == BS1000_PCIE0_P1_DBI_BASE ||
		    res->start == BS1000_PCIE1_P0_DBI_BASE ||
		    res->start == BS1000_PCIE1_P1_DBI_BASE ||
		    res->start == BS1000_PCIE2_P0_DBI_BASE ||
		    res->start == BS1000_PCIE2_P1_DBI_BASE) {
			bp->cpu_addr_mask = 0x7fffffffff;
		} else {
			bp->cpu_addr_mask = 0xffffffffff;
		}
	} else {
		dev_err(dev, "missing *dbi* reg space\n");
		ret = -EINVAL;
		goto err_pm_put;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	if (res) {
		devm_request_resource(dev, &iomem_resource, res);
		bp->apb_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(bp->apb_base)) {
			dev_err(dev, "error with ioremap\n");
			ret = PTR_ERR(bp->apb_base);
			goto err_pm_put;
		}
	} else {
		dev_err(dev, "missing *apb* reg space\n");
		ret = -EINVAL;
		goto err_pm_put;
	}

	msi_map = of_get_property(dev->of_node, "msi-map", &len);
	if (msi_map) {
		phandle = be32_to_cpup(msi_map + 1);
		if (of_find_node_by_phandle(phandle))
			bp->its_msi = true;
	}

	switch (mode) {
	case DW_PCIE_RC_TYPE:
		ret = baikal_pcie_add_pcie_port(bp, pdev);
		if (ret < 0) {
			goto err_pm_put;
		}

		break;
	case DW_PCIE_EP_TYPE:
		ret = baikal_pcie_add_pcie_ep(bp, pdev);
		if (ret < 0) {
			goto err_pm_put;
		}

		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	return 0;

err_pm_put:
	pm_runtime_put(dev);
err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static struct platform_driver baikal_pcie_bs1000_driver = {
	.driver = {
		.name = "baikal-pcie-bs1000",
		.of_match_table = baikal_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &baikal_pcie_pm_ops
	},
	.probe = baikal_pcie_probe
};

module_platform_driver(baikal_pcie_bs1000_driver);

#ifdef CONFIG_ACPI
static void baikal_pcie_setup_rc_acpi(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	acpi_status status;
	u64 lanes;
	u32 val;

	/*
	 * Enable DBI read-only registers for writing/updating configuration.
	 * Write permission gets disabled towards the end of this function.
	 */
	dw_pcie_dbi_ro_wr_en(pci);

	status = acpi_evaluate_integer(to_acpi_device(pci->dev)->handle,
				       "NUML", NULL, &lanes);
	if (ACPI_FAILURE(status)) {
		dev_dbg(pci->dev, "failed to get num-lanes\n");
	} else {
		/* Set the number of lanes */
		val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
		val &= ~PORT_LINK_MODE_MASK;
		switch (lanes) {
		case 1:
			val |= PORT_LINK_MODE_1_LANES;
			break;
		case 2:
			val |= PORT_LINK_MODE_2_LANES;
			break;
		case 4:
			val |= PORT_LINK_MODE_4_LANES;
			break;
		case 8:
			val |= PORT_LINK_MODE_8_LANES;
			break;
		default:
			dev_err(pci->dev, "NUML %llu: invalid value\n", lanes);
			goto skip_lanes;
		}

		dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

		/* Set link width speed control register */
		val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
		switch (lanes) {
		case 1:
			val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
			break;
		case 2:
			val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
			break;
		case 4:
			val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
			break;
		case 8:
			val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
			break;
		}
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);
	}

skip_lanes:
	/* Setup RC BARs */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0x00000004);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0x00000000);

	/* Setup interrupt pins */
	val = dw_pcie_readl_dbi(pci, PCI_INTERRUPT_LINE);
	val &= 0xffff00ff;
	val |= 0x00000100;
	dw_pcie_writel_dbi(pci, PCI_INTERRUPT_LINE, val);

	/* Setup bus numbers */
	val = dw_pcie_readl_dbi(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00ff0100;
	dw_pcie_writel_dbi(pci, PCI_PRIMARY_BUS, val);

	/* Setup command register */
	val = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, val);

	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);

	/* Program correct class for RC */
	dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_PCI);

	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val |= PORT_LOGIC_SPEED_CHANGE;
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	dw_pcie_dbi_ro_wr_dis(pci);
}

static int baikal_pcie_get_res_acpi(struct acpi_device *adev,
				    struct acpi_device **child,
				    struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *bp = to_baikal_pcie(pcie);
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

	if (ret != 3) {
		dev_err(dev, "invalid number of MEM resources present in RES0._CRS (%i, need 3)\n", ret);
		return -EINVAL;
	}

	/* ECAM */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->cfg0_size = resource_size(entry->res);
	pp->cfg0_base = entry->res->start;

	/* DBI */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	if (entry->res->start == 0x39000000 ||
	    entry->res->start == 0x39400000 ||
	    entry->res->start == 0x3d000000 ||
	    entry->res->start == 0x3d400000 ||
	    entry->res->start == 0x45000000 ||
	    entry->res->start == 0x45400000) {
		bp->cpu_addr_mask = 0x7fffffffff;
	} else {
		bp->cpu_addr_mask = 0xffffffffff;
	}

	pcie->dbi_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(pcie->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(pcie->dbi_base);
		return ret;
	}

	pcie->iatu_unroll_enabled = true;
	pcie->atu_base = pcie->dbi_base + DEFAULT_DBI_ATU_OFFSET;

	/* APB */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	bp->apb_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(bp->apb_base)) {
		dev_err(dev, "error with apb ioremap\n");
		ret = PTR_ERR(bp->apb_base);
		return ret;
	}

	acpi_dev_free_resource_list(&list);

	/* Non-prefetchable memory */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_MEM;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);

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

static int baikal_pcie_get_irq_acpi(struct acpi_device *adev,
				    struct acpi_device *child,
				    struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *bp = to_baikal_pcie(pcie);
	struct device *dev = &adev->dev;
	struct resource res;
	int ret;

	memset(&res, 0, sizeof(res));

	ret = acpi_irq_get(child->handle, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get irq %d\n", 0);
		return ret;
	}

	if (res.flags & IORESOURCE_BITS) {
		struct irq_data *irqd;

		irqd = irq_get_irq_data(res.start);
		if (!irqd) {
			return -ENXIO;
		}

		irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
	}

	pp->irq = res.start;
	ret = devm_request_irq(dev, pp->irq, baikal_pcie_intr_irq_handler,
			       IRQF_SHARED, "bs1000-pcie-intr", bp);
	if (ret) {
		dev_err(dev, "failed to request IRQ %d\n", pp->irq);
		return ret;
	}

	return 0;
}

static struct device_type baikal_pcie_acpi_device_type = {
	.name =         "baikal_pcie_acpi",
#if defined(CONFIG_PM) && defined(CONFIG_PM_SLEEP)
	.pm =           &baikal_pcie_pm_ops
#endif
};

static int baikal_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct acpi_device *adev = to_acpi_device(dev), *child;
	struct baikal_pcie *bp;
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

	bp = devm_kzalloc(dev, sizeof(*bp), GFP_KERNEL);
	if (!bp) {
		return -ENOMEM;
	}

	bp->pci = pcie;
	cfg->priv = bp;
	pp = &pcie->pp;
	dev_set_drvdata(dev, bp);

	ret = baikal_pcie_get_res_acpi(adev, &child, pp);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	ret = baikal_pcie_get_irq_acpi(adev, child, pp);
	if (ret) {
		dev_err(dev, "failed to get irq info\n");
		return ret;
	}

	pp->ops = &baikal_pcie_host_ops;

	raw_spin_lock_init(&pp->lock);
	pp->va_cfg0_base = devm_pci_remap_cfgspace(dev, pp->cfg0_base,
						   pp->cfg0_size);
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

	pcie->dev->type = &baikal_pcie_acpi_device_type;

	ret = baikal_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void __iomem *baikal_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie *bp = cfg->priv;
	unsigned int devfn_shift = cfg->ops->bus_shift - 8;
	unsigned int busn = bus->number;
	void __iomem *base;

	if (bus->number != cfg->busr.start &&
	    !baikal_pcie_link_up(bp->pci)) {
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
			return bp->pci->dbi_base + where;
		}
	}

	if (busn < cfg->busr.start || busn > cfg->busr.end) {
		return NULL;
	}

	busn -= cfg->busr.start;
	base = cfg->win + (busn << cfg->ops->bus_shift);
	return base + (devfn << devfn_shift) + where;
}

const struct pci_ecam_ops baikal_s_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= baikal_pcie_init,
	.pci_ops	= {
		.map_bus	= baikal_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#else /* CONFIG_ACPI */
static inline void baikal_acpi_pcie_setup_rc(struct pcie_port *pp)
{
}
#endif

MODULE_DESCRIPTION("Baikal BE-S1000 PCIe host controller driver");
MODULE_LICENSE("GPL v2");
