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
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pci-ecam.h>
#include <linux/pm_runtime.h>

#include "pcie-designware.h"

struct baikal_pcie;

struct baikal_pcie_of_data {
	enum dw_pcie_device_mode	mode;
	int				(*setup_res)(struct platform_device *, struct dw_pcie *);
	void				(*prog_ob_atu)(struct dw_pcie *pci,
						int index, int type,
		                                u64 cpu_addr, u64 pci_addr,
						u32 size, u32 cr2);
	int				(*get_link)(struct baikal_pcie *);
};

struct baikal_pcie {
	struct dw_pcie			*pci;
	struct gpio_desc		*reset_gpio;
	const struct baikal_pcie_of_data	*of_data;
	void __iomem			*gpr; // BM1000 only
	void __iomem			*apb_base; // BS1000 only
	int				cpu_addr_bits;
};

#define to_baikal_pcie(x)	dev_get_drvdata((x)->dev)

#define PCIE_ATU_MIN_SIZE				0x10000		/* 64K */
#define PCIE_ATU_CR2_CFG_SHIFT				BIT(28)
#define PCIE_ECAM_SIZE					0x10000000	/* 256M */
#define PCIE_ECAM_MASK					0x0fffffffULL
#define PCIE_ECAM_BUS_SHIFT				20
#define PCIE_ECAM_DEVFN_SHIFT				12

#define BAIKAL_PCIE_LTSSM_STATE_L0			0x11
#define BAIKAL_PCIE_LTSSM_STATE_L2W			0x16
#define BAIKAL_PCIE_LTSSM_STATE_MASK			0x3f
#define BAIKAL_PCIE_SMLH_LINK_UP			BIT(6)
#define BAIKAL_PCIE_RDLH_LINK_UP			BIT(7)

#define BM1000_LCRU_GPR_OFF				0x50000
#define BM1000_RC_GPR_SIZE				0x20
#define BM1000_GPR_RC_STATUS				0x4

#define BS1000_PCIE_APB_PE_LINK_DBG2			0xb4

#define BS1000_PCIE_APB_PE_ERR_STS			0xe0
#define BS1000_PCIE_APB_PE_INT_STS			0xe8

#define BS1000_DEFAULT_NUM_VECTORS			256

/* BM1000 specific functions */

static int bm_pcie_get_link(struct baikal_pcie *bp)
{
	return readl(bp->gpr + BM1000_GPR_RC_STATUS);
}

static void bm_prog_ob_atu(struct dw_pcie *pcie, int index, int type,
				 u64 cpu_addr, u64 pci_addr, u32 size, u32 cr2)
{
	u64 limit_addr = cpu_addr + size - 1;

	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT,
			   PCIE_ATU_REGION_DIR_OB | index);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_BASE,
			   lower_32_bits(cpu_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_BASE,
			   upper_32_bits(cpu_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LIMIT,
			   lower_32_bits(limit_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_LOWER_TARGET,
			   lower_32_bits(pci_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_UPPER_TARGET,
			   upper_32_bits(pci_addr));
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_REGION_CTRL1,
			   type);
	dw_pcie_writel_dbi(pcie, PCIE_ATU_VIEWPORT_BASE + PCIE_ATU_REGION_CTRL2,
			   PCIE_ATU_ENABLE | cr2);
}

static int bm_pcie_setup_res(struct platform_device *pdev, struct dw_pcie *pcie)
{
	int ret;
	struct resource res;
	struct baikal_pcie *bp = to_baikal_pcie(pcie);
	u32 lcru_prop[2], rc_idx;
	phandle phandle;
	struct device_node *lcru_node;

	ret = of_property_read_u32_array(pdev->dev.of_node,
					 "baikal,pcie-lcru", lcru_prop, 2);
	if (ret) {
		dev_err(&pdev->dev, "failed to read LCRU\n");
		return ret;
	}
	phandle = lcru_prop[0];
	rc_idx = lcru_prop[1];
	lcru_node = of_find_node_by_phandle(phandle);
	if (IS_ERR(lcru_node)) {
		dev_err(&pdev->dev, "Can't get lcru node\n");
		return PTR_ERR(lcru_node);
	}
	ret = of_address_to_resource(lcru_node, 0, &res);
	of_node_put(lcru_node);
	if (ret) {
		dev_err(&pdev->dev, "Can't get lcru resource: %d\n", ret);
		return ret;
	}

	bp->gpr = devm_ioremap(&pdev->dev, res.start + BM1000_LCRU_GPR_OFF +
			       BM1000_RC_GPR_SIZE * rc_idx,
			       BM1000_RC_GPR_SIZE);
	if (IS_ERR(bp->gpr)) {
		dev_err(&pdev->dev, "Can't ioremap LCRU GPR\n");
		return PTR_ERR(bp->gpr);
	}
	return 0;
}

/* BS1000 specific functions */

static int bs_pcie_get_link(struct baikal_pcie *bp)
{
	return readl(bp->apb_base + BS1000_PCIE_APB_PE_LINK_DBG2);
}

static void bs_writel_ob_atu(struct dw_pcie *pci, u32 index,
			  u32 reg, u32 val)
{
	u32 offset = index << 9;
	writel(val, pci->atu_base + offset + reg);
}

static void bs_prog_ob_atu(struct dw_pcie *pci, int index, int type,
				 u64 cpu_addr, u64 pci_addr, u32 size, u32 cr2)
{
	struct baikal_pcie *bp = to_baikal_pcie(pci);
	u64 limit_addr;

	cpu_addr &= (1ULL << bp->cpu_addr_bits) - 1;
	limit_addr = cpu_addr + size - 1;

	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_LOWER_BASE,
			 lower_32_bits(cpu_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_UPPER_BASE,
			 upper_32_bits(cpu_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_LOWER_LIMIT,
			 lower_32_bits(limit_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_UPPER_LIMIT,
			 upper_32_bits(limit_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_LOWER_TARGET,
			 lower_32_bits(pci_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_UPPER_TARGET,
			 upper_32_bits(pci_addr));
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_REGION_CTRL1, type);
	bs_writel_ob_atu(pci, index, PCIE_ATU_UNR_REGION_CTRL2,
			 PCIE_ATU_ENABLE | cr2);

}

static irqreturn_t bs_pcie_intr_irq_handler(int irq, void *arg)
{
	struct baikal_pcie *bp = arg;
	u32 apb_pe_int_status;

	/* clear interrupt status */
	apb_pe_int_status   = readl(bp->apb_base + BS1000_PCIE_APB_PE_INT_STS);
	writel(apb_pe_int_status, bp->apb_base + BS1000_PCIE_APB_PE_INT_STS);

	return IRQ_HANDLED;
}

static int bs_pcie_setup_res(struct platform_device *pdev, struct dw_pcie *pci)
{
	int ret;
	struct resource *res;
	struct baikal_pcie *bp = to_baikal_pcie(pci);
	struct dw_pcie_rp *pp = &pci->pp;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
	if (res) {
		devm_request_resource(&pdev->dev, &iomem_resource, res);
		pci->atu_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(pci->atu_base)) {
			dev_err(&pdev->dev, "error with ioremap\n");
			return PTR_ERR(pci->atu_base);
		}
	} else {
		pci->atu_base = pci->dbi_base + DEFAULT_DBI_ATU_OFFSET;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	if (res) {
		devm_request_resource(&pdev->dev, &iomem_resource, res);
		bp->apb_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(bp->apb_base)) {
			dev_err(&pdev->dev, "error with ioremap\n");
			return PTR_ERR(bp->apb_base);
		}
	} else {
		dev_err(&pdev->dev, "missing *apb* reg space\n");
		return -EINVAL;
	}

	pp->irq = platform_get_irq_byname(pdev, "intr");
	if (pp->irq < 0) {
		dev_warn(&pdev->dev, "failed to get \"intr\" IRQ\n");
		return 0;
	}

	ret = devm_request_irq(&pdev->dev, pp->irq, bs_pcie_intr_irq_handler,
			       IRQF_SHARED, "bs1000-pcie-intr", bp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ %d\n", pp->irq);
		return ret;
	}

	return ret;
}

static int baikal_pcie_link_up_internal(struct dw_pcie *pci)
{
	struct baikal_pcie *bp = to_baikal_pcie(pci);
	u32 reg;

	reg = bp->of_data->get_link(bp);
	if (!(reg & BAIKAL_PCIE_SMLH_LINK_UP) ||
	    !(reg & BAIKAL_PCIE_RDLH_LINK_UP))
		return 0;
	reg &= BAIKAL_PCIE_LTSSM_STATE_MASK;
	if (reg < BAIKAL_PCIE_LTSSM_STATE_L0 ||
	    reg > BAIKAL_PCIE_LTSSM_STATE_L2W)
		return 0;
	return 1;
}

static void __iomem *baikal_pcie_map_bus(struct pci_bus *bus,
					 unsigned int devfn, int where)
{
	struct dw_pcie_rp *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (!baikal_pcie_link_up_internal(pci)) {
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

static struct pci_ops baikal_child_pcie_ops = {
	.map_bus	= baikal_pcie_map_bus,
	.read		= pci_generic_config_read,
	.write		= pci_generic_config_write
};

static u64 baikal_pcie_cpu_addr_fixup(struct dw_pcie *pci, u64 cpu_addr)
{
	struct baikal_pcie *bp = to_baikal_pcie(pci);

	if (bp->cpu_addr_bits == 0)
		return cpu_addr;

	return cpu_addr & ((1 << bp->cpu_addr_bits) - 1);
}

static int baikal_pcie_link_up(struct dw_pcie *pci)
{
	return 1;
}

static int baikal_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct baikal_pcie *bp = to_baikal_pcie(pci);
	struct resource_entry *tmp, *entry = NULL;
	void (*prog_ob_atu)(struct dw_pcie *pci, int index, int type,
			    u64 cpu_addr, u64 pci_addr, u32 size, u32 cr2) =
		bp->of_data->prog_ob_atu;

	if (pp->cfg0_base & PCIE_ECAM_MASK ||
	    pp->cfg0_size < PCIE_ECAM_SIZE) {
		dev_warn(pci->dev, "No ECAM due to config region size/alignment!\n");
		goto skip_atu;
	}
	if (!prog_ob_atu)
		goto skip_atu;

	/* Initialize all outbound iATU regions */
	/* CFG0 for bus 1 */
	prog_ob_atu(pci, 0, PCIE_ATU_TYPE_CFG0,
		    pp->cfg0_base + (1 << PCIE_ECAM_BUS_SHIFT),
		    0, PCIE_ATU_MIN_SIZE, PCIE_ATU_CR2_CFG_SHIFT);
	/* CFG1 for bus > 1 */
	prog_ob_atu(pci, 1, PCIE_ATU_TYPE_CFG1,
		    pp->cfg0_base + (2 << PCIE_ECAM_BUS_SHIFT),
		    0, PCIE_ECAM_SIZE - (2 << PCIE_ECAM_BUS_SHIFT),
		    PCIE_ATU_CR2_CFG_SHIFT);

	/* Get last memory resource entry */
	resource_list_for_each_entry(tmp, &pp->bridge->windows)
		if (resource_type(tmp->res) == IORESOURCE_MEM)
			entry = tmp;

	prog_ob_atu(pci, 2, PCIE_ATU_TYPE_MEM,
		    entry->res->start,
		    entry->res->start - entry->offset,
		    resource_size(entry->res), 0);

	prog_ob_atu(pci, 3, PCIE_ATU_TYPE_IO,
		    pp->io_base,
		    pp->io_bus_addr, pp->io_size, 0);

	pp->bridge->child_ops = &baikal_child_pcie_ops;

skip_atu:
	pp->num_vectors = BS1000_DEFAULT_NUM_VECTORS;

	return 0;
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.host_init = baikal_pcie_host_init
};

static int baikal_pcie_add_pcie_port(struct baikal_pcie *bp,
				     struct platform_device *pdev)
{
	struct dw_pcie *pci = bp->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->ops = &baikal_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	dev_info(dev, "Link is %s\n", baikal_pcie_link_up_internal(pci)?"Up":"Down");

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
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = bp->pci;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	pci->dbi_base2 = pci->dbi_base + 0x100000;

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
	struct dw_pcie *pci = bp->pci;
	u32 reg;

	/* Set Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	reg |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, reg);
	return 0;
}

static int baikal_pcie_pm_resume_noirq(struct device *dev)
{
	return 0;
}

static int baikal_pcie_pm_suspend(struct device *dev)
{
	struct baikal_pcie *bp = dev_get_drvdata(dev);
	struct dw_pcie *pci = bp->pci;
	u32 reg;

	/* Clear Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pci, PCI_COMMAND);
	reg &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pci, PCI_COMMAND, reg);
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

static const struct baikal_pcie_of_data bm_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
	.get_link = bm_pcie_get_link,
	.prog_ob_atu = bm_prog_ob_atu,
	.setup_res = bm_pcie_setup_res,
};

static const struct baikal_pcie_of_data bs_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
	.get_link = bs_pcie_get_link,
	.prog_ob_atu = bs_prog_ob_atu,
	.setup_res = bs_pcie_setup_res,
};

static const struct baikal_pcie_of_data baikal_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id baikal_pcie_of_match[] = {
	{
		.compatible = "baikal,bm1000-pcie",
		.data = &bm_pcie_rc_of_data,
	},
	{
		.compatible = "baikal,bs1000-pcie",
		.data = &bs_pcie_rc_of_data,
	},
	{
		.compatible = "baikal,bs1000-pcie-ep",
		.data = &baikal_pcie_ep_of_data,
	},
	{}
};

static const struct dw_pcie_ops baikal_pcie_ops = {
	.cpu_addr_fixup = baikal_pcie_cpu_addr_fixup,
	.link_up = baikal_pcie_link_up,
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_pcie *bp;
	struct dw_pcie *pci;
	int ret;
	struct resource *res;
	const struct baikal_pcie_of_data *data;
	enum dw_pcie_device_mode mode;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "Device match failed!\n");
		return -EINVAL;
	}

	mode = data->mode;


	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci) {
		return -ENOMEM;
	}

	pci->dev = dev;
	pci->ops = &baikal_pcie_ops;

	bp = devm_kzalloc(dev, sizeof(*bp), GFP_KERNEL);
	if (!bp) {
		return -ENOMEM;
	}

	bp->pci = pci;
	bp->of_data = data;

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
		pci->dbi_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pci->dbi_base)) {
			dev_err(dev, "error with ioremap\n");
			ret = PTR_ERR(pci->dbi_base);
			goto err_pm_put;
		}

	} else {
		dev_err(dev, "missing *dbi* reg space\n");
		ret = -EINVAL;
		goto err_pm_put;
	}

	if (data->setup_res)
		ret = data->setup_res(pdev, pci);
	if (ret)
		return ret;

	of_property_read_u32(dev->of_node, "cpu-addr-bits", &bp->cpu_addr_bits);

	bp->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);

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

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name = "baikal-pcie",
		.of_match_table = baikal_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &baikal_pcie_pm_ops
	},
	.probe = baikal_pcie_probe
};

builtin_platform_driver(baikal_pcie_driver);

