// SPDX-License-Identifier: GPL-2.0-only
/*
 * Baikal-M memory controller edac kernel module.
 *
 * Copyright (C) 2021 Baikal Electronics, JSC.
 *
 */

#include <linux/edac.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "edac_mc.h"

#define BE_EDAC_MSG_SIZE	80

#define DDRC_ECCCFG0		0x70	/* ECC Configuration Register */
#define DDRC_ECCCFG1		0x74	/* ECC Configuration Register */
#define DDRC_ECCSTAT		0x78	/* ECC Status Register */
#define DDRC_ECCCLR		0x7c	/* ECC Clear Register */
#define DDRC_ECCERRCNT		0x80	/* ECC Error Counter Register */
#define DDRC_ECCCADDR0		0x84	/* ECC Corrected Error Address Register 0 */
#define DDRC_ECCCADDR1		0x88	/* ECC Corrected Error Address Register 1 */
#define DDRC_ECCCSYN0		0x8c	/* ECC Corrected Syndrome Register 0 */
#define DDRC_ECCCSYN1		0x90	/* ECC Corrected Syndrome Register 1 */
#define DDRC_ECCCSYN2		0x94	/* ECC Corrected Syndrome Register 2 */
#define DDRC_ECCBITMASK0	0x98	/* ECC Corrected Data Bit Mask Register 0 */
#define DDRC_ECCBITMASK1	0x9c	/* ECC Corrected Data Bit Mask Register 1 */
#define DDRC_ECCBITMASK2	0xa0	/* ECC Corrected Data Bit Mask Register 2 */
#define DDRC_ECCUADDR0		0xa4	/* ECC Uncorrected Error Address Register 0 */
#define DDRC_ECCUADDR1		0xa8	/* ECC Uncorrected Error Address Register 1 */
#define DDRC_ECCUSYN0		0xac	/* ECC Uncorrected Syndrome Register 0 */
#define DDRC_ECCUSYN1		0xb0	/* ECC Uncorrected Syndrome Register 1 */
#define DDRC_ECCUSYN2		0xb4	/* ECC Uncorrected Syndrome Register 2 */
#define DDRC_ECCPOISONADDR0	0xb8	/* ECC Data Poisoning Address Register 0 */
#define DDRC_ECCPOISONADDR1	0xbc	/* ECC Data Poisoning Address Register 1 */

#define ECCCTL_ENABLE_INTR	0x300
#define ECCCTL_CLEAR_CERR	(1 << 0)
#define ECCCTL_CLEAR_UERR	(1 << 1)

#define ECCCNT_CERRS_MASK	(0xffff << 0)
#define ECCCNT_UERRS_SHIFT	16

#define ECCADDR_RANK_SHIFT	24
#define ECCADDR_BG_SHIFT	24
#define ECCADDR_BANK_SHIFT	16

#define ECCADDR_RANK_MASK	(0x3 << ECCADDR_RANK_SHIFT)
#define ECCADDR_ROW_MASK	(0x3ffff)
#define ECCADDR_BG_MASK		(0x3 << ECCADDR_BG_SHIFT)
#define ECCADDR_BANK_MASK	(0x7 << ECCADDR_BANK_SHIFT)
#define ECCADDR_COL_MASK	(0xfff)

struct baikal_edac_priv {
	void __iomem *baseaddr;
	int irq_cer;
	int irq_uer;
};

static int ecc_mask_bitnum(unsigned mask)
{
	int bitnum = 0;
	while (mask) {
		mask >>= 1;
		bitnum++;
	}
	return bitnum;
}

static irqreturn_t baikal_mc_err_handler(int irq, void *dev_id)
{
	u32 regaddr0, regaddr1;
	char msg[BE_EDAC_MSG_SIZE];
	struct mem_ctl_info *mci = dev_id;
	const struct baikal_edac_priv *priv = mci->pvt_info;

	if (irq == priv->irq_cer) {
		regaddr0 = readl(priv->baseaddr + DDRC_ECCCADDR0);
		regaddr1 = readl(priv->baseaddr + DDRC_ECCCADDR1);

		snprintf(msg, BE_EDAC_MSG_SIZE, "catched at "
			"Rank: %d, BankGroup: %d, Bank: %d, Row: %d, Col: %d\n",
			(regaddr0 & ECCADDR_RANK_MASK) >> ECCADDR_RANK_SHIFT,
			(regaddr1 & ECCADDR_BG_MASK) >> ECCADDR_BG_SHIFT,
			(regaddr1 & ECCADDR_BANK_MASK) >> ECCADDR_BANK_SHIFT,
			ecc_mask_bitnum(regaddr0 & ECCADDR_ROW_MASK),
			ecc_mask_bitnum(regaddr1 & ECCADDR_COL_MASK));

		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci,
				readl(priv->baseaddr + DDRC_ECCERRCNT) & ECCCNT_CERRS_MASK,
				0, 0, (u64)readl(priv->baseaddr + DDRC_ECCCSYN0) << 32 |
				readl(priv->baseaddr + DDRC_ECCCSYN1), 0, -1, -1, msg, "");

		writel(ECCCTL_ENABLE_INTR | ECCCTL_CLEAR_CERR,
				priv->baseaddr + DDRC_ECCCLR);

		return IRQ_HANDLED;
	}
	if (irq == priv->irq_uer) {
		regaddr0 = readl(priv->baseaddr + DDRC_ECCUADDR0);
		regaddr1 = readl(priv->baseaddr + DDRC_ECCUADDR1);

		snprintf(msg, BE_EDAC_MSG_SIZE, "catched at "
			"Rank: %d, BankGroup: %d, Bank: %d, Row: %d, Col: %d\n",
			(regaddr0 & ECCADDR_RANK_MASK) >> ECCADDR_RANK_SHIFT,
			(regaddr1 & ECCADDR_BG_MASK) >> ECCADDR_BG_SHIFT,
			(regaddr1 & ECCADDR_BANK_MASK) >> ECCADDR_BANK_SHIFT,
			ecc_mask_bitnum(regaddr0 & ECCADDR_ROW_MASK),
			ecc_mask_bitnum(regaddr1 & ECCADDR_COL_MASK));

		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci,
				readl(priv->baseaddr + DDRC_ECCERRCNT) >> ECCCNT_UERRS_SHIFT,
				0, 0, (u64)readl(priv->baseaddr + DDRC_ECCUSYN0) << 32 |
				readl(priv->baseaddr + DDRC_ECCUSYN1), 0, -1, -1, msg, "");

		writel(ECCCTL_ENABLE_INTR | ECCCTL_CLEAR_UERR,
				priv->baseaddr + DDRC_ECCCLR);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static bool baikal_get_ecc_state(void __iomem *base)
{
	return (readl(base + DDRC_ECCCFG0) & 0x7) == 0x4;
}

static const struct of_device_id baikal_edac_match[] = {
	{
		.compatible = "baikal,bm1000-edac-mc",
	},
	{
		/* null entry */
	}
};
MODULE_DEVICE_TABLE(of, baikal_edac_match);

static int baikal_mc_probe(struct platform_device *pdev)
{
	struct edac_mc_layer layers;
	struct baikal_edac_priv *priv;
	struct mem_ctl_info *mci;
	struct dimm_info *dimm;
	void __iomem *baseaddr;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	baseaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(baseaddr))
		return PTR_ERR(baseaddr);

	if (!baikal_get_ecc_state(baseaddr)) {
		edac_printk(KERN_INFO, EDAC_MC, "%s: ECC not enabled\n",
				pdev->name);
		return -ENXIO;
	}

	/* oversimplified layout */
	layers.type = EDAC_MC_LAYER_ALL_MEM;
	layers.size = 1;
	layers.is_virt_csrow = false;

	mci = edac_mc_alloc(0, 1, &layers,
			    sizeof(struct baikal_edac_priv));
	if (!mci) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed memory allocation for mc instance\n");
		return -ENOMEM;
	}

	priv = mci->pvt_info;
	priv->baseaddr = baseaddr;

	platform_set_drvdata(pdev, mci);

	mci->mtype_cap = MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_SECDED;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->mod_name = pdev->dev.driver->name;
	mci->dev_name = dev_name(&pdev->dev);
	mci->scrub_mode = SCRUB_HW_TUNABLE;
	mci->mc_idx = pdev->name[0] != 'e' ? 1 : 0;
	mci->pdev = &pdev->dev;

	dimm = *mci->dimms;
	dimm->mci = mci;
	dimm->grain = 1;

	priv->irq_cer = platform_get_irq(pdev, 1);
	if (priv->irq_cer < 0) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "No IRQ in DT (%d)\n", priv->irq_cer);
		ret = priv->irq_cer;
		goto free_mc;
	}
	ret = devm_request_irq(&pdev->dev, priv->irq_cer, baikal_mc_err_handler,
				0, "baikal_mc_cerr", mci);
	if (ret) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Unable to request irq %d\n", priv->irq_cer);
		goto free_mc;
	}

	priv->irq_uer = platform_get_irq(pdev, 2);
	if (priv->irq_uer < 0) {
		edac_printk(KERN_ERR, EDAC_MC,
				"No IRQ in DT (%d)\n", priv->irq_uer);
		ret = priv->irq_uer;
		goto free_irq;
	}
	ret = devm_request_irq(&pdev->dev, priv->irq_uer, baikal_mc_err_handler,
				0, "baikal_mc_uerr", mci);
	if (ret) {
		edac_printk(KERN_ERR, EDAC_MC,
				"Unable to request irq %d\n", priv->irq_uer);
		goto free_irq;
	}

	ret = edac_mc_add_mc_with_groups(mci, NULL);
	if (ret) {
		edac_printk(KERN_ERR, EDAC_MC,
				"Failed to register with EDAC core\n");
		goto free_irq;
	}

	return 0;

free_irq:
	devm_free_irq(&pdev->dev, priv->irq_cer, (void *)mci);
	devm_free_irq(&pdev->dev, priv->irq_uer, (void *)mci);

free_mc:
	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);

	return ret;
}

static int baikal_mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);
	struct baikal_edac_priv *priv = mci->pvt_info;

	devm_free_irq(&pdev->dev, priv->irq_cer, (void *)mci);
	devm_free_irq(&pdev->dev, priv->irq_uer, (void *)mci);

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);

	return 0;
}

static struct platform_driver baikal_mc_driver = {
	.driver = {
		   .name = "baikal-edac",
		   .of_match_table = baikal_edac_match,
		   },
	.probe = baikal_mc_probe,
	.remove = baikal_mc_remove,
};
module_platform_driver(baikal_mc_driver);

MODULE_VERSION("1.0");
MODULE_AUTHOR("Ivan Kapaev <Ivan.Kapaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("DDR ECC driver for Baikal SoCs");
MODULE_LICENSE("GPL v2");
