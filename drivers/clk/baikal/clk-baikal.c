// SPDX-License-Identifier: GPL-2.0
/*
 * clk-baikal.c - Baikal Electronics clock driver.
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
 *
 * Author:
 *   Ekaterina Skachko <Ekaterina.Skachko@baikalelectronics.ru>
 */
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/setup.h>
#include <linux/arm-smccc.h>

#define CMU_PLL_SET_RATE		0
#define CMU_PLL_GET_RATE		1
#define CMU_PLL_ENABLE			2
#define CMU_PLL_DISABLE			3
#define CMU_PLL_ROUND_RATE		4
#define CMU_PLL_IS_ENABLED		5
#define CMU_CLK_CH_SET_RATE		6
#define CMU_CLK_CH_GET_RATE		7
#define CMU_CLK_CH_ENABLE		8
#define CMU_CLK_CH_DISABLE		9
#define CMU_CLK_CH_ROUND_RATE		10
#define CMU_CLK_CH_IS_ENABLED		11


struct baikal_clk_cmu {
	struct clk_hw	hw;
	uint32_t	cmu_id;
	unsigned int	parent;
	const char	*name;
	spinlock_t	*lock;
	void __iomem	*reg;
	unsigned int	latency; /* ns */
	unsigned int	min, max, step;
	unsigned int	clk_ch_num;
	uint32_t	is_clk_ch;
};

#define to_baikal_cmu(_hw) container_of(_hw, struct baikal_clk_cmu, hw)

/* Pointer to the place on handling SMC CMU calls in monitor */
#define BAIKAL_SMC_LCRU_ID		0x82000000

static int baikal_clk_enable(struct clk_hw *hw)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch)
		cmd = CMU_CLK_CH_ENABLE;
	else
		cmd = CMU_PLL_ENABLE;

	pr_debug("[%s, %x:%d:%s] %s\n",
		pclk->name,
		pclk->parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"enable");

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
		pclk->parent, 0, 0, 0, &res);

	return res.a0;
}

static void baikal_clk_disable(struct clk_hw *hw)
{

	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch)
		cmd = CMU_CLK_CH_DISABLE;
	else
		cmd = CMU_PLL_DISABLE;

	pr_debug("[%s, %x:%d:%s] %s\n",
		pclk->name,
		pclk->parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"disable");

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
	 pclk->parent, 0, 0, 0, &res);
}

static int baikal_clk_is_enabled(struct clk_hw *hw)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch)
		cmd = CMU_CLK_CH_IS_ENABLED;
	else
		cmd = CMU_PLL_IS_ENABLED;

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
	 pclk->parent, 0, 0, 0, &res);

	pr_debug("[%s, %x:%d:%s] %s, %ld\n",
		pclk->name,
		pclk->parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"is enable",
		res.a0);

	return res.a0;
}

static unsigned long baikal_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;
	unsigned long parent;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_GET_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_GET_RATE;
		parent = parent_rate;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
	parent, 0, 0, 0, &res);

	pr_debug("[%s, %lx:%d:%s] %s, %ld\n",
		pclk->name,
		parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"get rate",
		res.a0);

	/* Return actual freq */
	return res.a0;

}

static int baikal_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;
	unsigned long parent;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_SET_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_SET_RATE;
		parent = parent_rate;
	}

	pr_debug("[%s, %x:%d:%s] %s, %ld\n",
		pclk->name,
		pclk->parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"set rate",
		rate);

	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, rate,
			parent, 0, 0, 0, &res);

	return res.a0;
}

static long baikal_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	unsigned long parent;
	uint32_t cmd;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_ROUND_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_ROUND_RATE;
		parent = *prate;
	}


	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, rate,
			parent, 0, 0, 0, &res);

	pr_debug("[%s, %x:%d:%s] %s, %ld\n",
		pclk->name,
		pclk->parent,
		pclk->cmu_id,
		pclk->is_clk_ch?"ch":"pll",
		"round rate",
		res.a0);

	/* Return actual freq */
	return res.a0;
}

const struct clk_ops be_clk_pll_ops = {
	.enable = baikal_clk_enable,
	.disable = baikal_clk_disable,
	.is_enabled = baikal_clk_is_enabled,
	.recalc_rate = baikal_clk_recalc_rate,
	.set_rate = baikal_clk_set_rate,
	.round_rate = baikal_clk_round_rate,
};



static int  baikal_clk_probe(struct platform_device *pdev)
{
	struct clk_init_data init;
	struct clk_init_data *init_ch;
	struct baikal_clk_cmu *cmu;
	struct baikal_clk_cmu **cmu_ch;
	struct device_node *node = pdev->dev.of_node;

	struct clk *clk;
	struct clk_onecell_data *clk_ch;

	int number, i = 0;
	u32 rc, index;
	struct property *prop;
	const __be32 *p;
	const char *clk_ch_name;
	const char *parent_name;

	cmu = kmalloc(sizeof(struct baikal_clk_cmu *), GFP_KERNEL);
	if (!cmu) {
		/* Error */
		pr_err("%s: could not allocate CMU clk\n", __func__);
		kfree(cmu);
		return -ENOMEM;
	}

	/* property */
	of_property_read_string(node, "clock-output-names", &cmu->name);
	of_property_read_u32(node, "clock-frequency", &cmu->parent);
	of_property_read_u32(node, "cmu-id", &cmu->cmu_id);

	parent_name = of_clk_get_parent_name(node, 0);

	/* Setup clock init structure */
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.name = cmu->name;
	init.ops = &be_clk_pll_ops;
	init.flags = CLK_IGNORE_UNUSED;

	cmu->hw.init = &init;
	cmu->is_clk_ch = 0;

	/* Register the clock */
	pr_debug("Add %s (Parent %s)\n",
		cmu->name, parent_name ? parent_name:"null");
	clk = clk_register(NULL, &cmu->hw);

	if (IS_ERR(clk)) {
		/* Error */
		pr_err("%s: could not register clk %s\n", __func__, cmu->name);
		return -ENOMEM;
	}

	/* Register the clock for lookup */
	rc = clk_register_clkdev(clk, cmu->name, NULL);
	if (rc != 0) {
		/* Error */
		pr_err("%s: could not register lookup clk %s\n",
			__func__, cmu->name);
	}

	/* FIXME We probably SHOULDN'T enable it here */
	clk_prepare_enable(clk);

	number = of_property_count_u32_elems(node, "clock-indices");

	if (number > 0) {
		clk_ch = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
		if (!clk_ch) {
			/* Error */
			pr_err("%s: could not allocate CMU clk channel\n",
				__func__);
			return -ENOMEM;
		}
		/* Get the last index to find out max number of children*/
		of_property_for_each_u32(node, "clock-indices",
						prop, p, index) {
			;
		}
		clk_ch->clks = kcalloc(index + 1, sizeof(struct clk *),
					GFP_KERNEL);
		clk_ch->clk_num = index + 1;
		cmu_ch = kcalloc((index + 1), sizeof(struct baikal_clk_cmu *),
				GFP_KERNEL);
		if (!cmu_ch) {
			kfree(clk_ch);
			return -ENOMEM;
		}
		init_ch = kcalloc((number + 1), sizeof(struct clk_init_data),
				GFP_KERNEL);
		if (!init_ch) {
			/* Error */
			pr_err("%s: could not allocate CMU init structure\n",
				__func__);
			kfree(cmu_ch);
			kfree(clk_ch);
			return -ENOMEM;
		}

		of_property_for_each_u32(node, "clock-indices", prop,
					p, index) {
			of_property_read_string_index(node, "clock-names",
						      i, &clk_ch_name);
			pr_debug("%s, name <%s>, index %d, i %d\n", __func__,
				clk_ch_name, index, i);

			init_ch[i].parent_names = &cmu->name;
			init_ch[i].num_parents = 1;
			init_ch[i].name = clk_ch_name;
			init_ch[i].ops = &be_clk_pll_ops;
			init_ch[i].flags = CLK_IGNORE_UNUSED;

			cmu_ch[index] = kmalloc(sizeof(struct baikal_clk_cmu),
						GFP_KERNEL);
			cmu_ch[index]->name = clk_ch_name;
			cmu_ch[index]->cmu_id = index;
			cmu_ch[index]->parent = cmu->cmu_id;
			cmu_ch[index]->is_clk_ch = 1;
			cmu_ch[index]->hw.init = &init_ch[i];

			clk_ch->clks[index] = clk_register(NULL,
							&cmu_ch[index]->hw);

			if (IS_ERR(clk_ch->clks[index])) {
				/* Error */
				pr_err("%s: could not register clk %s\n",
					__func__, clk_ch_name);
			}
			/* Register the clock for lookup */
			rc = clk_register_clkdev(clk_ch->clks[index],
						clk_ch_name, NULL);
			if (rc != 0) {
				/* Error */
				pr_err("%s: could not register lookup clk %s\n",
				__func__, clk_ch_name);
			}
			/* FIXME We probably SHOULDN'T enable it here */
			clk_prepare_enable(clk_ch->clks[index]);
			i++;
		}
		return of_clk_add_provider(pdev->dev.of_node,
					of_clk_src_onecell_get, clk_ch);
	}

	return of_clk_add_provider(pdev->dev.of_node,
				of_clk_src_simple_get, clk);
}

static int baikal_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id baikal_clk_of_match[] =  {
	{.compatible = "baikal,cmu"},
	{ /* sentinel value */ }
};

static struct platform_driver clk_ca57_cmu_driver = {
	.probe  = baikal_clk_probe,
	.remove = baikal_clk_remove,
	.driver = {
		.name   = "baikal-ca57_cmu",
		.of_match_table = baikal_clk_of_match,
	},
};

module_platform_driver(clk_ca57_cmu_driver);

MODULE_DESCRIPTION("Clkout driver for the Baikal-M");
MODULE_AUTHOR("Ekaterina Skachko <Ekaterina.Skachko@baikalelectronics.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:baikal-cmu");
