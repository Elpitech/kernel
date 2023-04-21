// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021-2022 Baikal Electronics, JSC
 * Author: Ekaterina Skachko <ekaterina.skachko@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define BAIKAL_SMC_CLK			(0x82000000 + 0x400)
#define BAIKAL_SMC_CLK_ROUND		(BAIKAL_SMC_CLK + 0)
#define BAIKAL_SMC_CLK_SET		(BAIKAL_SMC_CLK + 1)
#define BAIKAL_SMC_CLK_GET		(BAIKAL_SMC_CLK + 2)
#define BAIKAL_SMC_CLK_ENABLE		(BAIKAL_SMC_CLK + 3)
#define BAIKAL_SMC_CLK_DISABLE		(BAIKAL_SMC_CLK + 4)
#define BAIKAL_SMC_CLK_IS_ENABLED	(BAIKAL_SMC_CLK + 5)

struct baikal_clk {
	struct clk_hw hw;
	uint32_t base;
};
#define to_baikal_clk(_hw) container_of(_hw, struct baikal_clk, hw)

static int baikal_clk_enable(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_ENABLE, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static void baikal_clk_disable(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_DISABLE, clk->base, 0, 0, 0, 0, 0, 0, &res);
}

static int baikal_clk_is_enabled(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_IS_ENABLED, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static unsigned long baikal_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_GET, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static int baikal_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_SET, clk->base, rate, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static long baikal_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_ROUND, clk->base, rate, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static const struct clk_ops baikal_clk_ops = {
	.enable      = baikal_clk_enable,
	.disable     = baikal_clk_disable,
	.is_enabled  = baikal_clk_is_enabled,
	.recalc_rate = baikal_clk_recalc_rate,
	.set_rate    = baikal_clk_set_rate,
	.round_rate  = baikal_clk_round_rate
};

static int baikal_clk_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_init_data init;
	struct baikal_clk *cmu;
	struct clk_onecell_data *clk_data;
	const char *clk_name;
	struct property *prop;
	const __be32 *p;
	int clk_index;
	int clk_index_max;
	int clk_index_cnt;
	int clk_name_cnt;
	int clk_cnt;
	int i;
	uint32_t base;
	struct clk *clk;
	int ret;
	int multi;

	// base
	ret = of_property_read_u32(node, "reg", &base);
	if (ret) {
		base = 0;
	}

	// cnt
	clk_index_cnt = of_property_count_u32_elems(node, "clock-indices");
	clk_name_cnt = of_property_count_strings (node, "clock-output-names");
	clk_cnt = clk_index_cnt > clk_name_cnt ? clk_index_cnt : clk_name_cnt;
	if (clk_cnt < 1) {
		clk_cnt = 1;
	}
	multi = clk_cnt > 1;

	if (multi) {
		clk_index_max = clk_cnt - 1;
		of_property_for_each_u32(node, "clock-indices", prop, p, clk_index) {
			if (clk_index_max < clk_index) {
				clk_index_max = clk_index;
			}
		}
		clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
		clk_data->clks = kcalloc(clk_index_max + 1, sizeof(struct clk*), GFP_KERNEL);
		clk_data->clk_num = clk_index_max + 1;
	}

	for (i = 0; i < clk_cnt; i++) {

		ret = of_property_read_u32_index (node, "clock-indices", i, &clk_index);
		if (ret) {
			clk_index = i;
		}
		ret = of_property_read_string_index (node, "clock-output-names", i, &clk_name);
		if (ret) {
			if (multi)
				init.name = kasprintf(GFP_KERNEL, "%s.%d", node->name, clk_index);
			else
				init.name = kasprintf(GFP_KERNEL, "%s",    node->name);
		} else {
				init.name = kasprintf(GFP_KERNEL, "%s.%s", node->name, clk_name);
		}

		init.ops = &baikal_clk_ops;
		init.flags = CLK_IGNORE_UNUSED;
		init.parent_names = NULL;
		init.num_parents = 0;

		cmu = kmalloc(sizeof(*cmu), GFP_KERNEL);
		cmu->base = base + 0x10 * clk_index;
		cmu->hw.init = &init;

		clk = clk_register(NULL, &cmu->hw);
		if (!IS_ERR(clk)) {
			clk_register_clkdev(clk, init.name, NULL);
			if (multi) {
				clk_data->clks[clk_index] = clk;
			}
		}
	}

	// add
	if (multi) {
		ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get, clk_data);
	} else {
		ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, clk);
	}
	return ret;
}

static int baikal_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

#ifdef CONFIG_ACPI
const char *baikal_acpi_ref_clk_str[] = { "baikal_ref_clk" };

static struct clk *baikal_acpi_ref_clk;

struct baikal_acpi_clk_data {
	struct clk *cmu_clk;
	struct clk_lookup *cmu_clk_l;
	struct clk **clk;
	struct clk_lookup **clk_l;
	unsigned int clk_num;
};

static int baikal_acpi_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct acpi_device *ref_dev, *adev = to_acpi_device_node(pdev->dev.fwnode);
	struct clk_init_data init, *init_ch;
	struct baikal_clk *cmu, *cmu_ch;
	struct baikal_acpi_clk_data *clk_data = NULL;
	union acpi_object *package, *element;
	acpi_status status;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	int size, i, index, ret = 0;
	char *str, *str2;
	const char *cmu_name;

	cmu = devm_kzalloc(dev, sizeof(*cmu), GFP_KERNEL);
	if (!cmu) {
		return -ENOMEM;
	}

	status = acpi_evaluate_object_typed(adev->handle, "PROP", NULL, &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get PROP data\n");
		return -ENODEV;
	}

	package = buffer.pointer;
	if (package->package.count != 2) {
		dev_err(dev, "invalid PROP data\n");
		ret = -EINVAL;
		goto ret;
	}

	element = &(package->package.elements[0]);
	if (element->type != ACPI_TYPE_INTEGER) {
		dev_err(dev, "failed to get CMU id\n");
		ret = -EINVAL;
		goto ret;
	}

	cmu->base = element->integer.value;

	element = &(package->package.elements[1]);
	if (element->type != ACPI_TYPE_STRING) {
		dev_err(dev, "failed to get CMU clock name\n");
		ret = -EINVAL;
		goto ret;
	}

	str = devm_kzalloc(dev, element->string.length + 1, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto ret;
	}

	memcpy(str, element->string.pointer, element->string.length);
	cmu_name = str;

	acpi_os_free(buffer.pointer);
	buffer.length = ACPI_ALLOCATE_BUFFER;
	buffer.pointer = NULL;

	init.parent_names = baikal_acpi_ref_clk_str;
	init.num_parents = 1;
	init.name = cmu_name;
	init.ops = &baikal_clk_ops;
	init.flags = CLK_IGNORE_UNUSED;

	cmu->hw.init = &init;

	clk_data = devm_kzalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data) {
		return -ENOMEM;
	}

	clk_data->cmu_clk = clk_register(NULL, &cmu->hw);
	if (IS_ERR(clk_data->cmu_clk)) {
		dev_err(dev, "failed to register CMU clock\n");
		return PTR_ERR(clk_data->cmu_clk);
	}

	clk_data->cmu_clk_l = clkdev_create(clk_data->cmu_clk, cmu_name, NULL);
	if (!clk_data->cmu_clk_l) {
		dev_err(dev, "failed to register CMU clock lookup\n");
		clk_unregister(clk_data->cmu_clk);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, clk_data);

	status = acpi_evaluate_object_typed(adev->handle, "CLKS", NULL, &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		buffer.pointer = NULL;
		goto ret;
	}

	package = buffer.pointer;
	if (!package->package.count || package->package.count % 4) {
		dev_err(dev, "invalid CLKS data\n");
		ret = -EINVAL;
		goto ret;
	}

	clk_data->clk_num = package->package.count >> 2;
	clk_data->clk = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk *), GFP_KERNEL);
	if (!clk_data->clk) {
		ret = -ENOMEM;
		goto ret;
	}

	clk_data->clk_l = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk_lookup *), GFP_KERNEL);
	if (!clk_data->clk_l) {
		ret = -ENOMEM;
		goto ret;
	}

	init_ch = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk_init_data), GFP_KERNEL);
	if (!init_ch) {
		ret = -ENOMEM;
		goto ret;
	}

	cmu_ch = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct baikal_clk), GFP_KERNEL);
	if (!cmu_ch) {
		ret = -ENOMEM;
		goto ret;
	}

	for (i = 0; i < clk_data->clk_num; ++i) {
		ref_dev = NULL;
		size = 0;

		element = &(package->package.elements[4 * i]);
		if (element->type == ACPI_TYPE_LOCAL_REFERENCE && element->reference.handle)
			ref_dev = acpi_fetch_acpi_dev(element->reference.handle);

		element = &(package->package.elements[4 * i + 1]);
		if (element->type == ACPI_TYPE_STRING) {
			if (ref_dev)
				size = strlen(dev_name(&ref_dev->dev)) + 1;

			str = devm_kzalloc(dev, size + element->string.length + 1, GFP_KERNEL);
			if (str) {
				if (ref_dev) {
					memcpy(str, dev_name(&ref_dev->dev), size - 1);
					str[size - 1] = '_';
					memcpy(str + size, element->string.pointer, element->string.length);
				} else
					memcpy(str, element->string.pointer, element->string.length);
			}
	        } else {
			dev_err(dev, "failed to process clock device name #%i\n", i);
			continue;
		}

		element = &(package->package.elements[4 * i + 2]);
		if (element->type == ACPI_TYPE_INTEGER) {
			index = element->integer.value;
		} else {
			dev_err(dev, "failed to process clock device id #%i\n", i);
			continue;
		}

		element = &(package->package.elements[4 * i + 3]);
		if (element->type == ACPI_TYPE_STRING) {
			str2 = devm_kzalloc(dev, element->string.length + 1, GFP_KERNEL);
			if (str2)
				memcpy(str2, element->string.pointer, element->string.length);
	        } else
			str2 = NULL;

		init_ch[i].parent_names = &cmu_name;
		init_ch[i].num_parents = 1;
		init_ch[i].name = str;
		init_ch[i].ops = &baikal_clk_ops;
		init_ch[i].flags = CLK_IGNORE_UNUSED;

		cmu_ch[i].base = cmu->base + 0x20 + 0x10 * index;
		cmu_ch[i].hw.init = &init_ch[i];

		clk_data->clk[i] = clk_register(ref_dev ? &ref_dev->dev : NULL, &cmu_ch[i].hw);
		if (IS_ERR(clk_data->clk[i])) {
			dev_err(dev, "failed to register CMU channel clock #%i\n", i);
			clk_data->clk[i] = NULL;
			continue;
		}

		if (ref_dev)
			clk_data->clk_l[i] = clkdev_create(clk_data->clk[i], str2, "%s", dev_name(&ref_dev->dev));
		else
			clk_data->clk_l[i] = clkdev_create(clk_data->clk[i], str2, NULL);
		if (!clk_data->clk_l[i]) {
			dev_err(dev, "failed to register CMU channel clock lookup #%i\n", i);
			clk_unregister(clk_data->clk[i]);
			clk_data->clk[i] = NULL;
			continue;
		}
	}

	clk_data = NULL;

ret:
	if (buffer.pointer)
		acpi_os_free(buffer.pointer);
	if (clk_data) {
		clk_disable_unprepare(clk_data->cmu_clk);
		clkdev_drop(clk_data->cmu_clk_l);
		clk_unregister(clk_data->cmu_clk);
	}
	return ret;
}

static int baikal_acpi_clk_remove(struct platform_device *pdev)
{
	struct baikal_acpi_clk_data *clk_data = platform_get_drvdata(pdev);
	int i;

	if (clk_data) {
		clk_disable_unprepare(clk_data->cmu_clk);
		clkdev_drop(clk_data->cmu_clk_l);
		clk_unregister(clk_data->cmu_clk);

		for (i = 0; i < clk_data->clk_num; ++i) {
			if (clk_data->clk_l[i])
				clkdev_drop(clk_data->clk_l[i]);
			if (clk_data->clk[i])
				clk_unregister(clk_data->clk[i]);
		}
	}

	return 0;
}

static const struct acpi_device_id baikal_acpi_clk_device_ids[] = {
	{ "BKLE0001" },
	{ }
};

static struct platform_driver baikal_acpi_clk_driver = {
	.probe		= baikal_acpi_clk_probe,
	.remove		= baikal_acpi_clk_remove,
	.driver		= {
		.name	= "bs1000-cmu-acpi",
		.acpi_match_table = ACPI_PTR(baikal_acpi_clk_device_ids)
	}
};

static int __init baikal_acpi_clk_driver_init(void)
{
	if (!acpi_disabled) {
		struct clk_lookup *baikal_acpi_ref_clk_lookup;

		baikal_acpi_ref_clk = clk_register_fixed_rate(NULL, baikal_acpi_ref_clk_str[0], NULL, 0, 25000000);
		if (IS_ERR(baikal_acpi_ref_clk)) {
			pr_err("%s: failed to register reference clock\n", __func__);
			return PTR_ERR(baikal_acpi_ref_clk);
		}

		baikal_acpi_ref_clk_lookup = clkdev_create(baikal_acpi_ref_clk, NULL, "%s", baikal_acpi_ref_clk_str[0]);
		if (!baikal_acpi_ref_clk_lookup) {
			clk_unregister_fixed_rate(baikal_acpi_ref_clk);
			pr_err("%s: failed to register reference clock lookup\n", __func__);
			return -ENOMEM;
		}

		clk_prepare_enable(baikal_acpi_ref_clk);

		return platform_driver_register(&baikal_acpi_clk_driver);
	}

	return 0;
}

device_initcall(baikal_acpi_clk_driver_init);
#endif

static const struct of_device_id baikal_clk_of_match[] = {
	{ .compatible = "baikal,bs1000-cmu" },
	{ /* sentinel */ }
};

static struct platform_driver bs1000_cmu_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "bs1000-cmu",
		.of_match_table = baikal_clk_of_match
	}
};
module_platform_driver(bs1000_cmu_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 clock driver");
MODULE_AUTHOR("Ekaterina Skachko <ekaterina.skachko@baikalelectronics.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bs1000-cmu");
