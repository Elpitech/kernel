// SPDX-License-Identifier: GPL-2.0
/*
 * Multiplexer driver for Baikal-S SoC low speed peripheral devices
 *
 * Copyright (C) 2022 Baikal Electronics, JSC
 *
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/of_platform.h>
#include <linux/mux/consumer.h>
#include <linux/mux/driver.h>
#include <linux/platform_device.h>

#define BAIKAL_LSP_MUX_SMC_ID		0x82000600
#define BAIKAL_LSP_MUX_COUNT		3
#define BAIKAL_LSP_MUX_STATE_COUNT	2
#define BAIKAL_LSP_MUX_AS_IS		2

static u8 uart_smbus = BAIKAL_LSP_MUX_AS_IS;
module_param(uart_smbus, byte, 0);
MODULE_PARM_DESC(uart_smbus, "UART/SMBus/SMBus. 0=Disable, 1=Enable");

static u8 qspi = BAIKAL_LSP_MUX_AS_IS;
module_param(qspi, byte, 0);
MODULE_PARM_DESC(qspi, "QSPI. 0=Disable, 1=Enable");

static u8 espi = BAIKAL_LSP_MUX_AS_IS;
module_param(espi, byte, 0);
MODULE_PARM_DESC(espi, "eSPI. 0=Disable, 1=Enable");

static const u8 * const baikal_lsp_mux_init_value[3] = {
	&uart_smbus,
	&qspi,
	&espi
};

struct baikal_lsp_mux_priv {
	struct fwnode_handle **fwnode[BAIKAL_LSP_MUX_STATE_COUNT];
	struct platform_device **plat_dev[BAIKAL_LSP_MUX_STATE_COUNT];
	u8 count[BAIKAL_LSP_MUX_STATE_COUNT];
	bool acpi_node_enabled[BAIKAL_LSP_MUX_STATE_COUNT];
	struct mutex lock;
	struct mux_control *mux;
	struct platform_device *pdev;
};

struct baikal_mux_chip_priv {
	unsigned int state;
	struct mutex chip_lock;
	struct baikal_lsp_mux_priv channel_priv[BAIKAL_LSP_MUX_COUNT];
};

static char *baikal_lsp_mux_property(unsigned int state)
{
	if (acpi_disabled)
		return state ? "mux-state1" : "mux-state0";
	else
		return state ? "DEV1" : "DEV0";
}

static struct platform_device *
baikal_lsp_mux_channel_create_device(struct fwnode_handle *fwnode,
				     struct device *parent)
{
	if (acpi_disabled) {
		return of_platform_device_create(to_of_node(fwnode), NULL,
						 parent);
	} else {
		return acpi_create_platform_device(to_acpi_device_node(fwnode),
						   NULL);
	}
}

static void baikal_lsp_mux_channel_remove_device(struct platform_device *pdev)
{
	if (acpi_disabled) {
		of_platform_device_destroy(&pdev->dev, NULL);
	} else {
		platform_device_unregister(pdev);
	}
}

static void baikal_lsp_mux_channel_remove_devices(struct baikal_lsp_mux_priv *priv,
						  unsigned int state)
{
	struct platform_device **pdev;
	unsigned int i;

	if (state >= BAIKAL_LSP_MUX_STATE_COUNT)
		return;

	for (i = 0; i < priv->count[state]; ++i) {
		pdev = &priv->plat_dev[state][i];

		if (*pdev) {
			baikal_lsp_mux_channel_remove_device(*pdev);
			*pdev = NULL;
		}
	}
}

static int baikal_lsp_mux_channel_switch_devices(struct baikal_lsp_mux_priv *priv,
						 unsigned int state)
{
	struct device *parent = priv->mux->chip->dev.parent;
	struct platform_device **pdev;
	struct fwnode_handle **fwnode;
	int i, ret = 0;

	state = !!state;

	mutex_lock(&priv->lock);

	if (priv->mux->cached_state == state)
		goto err_unlock;

	if (priv->mux->cached_state == MUX_IDLE_AS_IS) {
		for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i)
			baikal_lsp_mux_channel_remove_devices(priv, i);
	} else {
		baikal_lsp_mux_channel_remove_devices(priv, !state);
	}

	ret = mux_control_select(priv->mux, state);
	if (ret)
		goto err_unlock;

	ret = mux_control_deselect(priv->mux);
	if (ret)
		goto err_unlock;

	if (priv->mux->cached_state == MUX_IDLE_AS_IS) {
		baikal_lsp_mux_channel_remove_devices(priv, state);
		ret = -EPROTO;
		goto err_unlock;
	}

	if (!acpi_disabled && !priv->acpi_node_enabled[state]) {
		union acpi_object val = {
			.type = ACPI_TYPE_INTEGER,
			.integer.type = ACPI_TYPE_INTEGER,
			.integer.value = state
		};
		struct acpi_object_list args = {
			.count = 1,
			.pointer = &val
		};
		struct acpi_device *adev =
			to_acpi_device_node(priv->pdev->dev.fwnode);
		struct device *ref_dev;
		u64 is_enabled;
		acpi_status status;

		status = acpi_evaluate_integer(adev->handle,
					       state ? "STA1" : "STA0",
					       NULL,
					       &is_enabled);
		if (ACPI_FAILURE(status)) {
			dev_err(&adev->dev, "failed to get state for %s\n",
				baikal_lsp_mux_property(i));
			ret = -ENODEV;
			goto err_unlock;
		}

		if (is_enabled) {
			priv->acpi_node_enabled[state] = true;
			goto nodes_exist;
		}

		status = acpi_evaluate_object(adev->handle, "INIT", &args,
					      NULL);
		if (ACPI_FAILURE(status)) {
			dev_err(&adev->dev, "failed to enable %s devices\n",
				baikal_lsp_mux_property(i));
			ret = -ENODEV;
			goto err_unlock;
		}

		priv->acpi_node_enabled[state] = true;

		for (i = 0; i < priv->count[state]; ++i) {
			fwnode = &priv->fwnode[state][i];
			pdev = &priv->plat_dev[state][i];

			if (!*fwnode)
				continue;

			adev = to_acpi_device_node(*fwnode);
			acpi_scan_lock_acquire();
			if (adev)
				acpi_bus_scan(adev->handle);
			acpi_scan_lock_release();

			ref_dev = bus_find_device_by_fwnode(&platform_bus_type,
							    *fwnode);
			if (ref_dev)
				*pdev = to_platform_device(ref_dev);
		}

		goto err_unlock;
	}

nodes_exist:
	for (i = 0; i < priv->count[state]; ++i) {
		fwnode = &priv->fwnode[state][i];
		pdev = &priv->plat_dev[state][i];

		if (!*fwnode)
			continue;

		if (!*pdev) {
			*pdev = baikal_lsp_mux_channel_create_device(*fwnode,
								     parent);
			if (IS_ERR_OR_NULL(*pdev)) {
				*pdev = NULL;
			}
		} else {
			baikal_lsp_mux_channel_remove_device(*pdev);
		}
	}

err_unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static ssize_t baikal_lsp_mux_channel_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct baikal_lsp_mux_priv *priv = dev_get_drvdata(dev);
	unsigned int n = 0;
	int ret;

	ret = kstrtouint(buf, 10, &n);
	if (ret)
		return ret;

	ret = baikal_lsp_mux_channel_switch_devices(priv, n);

	ret = ret ? ret : count;

	return ret;
}

static ssize_t baikal_lsp_mux_channel_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct baikal_lsp_mux_priv *priv = dev_get_drvdata(dev);
	return sprintf(buf, "%i\n", priv->mux->cached_state);
}

static DEVICE_ATTR(enabled, S_IWUSR | S_IRUGO, baikal_lsp_mux_channel_show,
		   baikal_lsp_mux_channel_store);

static struct attribute *baikal_lsp_mux_channel_attrs[] = {
	&dev_attr_enabled.attr,
	NULL
};

static const struct attribute_group baikal_lsp_mux_channel_attr_group = {
	.attrs = baikal_lsp_mux_channel_attrs
};

static struct mux_control *
baikal_lsp_mux_channel_get_mux_acpi(struct acpi_device *adev)
{
	struct device *dev = &adev->dev, *mux_chip_dev;
	struct acpi_device *mux;
	struct mux_chip *mux_chip;
	union acpi_object *package = NULL;
	union acpi_object *element = NULL;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status = AE_OK;
	unsigned int id;
	int ret;

	status = acpi_evaluate_object_typed(adev->handle, "MUX", NULL,
					    &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get MUX data\n");
		return ERR_PTR(-ENODEV);
	}

	package = buffer.pointer;
	if (package->package.count != 2) {
		dev_err(dev, "invalid MUX data\n");
		ret = -EINVAL;
		goto err;
	}

	element = &(package->package.elements[0]);
	if (element->type != ACPI_TYPE_LOCAL_REFERENCE ||
	    !element->reference.handle) {
		dev_err(dev, "invalid MUX reference\n");
		ret = -EINVAL;
		goto err;
	}

	ret = acpi_bus_get_device(element->reference.handle, &mux);
	if (ret) {
		dev_err(dev, "failed to process MUX reference\n");
		ret = -ENODEV;
		goto err;
	}

	element = &(package->package.elements[1]);
	if (element->type != ACPI_TYPE_INTEGER) {
		dev_err(dev, "failed to get MUX index\n");
		ret = -EINVAL;
		goto err;
	}
	id = element->integer.value;

	mux_chip_dev = bus_find_device_by_fwnode(&platform_bus_type,
						 acpi_fwnode_handle(mux));
	if (!mux_chip_dev) {
		dev_err(dev, "failed to get mux chip device\n");
		ret = -ENODEV;
		goto err;
	}

	mux_chip = dev_get_drvdata(mux_chip_dev);
	if (!mux_chip) {
		dev_err(dev, "failed to get mux chip\n");
		ret = -ENODEV;
		goto err;
	}

	if (id >= mux_chip->controllers) {
		dev_err(dev, "bad mux controller %u\n", id);
		ret = -EINVAL;
		goto err;
	}

	ret = 0;

err:
	acpi_os_free(buffer.pointer);
	if (ret)
		return ERR_PTR(ret);

	return &mux_chip->mux[id];
}

static struct baikal_lsp_mux_priv *
baikal_lsp_mux_channel_get_of_data(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *mux_dev;
	struct baikal_lsp_mux_priv *priv;
	int count[BAIKAL_LSP_MUX_STATE_COUNT];
	int i, j;

	if (!np)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i) {
		count[i] = of_count_phandle_with_args(np,
					      baikal_lsp_mux_property(i), NULL);
		if (count[i] < 1) {
			dev_err(dev, "invalid number of phandles in '%s' property\n",
				baikal_lsp_mux_property(i));
			return ERR_PTR(-EINVAL);
		}
	}

	priv = devm_kzalloc(dev, sizeof(*priv) +
				 2 * (count[0] + count[1]) * sizeof(void *),
			    GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->count[0] = count[0];
	priv->count[1] = count[1];
	priv->fwnode[0] = (struct fwnode_handle **)(priv + 1);
	priv->fwnode[1] = priv->fwnode[0] + priv->count[0];
	priv->plat_dev[0] = (struct platform_device **)(priv->fwnode[1] +
			    priv->count[1]);
	priv->plat_dev[1] = priv->plat_dev[0] + priv->count[0];

	priv->mux = mux_control_get(dev, NULL);
	if (IS_ERR(priv->mux))
		return ERR_CAST(priv->mux);

	for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i) {
		for (j = 0; j < priv->count[i]; ++j) {
			mux_dev = of_parse_phandle(np,
						   baikal_lsp_mux_property(i),
						   j);
			if (!mux_dev) {
				dev_err(dev, "invalid phandle%i in '%s' property\n",
					j, baikal_lsp_mux_property(i));
				return ERR_PTR(-EINVAL);
			}

			priv->fwnode[i][j] = of_fwnode_handle(mux_dev);
		}
	}

	return priv;
}

static struct baikal_lsp_mux_priv *
baikal_lsp_mux_channel_get_acpi_data(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct acpi_device *adev = ACPI_COMPANION(dev);
	struct acpi_device *mux_dev;
	struct baikal_lsp_mux_priv *priv;
	int count[BAIKAL_LSP_MUX_STATE_COUNT];
	struct acpi_buffer buffer[BAIKAL_LSP_MUX_STATE_COUNT];
	union acpi_object *package = NULL;
	union acpi_object *element = NULL;
	acpi_status status = AE_OK;
	int i, j, ret;

	if (!adev)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i) {
		buffer[i].length = ACPI_ALLOCATE_BUFFER;
		buffer[i].pointer = NULL;
		status = acpi_evaluate_object_typed(adev->handle,
						    baikal_lsp_mux_property(i),
						    NULL,
						    &buffer[i],
						    ACPI_TYPE_PACKAGE);
		if (ACPI_FAILURE(status)) {
			dev_err(dev, "failed to get %s data\n",
				baikal_lsp_mux_property(i));
			return ERR_PTR(-ENODEV);
		}

		package = buffer[i].pointer;
		count[i] = package->package.count;
		if (count[i] < 1) {
			dev_err(dev, "invalid number of references in %s\n",
				baikal_lsp_mux_property(i));
			ret = -EINVAL;
			goto err;
		}
	}

	priv = devm_kzalloc(dev, sizeof(*priv) +
				 2 * (count[0] + count[1]) * sizeof(void *),
			    GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err;
	}

	priv->count[0] = count[0];
	priv->count[1] = count[1];
	priv->fwnode[0] = (struct fwnode_handle **)(priv + 1);
	priv->fwnode[1] = priv->fwnode[0] + priv->count[0];
	priv->plat_dev[0] = (struct platform_device **)(priv->fwnode[1] +
			    priv->count[1]);
	priv->plat_dev[1] = priv->plat_dev[0] + priv->count[0];

	priv->mux = baikal_lsp_mux_channel_get_mux_acpi(adev);
	if (IS_ERR(priv->mux)) {
		ret = PTR_ERR(priv->mux);
		goto err;
	}

	for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i) {
		package = buffer[i].pointer;
		for (j = 0; j < priv->count[i]; ++j) {
			element = &(package->package.elements[j]);

			if (element->type != ACPI_TYPE_LOCAL_REFERENCE ||
			    !element->reference.handle) {
				dev_err(dev, "invalid reference%i in %s\n", j,
					baikal_lsp_mux_property(i));
				ret = -EINVAL;
				goto err;
			}

			ret = acpi_bus_get_device(element->reference.handle,
						  &mux_dev);
			if (ret) {
				dev_err(dev, "failed to process reference%i in %s\n",
					j, baikal_lsp_mux_property(i));
				ret = -EINVAL;
				goto err;
			}

			priv->fwnode[i][j] = acpi_fwnode_handle(mux_dev);
		}
	}

	ret = 0;

err:
	for (i = 0; i < BAIKAL_LSP_MUX_STATE_COUNT; ++i)
		acpi_os_free(buffer[i].pointer);
	if (ret)
		return ERR_PTR(ret);

	return priv;
}

static int baikal_lsp_mux_channel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_lsp_mux_priv *priv;
	unsigned int id;
	int ret;

	if (acpi_disabled)
		priv = baikal_lsp_mux_channel_get_of_data(pdev);
	else
		priv = baikal_lsp_mux_channel_get_acpi_data(pdev);

	if (IS_ERR(priv)) {
		dev_err(dev, "failed to get mux data\n");
		return PTR_ERR(priv);
	}

	priv->pdev = pdev;
	mutex_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	id = mux_control_get_index(priv->mux);
	ret = baikal_lsp_mux_channel_switch_devices(priv,
						*baikal_lsp_mux_init_value[id]);
	if (ret) {
		dev_err(dev, "failed to initialize mux devices: %i\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &baikal_lsp_mux_channel_attr_group);
	if (ret) {
		dev_err(dev, "failed to create sysfs group: %i\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id baikal_lsp_mux_channel_of_match[] = {
	{ .compatible = "baikal,bs1000-lsp-mux-channel" },
	{}
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id baikal_lsp_mux_channel_acpi_match[] = {
	{ "BKLE0003" },
	{}
};

MODULE_DEVICE_TABLE(acpi, baikal_lsp_mux_channel_acpi_match);
#endif

static struct platform_driver baikal_lsp_mux_channel_driver = {
	.driver = {
		.name = "baikal-lsp-mux-channel",
		.of_match_table = baikal_lsp_mux_channel_of_match,
		.acpi_match_table = ACPI_PTR(baikal_lsp_mux_channel_acpi_match),
		.suppress_bind_attrs = true
	},
	.probe = baikal_lsp_mux_channel_probe
};

static int baikal_lsp_mux_set(struct mux_control *mux, int state)
{
	struct arm_smccc_res res;
	struct baikal_mux_chip_priv *chip_priv = mux_chip_priv(mux->chip);
	unsigned int new_state;

	mutex_lock(&chip_priv->chip_lock);
	new_state = chip_priv->state;
	if (state)
		new_state |= 1 << mux_control_get_index(mux);
	else
		new_state &= ~(1 << mux_control_get_index(mux));
	arm_smccc_smc(BAIKAL_LSP_MUX_SMC_ID,
		      new_state,
		      0, 0, 0, 0, 0, 0, &res);
	chip_priv->state = new_state;
	mutex_unlock(&chip_priv->chip_lock);

	return res.a0;
}

static const struct mux_control_ops baikal_lsp_mux_ops = {
	.set = baikal_lsp_mux_set
};

static int baikal_lsp_mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_chip *mux_chip;
	struct baikal_mux_chip_priv *chip_priv;
	int i, ret;

	mux_chip = devm_mux_chip_alloc(dev, BAIKAL_LSP_MUX_COUNT,
				       sizeof(struct baikal_mux_chip_priv));
	if (IS_ERR(mux_chip)) {
		dev_err(dev, "failed to allocate mux chip\n");
		return PTR_ERR(mux_chip);
	}

	mux_chip->ops = &baikal_lsp_mux_ops;
	chip_priv = mux_chip_priv(mux_chip);
	mutex_init(&chip_priv->chip_lock);

	if (uart_smbus == BAIKAL_LSP_MUX_AS_IS) {
		uart_smbus = !!device_property_present(dev, "uart-smbus-enabled");
	}
	baikal_lsp_mux_set(&mux_chip->mux[0], uart_smbus);

	if (qspi == BAIKAL_LSP_MUX_AS_IS) {
		qspi = !!device_property_present(dev, "qspi-enabled");
	}
	baikal_lsp_mux_set(&mux_chip->mux[1], qspi);

	if (espi == BAIKAL_LSP_MUX_AS_IS) {
		espi = !!device_property_present(dev, "espi-enabled");
	}
	baikal_lsp_mux_set(&mux_chip->mux[2], espi);

	for (i = 0; i < mux_chip->controllers; ++i)
		mux_chip->mux[i].states = BAIKAL_LSP_MUX_STATE_COUNT;

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret) {
		dev_err(dev, "failed to register mux chip\n");
		return ret;
	}

	platform_set_drvdata(pdev, mux_chip);

	return 0;
}

static const struct of_device_id baikal_lsp_mux_of_match[] = {
	{ .compatible = "baikal,bs1000-lsp-mux" },
	{}
};
MODULE_DEVICE_TABLE(of, baikal_lsp_mux_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id baikal_lsp_mux_acpi_match[] = {
	{ "BKLE0002" },
	{}
};

MODULE_DEVICE_TABLE(acpi, baikal_lsp_mux_acpi_match);
#endif

static struct platform_driver baikal_lsp_mux_driver = {
	.driver = {
		.name = "baikal-lsp-mux",
		.of_match_table = baikal_lsp_mux_of_match,
		.acpi_match_table = ACPI_PTR(baikal_lsp_mux_acpi_match),
		.suppress_bind_attrs = true
	},
	.probe = baikal_lsp_mux_probe
};

static struct platform_driver * const baikal_lsp_mux_drivers[] = {
	&baikal_lsp_mux_driver,
	&baikal_lsp_mux_channel_driver
};

static int __init baikal_lsp_mux_init(void)
{
	return platform_register_drivers(baikal_lsp_mux_drivers,
					 ARRAY_SIZE(baikal_lsp_mux_drivers));
}
module_init(baikal_lsp_mux_init);

static void __exit baikal_lsp_mux_exit(void)
{
	platform_unregister_drivers(baikal_lsp_mux_drivers,
				    ARRAY_SIZE(baikal_lsp_mux_drivers));
}
module_exit(baikal_lsp_mux_exit);

MODULE_AUTHOR("Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal-S SoC low speed peripheral mux driver");
MODULE_LICENSE("GPL v2");
