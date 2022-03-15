// SPDX-License-Identifier: GPL-2.0
/*
 * SMBus controller driver for Baikal SoC
 *
 * Copyright (C) 2019-2021 Baikal Electronics, JSC
 * Authors: Georgy Vlasov <georgy.vlasov@baikalelectronics.ru>
 *	    Mikhail Ivanov <michail.ivanov@baikalelectronics.ru>
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* Register definitions */
#define BE_SMBUS_CR1		0x00
#define BE_CR1_IRT		BIT(0)
#define BE_CR1_TRS		BIT(1)
#define BE_CR1_IEB		BIT(3)

#define BE_SMBUS_CR2		0x04
#define BE_CR2_FTE		BIT(1)

#define BE_SMBUS_FBCR1		0x08
#define BE_SMBUS_FIFO		0x0c
#define BE_SMBUS_SCD1		0x10

#define BE_SMBUS_SCD2		0x14
#define BE_SCD2_MASK		0x03
#define BE_SCD2_SHT		BIT(7)

#define BE_SMBUS_ADR1		0x18

#define BE_SMBUS_ISR1		0x20
#define BE_ISR1_MASK		0x7f
#define BE_ISR1_FER		BIT(2)
#define BE_ISR1_RNK		BIT(3)
#define BE_ISR1_ALD		BIT(4)
#define BE_ISR1_TCS		BIT(6)

#define BE_SMBUS_IMR1		0x24
#define BE_SMBUS_FBCR2		0x2c
#define BE_SMBUS_IMR2		0x48

#define BE_SMBUS_FIFO_SIZE	16

struct baikal_smbus_dev {
	struct device		*dev;
	void __iomem		*base;
	struct clk		*smbus_clk;
	struct i2c_adapter	adapter;
	u32			bus_clk_rate;
	u64			smbus_clk_rate;
};

static int baikal_smbus_init(struct baikal_smbus_dev *smbus)
{
	u32 divider = (smbus->smbus_clk_rate == 0 ?
			clk_get_rate(smbus->smbus_clk) :
			smbus->smbus_clk_rate) / smbus->bus_clk_rate - 1;

	writel(BE_CR1_IRT, smbus->base + BE_SMBUS_CR1);
	writel(0, smbus->base + BE_SMBUS_CR1);
	writel(0, smbus->base + BE_SMBUS_CR2);
	writel(0, smbus->base + BE_SMBUS_FBCR2);
	writel(0, smbus->base + BE_SMBUS_IMR1);
	writel(0, smbus->base + BE_SMBUS_IMR2);
	writel(divider & 0xff, smbus->base + BE_SMBUS_SCD1);
	writel(BE_SCD2_SHT | ((divider >> 8) & BE_SCD2_MASK),
		smbus->base + BE_SMBUS_SCD2);

	return 0;
}

static int baikal_smbus_xfer(struct i2c_adapter *adap, u16 addr,
			unsigned short flags, char read_write,
			u8 command, int size, union i2c_smbus_data *data)
{
	struct baikal_smbus_dev *const smbus = adap->dev.driver_data;
	int ret = -EINVAL;

	switch (size) {
	case I2C_SMBUS_BYTE:
		writel(BE_CR1_IRT, smbus->base + BE_SMBUS_CR1);
		writel(0, smbus->base + BE_SMBUS_CR1);
		writel(addr, smbus->base + BE_SMBUS_ADR1);
		writel(1, smbus->base + BE_SMBUS_FBCR1);

		if (read_write == I2C_SMBUS_WRITE) {
			writel(BE_CR1_TRS | BE_CR1_IEB,
				smbus->base + BE_SMBUS_CR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
		} else {
			writel(BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);
		}

		writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
		writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

		while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
			!(readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

		if (readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_ALD | BE_ISR1_RNK)) {
			ret = -ENXIO;
			goto exit;
		}

		if (read_write == I2C_SMBUS_READ) {
			data->byte = readl(smbus->base + BE_SMBUS_FIFO);

			if (readl(smbus->base + BE_SMBUS_ISR1) & BE_ISR1_FER) {
				ret = -EMSGSIZE;
				goto exit;
			}
		}

		ret = 0;
		break;
	case I2C_SMBUS_BYTE_DATA:
		writel(BE_CR1_IRT, smbus->base + BE_SMBUS_CR1);
		writel(0, smbus->base + BE_SMBUS_CR1);
		writel(addr, smbus->base + BE_SMBUS_ADR1);
		writel(BE_CR1_TRS | BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);

		if (read_write == I2C_SMBUS_WRITE) {
			writel(2, smbus->base + BE_SMBUS_FBCR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
			writel(data->byte, smbus->base + BE_SMBUS_FIFO);
		} else {
			writel(1, smbus->base + BE_SMBUS_FBCR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
		}

		writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
		writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

		while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
			!(readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

		if (readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_ALD | BE_ISR1_RNK)) {
			ret = -ENXIO;
			goto exit;
		}

		if (read_write == I2C_SMBUS_READ) {
			writel(BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);
			writel(1, smbus->base + BE_SMBUS_FBCR1);
			writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
			writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

			while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
				!(readl(smbus->base + BE_SMBUS_ISR1) &
				  (BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

			if (readl(smbus->base + BE_SMBUS_ISR1) &
					(BE_ISR1_ALD | BE_ISR1_RNK)) {
				ret = -ENXIO;
				goto exit;
			}

			data->byte = readl(smbus->base + BE_SMBUS_FIFO);
			if (readl(smbus->base + BE_SMBUS_ISR1) & BE_ISR1_FER) {
				ret = -EMSGSIZE;
				goto exit;
			}
		}

		ret = 0;
		break;
	case I2C_SMBUS_WORD_DATA:
		writel(BE_CR1_IRT, smbus->base + BE_SMBUS_CR1);
		writel(0, smbus->base + BE_SMBUS_CR1);
		writel(addr, smbus->base + BE_SMBUS_ADR1);
		writel(BE_CR1_TRS | BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);

		if (read_write == I2C_SMBUS_WRITE) {
			writel(3, smbus->base + BE_SMBUS_FBCR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
			writel((data->word >> 0) & 0xff,
				smbus->base + BE_SMBUS_FIFO);
			writel((data->word >> 8) & 0xff,
				smbus->base + BE_SMBUS_FIFO);
		} else {
			writel(1, smbus->base + BE_SMBUS_FBCR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
		}

		writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
		writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

		while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
			!(readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

		if (readl(smbus->base + BE_SMBUS_ISR1) &
				(BE_ISR1_ALD | BE_ISR1_RNK)) {
			ret = -ENXIO;
			goto exit;
		}

		if (read_write == I2C_SMBUS_READ) {
			writel(BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);
			writel(2, smbus->base + BE_SMBUS_FBCR1);
			writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
			writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

			while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
				!(readl(smbus->base + BE_SMBUS_ISR1) &
				  (BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

			if (readl(smbus->base + BE_SMBUS_ISR1) &
					(BE_ISR1_ALD| BE_ISR1_RNK)) {
				ret = -ENXIO;
				goto exit;
			}

			data->word  = readl(smbus->base + BE_SMBUS_FIFO);
			data->word |= readl(smbus->base + BE_SMBUS_FIFO) << 8;
			if (readl(smbus->base + BE_SMBUS_ISR1) & BE_ISR1_FER) {
				ret = -EMSGSIZE;
				goto exit;
			}
		}

		ret = 0;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		writel(BE_CR1_IRT, smbus->base + BE_SMBUS_CR1);
		writel(0, smbus->base + BE_SMBUS_CR1);
		writel(addr, smbus->base + BE_SMBUS_ADR1);
		writel(BE_CR1_TRS | BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);

		if (read_write == I2C_SMBUS_WRITE) {
			unsigned txedsize = 0;

			while (txedsize < data->block[0] + 1) {
				unsigned xblocknum;
				unsigned xfersize;

				xfersize = data->block[0] + 1 - txedsize;
				if (xfersize > BE_SMBUS_FIFO_SIZE) {
					xfersize = BE_SMBUS_FIFO_SIZE;
				}

				writel(xfersize, smbus->base + BE_SMBUS_FBCR1);
				if (!txedsize) {
					/*
					 * The first xfer should start with
					 * command byte
					 */
					writel(command,
						smbus->base + BE_SMBUS_FIFO);
					xblocknum = 1;
					++txedsize;
				} else {
					xblocknum = 0;
				}

				while (xblocknum < xfersize) {
					writel(data->block[txedsize++],
						smbus->base + BE_SMBUS_FIFO);
					++xblocknum;
				}

				writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
				writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

				while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
					!(readl(smbus->base + BE_SMBUS_ISR1) &
					  (BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

				if (readl(smbus->base + BE_SMBUS_ISR1) &
						(BE_ISR1_ALD | BE_ISR1_RNK)) {
					ret = -ENXIO;
					goto exit;
				}
			}
		} else {
			unsigned rxedsize = 0;

			writel(1, smbus->base + BE_SMBUS_FBCR1);
			writel(command, smbus->base + BE_SMBUS_FIFO);
			writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
			writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

			while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
				!(readl(smbus->base + BE_SMBUS_ISR1) &
				  (BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

			if (readl(smbus->base + BE_SMBUS_ISR1) &
					(BE_ISR1_ALD | BE_ISR1_RNK)) {
				ret = -ENXIO;
				goto exit;
			}

			writel(BE_CR1_IEB, smbus->base + BE_SMBUS_CR1);

			while (rxedsize < I2C_SMBUS_BLOCK_MAX) {
				unsigned i;

				writel(BE_SMBUS_FIFO_SIZE, smbus->base + BE_SMBUS_FBCR1);
				writel(BE_ISR1_MASK, smbus->base + BE_SMBUS_ISR1);
				writel(BE_CR2_FTE, smbus->base + BE_SMBUS_CR2);

				while ((readl(smbus->base + BE_SMBUS_CR2) & BE_CR2_FTE) &&
					!(readl(smbus->base + BE_SMBUS_ISR1) &
					  (BE_ISR1_TCS | BE_ISR1_ALD | BE_ISR1_RNK)));

				if (readl(smbus->base + BE_SMBUS_ISR1) &
						(BE_ISR1_ALD | BE_ISR1_RNK)) {
					ret = -ENXIO;
					goto exit;
				}

				for (i = 0; i < BE_SMBUS_FIFO_SIZE; ++i) {
					data->block[1 + rxedsize++] =
						readl(smbus->base + BE_SMBUS_FIFO);

					if (readl(smbus->base + BE_SMBUS_ISR1) &
							BE_ISR1_FER) {
						ret = -EMSGSIZE;
						goto exit;
					}
				}
			}

			data->block[0] = rxedsize;
		}

		ret = 0;
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

exit:
	writel(0, smbus->base + BE_SMBUS_CR1);
	return ret;
}

static u32 baikal_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm baikal_smbus_algorithm = {
	.smbus_xfer	= baikal_smbus_xfer,
	.functionality	= baikal_functionality
};

static irqreturn_t baikal_smbus_isr(int irq, void *_dev)
{
	return IRQ_HANDLED;
}

static int baikal_smbus_probe(struct platform_device *pdev)
{
	struct baikal_smbus_dev *smbus;
	void __iomem *base;
	int irq;
	int ret;

	smbus = devm_kzalloc(&pdev->dev, sizeof(*smbus), GFP_KERNEL);
	if (smbus == NULL) {
		return -ENOMEM;
	}

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing interrupt resource\n");
		return irq;
	}

	if (dev_of_node(&pdev->dev)) {
		smbus->smbus_clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(smbus->smbus_clk)) {
			dev_err(&pdev->dev, "missing clock\n");
			return PTR_ERR(smbus->smbus_clk);
		}

		ret = clk_prepare_enable(smbus->smbus_clk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable clock\n");
			return ret;
		}

		smbus->smbus_clk_rate = 0;
#ifdef CONFIG_ACPI
	} else if (to_acpi_device_node(pdev->dev.fwnode) &&
			acpi_evaluate_integer(to_acpi_device_node(pdev->dev.fwnode)->handle,
					"CLK", NULL, &smbus->smbus_clk_rate)) {
		dev_err(&pdev->dev, "missing clock-frequency value\n");
		return -EINVAL;
#endif
	}

	smbus->base = base;
	smbus->dev = &pdev->dev;

	device_property_read_u32(&pdev->dev, "clock-frequency",
					&smbus->bus_clk_rate);
	if (!smbus->bus_clk_rate) {
		smbus->bus_clk_rate = 100000; /* default clock rate */
	}

	ret = devm_request_irq(&pdev->dev, irq, baikal_smbus_isr, 0,
				pdev->name, smbus);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim IRQ%d\n", irq);
		return ret;
	}

	i2c_set_adapdata(&smbus->adapter, smbus);
	strlcpy(smbus->adapter.name, pdev->name, sizeof(smbus->adapter.name));
	smbus->adapter.owner = THIS_MODULE;
	smbus->adapter.algo = &baikal_smbus_algorithm;
	smbus->adapter.dev.parent = &pdev->dev;
	if (dev_of_node(&pdev->dev)) {
		smbus->adapter.dev.of_node = pdev->dev.of_node;
	} else {
		smbus->adapter.dev.fwnode = pdev->dev.fwnode;
	}

	platform_set_drvdata(pdev, smbus);

	ret = baikal_smbus_init(smbus);
	if (ret) {
		dev_err(&pdev->dev, "failed to init SMBus\n");
		goto err_clk;
	}

	ret = i2c_add_adapter(&smbus->adapter);
	if (ret) {
		goto err_clk;
	}

	return 0;

err_clk:
	if (dev_of_node(&pdev->dev)) {
		clk_disable_unprepare(smbus->smbus_clk);
	}

	return ret;
}

static int baikal_smbus_remove(struct platform_device *pdev)
{
	struct baikal_smbus_dev *smbus = platform_get_drvdata(pdev);

	clk_disable_unprepare(smbus->smbus_clk);
	i2c_del_adapter(&smbus->adapter);

	return 0;
}

static const struct of_device_id baikal_smbus_match[] = {
	{ .compatible = "baikal,bm1000-smbus" },

	/*
	 * TODO: "be,smbus" is legacy. Correct prefix is "baikal":
	 * https://www.kernel.org/doc/Documentation/devicetree/bindings/vendor-prefixes.yaml
	 * Use "baikal,b*-smbus" for future development.
	 */
	{ .compatible = "be,smbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, baikal_smbus_match);

static struct platform_driver baikal_smbus_driver = {
	.driver = {
		.name = "baikal-smbus",
		.of_match_table = baikal_smbus_match
	},
	.probe  = baikal_smbus_probe,
	.remove = baikal_smbus_remove
};

module_platform_driver(baikal_smbus_driver);

MODULE_AUTHOR("Georgy Vlasov <georgy.vlasov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal SMBus driver");
MODULE_LICENSE("GPL v2");
