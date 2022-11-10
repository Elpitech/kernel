// SPDX-License-Identifier: GPL-2.0
/*
 * I2C driver for Baikal-S SoC PVT data
 *
 * Copyright (C) 2022 Baikal Electronics, JSC
 *
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/arm-smccc.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spinlock.h>

#define BAIKAL_SMC_PVT_ID	0x82000001
#define PVT_READ		0

#define CA75_0_PVT_BASE		0x4030000
#define CA75_1_PVT_BASE		0x8030000
#define CA75_2_PVT_BASE		0xc030000
#define CA75_3_PVT_BASE		0x10030000
#define CA75_4_PVT_BASE		0x14030000
#define CA75_5_PVT_BASE		0x18030000
#define CA75_6_PVT_BASE		0x1c030000
#define CA75_7_PVT_BASE		0x20030000
#define CA75_8_PVT_BASE		0x24030000
#define CA75_9_PVT_BASE		0x28030000
#define CA75_10_PVT_BASE	0x2c030000
#define CA75_11_PVT_BASE	0x30030000
#define PCIE0_PVT_BASE		0x38030000
#define PCIE1_PVT_BASE		0x3c030000
#define PCIE2_PVT_BASE		0x44030000
#define PCIE3_PVT_BASE		0x48030000
#define PCIE4_PVT_BASE		0x4c030000
#define DDR0_PVT_BASE		0x50030000
#define DDR1_PVT_BASE		0x54030000
#define DDR2_PVT_BASE		0x58030000
#define DDR3_PVT_BASE		0x60030000
#define DDR4_PVT_BASE		0x64030000
#define DDR5_PVT_BASE		0x68030000

static u32 pvt_i2c_readl(u32 pvt_base, u32 offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_ID, PVT_READ, pvt_base, offset,
		      0, 0, 0, 0, &res);

	return res.a0;
}

static const u32 pvt_i2c_addr[] = {
	CA75_0_PVT_BASE,
	CA75_1_PVT_BASE,
	CA75_2_PVT_BASE,
	CA75_3_PVT_BASE,
	CA75_4_PVT_BASE,
	CA75_5_PVT_BASE,
	CA75_6_PVT_BASE,
	CA75_7_PVT_BASE,
	CA75_8_PVT_BASE,
	CA75_9_PVT_BASE,
	CA75_10_PVT_BASE,
	CA75_11_PVT_BASE,
	PCIE0_PVT_BASE,
	PCIE1_PVT_BASE,
	PCIE2_PVT_BASE,
	PCIE3_PVT_BASE,
	PCIE4_PVT_BASE,
	DDR0_PVT_BASE,
	DDR1_PVT_BASE,
	DDR2_PVT_BASE,
	DDR3_PVT_BASE,
	DDR4_PVT_BASE,
	DDR5_PVT_BASE
};

static const u8 pvt_i2c_count = ARRAY_SIZE(pvt_i2c_addr);

struct pvt_i2c_data {
	u8 pvt;
	u8 reg;
	u32 reg_val;
	bool reg_val_valid;
	u8 write_count;
	spinlock_t lock;
};

static bool pvt_i2c_is_in_range(u8 reg)
{
	if ((reg >= 0 && reg <= 0x2b) ||
	    (reg >= 0x40 && reg <= 0x43))
		return true;

	return false;
}

static u8 pvt_i2c_read_data(struct pvt_i2c_data *data)
{
	u8 *reg = &data->reg;

	if (pvt_i2c_is_in_range(*reg)) {
		if (!data->reg_val_valid) {
			data->reg_val = pvt_i2c_readl(pvt_i2c_addr[data->pvt], (*reg >> 2) << 2);
			data->reg_val_valid = true;
		}

		return ((u8 *)&data->reg_val)[*reg & 0x3];
	}

	data->reg_val_valid = false;
	return 0;
}

static int pvt_i2c_slave_cb(struct i2c_client *client, enum i2c_slave_event event, u8 *val)
{
	struct pvt_i2c_data *data = i2c_get_clientdata(client);

	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		data->write_count = 0;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (data->write_count < 2) {
			if (!data->write_count) {
				data->reg = *val;
				data->reg_val = 0;
				data->reg_val_valid = false;
			} else {
				if (*val < pvt_i2c_count)
					data->pvt = *val;
			}
			++data->write_count;
		}
		break;

	case I2C_SLAVE_READ_PROCESSED:
		/* The previous byte made it to the bus, get next one */
		if ((data->reg & 0x3) == 3)
			data->reg_val_valid = false;
		++data->reg;
		/* fallthrough */
	case I2C_SLAVE_READ_REQUESTED:
		spin_lock(&data->lock);
		*val = pvt_i2c_read_data(data);
		spin_unlock(&data->lock);
		/*
		 * Do not increment reg here, because we don't know if this
		 * byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;

	default:
		break;
	}

	return 0;
}

static int pvt_i2c_probe(struct i2c_client *client)
{
	struct pvt_i2c_data *data;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(struct pvt_i2c_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->lock);
	i2c_set_clientdata(client, data);

	ret = i2c_slave_register(client, pvt_i2c_slave_cb);
	return ret;
};

static int pvt_i2c_remove(struct i2c_client *client)
{
	i2c_slave_unregister(client);
	return 0;
}

static const struct of_device_id pvt_i2c_of_match[] = {
	{ .compatible = "baikal,bs1000-pvt-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, pvt_i2c_of_match);

static const struct i2c_device_id pvt_i2c_id[] = {
	{ "pvt-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pvt_i2c_id);

static struct i2c_driver pvt_i2c_driver = {
	.driver = {
		.name = "pvt-i2c",
		.of_match_table = of_match_ptr(pvt_i2c_of_match)
	},
	.probe_new = pvt_i2c_probe,
	.remove = pvt_i2c_remove,
	.id_table = pvt_i2c_id
};
module_i2c_driver(pvt_i2c_driver);

MODULE_AUTHOR("Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal-S SoC I2C PVT driver");
MODULE_LICENSE("GPL v2");
