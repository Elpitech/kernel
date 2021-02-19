/*
 * I2C multiplexer driver for PCA9641 bus master selector
 *
 * Derived from:
 *  pca9541.c
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>

/*
 * The PCA9641 is a bus master selector. It supports two I2C masters connected
 * to a single slave bus.
 *
 * Before each bus transaction, a master has to acquire bus ownership. After the
 * transaction is complete, bus ownership has to be released. This fits well
 * into the I2C multiplexer framework, which provides select and release
 * functions for this purpose. For this reason, this driver is modeled as
 * single-channel I2C bus multiplexer.
 *
 * This driver assumes that the two bus masters are controlled by two different
 * hosts. If a single host controls both masters, platform code has to ensure
 * that only one of the masters is instantiated at any given time.
 */

#define PCA9641_ID_REG		0x00
#define PCA9641_ID_VAL		0x38

#define PCA9641_CONTROL		0x01
#define PCA9641_STAT		0x02
#define PCA9641_RES_TIME	0x03

#define PCA9641_CTL_LOCK_REQ	(1 << 0)
#define PCA9641_CTL_LOCK_GRANT	(1 << 1)
#define PCA9641_CTL_BUSON	(1 << 2)
#define PCA9641_CTL_BUSINIT	(1 << 3)
#define PCA9641_CTL_IDLE_DIS	(1 << 5)

#define PCA9641_STAT_OTHER_LOCK	(1 << 0)

#define MYBUS		(PCA9641_CTL_LOCK_GRANT | PCA9641_CTL_BUSON)
#define mybus(x)	(((x) & MYBUS) == MYBUS)

/* arbitration timeouts, in jiffies */
#define ARB_TIMEOUT	(HZ / 8)	/* 125 ms until forcing bus ownership */
#define ARB2_TIMEOUT	(HZ / 4)	/* 250 ms until acquisition failure */

/* arbitration retry delays, in us */
#define SELECT_DELAY_SHORT	50
#define SELECT_DELAY_LONG	1000

struct pca9641 {
	struct i2c_client *client;
	unsigned long select_timeout;
	unsigned long arb_timeout;
};

static const struct i2c_device_id pca9641_id[] = {
	{"pca9641", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, pca9641_id);

#ifdef CONFIG_OF
static const struct of_device_id pca9641_of_match[] = {
	{ .compatible = "nxp,pca9641" },
	{}
};
MODULE_DEVICE_TABLE(of, pca9641_of_match);
#endif

/*
 * Write to chip register. Don't use i2c_transfer()/i2c_smbus_xfer()
 * as they will try to lock the adapter a second time.
 */
static int pca9641_reg_write(struct i2c_client *client, u8 command, u8 val)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data = { .byte = val };

	return __i2c_smbus_xfer(adap, client->addr, client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
}

/*
 * Read from chip register. Don't use i2c_transfer()/i2c_smbus_xfer()
 * as they will try to lock adapter a second time.
 */
static int pca9641_reg_read(struct i2c_client *client, u8 command)
{
	struct i2c_adapter *adap = client->adapter;
	union i2c_smbus_data data;
	int ret;

	ret = __i2c_smbus_xfer(adap, client->addr, client->flags,
			       I2C_SMBUS_READ, command,
			       I2C_SMBUS_BYTE_DATA, &data);

	return ret ?: data.byte;
}

/*
 * Arbitration management functions
 */

/* Release bus. */
static void pca9641_release_bus(struct i2c_client *client)
{
	int reg;

	reg = pca9641_reg_read(client, PCA9641_CONTROL);
	if (reg >= 0 && (reg & PCA9641_CTL_LOCK_GRANT))
		pca9641_reg_write(client, PCA9641_CONTROL, 0);
}

/*
 * Channel arbitration
 *
 * Return values:
 *  <0: error
 *  0 : bus not acquired
 *  1 : bus acquired
 */
static int pca9641_arbitrate(struct i2c_client *client)
{
	int reg;

	reg = pca9641_reg_read(client, PCA9641_CONTROL);
	if (reg < 0)
		return reg;

	if (mybus(reg)) {
		return 1;
	}

	if (!(reg & PCA9641_CTL_LOCK_REQ)) {
		/* Request ownership */
		pca9641_reg_write(client, PCA9641_RES_TIME, 0);
		pca9641_reg_write(client, PCA9641_CONTROL,
				PCA9641_CTL_LOCK_REQ | PCA9641_CTL_BUSON |
				PCA9641_CTL_IDLE_DIS);
	}
	return 0;
}

static int pca9641_select_chan(struct i2c_mux_core *muxc, u32 chan)
{
	struct pca9641 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;
	int ret;
	unsigned long timeout = jiffies + ARB2_TIMEOUT;
		/* give up after this time */

	data->arb_timeout = jiffies + ARB_TIMEOUT;
		/* force bus ownership after this time */

	do {
		ret = pca9641_arbitrate(client);
		if (ret)
			return ret < 0 ? ret : 0;

		if (data->select_timeout == SELECT_DELAY_SHORT)
			udelay(data->select_timeout);
		else
			msleep(data->select_timeout / 1000);
	} while (time_is_after_eq_jiffies(timeout));

	return -ETIMEDOUT;
}

static int pca9641_release_chan(struct i2c_mux_core *muxc, u32 chan)
{
	struct pca9641 *data = i2c_mux_priv(muxc);
	struct i2c_client *client = data->client;

#if 0 //vvv: Do not disconnect from the bus.
	pca9641_release_bus(client);
#endif
	return 0;
}

/*
 * I2C init/probing/exit functions
 */
static int pca9641_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_mux_core *muxc;
	struct pca9641 *data;
	int ret, reg;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/*
	 * I2C accesses are unprotected here.
	 * We have to lock the I2C segment before releasing the bus.
	 */
	i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	reg = pca9641_reg_read(client, PCA9641_ID_REG);
	if (reg != PCA9641_ID_VAL) {
		i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);
		return -ENXIO;
	}
	pca9641_release_bus(client);
	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

	/* Create mux adapter */

	muxc = i2c_mux_alloc(adap, &client->dev, 1, sizeof(*data),
			     I2C_MUX_ARBITRATOR,
			     pca9641_select_chan, pca9641_release_chan);
	if (!muxc)
		return -ENOMEM;

	data = i2c_mux_priv(muxc);
	data->client = client;

	i2c_set_clientdata(client, muxc);

	ret = i2c_mux_add_adapter(muxc, 0, 0, 0);
	if (ret)
		return ret;

	dev_info(&client->dev, "registered master selector for I2C %s\n",
		 client->name);

	return 0;
}

static int pca9641_remove(struct i2c_client *client)
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);

	i2c_mux_del_adapters(muxc);
	return 0;
}

static struct i2c_driver pca9641_driver = {
	.driver = {
		   .name = "pca9641",
		   .of_match_table = of_match_ptr(pca9641_of_match),
		   },
	.probe = pca9641_probe,
	.remove = pca9641_remove,
	.id_table = pca9641_id,
};

module_i2c_driver(pca9641_driver);

MODULE_AUTHOR("Guenter Roeck <linux@roeck-us.net>");
MODULE_DESCRIPTION("PCA9641 I2C master selector driver");
MODULE_LICENSE("GPL v2");
