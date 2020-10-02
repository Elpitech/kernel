// SPDX-License-Identifier: GPL-2.0
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/pm.h>
#include <linux/rtc.h>
#include <linux/serio.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

enum I2C_REGS {
	R_ID1 = 0,
	R_ID2,
	R_ID3,
	R_ID4,
	R_SOFTOFF_RQ,
	R_PWROFF_RQ,
	R_PWRBTN_STATE,
	R_VERSION1,
	R_VERSION2,
	R_BOOTREASON,
	R_BOOTREASON_ARG,
	R_SCRATCH1,
	R_SCRATCH2,
	R_SCRATCH3,
	R_SCRATCH4,
	R_CAP,
	R_GPIODIR0,
	R_GPIODIR1,
	R_GPIODIR2,
	R_COUNT
};

#define BMC_ID1_VAL 0x49
#define BMC_ID2_VAL 0x54
#define BMC_ID3_VAL 0x58
#define BMC_ID4_VAL0 0x32
#define BMC_ID4_VAL1 0x2

#define BMC_VERSION1	0
#define BMC_VERSION2	2
#define BMC_VERSION2_3	3

#define BMC_CAP_PWRBTN		0x1
#define BMC_CAP_TOUCHPAD	0x2
#define BMC_CAP_RTC		0x4
#define BMC_CAP_FRU		0x8
#define BMC_CAP_GPIODIR		0x10

#define BMC_SERIO_BUFSIZE	7

#define POLL_JIFFIES 100

struct bmc_poll_data {
	struct i2c_client *c;
};

static struct i2c_client *bmc_i2c;
static struct i2c_client *rtc_i2c;
static struct i2c_driver mitx2_bmc_i2c_driver;
static struct input_dev *button_dev;
static struct bmc_poll_data poll_data;
static struct task_struct *polling_task;
#ifdef CONFIG_SERIO
static struct i2c_client *serio_i2c;
static struct task_struct *touchpad_task;
#endif
static u8 bmc_proto_version[3];
static u8 bmc_bootreason[2];
static u8 bmc_scratch[4];
static int bmc_cap;
static const char input_name[] = "BMC input dev";
static u8 prev_ret;

/* BMC RTC */
static int
bmc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	uint8_t rtc_buf[8];
	struct i2c_msg msg;
	int t;
	int rc;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = 8;
	msg.buf = rtc_buf;
	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1) {
		dev_err(dev, "rtc_read_time: i2c_transfer error %d\n", rc);
		return rc;
	}

	tm->tm_sec = bcd2bin(rtc_buf[0] & 0x7f);
	tm->tm_min = bcd2bin(rtc_buf[1] & 0x7f);
	tm->tm_hour = bcd2bin(rtc_buf[2] & 0x3f);
	if (rtc_buf[3] & (1 << 6)) /* PM */
		tm->tm_hour += 12;
	tm->tm_mday  = bcd2bin(rtc_buf[4] & 0x3f);
	tm->tm_mon = bcd2bin(rtc_buf[5] & 0x1f);
	t = rtc_buf[5] >> 5;
	tm->tm_wday = (t == 7) ? 0 : t;
	tm->tm_year = bcd2bin(rtc_buf[6]) + 100; /* year since 1900 */
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	tm->tm_isdst = 0;

	return rtc_valid_tm(tm);
}

static int
bmc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	uint8_t rtc_buf[8];
	struct i2c_msg msg;
	int rc;
	uint8_t seconds, minutes, hours, wday, mday, month, years;

	seconds = bin2bcd(tm->tm_sec);
	minutes = bin2bcd(tm->tm_min);
	hours = bin2bcd(tm->tm_hour);
	wday = tm->tm_wday ? tm->tm_wday : 0x7;
	mday = bin2bcd(tm->tm_mday);
	month = bin2bcd(tm->tm_mon);
	years = bin2bcd(tm->tm_year % 100);

	/* Need sanity check??? */
	rtc_buf[0] = seconds;
	rtc_buf[1] = minutes;
	rtc_buf[2] = hours;
	rtc_buf[3] = 0;
	rtc_buf[4] = mday;
	rtc_buf[5] = month | (wday << 5);
	rtc_buf[6] = years;
	rtc_buf[7] = 0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 8;
	msg.buf = rtc_buf;
	dev_dbg(dev, "rtc_set_time: %08x-%08x\n", *(uint32_t *)&rtc_buf[0],
		*(uint32_t *)&rtc_buf[4]);
	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1)
		dev_err(dev, "i2c write: %d\n", rc);

	return (rc == 1) ? 0 : -EIO;
}

static const struct rtc_class_ops
bmc_rtc_ops = {
	.read_time = bmc_rtc_read_time,
	.set_time = bmc_rtc_set_time,
};

#ifdef CONFIG_SERIO
/* BMC serio (PS/2 touchpad) interface */

static int bmc_serio_write(struct serio *id, unsigned char val)
{
	struct i2c_client *client = id->port_data;
	uint8_t buf[4];
	struct i2c_msg msg;
	int rc;

	buf[0] = val;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = buf;
	dev_dbg(&client->dev, "%s: %02x\n", __func__, val);
	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1)
		dev_err(&client->dev, "i2c write: %d\n", rc);

	return (rc == 1) ? 0 : -EIO;
}

/* returns: -1 on error, +1 if more data available, 0 otherwise */
static int bmc_serio_read(struct i2c_client *client)
{
	struct serio *serio = dev_get_drvdata(&client->dev);
	int i, rc, cnt;
	uint8_t buf[BMC_SERIO_BUFSIZE];
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = BMC_SERIO_BUFSIZE;
	msg.buf = buf;
	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1) {
		dev_err(&client->dev, "%s: i2c_transfer error %d\n",
			__func__, rc);
		return -1;
	}

	cnt = buf[0];
	rc = 0;
	if (cnt > BMC_SERIO_BUFSIZE - 1) {
		cnt = BMC_SERIO_BUFSIZE - 1;
		rc = 1;
	}

	for (i = 0; i < cnt; i++)
		serio_interrupt(serio, buf[i + 1], 0);

	return 0;
}

int
touchpad_poll_fn(void *data)
{
	int ret;

	while (1) {
		if (kthread_should_stop())
			break;
		while ((ret = bmc_serio_read(serio_i2c)) > 0)
			;
		if (ret < 0)
			msleep_interruptible(10000);
		msleep_interruptible(10);
	}
	return 0;
}
#endif /* CONFIG_SERIO */

#ifdef CONFIG_PINCTRL
static uint8_t bmc_pincf_state[3];
#define BMC_NPINS	(sizeof(bmc_pincf_state) * 8)

static struct pinctrl_pin_desc bmc_pin_desc[BMC_NPINS] = {
	PINCTRL_PIN(0, "P0"),
	PINCTRL_PIN(1, "P1"),
	PINCTRL_PIN(2, "P2"),
	PINCTRL_PIN(3, "P3"),
	PINCTRL_PIN(4, "P4"),
	PINCTRL_PIN(5, "P5"),
	PINCTRL_PIN(6, "P6"),
	PINCTRL_PIN(7, "P7"),
	PINCTRL_PIN(8, "P8"),
	PINCTRL_PIN(9, "P9"),
	PINCTRL_PIN(10, "P10"),
	PINCTRL_PIN(11, "P11"),
	PINCTRL_PIN(12, "P12"),
	PINCTRL_PIN(13, "P13"),
	PINCTRL_PIN(14, "P14"),
	PINCTRL_PIN(15, "P15"),
	PINCTRL_PIN(16, "P16"),
	PINCTRL_PIN(17, "P17"),
	PINCTRL_PIN(18, "P18"),
	PINCTRL_PIN(19, "P19"),
	PINCTRL_PIN(20, "P20"),
	PINCTRL_PIN(21, "P21"),
	PINCTRL_PIN(22, "P22"),
	PINCTRL_PIN(23, "P23"),
};

#define PCTRL_DEV	"bmc_pinctrl"

static int bmc_pin_config_get(struct pinctrl_dev *pctldev,
			      unsigned int pin,
			      unsigned long *config)
{
	int idx, bit;

	if (pin > BMC_NPINS)
		return -EINVAL;

	idx = pin >> 3;
	bit = pin & 7;

	*config = !!(bmc_pincf_state[idx] & (1 << bit));
	return 0;
}

static int bmc_pin_config_set(struct pinctrl_dev *pctldev,
			      unsigned int pin,
			      unsigned long *config,
			      unsigned int nc)
{
	int idx, bit;
	enum pin_config_param param;
	int arg;

	if (pin > BMC_NPINS)
		return -EINVAL;

	idx = pin >> 3;
	bit = pin & 7;

	param = pinconf_to_config_param(*config);
	arg = pinconf_to_config_argument(*config);
	if (param != PIN_CONFIG_OUTPUT)
		return -EINVAL;

	if (arg)
		bmc_pincf_state[idx] |= (1 << bit);
	else
		bmc_pincf_state[idx] &= ~(1 << bit);
	dev_dbg(&bmc_i2c->dev, "%s: pin %u, dir %lu\n", __func__, pin,
		*config);

	return i2c_smbus_write_byte_data(bmc_i2c, R_GPIODIR0 + idx,
					 bmc_pincf_state[idx]);
}

void pinconf_generic_dump_config(struct pinctrl_dev *pctldev,
				 struct seq_file *s, unsigned long config);

void pinctrl_utils_free_map(struct pinctrl_dev *pctldev,
			    struct pinctrl_map *map, unsigned int num_maps);

static const struct pinconf_ops bmc_confops = {
	.pin_config_get = bmc_pin_config_get,
	.pin_config_set = bmc_pin_config_set,
	.pin_config_config_dbg_show = pinconf_generic_dump_config,
};

static int bmc_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *bmc_group_name(struct pinctrl_dev *pctldev,
				   unsigned int selector)
{
	return NULL;
}

static const struct pinctrl_ops bmc_ctrl_ops = {
	.get_groups_count = bmc_groups_count,
	.get_group_name = bmc_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static struct pinctrl_desc bmc_pincrtl_desc = {
	.name = PCTRL_DEV,
	.pins = bmc_pin_desc,
	.pctlops = &bmc_ctrl_ops,
	.npins = BMC_NPINS,
	.confops = &bmc_confops,
};

static struct pinctrl_dev *bmc_pinctrl_dev;

static int bmc_pinctrl_register(struct device *dev)
{
	struct pinctrl_dev *pctrl_dev;
	struct platform_device *pbdev;

	pbdev = platform_device_alloc(PCTRL_DEV, -1);
	pbdev->dev.parent = dev;
	pbdev->dev.of_node = of_find_node_by_name(dev->of_node, "bmc_pinctrl");
	platform_device_add(pbdev);
	pctrl_dev = devm_pinctrl_register(&pbdev->dev, &bmc_pincrtl_desc, NULL);
	if (IS_ERR(pctrl_dev)) {
		dev_err(&pbdev->dev, "Can't register pinctrl (%ld)\n",
			PTR_ERR(pctrl_dev));
		return PTR_ERR(pctrl_dev);

	dev_info(&pbdev->dev, "BMC pinctrl registered\n");
	bmc_pinctrl_dev = pctrl_dev;

	/* reset all pins to default state */
	i2c_smbus_write_byte_data(to_i2c_client(dev), R_GPIODIR0, 0);
	i2c_smbus_write_byte_data(to_i2c_client(dev), R_GPIODIR1, 0);
	i2c_smbus_write_byte_data(to_i2c_client(dev), R_GPIODIR2, 0);
	return 0;
}

static void bmc_pinctrl_unregister(void)
{
	if (bmc_pinctrl_dev)
		devm_pinctrl_unregister(&bmc_i2c->dev, bmc_pinctrl_dev);
}

#endif

void
bmc_pwroff_rq(void)
{
	int ret = 0;

	dev_info(&bmc_i2c->dev, "Write reg R_PWROFF_RQ\n");
	ret = i2c_smbus_write_byte_data(bmc_i2c, R_PWROFF_RQ, 0x01);
	dev_info(&bmc_i2c->dev, "ret: %i\n", ret);
}

int
pwroff_rq_poll_fn(void *data)
{
	int ret;

	while (1) {
		if (kthread_should_stop())
			break;
		dev_dbg(&poll_data.c->dev, "Polling\n");
		ret = i2c_smbus_read_byte_data(poll_data.c, R_SOFTOFF_RQ);
		dev_dbg(&poll_data.c->dev, "Polling returned: %i\n", ret);
		if (prev_ret != ret) {
			dev_info(&poll_data.c->dev, "key change [%i]\n", ret);
			if (ret < 0) {
				dev_err(&poll_data.c->dev,
					"Could not read register %x\n",
					R_SOFTOFF_RQ);
				return -EIO;
			} else if (ret != 0) {
				dev_info(&poll_data.c->dev,
					 "PWROFF \"irq\" detected [%i]\n", ret);
				input_event(button_dev, EV_KEY, KEY_POWER, 1);
			} else {
				input_event(button_dev, EV_KEY, KEY_POWER, 0);
			}
			input_sync(button_dev);
		}
		prev_ret = ret;

		msleep_interruptible(100);
	}
	do_exit(1);
	return 0;
}

static int
mitx2_bmc_validate(struct i2c_client *client)
{
	int ret = 0;
	int i = 0;
	static const u8 regs[] = {R_ID1, R_ID2, R_ID3};
	static const u8 vals[] = {BMC_ID1_VAL, BMC_ID2_VAL, BMC_ID3_VAL};

	bmc_proto_version[0] = 0;
	bmc_proto_version[1] = 0;
	bmc_proto_version[2] = 0;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = i2c_smbus_read_byte_data(client, regs[i]);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				regs[i]);
			return -EIO;
		}
		if (ret != vals[i]) {
			dev_err(&client->dev,
				"Bad value [0x%02x] in register 0x%02x, should be [0x%02x]\n",
				 ret, regs[i], vals[i]);

			return -ENODEV;
		}
	}
	ret = i2c_smbus_read_byte_data(client, R_ID4);
	if (ret < 0) {
		dev_err(&client->dev, "Could not read register %x\n", R_ID4);
		return -EIO;
	}
	if (ret == BMC_ID4_VAL0) {
		bmc_proto_version[0] = 0;
	} else if (ret == BMC_ID4_VAL1) {
		bmc_proto_version[0] = 2;
		ret = i2c_smbus_read_byte_data(client, R_VERSION1);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_VERSION1);
			return -EIO;
		}
		bmc_proto_version[1] = ret;
		ret = i2c_smbus_read_byte_data(client, R_VERSION2);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_VERSION2);
			return -EIO;
		}
		bmc_proto_version[2] = ret;
		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_BOOTREASON);
			return -EIO;
		}
		bmc_bootreason[0] = ret;
		dev_info(&client->dev, "BMC bootreason[0]->%i\n", ret);
		ret = i2c_smbus_read_byte_data(client, R_BOOTREASON_ARG);
		if (ret < 0) {
			dev_err(&client->dev, "Could not read register %x\n",
				R_BOOTREASON_ARG);
			return -EIO;
		}
		bmc_bootreason[1] = ret;
		dev_info(&client->dev, "BMC bootreason[1]->%i\n", ret);
		for (i = R_SCRATCH1; i <= R_SCRATCH4; i++) {
			ret = i2c_smbus_read_byte_data(client, i);
			if (ret < 0) {
				dev_err(&client->dev,
					"Could not read register %x\n", i);
				return -EIO;
			}
			bmc_scratch[i - R_SCRATCH1] = ret;
		}
		if (bmc_proto_version[2] >= BMC_VERSION2_3) {
			ret = i2c_smbus_read_byte_data(client, R_CAP);
			if (ret >= 0)
				bmc_cap = ret;
			dev_info(&client->dev,
				 "BMC extended capabilities %x\n", bmc_cap);
		} else {
			bmc_cap = BMC_CAP_PWRBTN;
		}
	} else {
		dev_err(&client->dev, "Bad value [0x%02x] in register 0x%02x\n",
			ret, R_ID4);
		return -ENODEV;
	}
	dev_info(&client->dev, "BMC seems to be valid\n");
	return 0;
}

static int
bmc_create_client_devices(struct device *bmc_dev)
{
	int ret = 0;
	struct rtc_device *rtc_dev;
	struct i2c_client *client = to_i2c_client(bmc_dev);
	int client_addr = client->addr + 1;

	if (bmc_cap & BMC_CAP_TOUCHPAD) {
#ifdef CONFIG_SERIO
		struct serio *serio;

		serio_i2c = i2c_new_ancillary_device(client,
						     "bmc_serio", client_addr);
		if (!serio_i2c) {
			dev_err(&client->dev, "Can't get serio secondary\n");
			ret = -ENOMEM;
			goto fail;
		}
		serio = devm_kzalloc(&serio_i2c->dev, sizeof(struct serio),
				     GFP_KERNEL);
		if (!serio) {
			dev_err(&serio_i2c->dev, "Can't allocate serio\n");
			ret = -ENOMEM;
			i2c_unregister_device(serio_i2c);
			serio_i2c = NULL;
			goto skip_tp;
		}
		serio->write = bmc_serio_write;
		serio->port_data = serio_i2c;
		serio->id.type = SERIO_PS_PSTHRU;
		serio_register_port(serio);
		dev_set_drvdata(&serio_i2c->dev, serio);
		touchpad_task = kthread_run(touchpad_poll_fn, NULL,
					    "BMC serio poll task");

skip_tp:
#endif
		client_addr++;
	}

	if (bmc_cap & BMC_CAP_RTC) {
		rtc_i2c = i2c_new_ancillary_device(client,
						   "bmc_rtc", client_addr);
		if (!rtc_i2c) {
			dev_err(&client->dev, "Can't get RTC secondary\n");
			ret = -ENOMEM;
			goto fail;
		}

		rtc_dev = devm_rtc_device_register(&rtc_i2c->dev, "bmc_rtc",
						   &bmc_rtc_ops, THIS_MODULE);
		if (IS_ERR(rtc_dev)) {
			ret = PTR_ERR(rtc_dev);
			dev_err(&client->dev,
				"Failed to register RTC device: %d\n",
				ret);
			i2c_unregister_device(rtc_i2c);
			rtc_i2c = NULL;
		}
fail:
		client_addr++;
	}

#ifdef CONFIG_PINCTRL
	if (bmc_cap & BMC_CAP_GPIODIR || 1 /*vvv*/)
		bmc_pinctrl_register(bmc_dev);
#endif

	return ret;
}

static int
mitx2_bmc_i2c_probe(struct i2c_client *client,
		    const struct i2c_device_id *id)
{
	int err = 0;
	int i = 0;

	dev_info(&client->dev, "mitx2 bmc probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	for (i = 0; i < 10; i++) {
		err = mitx2_bmc_validate(client);
		if (!err)
			break;
		msleep_interruptible(20);
	}
	if (err)
		return err;

	if (bmc_cap & BMC_CAP_PWRBTN) {
		button_dev = input_allocate_device();
		if (!button_dev) {
			dev_err(&client->dev, "Not enough memory\n");
			return -ENOMEM;
		}

		button_dev->id.bustype = BUS_I2C;
		button_dev->dev.parent = &client->dev;
		button_dev->name = input_name;
		button_dev->phys = "bmc-input0";
		button_dev->evbit[0] = BIT_MASK(EV_KEY);
		button_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

		err = input_register_device(button_dev);
		if (err) {
			dev_err(&client->dev, "Failed to register device\n");
			input_free_device(button_dev);
			return err;
		}

		dev_info(&client->dev, "Starting polling thread\n");
		poll_data.c = client;
		polling_task = kthread_run(pwroff_rq_poll_fn, NULL,
					   "BMC poll task");
	}

	if (bmc_cap || 1 /*vvv*/)
		err = bmc_create_client_devices(&client->dev);

	bmc_i2c = client;
	/* register as poweroff handler */
	pm_power_off = bmc_pwroff_rq;

	return 0;
}

static int
mitx2_bmc_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_SERIO
	struct serio *serio;
#endif

	if (button_dev) {
		kthread_stop(polling_task);
		input_unregister_device(button_dev);
	}
#ifdef CONFIG_SERIO
	if (serio_i2c) {
		kthread_stop(touchpad_task);
		serio = dev_get_drvdata(&serio_i2c->dev);
		serio_unregister_port(serio);
		i2c_unregister_device(serio_i2c);
	}
#endif
	if (rtc_i2c)
		i2c_unregister_device(rtc_i2c);
#ifdef CONFIG_PINCTRL
	bmc_pinctrl_unregister();
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mitx2_bmc_of_match[] = {
	{ .compatible = "tp,mitx2-bmc" },
	{}
};
MODULE_DEVICE_TABLE(of, mitx2_bmc_of_match);
#endif

static const struct i2c_device_id mitx2_bmc_i2c_id[] = {
	{ "mitx2_bmc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mitx2_bmc_i2c_id);

static ssize_t
version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i.%i.%i\n", bmc_proto_version[0],
			bmc_proto_version[1], bmc_proto_version[2]);
}

static struct kobj_attribute version_attribute =
	__ATTR(version, 0664, version_show, NULL);

static ssize_t
bootreason_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (bmc_bootreason[0] |
			(bmc_bootreason[1] << 8)));
}

static struct kobj_attribute bootreason_attribute =
	__ATTR(bootreason, 0664, bootreason_show, NULL);

static ssize_t
scratch_show(struct kobject *kobj,
	     struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%i\n", (bmc_scratch[0] | (bmc_scratch[1] << 8) |
			(bmc_scratch[2] << 16) | (bmc_scratch[3] << 24)));
}

static struct kobj_attribute scratch_attribute =
	__ATTR(scratch, 0664, scratch_show, NULL);

static struct attribute *bmc_attrs[] = {
	&version_attribute.attr,
	&bootreason_attribute.attr,
	&scratch_attribute.attr,
	NULL,
};

ATTRIBUTE_GROUPS(bmc);

static struct i2c_driver mitx2_bmc_i2c_driver = {
	.driver		= {
		.name	= "mitx2-bmc",
		.of_match_table = of_match_ptr(mitx2_bmc_of_match),
		.groups = bmc_groups,
	},
	.probe		= mitx2_bmc_i2c_probe,
	.remove	  = mitx2_bmc_i2c_remove,
	.id_table	= mitx2_bmc_i2c_id,
};
module_i2c_driver(mitx2_bmc_i2c_driver);

MODULE_AUTHOR("Konstantin Kirik");
MODULE_DESCRIPTION("mITX2 BMC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("serial:bmc");
