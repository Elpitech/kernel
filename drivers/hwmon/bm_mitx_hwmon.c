// SPDX-License-Identifier: GPL-2.0
/*
 * bm_mitx_hwmon.c - driver for the Baikal-M-based Mini-ITX board
 *                   hardware monitoring features
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define MITX_HWMON_VMON_SIZE      2
#define MITX_HWMON_IMON_SIZE      2
#define MITX_HWMON_TEMP_SIZE      2
#define MITX_HWMON_FAN_RPM_SIZE   2
#define MITX_HWMON_FAN_LVL_SIZE   1
#define MITX_HWMON_FAN_FREQ_SIZE  2
#define MITX_HWMON_TEMP_RES       100
#define MITX_HWMON_FAN_LVL_MAX    100
#define MITX_HWMON_VMON_CHANNELS  3
#define MITX_HWMON_IMON_CHANNELS  2
#define MITX_HWMON_TEMP_CHANNELS  3
#define MITX_HWMON_FAN_CHANNELS   2
#define MITX_HWMON_TABLE_SIZE_MAX 8
#define MITX_HWMON_TMON_CONFIG_SIZE 0x20

/* HWMON registers  */
#define MITX_HWMON_REG_VEN_ID          0x00
#define MITX_HWMON_REG_DEV_ID          0x02
#define MITX_HWMON_REG_CAP             0x04
#define MITX_HWMON_REG_VMON_BASE       0x11
#define MITX_HWMON_REG_IMON_BASE       0x19
#define MITX_HWMON_REG_VLOW_BASE       0x21
#define MITX_HWMON_REG_ILOW_BASE       0x29
#define MITX_HWMON_REG_VHIGH_BASE      0x31
#define MITX_HWMON_REG_IHIGH_BASE      0x39
#define MITX_HWMON_REG_VMON(n)         (MITX_HWMON_REG_VMON_BASE + \
					(n) * MITX_HWMON_VMON_SIZE)
#define MITX_HWMON_REG_IMON(n)         (MITX_HWMON_REG_IMON_BASE + \
					(n) * MITX_HWMON_IMON_SIZE)
#define MITX_HWMON_REG_VLOW(n)         (MITX_HWMON_REG_VLOW_BASE + \
					(n) * MITX_HWMON_VMON_SIZE)
#define MITX_HWMON_REG_ILOW(n)         (MITX_HWMON_REG_ILOW_BASE + \
					(n) * MITX_HWMON_IMON_SIZE)
#define MITX_HWMON_REG_VHIGH(n)        (MITX_HWMON_REG_VHIGH_BASE + \
					(n) * MITX_HWMON_VMON_SIZE)
#define MITX_HWMON_REG_IHIGH(n)        (MITX_HWMON_REG_IHIGH_BASE + \
					(n) * MITX_HWMON_IMON_SIZE)
#define MITX_HWMON_REG_TEMP_BASE       0x41
#define MITX_HWMON_REG_TCRIT_BASE      0x47
#define MITX_HWMON_REG_TLOW_BASE       0x51
#define MITX_HWMON_REG_THIGH_BASE      0x57
#define MITX_HWMON_REG_TEMP(n)         (MITX_HWMON_REG_TEMP_BASE + \
					(n) * MITX_HWMON_TEMP_SIZE)
#define MITX_HWMON_REG_TLOW(n)         (MITX_HWMON_REG_TLOW_BASE + \
					(n) * MITX_HWMON_TEMP_SIZE)
#define MITX_HWMON_REG_THIGH(n)        (MITX_HWMON_REG_THIGH_BASE + \
					(n) * MITX_HWMON_TEMP_SIZE)
#define MITX_HWMON_REG_TCRIT(n)        (MITX_HWMON_REG_TCRIT_BASE + \
					(n) * MITX_HWMON_TEMP_SIZE)
#define MITX_HWMON_REG_TMON_CONTROL    0x40
#define MITX_HWMON_REG_FAN_RPM_BASE    0x61
#define MITX_HWMON_REG_FAN_LVL_BASE    0x71
#define MITX_HWMON_REG_FAN_FREQ_BASE   0x75
#define MITX_HWMON_REG_FAN_RPM(n)      (MITX_HWMON_REG_FAN_RPM_BASE + \
					(n) * MITX_HWMON_FAN_RPM_SIZE)
#define MITX_HWMON_REG_FAN_LVL(n)      (MITX_HWMON_REG_FAN_LVL_BASE + \
					(n) * MITX_HWMON_FAN_LVL_SIZE)
#define MITX_HWMON_REG_FAN_FREQ(n)     (MITX_HWMON_REG_FAN_FREQ_BASE + \
					(n) * MITX_HWMON_FAN_FREQ_SIZE)
#define MITX_HWMON_REG_TMON_CONFIG_TEMP_BASE    0x90
#define MITX_HWMON_REG_TMON_CONFIG_SPEED_BASE   0xA0
#define MITX_HWMON_REG_TMON_CONFIG_HYST_BASE    0xAC
#define MITX_HWMON_REG_TMON_CONFIG_SIZE_BASE    0xAE
#define MITX_HWMON_REG_TMON_CONFIG_TEMP(id, n)  \
				(MITX_HWMON_REG_TMON_CONFIG_TEMP_BASE + \
				(id) * MITX_HWMON_TMON_CONFIG_SIZE + \
				(n) * MITX_HWMON_TEMP_SIZE)
#define MITX_HWMON_REG_TMON_CONFIG_SPEED(id, n) \
				(MITX_HWMON_REG_TMON_CONFIG_SPEED_BASE + \
				(id) * MITX_HWMON_TMON_CONFIG_SIZE + \
				(n) * MITX_HWMON_FAN_LVL_SIZE)
#define MITX_HWMON_REG_TMON_CONFIG_HYST(id)     \
				(MITX_HWMON_REG_TMON_CONFIG_HYST_BASE + \
				(id) * MITX_HWMON_TMON_CONFIG_SIZE)
#define MITX_HWMON_REG_TMON_CONFIG_SIZE(id)     \
				(MITX_HWMON_REG_TMON_CONFIG_SIZE_BASE + \
				(id) * MITX_HWMON_TMON_CONFIG_SIZE)

#define MITX_HWMON_REG_VEN_ID_TPL            0xCCED
#define MITX_HWMON_REG_DEV_ID_MITX_HWMON     0xA007
#define MITX_HWMON_REG_CAP_POWER             0x01
#define MITX_HWMON_REG_CAP_TMON              0x02
#define MITX_HWMON_REG_CAP_TMON_CONFIG       0x04
#define MITX_HWMON_REG_CAP_FANCTL            0x08
#define MITX_HWMON_REG_TMON_CONTROL_ACTIVE   0x01
#define MITX_HWMON_REG_TMON_CONTROL_STATE    0x0F
#define MITX_HWMON_REG_TMON_CONTROL_OP       0xF0
#define MITX_HWMON_REG_TMON_CONTROL_OP_LOAD  0x10
#define MITX_HWMON_REG_TMON_CONTROL_OP_STORE 0x30

/*
 * Data structures and manipulation thereof
 */

#define MITX_HWMON_TMON_TABLE_ID(x) (((x) >> 4) & 0x0F)
#define MITX_HWMON_TMON_TABLE_VALUE_INDEX(x) ((x) & 0x0F)
#define MITX_HWMON_TMON_DEV_INDEX(id, idx) (((id) << 4) | ((idx) & 0x0F))

#define MITX_HWMON_VALUE_TYPE(type) \
struct bm_mitx_hwmon_##type { \
	type value; \
	type low; \
	type high; \
	type crit; \
}

MITX_HWMON_VALUE_TYPE(u16);
MITX_HWMON_VALUE_TYPE(s16);

struct bm_mitx_hwmon_fan_data {
	struct bm_mitx_hwmon_u16 rpm;
	u16 freq;
	u8 speed;
};

struct bm_mitx_hwmon_tmon_table_value {
	s16 temp;
	u8 speed;
};

struct bm_mitx_hwmon_tmon_table {
	u8 size;
	s16 hysteresis;
	struct bm_mitx_hwmon_tmon_table_value value[MITX_HWMON_TABLE_SIZE_MAX];
};

struct bm_mitx_hwmon_tmon_data {
	u8 control;
	struct bm_mitx_hwmon_tmon_table table[MITX_HWMON_FAN_CHANNELS];
};

struct bm_mitx_hwmon_data {
	unsigned short addr;
	struct device *hwmon_dev;

	const char *name;
	struct mutex update_lock; /* protect register access */
	char valid;
	unsigned long last_updated;	/* In jiffies */
	unsigned long last_limits;	/* In jiffies */

	/* Register values */
	u8 caps;
	struct bm_mitx_hwmon_u16 vmon[MITX_HWMON_VMON_CHANNELS];
	struct bm_mitx_hwmon_u16 imon[MITX_HWMON_IMON_CHANNELS];
	struct bm_mitx_hwmon_s16 temp[MITX_HWMON_TEMP_CHANNELS];
	struct bm_mitx_hwmon_fan_data fan[MITX_HWMON_FAN_CHANNELS];
	struct bm_mitx_hwmon_tmon_data tmon;
};

static bool extended_attrs;

static u8 bm_mitx_hwmon_read_byte(struct i2c_client *client, u8 reg)
{
	s32 result = i2c_smbus_read_byte_data(client, reg);

	if (result < 0) {
		dev_err(&client->dev, "read byte failed: %d", result);
		result = 0;
	}
	return result;
}

static u16 bm_mitx_hwmon_read_word(struct i2c_client *client, u8 reg)
{
	s32 result = i2c_smbus_read_word_data(client, reg);

	if (result < 0) {
		dev_err(&client->dev, "read word failed: %d", result);
		result = 0;
	}
	return result;
}

static void bm_mitx_hwmon_write_byte(struct i2c_client *client,
		u8 reg, u8 data)
{
	s32 result = i2c_smbus_write_byte_data(client, reg, data);

	if (result < 0)
		dev_err(&client->dev, "write byte failed: %d", result);
}

static void bm_mitx_hwmon_write_word(struct i2c_client *client,
		u8 reg, u16 data)
{
	s32 result = i2c_smbus_write_word_data(client, reg, data);

	if (result < 0)
		dev_err(&client->dev, "write word failed: %d", result);
}

static void bm_mitx_hwmon_update_temp_thresholds(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->temp); index++) {
		data->temp[index].crit = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_TCRIT(index));
		data->temp[index].high = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_THIGH(index));
		data->temp[index].low = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_TLOW(index));
	}
}

static void bm_mitx_hwmon_update_vmon_thresholds(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->vmon); index++) {
		data->vmon[index].high = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_VHIGH(index));
		data->vmon[index].low = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_VLOW(index));
	}
}

static void bm_mitx_hwmon_update_imon_thresholds(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->imon); index++) {
		data->imon[index].high = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_IHIGH(index));
		data->imon[index].low = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_ILOW(index));
	}
}

static void bm_mitx_hwmon_update_temp(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->temp); index++) {
		data->temp[index].value = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_TEMP(index));
	}
}

static void bm_mitx_hwmon_update_fan(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->fan); index++) {
		data->fan[index].freq = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_FAN_FREQ(index));
		data->fan[index].speed = bm_mitx_hwmon_read_byte(client,
			MITX_HWMON_REG_FAN_LVL(index));
		data->fan[index].rpm.value = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_FAN_RPM(index));
	}
}

static void bm_mitx_hwmon_update_vmon(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->vmon); index++) {
		data->vmon[index].value = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_VMON(index));
	}
}

static void bm_mitx_hwmon_update_imon(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->imon); index++) {
		data->imon[index].value = bm_mitx_hwmon_read_word(client,
			MITX_HWMON_REG_IMON(index));
	}
}

static void bm_mitx_hwmon_update_tmon_table_value(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data, size_t id)
{
	size_t index;

	for (index = 0;
	index < ARRAY_SIZE(data->tmon.table[id].value);
	index++) {
		data->tmon.table[id].value[index].temp =
			bm_mitx_hwmon_read_word(client,
				MITX_HWMON_REG_TMON_CONFIG_TEMP(id, index));
		data->tmon.table[id].value[index].speed =
			bm_mitx_hwmon_read_byte(client,
				MITX_HWMON_REG_TMON_CONFIG_SPEED(id, index));
	}
}

static void bm_mitx_hwmon_update_tmon_tables(struct i2c_client *client,
				struct bm_mitx_hwmon_data *data)
{
	size_t index;

	for (index = 0; index < ARRAY_SIZE(data->tmon.table); index++) {
		data->tmon.table[index].size = bm_mitx_hwmon_read_byte(client,
				MITX_HWMON_REG_TMON_CONFIG_SIZE(index));
		data->tmon.table[index].hysteresis =
			bm_mitx_hwmon_read_word(client,
				MITX_HWMON_REG_TMON_CONFIG_HYST(index));
		bm_mitx_hwmon_update_tmon_table_value(client, data, index);
	}
}

static struct bm_mitx_hwmon_data *bm_mitx_hwmon_update_device(
		struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	/* Limit registers cache is refreshed after 60 seconds */
	if (time_after(jiffies, data->last_limits + 60 * HZ)
		|| !data->valid) {
		bm_mitx_hwmon_update_temp_thresholds(client, data);
		if (data->caps & MITX_HWMON_REG_CAP_POWER) {
			bm_mitx_hwmon_update_vmon_thresholds(client, data);
			bm_mitx_hwmon_update_imon_thresholds(client, data);
		}
		if (data->caps & MITX_HWMON_REG_CAP_TMON_CONFIG)
			bm_mitx_hwmon_update_tmon_tables(client, data);
		data->last_limits = jiffies;
	}

	/* Measurement registers cache is refreshed after 2 second */
	if (time_after(jiffies, data->last_updated + 2 * HZ)
		|| !data->valid) {
		bm_mitx_hwmon_update_temp(client, data);
		bm_mitx_hwmon_update_fan(client, data);
		if (data->caps & MITX_HWMON_REG_CAP_POWER) {
			bm_mitx_hwmon_update_vmon(client, data);
			bm_mitx_hwmon_update_imon(client, data);
		}
		data->tmon.control = bm_mitx_hwmon_read_byte(client,
			MITX_HWMON_REG_TMON_CONTROL);
		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);
	return data;
}

static bool auto_mode_enabled(u8 control)
{
	return (control & MITX_HWMON_REG_TMON_CONTROL_ACTIVE) != 0;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute
		*attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->fan[nr].speed);
}

static ssize_t set_pwm(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	if (auto_mode_enabled(data->tmon.control))
		return -EINVAL;

	val = clamp_val(val, 0, MITX_HWMON_FAN_LVL_MAX);
	mutex_lock(&data->update_lock);
	data->fan[nr].speed = val;
	bm_mitx_hwmon_write_byte(client, MITX_HWMON_REG_FAN_LVL(nr),
		data->fan[nr].speed);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_pwm_auto(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n",
		auto_mode_enabled(data->tmon.control) ? 1 : 0);
}

static int set_pwm_auto_direct(struct i2c_client *client, int nr, int val)
{
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	u8 control;

	if (val < 0 || val > 1)
		return -EINVAL;

	control = bm_mitx_hwmon_read_byte(client, MITX_HWMON_REG_TMON_CONTROL);
	control &= ~MITX_HWMON_REG_TMON_CONTROL_ACTIVE;
	switch (val) {
	case 0: /* PWM */
		data->fan[nr].speed = MITX_HWMON_FAN_LVL_MAX;
		break;
	case 1: /* AUTOMATIC */
		control |= MITX_HWMON_REG_TMON_CONTROL_ACTIVE;
		break;
	}

	bm_mitx_hwmon_write_byte(client, MITX_HWMON_REG_TMON_CONTROL, control);
	data->tmon.control = control;
	if (val == 0)
		bm_mitx_hwmon_write_byte(client, MITX_HWMON_REG_FAN_LVL(nr),
				data->fan[nr].speed);
	return 0;
}

static ssize_t set_pwm_auto(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->update_lock);
	err = set_pwm_auto_direct(client, nr, val);
	mutex_unlock(&data->update_lock);
	return err ? err : count;
}

static ssize_t show_vmon(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->vmon[nr].value);
}

static ssize_t show_vmon_max(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->vmon[nr].high);
}

static ssize_t show_vmon_min(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->vmon[nr].low);
}

static ssize_t set_vmon_max(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, U16_MAX);
	mutex_lock(&data->update_lock);
	data->vmon[nr].high = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_VHIGH(nr),
		data->vmon[nr].high);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_vmon_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, U16_MAX);
	mutex_lock(&data->update_lock);
	data->vmon[nr].low = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_VLOW(nr),
		data->vmon[nr].low);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_imon(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->imon[nr].value);
}

static ssize_t show_imon_max(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->imon[nr].high);
}

static ssize_t show_imon_min(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->imon[nr].low);
}

static ssize_t set_imon_max(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, U16_MAX);
	mutex_lock(&data->update_lock);
	data->imon[nr].high = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_IHIGH(nr),
		data->imon[nr].high);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_imon_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, U16_MAX);
	mutex_lock(&data->update_lock);
	data->imon[nr].low = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_ILOW(nr),
		data->imon[nr].low);
	mutex_unlock(&data->update_lock);
	return count;
}

#define TEMP_FROM_REG(val)   (s32)((val) * 1000 / MITX_HWMON_TEMP_RES)
#define TEMP_TO_REG(val)     (s32)((val) * MITX_HWMON_TEMP_RES / 1000)

static ssize_t show_temp(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[nr].value));
}

static ssize_t show_temp_crit(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[nr].crit));
}

static ssize_t show_temp_max(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[nr].high));
}

static ssize_t show_temp_min(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[nr].low));
}

static ssize_t set_temp_crit(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(TEMP_TO_REG(val), S16_MIN, S16_MAX);
	mutex_lock(&data->update_lock);
	data->temp[nr].crit = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_TCRIT(nr),
		data->temp[nr].crit);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_temp_max(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(TEMP_TO_REG(val), S16_MIN, S16_MAX);
	mutex_lock(&data->update_lock);
	data->temp[nr].high = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_THIGH(nr),
		data->temp[nr].high);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_temp_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(TEMP_TO_REG(val), S16_MIN, S16_MAX);
	mutex_lock(&data->update_lock);
	data->temp[nr].low = val;
	bm_mitx_hwmon_write_word(client, MITX_HWMON_REG_TLOW(nr),
		data->temp[nr].low);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_temp_table_size(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t id = MITX_HWMON_TMON_TABLE_ID(to_sensor_dev_attr(attr)->index);
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->tmon.table[id].size);
}

static ssize_t set_temp_table_size(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	size_t id = MITX_HWMON_TMON_TABLE_ID(to_sensor_dev_attr(attr)->index);
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, ARRAY_SIZE(data->tmon.table[id].value));
	mutex_lock(&data->update_lock);
	data->tmon.table[id].size = val;
	bm_mitx_hwmon_write_byte(client,
		MITX_HWMON_REG_TMON_CONFIG_SIZE(id),
		data->tmon.table[id].size);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_temp_table_hysteresis(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t id = MITX_HWMON_TMON_TABLE_ID(to_sensor_dev_attr(attr)->index);
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n",
		TEMP_FROM_REG(data->tmon.table[id].hysteresis));
}

static ssize_t set_temp_table_hysteresis(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	size_t id = MITX_HWMON_TMON_TABLE_ID(to_sensor_dev_attr(attr)->index);
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(TEMP_TO_REG(val), S16_MIN, S16_MAX);
	mutex_lock(&data->update_lock);
	data->tmon.table[id].hysteresis = val;
	bm_mitx_hwmon_write_word(client,
		MITX_HWMON_REG_TMON_CONFIG_HYST(id),
		data->tmon.table[id].hysteresis);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_temp_table_value_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	size_t id = MITX_HWMON_TMON_TABLE_ID(nr);
	size_t index = MITX_HWMON_TMON_TABLE_VALUE_INDEX(nr);
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%d\n",
			TEMP_FROM_REG(data->tmon.table[id].value[index].temp));
}

static ssize_t set_temp_table_value_temp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	size_t id = MITX_HWMON_TMON_TABLE_ID(nr);
	size_t index = MITX_HWMON_TMON_TABLE_VALUE_INDEX(nr);
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(TEMP_TO_REG(val), S16_MIN, S16_MAX);
	mutex_lock(&data->update_lock);
	data->tmon.table[id].value[index].temp = val;
	bm_mitx_hwmon_write_word(client,
		MITX_HWMON_REG_TMON_CONFIG_TEMP(id, index),
		data->tmon.table[id].value[index].temp);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t show_temp_table_value_speed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int nr = to_sensor_dev_attr(attr)->index;
	size_t id = MITX_HWMON_TMON_TABLE_ID(nr);
	size_t index = MITX_HWMON_TMON_TABLE_VALUE_INDEX(nr);
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev);

	return sprintf(buf, "%u\n", data->tmon.table[id].value[index].speed);
}

static ssize_t set_temp_table_value_speed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = to_sensor_dev_attr(attr)->index;
	size_t id = MITX_HWMON_TMON_TABLE_ID(nr);
	size_t index = MITX_HWMON_TMON_TABLE_VALUE_INDEX(nr);
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = clamp_val(val, 0, MITX_HWMON_FAN_LVL_MAX);
	mutex_lock(&data->update_lock);
	data->tmon.table[id].value[index].speed = val;
	bm_mitx_hwmon_write_byte(client,
		MITX_HWMON_REG_TMON_CONFIG_SPEED(id, index),
		data->tmon.table[id].value[index].speed);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_config_load(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	u8 val = data->tmon.control;

	val &= ~MITX_HWMON_REG_TMON_CONTROL_OP;
	val |= MITX_HWMON_REG_TMON_CONTROL_OP_LOAD;
	mutex_lock(&data->update_lock);
	bm_mitx_hwmon_write_byte(client, MITX_HWMON_REG_TMON_CONTROL, val);
	/* Configuration values have been reset, so we ought to try and
	 * update cached ones as soon as we get a chance
	 */
	data->last_limits = 0;
	data->last_updated = 0;
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t set_config_save(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);
	u8 val = data->tmon.control;

	val &= ~MITX_HWMON_REG_TMON_CONTROL_OP;
	val |= MITX_HWMON_REG_TMON_CONTROL_OP_STORE;
	mutex_lock(&data->update_lock);
	bm_mitx_hwmon_write_byte(client, MITX_HWMON_REG_TMON_CONTROL, val);
	mutex_unlock(&data->update_lock);
	return count;
}

#define show_fan_rpm(thing) \
static ssize_t show_fan_rpm_##thing(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int nr = to_sensor_dev_attr(attr)->index; \
	struct bm_mitx_hwmon_data *data = bm_mitx_hwmon_update_device(dev); \
	return sprintf(buf, "%u\n", data->fan[nr].rpm.thing); \
}

show_fan_rpm(value);

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_vmon, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_max, S_IRUGO|S_IWUSR,
	show_vmon_max, set_vmon_max, 0);
static SENSOR_DEVICE_ATTR(in0_min, S_IRUGO|S_IWUSR,
	show_vmon_min, set_vmon_min, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_vmon, NULL, 1);
static SENSOR_DEVICE_ATTR(in1_max, S_IRUGO|S_IWUSR,
	show_vmon_max, set_vmon_max, 1);
static SENSOR_DEVICE_ATTR(in1_min, S_IRUGO|S_IWUSR,
	show_vmon_min, set_vmon_min, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_vmon, NULL, 2);
static SENSOR_DEVICE_ATTR(in2_max, S_IRUGO|S_IWUSR,
	show_vmon_max, set_vmon_max, 2);
static SENSOR_DEVICE_ATTR(in2_min, S_IRUGO|S_IWUSR,
	show_vmon_min, set_vmon_min, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_imon, NULL, 0);
static SENSOR_DEVICE_ATTR(in3_max, S_IRUGO|S_IWUSR,
	show_imon_max, set_imon_max, 0);
static SENSOR_DEVICE_ATTR(in3_min, S_IRUGO|S_IWUSR,
	show_imon_min, set_imon_min, 0);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_imon, NULL, 1);
static SENSOR_DEVICE_ATTR(in4_max, S_IRUGO|S_IWUSR,
	show_imon_max, set_imon_max, 1);
static SENSOR_DEVICE_ATTR(in4_min, S_IRUGO|S_IWUSR,
	show_imon_min, set_imon_min, 1);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_crit, S_IRUGO|S_IWUSR,
	show_temp_crit, set_temp_crit, 0);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO|S_IWUSR,
	show_temp_max, set_temp_max, 0);
static SENSOR_DEVICE_ATTR(temp1_min, S_IRUGO|S_IWUSR,
	show_temp_min, set_temp_min, 0);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp2_crit, S_IRUGO|S_IWUSR,
	show_temp_crit, set_temp_crit, 1);
static SENSOR_DEVICE_ATTR(temp2_max, S_IRUGO|S_IWUSR,
	show_temp_max, set_temp_max, 1);
static SENSOR_DEVICE_ATTR(temp2_min, S_IRUGO|S_IWUSR,
	show_temp_min, set_temp_min, 1);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp3_crit, S_IRUGO|S_IWUSR,
	show_temp_crit, set_temp_crit, 2);
static SENSOR_DEVICE_ATTR(temp3_max, S_IRUGO|S_IWUSR,
	show_temp_max, set_temp_max, 2);
static SENSOR_DEVICE_ATTR(temp3_min, S_IRUGO|S_IWUSR,
	show_temp_min, set_temp_min, 2);
static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan_rpm_value, NULL, 0);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan_rpm_value, NULL, 1);
static SENSOR_DEVICE_ATTR(pwm1, S_IRUGO|S_IWUSR,
	show_pwm, set_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm1_mode, S_IRUGO|S_IWUSR,
	show_pwm_auto, set_pwm_auto, 0);
static SENSOR_DEVICE_ATTR(pwm2, S_IRUGO | S_IWUSR,
	show_pwm, set_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm2_mode, S_IRUGO|S_IWUSR,
	show_pwm_auto, set_pwm_auto, 1);

static SENSOR_DEVICE_ATTR(config_load, S_IWUSR, NULL, set_config_load, 0);
static SENSOR_DEVICE_ATTR(config_save, S_IWUSR, NULL, set_config_save, 0);

static SENSOR_DEVICE_ATTR(table1_size, S_IRUGO|S_IWUSR,
	show_temp_table_size, set_temp_table_size,
	MITX_HWMON_TMON_DEV_INDEX(0, 0));
static SENSOR_DEVICE_ATTR(table1_hyst, S_IRUGO|S_IWUSR,
	show_temp_table_hysteresis, set_temp_table_hysteresis,
	MITX_HWMON_TMON_DEV_INDEX(0, 0));
static SENSOR_DEVICE_ATTR(table1_temp1, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 0));
static SENSOR_DEVICE_ATTR(table1_pwm1, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 0));
static SENSOR_DEVICE_ATTR(table1_temp2, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 1));
static SENSOR_DEVICE_ATTR(table1_pwm2, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 1));
static SENSOR_DEVICE_ATTR(table1_temp3, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 2));
static SENSOR_DEVICE_ATTR(table1_pwm3, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 2));
static SENSOR_DEVICE_ATTR(table1_temp4, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 3));
static SENSOR_DEVICE_ATTR(table1_pwm4, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 3));
static SENSOR_DEVICE_ATTR(table1_temp5, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 4));
static SENSOR_DEVICE_ATTR(table1_pwm5, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 4));
static SENSOR_DEVICE_ATTR(table1_temp6, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 5));
static SENSOR_DEVICE_ATTR(table1_pwm6, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 5));
static SENSOR_DEVICE_ATTR(table1_temp7, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 6));
static SENSOR_DEVICE_ATTR(table1_pwm7, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 6));
static SENSOR_DEVICE_ATTR(table1_temp8, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(0, 7));
static SENSOR_DEVICE_ATTR(table1_pwm8, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(0, 7));
static SENSOR_DEVICE_ATTR(table2_size, S_IRUGO|S_IWUSR,
	show_temp_table_size, set_temp_table_size,
	MITX_HWMON_TMON_DEV_INDEX(1, 0));
static SENSOR_DEVICE_ATTR(table2_hyst, S_IRUGO|S_IWUSR,
	show_temp_table_hysteresis, set_temp_table_hysteresis,
	MITX_HWMON_TMON_DEV_INDEX(1, 0));
static SENSOR_DEVICE_ATTR(table2_temp1, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 0));
static SENSOR_DEVICE_ATTR(table2_pwm1, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 0));
static SENSOR_DEVICE_ATTR(table2_temp2, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 1));
static SENSOR_DEVICE_ATTR(table2_pwm2, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 1));
static SENSOR_DEVICE_ATTR(table2_temp3, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 2));
static SENSOR_DEVICE_ATTR(table2_pwm3, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 2));
static SENSOR_DEVICE_ATTR(table2_temp4, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 3));
static SENSOR_DEVICE_ATTR(table2_pwm4, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 3));
static SENSOR_DEVICE_ATTR(table2_temp5, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 4));
static SENSOR_DEVICE_ATTR(table2_pwm5, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 4));
static SENSOR_DEVICE_ATTR(table2_temp6, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 5));
static SENSOR_DEVICE_ATTR(table2_pwm6, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 5));
static SENSOR_DEVICE_ATTR(table2_temp7, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 6));
static SENSOR_DEVICE_ATTR(table2_pwm7, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 6));
static SENSOR_DEVICE_ATTR(table2_temp8, S_IRUGO|S_IWUSR,
	show_temp_table_value_temp, set_temp_table_value_temp,
	MITX_HWMON_TMON_DEV_INDEX(1, 7));
static SENSOR_DEVICE_ATTR(table2_pwm8, S_IRUGO|S_IWUSR,
	show_temp_table_value_speed, set_temp_table_value_speed,
	MITX_HWMON_TMON_DEV_INDEX(1, 7));

static struct attribute *bm_mitx_hwmon_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_crit.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_min.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_mode.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm2_mode.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	NULL
};

static struct attribute *bm_mitx_hwmon_attributes_extended[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_crit.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_min.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_mode.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm2_mode.dev_attr.attr,
	&sensor_dev_attr_config_load.dev_attr.attr,
	&sensor_dev_attr_config_save.dev_attr.attr,
	&sensor_dev_attr_table1_size.dev_attr.attr,
	&sensor_dev_attr_table1_hyst.dev_attr.attr,
	&sensor_dev_attr_table1_temp1.dev_attr.attr,
	&sensor_dev_attr_table1_pwm1.dev_attr.attr,
	&sensor_dev_attr_table1_temp2.dev_attr.attr,
	&sensor_dev_attr_table1_pwm2.dev_attr.attr,
	&sensor_dev_attr_table1_temp3.dev_attr.attr,
	&sensor_dev_attr_table1_pwm3.dev_attr.attr,
	&sensor_dev_attr_table1_temp4.dev_attr.attr,
	&sensor_dev_attr_table1_pwm4.dev_attr.attr,
	&sensor_dev_attr_table1_temp5.dev_attr.attr,
	&sensor_dev_attr_table1_pwm5.dev_attr.attr,
	&sensor_dev_attr_table1_temp6.dev_attr.attr,
	&sensor_dev_attr_table1_pwm6.dev_attr.attr,
	&sensor_dev_attr_table1_temp7.dev_attr.attr,
	&sensor_dev_attr_table1_pwm7.dev_attr.attr,
	&sensor_dev_attr_table1_temp8.dev_attr.attr,
	&sensor_dev_attr_table1_pwm8.dev_attr.attr,
	&sensor_dev_attr_table2_size.dev_attr.attr,
	&sensor_dev_attr_table2_hyst.dev_attr.attr,
	&sensor_dev_attr_table2_temp1.dev_attr.attr,
	&sensor_dev_attr_table2_pwm1.dev_attr.attr,
	&sensor_dev_attr_table2_temp2.dev_attr.attr,
	&sensor_dev_attr_table2_pwm2.dev_attr.attr,
	&sensor_dev_attr_table2_temp3.dev_attr.attr,
	&sensor_dev_attr_table2_pwm3.dev_attr.attr,
	&sensor_dev_attr_table2_temp4.dev_attr.attr,
	&sensor_dev_attr_table2_pwm4.dev_attr.attr,
	&sensor_dev_attr_table2_temp5.dev_attr.attr,
	&sensor_dev_attr_table2_pwm5.dev_attr.attr,
	&sensor_dev_attr_table2_temp6.dev_attr.attr,
	&sensor_dev_attr_table2_pwm6.dev_attr.attr,
	&sensor_dev_attr_table2_temp7.dev_attr.attr,
	&sensor_dev_attr_table2_pwm7.dev_attr.attr,
	&sensor_dev_attr_table2_temp8.dev_attr.attr,
	&sensor_dev_attr_table2_pwm8.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	NULL
};

#define MITX_HWMON_DEV_ATTR_POWER           18
#define MITX_HWMON_DEV_ATTR_EXT_TMON_CONFIG 18
#define MITX_HWMON_DEV_ATTR_EXT_POWER       56

static struct attribute_group bm_mitx_hwmon_group = {
	.attrs = bm_mitx_hwmon_attributes,
};

static bool bm_mitx_hwmon_device_match(struct i2c_client *client)
{
	u16 vendor_id;
	u16 device_id;

	vendor_id = bm_mitx_hwmon_read_word(client, MITX_HWMON_REG_VEN_ID);
	device_id = bm_mitx_hwmon_read_word(client, MITX_HWMON_REG_DEV_ID);
	return (vendor_id == MITX_HWMON_REG_VEN_ID_TPL) &&
		(device_id == MITX_HWMON_REG_DEV_ID_MITX_HWMON);
}

static int bm_mitx_hwmon_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bm_mitx_hwmon_data *data;
	int err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!bm_mitx_hwmon_device_match(client))
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(struct bm_mitx_hwmon_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	data->caps = bm_mitx_hwmon_read_byte(client, MITX_HWMON_REG_CAP);
	if (extended_attrs) {
		if ((data->caps & MITX_HWMON_REG_CAP_TMON_CONFIG) == 0)
			bm_mitx_hwmon_attributes[MITX_HWMON_DEV_ATTR_EXT_TMON_CONFIG] = NULL;
		bm_mitx_hwmon_group.attrs = bm_mitx_hwmon_attributes_extended;
	}
	if ((data->caps & MITX_HWMON_REG_CAP_POWER) == 0) {
		bm_mitx_hwmon_attributes[extended_attrs ?
					MITX_HWMON_DEV_ATTR_EXT_POWER :
					MITX_HWMON_DEV_ATTR_POWER] = NULL;
	}

	err = sysfs_create_group(&client->dev.kobj, &bm_mitx_hwmon_group);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &bm_mitx_hwmon_group);
	return err;
}

static int bm_mitx_hwmon_remove(struct i2c_client *client)
{
	struct bm_mitx_hwmon_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &bm_mitx_hwmon_group);
	return 0;
}

static const struct i2c_device_id bm_mitx_hwmon_id[] = {
	{ "bm_mitx_hwmon", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bm_mitx_hwmon_id);

static struct i2c_driver bm_mitx_hwmon_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "bm_mitx_hwmon",
	},
	.probe = bm_mitx_hwmon_probe,
	.remove = bm_mitx_hwmon_remove,
	.id_table = bm_mitx_hwmon_id,
};

module_i2c_driver(bm_mitx_hwmon_driver);

module_param(extended_attrs, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(extended_attrs, "Enable extended hwmon attributes");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Baikal-M-based Mini-ITX board hardware monitoring driver");
