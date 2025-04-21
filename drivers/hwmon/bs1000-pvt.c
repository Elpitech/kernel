// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2024-2025 BAIKAL ELECTRONICS, JSC
 *
 * Baikal BE-S1000 SoC Process, Voltage, Temperature sensor driver.
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/firmware/baikal/baikal-smc.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define BS1000_SCP_CMD_PVT_THR_SET	0x8
#define BS1000_SCP_CMD_PVT_THR_GET	0x9
#define BS1000_SCP_CMD_PVT_READ		0xA

#define BS1000_SCP_PVT_THRLO_MASK	GENMASK(15, 0)
#define BS1000_SCP_PVT_THRHI_MASK	GENMASK(31, 16)

#define BAIKAL_MBOX_SMS_PVT_MODE	BIT(23)		/* 0 - Temperature, 1 - Voltage */
#define BAIKAL_MBOX_SMS_THR_CRIT	BIT(22)		/* 0 - Yellow, 1 - Red */
#define BAIKAL_MBOX_SMS_THR_MAX		BIT(21)		/* 0 - min, 1 - max */
#define BAIKAL_MBOX_SMS_PVT_ID		GENMASK(20, 16)
#define BAIKAL_MBOX_SMS_PVT_DATA	GENMASK(15, 0)

#define BS1000_PVT_PER_CHIP	27
#define BS1000_PVT_SCP_START	0
#define BS1000_PVT_CPU_START	1
#define BS1000_PVT_PCIE_START	13
#define BS1000_PVT_DDR_START	18
#define BS1000_PVT_MIN_START	24
#define BS1000_PVT_MAX_START	25
#define BS1000_PVT_AVG_START	26

#define BS1000_PVT_TEMP_CHS	1
#define BS1000_PVT_VOLT_CHS	4

#define PVT_HWMON_NAME_LEN	14

#define PVT_THR_MIN	0
#define PVT_THR_MAX	GENMASK(15, 0)

#define PVT_THERMAL_POLLING_DELAY	5000

#define PVT_THERMAL_CRIT	85000	/* 85 degrees default critical temperature limit */
#define PVT_THERMAL_CRIT_HYST	2000

enum pvt_mode {
	PVT_TEMP = 0,
	PVT_VOLT = 1,
	PVT_LVT  = 2,
	PVT_ULVT = 4,
	PVT_SVT  = 6
};

struct pvt_dev;

struct pvt_hwmon {
	char name[PVT_HWMON_NAME_LEN];
	struct device *dev;
	struct mutex mutex;
	struct pvt_dev *pvt;
	struct thermal_zone_device *tzd;
	bool enabled;
};

struct pvt_dev {
//vvv	struct mbox_chan *mbox;
	struct pvt_hwmon hwmon[BS1000_PVT_PER_CHIP];
};

struct pvt_info {
	unsigned int channel;
	const char *label;
	enum pvt_mode mode;
	long (*convert)(long value, bool to_pvt);
};

static const struct pvt_format {
	unsigned int start_id;
	const char *format;
} pvt_formats[] = {
	{ .start_id = BS1000_PVT_SCP_START,  .format = "pvt%1uscp"        },
	{ .start_id = BS1000_PVT_CPU_START,  .format = "pvt%1ucluster%1u" },
	{ .start_id = BS1000_PVT_PCIE_START, .format = "pvt%1upcie%1u"    },
	{ .start_id = BS1000_PVT_DDR_START,  .format = "pvt%1uddr%1u"     },
	{ .start_id = BS1000_PVT_MIN_START,  .format = "pvt%1umin"        },
	{ .start_id = BS1000_PVT_MAX_START,  .format = "pvt%1umax"        },
	{ .start_id = BS1000_PVT_AVG_START,  .format = "pvt%1uavg"        }
};

static long pvt_temp_convert(long data, bool to_pvt)
{
	return to_pvt ? data / 10 : data * 10;
}

static const struct pvt_info pvt_temp_info[] = {
	{ .channel = 0, .label = "Temperature", .mode = PVT_TEMP, .convert = pvt_temp_convert }
};

static const struct pvt_info pvt_volt_info[] = {
	{ .channel = 0, .label = "Voltage",      .mode = PVT_VOLT, .convert = NULL },
	{ .channel = 1, .label = "Low-Vt",       .mode = PVT_LVT,  .convert = NULL },
	{ .channel = 2, .label = "Ultra-Low-Vt", .mode = PVT_ULVT, .convert = NULL },
	{ .channel = 3, .label = "Standard-Vt",  .mode = PVT_SVT,  .convert = NULL }
};

static int pvt_read_data(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			 long *val)
{
	struct arm_smccc_res res;
	int chip, id;

	chip = dev_to_node(hwmon->dev);
	if (chip == NUMA_NO_NODE)
		chip = 0;
	id = (hwmon - hwmon->pvt->hwmon) + chip * BS1000_PVT_PER_CHIP;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_READ,
		      id, info->mode, 0, 0, 0, 0, &res);
	if ((long)(res.a0) < 0)
		return res.a0;

	*val = res.a0;
	if (info->convert)
		*val = info->convert(*val, false);

	return 0;
}

static int pvt_read_limit(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			  bool is_low, long *val)
{
	struct arm_smccc_res res;
	u32 data;
	int chip, id;

	chip = dev_to_node(hwmon->dev);
	if (chip == NUMA_NO_NODE)
		chip = 0;
	id = (hwmon - hwmon->pvt->hwmon) + chip * BS1000_PVT_PER_CHIP;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_GET,
		      id, info->mode, 0, 0, 0, 0, &res);
	if ((long)(res.a0) < 0)
		return res.a0;

	data = res.a0;
	if (is_low)
		*val = FIELD_GET(BS1000_SCP_PVT_THRLO_MASK, data);
	else
		*val = FIELD_GET(BS1000_SCP_PVT_THRHI_MASK, data);
	if (info->convert)
		*val = info->convert(*val, false);

	return 0;
}

static int pvt_write_limit(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			   bool is_low, long val)
{
	struct arm_smccc_res res;
	int chip, id;
	u32 data, limit;
	int err;

	chip = dev_to_node(hwmon->dev);
	if (chip == NUMA_NO_NODE)
		chip = 0;
	id = (hwmon - hwmon->pvt->hwmon) + chip * BS1000_PVT_PER_CHIP;

	if (info->convert)
		val = info->convert(val, true);
	val = clamp_val(val, PVT_THR_MIN, PVT_THR_MAX);

	err = mutex_lock_interruptible(&hwmon->mutex);
	if (err)
		return err;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_GET,
		      id, info->mode, 0, 0, 0, 0, &res);
	if ((long)(res.a0) < 0) {
		err = res.a0;
		goto out;
	}

	data = res.a0;
	/* Make sure the upper and lower ranges don't intersect. */
	if (is_low) {
		if (val) {
			limit = FIELD_GET(BS1000_SCP_PVT_THRHI_MASK, data);
			if (limit)
				val = clamp_val(val, PVT_THR_MIN, limit);
		}
		data &= ~BS1000_SCP_PVT_THRLO_MASK;
		data |= FIELD_PREP(BS1000_SCP_PVT_THRLO_MASK, val);
	}
	else {
		if (val) {
			limit = FIELD_GET(BS1000_SCP_PVT_THRLO_MASK, data);
			if (limit)
				val = clamp_val(val, limit, PVT_THR_MAX);
		}
		data &= ~BS1000_SCP_PVT_THRHI_MASK;
		data |= FIELD_PREP(BS1000_SCP_PVT_THRHI_MASK, val);
	}
	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, BS1000_SCP_CMD_PVT_THR_SET,
		      id, info->mode, data, 0, 0, 0, &res);
	err = res.a0;

out:
	mutex_unlock(&hwmon->mutex);
	return err;
}

static int pvt_read_alarm(struct pvt_hwmon *hwmon, const struct pvt_info *info,
			  bool is_low, long *val)
{
	long limit, data;
	int err;

	err = pvt_read_limit(hwmon, info, is_low, &limit);
	if (err)
		return err;

	if (limit == 0) {
		*val = 0;
		return 0;
	}

	err = pvt_read_data(hwmon, info, &data);
	if (err)
		return err;

	if (is_low)
		*val = (data < limit);
	else
		*val = (data > limit);

	return 0;
}

static inline bool pvt_hwmon_channel_is_valid(enum hwmon_sensor_types type,
					      int ch)
{
	switch (type) {
	case hwmon_temp:
		if (ch < 0 || ch >= BS1000_PVT_TEMP_CHS)
			return false;
		break;
	case hwmon_in:
		if (ch < 0 || ch >= BS1000_PVT_VOLT_CHS)
			return false;
		break;
	default:
		break;
	}

	/* The rest of the types are independent from the channel number. */
	return true;
}

static umode_t pvt_hwmon_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int ch)
{
	if (!pvt_hwmon_channel_is_valid(type, ch))
		return 0;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_type:
		case hwmon_temp_label:
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
			return 0444;
		case hwmon_temp_min:
		case hwmon_temp_max:
			return 0644;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			return 0444;
		case hwmon_in_min:
		case hwmon_in_max:
			if (ch == 0)
				return 0644;
			break;
		case hwmon_in_min_alarm:
		case hwmon_in_max_alarm:
			if (ch == 0)
				return 0444;
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int pvt_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int ch, long *val)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return pvt_read_data(hwmon, &pvt_temp_info[ch], val);
		case hwmon_temp_type:
			*val = 1;
			return 0;
		case hwmon_temp_min:
			return pvt_read_limit(hwmon, &pvt_temp_info[ch], true,
					      val);
		case hwmon_temp_max:
			return pvt_read_limit(hwmon, &pvt_temp_info[ch], false,
					      val);
		case hwmon_temp_min_alarm:
			return pvt_read_alarm(hwmon, &pvt_temp_info[ch], true,
					      val);
		case hwmon_temp_max_alarm:
			return pvt_read_alarm(hwmon, &pvt_temp_info[ch], false,
					      val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return pvt_read_data(hwmon, &pvt_volt_info[ch], val);
		case hwmon_in_min:
			return pvt_read_limit(hwmon, &pvt_volt_info[ch], true,
					      val);
		case hwmon_in_max:
			return pvt_read_limit(hwmon, &pvt_volt_info[ch], false,
					      val);
		case hwmon_in_min_alarm:
			return pvt_read_alarm(hwmon, &pvt_volt_info[ch], true,
					      val);
		case hwmon_in_max_alarm:
			return pvt_read_alarm(hwmon, &pvt_volt_info[ch], false,
					      val);
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int pvt_hwmon_read_string(struct device *dev,
				 enum hwmon_sensor_types type,
				 u32 attr, int ch, const char **str)
{
	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_label:
			*str = pvt_temp_info[ch].label;
			return 0;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_label:
			*str = pvt_volt_info[ch].label;
			return 0;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int pvt_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int ch, long val)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	if (!pvt_hwmon_channel_is_valid(type, ch))
		return -EINVAL;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_min:
			return pvt_write_limit(hwmon, &pvt_temp_info[ch], true,
					       val);
		case hwmon_temp_max:
			return pvt_write_limit(hwmon, &pvt_temp_info[ch], false,
					       val);
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_min:
			return pvt_write_limit(hwmon, &pvt_volt_info[ch], true,
					       val);
		case hwmon_in_max:
			return pvt_write_limit(hwmon, &pvt_volt_info[ch], false,
					       val);
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops pvt_hwmon_ops = {
	.is_visible = pvt_hwmon_is_visible,
	.read = pvt_hwmon_read,
	.read_string = pvt_hwmon_read_string,
	.write = pvt_hwmon_write
};

static const struct hwmon_channel_info * const pvt_channel_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_TYPE | HWMON_T_LABEL |
			   HWMON_T_MIN | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL |
			   HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX | HWMON_I_MAX_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info pvt_hwmon_info = {
	.ops = &pvt_hwmon_ops,
	.info = pvt_channel_info
};

static const struct hwmon_channel_info * const pvt_misc_channel_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info pvt_hwmon_misc_info = {
	.ops = &pvt_hwmon_ops,
	.info = pvt_misc_channel_info
};

#define thermal_zone_device_priv(tzd)	(tzd->devdata)

static int pvt_thermal_get_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pvt_hwmon *hwmon = thermal_zone_device_priv(tzd);
	long t;
	int err;

	err = pvt_read_data(hwmon, &pvt_temp_info[0], &t);
	if (err)
		return err;

	*temp = t;

	return 0;
}

static int pvt_thermal_set_trips(struct thermal_zone_device *tzd,
				 int low, int high)
{
	struct pvt_hwmon *hwmon = thermal_zone_device_priv(tzd);
	int err;

	err = pvt_write_limit(hwmon, &pvt_temp_info[0], true, low);
	if (err)
		return err;
	return pvt_write_limit(hwmon, &pvt_temp_info[0], false, high);
}

static struct thermal_zone_device_ops pvt_thermal_ops = {
	.get_temp = pvt_thermal_get_temp,
	.set_trips = pvt_thermal_set_trips,
};

static const struct thermal_trip pvt_trips[] = {
	{
		.type = THERMAL_TRIP_CRITICAL,
		.temperature = PVT_THERMAL_CRIT,
		.hysteresis = PVT_THERMAL_CRIT_HYST,
	},
};

static void pvt_thermal_unregister(void *data)
{
	struct thermal_zone_device *tzd = data;

	thermal_zone_device_disable(tzd);
	thermal_zone_device_unregister(tzd);
}

static struct thermal_zone_device *pvt_thermal_register(struct pvt_hwmon *hwmon)
{
	struct thermal_zone_device *tzd;
	struct thermal_zone_params *tzp;
	struct thermal_trip *trips;
	int mask = GENMASK_ULL((ARRAY_SIZE(pvt_trips)) - 1, 0);
	int err;

	trips = devm_kmemdup(hwmon->dev, pvt_trips, sizeof(pvt_trips),
			     GFP_KERNEL);
	if (!trips)
		return ERR_PTR(-ENOMEM);

	tzp = devm_kzalloc(hwmon->dev, sizeof(*tzp), GFP_KERNEL);
	if (!tzp) {
		devm_kfree(hwmon->dev, trips);
		return ERR_PTR(-ENOMEM);
	}
	tzp->no_hwmon = true;
	tzp->slope = 1;
	tzp->offset = 0;

	tzd = thermal_zone_device_register_with_trips(hwmon->name,
		trips, ARRAY_SIZE(pvt_trips), mask, hwmon,
		&pvt_thermal_ops, tzp, 0, PVT_THERMAL_POLLING_DELAY);
	if (IS_ERR(tzd)) {
		devm_kfree(hwmon->dev, tzp);
		devm_kfree(hwmon->dev, trips);
		return tzd;
	}
	err = thermal_zone_device_enable(tzd);
	if (err)
		goto out_unregister;
	err = devm_add_action(hwmon->dev, pvt_thermal_unregister, tzd);
	if (err)
		goto out_disable;
	return tzd;

out_disable:
	thermal_zone_device_disable(tzd);
out_unregister:
	thermal_zone_device_unregister(tzd);
	devm_kfree(hwmon->dev, tzp);
	devm_kfree(hwmon->dev, trips);
	return ERR_PTR(err);
}

static int pvt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pvt_dev *pvt;
	struct fwnode_handle *child;
	int chip, format_idx = 0;

	chip = dev_to_node(dev);
	if (chip == NUMA_NO_NODE)
		chip = 0;

	pvt = devm_kzalloc(dev, sizeof(*pvt), GFP_KERNEL);
	if (!pvt)
		return -ENOMEM;

	platform_set_drvdata(pdev, pvt);

	fwnode_for_each_available_child_node(dev_fwnode(dev), child) {
		u32 id;
		struct pvt_hwmon *hwmon;
		const struct hwmon_chip_info *info;

		if (fwnode_property_read_u32(child, "reg", &id)) {
			dev_warn(dev, "Couldn't get child node id - skipped\n");
			continue;
		}

		if (id >= BS1000_PVT_PER_CHIP) {
			dev_warn(dev, "Wrong child node id (%u) - skipped \n", id);
			continue;
		}

		hwmon = &pvt->hwmon[id];
		info = (id >= BS1000_PVT_MIN_START) ? &pvt_hwmon_misc_info :
						      &pvt_hwmon_info;
		devm_mutex_init(dev, &hwmon->mutex);
		hwmon->pvt = pvt;
		while (format_idx < ARRAY_SIZE(pvt_formats) - 1 &&
		    id >= pvt_formats[format_idx + 1].start_id) {
			++format_idx;
		}
		snprintf(hwmon->name, sizeof(hwmon->name),
			 pvt_formats[format_idx].format, chip,
			 id - pvt_formats[format_idx].start_id);
		hwmon->dev = devm_hwmon_device_register_with_info(dev,
			hwmon->name, hwmon, info, NULL);
		if (IS_ERR(hwmon->dev)) {
			dev_warn(dev, "Couldn't create %s hwmon device (%ld)\n",
				 hwmon->name, PTR_ERR(hwmon->dev));
			continue;
		}
		hwmon->enabled = true;

		if (id < BS1000_PVT_MIN_START) {
			if (__is_defined(CONFIG_THERMAL_OF) && acpi_disabled) {
				hwmon->dev->of_node = to_of_node(child);
				hwmon->tzd = devm_thermal_of_zone_register(
					hwmon->dev, 0, hwmon, &pvt_thermal_ops);
			}
			else {
				hwmon->tzd = pvt_thermal_register(hwmon);
			}
			if (IS_ERR(hwmon->tzd)) {
				dev_warn(hwmon->dev,
					 "Couldn't register to thermal (%ld)\n",
					 PTR_ERR(hwmon->tzd));
			}
		}
	}

	return 0;
}

static int pvt_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id pvt_of_match[] = {
	{ .compatible = "baikal,bs1000-xcp-pvt" },
	{ }
};
MODULE_DEVICE_TABLE(of, pvt_of_match);

static struct platform_driver pvt_driver = {
	.probe  = pvt_probe,
	.remove = pvt_remove,
	.driver = {
		.name = "bs1000-pvt",
		.of_match_table = pvt_of_match
	}
};

module_platform_driver(pvt_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 SoC PVT driver");
MODULE_LICENSE("GPL v2");
