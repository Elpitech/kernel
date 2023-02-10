// SPDX-License-Identifier: GPL-2.0+
/*
 * An hwmon driver for Baikal PVT sensors based on Analog Bits.
 * PVT Sensor Datasheet version: 2014.07.23
 *
 * Copyright (C) 2017-2022 Baikal Electronics, JSC
 * Author: Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/io.h>
#include <linux/arm-smccc.h>

#define BAIKAL_SMC_PVT_ID	0x82000001
#define PVT_READ		0
#define PVT_WRITE		1

/* "baikal,bm1000-pvt" base addresses */
#define MMCA57_0_PVT_BASE	0x28200000
#define MMCA57_1_PVT_BASE	0xc200000
#define MMCA57_2_PVT_BASE	0xa200000
#define MMCA57_3_PVT_BASE	0x26200000
#define MMMALI_PVT_BASE		0x2a060000

/* "baikal,bs1000-pvt" base addresses */
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

/* PVT registers */
#define PVT_CTRL		0x00
#define PVT_DATA		0x04
#define PVT_TTHRES		0x08
#define PVT_VTHRES		0x0c
#define PVT_LTHRES		0x10
#define PVT_ULTHRES		0x14
#define PVT_STHRES		0x18
#define PVT_TTIMEOUT		0x1c
#define PVT_INTR_STAT		0x20
#define PVT_INTR_MASK		0x24
#define PVT_RAW_INTR_STAT	0x28
#define PVT_CLR_INTR		0x2c
#define PVT_CLKCH_CTL		0x40
/* PVT VALID bit reads timeout */
#define PVT_VALID_TIMEOUT	10000

/* PVT values and masks */
#define PVT_CTRL_EN		BIT(0)
#define PVT_CTRL_TMOD		0x0
#define PVT_CTRL_VMOD		0x2
#define PVT_CTRL_LVTMOD		0b0100
#define PVT_CTRL_HVTMOD		0b1000
#define PVT_CTRL_SVTMOD		0b1100

#define PVT_INTR_MASK_TVONLY	0x7e1
#define PVT_INTR_MASK_ALL	0x7ff

#define PVT_DATA_MASK		0x3ff
#define PVT_DATA_VALID		BIT(10)

#define PVT_THRES_HI		0xffc00
#define PVT_THRES_LO		0x3ff

#define PVT_TTIMEOUT_SET	10000000

#define PVT_INTR_STAT_TTHRES_LO	BIT(1)
#define PVT_INTR_STAT_TTHRES_HI	BIT(2)
#define PVT_INTR_STAT_VTHRES_LO	BIT(3)
#define PVT_INTR_STAT_VTHRES_HI	BIT(4)

/* Temperature limits */
#define TEMP_PVT_MAX		125000
#define TEMP_PVT_MIN		-40000
#define TEMP_PVT_WARN		67000
#define TEMP_PVT_REBOOT		75000

/* Voltage limits */
#define VOLT_PVT_MIN		800
#define VOLT_PVT_MAX		1000

#define MAX_POLY_POWER		5

struct pvt_poly_term {
	unsigned deg;
	long coef;
	long divider;
	long divider_leftover;
};

/*
 * struct pvt_poly - PVT data translation polynomial descriptor
 * @total_divider: total data divider
 * @terms: polynomial terms up to a free one
 */
struct pvt_poly {
	long total_divider;
	struct pvt_poly_term terms[MAX_POLY_POWER];
};

enum chips { bm1000, bs1000, chips_number };

static const struct pvt_poly poly_temp_to_N[chips_number] = {
	[bm1000] = {
		.total_divider = 10000,
		.terms = {
			{4,   18322, 10000, 10000},
			{3,    2343, 10000,    10},
			{2,   87018, 10000,    10},
			{1,   39269,  1000,     1},
			{0, 1720400,     1,     1}
		}
	},
	[bs1000] = {
		.total_divider = 10000,
		.terms = {
			{4,   12569, 10000, 10000},
			{3,    2476, 10000,    10},
			{2,   66309, 10000,    10},
			{1,   36384,  1000,     1},
			{0, 2070500,     1,     1}
		}
	}
};

static const struct pvt_poly poly_N_to_temp[chips_number] = {
	[bm1000] = {
		.total_divider = 1,
		.terms = {
			{4,  -16743, 1000, 1},
			{3,   81542, 1000, 1},
			{2, -182010, 1000, 1},
			{1,  310200, 1000, 1},
			{0,  -48380,    1, 1}
		}
	},
	[bs1000] = {
		.total_divider = 1,
		.terms = {
			{4,   16034, 1000, 1},
			{3,   15608, 1000, 1},
			{2, -150890, 1000, 1},
			{1,  334080, 1000, 1},
			{0,  -62861,    1, 1}
		}
	}
};

static const struct pvt_poly poly_volt_to_N[chips_number] = {
	[bm1000] = {
		.total_divider = 10,
		.terms = {
			{1,  18658, 1000, 1},
			{0, -11572,    1, 1}
		}
	},
	[bs1000] = {
		.total_divider = 10,
		.terms = {
			{1, 16757, 1000, 1},
			{0, -8564,    1, 1}
		}
	}
};

static const struct pvt_poly poly_N_to_volt[chips_number] = {
	[bm1000] = {
		.total_divider = 10,
		.terms = {
			{1,    100000, 18658,     1},
			{0, 115720000,     1, 18658}
		}
	},
	[bs1000] = {
		.total_divider = 10,
		.terms = {
			{1,   100000, 16757,     1},
			{0, 85639000,     1, 16757}
		}
	}
};

/*
 * Here is the polynomial calculation function, which performs the
 * redistributed terms calculations. It's pretty straightforward. We walk
 * over each degree term up to the free one, and perform the redistributed
 * multiplication of the term coefficient, its divider (as for the rationale
 * fraction representation), data power and the rational fraction divider
 * leftover. Then all of this is collected in a total sum variable, which
 * value is normalized by the total divider before being returned.
 */
static long pvt_calc_poly(const struct pvt_poly *poly, long data)
{
	const struct pvt_poly_term *term = poly->terms;
	long tmp, ret = 0;
	int deg;

	do {
		tmp = term->coef;
		for (deg = 0; deg < term->deg; ++deg) {
			tmp = mult_frac(tmp, data, term->divider);
		}

		ret += tmp / term->divider_leftover;
	} while ((term++)->deg);

	return ret / poly->total_divider;
}

static uint32_t writel_pvt(uint32_t val, uint32_t pvt_base, uint32_t offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_ID, PVT_WRITE, pvt_base, offset, val,
		      0, 0, 0, &res);

	return res.a0;
}

static uint32_t readl_pvt(uint32_t pvt_base, uint32_t offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_ID, PVT_READ, pvt_base, offset,
		      0, 0, 0, 0, &res);

	return res.a0;
}

struct pvt_hwmon {
	int base;
	enum chips type_cpu;
	int irq;
	const struct mfd_cell *cell;
	struct device *hwmon;
	struct completion read_completion;
	struct mutex lock;
	int temp;
	int volt;
	int svt;
	int hvt;
	int lvt;
	bool mon_mod;
};

static void switch_to_mon_mod(struct pvt_hwmon *hwmon)
{
	/* PVT off */
	writel_pvt(0, hwmon->base, PVT_CTRL);
	/* Set timeout of interrupts */
	writel_pvt(PVT_TTIMEOUT_SET, hwmon->base, PVT_TTIMEOUT);
	pr_debug("PVT switch_to_mon_mod and set PVT_TTIMEOUT %d\n",
		 readl_pvt(hwmon->base, PVT_TTIMEOUT));

	/* Mask all interrupts except TTRES_HILO */
	writel_pvt(PVT_INTR_MASK_TVONLY, hwmon->base, PVT_INTR_MASK);
	/* Switch to last VOLT or temperature mon_mod */
	writel_pvt(hwmon->mon_mod << 1, hwmon->base, PVT_CTRL);
	pr_debug("PVT switch_to_mon_mod and set PVT_CTRL %d\n",
		 readl_pvt(hwmon->base, PVT_CTRL));

	/* PVT on */
	writel_pvt(PVT_CTRL_EN | (hwmon->mon_mod << 1), hwmon->base, PVT_CTRL);
}

static int read_valid_datareg(struct pvt_hwmon *hwmon)
{
	register int data, i = 0;
	data = readl_pvt(hwmon->base, PVT_DATA);
	data = 0;
	while (!(data & PVT_DATA_VALID)) {
		data = readl_pvt(hwmon->base, PVT_DATA);
		if (++i == PVT_VALID_TIMEOUT) {
			return -EINVAL;
		}
	}

	data &= PVT_DATA_MASK;
	switch_to_mon_mod(hwmon);
	return data;
}

static void switch_pvt_mod(int pvt_base, long mod)
{
	pr_debug("PVT now 0x%x but need 0x%lx\n",
		 readl_pvt(pvt_base, PVT_CTRL), (unsigned long)mod);

	writel_pvt(0, pvt_base, PVT_CTRL);
	/* Set timeout of PVT measurement */
	writel_pvt(0, pvt_base, PVT_TTIMEOUT);
	/* Mask all interrupts */
	writel_pvt(PVT_INTR_MASK_ALL, pvt_base, PVT_INTR_MASK);
	writel_pvt(mod, pvt_base, PVT_CTRL);
	writel_pvt(PVT_CTRL_EN | mod, pvt_base, PVT_CTRL);
	pr_debug("PVT MOD 0x%x\n", readl_pvt(pvt_base, PVT_CTRL));
}

static long temp2data(int temp, int tp)
{
	if (temp > TEMP_PVT_MAX) {
		temp = TEMP_PVT_MAX;
	}

	if (temp < TEMP_PVT_MIN) {
		temp = TEMP_PVT_MIN;
	}

	return pvt_calc_poly(&poly_temp_to_N[tp], temp);
}

static long volt2data(int volt, int tp)
{
	if (volt > VOLT_PVT_MAX) {
		volt = VOLT_PVT_MAX;
	}

	if (volt < VOLT_PVT_MIN) {
		volt = VOLT_PVT_MIN;
	}

	return pvt_calc_poly(&poly_volt_to_N[tp], volt);
}

static long data2temp(int data, int tp)
{
	return pvt_calc_poly(&poly_N_to_temp[tp], data);
}

static long data2volt(int data, int tp)
{
	return pvt_calc_poly(&poly_N_to_volt[tp], data);
}

static irqreturn_t pvt_hwmon_irq(int irq, void *data)
{
	long val;
	int tmp, temp;
	struct pvt_hwmon *hwmon = data;

	val = readl_pvt(hwmon->base, PVT_INTR_STAT);
	if (PVT_INTR_STAT_TTHRES_LO & val) {
		printk(KERN_INFO "PVT WARNING Lo temperature\n");
	}

	if (PVT_INTR_STAT_TTHRES_HI & val) {
		switch_pvt_mod(hwmon->base, PVT_CTRL_TMOD);
		tmp = read_valid_datareg(hwmon);
		temp = data2temp(tmp, hwmon->type_cpu);
		printk(KERN_INFO "PVT WARNING Hi temperature %d\n", temp);
		if (temp > TEMP_PVT_REBOOT) {
			printk(KERN_INFO "PVT TOO Hi temperature (%d > %d), pm_power_off!\n",
			       temp, TEMP_PVT_REBOOT);

			pm_power_off();
		}
	}

	if (PVT_INTR_STAT_VTHRES_LO & val) {
		printk(KERN_INFO "PVT WARNING Lo voltage\n");
	}

	if (PVT_INTR_STAT_VTHRES_HI & val) {
		printk(KERN_INFO "PVT WARNING Lo voltage\n");
	}

	val = readl_pvt(hwmon->base, PVT_CLR_INTR);
	complete(&hwmon->read_completion);
	return IRQ_HANDLED;
}

static ssize_t pvt_show_name(struct device *dev,
			     struct device_attribute *dev_attr, char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	return sprintf(buf, "pvt@%x\n", hwmon->base);
}

static ssize_t pvt_show_mon_mod(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hwmon->mon_mod);
}

static ssize_t set_mon_mod(struct device *dev, struct device_attribute *devattr,
			   const char *buf, size_t count)
{
	int err;
	long data;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &data);
	if (err) {
		return err;
	}

	mutex_lock(&hwmon->lock);
	hwmon->mon_mod = data;
	switch_to_mon_mod(hwmon);
	mutex_unlock(&hwmon->lock);
	return count;
}

/* sysfs attributes for hwmon */
static ssize_t pvt_show_temp(struct device *dev, struct device_attribute *da,
			     char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data, temp;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->base, PVT_CTRL_TMOD);
	data = read_valid_datareg(hwmon);
	temp = data2temp(data, hwmon->type_cpu);
	hwmon->temp = temp;
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", temp);
}

static ssize_t set_temp_min(struct device *dev, struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned val, data;
	long temp;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &temp);
	if (err) {
		return err;
	}

	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->base, PVT_TTHRES);
	val = temp2data(temp, hwmon->type_cpu);
	data = (data & PVT_THRES_HI) + (PVT_THRES_LO & val);
	writel_pvt(data, hwmon->base, PVT_TTHRES);
	mutex_unlock(&hwmon->lock);
	return count;
}

static ssize_t set_temp_max(struct device *dev, struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned val, data;
	long temp;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &temp);
	if (err) {
		return err;
	}

	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->base, PVT_TTHRES);
	val = temp2data(temp, hwmon->type_cpu);
	data = ((val << 10) & PVT_THRES_HI) + (PVT_THRES_LO & data);
	writel_pvt(data, hwmon->base, PVT_TTHRES);
	mutex_unlock(&hwmon->lock);
	return count;
}

static ssize_t set_volt_min(struct device *dev, struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned val, data;
	long volt;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &volt);
	if (err) {
		return err;
	}

	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->base, PVT_VTHRES);
	val = volt2data(volt, hwmon->type_cpu);
	data = (data & PVT_THRES_HI) + (PVT_THRES_LO & val);
	writel_pvt(data, hwmon->base, PVT_VTHRES);
	mutex_unlock(&hwmon->lock);
	return count;
}

static ssize_t set_volt_max(struct device *dev, struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned val, data;
	long volt;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &volt);
	if (err) {
		return err;
	}

	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->base, PVT_VTHRES);
	val = volt2data(volt, hwmon->type_cpu);
	pr_debug("PVT set volt max %ld and val 0x%x\n", volt, val);
	data = ((val << 10) & PVT_THRES_HI) + (PVT_THRES_LO & data);
	writel_pvt(data, hwmon->base, PVT_VTHRES);
	mutex_unlock(&hwmon->lock);
	return count;
}

static ssize_t pvt_show_temp_min(struct device *dev, struct device_attribute *devattr,
				 char *buf)
{
	unsigned val, data;
	long temp;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->base, PVT_TTHRES);
	data = PVT_THRES_LO & val;
	temp = data2temp(data, hwmon->type_cpu);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%ld\n", temp);
}

static ssize_t pvt_show_temp_max(struct device *dev, struct device_attribute *devattr,
				 char *buf)
{
	long val, data, temp;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->base, PVT_TTHRES);
	data = (PVT_THRES_HI & val) >> 10;
	temp = data2temp(data, hwmon->type_cpu);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%ld\n", temp);
}

static ssize_t pvt_show_volt_min(struct device *dev, struct device_attribute *devattr,
				 char *buf)
{
	unsigned val;
	long volt, data;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->base, PVT_VTHRES);
	data = PVT_THRES_LO & val;
	volt = data2volt(data, hwmon->type_cpu);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%ld\n", volt);
}

static ssize_t pvt_show_volt_max(struct device *dev, struct device_attribute *devattr,
				 char *buf)
{
	unsigned val, data;
	int volt;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->base, PVT_VTHRES);
	data = (PVT_THRES_HI & val) >> 10;
	volt = data2volt(data, hwmon->type_cpu);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", volt);
}

static ssize_t pvt_show_voltage(struct device *dev, struct device_attribute *da,
				char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data, volt;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->base, PVT_CTRL_VMOD);
	data = read_valid_datareg(hwmon);
	volt = data2volt(data, hwmon->type_cpu);
	hwmon->volt = volt;
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", volt);
}

static ssize_t lvt_show(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->base, PVT_CTRL_LVTMOD);
	data = read_valid_datareg(hwmon);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", data);
}

static ssize_t hvt_show(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->base, PVT_CTRL_HVTMOD);
	data = read_valid_datareg(hwmon);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", data);
}

static ssize_t svt_show(struct device *dev, struct device_attribute *da,
			char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->base, PVT_CTRL_SVTMOD);
	data = read_valid_datareg(hwmon);
	mutex_unlock(&hwmon->lock);
	return sprintf(buf, "%d\n", data);
}

static DEVICE_ATTR(name, S_IRUGO, pvt_show_name, NULL);
static DEVICE_ATTR(temp1_input, S_IRUGO, pvt_show_temp, NULL);
static DEVICE_ATTR(in1_input, S_IRUGO, pvt_show_voltage, NULL);
static DEVICE_ATTR(lvt_input, S_IRUGO, lvt_show, NULL);
static DEVICE_ATTR(hvt_input, S_IRUGO, hvt_show, NULL);
static DEVICE_ATTR(svt_input, S_IRUGO, svt_show, NULL);
static DEVICE_ATTR(temp1_min, S_IWUSR | S_IRUGO, pvt_show_temp_min, set_temp_min);
static DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, pvt_show_temp_max, set_temp_max);
static DEVICE_ATTR(in1_min, S_IWUSR | S_IRUGO, pvt_show_volt_min, set_volt_min);
static DEVICE_ATTR(in1_max, S_IWUSR | S_IRUGO, pvt_show_volt_max, set_volt_max);
static DEVICE_ATTR(mon_mod, S_IWUSR | S_IRUGO, pvt_show_mon_mod, set_mon_mod);

static struct attribute *pvt_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_temp1_input.attr,
	&dev_attr_temp1_min.attr,
	&dev_attr_temp1_max.attr,
	&dev_attr_in1_input.attr,
	&dev_attr_in1_min.attr,
	&dev_attr_in1_max.attr,
	&dev_attr_lvt_input.attr,
	&dev_attr_hvt_input.attr,
	&dev_attr_svt_input.attr,
	&dev_attr_mon_mod.attr,
	NULL
};

static const struct attribute_group pvt_attr_group = {
	.attrs = pvt_attrs
};

static int pvt_probe(struct platform_device *pdev)
{
	int ret, hwmon_type_cpu;
	unsigned val, data;
	struct pvt_hwmon *hwmon;
	struct resource *mem;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon) {
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hwmon->base = (int)mem->start;

	/*
	 * Set PVT ID for secure monitor calls.
	 * TODO: it is a legacy awkward design because
	 *	 secure monitor has to perform the opposite conversion.
	 */
	hwmon_type_cpu = (enum chips)device_get_match_data(&pdev->dev);
	if (hwmon->type_cpu == bm1000 &&
		(hwmon->base == MMCA57_0_PVT_BASE ||
		 hwmon->base == MMCA57_1_PVT_BASE ||
		 hwmon->base == MMCA57_2_PVT_BASE ||
		 hwmon->base == MMCA57_3_PVT_BASE ||
		 hwmon->base == MMMALI_PVT_BASE)) {
		hwmon->type_cpu = bm1000;
	} else if (hwmon_type_cpu == bs1000 &&
		(hwmon->base == CA75_0_PVT_BASE	 ||
		 hwmon->base == CA75_1_PVT_BASE	 ||
		 hwmon->base == CA75_2_PVT_BASE	 ||
		 hwmon->base == CA75_3_PVT_BASE	 ||
		 hwmon->base == CA75_4_PVT_BASE	 ||
		 hwmon->base == CA75_5_PVT_BASE	 ||
		 hwmon->base == CA75_6_PVT_BASE	 ||
		 hwmon->base == CA75_7_PVT_BASE	 ||
		 hwmon->base == CA75_8_PVT_BASE	 ||
		 hwmon->base == CA75_9_PVT_BASE	 ||
		 hwmon->base == CA75_10_PVT_BASE ||
		 hwmon->base == CA75_11_PVT_BASE ||
		 hwmon->base == DDR0_PVT_BASE	 ||
		 hwmon->base == DDR1_PVT_BASE	 ||
		 hwmon->base == DDR2_PVT_BASE	 ||
		 hwmon->base == DDR3_PVT_BASE	 ||
		 hwmon->base == DDR4_PVT_BASE	 ||
		 hwmon->base == DDR5_PVT_BASE	 ||
		 hwmon->base == PCIE0_PVT_BASE	 ||
		 hwmon->base == PCIE1_PVT_BASE	 ||
		 hwmon->base == PCIE2_PVT_BASE	 ||
		 hwmon->base == PCIE3_PVT_BASE	 ||
		 hwmon->base == PCIE4_PVT_BASE)) {
		hwmon->type_cpu = bs1000;
	} else {
		dev_err(&pdev->dev, "invalid base address or failed to get type: 0x%x\n", hwmon->base);
		return -EINVAL;
	}

	hwmon->cell = mfd_get_cell(pdev);
	if (!(hwmon->type_cpu == bs1000 &&
		(hwmon->base == PCIE0_PVT_BASE	 ||
		 hwmon->base == PCIE1_PVT_BASE	 ||
		 hwmon->base == PCIE2_PVT_BASE	 ||
		 hwmon->base == CA75_0_PVT_BASE	 ||
		 hwmon->base == CA75_1_PVT_BASE	 ||
		 hwmon->base == CA75_2_PVT_BASE	 ||
		 hwmon->base == CA75_3_PVT_BASE	 ||
		 hwmon->base == CA75_4_PVT_BASE	 ||
		 hwmon->base == CA75_5_PVT_BASE	 ||
		 hwmon->base == CA75_6_PVT_BASE	 ||
		 hwmon->base == CA75_7_PVT_BASE	 ||
		 hwmon->base == CA75_8_PVT_BASE	 ||
		 hwmon->base == CA75_9_PVT_BASE	 ||
		 hwmon->base == CA75_10_PVT_BASE ||
		 hwmon->base == CA75_11_PVT_BASE))) {
		hwmon->irq = platform_get_irq(pdev, 0);
		if (hwmon->irq < 0) {
			dev_err(&pdev->dev, "failed to get platform irq: %d\n",
				hwmon->irq);
		}
	}

	init_completion(&hwmon->read_completion);
	mutex_init(&hwmon->lock);

	/* Mask all interrupts except TTRES_HILO */
	writel_pvt(PVT_INTR_MASK_TVONLY, hwmon->base, PVT_INTR_MASK);
	pr_debug("pvt_probe PVT_INTR_MASK 0x%x\n",
		 readl_pvt(hwmon->base, PVT_INTR_MASK));

	/* Set timeout of PVT measurement */
	writel_pvt(PVT_TTIMEOUT_SET, hwmon->base, PVT_TTIMEOUT);
	pr_debug("pvt_probe PVT_TTIMEOUT %d\n",
		 readl_pvt(hwmon->base, PVT_TTIMEOUT));

	platform_set_drvdata(pdev, hwmon);

	/*
	 * TODO: it is workaround for BE-S1000 PCIe[0-2] PVTs.
	 *	 These PVTs generate infinite interrupts. Why?
	 *	 The issue should be researched thoroughly.
	 */
	if (!(hwmon->type_cpu == bs1000 &&
	      (hwmon->base == PCIE0_PVT_BASE  ||
	       hwmon->base == PCIE1_PVT_BASE  ||
	       hwmon->base == PCIE2_PVT_BASE  ||
	       hwmon->base == CA75_0_PVT_BASE ||
	       hwmon->base == CA75_1_PVT_BASE ||
	       hwmon->base == CA75_2_PVT_BASE ||
	       hwmon->base == CA75_3_PVT_BASE ||
	       hwmon->base == CA75_4_PVT_BASE ||
	       hwmon->base == CA75_5_PVT_BASE ||
	       hwmon->base == CA75_6_PVT_BASE ||
	       hwmon->base == CA75_7_PVT_BASE ||
	       hwmon->base == CA75_8_PVT_BASE ||
	       hwmon->base == CA75_9_PVT_BASE ||
	       hwmon->base == CA75_10_PVT_BASE ||
	       hwmon->base == CA75_11_PVT_BASE))) {
		ret = devm_request_irq(&pdev->dev, hwmon->irq, pvt_hwmon_irq, 0,
					pdev->name, hwmon);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq: %d\n", ret);
			/* TODO: ncdofail if no IRQ return ret */
		}
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &pvt_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs group: %d\n", ret);
		return ret;
	}

	hwmon->hwmon = hwmon_device_register_with_info(&pdev->dev, pdev->name,
							hwmon, NULL, NULL);
	if (IS_ERR(hwmon->hwmon)) {
		ret = PTR_ERR(hwmon->hwmon);
		goto err_remove_file;
	}

	/* Set WARN temperature */
	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->base, PVT_TTHRES);
	val = temp2data(TEMP_PVT_WARN, hwmon->type_cpu);
	data = ((val << 10) & PVT_THRES_HI) + (PVT_THRES_LO & data);
	writel_pvt(data, hwmon->base, PVT_TTHRES);
	mutex_unlock(&hwmon->lock);
	/* Set monitoring mod for temperature */
	hwmon->mon_mod = 0;
	switch_to_mon_mod(hwmon);
	pr_debug("pvt_probe hwmon_device_register %d\n", ret);
	return 0;

err_remove_file:
	sysfs_remove_group(&pdev->dev.kobj, &pvt_attr_group);
	return ret;
}

static int pvt_remove(struct platform_device *pdev)
{
	struct pvt_hwmon *hwmon = platform_get_drvdata(pdev);

	hwmon_device_unregister(hwmon->hwmon);
	sysfs_remove_group(&pdev->dev.kobj, &pvt_attr_group);
	return 0;
}

static const struct of_device_id pvt_dt_match[] = {
	{ .compatible = "baikal,bm1000-pvt", .data = (void *)bm1000 },
	{ .compatible = "baikal,bs1000-pvt", .data = (void *)bs1000 },
	{ }
};

static struct platform_driver pvt_hwmon_driver = {
	.probe	= pvt_probe,
	.remove	= pvt_remove,
	.driver	= {
		.name = "pvt-hwmon",
		.of_match_table = of_match_ptr(pvt_dt_match)
	}
};
module_platform_driver(pvt_hwmon_driver);

MODULE_DESCRIPTION("Baikal PVT driver");
MODULE_AUTHOR("Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pvt-hwmon");
