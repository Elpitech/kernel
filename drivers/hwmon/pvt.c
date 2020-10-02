// SPDX-License-Identifier: GPL-2.0
/*
 * An hwmon driver for BAIKAL-M PVT Sensors based on
 *
 * Analog Bits. PVT Sensor Datasheet. Version: 2014.07.23
 *
 *  Copyright (C) 2017 Baikal Electronics JSC
 *  Author:
 *      Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/io.h>
#include <linux/arm-smccc.h>

#define DRV_NAME "pvt"
#define DRV_VERSION "2.0.0"

#define BAIKAL_SMC_PVT_ID	0x82000001
#define PVT_READ	0
#define PVT_WRITE	1

/* PVT registers */
#define BK_PVT_CTRL		0x00
#define BK_PVT_DATA		0x04
#define BK_PVT_TTHRES		0x08
#define BK_PVT_VTHRES		0x0C
#define BK_PVT_TTIMEOUT		0x1C
#define BK_PVT_INTR_STAT	0x20
#define BK_PVT_INTR_MASK	0x24
#define BK_PVT_CLR_INTR		0x2C
/* PVT VALID bit reads TIMEOUT */
#define BK_PVT_VALID_TIMEOUT	10000

/* PVT VALUES and MASKS */
#define BK_PVT_CTRL_EN_BIT  0x1
#define BK_PVT_CTRL_TMOD    0x0
#define BK_PVT_CTRL_VMOD    0x2
#define BK_PVT_CTRL_LVTMOD  0b0100
#define BK_PVT_CTRL_HVTMOD  0b1000
#define BK_PVT_CTRL_SVTMOD  0b1100

#define BK_PVT_INTR_MASK_TONLY  0x7F9
#define BK_PVT_INTR_MASK_TVONLY 0x7E1
#define BK_PVT_INTR_MASK_ALL  0x7FF

#define BK_PVT_DATA_MASK    0x3ff
#define BK_PVT_DATA_VALID   (1 << 10)

#define BK_PVT_THRES_HI    0xFFC00
#define BK_PVT_THRES_LO    0x3FF

#define BK_PVT_TTIMEOUT_SET 10000000

#define BK_PVT_INTR_STAT_TTHRES_LO 0x02
#define BK_PVT_INTR_STAT_TTHRES_HI 0x04
#define BK_PVT_INTR_STAT_VTHRES_LO 0x08
#define BK_PVT_INTR_STAT_VTHRES_HI 0x10

/* TEMP limits */
#define TEMP_PVT_MAX 125000
#define TEMP_PVT_MIN -40000
/* Voltage limits */
#define VOLT_PVT_MAX 800
#define VOLT_PVT_MIN 1000


/* coef for transformtion to T,C (10^-3) times 10^6
 * DATA = BK_PVT_DATA [0:9]
 * T =  COEF4 * DATA ^ 4 + COEF3 * DATA ^ 3 + COEF2 * DATA ^ 2 +
 *    + COEF1 * DATA ^ 1 + COEF0
 */
#define COEF4	(-16743)	/* (-1.6743E-11f)  * 10^15 */
#define COEF3	(81542)		/* (8.1542E-08f)   * 10^12 */
#define COEF2	(-182010)	/* (-1.8201E-04f)  * 10^9  */
#define COEF1	(310200)	/* (3.1020E-01f)   * 10^6  */
#define COEF0	(-48380)	/* (-4.8380E+01f)  * 10^3  */

/* coef for transformation T,C (10^-3) to DATA
 * DATA = DCOEF3 * T^3 + DCOEF2 * T ^ 2 + DCOEF1 * T + DCOEF 0
 */

#define DCOEF3  (2617)
#define DCOEF2  (8654)
#define DCOEF1  (3923)
#define DCOEF0  (172)

/*  coef for transformatio to V, mV
 *  DATA = 1865.8 *  VOLTAGE- 1157.2 =>
 *  VOLTAGE = 620 + data * 10000 / 18658;
 */
#define COEF0_V 620
#define COEF1_V 18658

static uint32_t writel_pvt(uint32_t val, uint32_t pvt_id, uint32_t offset)
{

	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_ID, PVT_WRITE, pvt_id, offset,
		      val, 0, 0, 0, &res);

	return res.a0;
}

static uint32_t readl_pvt(uint32_t pvt_id, uint32_t offset)
{

	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_ID, PVT_READ, pvt_id, offset,
		      0, 0, 0, 0, &res);

	return res.a0;
}

struct pvt_hwmon {
	int pvt_id;
	int base;
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
	//OFF PVT
	writel_pvt(0, hwmon->pvt_id, BK_PVT_CTRL);
	//Set timeout of inerupts
	writel_pvt(BK_PVT_TTIMEOUT_SET, hwmon->pvt_id, BK_PVT_TTIMEOUT);
	pr_debug("pvt switch_to_mon_mod and set BK_PVT_TTIMEOUT %d\n",
		 readl_pvt(hwmon->pvt_id, BK_PVT_TTIMEOUT));
	//Mask all interupts
	writel_pvt(BK_PVT_INTR_MASK_TVONLY, hwmon->pvt_id, BK_PVT_INTR_MASK);
	//Switch to last VOLT or Temprature mon_mod
	writel_pvt(((hwmon->mon_mod)<<1), hwmon->pvt_id, BK_PVT_CTRL);
	pr_debug("pvt switch_to_mon_mod and set BK_PVT_CTRL %d\n",
		 readl_pvt(hwmon->pvt_id, BK_PVT_CTRL));
	//ON PVT
	writel_pvt((BK_PVT_CTRL_EN_BIT) | ((hwmon->mon_mod)<<1),
		   hwmon->pvt_id, BK_PVT_CTRL);
}

static int read_valid_datareg(struct pvt_hwmon *hwmon)
{
	register int data, i = 0;

	data = readl_pvt(hwmon->pvt_id, BK_PVT_DATA);
	data = 0;
	while (!(data & (BK_PVT_DATA_VALID))) {
		data = readl_pvt(hwmon->pvt_id, BK_PVT_DATA);
		if (++i == BK_PVT_VALID_TIMEOUT)
			return -EINVAL;
	}

	data &= (BK_PVT_DATA_MASK);
	switch_to_mon_mod(hwmon);

	return data;
}


static void switch_pvt_mod(int pvt_id, long mod)
{
	pr_debug("BK PVT now %x, but need %lx\n",
		 readl_pvt(pvt_id, BK_PVT_CTRL), (unsigned long)mod);
	writel_pvt(0, pvt_id, BK_PVT_CTRL);
	//Set timeout of PVT measurment
	writel_pvt(0, pvt_id, BK_PVT_TTIMEOUT);
	//Mask all interupts
	writel_pvt(BK_PVT_INTR_MASK_ALL, pvt_id, BK_PVT_INTR_MASK);
	writel_pvt(mod, pvt_id, BK_PVT_CTRL);
	writel_pvt(((BK_PVT_CTRL_EN_BIT)|mod), pvt_id, BK_PVT_CTRL);
	pr_debug("BK PVT MOD %x\n", readl_pvt(pvt_id, BK_PVT_CTRL));
}

static int data2temp(int data)
{
	int temp, temp4, temp3, temp2, temp1, temp0;

	pr_debug("pvt %d and data %d\n", (BK_PVT_DATA_MASK), data);
	/*Dont changer the order of multiplication !!! */
	temp4 = (COEF4) * data / 1000 * data / 1000 * data / 1000 * data / 1000;
	temp3 = (COEF3) * data / 1000 * data / 1000 * data / 1000;
	temp2 = (COEF2) * data / 1000 * data / 1000;
	temp1 = (COEF1) * data / 1000;
	temp0 = (COEF0);
	temp = temp0 + temp1 + temp2 + temp3 + temp4;
	pr_debug("BK PVT temp  %d = %d + %d + %d + %d + %d\n",
		 temp, temp4, temp3, temp2, temp1, temp0);

	return temp;
}

static irqreturn_t pvt_hwmon_irq(int irq, void *data)
{
	long val;
	struct pvt_hwmon *hwmon = data;

	val = readl_pvt(hwmon->pvt_id, BK_PVT_INTR_STAT);
	if (BK_PVT_INTR_STAT_TTHRES_LO & val)
		pr_info("PVT WARNING Lo Temperature\n");
	if (BK_PVT_INTR_STAT_TTHRES_HI & val)
		pr_info("PVT WARNING Hi Temperature\n");
	if (BK_PVT_INTR_STAT_VTHRES_LO & val)
		pr_info("PVT WARNING Lo Voltage\n");
	if (BK_PVT_INTR_STAT_VTHRES_HI & val)
		pr_info("PVT WARNING Hi Voltage\n");
	val = readl_pvt(hwmon->pvt_id, BK_PVT_CLR_INTR);
	complete(&hwmon->read_completion);

	return IRQ_HANDLED;
}

static ssize_t pvt_show_name(struct device *dev,
			     struct device_attribute *dev_attr, char *buf)
{
	return sprintf(buf, "pvt-baikal\n");
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
	if (err)
		return err;
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
	switch_pvt_mod(hwmon->pvt_id, BK_PVT_CTRL_TMOD);
	data = read_valid_datareg(hwmon);
	temp = data2temp(data);
	hwmon->temp = temp;
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", temp);
}

static int temp2data(int temp)
{
	int data3, data2, data1, data0, data;

	if (temp > TEMP_PVT_MAX)
		temp = TEMP_PVT_MAX;
	if (temp < TEMP_PVT_MIN)
		temp = TEMP_PVT_MIN;

	/*Dont changer the order of multiplication !!! */
	data3 = DCOEF3 * temp / 1000000 * temp / 1000000 * temp / 100000;
	data2 = DCOEF2 * temp / 1000000 * temp / 1000000;
	data1 = DCOEF1 * temp / 1000000;
	data0 = DCOEF0;
	data = data0 + data1 + data2 + data3;

	pr_debug("pvt %d and data %d\n", (BK_PVT_DATA_MASK), data);

	return data;
}

static int data2volt(int data)
{
	/* DATA = 1865.8 *  VOLTAGE- 1157.2 */
	return (COEF0_V + (data * 10000) / COEF1_V);
}

int volt2data(int volt)
{
	if (volt > VOLT_PVT_MAX)
		volt = VOLT_PVT_MAX;
	if (volt < VOLT_PVT_MIN)
		volt = VOLT_PVT_MIN;
	/* DATA = 1865.8 *  VOLTAGE- 1157.2 */

	return (18658 * volt / 10000 - 1157);
}

static ssize_t set_temp_min(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned int val, data;
	long temp;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &temp);
	if (err)
		return err;
	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->pvt_id, BK_PVT_TTHRES);
	val = temp2data(temp);
	data = (data & BK_PVT_THRES_HI) + (BK_PVT_THRES_LO & val);
	writel_pvt(data, hwmon->pvt_id, BK_PVT_TTHRES);
	mutex_unlock(&hwmon->lock);

	return count;
}

static ssize_t set_temp_max(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned int val, data;
	long temp;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &temp);
	if (err)
		return err;
	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->pvt_id, BK_PVT_TTHRES);
	val = temp2data(temp);
	data = ((val<<10) & BK_PVT_THRES_HI) + (BK_PVT_THRES_LO & data);
	writel_pvt(data, hwmon->pvt_id, BK_PVT_TTHRES);
	mutex_unlock(&hwmon->lock);

	return count;
}

static ssize_t set_volt_min(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned int val, data;
	long volt;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &volt);
	if (err)
		return err;
	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->pvt_id, BK_PVT_VTHRES);
	val = volt2data(volt);
	data = (data & BK_PVT_THRES_HI) + (BK_PVT_THRES_LO & val);
	writel_pvt(data, hwmon->pvt_id, BK_PVT_VTHRES);
	mutex_unlock(&hwmon->lock);

	return count;
}

static ssize_t set_volt_max(struct device *dev,
			    struct device_attribute *devattr,
			    const char *buf, size_t count)
{
	unsigned int val, data;
	long volt;
	int err;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	err = kstrtol(buf, 10, &volt);
	if (err)
		return err;
	mutex_lock(&hwmon->lock);
	data = readl_pvt(hwmon->pvt_id, BK_PVT_VTHRES);
	val = volt2data(volt);
	pr_debug("pvt set volt max %ld and val %x\n", volt, val);
	data = ((val<<10) & BK_PVT_THRES_HI) + (BK_PVT_THRES_LO & data);
	writel_pvt(data, hwmon->pvt_id, BK_PVT_VTHRES);
	mutex_unlock(&hwmon->lock);

	return count;
}

static ssize_t pvt_show_temp_min(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	unsigned int val, data;
	int temp;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->pvt_id, BK_PVT_TTHRES);
	data = BK_PVT_THRES_LO & val;
	temp = data2temp(data);
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", temp);
}

static ssize_t pvt_show_temp_max(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	unsigned int val, data;
	int temp;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->pvt_id, BK_PVT_TTHRES);
	data = (BK_PVT_THRES_HI & val) >> 10;
	temp = data2temp(data);
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", temp);
}

static ssize_t pvt_show_volt_min(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	unsigned int val, data;
	int volt;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->pvt_id, BK_PVT_VTHRES);
	data = BK_PVT_THRES_LO & val;
	volt = data2volt(data);
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", volt);
}


static ssize_t pvt_show_volt_max(struct device *dev,
				 struct device_attribute *devattr,
				 char *buf)
{
	unsigned int val, data;
	int volt;
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);

	mutex_lock(&hwmon->lock);
	val = readl_pvt(hwmon->pvt_id, BK_PVT_VTHRES);
	data = (BK_PVT_THRES_HI & val) >> 10;
	volt = data2volt(data);
	mutex_unlock(&hwmon->lock);

	return sprintf(buf, "%d\n", volt);
}

static ssize_t pvt_show_voltage(struct device *dev, struct device_attribute *da,
				char *buf)
{
	struct pvt_hwmon *hwmon = dev_get_drvdata(dev);
	int data, volt;

	mutex_lock(&hwmon->lock);
	switch_pvt_mod(hwmon->pvt_id, BK_PVT_CTRL_VMOD);
	data = read_valid_datareg(hwmon);
	/* Don't change the order of multiplication!!! */
	volt = data2volt(data);
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
	switch_pvt_mod(hwmon->pvt_id, BK_PVT_CTRL_LVTMOD);
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
	switch_pvt_mod(hwmon->pvt_id, BK_PVT_CTRL_HVTMOD);
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
	switch_pvt_mod(hwmon->pvt_id, BK_PVT_CTRL_SVTMOD);
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
static DEVICE_ATTR(temp1_min, S_IWUSR | S_IRUGO, pvt_show_temp_min,
		   set_temp_min);
static DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO, pvt_show_temp_max,
		   set_temp_max);
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
	.attrs = pvt_attrs,
};


static int pvt_probe(struct platform_device *pdev)
{
	int ret;
	struct pvt_hwmon *hwmon;
	struct resource *mem;
	struct device_node *np = pdev->dev.of_node;

	pr_debug("driver pvt_probe\n");
	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hwmon->base = (int)mem->start;//devm_ioremap_resource(&pdev->dev, mem);

	/* Set PVT ID for secure monitor calls */
	of_property_read_u32(np, "pvt_id", &(hwmon->pvt_id));

	hwmon->cell = mfd_get_cell(pdev);
	hwmon->irq = platform_get_irq(pdev, 0);

	if (hwmon->irq < 0) {
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n",
			hwmon->irq);
		return hwmon->irq;
	}

	init_completion(&hwmon->read_completion);
	mutex_init(&hwmon->lock);

	//Mask all interupts except TTRES_HILO
	writel_pvt(BK_PVT_INTR_MASK_TVONLY, hwmon->pvt_id, BK_PVT_INTR_MASK);
	pr_debug("pvt_probe BK_PVT_INTR_MASK %x\n",
		 readl_pvt(hwmon->pvt_id, BK_PVT_INTR_MASK));

	//Set timeout of PVT measurment
	writel_pvt(BK_PVT_TTIMEOUT_SET, hwmon->pvt_id, BK_PVT_TTIMEOUT);
	pr_debug("pvt_probe BK_PVT_TTIMEOUT %d\n",
		 readl_pvt(hwmon->pvt_id, BK_PVT_TTIMEOUT));

	platform_set_drvdata(pdev, hwmon);
	ret = devm_request_irq(&pdev->dev, hwmon->irq, pvt_hwmon_irq, 0,
			       pdev->name, hwmon);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &pvt_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs group: %d\n", ret);
		return ret;
	}

	hwmon->hwmon = hwmon_device_register_with_info(&pdev->dev, pdev->name,
						       hwmon, NULL, NULL);
	if (IS_ERR(hwmon->hwmon)) {
		ret = PTR_ERR(hwmon->hwmon);
		goto err_remove_file;
	}

	//Set Monitoring mod for temperature
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
	{ .compatible = "baikal,pvt" },
	{ },
};

static struct platform_driver pvt_hwmon_driver = {
	.probe	  = pvt_probe,
	.remove	  = pvt_remove,
	.driver = {
		.name = "pvt-hwmon",
		.of_match_table = of_match_ptr(pvt_dt_match),
	},
};

module_platform_driver(pvt_hwmon_driver);

MODULE_DESCRIPTION("PVT BAIKAL driver");
MODULE_AUTHOR("Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pvt-hwmon");
