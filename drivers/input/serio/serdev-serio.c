// SPDX-License-Identifier: GPL-2.0-only
/*
 * Serdev Serio driver
 *
 * Copyright (C) 2022 Elpitech
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serio.h>
#include <linux/serdev.h>

struct serdev_serio {
	struct serdev_device *serdev;
	struct serio *serio;
};

static int ss_serio_write(struct serio *serio, unsigned char data)
{
	struct serdev_serio *ss = serio->port_data;
	struct serdev_device *serdev = ss->serdev;

	dev_dbg(&serdev->dev, "ss_write: data %02x\n", data);
	serdev_device_write(serdev, &data, 1, 0);

	return 0;
}

static int ss_receive_buf(struct serdev_device *serdev,
			  const unsigned char *buf, size_t count)
{
	struct serdev_serio *ss = serdev_device_get_drvdata(serdev);
	int ret = count;

	dev_dbg(&serdev->dev, "ss_receive: count %d, data %02x\n", (int)count, *buf);
	while (count--)
		serio_interrupt(ss->serio, *buf++, 0);

	return ret;
}

static const struct serdev_device_ops ss_serdev_ops = {
	.receive_buf    = ss_receive_buf,
	.write_wakeup   = serdev_device_write_wakeup,
};

static int ss_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct device_node *node = dev->of_node;
	struct serdev_serio *ss;
	struct serio *serio;
	u32 speed = 0, proto;
	int ret;

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	ss = devm_kzalloc(dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;
	ss->serdev = serdev;
	ss->serio = serio;
	ret = of_property_read_u32(node, "protocol", &proto);
	if (ret < 0) {
		dev_err(dev, "Can't read protocol property (ret %d)\n", ret);
		return ret;
	}
	of_property_read_u32(node, "current-speed", &speed);
	serdev_device_set_drvdata(serdev, ss);
	serdev_device_set_client_ops(serdev, &ss_serdev_ops);
	ret = serdev_device_open(serdev);
	if (ret)
		return ret;

	if (speed)
		serdev_device_set_baudrate(serdev, speed);
	serdev_device_set_flow_control(serdev, false);

	serio->port_data = ss;
	strlcpy(serio->name, "Serdev Serio", sizeof(serio->name));
	strlcpy(serio->phys, "serio", sizeof(serio->phys));
	serio->id.type = SERIO_RS232;
	serio->id.proto = proto;
	serio->id.id = SERIO_ANY;
	serio->id.extra = SERIO_ANY;
	serio->write = ss_serio_write;
	serio_register_port(serio);

	return 0;
}

static void ss_remove(struct serdev_device *serdev)
{
	struct serdev_serio *ss = serdev_device_get_drvdata(serdev);
	serdev_device_close(ss->serdev);
	serio_unregister_port(ss->serio);
}

static const struct of_device_id ss_of_match[] = {
	{ .compatible = "serdev,serio" },
	{},
};

static struct serdev_device_driver serdev_serio_drv = {
	.driver		= {
		.name	= "serdev_serio",
		.of_match_table = of_match_ptr(ss_of_match),
	},
	.probe  = ss_probe,
	.remove = ss_remove,
};

module_serdev_device_driver(serdev_serio_drv);

MODULE_AUTHOR("Vadim V. Vlasov <vvv19xx@gmail.com>");
MODULE_DESCRIPTION("Serdev Serio driver");
MODULE_LICENSE("GPL");
