// SPDX-License-Identifier: GPL-2.0
/*
 * T-Platforms serio port driver
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/serio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_irq.h>

MODULE_DESCRIPTION("T-Platforms serio port driver");
MODULE_LICENSE("GPL");

#define TP_SERIO_CHUNK_SIZE 4
#define TP_SERIO_SPI_SPEED_DEFAULT 500000
#define TP_SERIO_TX_QUEUE_SIZE 64
#define TP_SERIO_REQUEST_DELAY 2
#define TP_SERIO_POLL_READ_DELAY_MIN 1
#define TP_SERIO_POLL_READ_DELAY_MAX 2
#define TP_SERIO_POLL_WRITE_DELAY 1
#define TP_SERIO_POLL_ERROR_DELAY 100
#define TP_SERIO_POLL_READ_TIMEOUT 8
#define TP_SERIO_POLL_WAIT_TIMEOUT 100
#define TP_SERIO_CMD_QUERY 0xFC
#define TP_SERIO_CMD_RESET 0xFE

static const unsigned char tp_serio_cmd_reset_response[] = {
	TP_SERIO_CMD_RESET, 'P', 'S', '2'
};

struct tp_serio_tx {
	bool has_data;
	unsigned char data;
};

struct tp_serio_port {
	struct serio *serio;
	struct tp_serio_data *drv;
	struct tp_serio_tx tx;
	unsigned int id;
	bool registered;
};

struct tp_serio_data {
	struct i2c_client *dev_i2c;
	struct spi_device *dev_spi;
	struct task_struct *poll_task;
	wait_queue_head_t poll_wq;
	bool poll_ready;
	int rx_irq;
	unsigned int num_ports;
	struct tp_serio_port *ports;
};

struct tp_serio_driver {
#if defined(CONFIG_I2C)
	struct i2c_driver i2c;
#endif
#if defined(CONFIG_SPI)
	struct spi_driver spi;
#endif
};

#if defined(CONFIG_I2C)
static int tp_serio_i2c_write(struct tp_serio_data *drv,
		size_t size, void *data)
{
	struct i2c_msg m;

	m.addr = drv->dev_i2c->addr;
	m.flags = 0;
	m.len = size;
	m.buf = data;
	return i2c_transfer(drv->dev_i2c->adapter, &m, 1);
}

static int tp_serio_i2c_read(struct tp_serio_data *drv,
		size_t size, void *data)
{
	struct i2c_msg m;

	m.addr = drv->dev_i2c->addr;
	m.flags = I2C_M_RD;
	m.len = size;
	m.buf = data;
	return i2c_transfer(drv->dev_i2c->adapter, &m, 1);
}
#endif

#if defined(CONFIG_SPI)
static int tp_serio_spi_write(struct tp_serio_data *drv,
		size_t size, void *data)
{
	struct spi_transfer t = {
		.speed_hz = TP_SERIO_SPI_SPEED_DEFAULT,
		.tx_buf = data,
		.len = size,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(drv->dev_spi, &m);
}

static int tp_serio_spi_read(struct tp_serio_data *drv,
		size_t size, void *data)
{
	struct spi_transfer	t = {
		.speed_hz = TP_SERIO_SPI_SPEED_DEFAULT,
		.rx_buf = data,
		.len = size,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(drv->dev_spi, &m);
}
#endif

static int tp_serio_request(struct tp_serio_data *drv,
				unsigned char cmd,
				unsigned char *response)
{
	int result;
	size_t size;
	unsigned char message[TP_SERIO_CHUNK_SIZE];

	result = -ENODEV;
	memset(message, 0, sizeof(message));
	message[0] = cmd;
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL) {
		size = sizeof(message);
		result = tp_serio_spi_write(drv, size, message);
	} else
#endif
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL) {
		size = 1;
		result = tp_serio_i2c_write(drv, size, message);
	}
#endif
		;
	if (result < 0)
		return result;
	usleep_range(TP_SERIO_REQUEST_DELAY * 1000,
				TP_SERIO_REQUEST_DELAY * 1000);
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL)
		result = tp_serio_i2c_read(drv, TP_SERIO_CHUNK_SIZE, response);
	else
#endif
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL)
		result = tp_serio_spi_read(drv, TP_SERIO_CHUNK_SIZE, response);
#endif
		;
	return result;
}

static int tp_serio_data_read(struct tp_serio_data *drv)
{
	int result;
	size_t size;
	size_t index;
	size_t dbg_len;
	char dbg_line[256];
	unsigned int port_id;
	unsigned char message[TP_SERIO_CHUNK_SIZE];

	memset(message, 0, sizeof(message));
	result = -ENODEV;
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL)
		result = tp_serio_i2c_read(drv, sizeof(message), message);
	else
#endif
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL)
		result = tp_serio_spi_read(drv, sizeof(message), message);
#endif
		;
	if (result < 0)
		return result;

#if 0
	snprintf(dbg_line, ARRAY_SIZE(dbg_line) - 1, "raw read:");
	for (index = 0; index < ARRAY_SIZE(message); index++) {
		dbg_len = strlen(dbg_line);
		snprintf(dbg_line + dbg_len,
				ARRAY_SIZE(dbg_line) - 1 - dbg_len,
				" %02x", message[index]);
	}
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL)
		dev_dbg(&drv->dev_i2c->dev, "%s\n", dbg_line);
	else
#endif
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL)
		dev_dbg(&drv->dev_spi->dev, "%s\n", dbg_line);
#endif
		;
#endif

	result = 0;
	size = message[0] & 0x0F;
	port_id = (message[0] >> 4) & 0x0F;
	if ((size > 0) && (port_id < drv->num_ports)) {
		snprintf(dbg_line, ARRAY_SIZE(dbg_line) - 1,
			"port %u read:", port_id);

		if (size > (ARRAY_SIZE(message) - 1)) {
			size = ARRAY_SIZE(message) - 1;
			result = 1;
		}
		for (index = 0; index < size; index++) {
			dbg_len = strlen(dbg_line);
			snprintf(dbg_line + dbg_len,
					ARRAY_SIZE(dbg_line) - 1 - dbg_len,
					" %02x", message[index + 1]);
			serio_interrupt(drv->ports[port_id].serio,
				message[index + 1], 0);
		}
#if defined(CONFIG_I2C)
		if (drv->dev_i2c != NULL)
			dev_dbg(&drv->dev_i2c->dev, "%s\n", dbg_line);
		else
#endif
#if defined(CONFIG_SPI)
		if (drv->dev_spi != NULL)
			dev_dbg(&drv->dev_spi->dev, "%s\n", dbg_line);
#endif
			;
	}
	return result;
}

static int tp_serio_data_write(struct tp_serio_data *drv,
		u8 id, unsigned char data)
{
	int result;
	size_t size;
	unsigned char message[TP_SERIO_CHUNK_SIZE];
	struct tp_serio_port *port = drv->ports + id;

	result = -ENODEV;
	memset(message, 0, sizeof(message));
	message[0] = (port->id << 4) | 0x01;
	message[1] = data;
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL) {
		size = sizeof(message);
		dev_dbg(&drv->dev_spi->dev,
			"port %u write: %02x\n", port->id, data);
		result = tp_serio_spi_write(drv, size, message);
	} else
#endif
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL) {
		size = 2;
		dev_dbg(&drv->dev_i2c->dev,
			"port %u write: %02x\n", port->id, data);
		result = tp_serio_i2c_write(drv, size, message);
	}
#endif
		;
	return result;
}

static void tp_serio_trigger_tx(struct tp_serio_data *drv)
{
	drv->poll_ready = true;
	wake_up(&drv->poll_wq);
}

static int tp_serio_write(struct serio *serio, unsigned char data)
{
	int result = -EINVAL;
	struct tp_serio_data *drv;
	struct tp_serio_port *port = (struct tp_serio_port *)serio->port_data;

	if (port != NULL) {
		drv = port->drv;
		if (port->tx.has_data) {
			result = -ENOMEM;
		} else {
			port->tx.data = data;
			port->tx.has_data = true;
			result = 0;
		}
		tp_serio_trigger_tx(drv);
	}
	return result;
}

static int tp_serio_start(struct serio *serio)
{
	struct tp_serio_port *port = (struct tp_serio_port *)serio->port_data;

	if (port != NULL)
		port->registered = true;
	return 0;
}

static void tp_serio_stop(struct serio *serio)
{
	struct tp_serio_port *port = (struct tp_serio_port *)serio->port_data;

	if (port != NULL)
		port->registered = false;
}

static int tp_serio_create_port(struct tp_serio_data *drv, unsigned int id)
{
	struct serio *serio;
	struct device *dev;

#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL) {
		dev = &drv->dev_spi->dev;
	} else
#endif
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL) {
		dev = &drv->dev_i2c->dev;
	} else
#endif
	{
		return -ENODEV;
	}
	serio = devm_kzalloc(dev, sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;
	strlcpy(serio->name, "tp_serio", sizeof(serio->name));
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL) {
		snprintf(serio->phys, sizeof(serio->phys),
			 "%s/port%u", dev_name(&drv->dev_spi->dev), id);
	} else
#endif
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL) {
		snprintf(serio->phys, sizeof(serio->phys),
			 "%s/port%u", dev_name(&drv->dev_i2c->dev), id);
	}
#endif
		;
	serio->id.type = SERIO_8042;
	serio->write = tp_serio_write;
	serio->start = tp_serio_start;
	serio->stop = tp_serio_stop;
	serio->port_data = drv->ports + id;
	drv->ports[id].serio = serio;
	drv->ports[id].drv = drv;
	drv->ports[id].id = id;
	drv->ports[id].registered = false;
	drv->ports[id].tx.has_data = false;
	drv->ports[id].tx.data = 0x00;
	return 0;
}

static void tp_serio_destroy_port(struct tp_serio_data *drv, unsigned int id)
{
	if (drv->ports[id].registered)
		serio_unregister_port(drv->ports[id].serio);
}

static void tp_serio_read_error(struct tp_serio_data *drv, int error)
{
#if defined(CONFIG_I2C)
	if (drv->dev_i2c != NULL)
		dev_dbg(&drv->dev_i2c->dev,
			"i2c read failed: %d\n", error);
	else
#endif
#if defined(CONFIG_SPI)
	if (drv->dev_spi != NULL)
		dev_dbg(&drv->dev_spi->dev,
			"spi read failed: %d\n", error);
#endif
		;
	msleep_interruptible(TP_SERIO_POLL_ERROR_DELAY);
}

static void tp_serio_serio_process_tx(struct tp_serio_data *drv)
{
	unsigned int index;

	for (index = 0; index < drv->num_ports; index++) {
		if (drv->ports[index].tx.has_data) {
			tp_serio_data_write(drv, index,
					    drv->ports[index].tx.data);
			drv->ports[index].tx.has_data = false;
			usleep_range(TP_SERIO_POLL_WRITE_DELAY * 1000,
					TP_SERIO_POLL_WRITE_DELAY * 1000);
		}
	}
}

static int tp_serio_serio_process_rx(struct tp_serio_data *drv)
{
	int ret;

	do {
		ret = tp_serio_data_read(drv);
		usleep_range(TP_SERIO_POLL_READ_DELAY_MIN * 1000,
				TP_SERIO_POLL_READ_DELAY_MAX * 1000);
	} while (ret > 0);
	if ((ret < 0) && (ret != -EAGAIN))
		tp_serio_read_error(drv, ret);
	return ret;
}

static int tp_serio_poll(void *data)
{
	struct tp_serio_data *drv = (struct tp_serio_data *)data;
	const unsigned int poll_timeout = (drv->rx_irq < 0) ?
			TP_SERIO_POLL_READ_TIMEOUT :
			TP_SERIO_POLL_WAIT_TIMEOUT;

	while (!kthread_should_stop()) {
		drv->poll_ready = false;
		tp_serio_serio_process_tx(drv);

		if (drv->rx_irq < 0)
			while (tp_serio_serio_process_rx(drv))
				;

		wait_event_interruptible_timeout(drv->poll_wq, drv->poll_ready,
				msecs_to_jiffies(poll_timeout));
	}
	return 0;
}

static irqreturn_t tp_serio_alert_handler(int irq, void *dev_id)
{
	struct tp_serio_data *drv = (struct tp_serio_data *)dev_id;

	while (tp_serio_serio_process_rx(drv))
		;
	return IRQ_HANDLED;
}

static int tp_serio_device_reset(struct tp_serio_data *drv)
{
	int result;
	unsigned char response[TP_SERIO_CHUNK_SIZE];

	memset(response, 0, sizeof(response));
	result = tp_serio_request(drv, TP_SERIO_CMD_RESET, response);
	if (result < 0)
		return result;
	if (!memcmp(response, tp_serio_cmd_reset_response, sizeof(response)))
		result = 0;
	else
		result = -EINVAL;
	return result;
}

static int tp_serio_device_query(struct tp_serio_data *drv)
{
	int result;
	unsigned char response[TP_SERIO_CHUNK_SIZE];

	memset(response, 0, sizeof(response));
	result = tp_serio_request(drv, TP_SERIO_CMD_QUERY, response);
	if (result < 0)
		return result;
	if (response[0] == TP_SERIO_CMD_QUERY) {
		drv->num_ports = response[1];
		result = 0;
	} else {
		result = -EINVAL;
	}
	return result;
}

#if defined(CONFIG_I2C)
static int tp_serio_probe_i2c(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tp_serio_data *drv;
	unsigned int index;
	unsigned int free_index;
	int error;
	int irq;
	struct serio *s;

	drv = devm_kzalloc(&client->dev, sizeof(*drv), GFP_KERNEL);
	if (drv == NULL)
		return -ENOMEM;
	drv->dev_i2c = client;
#if defined(CONFIG_SPI)
	drv->dev_spi = NULL;
#endif
	if (tp_serio_device_reset(drv) < 0) {
		dev_err(&client->dev, "no compatible device found at %s\n",
			dev_name(&client->dev));
		return -ENODEV;
	}
	error = tp_serio_device_query(drv);
	if (error || (drv->num_ports == 0)) {
		dev_err(&client->dev, "no available ports found at %s\n",
			dev_name(&client->dev));
		return -ENODEV;
	}
	drv->ports = devm_kzalloc(&client->dev,
			sizeof(struct tp_serio_port) * drv->num_ports,
			GFP_KERNEL);
	if (drv->ports == NULL)
		return -ENOMEM;
	for (index = 0; index < drv->num_ports; index++) {
		error = tp_serio_create_port(drv, index);
		if (error)
			goto err_out;
	}
	init_waitqueue_head(&drv->poll_wq);
	drv->poll_ready = false;
	drv->rx_irq = -1;
	dev_set_drvdata(&client->dev, drv);

	for (index = 0; index < drv->num_ports; index++) {
		s = drv->ports[index].serio;
		dev_info(&client->dev, "%s port at %s\n", s->name, s->phys);
		serio_register_port(s);
	}

	if (client->dev.of_node != NULL) {
		irq = of_irq_get(client->dev.of_node, 0);
		if (irq >= 0) {
			drv->rx_irq = irq;
			error = devm_request_threaded_irq(&client->dev, irq,
					NULL, tp_serio_alert_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"tp_serio", drv);
			if (error) {
				dev_set_drvdata(&client->dev, NULL);
				index = drv->num_ports;
				goto err_out;
			} else {
				tp_serio_alert_handler(drv->rx_irq, drv);
			}
		}
	}
	drv->poll_task = kthread_run(tp_serio_poll, drv,
			"tp_serio i2c");
	return 0;
err_out:
	for (free_index = 0; free_index < index; free_index++)
		tp_serio_destroy_port(drv, free_index);
	return error;
}

static int tp_serio_remove_i2c(struct i2c_client *client)
{
	struct tp_serio_data *drv =
		(struct tp_serio_data *)dev_get_drvdata(&client->dev);
	unsigned int index;

	if (drv != NULL) {
		kthread_stop(drv->poll_task);
		for (index = 0; index < drv->num_ports; index++)
			tp_serio_destroy_port(drv, index);
		dev_set_drvdata(&client->dev, NULL);
	}
	return 0;
}
#endif

#if defined(CONFIG_SPI)
static int tp_serio_probe_spi(struct spi_device *spi)
{
	struct tp_serio_data *drv;
	unsigned int index;
	unsigned int free_index;
	int error;
	int irq;
	struct serio *s;

	drv = devm_kzalloc(&spi->dev, sizeof(*drv), GFP_KERNEL);
	if (drv == NULL)
		return -ENOMEM;
#if defined(CONFIG_I2C)
	drv->dev_i2c = NULL;
#endif
	drv->dev_spi = spi;
	if (tp_serio_device_reset(drv) < 0) {
		dev_err(&spi->dev, "no compatible device found at %s\n",
			dev_name(&spi->dev));
		return -ENODEV;
	}
	error = tp_serio_device_query(drv);
	if (error || (drv->num_ports == 0)) {
		dev_err(&spi->dev, "no available ports found at %s\n",
			dev_name(&spi->dev));
		return -ENODEV;
	}
	drv->ports = devm_kzalloc(&spi->dev,
			sizeof(struct tp_serio_port) * drv->num_ports,
			GFP_KERNEL);
	if (drv->ports == NULL)
		return -ENOMEM;
	for (index = 0; index < drv->num_ports; index++) {
		error = tp_serio_create_port(drv, index);
		if (error)
			goto err_out;
	}
	init_waitqueue_head(&drv->poll_wq);
	drv->poll_ready = false;
	drv->rx_irq = -1;
	spi_set_drvdata(spi, drv);

	for (index = 0; index < drv->num_ports; index++) {
		s = drv->ports[index].serio;
		dev_info(&spi->dev, "%s port at %s\n", s->name, s->phys);
		serio_register_port(s);
	}

	if (spi->dev.of_node != NULL) {
		irq = of_irq_get(spi->dev.of_node, 0);
		if (irq >= 0) {
			drv->rx_irq = irq;
			error = devm_request_threaded_irq(&spi->dev, irq,
					NULL, tp_serio_alert_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"tp_serio", drv);
			if (error) {
				spi_set_drvdata(spi, NULL);
				index = drv->num_ports;
				goto err_out;
			} else {
				tp_serio_alert_handler(drv->rx_irq, drv);
			}
		}
	}
	drv->poll_task = kthread_run(tp_serio_poll, drv,
			"tp_serio spi");
	return 0;
err_out:
	for (free_index = 0; free_index < index; free_index++)
		tp_serio_destroy_port(drv, free_index);
	return error;
}

static int tp_serio_remove_spi(struct spi_device *spi)
{
	struct tp_serio_data *drv =
		(struct tp_serio_data *)spi_get_drvdata(spi);
	unsigned int index;

	if (drv != NULL) {
		kthread_stop(drv->poll_task);
		for (index = 0; index < drv->num_ports; index++)
			tp_serio_destroy_port(drv, index);
		spi_set_drvdata(spi, NULL);
	}
	return 0;
}
#endif

static int tp_serio_register(struct tp_serio_driver *driver)
{
	int res = 0;
#if defined(CONFIG_I2C)
	res = i2c_register_driver(THIS_MODULE, &driver->i2c);
#endif
#if defined(CONFIG_SPI)
	if (res == 0)
		res = spi_register_driver(&driver->spi);
#endif
	return res;
}

static void tp_serio_unregister(struct tp_serio_driver *driver)
{
#if defined(CONFIG_SPI)
	spi_unregister_driver(&driver->spi);
#endif
#if defined(CONFIG_I2C)
	i2c_del_driver(&driver->i2c);
#endif
}

static const struct of_device_id tp_serio_of_ids[] = {
	{
		.compatible = "tp,tp_serio",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tp_serio_of_ids);

#if defined(CONFIG_I2C)
static const struct i2c_device_id tp_serio_i2c_ids[] = {
	{
		.name = "tp_serio",
	},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tp_serio_i2c_ids);
#endif

#if defined(CONFIG_SPI)
static const struct spi_device_id tp_serio_spi_ids[] = {
	{
		.name = "tp_serio",
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, tp_serio_spi_ids);
#endif

static struct tp_serio_driver tp_serio_drv = {
#if defined(CONFIG_I2C)
	{
		.driver = {
			.name = "tp_serio",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(tp_serio_of_ids)
		},
		.probe = tp_serio_probe_i2c,
		.remove = tp_serio_remove_i2c,
		.id_table = tp_serio_i2c_ids
	},
#endif
#if defined(CONFIG_SPI)
	{
		.driver = {
			.name = "tp_serio",
			.owner = THIS_MODULE,
			.of_match_table = of_match_ptr(tp_serio_of_ids)
		},
		.probe = tp_serio_probe_spi,
		.remove = tp_serio_remove_spi,
		.id_table = tp_serio_spi_ids
	}
#endif
};

module_driver(tp_serio_drv, tp_serio_register, tp_serio_unregister)
