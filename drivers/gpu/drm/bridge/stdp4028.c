// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for MegaChips STDP4028 LVDS to DP display bridge
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>
#include <drm/drmP.h>

/* video modes */
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define MAX_PIXEL_CLOCK 330000

#define EDID_EXT_BLOCK_CNT 0x7E

#define STDP4028_PRODUCT_ID_REG 0x00
#define STDP4028_IRQ_OUT_CONF_REG 0x02
#define STDP4028_IRQ_STS_REG 0x03
#define STDP4028_I2C_CTRL_REG 0x08
#define STDP4028_LVDS_FMT_REG 0x0B
#define STDP4028_LVDS_CTRL0_REG 0x0C
#define STDP4028_DPTX_IRQ_EN_REG 0x3C
#define STDP4028_DPTX_IRQ_STS_REG 0x3D
#define STDP4028_DPTX_STS_REG 0x3E

#define STDP4028_DPTX_DP_IRQ_EN 0x10

#define STDP4028_DPTX_HOTPLUG_IRQ_EN 0x04
#define STDP4028_DPTX_LINK_CH_IRQ_EN 0x20
#define STDP4028_DPTX_IRQ_CONFIG \
	(STDP4028_DPTX_LINK_CH_IRQ_EN | STDP4028_DPTX_HOTPLUG_IRQ_EN)

#define STDP4028_DPTX_HOTPLUG_STS 0x02
#define STDP4028_DPTX_LINK_STS 0x10
#define STDP4028_CON_STATE_CONNECTED \
	(STDP4028_DPTX_HOTPLUG_STS | STDP4028_DPTX_LINK_STS)

#define STDP4028_DPTX_HOTPLUG_CH_STS 0x04
#define STDP4028_DPTX_LINK_CH_STS 0x20
#define STDP4028_DPTX_IRQ_CLEAR \
	(STDP4028_DPTX_LINK_CH_STS | STDP4028_DPTX_HOTPLUG_CH_STS)

struct stdp4028 {
	struct drm_connector connector;
	struct drm_bridge bridge;
	struct i2c_client *stdp4028_i2c;
	struct i2c_client *edid_i2c;
	struct edid *edid;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	int channels;
	int chan_cfg;
};

static inline int stdp_read(struct stdp4028 *stdp, int reg)
{
	int ret;

	ret = i2c_smbus_read_word_data(stdp->stdp4028_i2c, reg);
	if (ret < 0)
		return ret;
	return be16_to_cpu(ret);
}

static inline int stdp_write(struct stdp4028 *stdp, int reg, u16 val)
{
	val = cpu_to_be16(val);
	return i2c_smbus_write_word_data(stdp->stdp4028_i2c, reg, val);
}

#define bridge_to_stdp4028(bridge) \
	container_of(bridge, struct stdp4028, bridge)

#define connector_to_stdp4028(connector) \
	container_of(connector, struct stdp4028, connector)

u8 *stdp4028_get_edid(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	unsigned char start = 0x00;
	unsigned int total_size;
	u8 *block = kmalloc(EDID_LENGTH, GFP_KERNEL);

	struct i2c_msg msgs[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = &start,
		}, {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = EDID_LENGTH,
			.buf    = block,
		}
	};

	if (!block)
		return NULL;

	if (i2c_transfer(adapter, msgs, 2) != 2) {
		DRM_ERROR("Unable to read EDID.\n");
		goto err;
	}

	if (!drm_edid_block_valid(block, 0, false, NULL)) {
		DRM_ERROR("Invalid EDID block\n");
		goto err;
	}

	total_size = (block[EDID_EXT_BLOCK_CNT] + 1) * EDID_LENGTH;
	if (total_size > EDID_LENGTH) {
		kfree(block);
		block = kmalloc(total_size, GFP_KERNEL);
		if (!block)
			return NULL;

		/* Yes, read the entire buffer, and do not skip the first
		 * EDID_LENGTH bytes.
		 */
		start = 0x00;
		msgs[1].len = total_size;
		msgs[1].buf = block;

		if (i2c_transfer(adapter, msgs, 2) != 2) {
			DRM_ERROR("Unable to read EDID extension blocks.\n");
			goto err;
		}
	}

	return block;

err:
	kfree(block);
	return NULL;
}

/*
 * Get videomode specified in the devicetree.
 * Return 1 on success, 0 otherwise.
 */
static int stdp4028_get_of_modes(struct drm_connector *connector)
{
	struct stdp4028 *stdp = connector_to_stdp4028(connector);
	struct i2c_client *client = stdp->stdp4028_i2c;
	struct drm_display_mode *mode;
	struct device_node *np = client->dev.of_node;
	struct display_timing timing;
	struct videomode video_mode;
	int ret;

	ret = of_get_display_timing(np, "panel-timing", &timing);
	if (ret < 0)
		return 0;

	videomode_from_timing(&timing, &video_mode);

	mode = drm_mode_create(connector->dev);
	if (!mode)
		return 0;
	drm_display_mode_from_videomode(&video_mode, mode);
	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_probed_add(connector, mode);
	return 1;
}

static int stdp4028_get_modes(struct drm_connector *connector)
{
	struct stdp4028 *stdp;
	struct i2c_client *client;
	int num_modes = 0;

	stdp = connector_to_stdp4028(connector);
	client = stdp->edid_i2c;

	mutex_lock(&stdp->lock);

	num_modes = stdp4028_get_of_modes(connector);
	if (num_modes > 0) {
		mutex_unlock(&stdp->lock);
		return num_modes;
	}

	kfree(stdp->edid);
	stdp->edid = (struct edid *) stdp4028_get_edid(client);

	if (stdp->edid) {
		drm_connector_update_edid_property(connector, stdp->edid);
		num_modes = drm_add_edid_modes(connector, stdp->edid);
	}

	mutex_unlock(&stdp->lock);

	return num_modes;
}


static enum drm_mode_status stdp4028_mode_valid(
		 struct drm_connector *connector, struct drm_display_mode *mode)
{
	if (mode->clock > MAX_PIXEL_CLOCK) {
		DRM_INFO("The pixel clock for the mode %s is too high, and not supported.",
			 mode->name);
		return MODE_CLOCK_HIGH;
	}

	return MODE_OK;
}

static const struct
drm_connector_helper_funcs stdp4028_connector_helper_funcs = {
	.get_modes = stdp4028_get_modes,
	.mode_valid = stdp4028_mode_valid,
};

static enum drm_connector_status stdp4028_detect(
		 struct drm_connector *connector, bool force)
{
	struct stdp4028 *stdp = connector_to_stdp4028(connector);
	s32 link_state;

	link_state = stdp_read(stdp, STDP4028_DPTX_STS_REG);

	if (link_state == STDP4028_CON_STATE_CONNECTED)
		return connector_status_connected;

	if (link_state == 0)
		return connector_status_disconnected;

	return connector_status_unknown;
}

static const struct drm_connector_funcs stdp4028_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = stdp4028_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static irqreturn_t stdp4028_irq_handler(int irq, void *dev_id)
{
	struct stdp4028 *stdp = dev_id;

	mutex_lock(&stdp->lock);

	stdp_write(stdp, STDP4028_DPTX_IRQ_STS_REG, STDP4028_DPTX_IRQ_CLEAR);

	mutex_unlock(&stdp->lock);

	if (stdp->connector.dev)
		drm_kms_helper_hotplug_event(stdp->connector.dev);

	return IRQ_HANDLED;
}

static int stdp4028_attach(struct drm_bridge *bridge)
{
	struct stdp4028 *stdp
			  = bridge_to_stdp4028(bridge);
	struct drm_connector *connector = &stdp->connector;
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	if (stdp->stdp4028_i2c->irq)
		connector->polled = DRM_CONNECTOR_POLL_HPD;
	else
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				    DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_helper_add(connector, &stdp4028_connector_helper_funcs);

	ret = drm_connector_init(bridge->dev, connector,
				 &stdp4028_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	ret = drm_connector_attach_encoder(connector, bridge->encoder);
	if (ret)
		return ret;

	/* Configures the bridge to re-enable interrupts after each ack. */
	stdp_write(stdp, STDP4028_IRQ_OUT_CONF_REG, STDP4028_DPTX_DP_IRQ_EN);

	/* Enable interrupts */
	stdp_write(stdp, STDP4028_DPTX_IRQ_EN_REG, STDP4028_DPTX_IRQ_CONFIG);

	return 0;
}

static const struct drm_bridge_funcs stdp4028_funcs = {
	.attach = stdp4028_attach,
};

static int stdp4028_probe(struct i2c_client *stdp4028_i2c,
			  const struct i2c_device_id *id)
{
	struct device *dev = &stdp4028_i2c->dev;
	struct stdp4028 *bridge;
	int ret;
	u32 edid_i2c_reg, channels, chan_cfg;
	enum of_gpio_flags flags;
	int reset_gpio, i;
	int reg;

	bridge = devm_kzalloc(dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge)
		return -ENOMEM;

	mutex_init(&bridge->lock);

	bridge->stdp4028_i2c = stdp4028_i2c;
	bridge->bridge.driver_private = bridge;
	i2c_set_clientdata(stdp4028_i2c, bridge);

	reset_gpio = of_get_named_gpio_flags(dev->of_node,
					     "reset-gpios", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		/*
		 * We will set GPIO to "inactive" state instead of toggling
		 * reset. If the chip is not ready we will return -EPROBE_DEFER
		 * and retry later.
		 */
		if (!(flags & OF_GPIO_ACTIVE_LOW))
			gpio_flags = GPIOF_ACTIVE_LOW | GPIOF_OUT_INIT_LOW;
		else
			gpio_flags = GPIOF_OUT_INIT_HIGH;
		ret = devm_gpio_request_one(dev, reset_gpio, gpio_flags,
					    "stdp-reset");
		if (ret) {
			dev_err(dev, "request GPIO failed (%d)\n", ret);
			/* continue anyway */
		} else {
			bridge->reset_gpio = gpio_to_desc(reset_gpio);
			udelay(100);
		}
	} else if (reset_gpio == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}

	ret = of_property_read_u32(dev->of_node, "channels", &channels);
	if (ret)
		channels = 1;
	bridge->channels = channels;

	ret = of_property_read_u32(dev->of_node, "chan-cfg", &chan_cfg);
	if (ret)
		chan_cfg = 0;
	bridge->chan_cfg = chan_cfg;

	ret = of_property_read_u32(dev->of_node, "edid-reg", &edid_i2c_reg);
	if (ret) {
		dev_warn(dev, "edid-reg not specified, assuming 0x50...\n");
		edid_i2c_reg = 0x50;
	}

	/* Configure stdp registers */
	reg = stdp_read(bridge, STDP4028_PRODUCT_ID_REG);
	if (reg < 0) {
		dev_err(dev, "Can't read stdp id (%d)\n", reg);
		return -EPROBE_DEFER; /* probably, reset not complete */
	}

	dev_info(dev, "stdp id word: %x\n", reg);

	for (i = 0; i < 10; i++) {
		reg = stdp_read(bridge, STDP4028_IRQ_STS_REG);
		if (reg > 0 && reg & 0x800)
			break;
		usleep_range(1000, 1500);
	}
	dev_dbg(dev, "STDP status word %x (i = %d)\n", reg, i);
	stdp_write(bridge, STDP4028_IRQ_STS_REG, 0x800); //clear
	/* enable edid addr */
	stdp_write(bridge, STDP4028_I2C_CTRL_REG, (edid_i2c_reg << 1) | 0x400);

	if (channels == 4)
		reg = 2;
	else if (channels == 2)
		reg = 1;
	else
		reg = 0;
	reg |= chan_cfg << 2;
	stdp_write(bridge, STDP4028_LVDS_CTRL0_REG, reg);

	bridge->edid_i2c = i2c_new_dummy(stdp4028_i2c->adapter, edid_i2c_reg);

	if (!bridge->edid_i2c)
		return -ENOMEM;

	bridge->bridge.funcs = &stdp4028_funcs;
	bridge->bridge.of_node = dev->of_node;
	drm_bridge_add(&bridge->bridge);

	/* Clear pending interrupts since power up. */
	stdp_write(bridge, STDP4028_DPTX_IRQ_STS_REG, STDP4028_DPTX_IRQ_CLEAR);

	if (stdp4028_i2c->irq) {
		ret = devm_request_threaded_irq(&stdp4028_i2c->dev,
					stdp4028_i2c->irq, NULL,
					stdp4028_irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"stdp-lvds-dp", bridge);
		if (ret)
			return ret;

		/* enable DPTX IRQs */
		stdp_write(bridge, STDP4028_IRQ_OUT_CONF_REG,
			   STDP4028_DPTX_DP_IRQ_EN);
		stdp_write(bridge, STDP4028_DPTX_IRQ_EN_REG,
			   STDP4028_DPTX_IRQ_CONFIG);
	}

	return 0;
}

static int stdp4028_remove(struct i2c_client *stdp4028_i2c)
{
	struct stdp4028 *stdp =	i2c_get_clientdata(stdp4028_i2c);

	drm_bridge_remove(&stdp->bridge);
	i2c_unregister_device(stdp->edid_i2c);

	kfree(stdp->edid);

	return 0;
}

static const struct i2c_device_id stdp4028_i2c_table[] = {
	{"stdp4028-lvds-dp", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, stdp4028_i2c_table);

static const struct of_device_id stdp4028_match[] = {
	{ .compatible = "megachips,stdp4028-lvds-dp" },
	{},
};
MODULE_DEVICE_TABLE(of, stdp4028_match);

static struct i2c_driver stdp4028_driver = {
	.id_table	= stdp4028_i2c_table,
	.probe		= stdp4028_probe,
	.remove		= stdp4028_remove,
	.driver		= {
		.name		= "stdp4028-lvds-dp",
		.of_match_table	= stdp4028_match,
	},
};
module_i2c_driver(stdp4028_driver);

MODULE_AUTHOR("Vadim V. Vlasov <vvv19xx at gmail.com>");
MODULE_DESCRIPTION("STDP4028 LVDS to DP display bridge)");
MODULE_LICENSE("GPL v2");
