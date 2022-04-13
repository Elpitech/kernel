// SPDX-License-Identifier: GPL-2.0-or-later
/*
* Implementation of primary ALSA driver code base for Baikal-M HDA controller.
*
* /sound/pci/hda/hda_baikal.c
*
* Copyright (C) 2021 Baikal Electronics, JSC
*        Danila Sharikov <Danila.Sharikov@baikalelectronics.com>
*
* Based on:
*	 hda_intel.c, Copyright(c) 2004 Intel Corporation. All rights reserved.
*        hda_tegra.c
*
*/

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/hda_codec.h>

#include "hda_controller.h"

#ifdef CONFIG_PM
static int power_save = CONFIG_SND_HDA_POWER_SAVE_DEFAULT;
module_param(power_save, bint, 0644);
MODULE_PARM_DESC(power_save,
		 "Automatic power-saving timeout (in seconds, 0 = disable).");
#else
#define power_save	0
#endif

/* max number of SDs */
#define NUM_CAPTURE_SD	4
#define NUM_PLAYBACK_SD	4

struct hda_baikal {
	struct azx chip;
	struct device *dev;
	void __iomem *regs;
	struct work_struct probe_work;
	struct work_struct irq_pending_work;
	unsigned int irq_pending_warned:1;
};

static void hda_baikal_probe_work(struct work_struct *work);

static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);
static const struct hda_controller_ops hda_baikal_ops;

/* calculate runtime delay from LPIB */
static int azx_get_delay_from_lpib(struct azx *chip, struct azx_dev *azx_dev,
				   unsigned int pos)
{
	struct snd_pcm_substream *substream = azx_dev->core.substream;
	int stream = substream->stream;
	unsigned int lpib_pos = azx_get_pos_lpib(chip, azx_dev);
	int delay;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		delay = pos - lpib_pos;
	else
		delay = lpib_pos - pos;
	if (delay < 0) {
		if (delay >= azx_dev->core.delay_negative_threshold)
			delay = 0;
		else
			delay += azx_dev->core.bufsize;
	}

	if (delay >= azx_dev->core.period_bytes) {
		dev_info(chip->card->dev,
			 "Unstable LPIB (%d >= %d); disabling LPIB delay counting\n",
			 delay, azx_dev->core.period_bytes);
		delay = 0;
		chip->driver_caps &= ~AZX_DCAPS_COUNT_LPIB_DELAY;
		chip->get_delay[stream] = NULL;
	}

	return bytes_to_frames(substream->runtime, delay);
}

/* called from IRQ */
static int azx_position_check(struct azx *chip, struct azx_dev *azx_dev)
{
	struct hda_baikal *hda = container_of(chip, struct hda_baikal, chip);
	int ok;

	ok = azx_position_ok(chip, azx_dev);

	if (ok == 1) {
		azx_dev->irq_pending = 0;
		return ok;
	} else if (ok == 0) {
		/* bogus IRQ, process it later */
		azx_dev->irq_pending = 1;
		schedule_work(&hda->irq_pending_work);
	}
	return 0;
}

/*
 * Check whether the current DMA position is acceptable for updating
 * periods.  Returns non-zero if it's OK.
 *
 * Many HD-audio controllers appear pretty inaccurate about
 * the update-IRQ timing.  The IRQ is issued before actually the
 * data is processed.  So, we need to process it afterwords in a
 * workqueue.
 */
static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev)
{
	struct snd_pcm_substream *substream = azx_dev->core.substream;
	int stream = substream->stream;
	u32 wallclk;
	unsigned int pos;

	wallclk = azx_readl(chip, WALLCLK) - azx_dev->core.start_wallclk;
	if (wallclk < (azx_dev->core.period_wallclk * 2) / 3)
		return -1;	/* bogus (too early) interrupt */

	if (chip->get_position[stream])
		pos = chip->get_position[stream](chip, azx_dev);
	else { /* use the position buffer as default */
		pos = azx_get_pos_posbuf(chip, azx_dev);
		if (!pos || pos == (u32)-1) {
			dev_info(chip->card->dev,
				 "Invalid position buffer, using LPIB read method instead.\n");
			chip->get_position[stream] = azx_get_pos_lpib;
			if (chip->get_position[0] == azx_get_pos_lpib &&
			    chip->get_position[1] == azx_get_pos_lpib)
				azx_bus(chip)->use_posbuf = false;
			pos = azx_get_pos_lpib(chip, azx_dev);
			chip->get_delay[stream] = NULL;
		} else {
			chip->get_position[stream] = azx_get_pos_posbuf;
			if (chip->driver_caps & AZX_DCAPS_COUNT_LPIB_DELAY)
				chip->get_delay[stream] = azx_get_delay_from_lpib;
		}
	}

	if (pos >= azx_dev->core.bufsize)
		pos = 0;

	if (WARN_ONCE(!azx_dev->core.period_bytes,
		      "hda-baikal: zero azx_dev->period_bytes"))
		return -1; /* this shouldn't happen! */
	if (wallclk < (azx_dev->core.period_wallclk * 5) / 4 &&
	    pos % azx_dev->core.period_bytes > azx_dev->core.period_bytes / 2)
		/* NG - it's below the first next period boundary */
		return chip->bdl_pos_adj ? 0 : -1;
	azx_dev->core.start_wallclk += wallclk;
	return 1; /* OK, it's fine */
}

/*
 * The work for pending PCM period updates.
 */
static void azx_irq_pending_work(struct work_struct *work)
{
	struct hda_baikal *hda = container_of(work, struct hda_baikal, irq_pending_work);
	struct azx *chip = &hda->chip;
	struct hdac_bus *bus = azx_bus(chip);
	struct hdac_stream *s;
	int pending, ok;

	if (!hda->irq_pending_warned) {
		dev_info(chip->card->dev,
			 "IRQ timing workaround is activated for card #%d. Suggest a bigger bdl_pos_adj.\n",
			 chip->card->number);
		hda->irq_pending_warned = 1;
	}

	for (;;) {
		pending = 0;
		spin_lock_irq(&bus->reg_lock);
		list_for_each_entry(s, &bus->stream_list, list) {
			struct azx_dev *azx_dev = stream_to_azx_dev(s);
			if (!azx_dev->irq_pending ||
			    !s->substream ||
			    !s->running)
				continue;
			ok = azx_position_ok(chip, azx_dev);
			if (ok > 0) {
				azx_dev->irq_pending = 0;
				spin_unlock(&bus->reg_lock);
				snd_pcm_period_elapsed(s->substream);
				spin_lock(&bus->reg_lock);
			} else if (ok < 0) {
				pending = 0;	/* too early */
			} else
				pending++;
		}
		spin_unlock_irq(&bus->reg_lock);
		if (!pending)
			return;
		msleep(1);
	}
}

/* clear irq_pending flags and assure no on-going workq */
static void azx_clear_irq_pending(struct azx *chip)
{
	struct hdac_bus *bus = azx_bus(chip);
	struct hdac_stream *s;

	spin_lock_irq(&bus->reg_lock);
	list_for_each_entry(s, &bus->stream_list, list) {
		struct azx_dev *azx_dev = stream_to_azx_dev(s);
		azx_dev->irq_pending = 0;
	}
	spin_unlock_irq(&bus->reg_lock);
}

static int hda_baikal_dev_disconnect(struct snd_device *device)
{
	struct azx *chip = device->device_data;

	chip->bus.shutdown = 1;
	return 0;
}

static int hda_baikal_dev_free(struct snd_device *device)
{
	struct azx *chip = device->device_data;
	struct hda_baikal *hda = container_of(chip, struct hda_baikal, chip);

	cancel_work_sync(&hda->probe_work);
	if (azx_bus(chip)->chip_init) {
		azx_clear_irq_pending(chip);
		azx_stop_all_streams(chip);
		azx_stop_chip(chip);
	}

	azx_free_stream_pages(chip);
	azx_free_streams(chip);
	snd_hdac_bus_exit(azx_bus(chip));

	return 0;
}

static int hda_baikal_init_chip(struct azx *chip, struct platform_device *pdev)
{
	struct hda_baikal *hda = container_of(chip, struct hda_baikal, chip);
	struct hdac_bus *bus = azx_bus(chip);
	struct device *dev = hda->dev;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hda->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hda->regs))
		return PTR_ERR(hda->regs);

	bus->remap_addr = hda->regs;
	bus->addr = res->start;

	return 0;
}

static int hda_baikal_first_init(struct azx *chip, struct platform_device *pdev)
{
	struct hdac_bus *bus = azx_bus(chip);
	struct snd_card *card = chip->card;
	int err;
	unsigned short gcap;
	int irq_id = platform_get_irq(pdev, 0);
	const char *sname, *drv_name = "baikal-hda";
	struct device_node *np = pdev->dev.of_node;

	err = hda_baikal_init_chip(chip, pdev);
	if (err)
		return err;

	err = devm_request_irq(chip->card->dev, irq_id, azx_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, chip);
	if (err) {
		dev_err(chip->card->dev,
			"unable to request IRQ %d, disabling device\n",
			irq_id);
		return err;
	}
	bus->irq = irq_id;

	synchronize_irq(bus->irq);

	gcap = azx_readw(chip, GCAP);
	dev_dbg(card->dev, "chipset global capabilities = 0x%x\n", gcap);

	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */
		chip->playback_streams = NUM_PLAYBACK_SD;
		chip->capture_streams = NUM_CAPTURE_SD;
	}
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;

	/* initialize streams */
	err = azx_init_streams(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to initialize streams: %d\n", err);
		return err;
	}

	err = azx_alloc_stream_pages(chip);
	if (err < 0) {
		dev_err(card->dev, "failed to allocate stream pages: %d\n",
			err);
		return err;
	}

	/* initialize chip */
	azx_init_chip(chip, 1);

	/* codec detection */
	if (!bus->codec_mask) {
		dev_err(card->dev, "no codecs found!\n");
		return -ENODEV;
	}

	/* driver name */
	strncpy(card->driver, drv_name, sizeof(card->driver));
	/* shortname for card */
	sname = of_get_property(np, "baikal,model", NULL);
	if (!sname)
		sname = drv_name;
	if (strlen(sname) > sizeof(card->shortname))
		dev_info(card->dev, "truncating shortname for card\n");
	strncpy(card->shortname, sname, sizeof(card->shortname));

	/* longname for card */
	snprintf(card->longname, sizeof(card->longname),
		 "%s at 0x%lx irq %i",
		 card->shortname, bus->addr, bus->irq);

	return 0;
}

static int hda_baikal_create(struct snd_card *card,
			    unsigned int driver_caps,
			    struct hda_baikal *hda)
{
	static struct snd_device_ops ops = {
		.dev_disconnect = hda_baikal_dev_disconnect,
		.dev_free = hda_baikal_dev_free,
	};
	struct azx *chip;
	int err;

	chip = &hda->chip;

	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->ops = &hda_baikal_ops;
	chip->driver_caps = driver_caps;
	chip->driver_type = driver_caps & 0xff;
	chip->dev_index = 0;
	INIT_LIST_HEAD(&chip->pcm_list);
	INIT_WORK(&hda->irq_pending_work, azx_irq_pending_work);

	chip->codec_probe_mask = 0x3; /* two codecs */

	chip->single_cmd = false;
	chip->snoop = true;

	chip->get_position[0] = chip->get_position[1] = azx_get_pos_lpib;
	chip->get_delay[0] = chip->get_delay[1] = azx_get_delay_from_lpib;

	INIT_WORK(&hda->probe_work, hda_baikal_probe_work);

	err = azx_bus_init(chip, NULL);
	if (err < 0)
		return err;

	chip->bus.core.needs_damn_long_delay = 1;
	chip->bus.core.aligned_mmio = 1;
	chip->bus.core.response_irq_broken = 1;
	chip->bus.core.baikal_codec_addr_quirk = 1;

	/* force polling mode, because RIRB interrupts don't working */
	if (of_property_read_bool(hda->dev->of_node, "cyclic-codec-probe"))
		chip->bus.core.polling_mode = 1;
	else
		chip->bus.core.polling_mode = 0;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0) {
		dev_err(card->dev, "Error creating device\n");
		return err;
	}

	return 0;
}

static int hda_baikal_probe(struct platform_device *pdev)
{
	const unsigned int driver_flags = AZX_DCAPS_PM_RUNTIME |
					  AZX_DCAPS_NO_64BIT |
					  AZX_DCAPS_4K_BDLE_BOUNDARY |
					  AZX_DCAPS_COUNT_LPIB_DELAY;
	struct snd_card *card;
	struct azx *chip;
	struct hda_baikal *hda;
	int err;

	hda = devm_kzalloc(&pdev->dev, sizeof(*hda), GFP_KERNEL);
	if (!hda)
		return -ENOMEM;
	hda->dev = &pdev->dev;
	chip = &hda->chip;

	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &card);
	if (err < 0) {
		dev_err(&pdev->dev, "Error creating card!\n");
		return err;
	}

	err = hda_baikal_create(card, driver_flags, hda);
	if (err < 0)
		goto out_free;
	card->private_data = chip;

	dev_set_drvdata(&pdev->dev, card);

	pm_runtime_enable(hda->dev);
	if (!azx_has_pm_runtime(chip))
		pm_runtime_forbid(hda->dev);

	schedule_work(&hda->probe_work);

	return 0;

out_free:
	snd_card_free(card);
	return err;
}

/*
 * Probe the given codec address
 */
static int baikal_probe_codec(struct azx *chip, int addr)
{
	unsigned int cmd = (addr << 28) | (AC_NODE_ROOT << 20) |
		(AC_VERB_PARAMETERS << 8) | AC_PAR_VENDOR_ID;
	struct hdac_bus *bus = azx_bus(chip);
	int err;
	unsigned int res = -1;

	mutex_lock(&bus->cmd_mutex);
	chip->probing = 1;
	bus->ops->command(bus, cmd);
	err = bus->ops->get_response(bus, addr, &res);
	chip->probing = 0;
	mutex_unlock(&bus->cmd_mutex);
	if (err < 0 || res == -1)
		return -EIO;
	dev_dbg(chip->card->dev, "codec #%d probed OK\n", addr);
	return 0;
}

/* Probe codecs */
static int azx_baikal_probe_codecs(struct azx *chip, unsigned int max_slots)
{
	struct hdac_bus *bus = azx_bus(chip);
	int c, codecs, err;

	int probe_retry;

	codecs = 0;
	if (!max_slots)
		max_slots = AZX_DEFAULT_CODECS;

	for (c = 0; c < max_slots; c++) {
		if ((bus->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			for (probe_retry = 0; probe_retry < 100; probe_retry++) {
				if (baikal_probe_codec(chip, c) < 0) {
					azx_stop_chip(chip);
					azx_init_chip(chip, true);
					continue;
				} else {
					dev_warn(chip->card->dev,
						"Codec #%d probe success; retry count = %d\n",
						c, probe_retry);
					break;
				}
				bus->codec_mask &= ~(1 << c);
				dev_warn(chip->card->dev,
					"Codec #%d probe error; disabling it...\n", c);
			}
		}
	}

	/* Then create codec instances */
	for (c = 0; c < max_slots; c++) {
		if ((bus->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			struct hda_codec *codec;
			err = snd_hda_codec_new(&chip->bus, chip->card, c, &codec);
			if (err < 0)
				continue;
			codec->jackpoll_interval = chip->jackpoll_interval;
			codec->beep_mode = chip->beep_mode;
			codecs++;
		}
	}
	if (!codecs) {
		dev_err(chip->card->dev, "no codecs initialized\n");
		return -ENXIO;
	}
	return 0;
}

static void hda_baikal_probe_work(struct work_struct *work)
{
	struct hda_baikal *hda = container_of(work, struct hda_baikal, 
					probe_work);
	struct azx *chip = &hda->chip;
	struct hdac_bus *bus = azx_bus(chip);
	struct platform_device *pdev = to_platform_device(hda->dev);
	int max_slots;
	int err;

	pm_runtime_get_sync(hda->dev);
	err = hda_baikal_first_init(chip, pdev);
	if (err < 0)
		goto out_free;

	switch (bus->codec_mask) {
	case 0x1:
		max_slots = 1;
		break;
	case 0x2:
		max_slots = 2;
		break;
	case 0x3:
		max_slots = 2;
		break;
	}

	/* create codec instances */
	if (of_property_read_bool(bus->dev->of_node, "cyclic-codec-probe"))
		err = azx_baikal_probe_codecs(chip, max_slots);
	else
		err = azx_probe_codecs(chip, max_slots);

	if (err < 0)
		goto out_free;

	err = azx_codec_configure(chip);
	if (err < 0)
		goto out_free;

	err = snd_card_register(chip->card);
	if (err < 0)
		goto out_free;

	chip->running = 1;

	snd_hda_set_power_save(&chip->bus, power_save * 1000);

 out_free:
	pm_runtime_put(hda->dev);
	return; /* no error return from async probe */
}

static int hda_baikal_remove(struct platform_device *pdev)
{
	int ret;

	ret = snd_card_free(dev_get_drvdata(&pdev->dev));
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static void hda_baikal_shutdown(struct platform_device *pdev)
{
	struct snd_card *card = dev_get_drvdata(&pdev->dev);
	struct azx *chip;

	if (!card)
		return;
	chip = card->private_data;
	if (chip && chip->running)
		azx_stop_chip(chip);
}

static const struct hda_controller_ops hda_baikal_ops = {
	.position_check = azx_position_check,
};

static const struct of_device_id hda_baikal_match[] = {
	{ .compatible = "be,cw-hda" },
	{},
};
MODULE_DEVICE_TABLE(of, hda_baikal_match);

static struct platform_driver baikal_platform_hda = {
	.driver = {
		.name = "baikal-hda",
		.of_match_table = hda_baikal_match,
	},
	.probe = hda_baikal_probe,
	.remove = hda_baikal_remove,
	.shutdown = hda_baikal_shutdown,
};
module_platform_driver(baikal_platform_hda);

MODULE_DESCRIPTION("Baikal HDA bus driver");
MODULE_LICENSE("GPL v2");
