// SPDX-License-Identifier: GPL-2.0+
/*
 * ipmi_si_pci.c
 *
 * Handling for IPMI devices on the PCI bus.
 */

#define pr_fmt(fmt) "ipmi_pci: " fmt

#include <linux/module.h>
#include <linux/pci.h>
#include "ipmi_si.h"

static bool pci_registered;

static bool si_trypci = true;

module_param_named(trypci, si_trypci, bool, 0);
MODULE_PARM_DESC(trypci,
		 "Setting this to zero will disable the default scan of the interfaces identified via pci");

#define PCI_DEVICE_ID_HP_MMC 0x121A

#define PCI_VENDOR_ID_ASPEED 0x1a03
#define PCI_DEVICE_ID_ASPEED_BMC 0x2402

static int ipmi_pci_probe_regspacing(struct si_sm_io *io)
{
	if (io->si_type == SI_KCS) {
		unsigned char	status;
		int		regspacing;

		io->regsize = DEFAULT_REGSIZE;
		io->regshift = 0;

		/* detect 1, 4, 16byte spacing */
		for (regspacing = DEFAULT_REGSPACING; regspacing <= 16;) {
			io->regspacing = regspacing;
			if (io->io_setup(io)) {
				dev_err(io->dev, "Could not setup I/O space\n");
				return DEFAULT_REGSPACING;
			}
			/* write invalid cmd */
			io->outputb(io, 1, 0x10);
			/* read status back */
			status = io->inputb(io, 1);
			io->io_cleanup(io);
			if (status)
				return regspacing;
			regspacing *= 4;
		}
	}
	return DEFAULT_REGSPACING;
}

static struct pci_device_id ipmi_pci_blacklist[] = {
	/*
	 * This is a "Virtual IPMI device", whatever that is.  It appears
	 * as a KCS device by the class, but it is not one.
	 */
	{ PCI_VDEVICE(REALTEK, 0x816c) },
	{ 0, }
};

static int ipmi_pci_aspeed_probe(struct pci_dev *pdev,
					struct si_sm_io *io)
{
	int rv;
	unsigned long addr_data;
	void *map;
	static int channel_offsets[4] =
		{0x3a0 * 4, 0x3a8 * 4, 0x3a2 * 4, 0x3a4 * 4};

	rv = pcim_enable_device(pdev);
	if (rv) {
		dev_err(&pdev->dev, "couldn't enable PCI device\n");
		return rv;
	}

	io->si_type = SI_KCS;
	io->addr_space = IPMI_MEM_ADDR_SPACE;
	io->io_setup = ipmi_si_mem_setup;
	addr_data = pci_resource_start(pdev, 1);

	io->dev = &pdev->dev;

	io->regspacing = 4;
	io->regsize = 4;
	io->regshift = 0;

	dev_info(&pdev->dev, "%pR regsize %d spacing %d irq %d\n",
		 &pdev->resource[1], io->regsize, io->regspacing, io->irq);

	/* First read returns 0xFF... Do it now before detection. */
	map = ioremap(addr_data, PAGE_SIZE);
	if (IS_ERR_OR_NULL(map)) {
		dev_err(&pdev->dev, "ioremap error: %lx\n", PTR_ERR(map));
		return PTR_ERR(map);
	}
	rv = readl(map + channel_offsets[0]);
	dev_info(&pdev->dev, "status reg = %x\n", rv);
	iounmap(map);

	io->addr_data = addr_data + channel_offsets[0];
	io->slave_addr = 1;
	rv = ipmi_si_add_smi(io);

	return rv;
}

static int ipmi_pci_probe(struct pci_dev *pdev,
				    const struct pci_device_id *ent)
{
	int rv;
	struct si_sm_io io;

	if (pci_match_id(ipmi_pci_blacklist, pdev))
		return -ENODEV;

	memset(&io, 0, sizeof(io));
	io.addr_source = SI_PCI;
	dev_info(&pdev->dev, "probing via PCI");

	if (pdev->vendor == PCI_VENDOR_ID_ASPEED)
		return ipmi_pci_aspeed_probe(pdev, &io);

	switch (pdev->class) {
	case PCI_CLASS_SERIAL_IPMI_SMIC:
		io.si_type = SI_SMIC;
		break;

	case PCI_CLASS_SERIAL_IPMI_KCS:
		io.si_type = SI_KCS;
		break;

	case PCI_CLASS_SERIAL_IPMI_BT:
		io.si_type = SI_BT;
		break;

	default:
		dev_info(&pdev->dev, "Unknown IPMI class: %x\n", pdev->class);
		return -ENOMEM;
	}

	rv = pcim_enable_device(pdev);
	if (rv) {
		dev_err(&pdev->dev, "couldn't enable PCI device\n");
		return rv;
	}

	if (pci_resource_flags(pdev, 0) & IORESOURCE_IO) {
		io.addr_space = IPMI_IO_ADDR_SPACE;
		io.io_setup = ipmi_si_port_setup;
	} else {
		io.addr_space = IPMI_MEM_ADDR_SPACE;
		io.io_setup = ipmi_si_mem_setup;
	}
	io.addr_data = pci_resource_start(pdev, 0);

	io.dev = &pdev->dev;

	io.regspacing = ipmi_pci_probe_regspacing(&io);
	io.regsize = DEFAULT_REGSIZE;
	io.regshift = 0;

	io.irq = pdev->irq;
	if (io.irq)
		io.irq_setup = ipmi_std_irq_setup;

	dev_info(&pdev->dev, "%pR regsize %d spacing %d irq %d\n",
		 &pdev->resource[0], io.regsize, io.regspacing, io.irq);

	return ipmi_si_add_smi(&io);
}

static void ipmi_pci_remove(struct pci_dev *pdev)
{
	ipmi_si_remove_by_dev(&pdev->dev);
}

static const struct pci_device_id ipmi_pci_devices[] = {
	{ PCI_VDEVICE(HP, PCI_DEVICE_ID_HP_MMC) },
	{ PCI_VDEVICE(ASPEED, PCI_DEVICE_ID_ASPEED_BMC) },
	{ PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_IPMI_SMIC, ~0) },
	{ PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_IPMI_KCS, ~0) },
	{ PCI_DEVICE_CLASS(PCI_CLASS_SERIAL_IPMI_BT, ~0) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ipmi_pci_devices);

static struct pci_driver ipmi_pci_driver = {
	.name =         SI_DEVICE_NAME,
	.id_table =     ipmi_pci_devices,
	.probe =        ipmi_pci_probe,
	.remove =       ipmi_pci_remove,
};

void ipmi_si_pci_init(void)
{
	if (si_trypci) {
		int rv = pci_register_driver(&ipmi_pci_driver);
		if (rv)
			pr_err("Unable to register PCI driver: %d\n", rv);
		else
			pci_registered = true;
	}
}

void ipmi_si_pci_shutdown(void)
{
	if (pci_registered)
		pci_unregister_driver(&ipmi_pci_driver);
}
