// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)

#include <linux/pci-ecam.h>

#if defined(CONFIG_PCIE_BAIKAL)
extern struct pci_ecam_ops baikal_pcie_ecam_ops;
#else
struct pci_ecam_ops baikal_pcie_ecam_ops = {
	.bus_shift	= 20,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif

#endif
