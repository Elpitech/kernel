// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Baikal Electronics, JSC
 * Author: Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)

#include <linux/pci-ecam.h>

#if defined(CONFIG_PCIE_BAIKAL)
extern const struct pci_ecam_ops baikal_m_pcie_ecam_ops;
extern const struct pci_ecam_ops baikal_s_pcie_ecam_ops;
#else
const struct pci_ecam_ops baikal_m_pcie_ecam_ops = {
	.bus_shift	= 20,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};

const struct pci_ecam_ops baikal_s_pcie_ecam_ops = baikal_m_pcie_ecam_ops;
#endif

#endif
