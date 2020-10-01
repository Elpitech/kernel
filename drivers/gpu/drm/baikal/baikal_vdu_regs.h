/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * Parts of this file were based on sources as follows:
 *
 *   David A Rusling
 *   Copyright (C) 2001 ARM Limited
 */

#ifndef __BAIKAL_VDU_REGS_H__
#define __BAIKAL_VDU_REGS_H__

#define CR1         0x000
#define HTR         0x008
#define VTR1        0x00C
#define VTR2        0x010
#define PCTR        0x014
#define ISR         0x018
#define IMR         0x01C
#define IVR         0x020
#define ISCR        0x024
#define DBAR        0x028
#define DCAR        0x02C
#define DEAR        0x030
#define HVTER       0x044
#define HPPLOR      0x048
#define GPIOR       0x1F8
#define OWER        0x600
#define OWXSER0     0x604
#define OWYSER0     0x608
#define OWDBAR0     0x60C
#define OWDCAR0     0x610
#define OWDEAR0     0x614
#define OWXSER1     0x618
#define OWYSER1     0x61C
#define OWDBAR1     0x620
#define OWDCAR1     0x624
#define OWDEAR1     0x628
#define MRR         0xFFC

#define INTR_BAU    BIT(7)
#define INTR_VCT    BIT(6)
#define INTR_MBE    BIT(5)
#define INTR_FER    BIT(4)

#define CR1_FBP             BIT(19)
#define CR1_FDW_4_WORDS     (0 << 16)
#define CR1_FDW_8_WORDS     (1 << 16)
#define CR1_FDW_16_WORDS    (2 << 16)
#define CR1_OPS_LCD18       (0 << 13)
#define CR1_OPS_LCD24       (1 << 13)
#define CR1_OPS_565         (0 << 12)
#define CR1_OPS_555         (1 << 12)
#define CR1_VSP             BIT(11)
#define CR1_HSP             BIT(10)
#define CR1_DEP             BIT(8)
#define CR1_BGR             BIT(5)
#define CR1_BPP_MASK        GENMASK(4, 2)
#define CR1_BPP1            (0 << 2)
#define CR1_BPP2            (1 << 2)
#define CR1_BPP4            (2 << 2)
#define CR1_BPP8            (3 << 2)
#define CR1_BPP16           (4 << 2)
#define CR1_BPP18           (5 << 2)
#define CR1_BPP24           (6 << 2)
#define CR1_LCE             BIT(0)

#define CR1_BPP16_555 ((CR1_BPP16) | (CR1_OPS_555))
#define CR1_BPP16_565 ((CR1_BPP16) | (CR1_OPS_565))

#define VTR1_VBP_MASK       GENMASK(23, 16)
#define VTR1_VBP(x)         ((x) << 16)
#define VTR1_VBP_LSB_WIDTH  8
#define VTR1_VFP_MASK       GENMASK(15, 8)
#define VTR1_VFP(x)         ((x) << 8)
#define VTR1_VFP_LSB_WIDTH  8
#define VTR1_VSW_MASK       GENMASK(7, 0)
#define VTR1_VSW(x)         ((x) << 0)
#define VTR1_VSW_LSB_WIDTH  8

#define VTR2_LPP_MASK       GENMASK(11, 0)

#define HTR_HSW_MASK        GENMASK(31, 24)
#define HTR_HSW(x)          ((x) << 24)
#define HTR_HSW_LSB_WIDTH   8
#define HTR_HBP_MASK        GENMASK(23, 16)
#define HTR_HBP(x)          ((x) << 16)
#define HTR_HBP_LSB_WIDTH   8
#define HTR_PPL_MASK        GENMASK(15, 8)
#define HTR_PPL(x)          ((x) << 8)
#define HTR_HFP_MASK        GENMASK(7, 0)
#define HTR_HFP(x)          ((x) << 0)
#define HTR_HFP_LSB_WIDTH   8

#define PCTR_PCI2           BIT(11)
#define PCTR_PCR            BIT(10)
#define PCTR_PCI            BIT(9)
#define PCTR_PCB            BIT(8)
#define PCTR_PCD_MASK       GENMASK(7, 0)
#define PCTR_MAX_PCD        128

#define ISCR_VSC_OFF        0x0
#define ISCR_VSC_VSW        0x4
#define ISCR_VSC_VBP        0x5
#define ISCR_VSC_VACTIVE    0x6
#define ISCR_VSC_VFP        0x7

#define HVTER_VSWE_MASK     GENMASK(25, 24)
#define HVTER_VSWE(x)       ((x) << 24)
#define HVTER_HSWE_MASK     GENMASK(17, 16)
#define HVTER_HSWE(x)       ((x) << 16)
#define HVTER_VBPE_MASK     GENMASK(13, 12)
#define HVTER_VBPE(x)       ((x) << 12)
#define HVTER_VFPE_MASK     GENMASK(9, 8)
#define HVTER_VFPE(x)       ((x) << 8)
#define HVTER_HBPE_MASK     GENMASK(5, 4)
#define HVTER_HBPE(x)       ((x) << 4)
#define HVTER_HFPE_MASK     GENMASK(1, 0)
#define HVTER_HFPE(x)       ((x) << 0)

#define HPPLOR_HPOE         BIT(31)
#define HPPLOR_HPPLO_MASK   GENMASK(11, 0)
#define HPPLOR_HPPLO(x)     ((x) << 0)

#define GPIOR_UHD_MASK      GENMASK(23, 16)
#define GPIOR_UHD_FMT_LDI   (0 << 20)
#define GPIOR_UHD_FMT_VESA  (1 << 20)
#define GPIOR_UHD_FMT_JEIDA (2 << 20)
#define GPIOR_UHD_SNGL_PORT (0 << 18)
#define GPIOR_UHD_DUAL_PORT (1 << 18)
#define GPIOR_UHD_QUAD_PORT (2 << 18)
#define GPIOR_UHD_ENB       BIT(17)
#define GPIOR_UHD_PIX_INTLV (0 << 16)
#define GPIOR_UHD_PIX_SQNTL (1 << 16)

#define MRR_DEAR_MRR_MASK   GENMASK(31, 3)
#define MRR_OUTSTND_RQ_MASK GENMASK(2, 0)
#define MRR_OUTSTND_RQ(x)   ((x >> 1) << 0)

#endif /* __BAIKAL_VDU_REGS_H__ */
