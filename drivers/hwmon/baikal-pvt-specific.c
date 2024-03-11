// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 BAIKAL ELECTRONICS, JSC
 *
 * Authors:
 *   Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>
 *   Serge Semin <Sergey.Semin@baikalelectronics.ru>
 *
 * Baikal-M/S Process, Voltage, Temperature sensor driver
 */
#include "baikal-pvt-common.c"

static int type_cpu = 10;
enum chips { bm1000, bs1000, chips_number };
/*
 * The original translation formulae of the temperature (in degrees of Celsius)
 * to PVT data and vice-versa are following:
 * N = 1.8322e-8*(T^4) + 2.343e-5*(T^3) + 8.7018e-3*(T^2) + 3.9269*(T^1) +
 *     1.7204e2,
 * T = -1.6743e-11*(N^4) + 8.1542e-8*(N^3) + -1.8201e-4*(N^2) +
 *     3.1020e-1*(N^1) - 4.838e1,
 * where T = [-48.380, 147.438]C and N = [0, 1023].
 * They must be accordingly altered to be suitable for the integer arithmetics.
 * The technique is called 'factor redistribution', which just makes sure the
 * multiplications and divisions are made so to have a result of the operations
 * within the integer numbers limit. In addition we need to translate the
 * formulae to accept millidegrees of Celsius. Here what they look like after
 * the alterations:
 * N = (18322e-20*(T^4) + 2343e-13*(T^3) + 87018e-9*(T^2) + 39269e-3*T +
 *     17204e2) / 1e4,
 * T = -16743e-12*(D^4) + 81542e-9*(D^3) - 182010e-6*(D^2) + 310200e-3*D -
 *     48380,
 * where T = [-48380, 147438] mC and N = [0, 1023].
 */

static const struct polynomial bm_poly_temp_to_N = {
	.total_divider = 10000,
	.terms = {
		{4,   18322, 10000, 10000},
		{3,    2343, 10000,    10},
		{2,   87018, 10000,    10},
		{1,   39269,  1000,     1},
		{0, 1720400,     1,     1}
	}
};

static const struct polynomial bs_poly_temp_to_N = {
	.total_divider = 10000,
	.terms = {
		{4,   12569, 10000, 10000},
		{3,    2476, 10000,    10},
		{2,   66309, 10000,    10},
		{1,   36384,  1000,     1},
		{0, 2070500,     1,     1}
	}
};

static const struct polynomial bm_poly_N_to_temp = {
	.total_divider = 1,
	.terms = {
		{4,  -16743, 1000, 1},
		{3,   81542, 1000, 1},
		{2, -182010, 1000, 1},
		{1,  310200, 1000, 1},
		{0,  -48380,    1, 1}
	}
};

static const struct polynomial bs_poly_N_to_temp = {
	.total_divider = 1,
	.terms = {
		{4,   16034, 1000, 1},
		{3,   15608, 1000, 1},
		{2, -150890, 1000, 1},
		{1,  334080, 1000, 1},
		{0,  -62861,    1, 1}
	}
};

/*
 * Similar alterations are performed for the voltage conversion equations.
 * The original formulae are:
 * N = 1.8658e3*V - 1.1572e3,
 * V = (N + 1.1572e3) / 1.8658e3,
 * where V = [0.620, 1.168] V and N = [0, 1023].
 * After the optimization they looks as follows:
 * N = (18658e-3*V - 11572) / 10,
 * V = N * 10^5 / 18658 + 11572 * 10^4 / 18658.
 */
static const struct polynomial bm_poly_volt_to_N = {
	.total_divider = 10,
	.terms = {
		{1,  18658, 1000, 1},
		{0, -11572,    1, 1}
	}
};

static const struct polynomial bs_poly_volt_to_N = {
	.total_divider = 10,
	.terms = {
		{1, 16757, 1000, 1},
		{0, -8564,    1, 1}
	}
};

static const struct polynomial bm_poly_N_to_volt = {
	.total_divider = 10,
	.terms = {
		{1,    100000, 18658,     1},
		{0, 115720000,     1, 18658}
	}
};

static const struct polynomial bs_poly_N_to_volt = {
	.total_divider = 10,
	.terms = {
		{1,   100000, 16757,     1},
		{0, 85639000,     1, 16757}
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

u32 writel_pvt(u32 val, u32 pvt_base, u32 offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, PVT_WRITE, (unsigned long)pvt_base, offset, val,
		      0, 0, 0, &res);
	return res.a0;
}

u32 readl_pvt(u32 pvt_base, u32 offset)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BAIKAL_SMC_PVT_CMD, PVT_READ, (unsigned long)pvt_base, offset,
		      0, 0, 0, 0, &res);
	return res.a0;
}

void set_pvt_base(struct pvt_hwmon *pvt, struct resource *res)
{
	pvt->base = res->start;
	pvt->regs = (void *)(uintptr_t)pvt->base;
}

const struct polynomial *get_polynomial_N_to_temp(void)
{
	if (type_cpu == bm1000)
		return &bm_poly_N_to_temp;
	else if (type_cpu == bs1000)
		return &bs_poly_N_to_temp;
	else
		return NULL;
}

const struct polynomial *get_polynomial_N_to_volt(void)
{
	if (type_cpu == bm1000)
		return &bm_poly_N_to_volt;
	else if (type_cpu == bs1000)
		return &bs_poly_N_to_volt;
	else
		return NULL;
}

const struct polynomial *get_polynomial_temp_to_N(void)
{
	if (type_cpu == bm1000)
		return &bm_poly_temp_to_N;
	else if (type_cpu == bs1000)
		return &bs_poly_temp_to_N;
	else
		return NULL;
}

const struct polynomial *get_polynomial_volt_to_N(void)
{
	if (type_cpu == bm1000)
		return &bm_poly_volt_to_N;
	else if (type_cpu == bs1000)
		return &bs_poly_volt_to_N;
	else
		return NULL;
}

int pvt_check_addr(struct pvt_hwmon *pvt)
{
	int base;
	struct platform_device *pdev = to_platform_device(pvt->dev);

	type_cpu = (enum chips)device_get_match_data(&pdev->dev);
	base = pvt->base;

	if (type_cpu == bm1000 &&
		(base == MMCA57_0_PVT_BASE  ||
		 base == MMCA57_1_PVT_BASE  ||
		 base == MMCA57_2_PVT_BASE  ||
		 base == MMCA57_3_PVT_BASE  ||
		 base == MMMALI_PVT_BASE))
		return 0;
	else if (type_cpu == bs1000 &&
		(base == CA75_0_PVT_BASE    ||
		 base == CA75_1_PVT_BASE    ||
		 base == CA75_2_PVT_BASE    ||
		 base == CA75_3_PVT_BASE    ||
		 base == CA75_4_PVT_BASE    ||
		 base == CA75_5_PVT_BASE    ||
		 base == CA75_6_PVT_BASE    ||
		 base == CA75_7_PVT_BASE    ||
		 base == CA75_8_PVT_BASE    ||
		 base == CA75_9_PVT_BASE    ||
		 base == CA75_10_PVT_BASE   ||
		 base == CA75_11_PVT_BASE   ||
		 base == DDR0_PVT_BASE	    ||
		 base == DDR1_PVT_BASE	    ||
		 base == DDR2_PVT_BASE	    ||
		 base == DDR3_PVT_BASE	    ||
		 base == DDR4_PVT_BASE	    ||
		 base == DDR5_PVT_BASE	    ||
		 base == PCIE0_PVT_BASE	    ||
		 base == PCIE1_PVT_BASE	    ||
		 base == PCIE2_PVT_BASE	    ||
		 base == PCIE3_PVT_BASE	    ||
		 base == PCIE4_PVT_BASE))
		return 0;

	dev_err(&pdev->dev, "base address: 0x%x\n", base);
	dev_err(&pdev->dev, "    type cpu: %d\n", type_cpu);
	return -EINVAL;
}

int is_interrupt_present(struct pvt_hwmon *pvt)
{
	if (type_cpu == bm1000)
		return 1;
	else if (type_cpu == bs1000)
		switch (pvt->base) {
		/*
		 * There is some hardware problems with our BE-S board,
		 * that's why interrupts for PCIE are temporarily disabled
		 */

		/*
		 * case PCIE0_PVT_BASE:
		 * case PCIE1_PVT_BASE:
		 * case PCIE2_PVT_BASE:
		 * case PCIE3_PVT_BASE:
		 * case PCIE4_PVT_BASE:
		 */
		case DDR0_PVT_BASE:
		case DDR1_PVT_BASE:
		case DDR2_PVT_BASE:
		case DDR3_PVT_BASE:
		case DDR4_PVT_BASE:
		case DDR5_PVT_BASE: return 1;
		default: return 0;
		}
	else
		return -1;
}

static const struct of_device_id pvt_of_match[] = {
	{ .compatible = "baikal,bs1000-pvt", .data = (void *)bs1000 },
	{ .compatible = "baikal,bm1000-pvt", .data = (void *)bm1000 },
	{ }
};
MODULE_DEVICE_TABLE(of, pvt_of_match);

static struct platform_driver pvt_driver = {
	.probe = pvt_probe,
	.driver = {
		.name = "baikal,baikal-pvt",
		.of_match_table = pvt_of_match
	}
};
module_platform_driver(pvt_driver);

MODULE_AUTHOR("Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal-T1 PVT driver");
MODULE_LICENSE("GPL v2");
