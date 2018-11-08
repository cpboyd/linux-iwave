/*
 * Copyright (c) 2017 iWave Systems Technologies Pvt. Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <linux/libata.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

#define HW_OCOTP_CFGn(n)		(0x00000410 + (n) * 0x10)
#define BSP_VERSION_PREPZ		"iW-PREPZ-SC-01-R3.0-REL1.0-Linux4.1.15"
#define SOM_REV					1
#define BOM_REV					2

static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan0_en_gpio;
static int flexcan1_en_gpio;

static struct fec_platform_data fec_pdata;

static void __init imx6_iwg15_hdmi_cec_init (void)
{
	struct device_node *np;
	int sm_hdmicec_gpio;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-hdmi-cec");
	if (!np) {
		pr_warn("failed to find g15-sm-hdmi node\n");
		goto put_node;
	}
	sm_hdmicec_gpio = of_get_named_gpio(np, "hdmi-cec-sm", 0);
	if (gpio_is_valid(sm_hdmicec_gpio) &&
			!gpio_request_one(sm_hdmicec_gpio, GPIOF_DIR_OUT, "SM-HDMI-CEC")) {

			gpio_set_value(sm_hdmicec_gpio, 1);
	}
put_node:
        of_node_put(np);
}

static void __init sm_lvds_init (void)
{
	struct device_node *np;
	int lcd_rst_gpio, lcd_pwr_gpio;

	np = of_find_compatible_node(NULL, NULL, "iwave,g15-sm_lcd_pwr");
	if (!np) {
		pr_warn("failed to find g15-sm-lcd node\n");
		goto put_node;
	}

	lcd_rst_gpio = of_get_named_gpio(np, "lcd-reset", 0);
	if (gpio_is_valid(lcd_rst_gpio) &&
			!gpio_request_one(lcd_rst_gpio, GPIOF_DIR_OUT, "lcd-reset GPIO")) {

		gpio_set_value(lcd_rst_gpio, 1);
		mdelay(100);
		gpio_set_value(lcd_rst_gpio, 0);
		mdelay(100);
		gpio_set_value(lcd_rst_gpio, 1);
	}

	lcd_pwr_gpio = of_get_named_gpio(np, "lcd-power", 0);
	if (gpio_is_valid(lcd_pwr_gpio) &&
			!gpio_request_one(lcd_pwr_gpio, GPIOF_DIR_OUT, "lcd-power GPIO"))
		gpio_set_value(lcd_pwr_gpio, 1);
put_node:
	of_node_put(np);
}


static void __init imx6_iwg15_common_reset (void)
{
	struct device_node *np;
	int com_rst_gpio;

		np = of_find_compatible_node(NULL, NULL, "iwave,g15-sm-com");
		if (!np) {
			pr_warn("failed to find g15-sm-com node\n");
			goto put_node;
		}
	com_rst_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio_is_valid(com_rst_gpio) &&
			!gpio_request_one(com_rst_gpio, GPIOF_DIR_OUT, "comm-rst")) {

		gpio_set_value(com_rst_gpio, 1);
		mdelay(100);
		gpio_set_value(com_rst_gpio, 0);
		mdelay(100);
		gpio_set_value(com_rst_gpio, 1);
	}
put_node:
	of_node_put(np);
}


static int __init sm_som_revision (void)
{
        struct device_node *np;
        int i,val,err,num_ctrl;
        unsigned int *ctrl;
        short revision = 0;

        np = of_find_compatible_node(NULL, NULL, "iwave,g15-sm-com");
        if (!np) {
                pr_warn("failed to find g15-sm-com node\n");
                revision =-1;
                goto put_node;
        }

        /* Fill GPIO pin array */
        num_ctrl = of_gpio_named_count(np, "som-rev-gpios");
        if (num_ctrl <= 0) {
                pr_warn("gpios DT property empty / missing\n");
                revision =-1;
                goto put_node;
        }

	ctrl = kzalloc(num_ctrl * sizeof(unsigned int), GFP_KERNEL);
        if (!ctrl) {
                pr_warn("unable to allocate the memory\n");
                revision =-1;
                goto put_node;
        }
        for (i = 0; i < num_ctrl; i++) {
                val = of_get_named_gpio(np, "som-rev-gpios",i);

               if (val < 0) {
                        pr_warn("unable to get the gpio - %d\n", i);
                        revision =-1;
                        goto put_node;
                }

                ctrl[i] = val;
        }
        /* Request as a input GPIO and read the value */
        for (i = 0; i < num_ctrl; i++) {
		if (revision == 0x2)
			continue;
                err = gpio_request(ctrl[i],"som-rev-gpios");
                if (err){
                        pr_warn("unable to request for gpio - %d\n", i);
                        revision =-1;
                        goto put_node;
                }

                err = gpio_direction_input(ctrl[i]);
                if (err) {
                        pr_warn("unable to set gpio as input - %d\n", i);
                        revision =-1;
                        goto put_node;
                }
                revision |= gpio_get_value(ctrl[i]) << i;
        }
put_node:
        of_node_put(np);
        return revision;
}

static int __init sm_bom_revision (int som_ver)
{
        struct device_node *np1;
        int i,val,err,num_ctrl,bit_shift=0;
        unsigned *ctrl;
        short revision = 0;

	
        np1 = of_find_compatible_node(NULL, NULL, "iwave,g15-sm-com");
        if (!np1) {
                pr_warn("failed to find g15-sm-com node\n");
                revision =-1;
                goto put_node;
        }

        /* Fill GPIO pin array */
        num_ctrl = of_gpio_named_count(np1, "bom-rev-gpios");
        if (num_ctrl <= 0) {
                pr_warn("gpios DT property empty / missing\n");
                revision =-1;
                goto put_node;
        }

        ctrl = kzalloc(num_ctrl * sizeof(unsigned), GFP_KERNEL);
        if (!ctrl) {
                pr_warn("unable to allocate the memory\n");
                revision =-1;
                goto put_node;
        }
        for (i = 0; i < num_ctrl; i++) {
                val = of_get_named_gpio(np1, "bom-rev-gpios",i);

               if (val < 0) {
                        pr_warn("unable to get the gpio - %d\n", i);
                        revision =-1;
                        goto put_node;
                }

                ctrl[i] = val;
        }
        /* Request as a input GPIO and read the value */

        for (i=0; i < num_ctrl; i++) {
		/* skip one bit in case of 3.0 SOM */
		if (som_ver > 2) 
		{
			som_ver = 0;
			bit_shift=1;
			continue;
		}

                err = gpio_request(ctrl[i],"bom-rev-gpios");
                if (err){
                        pr_warn("unable to request for gpio - %d\n", i);
                        revision =-1;
                        goto put_node;
                }

                err = gpio_direction_input(ctrl[i]);
                if (err) {
                        pr_warn("unable to set gpio as input - %d\n", i);
                        revision =-1;
                        goto put_node;
                }
		if(bit_shift == 1)
			/* skip one bit in case of 3.0 SOM */
			revision |= gpio_get_value(ctrl[i]) << (i-1);
		else
			revision |= gpio_get_value(ctrl[i]) << i;

        }
put_node:
        of_node_put(np1);
        return revision;
}

static int __init print_board_info (void)
{
	struct device_node *np;
	unsigned int unique_id1, unique_id2;
	void __iomem *base;
	int som_rev;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return 0;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	unique_id1 = readl_relaxed(base + HW_OCOTP_CFGn(0));
	unique_id2 = readl_relaxed(base + HW_OCOTP_CFGn(1));

	printk ("\n");
	printk ("Board Info:\n");
	printk ("\tBSP Version     : %s\n", BSP_VERSION_PREPZ);
	som_rev = sm_som_revision()+1;	
	printk ("\tSOM version     : iW-PREPZ-AP-01-R%1x.%1x\n",som_rev,sm_bom_revision(som_rev));
	printk ("\tCPU Unique ID   : 0x%08x%08x \n", unique_id2, unique_id1);
	printk ("\n");

	iounmap(base);
put_node:
	of_node_put(np);
	return 0;
}

static void imx6_iwg15_fec_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ,
					   IMX6Q_GPR13_ENET_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static void __init imx6_iwg15_enet_plt_init(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata.sleep_mode_enable = imx6_iwg15_fec_sleep_enable;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031_phy_fixup(struct phy_device *phydev)
{
	/* prefer master mode, 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1f00);

	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	if(of_machine_is_compatible("iw,dls_iwg15m_sm") ||
                        of_machine_is_compatible("iw,qd_iwg15m_sm")) {
		mmd_write_reg(phydev, 2, 4, 0x80);
		mmd_write_reg(phydev, 2, 5, 0x7787);
		mmd_write_reg(phydev, 2, 6, 0);
		mmd_write_reg(phydev, 2, 8, 0x03ff);
	}

	return 0;
}
static void __init imx6_iwg15_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031_phy_fixup);
	}
}

static void __init imx6_iwg15_1588_init(void)
{
	struct device_node *np;
	struct clk *ptp_clk;
	struct regmap *gpr;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-fec");
	if (!np) {
		pr_warn("%s: failed to find fec node\n", __func__);
		return;
	}

	ptp_clk = of_clk_get(np, 2);
	if (IS_ERR(ptp_clk)) {
		pr_warn("%s: failed to get ptp clock\n", __func__);
		goto put_node;
	}

	/*
	 * If enet_ref from ANATOP/CCM is the PTP clock source, we need to
	 * set bit IOMUXC_GPR1[21].  Or the PTP clock must be from pad
	 * (external OSC), and we need to clear the bit.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

	clk_put(ptp_clk);
put_node:
	of_node_put(np);
}

static void __init imx6q_axi_init(void)
{
	struct regmap *gpr;
	unsigned int mask;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		/*
		 * Enable the cacheable attribute of VPU and IPU
		 * AXI transactions.
		 */
		mask = IMX6Q_GPR4_VPU_WR_CACHE_SEL |
			IMX6Q_GPR4_VPU_RD_CACHE_SEL |
			IMX6Q_GPR4_VPU_P_WR_CACHE_VAL |
			IMX6Q_GPR4_VPU_P_RD_CACHE_VAL_MASK |
			IMX6Q_GPR4_IPU_WR_CACHE_CTL |
			IMX6Q_GPR4_IPU_RD_CACHE_CTL;
		regmap_update_bits(gpr, IOMUXC_GPR4, mask, mask);

		/* Increase IPU read QoS priority */
		regmap_update_bits(gpr, IOMUXC_GPR6,
				IMX6Q_GPR6_IPU1_ID00_RD_QOS_MASK |
				IMX6Q_GPR6_IPU1_ID01_RD_QOS_MASK,
				(0xf << 16) | (0x7 << 20));
		regmap_update_bits(gpr, IOMUXC_GPR7,
				IMX6Q_GPR7_IPU2_ID00_RD_QOS_MASK |
				IMX6Q_GPR7_IPU2_ID01_RD_QOS_MASK,
				(0xf << 16) | (0x7 << 20));
	} else {
		pr_warn("failed to find fsl,imx6q-iomuxc-gpr regmap\n");
	}
}

static void __init imx6q_enet_clk_sel(void)
{
        struct regmap *gpr;

        gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
        if (!IS_ERR(gpr))
                regmap_update_bits(gpr, IOMUXC_GPR5,
                                   IMX6Q_GPR5_ENET_TX_CLK_SEL, IMX6Q_GPR5_ENET_TX_CLK_SEL);
        else
                pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init imx6_iwg15_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np, *from = NULL;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low;
	u32 macaddr_high = 0;
	u32 macaddr1_high = 0;
	u8 *macaddr;
	int i;

	for (i = 0; i < 2; i++) {
		enet_np = of_find_compatible_node(from, NULL, compatible);
		if (!enet_np)
			return;

		from = enet_np;

		if (of_get_mac_address(enet_np))
			goto put_enet_node;

		ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
		if (!ocotp_np) {
			pr_warn("failed to find ocotp node\n");
			goto put_enet_node;
		}

		base = of_iomap(ocotp_np, 0);
		if (!base) {
			pr_warn("failed to map ocotp\n");
			goto put_ocotp_node;
		}

		macaddr_low = readl_relaxed(base + OCOTP_MACn(1));
		if (i)
			macaddr1_high = readl_relaxed(base + OCOTP_MACn(2));
		else
			macaddr_high = readl_relaxed(base + OCOTP_MACn(0));

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (!newmac)
			goto put_ocotp_node;

		newmac->value = newmac + 1;
		newmac->length = 6;
		newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newmac->name) {
			kfree(newmac);
			goto put_ocotp_node;
		}

		macaddr = newmac->value;
		if (i) {
			macaddr[5] = (macaddr_low >> 16) & 0xff;
			macaddr[4] = (macaddr_low >> 24) & 0xff;
			macaddr[3] = macaddr1_high & 0xff;
			macaddr[2] = (macaddr1_high >> 8) & 0xff;
			macaddr[1] = (macaddr1_high >> 16) & 0xff;
			macaddr[0] = (macaddr1_high >> 24) & 0xff;
		} else {
			macaddr[5] = macaddr_high & 0xff;
			macaddr[4] = (macaddr_high >> 8) & 0xff;
			macaddr[3] = (macaddr_high >> 16) & 0xff;
			macaddr[2] = (macaddr_high >> 24) & 0xff;
			macaddr[1] = macaddr_low & 0xff;
			macaddr[0] = (macaddr_low >> 8) & 0xff;
		}

		of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
	}
}

static inline void imx6_iwg15_enet_init(void)
{
	imx6_iwg15_enet_mac_init("fsl,imx6q-fec");
	imx6_iwg15_enet_phy_init();
	imx6_iwg15_1588_init();
	if (cpu_is_imx6q() && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)
		imx6q_enet_clk_sel();
	imx6_iwg15_enet_plt_init();
}


static void imx6_iwg15_flexcan0_switch_auto(int enable)
{
	/* Active low enables the CAN tranceiver */
	if (enable)
		gpio_set_value_cansleep(flexcan0_en_gpio, 0);
	else
		gpio_set_value_cansleep(flexcan0_en_gpio, 1);
}

static void imx6_iwg15_flexcan1_switch_auto(int enable)
{
	/* Active low enables the CAN tranceiver */
	if (enable)
		gpio_set_value_cansleep(flexcan1_en_gpio, 0);
	else
		gpio_set_value_cansleep(flexcan1_en_gpio, 1);
}

static int __init imx6_iwg15_flexcan_fixup_auto(void)
{
	struct device_node *can0,*can1;

	can0 = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!can0)
		return -ENODEV;

	flexcan0_en_gpio = of_get_named_gpio(can0, "trx-en-gpio", 0);
	if (gpio_is_valid(flexcan0_en_gpio) &&
			!gpio_request_one(flexcan0_en_gpio, GPIOF_DIR_OUT, "flexcan0-trx-en")) {
		flexcan_pdata[0].transceiver_switch = imx6_iwg15_flexcan0_switch_auto;
	}

		can1 = of_find_node_by_path("/soc/aips-bus@02000000/can@02094000");
		if (!can1)
			return -ENODEV;

		flexcan1_en_gpio = of_get_named_gpio(can1, "trx-en-gpio", 0);
		if (gpio_is_valid(flexcan1_en_gpio) &&
				!gpio_request_one(flexcan1_en_gpio, GPIOF_DIR_OUT, "flexcan1-trx-en")) {
			flexcan_pdata[1].transceiver_switch = imx6_iwg15_flexcan1_switch_auto;
			of_node_put(can1);
		}

	return 0;
}
/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6_iwg15_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	OF_DEV_AUXDATA("fsl,imx6q-fec", 0x02188000, NULL, &fec_pdata),
	{ /* sentinel */ }
};

static void __init imx6_iwg15_init_machine(void)
{
	struct device *parent;

	if (cpu_is_imx6dl()) {
		if (num_online_cpus() == 2)
			imx_print_silicon_rev("i.MX6DL",imx_get_soc_revision());
		else
			imx_print_silicon_rev("i.MX6S",imx_get_soc_revision());
	} else if (cpu_is_imx6q()  && imx_get_soc_revision() == IMX_CHIP_REVISION_2_0) {
		if (num_online_cpus() == 4)
			imx_print_silicon_rev("i.MX6QP",IMX_CHIP_REVISION_1_0);
		else
			imx_print_silicon_rev("i.MX6DP",IMX_CHIP_REVISION_1_0);
	} else if (cpu_is_imx6q()) {
		if (num_online_cpus() == 4)
			imx_print_silicon_rev("i.MX6Q",imx_get_soc_revision());
		else
			imx_print_silicon_rev("i.MX6D",imx_get_soc_revision());
	}

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6_iwg15_auxdata_lookup, parent);

	imx6_iwg15_enet_init();
	imx_anatop_init();
	cpu_is_imx6q() ?  imx6q_pm_init() : imx6dl_pm_init();
	imx6q_axi_init();
	imx6_iwg15_common_reset();
	sm_lvds_init();
	imx6_iwg15_hdmi_cec_init();
	print_board_info();
}
#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_996MHZ		0x2
#define OCOTP_CFG3_SPEED_852MHZ		0x1

static void __init imx6_iwg15_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 996000000Hz;
	 * 2b'01: 852000000Hz; -- i.MX6Q Only, exclusive with 996MHz.
	 * 2b'00: 792000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if ((val != OCOTP_CFG3_SPEED_1P2GHZ) && cpu_is_imx6q())
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if (val < OCOTP_CFG3_SPEED_996MHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 996 MHz OPP\n");
	if (cpu_is_imx6q()) {
		if (val != OCOTP_CFG3_SPEED_852MHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 852 MHz OPP\n");
	}
	iounmap(base);

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (dev_pm_opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6_iwg15_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6_iwg15_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6_iwg15_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};

static void __init imx6_iwg15_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6_iwg15_opp_init();
		platform_device_register(&imx6_iwg15_cpufreq_pdev);
	}

	imx6_iwg15_flexcan_fixup_auto();
}

static void __init imx6_iwg15_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static void __init imx6_iwg15_init_irq(void)
{
	imx_gpc_check_dt();
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	irqchip_init();
}

static const char * const imx6_iwg15_dt_compat[] __initconst = {
	"iw,qd_iwg15m_sm",
	"iw,dls_iwg15m_sm",
	NULL,
};

DT_MACHINE_START(IMX6_IWG15, "iW-RainboW-G15 platform based on i.MX6 (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6_iwg15_map_io,
	.init_irq	= imx6_iwg15_init_irq,
	.init_machine	= imx6_iwg15_init_machine,
	.init_late      = imx6_iwg15_init_late,
	.dt_compat	= imx6_iwg15_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
