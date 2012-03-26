/*
 *  LILLY-1131 module support
 *
 *    Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *
 *  based on code for other MX31 boards,
 *
 *    Copyright 2005-2007 Freescale Semiconductor
 *    Copyright (c) 2009 Alberto Panizzo <maramaopercheseimorto@gmail.com>
 *    Copyright (C) 2009 Valentin Longchamp, EPFL Mobots group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/smsc911x.h>
#include <linux/mtd/physmap.h>
#include <linux/spi/spi.h>
#include <linux/mfd/mc13783.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-mx3.h>
#include <mach/board-mx31lilly.h>
#include <mach/spi.h>
#include <mach/mxc_ehci.h>
#include <mach/ulpi.h>

#include "devices.h"

/*
 * This file contains module-specific initialization routines for LILLY-1131.
 * Initialization of peripherals found on the baseboard is implemented in the
 * appropriate baseboard support code.
 */

/* SMSC ethernet support */

static struct resource smsc91x_resources[] = {
	{
		.start	= MX31_CS4_BASE_ADDR,
		.end	= MX31_CS4_BASE_ADDR + 0xffff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IOMUX_TO_IRQ(MX31_PIN_GPIO1_0),
		.end	= IOMUX_TO_IRQ(MX31_PIN_GPIO1_0),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_FALLING,
	}
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags		= SMSC911X_USE_32BIT |
			  SMSC911X_SAVE_MAC_ADDRESS |
			  SMSC911X_FORCE_INTERNAL_PHY,
};

static struct platform_device smsc91x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc91x_resources),
	.resource	= smsc91x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	}
};

/* NOR flash */
static struct physmap_flash_data nor_flash_data = {
	.width  = 2,
};

static struct resource nor_flash_resource = {
	.start	= 0xa0000000,
	.end	= 0xa1ffffff,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device physmap_flash_device = {
	.name	= "physmap-flash",
	.id	= 0,
	.dev	= {
		.platform_data  = &nor_flash_data,
	},
	.resource = &nor_flash_resource,
	.num_resources = 1,
};

/* USB */

#if defined(CONFIG_USB_ULPI)

#define USB_PAD_CFG (PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST | PAD_CTL_HYS_CMOS | \
			PAD_CTL_ODE_CMOS | PAD_CTL_100K_PU)

static int usbotg_init(struct platform_device *pdev)
{
	unsigned int pins[] = {
		MX31_PIN_USBOTG_DATA0__USBOTG_DATA0,
		MX31_PIN_USBOTG_DATA1__USBOTG_DATA1,
		MX31_PIN_USBOTG_DATA2__USBOTG_DATA2,
		MX31_PIN_USBOTG_DATA3__USBOTG_DATA3,
		MX31_PIN_USBOTG_DATA4__USBOTG_DATA4,
		MX31_PIN_USBOTG_DATA5__USBOTG_DATA5,
		MX31_PIN_USBOTG_DATA6__USBOTG_DATA6,
		MX31_PIN_USBOTG_DATA7__USBOTG_DATA7,
		MX31_PIN_USBOTG_CLK__USBOTG_CLK,
		MX31_PIN_USBOTG_DIR__USBOTG_DIR,
		MX31_PIN_USBOTG_NXT__USBOTG_NXT,
		MX31_PIN_USBOTG_STP__USBOTG_STP,
	};

	mxc_iomux_setup_multiple_pins(pins, ARRAY_SIZE(pins), "USB OTG");

	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA0, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA1, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA2, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA3, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA4, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA5, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA6, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DATA7, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_CLK, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_DIR, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_NXT, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBOTG_STP, USB_PAD_CFG);

	mxc_iomux_set_gpr(MUX_PGP_USB_4WIRE, true);
	mxc_iomux_set_gpr(MUX_PGP_USB_COMMON, true);

	/* chip select */
	mxc_iomux_alloc_pin(IOMUX_MODE(MX31_PIN_DTR_DCE2, IOMUX_CONFIG_GPIO),
				"USBOTG_CS");
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_DTR_DCE2), "USBH1 CS");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_DTR_DCE2), 0);

	return 0;
}

static int usbh1_init(struct platform_device *pdev)
{
	int pins[] = {
		MX31_PIN_CSPI1_MOSI__USBH1_RXDM,
		MX31_PIN_CSPI1_MISO__USBH1_RXDP,
		MX31_PIN_CSPI1_SS0__USBH1_TXDM,
		MX31_PIN_CSPI1_SS1__USBH1_TXDP,
		MX31_PIN_CSPI1_SS2__USBH1_RCV,
		MX31_PIN_CSPI1_SCLK__USBH1_OEB,
		MX31_PIN_CSPI1_SPI_RDY__USBH1_FS,
	};

	mxc_iomux_setup_multiple_pins(pins, ARRAY_SIZE(pins), "USB H1");

	mxc_iomux_set_pad(MX31_PIN_CSPI1_MOSI, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_MISO, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS0, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS1, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SS2, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SCLK, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_CSPI1_SPI_RDY, USB_PAD_CFG);

	mxc_iomux_set_gpr(MUX_PGP_USB_SUSPEND, true);

	return 0;
}

static int usbh2_init(struct platform_device *pdev)
{
	int pins[] = {
		MX31_PIN_USBH2_DATA0__USBH2_DATA0,
		MX31_PIN_USBH2_DATA1__USBH2_DATA1,
		MX31_PIN_USBH2_CLK__USBH2_CLK,
		MX31_PIN_USBH2_DIR__USBH2_DIR,
		MX31_PIN_USBH2_NXT__USBH2_NXT,
		MX31_PIN_USBH2_STP__USBH2_STP,
	};

	mxc_iomux_setup_multiple_pins(pins, ARRAY_SIZE(pins), "USB H2");

	mxc_iomux_set_pad(MX31_PIN_USBH2_CLK, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBH2_DIR, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBH2_NXT, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBH2_STP, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA0, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_USBH2_DATA1, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SRXD6, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_STXD6, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SFS3, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SCK3, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_SRXD3, USB_PAD_CFG);
	mxc_iomux_set_pad(MX31_PIN_STXD3, USB_PAD_CFG);

	mxc_iomux_set_gpr(MUX_PGP_UH2, true);

	/* chip select */
	mxc_iomux_alloc_pin(IOMUX_MODE(MX31_PIN_DTR_DCE1, IOMUX_CONFIG_GPIO),
				"USBH2_CS");
	gpio_request(IOMUX_TO_GPIO(MX31_PIN_DTR_DCE1), "USBH2 CS");
	gpio_direction_output(IOMUX_TO_GPIO(MX31_PIN_DTR_DCE1), 0);

	return 0;
}

static struct mxc_usbh_platform_data usbotg_pdata = {
	.init	= usbotg_init,
	.portsc	= MXC_EHCI_MODE_ULPI | MXC_EHCI_UTMI_8BIT,
	.flags	= MXC_EHCI_POWER_PINS_ENABLED,
};

static struct mxc_usbh_platform_data usbh1_pdata = {
	.init	= usbh1_init,
	.portsc	= MXC_EHCI_MODE_UTMI | MXC_EHCI_SERIAL,
	.flags	= MXC_EHCI_POWER_PINS_ENABLED | MXC_EHCI_INTERFACE_SINGLE_UNI,
};

static struct mxc_usbh_platform_data usbh2_pdata = {
	.init	= usbh2_init,
	.portsc	= MXC_EHCI_MODE_ULPI | MXC_EHCI_UTMI_8BIT,
	.flags	= MXC_EHCI_POWER_PINS_ENABLED,
};

static void lilly1131_usb_init(void)
{
	usbotg_pdata.otg = otg_ulpi_create(&mxc_ulpi_access_ops,
				ULPI_OTG_DRVVBUS | ULPI_OTG_DRVVBUS_EXT);
	usbh2_pdata.otg = otg_ulpi_create(&mxc_ulpi_access_ops,
				ULPI_OTG_DRVVBUS | ULPI_OTG_DRVVBUS_EXT);

	mxc_register_device(&mxc_usbh1, &usbh1_pdata);
	mxc_register_device(&mxc_usbh2, &usbh2_pdata);
}

#else
static inline void lilly1131_usb_init(void) {}
#endif /* CONFIG_USB_ULPI */

/* SPI */

static int spi_internal_chipselect[] = {
	MXC_SPI_CS(0),
	MXC_SPI_CS(1),
	MXC_SPI_CS(2),
};

static struct spi_imx_master spi0_pdata = {
	.chipselect = spi_internal_chipselect,
	.num_chipselect = ARRAY_SIZE(spi_internal_chipselect),
};

static struct spi_imx_master spi1_pdata = {
	.chipselect = spi_internal_chipselect,
	.num_chipselect = ARRAY_SIZE(spi_internal_chipselect),
};

static struct mc13783_platform_data mc13783_pdata __initdata = {
	.flags = MC13783_USE_RTC | MC13783_USE_TOUCHSCREEN,
};

static struct spi_board_info mc13783_dev __initdata = {
	.modalias	= "mc13783",
	.max_speed_hz	= 1000000,
	.bus_num	= 1,
	.chip_select	= 0,
	.platform_data	= &mc13783_pdata,
};

static struct platform_device *devices[] __initdata = {
	&smsc91x_device,
	&physmap_flash_device,
};

static int mx31lilly_baseboard;
core_param(mx31lilly_baseboard, mx31lilly_baseboard, int, 0444);

static void __init mx31lilly_board_init(void)
{
	switch (mx31lilly_baseboard) {
	case MX31LILLY_NOBOARD:
		break;
	case MX31LILLY_DB:
		mx31lilly_db_init();
		break;
	default:
		printk(KERN_ERR "Illegal mx31lilly_baseboard type %d\n",
			mx31lilly_baseboard);
	}

	mxc_iomux_alloc_pin(MX31_PIN_CS4__CS4, "Ethernet CS");

	/* SPI */
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_SCLK__SCLK, "SPI1_CLK");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_MOSI__MOSI, "SPI1_TX");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_MISO__MISO, "SPI1_RX");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_SPI_RDY__SPI_RDY, "SPI1_RDY");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_SS0__SS0, "SPI1_SS0");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_SS1__SS1, "SPI1_SS1");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI1_SS2__SS2, "SPI1_SS2");

	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_SCLK__SCLK, "SPI2_CLK");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_MOSI__MOSI, "SPI2_TX");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_MISO__MISO, "SPI2_RX");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_SPI_RDY__SPI_RDY, "SPI2_RDY");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_SS0__SS0, "SPI2_SS0");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_SS1__SS1, "SPI2_SS1");
	mxc_iomux_alloc_pin(MX31_PIN_CSPI2_SS2__SS2, "SPI2_SS2");

	mxc_register_device(&mxc_spi_device0, &spi0_pdata);
	mxc_register_device(&mxc_spi_device1, &spi1_pdata);
	spi_register_board_info(&mc13783_dev, 1);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* USB */
	lilly1131_usb_init();
}

static void __init mx31lilly_timer_init(void)
{
	mx31_clocks_init(26000000);
}

static struct sys_timer mx31lilly_timer = {
	.init	= mx31lilly_timer_init,
};

MACHINE_START(LILLY1131, "INCO startec LILLY-1131")
	.phys_io	= MX31_AIPS1_BASE_ADDR,
	.io_pg_offst	= (MX31_AIPS1_BASE_ADDR_VIRT >> 18) & 0xfffc,
	.boot_params	= MX3x_PHYS_OFFSET + 0x100,
	.map_io		= mx31_map_io,
	.init_irq	= mx31_init_irq,
	.init_machine	= mx31lilly_board_init,
	.timer		= &mx31lilly_timer,
MACHINE_END

