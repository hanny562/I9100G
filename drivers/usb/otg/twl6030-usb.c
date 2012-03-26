/*
 * twl6030_usb - TWL6030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Contact: Felipe Balbi <felipe.balbi@nokia.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 *	- HS USB ULPI mode works.
 *	- 3-pin mode support may be added in future.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/usb/composite.h>
#include <plat/microusbic.h>

/* usb register definitions*/
#define USB_VENDOR_ID_LSB		0x00
#define USB_VENDOR_ID_MSB		0x01
#define USB_PRODUCT_ID_LSB		0x02
#define USB_PRODUCT_ID_MSB		0x03
#define USB_VBUS_CTRL_SET		0x04
#define USB_VBUS_CTRL_CLR		0x05
#define USB_ID_CTRL_SET			0x06
#define USB_ID_CTRL_CLR			0x07
#define USB_VBUS_INT_SRC		0x08
#define USB_VBUS_INT_LATCH_SET		0x09
#define USB_VBUS_INT_LATCH_CLR		0x0A
#define USB_VBUS_INT_EN_LO_SET		0x0B
#define USB_VBUS_INT_EN_LO_CLR		0x0C
#define USB_VBUS_INT_EN_HI_SET		0x0D
#define USB_VBUS_INT_EN_HI_CLR		0x0E
#define USB_ID_INT_SRC			0x0F
#define ID_GND				(1 << 0)
#define ID_FLOAT			(1 << 4)

#define USB_ID_INT_LATCH_SET		0x10
#define USB_ID_INT_LATCH_CLR		0x11


#define USB_ID_INT_EN_LO_SET		0x12
#define USB_ID_INT_EN_LO_CLR		0x13
#define USB_ID_INT_EN_HI_SET		0x14
#define USB_ID_INT_EN_HI_CLR		0x15
#define USB_OTG_ADP_CTRL		0x16
#define USB_OTG_ADP_HIGH		0x17
#define USB_OTG_ADP_LOW			0x18
#define USB_OTG_ADP_RISE		0x19
#define USB_OTG_REVISION		0x1A

/* to be moved to LDO*/
#define MISC2				0xE5
#define CFG_LDO_PD2			0xF5
#define USB_BACKUP_REG			0xFA

#define STS_HW_CONDITIONS		0x21
#define STS_USB_ID			(1 << 2)


/* In module TWL6030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E
#define STS_HW_CONDITIONS		0x21

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1			0x7D
#define VUSB_DEDICATED2			0x7E
#define VUSB1V5_DEV_GRP			0x71
#define VUSB1V5_TYPE			0x72
#define VUSB1V5_REMAP			0x73
#define VUSB1V8_DEV_GRP			0x74
#define VUSB1V8_TYPE			0x75
#define VUSB1V8_REMAP			0x76
#define VUSB3V1_DEV_GRP			0x77
#define VUSB3V1_TYPE			0x78
#define VUSB3V1_REMAP			0x79

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module TWL6030_MODULE_MAIN_CHARGE*/

#define CHARGERUSB_CTRL1		0x8

#define CONTROLLER_STAT1		0x03
#define	VBUS_DET			(1 << 2)


/* OMAP control module register for UTMI PHY*/
#define CONTROL_DEV_CONF		0x300
#define PHY_PD				0x1

#define OCP2SCP_TIMING_OFFSET 0xAB018
#define USB2PHYCM_TRIM_OFFSET 0xAB0B8

struct twl6030_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	/* TWL6030 internal USB regulator supplies */
	struct regulator	*usb1v5;
	struct regulator	*usb1v8;
	struct regulator	*usb3v1;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool			irq_enabled;
	int			prev_vbus;

	struct	twl4030_usb_data * pdata;
};

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct twl6030_usb, otg);

static void __iomem *ctrl_base;

/*-------------------------------------------------------------------------*/

static inline int twl6030_writeb(struct twl6030_usb *twl, u8 module,
						u8 data, u8 address)
{
	int ret = 0;

	ret = twl_i2c_write_u8(module, data, address);
	if (ret < 0)
		dev_dbg(twl->dev,
			"TWL6030:USB:Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline int twl6030_readb(struct twl6030_usb *twl, u8 module, u8 address)
{
	u8 data = 0;
	int ret = 0;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_dbg(twl->dev,
			"TWL6030:readb[0x%x,0x%x] Error %d\n",
					module, address, ret);

	return ret;
}

/*-------------------------------------------------------------------------*/

static int twl6030_usb_ldo_init(struct twl6030_usb *twl)
{

	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP*/
	twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, USB_BACKUP_REG);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x1, CFG_LDO_PD2);

	/* Program MISC2 register and set bit VUSB_IN_VBAT */
	twl6030_writeb(twl, TWL6030_MODULE_ID0 , 0x10, MISC2);
	/*
	 * Program the VUSB_CFG_VOLTAGE register to set the VUSB
	 * voltage to 3.3V
	 */
	twl6030_writeb(twl, TWL_MODULE_PM_RECEIVER, 0x18,
						VUSB_CFG_VOLTAGE);

	/* Program the VUSB_CFG_TRANS for ACTIVE state. */
	twl6030_writeb(twl, TWL_MODULE_PM_RECEIVER, 0x3F,
						VUSB_CFG_TRANS);

	/* Program the VUSB_CFG_STATE register to ON on all groups. */
	twl6030_writeb(twl, TWL_MODULE_PM_RECEIVER, 0xE1,
						VUSB_CFG_STATE);

	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP bit */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x4, USB_VBUS_CTRL_SET);

	/*
	 * Program the USB_ID_CTRL_SET register to enable GND drive
	 * and the ID comparators
	 */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x4, USB_ID_CTRL_SET);

	return 0;

}

static ssize_t twl6030_usb_vbus_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl6030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);
	ret = sprintf(buf, "%s\n",
			(twl->linkstat == USB_EVENT_VBUS) ? "on" : "off");
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}

static ssize_t twl6030_usb_vbus_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{

	int status  = USB_EVENT_NONE;
	struct twl6030_usb *twl = dev_get_drvdata(dev);
	
   	if (strstr(buf, "ON") || strstr(buf, "on"))
   		{
   		   status = USB_EVENT_VBUS;
   		}
	else if (strstr(buf, "OFF") || strstr(buf, "off"))
		{
		   status = USB_EVENT_NONE;
		}
	else
		return -EINVAL;

	//twl->linkstat = status;
	blocking_notifier_call_chain(&twl->otg.notifier,status, twl->otg.gadget);
    //sysfs_notify(&twl->dev->kobj, NULL, "vbus");
	
    return count;
}
static DEVICE_ATTR(vbus, 0660, twl6030_usb_vbus_show, twl6030_usb_vbus_store);


static irqreturn_t twl6030_usb_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	int status  = USB_EVENT_NONE;
	int vbus_state, hw_state;
	struct usb_composite_dev *cdev;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);
	vbus_state = twl6030_readb(twl, TWL6030_MODULE_CHARGER, CONTROLLER_STAT1);

	pr_alert("usb_irq : cond 0x%x, vbus 0x%x, otg %d, linkstat %d, muic %d type\n", 
			hw_state, vbus_state, twl->otg.state, twl->linkstat, get_usbic_state());

	/* AC unplugg can also generate this IRQ
	 * we only call the notifier in case of VBUS change
	 */
	if (twl->pdata && (twl->pdata->host_mode & TWL_HOST_GPIO_ID)) {

		if (twl->linkstat == USB_EVENT_ID) {
			pr_alert("usb_irq : host mode\n");
			twl->prev_vbus = 0;
			return IRQ_HANDLED;
		}
	}

	if (twl->prev_vbus != (vbus_state & VBUS_DET)) {
		if (!(hw_state & STS_USB_ID)) {
			if (vbus_state & VBUS_DET) {
				status = USB_EVENT_VBUS;
				twl->otg.default_a = false;
				twl->otg.state = OTG_STATE_B_IDLE;

				pr_alert("usb_irq : vbus\n");
				usb_gadget_connect(twl->otg.gadget);
			}
			else{ // CABLE OUT
				status = USB_EVENT_NONE;
				cdev = get_gadget_data(twl->otg.gadget);
				
				pr_alert("usb_irq : disconnect\n");
				cdev->mute_switch = 0;
				usb_gadget_disconnect(twl->otg.gadget);
			}

			twl->linkstat = status;
			blocking_notifier_call_chain(&twl->otg.notifier,
					status, twl->otg.gadget);
		}
		sysfs_notify(&twl->dev->kobj, NULL, "vbus");
	}
	twl->prev_vbus = vbus_state & VBUS_DET;

	return IRQ_HANDLED;
}

static irqreturn_t twl6030_usbotg_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	int status = USB_EVENT_NONE;
	u8 hw_state;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	pr_alert("usbotg_irq : hw_condition 0x%x\n", hw_state);

	if (hw_state & STS_USB_ID) {

		usb_gadget_connect(twl->otg.gadget);

		twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_CLR, 0x1);
		twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_SET,
									0x10);
		status = USB_EVENT_ID;
		twl->otg.default_a = true;
		twl->otg.state = OTG_STATE_A_IDLE;
		blocking_notifier_call_chain(&twl->otg.notifier, status,
							twl->otg.gadget);
		host_state_notify(&twl->otg.ndev, NOTIFY_HOST_ADD);
	} else {
		twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_CLR,
									0x10);
		twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_SET,
									0x1);
	}
	twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_LATCH_CLR, status);
	twl->linkstat = status;

	pr_alert("usbotg_irq : linkstat %d\n", status);

	return IRQ_HANDLED;
}

static irqreturn_t twl6030_usbid_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	int status = USB_EVENT_NONE;
	int val;

	val = gpio_get_value(twl->pdata->gpio_id);
	if (val < 0) {
		pr_err("usb_id_irq: gpio_get_value error %d\n", val);
		return IRQ_HANDLED;
	}

	status = get_usbic_state();

	pr_alert("usb_id_irq : gpio %d, otg %d, linkstat %d, muic %d type\n",
			val, twl->otg.state, twl->linkstat, status);

	if (!val) { // usb connector connected

		if (status == MICROUSBIC_MHL_CHARGER) {
			pr_alert("usb_id_irq : hdmi connected. type=%d\n", status);
			return IRQ_HANDLED;
		}

		usb_gadget_connect(twl->otg.gadget);

		status = USB_EVENT_ID;
		twl->otg.default_a = true;
		twl->otg.state = OTG_STATE_A_IDLE;
		blocking_notifier_call_chain(&twl->otg.notifier, status, twl->otg.gadget);

		host_state_notify(&twl->otg.ndev, NOTIFY_HOST_ADD);

	} else { // usb connector disconnected

		if (twl->linkstat == USB_EVENT_ID) {

			pr_alert("usb_id_irq : disconnected\n");

			status = USB_EVENT_NONE;
			twl->otg.default_a = false;
			blocking_notifier_call_chain(&twl->otg.notifier, status, twl->otg.gadget);

			host_state_notify(&twl->otg.ndev, NOTIFY_HOST_REMOVE);
			if (twl->pdata->muic_id_open)
				twl->pdata->muic_id_open();
		}
	}
	twl->linkstat = status;
	pr_alert("usb_id_irq : linkstat %d\n", status);

	return IRQ_HANDLED;
}

static int twl6030_set_suspend(struct otg_transceiver *x, int suspend)
{
	return 0;
}

static int twl6030_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct twl6030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.gadget = gadget;
	if (!gadget)
		twl->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int twl6030_enable_irq(struct otg_transceiver *x)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);

	twl6030_writeb(twl, TWL_MODULE_USB, USB_ID_INT_EN_HI_SET, 0x1);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_STS_C);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);
	twl6030_usb_irq(twl->irq, twl);
	twl6030_usbotg_irq(twl->irq, twl);

	return 0;
}

static inline int twl_otg_power_enable(struct otg_transceiver *x, bool enabled)
{
	struct twl6030_usb * twl = xceiv_to_twl(x);

	twl->pdata->booster_enable(enabled);

	if (enabled && x->ndev.mon)
		host_monitor_start(&x->ndev);

	return 0;
}

static int twl6030_set_vbus(struct otg_transceiver *x, bool enabled)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);

	/*
	 * Start driving VBUS. Set OPA_MODE bit in CHARGERUSB_CTRL1
	 * register. This enables boost mode.
	 */
	dev_alert(twl->dev, "twl6030_set_vbus : %d\n", enabled);

	if (enabled) {

		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE , 0x40,
						CHARGERUSB_CTRL1);
	} else {
		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE , 0x00,
						CHARGERUSB_CTRL1);
	}

	return 0;
}


static int twl6030_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct twl6030_usb *twl;

	if (!x)
		return -ENODEV;

	twl = xceiv_to_twl(x);
	twl->otg.host = host;
	if (!host)
		twl->otg.state = OTG_STATE_UNDEFINED;
	return 0;
}

static int set_phy_clk(struct otg_transceiver *x, int on)
{
	static int state;
	struct clk *phyclk;
	struct clk *clk48m;
	struct clk *clk32k;

	phyclk = clk_get(NULL, "ocp2scp_usb_phy_ick");
	if (IS_ERR(phyclk)) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_ick\n");
		return PTR_ERR(phyclk);
	}

	clk48m = clk_get(NULL, "ocp2scp_usb_phy_phy_48m");
	if (IS_ERR(clk48m)) {
		pr_warning("cannot clk_get ocp2scp_usb_phy_phy_48m\n");
		clk_put(phyclk);
		return PTR_ERR(clk48m);
	}

	clk32k = clk_get(NULL, "usb_phy_cm_clk32k");
	if (IS_ERR(clk32k)) {
		pr_warning("cannot clk_get usb_phy_cm_clk32k\n");
		clk_put(phyclk);
		clk_put(clk48m);
		return PTR_ERR(clk32k);
	}

	if (on) {
		if (!state) {
			/* Enable the phy clocks*/
			clk_enable(phyclk);
			clk_enable(clk48m);
			clk_enable(clk32k);
			state = 1;
		}
	} else if (state) {
		/* Disable the phy clocks*/
		clk_disable(phyclk);
		clk_disable(clk48m);
		clk_disable(clk32k);
		state = 0;
	}
	return 0;
}

//#define EYE_DIAGRAM_TEST
#ifdef EYE_DIAGRAM_TEST
static void eye_diagram_test(void)
{
	static int phy_call_counter = 0;
	static int swcap_trim_offset[] = {0, 0xC, 0x18, 0x24, 0x7F};

	//these are exemple values, final optimzed value will be optained by multiple iterations
	//for test purpose each time usb cable is plugged, next swcap_offset value is used

	static int swcap_trim_ref = 0;

	u32 read_val = 0;
	u32 swcap_trim = 0;

	// First apply workaround to prevent the 4-bit shift issue
	// Bit-field SYNC2 of OCP2SCP_TIMING should be set to 0xF

	read_val = __raw_readl(ctrl_base+OCP2SCP_TIMING_OFFSET);
	read_val |= 0x0000000F;
	__raw_writel(read_val, ctrl_base+0xAB018);
	pr_crit("OCP2SCP_TIMING register = %2x\n", __raw_readl(ctrl_base+0xAB018));

	read_val = __raw_readl(ctrl_base+USB2PHYCM_TRIM_OFFSET);

	if (phy_call_counter == 0) {
		pr_crit("USBPHY_USB2PHYCM_TRIM register = %2x\n", read_val);
		swcap_trim = (read_val & 0x00007F00) >> 8;
		pr_crit("Read value of SWCAP_TRIM = %2x\n", swcap_trim);
		swcap_trim_ref = swcap_trim;
	} else
		swcap_trim = swcap_trim_ref;

	swcap_trim += swcap_trim_offset[phy_call_counter];

	if (swcap_trim >= 0x7F)
		swcap_trim = 0x7F;

	swcap_trim = swcap_trim << 8;

	read_val &= ~0x00007F00;
	read_val |= swcap_trim;

	read_val |= 0x00008000; // USE_SW_TRIM = 1

	pr_crit("Value programmed in USBPHY_USB2PHYCM_TRIM register = %2x\n", read_val);

	__raw_writel(read_val, ctrl_base+USB2PHYCM_TRIM_OFFSET);
	pr_crit("New value of USBPHY_USB2PHYCM_TRIM register = %2x\n", __raw_readl(ctrl_base+USB2PHYCM_TRIM_OFFSET));

	pr_crit("count %d, swcap_offset 0x%x\n", phy_call_counter, swcap_trim_offset[phy_call_counter]);

	phy_call_counter++;
	if (phy_call_counter == 5)
		phy_call_counter = 0;
}
#endif

static void phy_init_for_eyediagram(u32 swcap_trim_offset)
{
	u32 read_val = 0;
	u32 swcap_trim = 0;

	if (__raw_readl(ctrl_base + OCP2SCP_TIMING_OFFSET) != 0x0000000F) {
		__raw_writel(0x00000000F, ctrl_base + OCP2SCP_TIMING_OFFSET);
	}

	read_val = __raw_readl(ctrl_base + USB2PHYCM_TRIM_OFFSET);
	swcap_trim = (read_val & 0x00007F00) >> 8;

	if (swcap_trim != (0x4E + swcap_trim_offset)) { // 4E(default) + 0x24(offset) = 0xF2
		swcap_trim = 0x4E + swcap_trim_offset;

		read_val &= ~0x00007F00;
		read_val |= swcap_trim << 8;
		read_val |= 0x00008000; // USE_SW_TRIM = 1

		__raw_writel(read_val, ctrl_base + USB2PHYCM_TRIM_OFFSET);
	}
	pr_alert("usb swcap_trim_offset = 0x%x, USB2PHYCM_TRIM = 0x%x\n", swcap_trim_offset,
			__raw_readl(ctrl_base + USB2PHYCM_TRIM_OFFSET));
}

static int phy_init(struct otg_transceiver *x)
{
	struct twl6030_usb *twl = xceiv_to_twl(x);
	set_phy_clk(x, 1);

	if (__raw_readl(ctrl_base + CONTROL_DEV_CONF) & PHY_PD) {
		__raw_writel(~PHY_PD, ctrl_base + CONTROL_DEV_CONF);
		msleep(200);
	}

#ifdef EYE_DIAGRAM_TEST
	eye_diagram_test();
#else
	if (twl->pdata && twl->pdata->swcap_trim_offset) {
			phy_init_for_eyediagram(twl->pdata->swcap_trim_offset);
	}
#endif

	return 0;
}

static void phy_shutdown(struct otg_transceiver *x)
{
	set_phy_clk(x, 0);
	__raw_writel(PHY_PD, ctrl_base + CONTROL_DEV_CONF);
}


#if defined(CONFIG_USB_HOST_NOTIFY)
static int twl6030_check_id(void * data)
{
	struct twl6030_usb * twl = data;
	int ret = 0;
	u8 hw_state;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	dev_dbg(twl->dev, "check_id : hw_condition 0x%x\n", hw_state);

	if (hw_state & STS_USB_ID) {
		dev_dbg(twl->dev, "check_id : ID 1\n");
	}
	else {
		ret = 1;
		dev_dbg(twl->dev, "check_id : ID 0\n");
	}
	return ret;
}

static int twl6030_host_stop(void * data)
{
	struct twl6030_usb * twl = data;
	int status = USB_EVENT_NONE;

	pr_alert("twl6030_host_stop\n");
	twl->otg.default_a = false;
	blocking_notifier_call_chain(&twl->otg.notifier, status, twl->otg.gadget);
	host_state_notify(&twl->otg.ndev, NOTIFY_HOST_REMOVE);
	//twl->pdata->muic_id_open();
	twl->linkstat = status;
	return 0;
}

static int twl_register_host_notify_dev(struct twl6030_usb * twl)
{
	struct host_monitor_data * mon;

	if (twl->pdata->host_mode & TWL_HOST_MON) {

		mon = kzalloc(sizeof *mon, GFP_KERNEL);
		if (!mon)
			return -ENOMEM;

		mon->check_data = twl;
		mon->result_data = twl;
		mon->check_cb = twl6030_check_id;
		mon->result_cb = twl6030_host_stop;

		twl->otg.ndev.mon = mon;
	}

	twl->otg.ndev.name = "usb_otg";

	if (twl->pdata->booster_enable)
		twl->otg.ndev.set_booster = twl->pdata->booster_enable;

	host_notify_dev_register(&twl->otg.ndev);
	return 0;
}

static void twl_remove_host_notify_dev(struct twl6030_usb * twl)
{
	if (twl->otg.ndev.mon)
		kfree(twl->otg.ndev.mon);

	host_notify_dev_unregister(&twl->otg.ndev);
}
#endif /* CONFIG_USB_HOST_NOTIFY */



static int twl_init_gpio_id(struct twl6030_usb * twl)
{
	int status = 0;
	int irq = 0;

	dev_alert(twl->dev, "init gpio_id : %d\n", twl->pdata->gpio_id);

	status = gpio_request(twl->pdata->gpio_id, "usb_otg_id");
	if (status < 0) {
		dev_alert(twl->dev, "gpio %d request failed.\n", twl->pdata->gpio_id);
		return status;
	}

	status = gpio_direction_input(twl->pdata->gpio_id);
	if (status < 0) {
		dev_alert(twl->dev, "failed to set gpio %d as input\n", twl->pdata->gpio_id);
		return status;
	}

	irq = gpio_to_irq(twl->pdata->gpio_id);
	dev_alert(twl->dev, "request_irq : %d(gpio: %d)\n", irq, twl->pdata->gpio_id);

	status = request_threaded_irq(irq, NULL, twl6030_usbid_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl6030_usbid", twl);
	if (status < 0) {
		dev_alert(twl->dev, "request irq %d failed for gpio %d\n", irq, twl->pdata->gpio_id);
		return status;
	}
	return 0;
}

static int twl_exit_gpio_id(struct twl6030_usb * twl)
{
	if (twl->pdata && twl->pdata->gpio_id)
		gpio_free(twl->pdata->gpio_id);
	return 0;
}

static int __devinit twl6030_usb_probe(struct platform_device *pdev)
{
	struct twl6030_usb	*twl;
	int			status, err;

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	if (pdev->dev.platform_data) {
		twl->pdata = pdev->dev.platform_data;

		pr_alert("twl6030: host_mode(0x%x), ex-power(%s), gpio_id(%s,%d), muic(%s), mon(%s)\n", 
				twl->pdata->host_mode,
				twl->pdata->host_mode & TWL_HOST_EXTERNAL_POWER ? "o":"x",
				twl->pdata->host_mode & TWL_HOST_GPIO_ID ? "o":"x", twl->pdata->gpio_id,
				twl->pdata->host_mode & TWL_HOST_MUIC_ID_OPEN ? "o":"x",
				twl->pdata->host_mode & TWL_HOST_MON ? "o":"x");
	}

	twl->dev		= &pdev->dev;
	twl->irq		= platform_get_irq(pdev, 0);
	twl->otg.dev		= twl->dev;
	twl->otg.label		= "twl6030";
	twl->otg.set_host	= twl6030_set_host;
	twl->otg.set_peripheral	= twl6030_set_peripheral;
	twl->otg.set_suspend	= twl6030_set_suspend;
	twl->asleep		= 1;
	twl->otg.init		= phy_init;
	twl->otg.enable_irq	= twl6030_enable_irq;
	twl->otg.set_clk	= set_phy_clk;
	twl->otg.shutdown	= phy_shutdown;
	twl->prev_vbus		= 0;

	if (twl->pdata && (twl->pdata->host_mode & TWL_HOST_EXTERNAL_POWER))
		twl->otg.set_vbus	= twl_otg_power_enable;
	else
		twl->otg.set_vbus	= twl6030_set_vbus;

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = twl6030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		kfree(twl);
		return err;
	}
	otg_set_transceiver(&twl->otg);

	platform_set_drvdata(pdev, twl);
	if (device_create_file(&pdev->dev, &dev_attr_vbus))
		dev_warn(&pdev->dev, "could not create sysfs file\n");

	BLOCKING_INIT_NOTIFIER_HEAD(&twl->otg.notifier);

	/* Our job is to use irqs and status from the power module
	 * to keep the transceiver disabled when nothing's connected.
	 *
	 * FIXME we actually shouldn't start enabling it until the
	 * USB controller drivers have said they're ready, by calling
	 * set_host() and/or set_peripheral() ... OTG_capable boards
	 * need both handles, otherwise just one suffices.
	 */
	twl->irq_enabled = true;
	status = request_threaded_irq(twl->irq, NULL, twl6030_usbotg_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl6030_usbotg", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return status;
	}
	twl->irq = platform_get_irq(pdev, 1);

	status = request_threaded_irq(twl->irq, NULL, twl6030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"twl6030_usb", twl);
	if (status < 0) {
		dev_dbg(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return status;
	}

	//To just extend ioremap size for adjusting SCP timing
	//ctrl_base = ioremap(0x4A002000, SZ_1K);
	ctrl_base = ioremap(0x4A002000, SZ_1M);

	/* power down the phy by default can be enabled on connect */
	__raw_writel(PHY_PD, ctrl_base + CONTROL_DEV_CONF);


	if (twl->pdata && (twl->pdata->host_mode & TWL_HOST_GPIO_ID)) {
		status = twl_init_gpio_id(twl);
		if (status < 0) {
			dev_dbg(&pdev->dev, "gpio_id init failed.\n");
			kfree(twl);
			return status;
		}
	}

#if defined(CONFIG_USB_HOST_NOTIFY)
	twl_register_host_notify_dev(twl);
#endif

	dev_alert(&pdev->dev, "Initialized TWL6030 USB module\n");
	return 0;
}

static int __exit twl6030_usb_remove(struct platform_device *pdev)
{
	struct twl6030_usb *twl = platform_get_drvdata(pdev);

	twl_exit_gpio_id(twl);

#if defined(CONFIG_USB_HOST_NOTIFY)
	twl_remove_host_notify_dev(twl);
#endif
	free_irq(twl->irq, twl);
	device_remove_file(twl->dev, &dev_attr_vbus);

	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
		REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
			REG_INT_MSK_STS_C);
	kfree(twl);
	iounmap(ctrl_base);

	return 0;
}

static struct platform_driver twl6030_usb_driver = {
	.probe		= twl6030_usb_probe,
	.remove		= __exit_p(twl6030_usb_remove),
	.driver		= {
		.name	= "twl6030_usb",
		.owner	= THIS_MODULE,
	},
};

static int __init twl6030_usb_init(void)
{
	return platform_driver_register(&twl6030_usb_driver);
}
subsys_initcall(twl6030_usb_init);

static void __exit twl6030_usb_exit(void)
{
	platform_driver_unregister(&twl6030_usb_driver);
}
module_exit(twl6030_usb_exit);

MODULE_ALIAS("platform:twl6030_usb");
MODULE_AUTHOR("Texas Instruments, Inc, Nokia Corporation");
MODULE_DESCRIPTION("TWL6030 USB transceiver driver");
MODULE_LICENSE("GPL");
