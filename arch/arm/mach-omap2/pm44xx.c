/*
 * OMAP4 Power Management Routines
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/serial.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/smartreflex.h>
#include <plat/voltage.h>
#include <plat/prcm.h>
#include <plat/io.h>

#ifdef CONFIG_MACH_SAMSUNG_T1
#include <plat/control.h>
#include <mach/board-t1.h>
#endif
#ifdef CONFIG_MACH_SAMSUNG_Q1
#include <plat/control.h>
#include <mach/board-q1.h>
#endif

#include <mach/omap4-common.h>
#include <mach/omap4-wakeupgen.h>
#include <mach/tiler.h>
#include "mach/omap_hsi.h"
#include <linux/i2c/twl.h>

#include "prm.h"
#include "pm.h"
#include "cm.h"
#include "cm-regbits-44xx.h"
#include "prm-regbits-44xx.h"
#include "clock.h"
#include "mux44xx.h"

#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1)
#include "mux.h"
#endif

void *so_ram_address;

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
	u32 saved_logic_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);
static struct powerdomain *mpu_pwrdm;

static struct powerdomain *mpu_pwrdm, *cpu0_pwrdm, *cpu1_pwrdm;
static struct powerdomain *core_pwrdm, *per_pwrdm, *tesla_pwrdm;

static struct voltagedomain *vdd_mpu, *vdd_iva, *vdd_core;

#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1)
#define OMAP_WLAN_PADCONF_PIN		"cam_shutter.gpio_81"
#define OMAP_WLAN_PADCONF_MODE           OMAP_MUX_MODE3
#endif

#define MAX_IOPAD_LATCH_TIME 1000

void omap4_check_clk_idle_state(void)
{
	u32 reg_val = 0;
	u32 reg_count = 0;

	u32 pwrstst_registers[] = {
		0x4A306404, 0x4A306504, 0x4A306F04, 0x4A307004, 0x4A307104,
		0x4A307204, 0xFFFFFFFF
	};

	u32 clkctrl_registers[] = {
		0x4A306040, 0x4A307830,	0x4A004040, 0x4A004420, 0x4A004528,
		0x4A004530, 0x4A004538, 0x4A004540, 0x4A004548, 0x4A004550,
		0x4A004558, 0x4A004560, 0x4A004568, 0x4A004570, 0x4A004578,
		0x4A004580, 0x4A004588, 0x4A008040, 0x4A008828, 0x4A008920,
		0x4A008F20, 0x4A008F28, 0x4A009020, 0x4A009028, 0x4A009120,
		0x4A009220, 0x4A009328, 0x4A009330, 0x4A009338, 0x4A009358,
		0x4A009368, 0x4A0093D0,	0x4A0093E0, 0x4A009428, 0x4A009430,
		0x4A009438, 0x4A009440,	0x4A009448, 0x4A009450, 0x4A009488,
		0x4A0094A0, 0x4A0094A8, 0x4A0094B0, 0x4A0094B8, 0x4A0094E0,
		0x4A0094F0, 0x4A0094F8, 0x4A009500, 0x4A009508, 0x4A009520,
		0x4A009528, 0x4A009538, 0x4A009560, 0xFFFFFFFF
	};

	u32 clkstctrl_registers[] = {
		0x4A004400, 0x4A004500, 0x4A008900, 0x4A008A00, 0x4A008C00,
		0x4A008F00, 0x4A009000, 0x4A009100, 0x4A009200,	0x48243418,
		0xFFFFFFFF
	};

	while(pwrstst_registers[reg_count] != 0xffffffff) {
		/* Check CM_<Power Domain>_PWRSTST registers */
		reg_val = omap_readl(pwrstst_registers[reg_count]);
		if ((reg_val & ~(0x00000003)) != 0x00000000) {
			pr_err("[Active power domain] 0x%08X=0x%08X",
				pwrstst_registers[reg_count], reg_val);
		}
		reg_count += 1;
	}

	reg_count= 0;
	while (clkctrl_registers[reg_count] != 0xffffffff) {
		/* check IDLEST bit in CM_<ClockDomain>_CLKCTRL register */
		reg_val = omap_readl(clkctrl_registers[reg_count]);
		if((reg_val & 0x00030000) != 0x00030000) {
			pr_err("[Active clock domain] 0x%08X=0x%08X",
				clkctrl_registers[reg_count], reg_val);
		}
		reg_count += 1;
	}

	reg_count= 0;
	while (clkstctrl_registers[reg_count] != 0xffffffff) {
		/* Check CM_<Clock Domain>_CLKSTCTRL registers */
		reg_val = omap_readl(clkstctrl_registers[reg_count]);
		if ((reg_val & ~(0x00000003)) != 0x00000000) {
			pr_err("[Active clock status] 0x%08X=0x%08X",
				clkstctrl_registers[reg_count], reg_val);
		}
		reg_count += 1;
	}

	return;
}

#if defined(CONFIG_MACH_T1_CHN)
u32 power_on_alarm_check;
EXPORT_SYMBOL(power_on_alarm_check);
#endif

void omap4_check_gpio_wkup_state(void)
{
	u32 reg_addr = 0;
	u32 reg_val = 0;
	u16 count = 0;
#if defined(CONFIG_MACH_T1_CHN)
	power_on_alarm_check = 0;
#endif

#if 0
	/* Check PRM_IRQSTATUS_MPU_A9, PRM_IRQSTATUS_MPU_A9_2 registers */
	reg_addr = OMAP4430_PRM_BASE + 0x00000010;
	reg_val = omap_readl(reg_addr) & ~(0x61610000);
	if (reg_val) {
		pr_err("[MPU irq status1] 0x%08X=0x%08X\n",
			reg_addr, reg_val);
	}

	reg_addr = OMAP4430_PRM_BASE + 0x00000014;
	reg_val = omap_readl(reg_addr) & ~(0x00000061);
	if (reg_val) {
		pr_err("[MPU irq status2] 0x%08X=0x%08X\n",
			reg_addr, reg_val);
	}
#endif

#if defined(CONFIG_MACH_T1_CHN)
	if(omap_readl(0x4A1001EC) & 0x2000) {
		power_on_alarm_check = 1;
	}
#endif

	/* Check core domain control module register */
	reg_addr = OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE +
			OMAP4_CTRL_MODULE_WAKEUPEVENT_0;

	while (reg_addr < OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE +
				OMAP4_CTRL_MODULE_WAKEUPEVENT_6) {
		reg_addr = OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE +
				OMAP4_CTRL_MODULE_WAKEUPEVENT_0 +
				(count * 0x04);
		reg_val = omap_readl(reg_addr);
		if (reg_val != 0) {
			pr_err("[Core gpio wakeup] 0x%08X=0x%08X\n",
				reg_addr, reg_val);
		}
		count++;
	}

	/* Check wk domain control module register */
	reg_addr = OMAP4_CTRL_MODULE_PAD_WKUP_MUX_PBASE +
			OMAP4_CTRL_WKMODULE_WAKEUPEVENT_0;
	reg_val = omap_readl(reg_addr);
	if (reg_val) {
		pr_err("[WK gpio wakeup] 0x%08X=0x%08X\n",
			reg_addr, reg_val);
	}

	return;
}

/**
 * omap4_device_off_set_state -
 * When Device OFF is enabled, Device is allowed to perform
 * transition to off mode as soon as all power domains in MPU, IVA
 * and CORE voltage are in OFF or OSWR state (open switch retention)
 */
void omap4_device_off_set_state(u8 enable)
{
	if (enable)
		prm_write_mod_reg(0x1 << OMAP4430_DEVICE_OFF_ENABLE_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_DEVICE_OFF_CTRL_OFFSET);
	else
		prm_write_mod_reg(0x0 << OMAP4430_DEVICE_OFF_ENABLE_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_DEVICE_OFF_CTRL_OFFSET);
}

/**
 * omap4_device_off_read_prev_state:
 * returns 1 if the device hit OFF mode
 * This is API to check whether OMAP is waking up from device OFF mode.
 * There is no other status bit available for SW to read whether last state
 * entered was device OFF. To work around this, CORE PD, RFF context state
 * is used which is lost only when we hit device OFF state
 */
u32 omap4_device_off_read_prev_state(void)
{
	u32 reg = 0;

	reg = prm_read_mod_reg(core_pwrdm->prcm_offs,
					core_pwrdm->context_offset);
	reg = (reg >> 0x1) & 0x1;
	return reg;
}

/**
 * omap4_device_off_read_prev_state:
 * returns 1 if the device next state is OFF
 * This is API to check whether OMAP is programmed for device OFF
 */
u32 omap4_device_off_read_next_state(void)
{
	return prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
			OMAP4_PRM_DEVICE_OFF_CTRL_OFFSET)
			& OMAP4430_DEVICE_OFF_ENABLE_MASK;
}

int omap4_can_sleep(void)
{
	if (!omap_uart_can_sleep())
		return 0;
	return 1;
}

void omap4_trigger_ioctrl(void)
{
	int i = 0;

	/* Trigger WUCLKIN enable */
	prm_rmw_mod_reg_bits(OMAP4430_WUCLK_CTRL_MASK, OMAP4430_WUCLK_CTRL_MASK,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_IO_PMCTRL_OFFSET);

	omap_test_timeout(
		(prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
				OMAP4_PRM_IO_PMCTRL_OFFSET)
			& OMAP4430_WUCLK_STATUS_MASK),
		MAX_IOPAD_LATCH_TIME, i);
	/* Trigger WUCLKIN disable */
	prm_rmw_mod_reg_bits(OMAP4430_WUCLK_CTRL_MASK, 0x0,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_IO_PMCTRL_OFFSET);

	i=0;
	omap_test_timeout(
		!(prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
				OMAP4_PRM_IO_PMCTRL_OFFSET)
			& OMAP4430_WUCLK_STATUS_MASK),
		MAX_IOPAD_LATCH_TIME, i);
	return;
}

/* This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled. */
int omap4_set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		if ((pwrdm_read_pwrst(pwrdm) > state) &&
			(pwrdm->flags & PWRDM_HAS_LOWPOWERSTATECHANGE)) {
				ret = pwrdm_set_next_pwrst(pwrdm, state);
				pwrdm_set_lowpwrstchange(pwrdm);
				pwrdm_wait_transition(pwrdm);
				pwrdm_state_switch(pwrdm);
				return ret;
		}
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		if (pwrdm->pwrdm_clkdms[0]->flags & CLKDM_CAN_ENABLE_AUTO)
			omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		else
			omap2_clkdm_sleep(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}
/* This is a common low power function called from suspend and
 * cpuidle
 */

void omap4_enter_sleep(unsigned int cpu, unsigned int power_state)
{
	int cpu0_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int mpu_next_state = PWRDM_POWER_ON;
	int volatile val;


	pwrdm_clear_all_prev_pwrst(cpu0_pwrdm);
	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

	cpu0_next_state = pwrdm_read_next_pwrst(cpu0_pwrdm);
	per_next_state = pwrdm_read_next_pwrst(per_pwrdm);
	core_next_state = pwrdm_read_next_pwrst(core_pwrdm);
	mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);

	if (mpu_next_state < PWRDM_POWER_INACTIVE) {
		/* Disable SR for MPU VDD */
		omap_smartreflex_disable(vdd_mpu);
		/* Enable AUTO RET for mpu */
		if (!omap4_device_off_read_next_state())
			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_MPU_L_MASK,
			0x2 << OMAP4430_AUTO_CTRL_VDD_MPU_L_SHIFT,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
	}

	if (core_next_state < PWRDM_POWER_ON) {
		/*
		 * NOTE: IVA can hit RET outside of cpuidle and
		 * hence this is not the right (optimal) place
		 * to enable IVA AUTO RET. But since enabling AUTO
		 * RET requires SR to be disabled, its done here
		 * for now. Needs a relook to see if this can be
		 * optimized.
		 */


		omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		omap_uart_prepare_idle(2);
		omap_uart_prepare_idle(3);

		if (omap4_device_off_read_next_state()) {
			omap2_gpio_prepare_for_idle(1);

			/* Extend Non-EMIF I/O isolation */
			prm_rmw_mod_reg_bits(OMAP4430_ISOOVR_EXTEND_MASK,
				OMAP4430_ISOOVR_EXTEND_MASK,
				OMAP4430_PRM_DEVICE_MOD,
				OMAP4_PRM_IO_PMCTRL_OFFSET);
		}
		else
			omap2_gpio_prepare_for_idle(0);

		omap4_trigger_ioctrl();
	}
	if (core_next_state < PWRDM_POWER_INACTIVE) {
		/* Disable SR for CORE and IVA VDD*/
		omap_smartreflex_disable(vdd_iva);
		omap_smartreflex_disable(vdd_core);


		if (!omap4_device_off_read_next_state()) {
			/* Enable AUTO RET for IVA and CORE */
#if 0
			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_IVA_L_MASK,
			0x2 << OMAP4430_AUTO_CTRL_VDD_IVA_L_SHIFT,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
#endif

			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_CORE_L_MASK,
			0x2 << OMAP4430_AUTO_CTRL_VDD_CORE_L_SHIFT,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
		}
	}

	/* FIXME  This call is not needed now for retention support and global
	 * suspend resume support. All the required actions are taken based
	 * the connect disconnect events.
	 * This call will be required for offmode support to save and restore
	 * context in the idle path oddmode support only.
	*/
#if 0
	if (core_next_state < PWRDM_POWER_ON)
		musb_context_save_restore(disable_clk);
#endif

	if (omap4_device_off_read_next_state()) {
		omap4_prcm_prepare_off();
		/* Save the device context to SAR RAM */
		omap4_sar_save();
		omap4_sar_overwrite();
	}

	/* warm reset rst time1 */
	val = omap_readl(0x4a307b08);	
	val = val & 0xFFFFFc00;
	val = val | 0x0000015e;
	omap_writel(val , 0x4a307b08);

	/* warm voltset warm reset */
	omap_writel(0xFFFFFFFF , 0x4a307b24);

	omap4_enter_lowpower(cpu, power_state);

	if (omap4_device_off_read_prev_state()) {

		omap4_prcm_resume_off();
#ifdef CONFIG_PM_DEBUG
		omap4_device_off_counter++;
#endif
	}


	/* FIXME  This call is not needed now for retention support and global
	 * suspend resume support. All the required actions are taken based
	 * the connect disconnect events.
	 * This call will be required for offmode support to save and restore
	 * context in the idle path oddmode support only.
	*/
#if 0
	if (core_next_state < PWRDM_POWER_ON)
		musb_context_save_restore(enable_clk);

#endif

	if (core_next_state < PWRDM_POWER_ON) {

		if (omap4_device_off_read_prev_state())
			omap2_gpio_resume_after_idle(1);
		else
			omap2_gpio_resume_after_idle(0);

		if (omap4_device_off_read_next_state())
			/* Disable the extention of Non-EMIF I/O isolation */
			prm_rmw_mod_reg_bits(OMAP4430_ISOOVR_EXTEND_MASK, 0x0,
				OMAP4430_PRM_DEVICE_MOD,
				OMAP4_PRM_IO_PMCTRL_OFFSET);

		omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
		omap_uart_resume_idle(2);
		omap_uart_resume_idle(3);
	}

	if (core_next_state < PWRDM_POWER_INACTIVE) {

		if (!omap4_device_off_read_next_state()) {
				/* Disable AUTO RET for IVA and CORE */
#if 0
			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_IVA_L_MASK,
			0x0,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
#endif

			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_CORE_L_MASK,
			0x0,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
		}


		/* Enable SR for IVA and CORE */
		omap_smartreflex_enable(vdd_iva);
		omap_smartreflex_enable(vdd_core);
	}

	if (mpu_next_state < PWRDM_POWER_INACTIVE) {
		if (!omap4_device_off_read_next_state())
			/* Disable AUTO RET for mpu */
			prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_MPU_L_MASK,
			0x0,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
		/* Enable SR for MPU VDD */
		omap_smartreflex_enable(vdd_mpu);
	}

	return;
}

#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1) /* T1 hostwakeup */
wl_isr_t wl_interrupt_fn = NULL;
void register_isr_wlan(wl_isr_t isr)
{
	wl_interrupt_fn = isr;
}
void wlan_irq_callback(void)
{
	if(wl_interrupt_fn){
		(*wl_interrupt_fn)(0,NULL);
	    }else{
		 /* before register wlan callback by insmod/rmmoe, must ignore this interrupt */
		 /* this interrupt processed real isr that register by request_irq */
		 ;
    }
}
EXPORT_SYMBOL(register_isr_wlan);
DECLARE_TASKLET(wifi_tasklet,wlan_irq_callback, 0);


int omap_wlan_is_io_wakeup_from_wlan(void)
{
        u16 val;

        val = omap_mux_read_signal(OMAP_WLAN_PADCONF_PIN);
        if (val == -ENODEV)
                return 0;
        if ((val & OMAP_MUX_MODE_MASK) != OMAP_WLAN_PADCONF_MODE)
                return 0;
        if (val & OMAP44XX_PADCONF_WAKEUPEVENT0)
                return 1;
        return 0;
}

#endif

static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqenable_mpu, irqstatus_mpu;

	irqenable_mpu = prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
					 OMAP4_PRM_IRQENABLE_MPU_OFFSET);
	irqstatus_mpu = prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
					 OMAP4_PRM_IRQSTATUS_MPU_OFFSET);

	/* Check if a IO_ST interrupt */
	if (irqstatus_mpu & OMAP4430_IO_ST_MASK) {
		/* Re-enable UART3 */
		omap_writel(0x2, 0x4A009550);
		omap_writel(0xD, 0x48020054);
		/* Modem HSI wakeup */
		if (omap_hsi_is_io_wakeup_from_hsi())
			omap_hsi_wakeup();
#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1)
	if (omap_wlan_is_io_wakeup_from_wlan())
	{
		tasklet_schedule(&wifi_tasklet);
	}
#endif
		/* usbhs remote wakeup */
		usbhs_wakeup();
		omap4_trigger_ioctrl();
	}

	/* Clear the interrupt */
	irqstatus_mpu &= irqenable_mpu;
	prm_write_mod_reg(irqstatus_mpu, OMAP4430_PRM_OCP_SOCKET_MOD,
					OMAP4_PRM_IRQSTATUS_MPU_OFFSET);

	return IRQ_HANDLED;
}


#ifdef CONFIG_SUSPEND
static int omap4_pm_prepare(void)
{
	u32 volatile reg_val = 0;
	u32 volatile status = 0;

	u32 max_time_us = 100;
	/* WA : System can't enter off mode because of tesla. */

	/* Read PM_DSP_PWRSTST register */
	reg_val = omap_readl(0x4A306404) & 0x00100003;

	if (reg_val != 0x00000000) {
		pr_err("Turn off tesla power domain!!\n");
		/* Force a wake up on CM_DSP_CLKSTCTRL,
		 * before modifying PM_DSP_PWRSTCTRL register */
		omap_writel(0x00000002, 0x4A004400);

		status = omap_readl(0x4A306404);
		while(((status & 0x3) == 0x0) && max_time_us--) {
			status = omap_readl(0x4A306404);
			udelay(1);
		}

		/* Switch PM_DSP_PWRSTCTRL - ON & OFF */
		//omap_writel(0x003F0703, 0x4A306400);
		//omap_writel(0x003F0700, 0x4A306400);

		/* Put CM_DSP_CLKSTCTRL in HW_AUTO */
		omap_writel(0x00000003, 0x4A004400);

		// wait until DSP power status shows OFF
		max_time_us = 100;
		status = omap_readl(0x4A306404);
		while(((status & 0x3) == 0x3) && max_time_us--) {
			status = omap_readl(0x4A306404);
			udelay(1);
		}

	}

		/* Switch PM_DSP_PWRSTCTRL - ON & OFF */

	/* Turn off TWL6030 LDO before entering suspend */
	twl6030_suspend_ldo_off();
	return 0;
}
#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1)
static int wlan_wakeup_enable()
{
	omap_mux_enable_wakeup(OMAP_WLAN_PADCONF_PIN);
	return 0;
}
#endif
extern void gic_dist_pending_show_all(void) ;
#if 1
extern void restore_mux_offmode_setting(void);
extern void force_mux_offmode_setting(void);
#endif
static int omap4_pm_suspend(void)
{
	struct power_state *pwrst;
	int state;
	u32 cpu_id = 0;
	u32 cpu1_state;
	/*
	 * Wakeup timer from suspend
	 */
	if (wakeup_timer_seconds || wakeup_timer_milliseconds)
		omap2_pm_wakeup_on_timer(wakeup_timer_seconds,
					 wakeup_timer_milliseconds);
#ifdef CONFIG_PM_DEBUG
	pwrdm_pre_transition();
#endif

	/*
	 * Clear all wakeup sources and keep
	 * only Debug UART, Keypad, HSI(CAWAKE+DMA) and GPT1 interrupt
	 * as a wakeup event from MPU/Device OFF
	 */
	omap4_wakeupgen_clear_all(cpu_id);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_UART4);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_GPT1);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_PRCM);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_SYS_1N);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_HSI_P1);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_HSI_DMA);

	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_L3_DBG);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_L3_APP);

#ifdef CONFIG_ENABLE_L3_ERRORS
	/* Allow the L3 errors to be logged */
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_L3_DBG);
	omap4_wakeupgen_set_interrupt(cpu_id, OMAP44XX_IRQ_L3_APP);
#endif
#if defined(CONFIG_MACH_SAMSUNG_T1 )|| defined(CONFIG_MACH_SAMSUNG_Q1)
	wlan_wakeup_enable();
#endif

	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
		pwrst->saved_logic_state = pwrdm_read_logic_retst(pwrst->pwrdm);
	}

	/* program all powerdomains to sleep */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		pwrdm_clear_all_prev_pwrst(pwrst->pwrdm);
		/*
		 * While attempting a system wide suspend, all non core cpu;s
		 * are already offlined using the cpu hotplug callback.
		 * Hence we do not have to program non core cpu (cpu1) target
		 * state in the omap4_pm_suspend function
		 */
		if (strcmp(pwrst->pwrdm->name, "cpu1_pwrdm")) {

#ifdef CONFIG_OMAP_ALLOW_OSWR
			if ((pwrst->pwrdm->pwrsts_logic_ret == PWRSTS_OFF_RET)
			 && (omap_rev() >= OMAP4430_REV_ES2_1))
				pwrdm_set_logic_retst(pwrst->pwrdm,
							PWRDM_POWER_OFF);
#endif
			if (omap4_set_pwrdm_state(pwrst->pwrdm,
							pwrst->next_state))
				goto restore;
		}
	}

	/*
	 * To Ensure that we don't attempt suspend when CPU1 is
	 * not in OFF state to avoid un-supported H/W mode
	 */
	cpu1_state = pwrdm_read_pwrst(cpu1_pwrdm);
	if (cpu1_state != PWRDM_POWER_OFF)
		goto restore;

	/* Check clock domain idle status */
	omap4_check_clk_idle_state();

	omap_uart_prepare_suspend();
	omap_hsi_prepare_suspend();

	/* Enable Device OFF */
	if (enable_off_mode){
		omap4_device_off_set_state(1);
		tiler_restore_pat_entry();
	}

	gic_dist_pending_show_all();

	omap4_enter_sleep(0, PWRDM_POWER_OFF);

	/* Check wakeup event */
	omap4_check_gpio_wkup_state();

	/* Disable Device OFF state*/
	if (enable_off_mode)
		omap4_device_off_set_state(0);

restore:
	pr_err("Off mode count = %d\n", omap4_device_off_counter);
	/* Print the previous power domain states */
	pr_info("Read Powerdomain states as ...\n");
	pr_info("0 : OFF, 1 : RETENTION, 2 : ON-INACTIVE, 3 : ON-ACTIVE\n");
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state == -EINVAL) {
			state = pwrdm_read_pwrst(pwrst->pwrdm);
			pr_info("Powerdomain (%s) is in state %d\n",
				pwrst->pwrdm->name, state);
		} else {
			pr_info("Powerdomain (%s) entered state %d\n",
				pwrst->pwrdm->name, state);
		}

	}

//	if(check_coredomain)
	gic_dist_pending_show_all();

	/* restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (strcmp(pwrst->pwrdm->name, "cpu1_pwrdm")) {
			omap4_set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
			pwrdm_set_logic_retst(pwrst->pwrdm,
						pwrst->saved_logic_state);
		}
	}

#ifdef CONFIG_PM_DEBUG
	pwrdm_post_transition();
#endif
	/*
	 * Enable all wakeup sources post wakeup
	 */
	omap4_wakeupgen_set_all(cpu_id);

	return 0;
}

static int omap4_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap4_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap4_pm_finish(void)
{
	twl6030_resume_ldo_on();
	return;
}

static struct device omappm_dev = {
		.init_name = "omap_pm",
};
static int omap4_pm_begin(suspend_state_t state)
{
	disable_hlt();
	dpll_cascading_blocker_hold(&omappm_dev);
	return 0;
}

static void omap4_pm_end(void)
{
	dpll_cascading_blocker_release(&omappm_dev);
	enable_hlt();
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap4_pm_begin,
	.end		= omap4_pm_end,
	.prepare	= omap4_pm_prepare,
	.enter		= omap4_pm_enter,
	.finish		= omap4_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __attribute__ ((unused)) __init
		clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		omap2_clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
			atomic_read(&clkdm->usecount) == 0)
		omap2_clkdm_sleep(clkdm);
	return 0;
}


static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	if ((!strcmp(pwrdm->name, mpu_pwrdm->name)) ||
			(!strcmp(pwrdm->name, core_pwrdm->name)) ||
			(!strcmp(pwrdm->name, cpu0_pwrdm->name)) ||
			(!strcmp(pwrdm->name, cpu1_pwrdm->name)))
		pwrst->next_state = PWRDM_POWER_ON;
	else if ((!strcmp(pwrdm->name, tesla_pwrdm->name)))
		pwrst->next_state = PWRDM_POWER_OFF;
	else
		pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	return omap4_set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

/**
 * omap4_pm_off_mode_enable :
 *	Program all powerdomain to OFF
 */
void omap4_pm_off_mode_enable(int enable)
{
	struct power_state *pwrst;
	u32 state;
	u32 logic_state;

	if (enable) {
		state = PWRDM_POWER_OFF;
		logic_state = PWRDM_POWER_OFF;
	} else {
		state = PWRDM_POWER_RET;
		logic_state = PWRDM_POWER_RET;
	}

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (!plist_head_empty(&pwrst->pwrdm->wakeuplat_dev_list)) {
			pr_crit("Device is holding contsraint on %s\n",
				 pwrst->pwrdm->name);
			continue;
		}
		pwrdm_set_logic_retst(pwrst->pwrdm, logic_state);
		if ((state == PWRDM_POWER_OFF) &&
				!(pwrst->pwrdm->pwrsts & (1 << state)))
			pwrst->next_state = PWRDM_POWER_RET;
		else
			pwrst->next_state = state;
		omap4_set_pwrdm_state(pwrst->pwrdm, state);
	}
}

static void __init prcm_setup_regs(void)
{
	struct clk *dpll_abe_ck, *dpll_core_ck, *dpll_iva_ck;
	struct clk *dpll_mpu_ck, *dpll_per_ck, *dpll_usb_ck;
	struct clk *dpll_unipro_ck;

	/*Enable all the DPLL autoidle */
	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");
	omap3_dpll_allow_idle(dpll_abe_ck);
	dpll_core_ck = clk_get(NULL, "dpll_core_ck");
	omap3_dpll_allow_idle(dpll_core_ck);
	dpll_iva_ck = clk_get(NULL, "dpll_iva_ck");
	omap3_dpll_allow_idle(dpll_iva_ck);
	dpll_mpu_ck = clk_get(NULL, "dpll_mpu_ck");
	omap3_dpll_allow_idle(dpll_mpu_ck);
	dpll_per_ck = clk_get(NULL, "dpll_per_ck");
	omap3_dpll_allow_idle(dpll_per_ck);
	dpll_usb_ck = clk_get(NULL, "dpll_usb_ck");
	omap3_dpll_allow_idle(dpll_usb_ck);
	dpll_unipro_ck = clk_get(NULL, "dpll_unipro_ck");
	omap3_dpll_allow_idle(dpll_unipro_ck);

	/* Enable autogating for all DPLL post dividers */
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD, OMAP4_CM_DIV_M2_DPLL_MPU_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M4_DPLL_IVA_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M5_DPLL_IVA_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M3_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M4_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M5_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M6_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M7_DPLL_CORE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD, OMAP4_CM_DIV_M2_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M3_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT1_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M4_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M5_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT3_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M6_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_HSDIVIDER_CLKOUT4_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M7_DPLL_PER_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_ABE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_ABE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTHIF_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM1_CKGEN_MOD,	OMAP4_CM_DIV_M3_DPLL_ABE_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_USB_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKDCOLDO_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD, OMAP4_CM_CLKDCOLDO_DPLL_USB_OFFSET);
	cm_rmw_mod_reg_bits(OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK, 0x0,
		OMAP4430_CM2_CKGEN_MOD,	OMAP4_CM_DIV_M2_DPLL_UNIPRO_OFFSET);

	/* Enable IO_ST interrupt */
	prm_rmw_mod_reg_bits(OMAP4430_IO_ST_MASK, OMAP4430_IO_ST_MASK,
		OMAP4430_PRM_OCP_SOCKET_MOD, OMAP4_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable WKUP Dpendency for GPIO1*/
	prm_write_mod_reg(0x1, OMAP4430_PRM_WKUP_MOD, OMAP4_PM_WKUP_GPIO1_WKDEP_OFFSET);

	/* Enable WKUP Dependency for GPIO2*/
	prm_write_mod_reg(0x1, OMAP4430_PRM_L4PER_MOD, OMAP4_PM_L4PER_GPIO2_WKDEP_OFFSET);

	/* Enable GLOBAL_WUEN */
	prm_rmw_mod_reg_bits(OMAP4430_GLOBAL_WUEN_MASK, OMAP4430_GLOBAL_WUEN_MASK,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_IO_PMCTRL_OFFSET);

	/*
	 * Errata ID: i608 Impacted OMAP4430 ES 1.0,2.0,2.1,2.2
	 * On OMAP4, Retention-Till-Access Memory feature is not working
	 * reliably and hardware recommondation is keep it disabled by
	 * default
	 */
	prm_rmw_mod_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_SRAM_WKUP_SETUP_OFFSET);
	prm_rmw_mod_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_CORE_SETUP_OFFSET);
	prm_rmw_mod_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_MPU_SETUP_OFFSET);
	prm_rmw_mod_reg_bits(OMAP4430_DISABLE_RTA_EXPORT_MASK,
		0x1 << OMAP4430_DISABLE_RTA_EXPORT_SHIFT,
		OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_IVA_SETUP_OFFSET);

	/* Toggle CLKREQ in RET and OFF states */
	prm_write_mod_reg(0x3, OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_CLKREQCTRL_OFFSET);

	/*
	 * De-assert PWRREQ signal in Device OFF state
	 *	0x3: PWRREQ is de-asserted if all voltage domain are in
	 *	OFF state. Conversely, PWRREQ is asserted upon any
	 *	voltage domain entering or staying in ON or SLEEP or
	 *	RET state.
	 */
	prm_write_mod_reg(0x3, OMAP4430_PRM_DEVICE_MOD,
				OMAP4_PRM_PWRREQCTRL_OFFSET);


	prm_rmw_mod_reg_bits(OMAP4430_AUTO_CTRL_VDD_IVA_L_MASK,
			0x0, OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_VOLTCTRL_OFFSET);
}

/**
 * prcm_clear_statdep_regs :
 * Clear Static dependencies for the modules
 * Refer TRM section 3.1.1.1.7.1 for more information
 */                                         
static void __init prcm_clear_statdep_regs(void)
{                                           
#ifdef CONFIG_OMAP4_KEEP_STATIC_DEPENDENCIES
	pr_info("%s: Keep static depndencies\n", __func__);
	return;                                 
#else                                       
	u32 reg;                                
	pr_info("%s: Clearing static depndencies\n", __func__);

#if 0
	/*
	 * REVISIT: REVISIT: Seen SGX issues with MPU -> EMIF. Keeping
	 * it enabled.
	 * REVISIT: Seen issue with MPU/DSP -> L3_2 and L4CFG.
	 * Keeping them enabled
	 */
	/* MPU towards EMIF clockdomains */
	reg = OMAP4430_MEMIF_STATDEP_MASK;
	cm_rmw_mod_reg_bits(reg, 0, OMAP4430_CM1_MPU_MOD,
		OMAP4_CM_MPU_STATICDEP_OFFSET);
#endif
	 /*
	  * REVISIT: Issue seen with Ducati towards EMIF, L3_2, L3_1,
	  * L4CFG and L4WKUP static
	  * dependency. Keep it enabled as of now.
	  */
#if 0
	/* Ducati towards EMIF, L3_2, L3_1, L4CFG and L4WKUP clockdomains */
	reg = OMAP4430_MEMIF_STATDEP_MASK | OMAP4430_L3_1_STATDEP_MASK
		| OMAP4430_L3_2_STATDEP_MASK | OMAP4430_L4CFG_STATDEP_MASK
		| OMAP4430_L4WKUP_STATDEP_MASK;
	cm_rmw_mod_reg_bits(reg, 0, OMAP4430_CM2_CORE_MOD,
		OMAP4_CM_DUCATI_STATICDEP_OFFSET);
#endif

	/* SDMA towards EMIF, L3_2, L3_1, L4CFG, L4WKUP, L3INIT
	 * and L4PER clockdomains
	*/
	reg = OMAP4430_MEMIF_STATDEP_MASK | OMAP4430_L3_1_STATDEP_MASK
		| OMAP4430_L3_2_STATDEP_MASK | OMAP4430_L4CFG_STATDEP_MASK
		| OMAP4430_L4WKUP_STATDEP_MASK | OMAP4430_L4PER_STATDEP_MASK
		| OMAP4430_L3INIT_STATDEP_MASK;
	cm_rmw_mod_reg_bits(reg, 0, OMAP4430_CM2_CORE_MOD,
		OMAP4_CM_SDMA_STATICDEP_OFFSET);

	/* C2C towards EMIF clockdomains */
	cm_rmw_mod_reg_bits(OMAP4430_MEMIF_STATDEP_MASK, 0,
		OMAP4430_CM2_CORE_MOD, OMAP4_CM_D2D_STATICDEP_OFFSET);

	/* C2C_STATICDEP_RESTORE towards EMIF clockdomains */
	cm_rmw_mod_reg_bits(OMAP4430_MEMIF_STATDEP_MASK, 0,
			OMAP4430_CM2_RESTORE_MOD,
			OMAP4_CM_D2D_STATICDEP_RESTORE_OFFSET);

	 /* SDMA_RESTORE towards EMIF, L3_1, L4_CFG,L4WKUP clockdomains */
	reg = OMAP4430_MEMIF_STATDEP_MASK | OMAP4430_L3_1_STATDEP_MASK
		| OMAP4430_L4CFG_STATDEP_MASK | OMAP4430_L4WKUP_STATDEP_MASK;
	cm_rmw_mod_reg_bits(reg, 0, OMAP4430_CM2_RESTORE_MOD,
		OMAP4_CM_SDMA_STATICDEP_RESTORE_OFFSET);
#endif
};

/**
 * omap4_pm_init - Init routine for OMAP4 PM
 *
 * Initializes all powerdomain and clockdomain target states
 * and all PRCM settings.
 */
static int __init omap4_pm_init(void)
{
	int ret;
	int ram_addr;

	if (!cpu_is_omap44xx())
		return -ENODEV;

	pr_err("Power Management for TI OMAP4.\n");

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	cpu0_pwrdm = pwrdm_lookup("cpu0_pwrdm");
	cpu1_pwrdm = pwrdm_lookup("cpu1_pwrdm");
	core_pwrdm = pwrdm_lookup("core_pwrdm");
	per_pwrdm = pwrdm_lookup("l4per_pwrdm");
	tesla_pwrdm = pwrdm_lookup("tesla_pwrdm");

#ifdef CONFIG_PM
	prcm_setup_regs();
	prcm_clear_statdep_regs();

	ret = request_irq(OMAP44XX_IRQ_PRCM,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       OMAP44XX_IRQ_PRCM);
		goto err2;
	}

	so_ram_address = (void *)dma_alloc_so_coherent(NULL, 1024,
			(dma_addr_t *)&ram_addr, GFP_KERNEL);

	if (!so_ram_address) {
		printk ("omap4_pm_init: failed to allocate SO mem.\n");
		return -ENOMEM;
	}

	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		pr_err("Failed to setup powerdomains\n");
		goto err2;
	}

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (!mpu_pwrdm) {
		printk(KERN_ERR "Failed to get lookup for MPU pwrdm's\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

	/* Get handles for VDD's for enabling/disabling SR */
	vdd_mpu = omap_voltage_domain_get("mpu");
	if (IS_ERR(vdd_mpu)) {
		printk(KERN_ERR "Failed to get handle for VDD MPU\n");
		goto err2;
	}

	vdd_iva = omap_voltage_domain_get("iva");
	if (IS_ERR(vdd_iva)) {
		printk(KERN_ERR "Failed to get handle for VDD IVA\n");
		goto err2;
	}

	vdd_core = omap_voltage_domain_get("core");
	if (IS_ERR(vdd_core)) {
		printk(KERN_ERR "Failed to get handle for VDD CORE\n");
		goto err2;
	}

	omap4_mpuss_init();
#endif

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	omap4_idle_init();
	omap4_trigger_ioctrl();

err2:
	return ret;
}
late_initcall(omap4_pm_init);
