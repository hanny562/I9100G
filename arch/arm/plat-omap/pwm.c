/*
 * linux/arch/arm/plat-omap/pwm.c
 *
 * simple driver for PWM (Pulse Width Modulator) controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2008-02-13  initial version
 *		eric miao <eric.miao@marvell.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <asm/uaccess.h>	/* for copy_from_user */
#include <linux/slab.h>

#include <asm/div64.h>

/* PWM registers and bits definitions */
#define GPT1MS_TIOCP_CFG	0x10
#define GPT_TCLR		0x24
#define GPT_TCRR		0x28
#define GPT_TLDR		0x2C
#define GPT_TTGR		0x30
#define GPT_TWPS		0x34
#define GPT_TMAR		0x38

#define OMAP44XX_GPTIMER10_BASE				0x48086000
#define CM_L4PER_CLKSTCTRL				0x4A009400
#define CTL_CORE_PAD0_DPM_EMU17_PAD1_DPM_EMU18		0x4a1001d0
#define CM_L4PER_GPTIMER10_CLKCTRL			0x4A009428

static const struct platform_device_id pwm_id_table[] = {
	{ "omap44xx-pwm", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, pwm_id_table);

struct pwm_device {
	struct list_head	node;
	struct platform_device	*pdev;

	const char	*label;
	struct clk	*clk;
	int		clk_enabled;
	void __iomem	*mmio_base;

	unsigned int	use_count;
	unsigned int	pwm_id;
};

void __iomem *cm_l4per_gptimer10_clkctrl;

static int pwm_controller_init(struct pwm_device *pwm)
{
	void __iomem *ctl_corepad_reg;
	void __iomem *cm_l4perclkstctrl;

	cm_l4perclkstctrl = (u32 *) ioremap(CM_L4PER_CLKSTCTRL,4);
	ctl_corepad_reg = (u32 *) ioremap(CTL_CORE_PAD0_DPM_EMU17_PAD1_DPM_EMU18,4);
	cm_l4per_gptimer10_clkctrl = (u32 *) ioremap(CM_L4PER_GPTIMER10_CLKCTRL, 4);

	__raw_writel(0x004cff00, cm_l4perclkstctrl);
	__raw_writel((__raw_readl(cm_l4per_gptimer10_clkctrl)) | 0x02, cm_l4per_gptimer10_clkctrl);
	__raw_writel(((__raw_readl(ctl_corepad_reg)&(0xfff8ffff)) | 0x00010000), ctl_corepad_reg);

	iounmap(cm_l4perclkstctrl);
	iounmap(ctl_corepad_reg);

	__raw_writel(0x328, pwm->mmio_base+GPT1MS_TIOCP_CFG);
	__raw_writel(0x000018b6, pwm->mmio_base+GPT_TCLR);
	__raw_writel(0xfffff000, (pwm->mmio_base+GPT_TLDR));
}

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned char setpwm;

	if (pwm == NULL || period_ns == 0 || duty_ns > period_ns)
		return -EINVAL;

	setpwm= (255 * duty_ns) / period_ns;
	__raw_writel((__raw_readl(cm_l4per_gptimer10_clkctrl)) | 0x02, cm_l4per_gptimer10_clkctrl);
	omap_writel(0x00001836, (OMAP44XX_GPTIMER10_BASE + GPT_TCLR));
	omap_writel(0x0, (OMAP44XX_GPTIMER10_BASE + GPT_TTGR));
	omap_writel((0xfffff000 + (setpwm << 4)), (OMAP44XX_GPTIMER10_BASE + GPT_TMAR));
	omap_writel((omap_readl(OMAP44XX_GPTIMER10_BASE + GPT_TCLR) | 0x40), (OMAP44XX_GPTIMER10_BASE + GPT_TCLR));

	return 0;
}
EXPORT_SYMBOL(pwm_config);

int pwm_enable(struct pwm_device *pwm)
{
	__raw_writel((__raw_readl(cm_l4per_gptimer10_clkctrl)) | 0x02, cm_l4per_gptimer10_clkctrl);
	__raw_writel((__raw_readl(pwm->mmio_base+GPT_TCLR)| 0x1), pwm->mmio_base+GPT_TCLR);
	return 0;
}
EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	__raw_writel((__raw_readl(cm_l4per_gptimer10_clkctrl)) | 0x02, cm_l4per_gptimer10_clkctrl);
	__raw_writel((__raw_readl(pwm->mmio_base+GPT_TCLR) & ~0x1), pwm->mmio_base+GPT_TCLR);
}
EXPORT_SYMBOL(pwm_disable);

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
	struct pwm_device *pwm;
	int found = 0;
	mutex_lock(&pwm_lock);
	list_for_each_entry(pwm, &pwm_list, node) {
		if (pwm->pwm_id == pwm_id) {
			found = 1;
			break;
		}
	}

	if (found) {
		if (pwm->use_count == 0) {
			pwm->use_count++;
			pwm->label = label;
		} else
			pwm = ERR_PTR(-EBUSY);
	} else
		pwm = ERR_PTR(-ENOENT);

	mutex_unlock(&pwm_lock);
	return pwm;
}
EXPORT_SYMBOL(pwm_request);


void pwm_free(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);
	if (pwm->use_count) {
		pwm->use_count--;
		pwm->label = NULL;
	} else
		pr_warning("PWM device already freed\n");

	mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_free);

static inline void __add_pwm(struct pwm_device *pwm)
{
	mutex_lock(&pwm_lock);
	list_add_tail(&pwm->node, &pwm_list);
	mutex_unlock(&pwm_lock);
}

static int __devinit pwm_probe(struct platform_device *pdev)
{
	struct pwm_device *pwm = NULL;
	int ret = 0;

	pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
	if (pwm == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	pwm->clk_enabled = 0;
	pwm->use_count = 0;
	pwm->pwm_id = pdev->id;
	pwm->pdev = pdev;
	pwm->mmio_base = ioremap(OMAP44XX_GPTIMER10_BASE, SZ_4K);
	if (pwm->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	__add_pwm(pwm);

	platform_set_drvdata(pdev, pwm);
	pwm_controller_init(pwm);

	return 0;

err_free_mem:
	iounmap(pwm->mmio_base);
	kfree(pwm);
	return ret;
}

static int __devexit pwm_remove(struct platform_device *pdev)
{
	struct pwm_device *pwm;
	struct resource *r;

	pwm = platform_get_drvdata(pdev);
	if (pwm == NULL)
		return -ENODEV;

	mutex_lock(&pwm_lock);
	list_del(&pwm->node);
	mutex_unlock(&pwm_lock);

	iounmap(pwm->mmio_base);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	kfree(pwm);
	return 0;
}

static struct platform_driver pwm_driver = {
	.driver		= {
		.name	= "omap44xx-pwm",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_probe,
	.remove		= __devexit_p(pwm_remove),
	.id_table	= pwm_id_table,

};

static int __init pwm_init(void)
{
	return platform_driver_register(&pwm_driver);
}
arch_initcall(pwm_init);

static void __exit pwm_exit(void)
{
	platform_driver_unregister(&pwm_driver);
}
module_exit(pwm_exit);

MODULE_LICENSE("GPL v2");
