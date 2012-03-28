/*
 * omap-abe-dsp.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Author : Liam Girdwood <lrg@slimlogic.co.uk>
 *
 */

#undef DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c/twl.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/debugfs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/dma.h>
#include <plat/omap-pm.h>
#include <plat/powerdomain.h>
#include <plat/prcm.h>

#include <mach/omap4-common.h>

#include "../../../arch/arm/mach-omap2/pm.h"

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/omap-abe-dsp.h>

#include "omap-abe-dsp.h"
#include "omap-abe-coef.h"
#include "omap-abe.h"
#include "abe/abe_main.h"


//#define SEC_AUDIO_DEBUG
//#define SEC_AUDIO_VOLUME_DEBUG
#define KEYTONE_MUTE_GLITCH
//#define PING_PONG_4

#ifdef SEC_AUDIO_DEBUG
#define P(format,...)\
  printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

#ifdef SEC_AUDIO_VOLUME_DEBUG
#define P_Volume(format,...)\
  printk("[audio gain:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P_Volume(format,...)
#endif


// TODO: change to S16 and use ARM SIMD to re-format to S32
#define ABE_FORMATS	 (SNDRV_PCM_FMTBIT_S32_LE)

// TODO: make all these into virtual registers or similar - split out by type */
#define ABE_NUM_MIXERS		24
#define ABE_NUM_MUXES		12
#define ABE_NUM_WIDGETS	        46	/* TODO - refine this val */
#define ABE_NUM_DAPM_REG		\
	(ABE_NUM_MIXERS + ABE_NUM_MUXES + ABE_NUM_WIDGETS)
#define ABE_WIDGET_START	(ABE_NUM_MIXERS + ABE_NUM_MUXES)
#define ABE_WIDGET_END	(ABE_WIDGET_START + ABE_NUM_WIDGETS)
#define ABE_FE_START		ABE_WIDGET_START
#define ABE_NUM_FE		10
#define ABE_FE_END		(ABE_FE_START + ABE_NUM_FE)
#define ABE_BE_START		ABE_FE_END
#define ABE_NUM_BE		11
#define ABE_BE_END	(ABE_BE_START + ABE_NUM_BE)
#define ABE_MUX_BASE	ABE_NUM_MIXERS

/* Uplink MUX path identifiers from ROUTE_UL */
#define ABE_MM_UL1(x)		(x + ABE_NUM_MIXERS)
#define ABE_MM_UL2(x)		(x + ABE_NUM_MIXERS + 8)
#define ABE_VX_UL(x)			(x + ABE_NUM_MIXERS + 10)
#define ABE_WIDGET(x)		(x + ABE_NUM_MIXERS + ABE_NUM_MUXES)
//#define ABE_BE_WIDGET(x)		(x + ABE_NUM_MIXERS + ABE_NUM_MUXES)

#define VIRT_SWITCH	0

// TODO: OPP bitmask - Use HAL version after update
#define ABE_OPP_25		0
#define ABE_OPP_50		1
#define ABE_OPP_100		2

#define ABE_ROUTES_UL		14

/* size in bytes of debug options */
#define ABE_DBG_FLAG1_SIZE	0
#define ABE_DBG_FLAG2_SIZE	0
#define ABE_DBG_FLAG3_SIZE	0

bool abe_can_enter_dpll_cascading = false;

/* TODO: fine tune for ping pong - buffer is 2 periods of 12k each*/
static const struct snd_pcm_hardware omap_abe_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 4 * 1024,
	.period_bytes_max	= 24 * 1024,
#ifdef KEYTONE_MUTE_GLITCH	
#ifdef PING_PONG_4
	.periods_min		= 4,
	.periods_max		= 4,
#else
	.periods_min		= 8,
	.periods_max		= 8,	
#endif	
#else
	.periods_min		= 2,
	.periods_max		= 2,
#endif	
	.buffer_bytes_max	= 24 * 1024 * 2,
};

/*
 * ABE driver
 *
 * TODO: tasks 1,2,3,4,5,6,7,8 below
 *
 * This driver is responsible for :-
 * 1) ABE init - including loading ABE firmware
 * 2) ABE coefficient mgmt - inc user space re-calcs
 * 8) Process ABE platform data
 *
 */

/* ABE private data */
struct abe_data {
	struct omap4_abe_dsp_pdata *abe_pdata;
	struct platform_device *pdev;
	struct snd_soc_platform *platform;

	struct delayed_work delayed_work;
	struct mutex mutex;
	struct mutex opp_mutex;

	struct clk *clk;

	void __iomem *io_base;
	int irq;
	int opp;

	int fe_id;
	int fe_active[ABE_NUM_FE];

	unsigned int dl1_equ_profile;
	unsigned int dl20_equ_profile;
	unsigned int dl21_equ_profile;
	unsigned int sdt_equ_profile;
	unsigned int amic_equ_profile;
	unsigned int dmic_equ_profile;

	int dl1_mono;
        int dl2_mono;

	int active;

	/* DAPM mixer config - TODO: some of this can be replaced with HAL update */
	u32 dapm[ABE_NUM_DAPM_REG];

	u16 router[16];

	int loss_count;

	int first_irq;

	struct snd_pcm_substream *psubs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int early_suspended;

#ifdef CONFIG_DEBUG_FS
	/* its intended we can switch on/off individual debug items */
	u32 dbg_format1; /* TODO: match flag names here to debug format flags */
	u32 dbg_format2;
	u32 dbg_format3;

	u32 dbg_buffer_bytes;
	u32 dbg_circular;
	u32 dbg_buffer_msecs;  /* size of buffer in secs */
	u32 dbg_elem_bytes;
	dma_addr_t dbg_buffer_addr;
	wait_queue_head_t wait;
	int dbg_reader_offset;
	int dbg_dma_offset;
	int dbg_complete;
	struct dentry *debugfs_root;
	struct dentry *debugfs_fmt1;
	struct dentry *debugfs_fmt2;
	struct dentry *debugfs_fmt3;
	struct dentry *debugfs_size;
	struct dentry *debugfs_data;
	struct dentry *debugfs_circ;
	struct dentry *debugfs_elem_bytes;
	char *dbg_buffer;
	struct omap_pcm_dma_data *dma_data;
	int dma_ch;
	int dma_req;
#endif
};

static struct abe_data *abe;

static struct powerdomain *abe_pwrdm;

// TODO: map to the new version of HAL
static unsigned int abe_dsp_read(struct snd_soc_platform *platform,
		unsigned int reg)
{
	struct abe_data *priv = snd_soc_platform_get_drvdata(platform);
	return priv->dapm[reg];
}

static int abe_dsp_write(struct snd_soc_platform *platform, unsigned int reg,
		unsigned int val)
{
	struct abe_data *priv = snd_soc_platform_get_drvdata(platform);
	priv->dapm[reg] = val;

	//dev_dbg(platform->dev, "fe: %d widget %d %s\n", abe->fe_id,
	//		reg - ABE_WIDGET_START, val ? "on" : "off");
	return 0;
}

static void abe_irq_pingpong_subroutine(void)
{
	u32 dst, n_bytes;

	abe_read_next_ping_pong_buffer(MM_DL_PORT, &dst, &n_bytes);
	abe_set_ping_pong_buffer(MM_DL_PORT, n_bytes);

	/* Do not call ALSA function for first IRQ */
	if (abe->first_irq) {
		abe->first_irq = 0;
	} else {
		/* 
		 * Do not call ALSA function when substream may not have a buffer and
		 * also when runtime information is NULL.
		 */
		if (abe->psubs && (abe->psubs->hw_no_buffer) && (abe->psubs->runtime)) 
			snd_pcm_period_elapsed(abe->psubs);
	}
}

static irqreturn_t abe_irq_handler(int irq, void *dev_id)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	/* TODO: handle underruns/overruns/errors */
	pm_runtime_get_sync(&pdev->dev);
	abe_clear_irq();
	abe_irq_processing();
	pm_runtime_put_sync(&pdev->dev);

	return IRQ_HANDLED;
}

static int abe_init_engine(struct snd_soc_platform *platform)
{
	struct abe_data *priv = snd_soc_platform_get_drvdata(platform);
#ifndef CONFIG_PM_RUNTIME
	struct omap4_abe_dsp_pdata *pdata = priv->abe_pdata;
#endif
	struct platform_device *pdev = priv->pdev;
	abe_equ_t dl2_eq;
	int ret = 0;

	abe_init_mem(abe->io_base);

	/* aess_clk has to be enabled to access hal register.
	 * Disable the clk after it has been used.
	 */
	pm_runtime_get_sync(&pdev->dev);

	ret = request_threaded_irq(abe->irq, NULL, abe_irq_handler,
				IRQF_ONESHOT, "ABE", (void *)abe);
	if (ret) {
		dev_err(platform->dev, "request for ABE IRQ %d failed %d\n",
				abe->irq, ret);
		return ret;
	}

	abe_reset_hal();

	abe_load_fw();	// TODO: use fw API here

	/* Config OPP 100 for now */
	mutex_lock(&abe->opp_mutex);
	abe_set_opp_processing(ABE_OPP100);
	abe->opp = 100;
	mutex_unlock(&abe->opp_mutex);

	/* "tick" of the audio engine */
	abe_write_event_generator(EVENT_TIMER);

	dl2_eq.equ_length = NBDL2COEFFS;

	/* build the coefficient parameter for the equalizer api */
	memcpy(dl2_eq.coef.type1, dl20_equ_coeffs[1],
				sizeof(dl20_equ_coeffs[1]));

	/* load the high-pass coefficient of IHF-Right */
	abe_write_equalizer(EQ2L, &dl2_eq);
	abe->dl20_equ_profile = 1;

	/* build the coefficient parameter for the equalizer api */
	memcpy(dl2_eq.coef.type1, dl21_equ_coeffs[1],
				sizeof(dl21_equ_coeffs[1]));

	/* load the high-pass coefficient of IHF-Left */
	abe_write_equalizer(EQ2R, &dl2_eq);
	abe->dl21_equ_profile = 1;

	pm_runtime_put_sync(&pdev->dev);

	/* set initial state to all-pass with gain=1 coefficients */
	abe->amic_equ_profile = 0;
	abe->dmic_equ_profile = 0;
	abe->dl1_equ_profile = 0;
	abe->sdt_equ_profile = 0;


	return ret;
}

void abe_dsp_pm_get(void)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	pm_runtime_get_sync(&pdev->dev);
}

void abe_dsp_pm_put(void)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	pm_runtime_put_sync(&pdev->dev);
}

void abe_dsp_shutdown(void)
{
        /* TODO: do not use abe global structure to assign pdev */
        struct platform_device *pdev = abe->pdev;

	if (!abe->active && !abe_check_activity()) {
		abe_set_opp_processing(ABE_OPP25);
		abe->opp = 25;
		abe_stop_event_generator();
		udelay(250);
		omap_device_set_rate(&pdev->dev, &pdev->dev, 0);
	}
}

void abe_dsp_mcpdm_shutdown(void)
{
	mutex_lock(&abe->mutex);

	abe_dsp_shutdown();

	mutex_unlock(&abe->mutex);

	return;
}

static int abe_fe_active_count(struct abe_data *abe)
{
	int i, count = 0;

	for (i = 0; i < ABE_NUM_FE; i++)
		count += abe->fe_active[i];

	return count;
}

static int abe_fe_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	int index, active, ret = 0;

	if ((w->reg < ABE_FE_START) || (w->reg >= ABE_FE_END))
		return -EINVAL;

	index = w->reg - ABE_FE_START;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		abe->fe_active[index]++;
		active = abe_fe_active_count(abe);
		if (!abe->early_suspended || (active > 1) || !abe->fe_active[6])
			ret = dpll_cascading_blocker_hold(&abe->pdev->dev);
	} else {
		abe->fe_active[index]--;
	}

	return ret;
}

/*
 * These TLV settings will need fine tuned for each individual control
 */

/* Media DL1 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(mm_dl1_tlv, -12000, 100, 3000);

/* Media DL1 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(tones_dl1_tlv, -12000, 100, 3000);

/* Media DL1 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(voice_dl1_tlv, -12000, 100, 3000);

/* Media DL1 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(capture_dl1_tlv, -12000, 100, 3000);

/* Media DL2 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(mm_dl2_tlv, -12000, 100, 3000);

/* Media DL2 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(tones_dl2_tlv, -12000, 100, 3000);

/* Media DL2 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(voice_dl2_tlv, -12000, 100, 3000);

/* Media DL2 volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(capture_dl2_tlv, -12000, 100, 3000);

/* SDT volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(sdt_ul_tlv, -12000, 100, 3000);

/* SDT volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(sdt_dl_tlv, -12000, 100, 3000);

/* AUDUL volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(audul_mm_tlv, -12000, 100, 3000);

/* AUDUL volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(audul_tones_tlv, -12000, 100, 3000);

/* AUDUL volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(audul_vx_ul_tlv, -12000, 100, 3000);

/* AUDUL volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(audul_vx_dl_tlv, -12000, 100, 3000);

/* VXREC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(vxrec_mm_dl_tlv, -12000, 100, 3000);

/* VXREC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(vxrec_tones_tlv, -12000, 100, 3000);

/* VXREC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(vxrec_vx_dl_tlv, -12000, 100, 3000);

/* VXREC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(vxrec_vx_ul_tlv, -12000, 100, 3000);

/* DMIC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(dmic_tlv, -12000, 100, 3000);

/* AMIC volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(amic_tlv, -12000, 100, 3000);

/* BT UL volume control from -120 to 30 dB in 1 dB steps */
static DECLARE_TLV_DB_SCALE(btul_tlv, -12000, 100, 3000);

//TODO: we have to use the shift value atm to represent register id due to current HAL
static int dl1_put_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
		abe_enable_gain(MIXDL1, mc->reg);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
		abe_disable_gain(MIXDL1, mc->reg);
	}
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int dl2_put_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	
	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
		abe_enable_gain(MIXDL2, mc->reg);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
		abe_disable_gain(MIXDL2, mc->reg);
	}

	pm_runtime_put_sync(&pdev->dev);
	return 1;
}

static int audio_ul_put_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
		abe_enable_gain(MIXAUDUL, mc->reg);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
		abe_disable_gain(MIXAUDUL, mc->reg);
	}
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int vxrec_put_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	
	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
		abe_enable_gain(MIXVXREC, mc->reg);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
		abe_disable_gain(MIXVXREC, mc->reg);
	}
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int sdt_put_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
		abe_enable_gain(MIXSDT, mc->reg);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
		abe_disable_gain(MIXSDT, mc->reg);
	}
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static void abe_dsp_set_mono_mixer(unsigned int id, int enable)
{
       /* TODO: do not use abe global structure to assign pdev */
       struct platform_device *pdev = abe->pdev;

       switch (id) {
       case MIXDL1:
               abe->dl1_mono = enable;
               break;
       case MIXDL2:
               abe->dl2_mono = enable;
               break;
       default:
               return;
       }

       pm_runtime_get_sync(&pdev->dev);
       abe_mono_mixer(id, enable);
       pm_runtime_put_sync(&pdev->dev);
}

static int abe_put_mono_mixer(struct snd_kcontrol *kcontrol,
       struct snd_ctl_elem_value *ucontrol)
{
       struct soc_mixer_control *mc =
               (struct soc_mixer_control *)kcontrol->private_value;

       abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
       abe_dsp_set_mono_mixer(mc->reg, abe->dapm[mc->shift]);

       return 1;
}

static int abe_get_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = abe->dapm[mc->shift];

	return 0;
}

/* router IDs that match our mixer strings */
static const abe_router_t router[] = {
		ZERO_labelID, /* strangely this is not 0 */
		DMIC1_L_labelID, DMIC1_R_labelID,
		DMIC2_L_labelID, DMIC2_R_labelID,
		DMIC3_L_labelID, DMIC3_R_labelID,
		BT_UL_L_labelID, BT_UL_R_labelID,
		AMIC_L_labelID, AMIC_R_labelID,
		VX_REC_L_labelID, VX_REC_R_labelID,
};

static int ul_mux_put_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	int mux = ucontrol->value.enumerated.item[0];
	int reg = e->reg - ABE_MUX_BASE, i;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",e->reg, ucontrol->value.integer.value[0]);
	
	pm_runtime_get_sync(&pdev->dev);

	if (mux >= ABE_ROUTES_UL)
		return 0;

	if (reg < 8) {
		/* 0  .. 9   = MM_UL */
		abe->router[reg] = router[mux];
	} else if (reg < 12) {
		/* 10 .. 11  = MM_UL2 */
		/* 12 .. 13  = VX_UL */
		abe->router[reg + 2] = router[mux];
	}

	/* there is a 2 slot gap in the table, making it ABE_ROUTES_UL + 2 in size */
	for (i = 0; i < ABE_ROUTES_UL + 2; i++)
		dev_dbg(widget->dapm->dev, "router table [%d] = %d\n", i, abe->router[i]);

	/* 2nd arg here is unused */
	abe_set_router_configuration(UPROUTE, 0, (u32 *)abe->router);

	abe->dapm[e->reg] = ucontrol->value.integer.value[0];
	snd_soc_dapm_mux_update_power(widget, kcontrol, abe->dapm[e->reg], mux, e);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int ul_mux_get_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;

	// TODO: get mux via HAL

	ucontrol->value.integer.value[0] = abe->dapm[e->reg];
	return 0;
}


static int abe_put_switch(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	pm_runtime_get_sync(&pdev->dev);

	if (ucontrol->value.integer.value[0]) {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 1);
	} else {
		abe->dapm[mc->shift] = ucontrol->value.integer.value[0];
		snd_soc_dapm_mixer_update_power(widget, kcontrol, 0);
	}
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}


static int volume_put_sdt_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	pm_runtime_get_sync(&pdev->dev);

	abe_write_mixer(MIXSDT, -12000 + (ucontrol->value.integer.value[0] * 100),
				RAMP_0MS, mc->reg);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_put_audul_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_write_mixer(MIXAUDUL, -12000 + (ucontrol->value.integer.value[0] * 100),
				RAMP_0MS, mc->reg);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_put_vxrec_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_write_mixer(MIXVXREC, -12000 + (ucontrol->value.integer.value[0] * 100),
				RAMP_0MS, mc->reg);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_put_dl1_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_write_mixer(MIXDL1, -12000 + (ucontrol->value.integer.value[0] * 100),
				RAMP_0MS, mc->reg);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_put_dl2_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	
	pm_runtime_get_sync(&pdev->dev);
	abe_write_mixer(MIXDL2, -12000 + (ucontrol->value.integer.value[0] * 100),
				RAMP_0MS, mc->reg);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_put_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_write_gain(mc->reg,
		       -12000 + (ucontrol->value.integer.value[0] * 100),
		       RAMP_20MS, mc->shift);
	abe_write_gain(mc->reg,
		       -12000 + (ucontrol->value.integer.value[1] * 100),
		       RAMP_20MS, mc->rshift);
	pm_runtime_put_sync(&pdev->dev);

	return 1;
}

static int volume_get_dl1_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u32 val;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);
	pm_runtime_get_sync(&pdev->dev);
	abe_read_mixer(MIXDL1, &val, mc->reg);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int volume_get_dl2_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	u32 val;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_read_mixer(MIXDL2, &val, mc->reg);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int volume_get_audul_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	u32 val;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_read_mixer(MIXAUDUL, &val, mc->reg);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int volume_get_vxrec_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	u32 val;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_read_mixer(MIXVXREC, &val, mc->reg);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int volume_get_sdt_mixer(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	u32 val;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_read_mixer(MIXSDT, &val, mc->reg);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int volume_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	u32 val;

	P_Volume("reg %d, value %d",mc->reg, ucontrol->value.integer.value[0]);

	pm_runtime_get_sync(&pdev->dev);
	abe_read_gain(mc->reg, &val, mc->shift);
	ucontrol->value.integer.value[0] = (val + 12000) / 100;
	abe_read_gain(mc->reg, &val, mc->rshift);
	ucontrol->value.integer.value[1] = (val + 12000) / 100;
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}


static int abe_get_equalizer(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *eqc = (struct soc_enum *)kcontrol->private_value;
	switch (eqc->reg) {
	case EQ1:
		ucontrol->value.integer.value[0] = abe->dl1_equ_profile;
		break;
	case EQ2L:
		ucontrol->value.integer.value[0] = abe->dl20_equ_profile;
		break;
	case EQ2R:
		ucontrol->value.integer.value[0] = abe->dl21_equ_profile;
		break;
	case EQAMIC:
		ucontrol->value.integer.value[0] = abe->amic_equ_profile;
		break;
	case EQDMIC:
		ucontrol->value.integer.value[0] = abe->dmic_equ_profile;
		break;
	case EQSDT:
		ucontrol->value.integer.value[0] = abe->sdt_equ_profile;
		break;
	default:
		break;
	}

	return 0;
}

static void abe_dsp_set_equalizer(unsigned int id, unsigned int profile)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	abe_equ_t equ_params;

	switch (id) {
	case EQ1:
		equ_params.equ_length = NBDL1COEFFS;
		memcpy(equ_params.coef.type1, dl1_equ_coeffs[profile],
				sizeof(dl1_equ_coeffs[profile]));
		abe->dl1_equ_profile = profile;
		break;
	case EQ2L:
		equ_params.equ_length = NBDL2COEFFS;
		memcpy(equ_params.coef.type1, dl20_equ_coeffs[profile],
				sizeof(dl20_equ_coeffs[profile]));
		abe->dl20_equ_profile = profile;
		break;
	case EQ2R:
		equ_params.equ_length = NBDL2COEFFS;
		memcpy(equ_params.coef.type1, dl21_equ_coeffs[profile],
				sizeof(dl21_equ_coeffs[profile]));
		abe->dl21_equ_profile = profile;
		break;
	case EQAMIC:
		equ_params.equ_length = NBAMICCOEFFS;
		memcpy(equ_params.coef.type1, amic_equ_coeffs[profile],
				sizeof(amic_equ_coeffs[profile]));
		abe->amic_equ_profile = profile;
		break;
	case EQDMIC:
		equ_params.equ_length = NBDMICCOEFFS;
		memcpy(equ_params.coef.type1, dmic_equ_coeffs[profile],
				sizeof(dmic_equ_coeffs[profile]));
		abe->dmic_equ_profile = profile;
		break;
	case EQSDT:
		equ_params.equ_length = NBSDTCOEFFS;
		memcpy(equ_params.coef.type1, sdt_equ_coeffs[profile],
				sizeof(sdt_equ_coeffs[profile]));
		abe->sdt_equ_profile = profile;
		break;
	default:
		return;
	}

	pm_runtime_get_sync(&pdev->dev);
	abe_write_equalizer(id, &equ_params);
	pm_runtime_put_sync(&pdev->dev);
}

static int abe_put_equalizer(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *eqc = (struct soc_enum *)kcontrol->private_value;
	u16 val = ucontrol->value.enumerated.item[0];

	P_Volume("reg %d, value %d\n",eqc->reg, ucontrol->value.integer.value[0]);

	if(eqc->reg == 1)//DL1 EQ
		printk("abe_put_equalizer reg %d, value %d\n", eqc->reg, ucontrol->value.integer.value[0]);

	abe_dsp_set_equalizer(eqc->reg, val);

	return 1;
}

static const char *dl1_equ_texts[] = {
	"Flat response",
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -20dB",
	"4KHz LPF - 0dB",
};

static const char *dl20_equ_texts[] = {
	"Flat response",
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -20dB",
};

static const char *dl21_equ_texts[] = {
	"Flat response",
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -20dB",
};

static const char *amic_equ_texts[] = {
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -18dB",
};

static const char *dmic_equ_texts[] = {
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -18dB",
};

static const char *sdt_equ_texts[] = {
	"Flat response",
	"High-pass 0dB",
	"High-pass -12dB",
	"High-pass -20dB",
};

static const struct soc_enum dl1_equalizer_enum =
	SOC_ENUM_SINGLE(EQ1, 0, NBDL1EQ_PROFILES, dl1_equ_texts);

static const struct soc_enum dl20_equalizer_enum =
	SOC_ENUM_SINGLE(EQ2L, 0, NBDL20EQ_PROFILES, dl20_equ_texts);

static const struct soc_enum dl21_equalizer_enum =
	SOC_ENUM_SINGLE(EQ2R, 0, NBDL21EQ_PROFILES, dl21_equ_texts);

static const struct soc_enum amic_equalizer_enum =
	SOC_ENUM_SINGLE(EQAMIC, 0, NBAMICEQ_PROFILES, amic_equ_texts);

static const struct soc_enum dmic_equalizer_enum =
	SOC_ENUM_SINGLE(EQDMIC, 0, NBDMICEQ_PROFILES, dmic_equ_texts);

static const struct soc_enum sdt_equalizer_enum =
	SOC_ENUM_SINGLE(EQSDT, 0, NBSDTEQ_PROFILES, sdt_equ_texts);

static const char *route_ul_texts[] =
	{"None", "DMic0L", "DMic0R", "DMic1L", "DMic1R", "DMic2L", "DMic2R",
	"BT Left", "BT Right", "AMic0", "AMic1", "VX Left", "VX Right"};

/* ROUTE_UL Mux table */
static const struct soc_enum abe_enum[] = {
		SOC_ENUM_SINGLE(ABE_MM_UL1(0), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(1), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(2), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(3), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(4), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(5), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(6), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL1(7), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL2(0), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_MM_UL2(1), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_VX_UL(0), 0, 13, route_ul_texts),
		SOC_ENUM_SINGLE(ABE_VX_UL(1), 0, 13, route_ul_texts),
};

static const struct snd_kcontrol_new mm_ul00_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[0],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul01_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[1],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul02_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[2],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul03_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[3],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul04_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[4],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul05_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[5],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul06_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[6],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul07_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[7],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul10_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[8],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_ul11_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[9],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_vx0_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[10],
	ul_mux_get_route, ul_mux_put_route);

static const struct snd_kcontrol_new mm_vx1_control =
	SOC_DAPM_ENUM_EXT("Route", abe_enum[11],
	ul_mux_get_route, ul_mux_put_route);

/* DL1 mixer paths */
static const struct snd_kcontrol_new dl1_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones", MIX_DL1_INPUT_TONES, 0, 1, 0,
		abe_get_mixer, dl1_put_mixer),
	SOC_SINGLE_EXT("Voice", MIX_DL1_INPUT_VX_DL, 1, 1, 0,
		abe_get_mixer, dl1_put_mixer),
	SOC_SINGLE_EXT("Capture", MIX_DL1_INPUT_MM_UL2, 2, 1, 0,
		abe_get_mixer, dl1_put_mixer),
	SOC_SINGLE_EXT("Multimedia", MIX_DL1_INPUT_MM_DL, 3, 1, 0,
		abe_get_mixer, dl1_put_mixer),
};

/* DL2 mixer paths */
static const struct snd_kcontrol_new dl2_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones", MIX_DL2_INPUT_TONES, 4, 1, 0,
		abe_get_mixer, dl2_put_mixer),
	SOC_SINGLE_EXT("Voice", MIX_DL2_INPUT_VX_DL, 5, 1, 0,
		abe_get_mixer, dl2_put_mixer),
	SOC_SINGLE_EXT("Capture", MIX_DL2_INPUT_MM_UL2, 6, 1, 0,
		abe_get_mixer, dl2_put_mixer),
	SOC_SINGLE_EXT("Multimedia", MIX_DL2_INPUT_MM_DL, 7, 1, 0,
		abe_get_mixer, dl2_put_mixer),
};

/* AUDUL ("Voice Capture Mixer") mixer paths */
static const struct snd_kcontrol_new audio_ul_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones Playback", MIX_AUDUL_INPUT_TONES, 8, 1, 0,
		abe_get_mixer, audio_ul_put_mixer),
	SOC_SINGLE_EXT("Media Playback", MIX_AUDUL_INPUT_MM_DL, 9, 1, 0,
		abe_get_mixer, audio_ul_put_mixer),
	SOC_SINGLE_EXT("Capture", MIX_AUDUL_INPUT_UPLINK, 10, 1, 0,
		abe_get_mixer, audio_ul_put_mixer),
};

/* VXREC ("Capture Mixer")  mixer paths */
static const struct snd_kcontrol_new vx_rec_mixer_controls[] = {
	SOC_SINGLE_EXT("Tones", MIX_VXREC_INPUT_TONES, 11, 1, 0,
		abe_get_mixer, vxrec_put_mixer),
	SOC_SINGLE_EXT("Voice Playback", MIX_VXREC_INPUT_VX_DL, 12, 1, 0,
		abe_get_mixer, vxrec_put_mixer),
	SOC_SINGLE_EXT("Voice Capture", MIX_VXREC_INPUT_VX_UL, 13, 1, 0,
		abe_get_mixer, vxrec_put_mixer),
	SOC_SINGLE_EXT("Media Playback", MIX_VXREC_INPUT_MM_DL, 14, 1, 0,
		abe_get_mixer, vxrec_put_mixer),
};

/* SDT ("Sidetone Mixer") mixer paths */
static const struct snd_kcontrol_new sdt_mixer_controls[] = {
	SOC_SINGLE_EXT("Capture", MIX_SDT_INPUT_UP_MIXER, 15, 1, 0,
		abe_get_mixer, sdt_put_mixer),
	SOC_SINGLE_EXT("Playback", MIX_SDT_INPUT_DL1_MIXER, 16, 1, 0,
		abe_get_mixer, sdt_put_mixer),
};

/* Virtual PDM_DL Switch */
static const struct snd_kcontrol_new pdm_dl1_switch_controls =
	SOC_SINGLE_EXT("Switch", VIRT_SWITCH, 17, 1, 0,
			abe_get_mixer, abe_put_switch);

/* Virtual BT_VX_DL Switch */
static const struct snd_kcontrol_new bt_vx_dl_switch_controls =
	SOC_SINGLE_EXT("Switch", VIRT_SWITCH, 18, 1, 0,
			abe_get_mixer, abe_put_switch);

/* Virtual MM_EXT_DL Switch */
static const struct snd_kcontrol_new mm_ext_dl_switch_controls =
	SOC_SINGLE_EXT("Switch", VIRT_SWITCH, 19, 1, 0,
			abe_get_mixer, abe_put_switch);

/* Virtual MM_EXT_UL Switch */
static const struct snd_kcontrol_new mm_ext_ul_switch_controls =
	SOC_SINGLE_EXT("Switch", VIRT_SWITCH, 20, 1, 0,
			abe_get_mixer, abe_put_switch);

/* Virtual PDM_UL Switch */
static const struct snd_kcontrol_new pdm_ul1_switch_controls =
	SOC_SINGLE_EXT("Switch", VIRT_SWITCH, 21, 1, 0,
			abe_get_mixer, abe_put_switch);

static const struct snd_kcontrol_new abe_controls[] = {
	/* DL1 mixer gains */
	SOC_SINGLE_EXT_TLV("DL1 Media Playback Volume",
		MIX_DL1_INPUT_MM_DL, 0, 149, 0,
		volume_get_dl1_mixer, volume_put_dl1_mixer, mm_dl1_tlv),
	SOC_SINGLE_EXT_TLV("DL1 Tones Playback Volume",
		MIX_DL1_INPUT_TONES, 0, 149, 0,
		volume_get_dl1_mixer, volume_put_dl1_mixer, tones_dl1_tlv),
	SOC_SINGLE_EXT_TLV("DL1 Voice Playback Volume",
		MIX_DL1_INPUT_VX_DL, 0, 149, 0,
		volume_get_dl1_mixer, volume_put_dl1_mixer, voice_dl1_tlv),
	SOC_SINGLE_EXT_TLV("DL1 Capture Playback Volume",
		MIX_DL1_INPUT_MM_UL2, 0, 149, 0,
		volume_get_dl1_mixer, volume_put_dl1_mixer, capture_dl1_tlv),

	/* DL2 mixer gains */
	SOC_SINGLE_EXT_TLV("DL2 Media Playback Volume",
		MIX_DL2_INPUT_MM_DL, 0, 149, 0,
		volume_get_dl2_mixer, volume_put_dl2_mixer, mm_dl2_tlv),
	SOC_SINGLE_EXT_TLV("DL2 Tones Playback Volume",
		MIX_DL2_INPUT_TONES, 0, 149, 0,
		volume_get_dl2_mixer, volume_put_dl2_mixer, tones_dl2_tlv),
	SOC_SINGLE_EXT_TLV("DL2 Voice Playback Volume",
		MIX_DL2_INPUT_VX_DL, 0, 149, 0,
		volume_get_dl2_mixer, volume_put_dl2_mixer, voice_dl2_tlv),
	SOC_SINGLE_EXT_TLV("DL2 Capture Playback Volume",
		MIX_DL2_INPUT_MM_UL2, 0, 149, 0,
		volume_get_dl2_mixer, volume_put_dl2_mixer, capture_dl2_tlv),

	/* VXREC mixer gains */
	SOC_SINGLE_EXT_TLV("VXREC Media Volume",
		MIX_VXREC_INPUT_MM_DL, 0, 149, 0,
		volume_get_vxrec_mixer, volume_put_vxrec_mixer, vxrec_mm_dl_tlv),
	SOC_SINGLE_EXT_TLV("VXREC Tones Volume",
		MIX_VXREC_INPUT_TONES, 0, 149, 0,
		volume_get_vxrec_mixer, volume_put_vxrec_mixer, vxrec_tones_tlv),
	SOC_SINGLE_EXT_TLV("VXREC Voice DL Volume",
		MIX_VXREC_INPUT_VX_UL, 0, 149, 0,
		volume_get_vxrec_mixer, volume_put_vxrec_mixer, vxrec_vx_dl_tlv),
	SOC_SINGLE_EXT_TLV("VXREC Voice UL Volume",
		MIX_VXREC_INPUT_VX_DL, 0, 149, 0,
		volume_get_vxrec_mixer, volume_put_vxrec_mixer, vxrec_vx_ul_tlv),

	/* AUDUL mixer gains */
	SOC_SINGLE_EXT_TLV("AUDUL Media Volume",
		MIX_AUDUL_INPUT_MM_DL, 0, 149, 0,
		volume_get_audul_mixer, volume_put_audul_mixer, audul_mm_tlv),
	SOC_SINGLE_EXT_TLV("AUDUL Tones Volume",
		MIX_AUDUL_INPUT_TONES, 0, 149, 0,
		volume_get_audul_mixer, volume_put_audul_mixer, audul_tones_tlv),
	SOC_SINGLE_EXT_TLV("AUDUL Voice UL Volume",
		MIX_AUDUL_INPUT_UPLINK, 0, 149, 0,
		volume_get_audul_mixer, volume_put_audul_mixer, audul_vx_ul_tlv),
	SOC_SINGLE_EXT_TLV("AUDUL Voice DL Volume",
		MIX_AUDUL_INPUT_VX_DL, 0, 149, 0,
		volume_get_audul_mixer, volume_put_audul_mixer, audul_vx_dl_tlv),

	/* SDT mixer gains */
	SOC_SINGLE_EXT_TLV("SDT UL Volume",
		MIX_SDT_INPUT_UP_MIXER, 0, 149, 0,
		volume_get_sdt_mixer, volume_put_sdt_mixer, sdt_ul_tlv),
	SOC_SINGLE_EXT_TLV("SDT DL Volume",
		MIX_SDT_INPUT_DL1_MIXER, 0, 149, 0,
		volume_get_sdt_mixer, volume_put_sdt_mixer, sdt_dl_tlv),

	/* DMIC gains */
	SOC_DOUBLE_EXT_TLV("DMIC1 UL Volume",
		GAINS_DMIC1, GAIN_LEFT_OFFSET, GAIN_RIGHT_OFFSET, 149, 0,
		volume_get_gain, volume_put_gain, dmic_tlv),

	SOC_DOUBLE_EXT_TLV("DMIC2 UL Volume",
		GAINS_DMIC2, GAIN_LEFT_OFFSET, GAIN_RIGHT_OFFSET, 149, 0,
		volume_get_gain, volume_put_gain, dmic_tlv),

	SOC_DOUBLE_EXT_TLV("DMIC3 UL Volume",
		GAINS_DMIC3, GAIN_LEFT_OFFSET, GAIN_RIGHT_OFFSET, 149, 0,
		volume_get_gain, volume_put_gain, dmic_tlv),

	SOC_DOUBLE_EXT_TLV("AMIC UL Volume",
		GAINS_AMIC, GAIN_LEFT_OFFSET, GAIN_RIGHT_OFFSET, 149, 0,
		volume_get_gain, volume_put_gain, amic_tlv),

	SOC_DOUBLE_EXT_TLV("BT UL Volume",
		GAINS_BTUL, GAIN_LEFT_OFFSET, GAIN_RIGHT_OFFSET, 149, 0,
		volume_get_gain, volume_put_gain, btul_tlv),

	SOC_ENUM_EXT("DL1 Equalizer",
			dl1_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_ENUM_EXT("DL2 Left Equalizer",
			dl20_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_ENUM_EXT("DL2 Right Equalizer",
			dl21_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_ENUM_EXT("AMIC Equalizer",
			amic_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_ENUM_EXT("DMIC Equalizer",
			dmic_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_ENUM_EXT("Sidetone Equalizer",
			sdt_equalizer_enum ,
			abe_get_equalizer, abe_put_equalizer),

	SOC_SINGLE_EXT("DL1 Mono Mixer", MIXDL1, 22, 1, 0,
                       abe_get_mixer, abe_put_mono_mixer),
                       
       SOC_SINGLE_EXT("DL2 Mono Mixer", MIXDL2, 23, 1, 0,
                       abe_get_mixer, abe_put_mono_mixer),

};

static const struct snd_soc_dapm_widget abe_dapm_widgets[] = {

	/* Frontend AIFs */
	SND_SOC_DAPM_AIF_IN_E("TONES_DL", "Tones Playback", 0,
			ABE_WIDGET(0), ABE_OPP_25, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("VX_DL", "Voice Playback", 0,
			ABE_WIDGET(1), ABE_OPP_50, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("VX_UL", "Voice Capture", 0,
			ABE_WIDGET(2), ABE_OPP_50, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	/* the MM_UL mapping is intentional */
	SND_SOC_DAPM_AIF_OUT_E("MM_UL1", "MultiMedia1 Capture", 0,
			ABE_WIDGET(3), ABE_OPP_100, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("MM_UL2", "MultiMedia2 Capture", 0,
			ABE_WIDGET(4), ABE_OPP_50, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("MM_DL", " MultiMedia1 Playback", 0,
			ABE_WIDGET(5), ABE_OPP_25, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("MM_DL_LP", " MultiMedia1 LP Playback", 0,
			ABE_WIDGET(6), ABE_OPP_25, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("VIB_DL", "Vibra Playback", 0,
			ABE_WIDGET(7), ABE_OPP_100, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_IN_E("MODEM_DL", "MODEM Playback", 0,
			ABE_WIDGET(8), ABE_OPP_50, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT_E("MODEM_UL", "MODEM Capture", 0,
			ABE_WIDGET(9), ABE_OPP_50, 0,
			abe_fe_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Backend DAIs  */
	// FIXME: must match BE order in abe_dai.h
	SND_SOC_DAPM_AIF_IN("PDM_UL1", "Analog Capture", 0,
			ABE_WIDGET(10), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_OUT("PDM_DL1", "HS Playback", 0,
			ABE_WIDGET(11), ABE_OPP_25, 0),
	SND_SOC_DAPM_AIF_OUT("PDM_DL2", "HF Playback", 0,
			ABE_WIDGET(12), ABE_OPP_100, 0),
	SND_SOC_DAPM_AIF_OUT("PDM_VIB", "Vibra Playback", 0,
			ABE_WIDGET(13), ABE_OPP_100, 0),
	SND_SOC_DAPM_AIF_IN("BT_VX_UL", "BT Capture", 0,
			ABE_WIDGET(14), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_OUT("BT_VX_DL", "BT Playback", 0,
			ABE_WIDGET(15), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_IN("MM_EXT_UL", "FM Capture", 0,
			ABE_WIDGET(16), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_OUT("MM_EXT_DL", "FM Playback", 0,
			ABE_WIDGET(17), ABE_OPP_25, 0),
	SND_SOC_DAPM_AIF_IN("DMIC0", "DMIC0 Capture", 0,
			ABE_WIDGET(18), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_IN("DMIC1", "DMIC1 Capture", 0,
			ABE_WIDGET(19), ABE_OPP_50, 0),
	SND_SOC_DAPM_AIF_IN("DMIC2", "DMIC2 Capture", 0,
			ABE_WIDGET(20), ABE_OPP_50, 0),

	/* ROUTE_UL Capture Muxes */
	SND_SOC_DAPM_MUX("MUX_UL00",
			ABE_WIDGET(21), ABE_OPP_50, 0, &mm_ul00_control),
	SND_SOC_DAPM_MUX("MUX_UL01",
			ABE_WIDGET(22), ABE_OPP_50, 0, &mm_ul01_control),
	SND_SOC_DAPM_MUX("MUX_UL02",
			ABE_WIDGET(23), ABE_OPP_50, 0, &mm_ul02_control),
	SND_SOC_DAPM_MUX("MUX_UL03",
			ABE_WIDGET(24), ABE_OPP_50, 0, &mm_ul03_control),
	SND_SOC_DAPM_MUX("MUX_UL04",
			ABE_WIDGET(25), ABE_OPP_50, 0, &mm_ul04_control),
	SND_SOC_DAPM_MUX("MUX_UL05",
			ABE_WIDGET(26), ABE_OPP_50, 0, &mm_ul05_control),
	SND_SOC_DAPM_MUX("MUX_UL06",
			ABE_WIDGET(27), ABE_OPP_50, 0, &mm_ul06_control),
	SND_SOC_DAPM_MUX("MUX_UL07",
			ABE_WIDGET(28), ABE_OPP_50, 0, &mm_ul07_control),
	SND_SOC_DAPM_MUX("MUX_UL10",
			ABE_WIDGET(29), ABE_OPP_50, 0, &mm_ul10_control),
	SND_SOC_DAPM_MUX("MUX_UL11",
			ABE_WIDGET(30), ABE_OPP_50, 0, &mm_ul11_control),
	SND_SOC_DAPM_MUX("MUX_VX0",
			ABE_WIDGET(31), ABE_OPP_50, 0, &mm_vx0_control),
	SND_SOC_DAPM_MUX("MUX_VX1",
			ABE_WIDGET(32), ABE_OPP_50, 0, &mm_vx1_control),

	/* DL1 & DL2 Playback Mixers */
	SND_SOC_DAPM_MIXER("DL1 Mixer",
			ABE_WIDGET(33), ABE_OPP_25, 0, dl1_mixer_controls,
			ARRAY_SIZE(dl1_mixer_controls)),
	SND_SOC_DAPM_MIXER("DL2 Mixer",
			ABE_WIDGET(34), ABE_OPP_100, 0, dl2_mixer_controls,
			ARRAY_SIZE(dl2_mixer_controls)),

	/* DL1 Mixer Input volumes ?????*/
	SND_SOC_DAPM_PGA("DL1 Media Volume",
			ABE_WIDGET(35), 0, 0, NULL, 0),

	/* AUDIO_UL_MIXER */
	SND_SOC_DAPM_MIXER("Voice Capture Mixer",
			ABE_WIDGET(36), ABE_OPP_50, 0, audio_ul_mixer_controls,
			ARRAY_SIZE(audio_ul_mixer_controls)),

	/* VX_REC_MIXER */
	SND_SOC_DAPM_MIXER("Capture Mixer",
			ABE_WIDGET(37), ABE_OPP_50, 0, vx_rec_mixer_controls,
			ARRAY_SIZE(vx_rec_mixer_controls)),

	/* SDT_MIXER */
	SND_SOC_DAPM_MIXER("Sidetone Mixer",
			ABE_WIDGET(38), ABE_OPP_25, 0, sdt_mixer_controls,
			ARRAY_SIZE(sdt_mixer_controls)),

	/*
	 * The Following three are virtual switches to select the output port
	 * after DL1 Gain - HAL V0.6x
	 */

	/* Virtual PDM_DL1 Switch */
	SND_SOC_DAPM_MIXER("DL1 PDM",
			ABE_WIDGET(39), ABE_OPP_25, 0, &pdm_dl1_switch_controls, 1),

	/* Virtual BT_VX_DL Switch */
	SND_SOC_DAPM_MIXER("DL1 BT_VX",
			ABE_WIDGET(40), ABE_OPP_50, 0, &bt_vx_dl_switch_controls, 1),

	/* Virtual MM_EXT_DL Switch TODO: confrm OPP level here */
	SND_SOC_DAPM_MIXER("DL1 MM_EXT",
			ABE_WIDGET(41), ABE_OPP_50, 0, &mm_ext_dl_switch_controls, 1),

	/*
	 * The Following three are virtual switches to select the input port
	 * before AMIC_UL enters ROUTE_UL - HAL V0.6x
	 */

	/* Virtual MM_EXT_UL Switch */
	SND_SOC_DAPM_MIXER("AMIC_UL MM_EXT",
			ABE_WIDGET(42), ABE_OPP_50, 0, &mm_ext_ul_switch_controls, 1),

	/* Virtual PDM_UL1 Switch */
	SND_SOC_DAPM_MIXER("AMIC_UL PDM",
			ABE_WIDGET(43), ABE_OPP_50, 0, &pdm_ul1_switch_controls, 1),

	/* Virtual to join MM_EXT and PDM+UL1 switches */
	SND_SOC_DAPM_MIXER("AMIC_UL", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Virtuals to join our capture sources */
	SND_SOC_DAPM_MIXER("Sidetone Capture VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Voice Capture VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DL1 Capture VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("DL2 Capture VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Join our MM_DL and MM_DL_LP playback */
	SND_SOC_DAPM_MIXER("MM_DL VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Virtual MODEM and VX_UL mixer */
	SND_SOC_DAPM_MIXER("VX UL VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("VX DL VMixer", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Virtual Pins to force backends ON atm */
	SND_SOC_DAPM_OUTPUT("BE_OUT"),
	SND_SOC_DAPM_INPUT("BE_IN"),
};

static const struct snd_soc_dapm_route intercon[] = {

	/* MUX_UL00 - ROUTE_UL - Chan 0  */
	{"MUX_UL00", "DMic0L", "DMIC0"},
	{"MUX_UL00", "DMic0R", "DMIC0"},
	{"MUX_UL00", "DMic1L", "DMIC1"},
	{"MUX_UL00", "DMic1R", "DMIC1"},
	{"MUX_UL00", "DMic2L", "DMIC2"},
	{"MUX_UL00", "DMic2R", "DMIC2"},
	{"MUX_UL00", "BT Left", "BT_VX_UL"},
	{"MUX_UL00", "BT Right", "BT_VX_UL"},
	{"MUX_UL00", "AMic0", "AMIC_UL"},
	{"MUX_UL00", "AMic1", "AMIC_UL"},
	{"MUX_UL00", "VX Left", "Capture Mixer"},
	{"MUX_UL00", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL00"},

	/* MUX_UL01 - ROUTE_UL - Chan 1  */
	{"MUX_UL01", "DMic0L", "DMIC0"},
	{"MUX_UL01", "DMic0R", "DMIC0"},
	{"MUX_UL01", "DMic1L", "DMIC1"},
	{"MUX_UL01", "DMic1R", "DMIC1"},
	{"MUX_UL01", "DMic2L", "DMIC2"},
	{"MUX_UL01", "DMic2R", "DMIC2"},
	{"MUX_UL01", "BT Left", "BT_VX_UL"},
	{"MUX_UL01", "BT Right", "BT_VX_UL"},
	{"MUX_UL01", "AMic0", "AMIC_UL"},
	{"MUX_UL01", "AMic1", "AMIC_UL"},
	{"MUX_UL01", "VX Left", "Capture Mixer"},
	{"MUX_UL01", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL01"},

	/* MUX_UL02 - ROUTE_UL - Chan 2  */
	{"MUX_UL02", "DMic0L", "DMIC0"},
	{"MUX_UL02", "DMic0R", "DMIC0"},
	{"MUX_UL02", "DMic1L", "DMIC1"},
	{"MUX_UL02", "DMic1R", "DMIC1"},
	{"MUX_UL02", "DMic2L", "DMIC2"},
	{"MUX_UL02", "DMic2R", "DMIC2"},
	{"MUX_UL02", "BT Left", "BT_VX_UL"},
	{"MUX_UL02", "BT Right", "BT_VX_UL"},
	{"MUX_UL02", "AMic0", "AMIC_UL"},
	{"MUX_UL02", "AMic1", "AMIC_UL"},
	{"MUX_UL02", "VX Left", "Capture Mixer"},
	{"MUX_UL02", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL02"},

	/* MUX_UL03 - ROUTE_UL - Chan 3  */
	{"MUX_UL03", "DMic0L", "DMIC0"},
	{"MUX_UL03", "DMic0R", "DMIC0"},
	{"MUX_UL03", "DMic1L", "DMIC1"},
	{"MUX_UL03", "DMic1R", "DMIC1"},
	{"MUX_UL03", "DMic2L", "DMIC2"},
	{"MUX_UL03", "DMic2R", "DMIC2"},
	{"MUX_UL03", "BT Left", "BT_VX_UL"},
	{"MUX_UL03", "BT Right", "BT_VX_UL"},
	{"MUX_UL03", "AMic0", "AMIC_UL"},
	{"MUX_UL03", "AMic1", "AMIC_UL"},
	{"MUX_UL03", "VX Left", "Capture Mixer"},
	{"MUX_UL03", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL03"},

	/* MUX_UL04 - ROUTE_UL - Chan 4  */
	{"MUX_UL04", "DMic0L", "DMIC0"},
	{"MUX_UL04", "DMic0R", "DMIC0"},
	{"MUX_UL04", "DMic1L", "DMIC1"},
	{"MUX_UL04", "DMic1R", "DMIC1"},
	{"MUX_UL04", "DMic2L", "DMIC2"},
	{"MUX_UL04", "DMic2R", "DMIC2"},
	{"MUX_UL04", "BT Left", "BT_VX_UL"},
	{"MUX_UL04", "BT Right", "BT_VX_UL"},
	{"MUX_UL04", "AMic0", "AMIC_UL"},
	{"MUX_UL04", "AMic1", "AMIC_UL"},
	{"MUX_UL04", "VX Left", "Capture Mixer"},
	{"MUX_UL04", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL04"},

	/* MUX_UL05 - ROUTE_UL - Chan 5  */
	{"MUX_UL05", "DMic0L", "DMIC0"},
	{"MUX_UL05", "DMic0R", "DMIC0"},
	{"MUX_UL05", "DMic1L", "DMIC1"},
	{"MUX_UL05", "DMic1R", "DMIC1"},
	{"MUX_UL05", "DMic2L", "DMIC2"},
	{"MUX_UL05", "DMic2R", "DMIC2"},
	{"MUX_UL05", "BT Left", "BT_VX_UL"},
	{"MUX_UL05", "BT Right", "BT_VX_UL"},
	{"MUX_UL05", "AMic0", "AMIC_UL"},
	{"MUX_UL05", "AMic1", "AMIC_UL"},
	{"MUX_UL05", "VX Left", "Capture Mixer"},
	{"MUX_UL05", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL05"},

	/* MUX_UL06 - ROUTE_UL - Chan 6  */
	{"MUX_UL06", "DMic0L", "DMIC0"},
	{"MUX_UL06", "DMic0R", "DMIC0"},
	{"MUX_UL06", "DMic1L", "DMIC1"},
	{"MUX_UL06", "DMic1R", "DMIC1"},
	{"MUX_UL06", "DMic2L", "DMIC2"},
	{"MUX_UL06", "DMic2R", "DMIC2"},
	{"MUX_UL06", "BT Left", "BT_VX_UL"},
	{"MUX_UL06", "BT Right", "BT_VX_UL"},
	{"MUX_UL06", "AMic0", "AMIC_UL"},
	{"MUX_UL06", "AMic1", "AMIC_UL"},
	{"MUX_UL06", "VX Left", "Capture Mixer"},
	{"MUX_UL06", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL06"},

	/* MUX_UL07 - ROUTE_UL - Chan 7  */
	{"MUX_UL07", "DMic0L", "DMIC0"},
	{"MUX_UL07", "DMic0R", "DMIC0"},
	{"MUX_UL07", "DMic1L", "DMIC1"},
	{"MUX_UL07", "DMic1R", "DMIC1"},
	{"MUX_UL07", "DMic2L", "DMIC2"},
	{"MUX_UL07", "DMic2R", "DMIC2"},
	{"MUX_UL07", "BT Left", "BT_VX_UL"},
	{"MUX_UL07", "BT Right", "BT_VX_UL"},
	{"MUX_UL07", "AMic0", "AMIC_UL"},
	{"MUX_UL07", "AMic1", "AMIC_UL"},
	{"MUX_UL07", "VX Left", "Capture Mixer"},
	{"MUX_UL07", "VX Right", "Capture Mixer"},
	{"MM_UL1", NULL, "MUX_UL07"},

	/* MUX_UL10 - ROUTE_UL - Chan 10  */
	{"MUX_UL10", "DMic0L", "DMIC0"},
	{"MUX_UL10", "DMic0R", "DMIC0"},
	{"MUX_UL10", "DMic1L", "DMIC1"},
	{"MUX_UL10", "DMic1R", "DMIC1"},
	{"MUX_UL10", "DMic2L", "DMIC2"},
	{"MUX_UL10", "DMic2R", "DMIC2"},
	{"MUX_UL10", "BT Left", "BT_VX_UL"},
	{"MUX_UL10", "BT Right", "BT_VX_UL"},
	{"MUX_UL10", "AMic0", "AMIC_UL"},
	{"MUX_UL10", "AMic1", "AMIC_UL"},
	{"MUX_UL10", "VX Left", "Capture Mixer"},
	{"MUX_UL10", "VX Right", "Capture Mixer"},
	{"MM_UL2", NULL, "MUX_UL10"},

	/* MUX_UL11 - ROUTE_UL - Chan 11  */
	{"MUX_UL11", "DMic0L", "DMIC0"},
	{"MUX_UL11", "DMic0R", "DMIC0"},
	{"MUX_UL11", "DMic1L", "DMIC1"},
	{"MUX_UL11", "DMic1R", "DMIC1"},
	{"MUX_UL11", "DMic2L", "DMIC2"},
	{"MUX_UL11", "DMic2R", "DMIC2"},
	{"MUX_UL11", "BT Left", "BT_VX_UL"},
	{"MUX_UL11", "BT Right", "BT_VX_UL"},
	{"MUX_UL11", "AMic0", "AMIC_UL"},
	{"MUX_UL11", "AMic1", "AMIC_UL"},
	{"MUX_UL11", "VX Left", "Capture Mixer"},
	{"MUX_UL11", "VX Right", "Capture Mixer"},
	{"MM_UL2", NULL, "MUX_UL11"},

	/* MUX_VX0 - ROUTE_UL - Chan 20  */
	{"MUX_VX0", "DMic0L", "DMIC0"},
	{"MUX_VX0", "DMic0R", "DMIC0"},
	{"MUX_VX0", "DMic1L", "DMIC1"},
	{"MUX_VX0", "DMic1R", "DMIC1"},
	{"MUX_VX0", "DMic2L", "DMIC2"},
	{"MUX_VX0", "DMic2R", "DMIC2"},
	{"MUX_VX0", "BT Left", "BT_VX_UL"},
	{"MUX_VX0", "BT Right", "BT_VX_UL"},
	{"MUX_VX0", "AMic0", "AMIC_UL"},
	{"MUX_VX0", "AMic1", "AMIC_UL"},
	{"MUX_VX0", "VX Left", "Capture Mixer"},
	{"MUX_VX0", "VX Right", "Capture Mixer"},

	/* MUX_VX1 - ROUTE_UL - Chan 20  */
	{"MUX_VX1", "DMic0L", "DMIC0"},
	{"MUX_VX1", "DMic0R", "DMIC0"},
	{"MUX_VX1", "DMic1L", "DMIC1"},
	{"MUX_VX1", "DMic1R", "DMIC1"},
	{"MUX_VX1", "DMic2L", "DMIC2"},
	{"MUX_VX1", "DMic2R", "DMIC2"},
	{"MUX_VX1", "BT Left", "BT_VX_UL"},
	{"MUX_VX1", "BT Right", "BT_VX_UL"},
	{"MUX_VX1", "AMic0", "AMIC_UL"},
	{"MUX_VX1", "AMic1", "AMIC_UL"},
	{"MUX_VX1", "VX Left", "Capture Mixer"},
	{"MUX_VX1", "VX Right", "Capture Mixer"},

	/* Capture Input Selection  for AMIC_UL  */
	{"AMIC_UL MM_EXT", "Switch", "MM_EXT_UL"},
	{"AMIC_UL PDM", "Switch", "PDM_UL1"},
	{"AMIC_UL", NULL, "AMIC_UL MM_EXT"},
	{"AMIC_UL", NULL, "AMIC_UL PDM"},

	/* Headset (DL1)  playback path */
	{"DL1 Mixer", "Tones", "TONES_DL"},
	{"DL1 Mixer", "Voice", "VX DL VMixer"},
	{"DL1 Mixer", "Capture", "DL1 Capture VMixer"},
	{"DL1 Capture VMixer", NULL, "MUX_UL10"},
	{"DL1 Capture VMixer", NULL, "MUX_UL11"},
	{"DL1 Mixer", "Multimedia", "MM_DL VMixer"},
	{"MM_DL VMixer", NULL, "MM_DL"},
	{"MM_DL VMixer", NULL, "MM_DL_LP"},

	/* Sidetone Mixer */
	{"Sidetone Mixer", "Playback", "DL1 Mixer"},
	{"Sidetone Mixer", "Capture", "Sidetone Capture VMixer"},
	{"Sidetone Capture VMixer", NULL, "MUX_VX0"},
	{"Sidetone Capture VMixer", NULL, "MUX_VX1"},

	/* Playback Output selection after DL1 Gain */
	{"DL1 BT_VX", "Switch", "Sidetone Mixer"},
	{"DL1 MM_EXT", "Switch", "Sidetone Mixer"},
	{"DL1 PDM", "Switch", "Sidetone Mixer"},
	{"PDM_DL1", NULL, "DL1 PDM"},
	{"BT_VX_DL", NULL, "DL1 BT_VX"},
	{"MM_EXT_DL", NULL, "DL1 MM_EXT"},

	/* Handsfree (DL2) playback path */
	{"DL2 Mixer", "Tones", "TONES_DL"},
	{"DL2 Mixer", "Voice", "VX DL VMixer"},
	{"DL2 Mixer", "Capture", "DL2 Capture VMixer"},
	{"DL2 Capture VMixer", NULL, "MUX_UL10"},
	{"DL2 Capture VMixer", NULL, "MUX_UL11"},
	{"DL2 Mixer", "Multimedia", "MM_DL VMixer"},
	{"MM_DL VMixer", NULL, "MM_DL"},
	{"MM_DL VMixer", NULL, "MM_DL_LP"},
	{"PDM_DL2", NULL, "DL2 Mixer"},

	/* VxREC Mixer */
	{"Capture Mixer", "Tones", "TONES_DL"},
	{"Capture Mixer", "Voice Playback", "VX DL VMixer"},
	{"Capture Mixer", "Voice Capture", "VX UL VMixer"},
	{"Capture Mixer", "Media Playback", "MM_DL VMixer"},
	{"MM_DL VMixer", NULL, "MM_DL"},
	{"MM_DL VMixer", NULL, "MM_DL_LP"},

	/* Audio UL mixer */
	{"Voice Capture Mixer", "Tones Playback", "TONES_DL"},
	{"Voice Capture Mixer", "Media Playback", "MM_DL VMixer"},
	{"MM_DL VMixer", NULL, "MM_DL"},
	{"MM_DL VMixer", NULL, "MM_DL_LP"},
	{"Voice Capture Mixer", "Capture", "Voice Capture VMixer"},
	{"Voice Capture VMixer", NULL, "MUX_VX0"},
	{"Voice Capture VMixer", NULL, "MUX_VX1"},

	/* BT */
	{"VX UL VMixer", NULL, "Voice Capture Mixer"},

	/* Vibra */
	{"PDM_VIB", NULL, "VIB_DL"},

	/* VX and MODEM */
	{"VX_UL", NULL, "VX UL VMixer"},
	{"MODEM_UL", NULL, "VX UL VMixer"},
	{"VX DL VMixer", NULL, "VX_DL"},
	{"VX DL VMixer", NULL, "MODEM_DL"},

	/* Backend Enablement - TODO: maybe re-work*/
	{"BE_OUT", NULL, "PDM_DL1"},
	{"BE_OUT", NULL, "PDM_DL2"},
	{"BE_OUT", NULL, "PDM_VIB"},
	{"BE_OUT", NULL, "MM_EXT_DL"},
	{"BE_OUT", NULL, "BT_VX_DL"},
	{"PDM_UL1", NULL, "BE_IN"},
	{"BT_VX_UL", NULL, "BE_IN"},
	{"MM_EXT_UL", NULL, "BE_IN"},
	{"DMIC0", NULL, "BE_IN"},
	{"DMIC1", NULL, "BE_IN"},
	{"DMIC2", NULL, "BE_IN"},
};

static int abe_add_widgets(struct snd_soc_platform *platform)
{
	snd_soc_add_platform_controls(platform, abe_controls,
				ARRAY_SIZE(abe_controls));

	snd_soc_dapm_new_controls(platform->dapm, abe_dapm_widgets,
				 ARRAY_SIZE(abe_dapm_widgets));

	snd_soc_dapm_add_routes(platform->dapm, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_new_widgets(platform->dapm);

	return 0;
}

static int aess_set_opp_mode(void)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	int i, opp = 0;

	pm_runtime_get_sync(&pdev->dev);

	mutex_lock(&abe->opp_mutex);

	/* now calculate OPP level based upon DAPM widget status */
	for (i = ABE_WIDGET_START; i < ABE_WIDGET_END; i++)
		opp |= abe->dapm[i];

	opp = (1 << (fls(opp) - 1)) * 25;

	if (abe->opp > opp) {
		/* Decrease OPP mode - no need of OPP100% */
		switch (opp) {
		case 25:
			abe_set_opp_processing(ABE_OPP25);
			udelay(250);
			omap_device_set_rate(&pdev->dev, &pdev->dev, 98304000);
			break;
		case 50:
		default:
			abe_set_opp_processing(ABE_OPP50);
			udelay(250);
			omap_device_set_rate(&pdev->dev, &pdev->dev, 98304000);
			break;
		}
	} else if (abe->opp < opp) {
		/* Increase OPP mode */
		switch (opp) {
		case 25:
			omap_device_set_rate(&pdev->dev, &pdev->dev, 98304000);
			abe_set_opp_processing(ABE_OPP25);
			break;
		case 50:
			omap_device_set_rate(&pdev->dev, &pdev->dev, 98304000);
			abe_set_opp_processing(ABE_OPP50);
			break;
		case 100:
		default:
			omap_device_set_rate(&pdev->dev, &pdev->dev, 196608000);
			abe_set_opp_processing(ABE_OPP100);
			break;
		}
	}
	abe->opp = opp;

	mutex_unlock(&abe->opp_mutex);

	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static int abe_dbg_get_dma_pos(struct abe_data *abe)
{
	return omap_get_dma_dst_pos(abe->dma_ch) - abe->dbg_buffer_addr;
}

static void abe_dbg_dma_irq(int ch, u16 stat, void *data)
{

}

static int abe_dbg_start_dma(struct abe_data *abe, int circular)
{
	struct omap_dma_channel_params dma_params;
	struct platform_device *pdev = abe->pdev;
	int err;

	/* TODO: start the DMA in either :-
	 *
	 * 1) circular buffer mode where the DMA will restart when it get to
	 *    the end of the buffer.
	 * 2) default mode, where DMA stops at the end of the buffer.
	 */
	abe->dma_req = OMAP44XX_DMA_ABE_REQ_7;
	err = omap_request_dma(abe->dma_req, "ABE debug",
			       abe_dbg_dma_irq, abe, &abe->dma_ch);
	if (abe->dbg_circular) {
		/*
		 * Link channel with itself so DMA doesn't need any
		 * reprogramming while looping the buffer
		 */
		omap_dma_link_lch(abe->dma_ch, abe->dma_ch);
	}

	memset(&dma_params, 0, sizeof(dma_params));
	dma_params.data_type		= OMAP_DMA_DATA_TYPE_S32;
	dma_params.trigger		= abe->dma_req;
	dma_params.sync_mode		= OMAP_DMA_SYNC_FRAME;
	dma_params.src_amode		= OMAP_DMA_AMODE_DOUBLE_IDX;
	dma_params.dst_amode		= OMAP_DMA_AMODE_POST_INC;
	dma_params.src_or_dst_synch	= OMAP_DMA_SRC_SYNC;
	dma_params.src_start		= D_DEBUG_FIFO_ADDR +
					  ABE_DMEM_BASE_ADDRESS_L3;
	dma_params.dst_start		= abe->dbg_buffer_addr;
	dma_params.src_port		= OMAP_DMA_PORT_MPUI;
	dma_params.src_ei		= 1;
	dma_params.src_fi		= 1 - abe->dbg_elem_bytes;

	/* 128 bytes shifted into words */
	dma_params.elem_count	= abe->dbg_elem_bytes >> 2;
	dma_params.frame_count	= abe->dbg_buffer_bytes / abe->dbg_elem_bytes;
	omap_set_dma_params(abe->dma_ch, &dma_params);

	omap_enable_dma_irq(abe->dma_ch, OMAP_DMA_FRAME_IRQ);
	omap_set_dma_src_burst_mode(abe->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(abe->dma_ch, OMAP_DMA_DATA_BURST_16);

	abe->dbg_reader_offset = 0;

	pm_runtime_get_sync(&pdev->dev);
	omap_start_dma(abe->dma_ch);
	return 0;
}

static void abe_dbg_stop_dma(struct abe_data *abe)
{
	struct platform_device *pdev = abe->pdev;

	while (omap_get_dma_active_status(abe->dma_ch))
		omap_stop_dma(abe->dma_ch);

	if (abe->dbg_circular)
		omap_dma_unlink_lch(abe->dma_ch, abe->dma_ch);
	omap_free_dma(abe->dma_ch);
	pm_runtime_put_sync(&pdev->dev);
}

static int abe_open_data(struct inode *inode, struct file *file)
{
	struct abe_data *abe = inode->i_private;

	abe->dbg_elem_bytes = 128; /* size of debug data per tick */

	if (abe->dbg_format1)
		abe->dbg_elem_bytes += ABE_DBG_FLAG1_SIZE;
	if (abe->dbg_format2)
		abe->dbg_elem_bytes += ABE_DBG_FLAG2_SIZE;
	if (abe->dbg_format3)
		abe->dbg_elem_bytes += ABE_DBG_FLAG3_SIZE;

	abe->dbg_buffer_bytes = abe->dbg_elem_bytes * 4 *
				abe->dbg_buffer_msecs;

	abe->dbg_buffer = dma_alloc_writecombine(&abe->pdev->dev,
				abe->dbg_buffer_bytes, &abe->dbg_buffer_addr,
				GFP_KERNEL);
	if (abe->dbg_buffer == NULL)
		return -ENOMEM;

	file->private_data = inode->i_private;
	abe->dbg_complete = 0;
	abe_dbg_start_dma(abe, abe->dbg_circular);

	return 0;
}

static int abe_release_data(struct inode *inode, struct file *file)
{
	struct abe_data *abe = inode->i_private;

	abe_dbg_stop_dma(abe);

	dma_free_writecombine(&abe->pdev->dev, abe->dbg_buffer_bytes,
				      abe->dbg_buffer, abe->dbg_buffer_addr);
	return 0;
}

static ssize_t abe_copy_to_user(struct abe_data *abe, char __user *user_buf,
			       size_t count)
{
	/* check for reader buffer wrap */
	if (abe->dbg_reader_offset + count > abe->dbg_buffer_bytes) {
		int size = abe->dbg_buffer_bytes - abe->dbg_reader_offset;

		/* wrap */
		if (copy_to_user(user_buf,
			abe->dbg_buffer + abe->dbg_reader_offset, size))
			return -EFAULT;

		/* need to just return if non circular */
		if (!abe->dbg_circular) {
			abe->dbg_complete = 1;
			return count;
		}

		if (copy_to_user(user_buf,
			abe->dbg_buffer, count - size))
			return -EFAULT;
		abe->dbg_reader_offset = count - size;
		return count;
	} else {
		/* no wrap */
		if (copy_to_user(user_buf,
			abe->dbg_buffer + abe->dbg_reader_offset, count))
			return -EFAULT;
		abe->dbg_reader_offset += count;

		if (!abe->dbg_circular &&
				abe->dbg_reader_offset == abe->dbg_buffer_bytes)
			abe->dbg_complete = 1;

		return count;
	}
}

static ssize_t abe_read_data(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	struct abe_data *abe = file->private_data;
	DECLARE_WAITQUEUE(wait, current);
	int dma_offset, bytes;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	add_wait_queue(&abe->wait, &wait);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		/* TODO: Check if really needed. Or adjust sleep delay
		 * If not delay trace is not working */
		msleep_interruptible(1);
		dma_offset = abe_dbg_get_dma_pos(abe);

		/* is DMA finished ? */
		if (abe->dbg_complete)
			break;

		/* get maximum amount of debug bytes we can read */
		if (dma_offset >= abe->dbg_reader_offset) {
			/* dma ptr is ahead of reader */
			bytes = dma_offset - abe->dbg_reader_offset;
		} else {
			/* dma ptr is behind reader */
			bytes = dma_offset + abe->dbg_buffer_bytes -
				abe->dbg_reader_offset;
		}

		if (count > bytes)
			count = bytes;

		if (count > 0) {
			ret = abe_copy_to_user(abe, user_buf, count);
			if (count < bytes)
				dev_dbg(&pdev->dev, "ABE Trace Max:%x Data:%x\n", count, bytes);
			break;
		}

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		schedule();

	} while (1);

	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&abe->wait, &wait);

	return ret;
}

static const struct file_operations abe_data_fops = {
	.open = abe_open_data,
	.read = abe_read_data,
	.release = abe_release_data,
};

static void abe_init_debugfs(struct abe_data *abe)
{
	struct platform_device *pdev = abe->pdev;

	
	abe->debugfs_root = debugfs_create_dir("omap4-abe", NULL);
	if (!abe->debugfs_root) {
		dev_warn(&pdev->dev, "failed to create debugfs directory\n");
		return;
	}

	abe->debugfs_fmt1 = debugfs_create_bool("format1", 0644,
						 abe->debugfs_root,
						 &abe->dbg_format1);
	if (!abe->debugfs_fmt1)
		dev_warn(&pdev->dev, "failed to create format1 debugfs file\n");

	abe->debugfs_fmt2 = debugfs_create_bool("format2", 0644,
						 abe->debugfs_root,
						 &abe->dbg_format2);
	if (!abe->debugfs_fmt2)
		dev_warn(&pdev->dev, "failed to create format2 debugfs file\n");

	abe->debugfs_fmt3 = debugfs_create_bool("format3", 0644,
						 abe->debugfs_root,
						 &abe->dbg_format3);
	if (!abe->debugfs_fmt3)
		dev_warn(&pdev->dev, "failed to create format3 debugfs file\n");

	abe->debugfs_elem_bytes = debugfs_create_u32("element_bytes", 0604,
						 abe->debugfs_root,
						 &abe->dbg_elem_bytes);
	if (!abe->debugfs_elem_bytes)
		dev_warn(&pdev->dev, "failed to create element size debugfs file\n");

	abe->debugfs_size = debugfs_create_u32("msecs", 0644,
						 abe->debugfs_root,
						 &abe->dbg_buffer_msecs);
	if (!abe->debugfs_size)
		dev_warn(&pdev->dev, "failed to create buffer size debugfs file\n");

	abe->debugfs_circ = debugfs_create_bool("circular", 0644,
						 abe->debugfs_root,
						 &abe->dbg_circular);
	if (!abe->debugfs_size)
		dev_warn(&pdev->dev, "failed to create circular mode debugfs file\n");

	abe->debugfs_data = debugfs_create_file("debug", 0644,
						 abe->debugfs_root,
						 abe, &abe_data_fops);
	if (!abe->debugfs_data)
		dev_warn(&pdev->dev, "failed to create data debugfs file\n");

	abe->dbg_buffer_msecs = 500;
	init_waitqueue_head(&abe->wait);
}

static void abe_cleanup_debugfs(struct abe_data *abe)
{
	debugfs_remove_recursive(abe->debugfs_root);
}

#else

static inline void abe_init_debugfs(struct abe_data *abe)
{
}

static inline void abe_cleanup_debugfs(struct abe_data *abe)
{
}
#endif

static int abe_probe(struct snd_soc_platform *platform)
{
	abe_init_engine(platform);
	abe_add_widgets(platform);
	abe->platform = platform;
	return 0;
}

static int  abe_remove(struct snd_soc_platform *platform)
{
	return 0;
}

static int aess_save_context(struct abe_data *abe)
{
	struct platform_device *pdev = abe->pdev;
	struct omap4_abe_dsp_pdata *pdata = pdev->dev.platform_data;

	/* TODO: Find a better way to save/retore gains after OFF mode */
	abe_mute_gain(MIXSDT, MIX_SDT_INPUT_UP_MIXER);
	abe_mute_gain(MIXSDT, MIX_SDT_INPUT_DL1_MIXER);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_MM_DL);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_TONES);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_UPLINK);
	abe_mute_gain(MIXAUDUL, MIX_AUDUL_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_TONES);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_MM_DL);
	abe_mute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_UL);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_MM_DL);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_MM_UL2);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_VX_DL);
	abe_mute_gain(MIXDL1, MIX_DL1_INPUT_TONES);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_TONES);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_VX_DL);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_MM_DL);
	abe_mute_gain(MIXDL2, MIX_DL2_INPUT_MM_UL2);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL1);
	abe_mute_gain(MIXECHO, MIX_ECHO_DL2);
	abe_mute_gain(GAINS_DMIC1, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DMIC2, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_DMIC3, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_DMIC3, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_AMIC, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_AMIC, GAIN_RIGHT_OFFSET);
	abe_mute_gain(GAINS_BTUL, GAIN_LEFT_OFFSET);
	abe_mute_gain(GAINS_BTUL, GAIN_RIGHT_OFFSET);

	if (pdata->get_context_loss_count)
		abe->loss_count = pdata->get_context_loss_count(&pdev->dev);

	return 0;
}

static int aess_restore_context(struct abe_data *abe)
{
	struct platform_device *pdev = abe->pdev;
	struct omap4_abe_dsp_pdata *pdata = pdev->dev.platform_data;
	int loss_count = 0;

	omap_device_set_rate(&pdev->dev, &pdev->dev, 98304000);

	if (pdata->get_context_loss_count)
		loss_count = pdata->get_context_loss_count(&pdev->dev);

	if  (loss_count != abe->loss_count)
		abe_reload_fw();

	/* TODO: Find a better way to save/retore gains after dor OFF mode */
	abe_unmute_gain(MIXSDT, MIX_SDT_INPUT_UP_MIXER);
	abe_unmute_gain(MIXSDT, MIX_SDT_INPUT_DL1_MIXER);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_MM_DL);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_TONES);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_UPLINK);
	abe_unmute_gain(MIXAUDUL, MIX_AUDUL_INPUT_VX_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_TONES);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_MM_DL);
	abe_unmute_gain(MIXVXREC, MIX_VXREC_INPUT_VX_UL);
	abe_unmute_gain(MIXDL1, MIX_DL1_INPUT_MM_DL);
	abe_unmute_gain(MIXDL1, MIX_DL1_INPUT_MM_UL2);
	abe_unmute_gain(MIXDL1, MIX_DL1_INPUT_VX_DL);
	abe_unmute_gain(MIXDL1, MIX_DL1_INPUT_TONES);
	abe_unmute_gain(MIXDL2, MIX_DL2_INPUT_TONES);
	abe_unmute_gain(MIXDL2, MIX_DL2_INPUT_VX_DL);
	abe_unmute_gain(MIXDL2, MIX_DL2_INPUT_MM_DL);
	abe_unmute_gain(MIXDL2, MIX_DL2_INPUT_MM_UL2);
	abe_unmute_gain(MIXECHO, MIX_ECHO_DL1);
	abe_unmute_gain(MIXECHO, MIX_ECHO_DL2);
	abe_unmute_gain(GAINS_DMIC1, GAIN_LEFT_OFFSET);
	abe_unmute_gain(GAINS_DMIC1, GAIN_RIGHT_OFFSET);
	abe_unmute_gain(GAINS_DMIC2, GAIN_LEFT_OFFSET);
	abe_unmute_gain(GAINS_DMIC2, GAIN_RIGHT_OFFSET);
	abe_unmute_gain(GAINS_DMIC3, GAIN_LEFT_OFFSET);
	abe_unmute_gain(GAINS_DMIC3, GAIN_RIGHT_OFFSET);
	abe_unmute_gain(GAINS_AMIC, GAIN_LEFT_OFFSET);
	abe_unmute_gain(GAINS_AMIC, GAIN_RIGHT_OFFSET);
	abe_unmute_gain(GAINS_BTUL, GAIN_LEFT_OFFSET);
	abe_unmute_gain(GAINS_BTUL, GAIN_RIGHT_OFFSET);

	abe_dsp_set_equalizer(EQ1, abe->dl1_equ_profile);
	abe_dsp_set_equalizer(EQ2L, abe->dl20_equ_profile);
	abe_dsp_set_equalizer(EQ2R, abe->dl21_equ_profile);
	abe_dsp_set_equalizer(EQAMIC, abe->amic_equ_profile);
	abe_dsp_set_equalizer(EQDMIC, abe->dmic_equ_profile);
	abe_dsp_set_equalizer(EQSDT, abe->sdt_equ_profile);

	abe_dsp_set_mono_mixer(MIXDL1, abe->dl1_mono);
       abe_dsp_set_mono_mixer(MIXDL2, abe->dl2_mono);

	abe_set_router_configuration(UPROUTE, 0, (u32 *)abe->router);

	return 0;
}

static int aess_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	mutex_lock(&abe->mutex);

	abe->fe_id = dai->id;
	dev_dbg(&rtd->dev, "%s ID %d\n", __func__, dai->id);

	pm_runtime_get_sync(&pdev->dev);

	if (!abe->active++) {
		abe->opp = 0;
		aess_restore_context(abe);
		aess_set_opp_mode();
		abe_wakeup();
	}

	switch (dai->id) {
	case ABE_FRONTEND_DAI_MODEM:
		break;
	case ABE_FRONTEND_DAI_LP_MEDIA:
		snd_soc_set_runtime_hwparams(substream, &omap_abe_hardware);
		#if 0 /* ASoC: ABE DSP: remove buffer size constraint on MM_LP BE */
		ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 1024);
		#endif
		break;
	default:
		break;
	}

	mutex_unlock(&abe->mutex);
	return 0;
}

static int abe_ping_pong_init(struct snd_pcm_hw_params *params,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	abe_data_format_t format;
	size_t period_size;
	u32 dst;

	/*Storing substream pointer for irq*/
	abe->psubs = substream;

	format.f = params_rate(params);
	format.samp_format = STEREO_16_16;

	if (format.f == 44100)
		abe_write_event_generator(EVENT_44100);

	period_size = params_period_bytes(params);

	/*Adding ping pong buffer subroutine*/
	abe_add_subroutine(&abe_irq_pingpong_player_id,
				(abe_subroutine2) abe_irq_pingpong_subroutine,
				SUB_0_PARAM, (u32 *)0);

	/* Connect a Ping-Pong cache-flush protocol to MM_DL port */
	abe_connect_irq_ping_pong_port(MM_DL_PORT, &format,
				abe_irq_pingpong_player_id,
				period_size, &dst,
				PING_PONG_WITH_MCU_IRQ);

	/* Memory mapping for hw params */
	runtime->dma_area  = abe->io_base + ABE_DMEM_BASE_OFFSET_MPU + dst;
	runtime->dma_addr  = 0;
#ifdef KEYTONE_MUTE_GLITCH
#ifdef PING_PONG_4
	runtime->dma_bytes = period_size * 4;
#else
	runtime->dma_bytes = period_size * 8;
#endif
#else	
	runtime->dma_bytes = period_size * 2;
#endif

	/* Need to set the first buffer in order to get interrupt */
	abe_set_ping_pong_buffer(MM_DL_PORT, period_size);
	abe->first_irq = 1;

	return 0;
}

static int aess_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	int ret = 0;

	dev_dbg(&rtd->dev, "%s ID %d\n", __func__, dai->id);

	switch (dai->id) {
	case ABE_FRONTEND_DAI_MODEM:
		break;
	case ABE_FRONTEND_DAI_LP_MEDIA:
		ret = abe_ping_pong_init(params, substream);
		if (ret < 0)
			return ret;
		break;
	default:
		break;
	}

	return 0;
}

static int aess_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime  *rtd = substream->private_data;


	mutex_lock(&abe->mutex);

	dev_dbg(&rtd->dev, "%s ID %d\n", __func__, rtd->cpu_dai->id);

	aess_set_opp_mode();

	mutex_unlock(&abe->mutex);
	return 0;
}

static int aess_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime  *rtd = substream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;

	mutex_lock(&abe->mutex);

	abe->fe_id = dai->id;
	dev_dbg(&rtd->dev, "%s ID %d\n", __func__, dai->id);

	if (!--abe->active) {
		abe_disable_irq();
		aess_save_context(abe);
		abe_dsp_shutdown();
	}
	pm_runtime_put_sync(&pdev->dev);

	mutex_unlock(&abe->mutex);
	return 0;
}

static int aess_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *dai = rtd->cpu_dai;
	int offset, size, err = 0;

	switch (dai->id) {
	case ABE_FRONTEND_DAI_LP_MEDIA:
		/* TODO: we may need to check for underrun. */
		vma->vm_flags |= VM_IO | VM_RESERVED;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		size = vma->vm_end - vma->vm_start;
		offset = vma->vm_pgoff << PAGE_SHIFT;

		err = io_remap_pfn_range(vma, vma->vm_start,
				(ABE_DMEM_BASE_ADDRESS_MPU +
				ABE_DMEM_BASE_OFFSET_PING_PONG +
				offset) >> PAGE_SHIFT,
				size, vma->vm_page_prot);
		break;

	default:
		break;
	}

	if (err)
		return -EAGAIN;

	return 0;
}

static snd_pcm_uframes_t aess_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t offset;
	u32 pingpong;

	abe_read_offset_from_ping_buffer(MM_DL_PORT, &pingpong);
	offset = (snd_pcm_uframes_t)pingpong;
/*
	if (offset >= runtime->buffer_size)
		offset = 0;
*/
	return offset;
}


static struct snd_pcm_ops omap_aess_pcm_ops = {
	.open           = aess_open,
	.hw_params	= aess_hw_params,
	.prepare	= aess_prepare,
	.close	        = aess_close,
	.pointer	= aess_pointer,
	.mmap		= aess_mmap,
};

static int aess_stream_event(struct snd_soc_dapm_context *dapm, int event)
{
	/* TODO: do not use abe global structure to assign pdev */
	struct platform_device *pdev = abe->pdev;
	int active = abe_fe_active_count(abe);
	int ret = 0;

	if (!abe->active)
		return 0;

	pm_runtime_get_sync(&pdev->dev);
	aess_set_opp_mode();
	pm_runtime_put_sync(&pdev->dev);

	switch (event) {
	case SND_SOC_DAPM_STREAM_START:
		/*
		 * enter dpll cascading when all conditions are met:
		 * - system is in early suspend (screen is off)
		 * - single stream is active and is LP (ping-pong)
		 * - OPP is 50 or less (DL1 path only)
		 */
		if (abe->early_suspended && (active == 1) &&
		    abe->fe_active[6] && (abe->opp <= 50))
			ret = dpll_cascading_blocker_release(&pdev->dev);
		else
			ret = dpll_cascading_blocker_hold(&pdev->dev);
		break;
	case SND_SOC_DAPM_STREAM_STOP:
		ret = dpll_cascading_blocker_hold(&pdev->dev);
		break;
	default:
		break;
	}

	return ret;
}

static struct snd_soc_platform_driver omap_aess_platform = {
	.ops	= &omap_aess_pcm_ops,
	.probe = abe_probe,
	.remove = abe_remove,
	.read = abe_dsp_read,
	.write = abe_dsp_write,
	.stream_event = aess_stream_event,
};

#if defined(CONFIG_PM)
static int aess_suspend(struct device *dev)
{
	/*
	 * ensure we're out of DPLL cascading to properly
	 * enter into suspend state
	 */
	return dpll_cascading_blocker_hold(dev);
}

static int aess_resume(struct device *dev)
{
	/* block DPLL cascading till conditions are met */
	return dpll_cascading_blocker_hold(dev);
}

static int omap_pm_abe_get_dev_context_loss_count(struct device *dev)
{
	int ret;

	ret = prm_read_mod_reg(abe_pwrdm->prcm_offs,
			abe_pwrdm->context_offset);

	if ((ret & 0x0001) == 0x0001) {
		prm_write_mod_reg(0x0001, abe_pwrdm->prcm_offs,
			abe_pwrdm->context_offset);
		ret &= ~0x0001;
	}

	if ((ret & 0x0100) == 0x0100)
		prm_write_mod_reg(0x0100, abe_pwrdm->prcm_offs,
			abe_pwrdm->context_offset);

	return ret;
}

#else
#define aess_runtime_suspend   NULL
#define aess_runtime_resume    NULL
#define omap_pm_abe_get_dev_context_loss_count NULL
#endif

static const struct dev_pm_ops aess_pm_ops = {
	.suspend        = aess_suspend,
	.resume         = aess_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void abe_early_suspend(struct early_suspend *handler)
{
	struct abe_data *abe = container_of(handler, struct abe_data,
					    early_suspend);
	int active = abe_fe_active_count(abe);

	/*
	 * enter dpll cascading when all conditions are met:
	 * - system is in early suspend (screen is off)
	 * - single stream is active and is LP (ping-pong)
	 * - OPP is 50 or less (DL1 path only)
	 */
	if ((active == 1) && abe->fe_active[6] && (abe->opp <= 50))
		dpll_cascading_blocker_release(&abe->pdev->dev);

	abe->early_suspended = 1;
}

static void abe_late_resume(struct early_suspend *handler)
{
	struct abe_data *abe = container_of(handler, struct abe_data,
					    early_suspend);

	/* exit dpll cascading since screen will be turned on */
	dpll_cascading_blocker_hold(&abe->pdev->dev);

	abe->early_suspended = 0;
}
#endif

static int __devinit abe_engine_probe(struct platform_device *pdev)
{
	struct omap_hwmod *oh;
	struct omap4_abe_dsp_pdata *pdata = pdev->dev.platform_data;
	int ret = -EINVAL, i;

	oh = omap_hwmod_lookup("omap-aess-audio");
	if (oh == NULL) {
		dev_err(&pdev->dev, "no hwmod device found\n");
		return -ENODEV;
	}

	abe = kzalloc(sizeof(struct abe_data), GFP_KERNEL);
	if (abe == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, abe);

	/* ZERO_labelID should really be 0 */
	for (i = 0; i < ABE_ROUTES_UL + 2; i++)
		abe->router[i] = ZERO_labelID;

	abe->io_base = omap_hwmod_get_mpu_rt_va(oh);
	if (!abe->io_base) {
		ret = -ENOMEM;
		goto err;
	}

	abe->irq = platform_get_irq(pdev, 0);
	if (abe->irq < 0) {
		ret = abe->irq;
		goto err;
	}

#if defined(CONFIG_PM)
	abe_pwrdm = pwrdm_lookup("abe_pwrdm");
	if (!abe_pwrdm)
		return -ENODEV;

	pdata->get_context_loss_count = omap_pm_abe_get_dev_context_loss_count;
#endif

	pm_runtime_enable(&pdev->dev);

	abe->abe_pdata = pdata;
	abe->pdev = pdev;
	dpll_cascading_blocker_hold(&abe->pdev->dev);
	abe_can_enter_dpll_cascading = true;

	mutex_init(&abe->mutex);
	mutex_init(&abe->opp_mutex);

#ifdef CONFIG_HAS_EARLYSUSPEND
		abe->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
		abe->early_suspend.suspend = abe_early_suspend;
		abe->early_suspend.resume = abe_late_resume;
		register_early_suspend(&abe->early_suspend);
#endif

	ret = snd_soc_register_platform(&pdev->dev,
			&omap_aess_platform);
	if (ret)
		goto err;

	abe_init_debugfs(abe);

	return 0;

err:
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&abe->early_suspend);
#endif
	kfree(abe);
	return ret;
}

static int __devexit abe_engine_remove(struct platform_device *pdev)
{
	struct abe_data *priv = dev_get_drvdata(&pdev->dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&abe->early_suspend);
#endif
	abe_cleanup_debugfs(abe);
	snd_soc_unregister_platform(&pdev->dev);
	kfree(priv);
	return 0;
}

static struct platform_driver omap_aess_driver = {
	.driver = {
		.name = "omap-aess-audio",
		.owner = THIS_MODULE,
		.pm = &aess_pm_ops,
	},
	.probe = abe_engine_probe,
	.remove = __devexit_p(abe_engine_remove),
};

static int __init abe_engine_init(void)
{
	return platform_driver_register(&omap_aess_driver);
}
module_init(abe_engine_init);

static void __exit abe_engine_exit(void)
{
	platform_driver_unregister(&omap_aess_driver);
}
module_exit(abe_engine_exit);

MODULE_DESCRIPTION("ASoC OMAP4 ABE");
MODULE_AUTHOR("Liam Girdwood <lrg@slimlogic.co.uk>");
MODULE_LICENSE("GPL");
