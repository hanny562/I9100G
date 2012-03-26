/*
 * hdcp_top.c
 *
 * HDCP support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Fabrice Olivero <f-olivero@ti.com>
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <plat/display.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <../drivers/video/omap2/dss/hdmi.h>
#include <../drivers/video/omap2/dss/dss.h>
#include <plat/hdmi_lib.h>

#include "hdcp.h"

#define HDCP_DBG_MSG 0

#define DBG_ERR(format, ...) \
		if (ddc_timeout) printk(KERN_INFO " " format, ## __VA_ARGS__)
#define DBG_HDCP_MSG(format, ...) \
		if (HDCP_DBG_MSG) printk(KERN_INFO " " format, ## __VA_ARGS__)

struct hdcp hdcp;

/* State machine / workqueue */
static void hdcp_wq_disable(void);
static void hdcp_wq_start_authentication(void);
static void hdcp_wq_restart_hdmi(void);
static void hdcp_wq_check_r0(void);
static void hdcp_wq_step2_authentication(void);
static void hdcp_wq_authentication_failure(void);
static void hdcp_work_queue(struct work_struct *work);
static struct delayed_work *hdcp_submit_work(int event, int delay, int irq_context);
static void hdcp_cancel_work(struct delayed_work **work);

/* Callbacks */
static void hdcp_start_frame_cb(void);
static void hdcp_stop_frame_cb(void);
static void hdcp_irq_cb(int hpd_low);

/* Control */
static long hdcp_enable_ctl(void __user *argp);
static long hdcp_disable_ctl(void);
static long hdcp_query_status_ctl(void __user *argp);
static long hdcp_encrypt_key_ctl(void __user *argp);
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg);

/* Driver */
static int __init hdcp_init(void);
static void __exit hdcp_exit(void);

struct completion hdcp_comp;
bool ri_check_fail;
#define DSS_POWER

/*-----------------------------------------------------------------------------
 * Function: hdcp_request_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_request_dss(void)
{
#ifdef DSS_POWER
	hdcp.dss_state = dss_mainclk_enable();
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_release_dss
 *-----------------------------------------------------------------------------
 */
static void hdcp_release_dss(void)
{
#ifdef DSS_POWER
	if (hdcp.dss_state == 0)
		dss_mainclk_disable();
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_disable
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_disable(void)
{
	DBG_HDCP_MSG( "HDCP: disabled\n");
	hdcp_cancel_work(&hdcp.pending_wq_event);
	hdcp_lib_disable();
	hdcp.pending_disable = 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_start_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_start_authentication(void)
{
	int status = HDCP_OK;

	hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
	DBG_HDCP_MSG( "HDCP: authentication start\n");

	/* Load 3DES key */
	if (hdcp_3des_load_key(hdcp.en_ctrl->key) != HDCP_OK) {
		hdcp_wq_authentication_failure();
		return;
	}

	/* Step 1 part 1 (until R0 calc delay) */
	status = hdcp_lib_step1_start();

	if (status == -HDCP_AKSV_ERROR) {
		hdcp_wq_restart_hdmi();
	}
	else if (status == -HDCP_CANCELLED_AUTH) {
		DBG_HDCP_MSG("Authentication step 1 cancelled.\n");
		return;
	}
	else if (status != HDCP_OK) {
		hdcp_wq_authentication_failure();
	}
	else {
		hdcp.hdcp_state = HDCP_WAIT_R0_DELAY;
		hdcp.auth_state = HDCP_STATE_AUTH_1ST_STEP;
		hdcp.pending_wq_event = hdcp_submit_work(HDCP_R0_EXP_EVENT,
							 HDCP_R0_DELAY, 0);
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_restart_hdmi
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_restart_hdmi(void)
{
	hdcp_cancel_work(&hdcp.pending_wq_event);
	hdcp_lib_disable();
	hdcp.pending_disable = 0;

	if (hdcp.retry_cnt) {
		if (hdcp.retry_cnt < HDCP_INFINITE_REAUTH) {
			hdcp.retry_cnt--;
			DBG_HDCP_MSG("HDCP: authentication failed - restarting HDMI, attempts=%d\n", hdcp.retry_cnt);
		}
		else
			DBG_HDCP_MSG("HDCP: authentication failed - restarting HDMI\n");

		hdcp.hdmi_restart = 1;
		hdmi_restart();
		hdcp.hdmi_restart = 0;
		hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;
	}
	else {
		DBG_HDCP_MSG("HDCP: authentication failed - HDCP disabled\n");
		hdcp.hdcp_state = HDCP_DISABLED;
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_check_r0
 *-----------------------------------------------------------------------------
 */
extern  struct omap_dss_device *get_hdmi_device(void);
extern void hdmi_notify_status(struct omap_dss_device *dssdev, bool onoff);

static void hdcp_wq_check_r0(void)
{
	int status = hdcp_lib_step1_r0_check();
	struct omap_dss_device *dssdev = get_hdmi_device();

	if (status == -HDCP_CANCELLED_AUTH) {
		DBG_HDCP_MSG("Authentication step 1/R0 cancelled.\n");
		return;
	}
	else if (status < 0)
		hdcp_wq_authentication_failure();
	else
		if (hdcp_lib_check_repeater_bit_in_tx()) {
			/* Repeater */
			DBG_HDCP_MSG("HDCP: authentication step 1 successful - "
					  "Repeater\n");

			hdcp.hdcp_state = HDCP_WAIT_KSV_LIST;
			hdcp.auth_state = HDCP_STATE_AUTH_2ND_STEP;

			hdcp.pending_wq_event = hdcp_submit_work(HDCP_KSV_TIMEOUT_EVENT,
								 HDCP_KSV_TIMEOUT_DELAY,
								 0);

		}
		else {
			/* Receiver */
			DBG_HDCP_MSG("HDCP: authentication step 1 successful - "
					 "Receiver\n");

			hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
			hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;

			/* Restore retry counter */
			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;

			ri_check_fail = false;
		}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_step2_authentication
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_step2_authentication(void)
{
	int status = HDCP_OK;

	/* KSV list timeout is running and should be canceled */
	hdcp_cancel_work(&hdcp.pending_wq_event);

	status = hdcp_lib_step2();

	if (status == -HDCP_CANCELLED_AUTH) {
		DBG_HDCP_MSG("Authentication step 2 cancelled.\n");
		return;
	}
	else if (status < 0)
		hdcp_wq_authentication_failure();
	else {
		DBG_HDCP_MSG("HDCP: (Repeater) authentication step 2 "
				 "successful\n");

		hdcp.hdcp_state = HDCP_LINK_INTEGRITY_CHECK;
		hdcp.auth_state = HDCP_STATE_AUTH_3RD_STEP;

		/* Restore retry counter */
		if (hdcp.en_ctrl->nb_retry == 0)
			hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
		else
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_wq_authentication_failure
 *-----------------------------------------------------------------------------
 */
static void hdcp_wq_authentication_failure(void)
{
	if (hdcp.ddc_err_cnt >= HDCP_MAX_DDC_ERR) {
		DBG_HDCP_MSG("HDCP: too many DDC errors, attempts=%d\n", HDCP_MAX_DDC_ERR);
		hdcp.ddc_err_cnt = 0;
		hdcp_wq_restart_hdmi();
		return;
	}

	hdcp_lib_auto_ri_check(false);
	hdcp_lib_auto_bcaps_rdy_check(false);
	hdcp_lib_set_av_mute(AV_MUTE_SET);
	hdcp_lib_set_encryption(HDCP_ENC_OFF);

	hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp_lib_disable();
	hdcp.pending_disable = 0;

	if (hdcp.retry_cnt) {
		if (hdcp.retry_cnt < HDCP_INFINITE_REAUTH) {
			hdcp.retry_cnt--;
			DBG_HDCP_MSG("HDCP: authentication failed - retrying, attempts=%d\n", hdcp.retry_cnt);
		}
		else
			DBG_HDCP_MSG("HDCP: authentication failed - retrying\n");

		hdcp.hdcp_state = HDCP_AUTHENTICATION_START;
		hdcp.auth_state = HDCP_STATE_AUTH_FAIL_RESTARTING;

		hdcp.pending_wq_event = hdcp_submit_work(HDCP_AUTH_REATT_EVENT,
							 HDCP_REAUTH_DELAY, 0);
	}
	else {
		DBG_HDCP_MSG("HDCP: authentication failed - HDCP disabled\n");
		hdcp.hdcp_state = HDCP_DISABLED;
		hdcp.auth_state = HDCP_STATE_AUTH_FAILURE;
	}

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_work_queue
 *-----------------------------------------------------------------------------
 */
static void hdcp_work_queue(struct work_struct *work)
{
	struct hdcp_delayed_work *hdcp_w =
		container_of(work, struct hdcp_delayed_work, work.work);
	int event = hdcp_w->event;

	mutex_lock(&hdcp.lock);

	DBG_HDCP_MSG("hdcp_work_queue() - START - %u hdmi=%d hdcp=%d auth=%d evt= %x %d hdcp_ctrl=%02x\n",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF,
		RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__HDCP_CTRL));
	/* HPD_LOW: cancel_delayed_work can't be called from IRQ context
	 * so handling it here */
	if (hdcp.hpd_low) {
		if ((struct delayed_work *)hdcp_w != hdcp.pending_start)
			hdcp_cancel_work(&hdcp.pending_start);
		if ((struct delayed_work *)hdcp_w != hdcp.pending_wq_event)
			hdcp_cancel_work(&hdcp.pending_wq_event);
	}

	/* Clear pending_wq_event
	 * In case a delayed work is scheduled from the state machine "pending_wq_event"
	 * is used to memorize pointer on the event to be able to cancel any pending work in
	 * case HDCP is disabled
	 */
	if (event & HDCP_WORKQUEUE_SRC)
		hdcp.pending_wq_event = 0;

	/* First handle HDMI state */
	if (event == HDCP_START_FRAME_EVENT) {
		hdcp.pending_start = 0;
		hdcp.hdmi_state = HDMI_STARTED;
	}
	else if (event == HDCP_STOP_FRAME_EVENT)
		hdcp.hdmi_state = HDMI_STOPPED;

	/**********************/
	/* HDCP state machine */
	/**********************/


	/* Handle HDCP disable (from any state) */
	if ((event == HDCP_DISABLE_CTL) ||
	    (event == HDCP_STOP_FRAME_EVENT) ||
	    (event == HDCP_HPD_LOW_EVENT)) {
		DBG_HDCP_MSG("before disabling hdcp authentication: auth_state: %d, hdcp_state: %d, event: %d\n",
		       hdcp.auth_state, hdcp.hdcp_state, event);
		if (hdcp.hdcp_state != HDCP_DISABLED) {
			hdcp_wq_disable();

			if (event == HDCP_DISABLE_CTL) {
				if (hdcp.en_ctrl) {
					kfree(hdcp.en_ctrl);
					hdcp.en_ctrl = 0;
				}

				hdcp.hdcp_state = HDCP_DISABLED;
			}
			else if (event == HDCP_STOP_FRAME_EVENT) {
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;

				/* In case of HDCP_STOP_FRAME_EVENT, HDCP stop
				 * frame callback is blocked and waiting for
				 * HDCP driver to finish accessing the HW
				 * before returning
				 * Reason is to avoid HDMI driver to shutdown
				 * DSS/HDMI power before HDCP work is finished
				 */
				complete(&hdcp_comp);
				DBG_HDCP_MSG("Send event to wake-up HDCP stop frame callback %u\n", jiffies_to_msecs(jiffies));
			}
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;

			hdcp.auth_state = HDCP_STATE_DISABLED;
			DBG_HDCP_MSG("after disabling hdcp authentication: auth_state: %d, hdcp_state: %d, event: %d\n",
			       hdcp.auth_state, hdcp.hdcp_state, event);
		}

		hdcp.pending_disable = 0;
	}

	if (hdcp.hpd_low) {
		hdcp.hpd_low = 0;
		if (event & HDCP_WORKQUEUE_SRC)
			goto exit_wq;
	}
	switch (hdcp.hdcp_state) {

	/* State */
	/*********/
	case HDCP_DISABLED:
		/* HDCP enable control or re-authentication event */
		if (event == HDCP_ENABLE_CTL) {
			if (hdcp.en_ctrl->nb_retry == 0)
				hdcp.retry_cnt = HDCP_INFINITE_REAUTH;
			else
				hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;

			if (hdcp.hdmi_state == HDMI_STARTED) {
				hdcp_wq_start_authentication();
			}
			else
				hdcp.hdcp_state = HDCP_ENABLE_PENDING;
		}

		break;

	/* State */
	/*********/
	case HDCP_ENABLE_PENDING:
		/* HDMI start frame event */
		if (event == HDCP_START_FRAME_EVENT) {
			hdcp_wq_start_authentication();
		}

		break;

	/* State */
	/*********/
	case HDCP_AUTHENTICATION_START:
		/* Re-authentication */
		if (event == HDCP_AUTH_REATT_EVENT) {
			hdcp_wq_start_authentication();
		}
		break;

	/* State */
	/*********/
	case HDCP_WAIT_R0_DELAY:
		/* R0 timer elapsed */
		if (event == HDCP_R0_EXP_EVENT) {
			hdcp_wq_check_r0();
		}
		break;

	/* State */
	/*********/
	case HDCP_WAIT_KSV_LIST:
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			DBG_HDCP_MSG("HDCP: Ri check failure\n");

			hdcp_wq_authentication_failure();
		}
		/* KSV list ready event */
		else if (event == HDCP_KSV_LIST_RDY_EVENT) {
			hdcp_wq_step2_authentication();
		}
		/* Timeout */
		else if (event == HDCP_KSV_TIMEOUT_EVENT) {
			DBG_HDCP_MSG("HDCP: BCAPS polling timeout\n");
			hdcp_wq_authentication_failure();
		}
		break;

	/* State */
	/*********/
	case HDCP_LINK_INTEGRITY_CHECK:
		/* Ri failure */
		if (event == HDCP_RI_FAIL_EVENT) {
			DBG_HDCP_MSG("HDCP: Ri check failure\n");
			hdcp_wq_authentication_failure();
		}
		break;

	default:
		DBG_HDCP_MSG("HDCP: error - unknow HDCP state\n");
		break;
	}

exit_wq:
	kfree(hdcp_w);

	DBG_HDCP_MSG("hdcp_work_queue() - END - %u hdmi=%d hdcp=%d auth=%d evt=%x %d hdcp_ctrl=%02x\n",
		jiffies_to_msecs(jiffies),
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		hdcp.auth_state,
		(event & 0xFF00) >> 8,
		event & 0xFF,
		RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			  HDMI_IP_CORE_SYSTEM__HDCP_CTRL));

	mutex_unlock(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_submit_work
 *-----------------------------------------------------------------------------
 */
static struct delayed_work *hdcp_submit_work(int event, int delay, int irq_context)
{
	struct hdcp_delayed_work *work;
	if (irq_context)
		work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_ATOMIC);
	else
		work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_KERNEL);

	if (work) {
		work->event = event;
		INIT_DELAYED_WORK(&work->work, hdcp_work_queue);
		schedule_delayed_work(&work->work, msecs_to_jiffies(delay));
	}
	else {
		DBG_HDCP_MSG("HDCP: Cannot allocate memory to create work\n");
		return 0;
	}

	return &work->work;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_cancel_work
 *-----------------------------------------------------------------------------
 */
static void hdcp_cancel_work(struct delayed_work **work)
{
	int ret = 0;
	DBG_HDCP_MSG("hdcp_cancel_work()\n");

	if (*work) {
/*                 cancel_delayed_work(*work); */
/*                 kfree(*work);               */
/*                 *work = 0;                  */
		ret = cancel_delayed_work(*work);
		if(ret != 1) {
			ret = cancel_work_sync(*work);
			DBG_HDCP_MSG("%s() cancel_work_sync ret = %x",__func__, ret);
		}
		if(*work) {
			kfree(*work);
			*work = 0;
		}
	}

}


/******************************************************************************
 * HDCP callbacks
 *****************************************************************************/


/*-----------------------------------------------------------------------------
 * Function: hdcp_start_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_start_frame_cb(void)
{
	DBG_HDCP_MSG("hdcp_start_frame_cb() %u\n", jiffies_to_msecs(jiffies));

	/* Cancel any pending work */
	hdcp_cancel_work(&hdcp.pending_start);
	hdcp_cancel_work(&hdcp.pending_wq_event);
	hdcp.hpd_low = 0;

	hdcp.pending_start = hdcp_submit_work(HDCP_START_FRAME_EVENT,
					      HDCP_ENABLE_DELAY,
					      0);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_stop_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_stop_frame_cb(void)
{
	int i = 10;

	DBG_HDCP_MSG("hdcp_stop_frame_cb() HDCP_state: %d , auth_state: %d %u\n",
	       hdcp.hdcp_state, hdcp.auth_state, jiffies_to_msecs(jiffies));

	/* Cancel any pending work */
	hdcp_cancel_work(&hdcp.pending_start);
	hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp.pending_disable = 1;

	/* If encryption going on and suspend button pressed,
	 * this shall take care
	 */
	while(hdcp.hdcp_state == HDCP_KEY_ENCRYPTION_ONGOING && i--)
#ifndef POWER_TRANSITION_DBG
		mdelay(5);
#else
		mdelay(5000);
#endif

	if (i <= 0)
		DBG_HDCP_MSG("HDCP: Encryption not finishing\n");

	if ((hdcp.hdcp_state != HDCP_DISABLED)&& (hdcp.hdmi_restart == 0))
		INIT_COMPLETION(hdcp_comp);

	hdcp_submit_work(HDCP_STOP_FRAME_EVENT, 0, 0);

	/* Function is blocking until stop frame event fully processed
	 * in HDCP workqueue to avoid HDMI driver powering down DSS/HDMI
	 */
	if ((hdcp.hdcp_state != HDCP_DISABLED)&&(hdcp.hdmi_restart == 0)) {
		DBG("HDCP stop callback blocked %u",
		    jiffies_to_msecs(jiffies)); 
		DBG_HDCP_MSG("HDCP stop callback blocked %u\n", jiffies_to_msecs(jiffies));

#ifndef POWER_TRANSITION_DBG
		if (wait_for_completion_timeout(&hdcp_comp, msecs_to_jiffies(HDCP_STOP_FRAME_BLOCKING_TIMEOUT)))
#else
		if (wait_for_completion_timeout(&hdcp_comp, msecs_to_jiffies(15000)))
#endif
			DBG_HDCP_MSG("HDCP stop callback unblocked\n");
		else
			DBG_HDCP_MSG("HDCP: stop frame callback blocking timeout - hdcp_state = %d %u\n",
			       hdcp.hdcp_state, hdcp.auth_state, jiffies_to_msecs(jiffies));
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_irq_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_irq_cb(int status)
{
	DBG_HDCP_MSG("hdcp_irq_cb() status=%x\n hdcp_state=%x\n, hdmi_state=%x\n, hdcp_auth=%x ,pending_disable=%x\n",
		status,hdcp.hdcp_state, hdcp.hdmi_state,hdcp.auth_state,hdcp.pending_disable);

	/* Disable auto Ri/BCAPS immediately */
	if (status & HDMI_RI_ERR) {
		hdcp_lib_auto_ri_check(false);
		hdcp_lib_auto_bcaps_rdy_check(false);
	}
	else if (status & HDMI_BCAP) {
		hdcp_lib_auto_bcaps_rdy_check(false);
	}

	/* Work queue execution not required if HDCP is disabled */
	/* TODO: ignore interrupts if they are masked (cannnot access UMASK here
	 * so should use global variable
	 */
	if ((hdcp.hdcp_state != HDCP_DISABLED) &&
	    (hdcp.hdcp_state != HDCP_ENABLE_PENDING)) {
		/* HPD low event provided by HDMI IRQ since already handled by HDMI driver */
		if (status & HDMI_HPD_LOW) {
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp_lib_auto_ri_check(false);
			hdcp_lib_auto_bcaps_rdy_check(false);
			DBG_ERR("<%s> calling hdcp_ddc_abort %d \n", __func__, status);

			hdcp_ddc_abort();

			hdcp.pending_disable = 1; /* Used to exit on-going HDCP work */
			hdcp.hpd_low = 1;	  /* Us/ed to cancel HDCP works */
			hdcp_submit_work(HDCP_HPD_LOW_EVENT, 0, 1);
		}

		if (status & HDMI_RI_ERR) {
			ri_check_fail = true;
			hdcp_lib_set_av_mute(AV_MUTE_SET);
			hdcp_lib_set_encryption(HDCP_ENC_OFF);
			hdcp_submit_work(HDCP_RI_FAIL_EVENT, 0, 1);
		}
		/* RI error takes precedence over BCAP */
		else if (status & HDMI_BCAP) {
			hdcp_submit_work(HDCP_KSV_LIST_RDY_EVENT, 0, 1);
		}
	}
}

/******************************************************************************
 * HDCP control from ioctl
 *****************************************************************************/


/*-----------------------------------------------------------------------------
 * Function: hdcp_enable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_enable_ctl(void __user *argp)
{
	DBG_HDCP_MSG("hdcp_ioctl() - ENABLE %u\n", jiffies_to_msecs(jiffies));

	if (hdcp.en_ctrl == 0) {
		hdcp.en_ctrl =
			kmalloc(sizeof(struct hdcp_enable_control),
							GFP_KERNEL);

		if (hdcp.en_ctrl == 0) {
			DBG_HDCP_MSG("HDCP: Cannot allocate memory for HDCP"
				" enable control struct\n");
			return -EFAULT;
		}
	}

	if (copy_from_user(hdcp.en_ctrl, argp,
			   sizeof(struct hdcp_enable_control))) {
		DBG_HDCP_MSG("HDCP: Error copying from user space "
				    "- enable ioctl\n");
		return -EFAULT;
	}
	DBG_HDCP_MSG("Ioctl enable: ->val %x, %x, %x, %x\n",hdcp.en_ctrl->key[0],hdcp.en_ctrl->key[1],hdcp.en_ctrl->key[2],hdcp.en_ctrl->key[3] );

	//smpark
	hdcp.en_ctrl->nb_retry = 0;
	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_ENABLE_CTL, 0, 0) == 0)
		return -EFAULT;

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_disable_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_disable_ctl(void)
{
	DBG_HDCP_MSG("hdcp_ioctl() - DISABLE %u\n", jiffies_to_msecs(jiffies));

	hdcp_cancel_work(&hdcp.pending_start);
	hdcp_cancel_work(&hdcp.pending_wq_event);

	hdcp.pending_disable = 1;
	/* Post event to workqueue */
	if (hdcp_submit_work(HDCP_DISABLE_CTL, 0, 0) == 0)
		return -EFAULT;

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_query_status_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_query_status_ctl(void __user *argp)
{
	uint32_t *status = (uint32_t *)argp;

	DBG_HDCP_MSG("hdcp_ioctl() - QUERY %u\n", jiffies_to_msecs(jiffies));
	*status = hdcp.auth_state;

	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_encrypt_key_ctl
 *-----------------------------------------------------------------------------
 */
static long hdcp_encrypt_key_ctl(void __user *argp)
{
	struct hdcp_encrypt_control *ctrl;
	uint32_t *out_key;

	DBG_HDCP_MSG("hdcp_ioctl() - ENCRYPT KEY %u\n", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	if (hdcp.hdcp_state != HDCP_DISABLED) {
		DBG_HDCP_MSG("HDCP: Cannot encrypt keys while HDCP "
				   "is enabled\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	hdcp.hdcp_state = HDCP_KEY_ENCRYPTION_ONGOING;

	/* Encryption happens in ioctl / user context */
	ctrl = kmalloc(sizeof(struct hdcp_encrypt_control),
		       GFP_KERNEL);

	if (ctrl == 0) {
		DBG_HDCP_MSG("HDCP: Cannot allocate memory for HDCP"
				    " encryption control struct\n");
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	out_key = kmalloc(sizeof(uint32_t) *
					DESHDCP_KEY_SIZE, GFP_KERNEL);

	if (out_key == 0) {
		DBG_HDCP_MSG("HDCP: Cannot allocate memory for HDCP "
				    "encryption output key\n");
		kfree(ctrl);
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}

	if (copy_from_user(ctrl, argp,
				sizeof(struct hdcp_encrypt_control))) {
		DBG_HDCP_MSG("HDCP: Error copying from user space"
				    " - encrypt ioctl\n");
		//DBG_HDCP_MSG("Before encrypt: ->val %x, %x, %x, %x\n",ctrl->in_key[0],ctrl->in_key[1],ctrl->in_key[2],ctrl->in_key[3] );
		kfree(ctrl);
		kfree(out_key);
		mutex_unlock(&hdcp.lock);
		return -EFAULT;
	}
	DBG_HDCP_MSG("Before encrypt: ->val %d\n",ctrl->in_key);

	hdcp_request_dss();

	/* Call encrypt function */
	hdcp_3des_encrypt_key(ctrl, out_key);
	DBG_HDCP_MSG("After Encrypt:-> %d\n",ctrl->in_key );
	hdcp_release_dss();

	hdcp.hdcp_state = HDCP_DISABLED;
	mutex_unlock(&hdcp.lock);

	/* Store output data to output pointer */
	if (copy_to_user(ctrl->out_key, out_key,
				sizeof(uint32_t)*DESHDCP_KEY_SIZE)) {
		DBG_HDCP_MSG("HDCP: Error copying to user space -"
				    " encrypt ioctl\n");
		kfree(ctrl);
		kfree(out_key);
		return -EFAULT;
	}

	kfree(ctrl);
	kfree(out_key);
	return 0;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
 #if 0
static int hdcpFlagEnc=0;
static int hdcpFlagEnable=0;
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HDCP_ENABLE:
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_ENABLE\n");
		if (hdcpFlagEnable==0){


			hdcpFlagEnable++;
			return hdcp_enable_ctl(argp);

		} else
			return 0;

	case HDCP_DISABLE:
		return 0;
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_DISABLE\n");
		return hdcp_disable_ctl();

	case HDCP_ENCRYPT_KEY:
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_DISABLE\n");
		if (hdcpFlagEnc==0){

			hdcpFlagEnc ++;

			return hdcp_encrypt_key_ctl(argp);
		} else
				return 0;

	case HDCP_QUERY_STATUS:
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_QUERY_STATUS\n");
		return hdcp_query_status_ctl(argp);

	default:
		return -ENOTTY;
	} /* End switch */
}
#else

long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HDCP_ENABLE:
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_ENABLE\n");

			return hdcp_enable_ctl(argp);



	case HDCP_DISABLE:
	DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_DISABLE\n");
		return hdcp_disable_ctl();

	case HDCP_ENCRYPT_KEY:

			DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_ENCRYPT_KEY\n");
			
			return hdcp_encrypt_key_ctl(argp);

	case HDCP_QUERY_STATUS:
		DBG_HDCP_MSG ("[HDCP] hdcp_ioctl HDCP_QUERY_STATUS\n");
		return hdcp_query_status_ctl(argp);

	default:
		return -ENOTTY;
	} /* End switch */
}
#endif

/******************************************************************************
 * HDCP driver init/exit
 *****************************************************************************/


static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hdcp_ioctl,
};

static struct miscdevice hdcp_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "hdcp",
    .fops   = &hdcp_fops,
};

/*-----------------------------------------------------------------------------
 * Function: hdcp_init
 *-----------------------------------------------------------------------------
 */
static int __init hdcp_init(void)
{
	DBG_HDCP_MSG("HDCP: hdcp_init\n");

	/* Map HDMI WP address */
	hdcp.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!hdcp.hdmi_wp_base_addr) {
		DBG_HDCP_MSG("HDCP: HDMI WP IOremap error\n");
		return -EFAULT;
	}

	/* Map DESHDCP in kernel address space */
	hdcp.deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp.deshdcp_base_addr) {
		DBG_HDCP_MSG("HDCP: DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp.lock);
#if 0
	/* Get the major number for this module */
	if (alloc_chrdev_region(&hdcp.dev_id, 0, 1, "hdcp")) {
		DBG_HDCP_MSG(KERN_ERR "HDCP: Cound not register character device\n");
		goto err_register;
	}

	/* Initialize character device */
	cdev_init(&hdcp_cdev, &hdcp_fops);
	hdcp_cdev.owner = THIS_MODULE;
	hdcp_cdev.ops = &hdcp_fops;

	/* add char driver */
	if (cdev_add(&hdcp_cdev, hdcp.dev_id, 1)) {
		DBG_HDCP_MSG(KERN_ERR "HDCP: Could not add character driver\n");
		goto err_add_driver;
	}
#else


	if( !misc_register(&hdcp_device)) {
		pr_err(KERN_ERR "misc_register failed - mhl \n");
	}



#endif
	mutex_lock(&hdcp.lock);

	/* Variable init */
	hdcp.en_ctrl  = 0;
	hdcp.hdcp_state = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.pending_wq_event = 0;
	hdcp.retry_cnt = 0;
	hdcp.auth_state = HDCP_STATE_DISABLED;
	hdcp.pending_disable = 0;
	hdcp.hpd_low = 0;
	hdcp.hdmi_restart = 0;
	hdcp.ddc_err_cnt = 0;

	spin_lock_init(&hdcp.spinlock);

	init_completion(&hdcp_comp);

	hdcp_request_dss();

	/* Register HDCP callbacks to HDMI library */
	if (hdmi_register_hdcp_callbacks(&hdcp_start_frame_cb,
					 &hdcp_stop_frame_cb,
					 &hdcp_irq_cb))
		hdcp.hdmi_state = HDMI_STARTED;
	else
		hdcp.hdmi_state = HDMI_STOPPED;

	hdcp_release_dss();

	mutex_unlock(&hdcp.lock);

	return 0;

err_add_driver:
//	cdev_del(&hdcp_cdev);

	unregister_chrdev_region(hdcp.dev_id, 1);

err_register:
	mutex_destroy(&hdcp.lock);

	iounmap(hdcp.deshdcp_base_addr);

err_map_deshdcp:
	iounmap(hdcp.hdmi_wp_base_addr);

	return -EFAULT;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_exit
 *-----------------------------------------------------------------------------
 */
static void __exit hdcp_exit(void)
{
	DBG_HDCP_MSG("hdcp_exit() %u\n", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	if (hdcp.en_ctrl)
		kfree(hdcp.en_ctrl);

	hdcp_request_dss();

	/* Un-register HDCP callbacks to HDMI library */
	hdmi_register_hdcp_callbacks(0, 0, 0);

	hdcp_release_dss();

	/* Unmap HDMI WP / DESHDCP */
	iounmap(hdcp.hdmi_wp_base_addr);
	iounmap(hdcp.deshdcp_base_addr);

	/* Unregister char device */
	//cdev_del(&hdcp_cdev);
	unregister_chrdev_region(hdcp.dev_id, 1);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("BSD");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Fabrice Olivero");

