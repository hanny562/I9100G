/*
 * hdcp.c
 *
 * HDCP support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Fabrice Olivero <f-olivero@ti.com>
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <plat/display.h>
#include <linux/miscdevice.h>
#include <../drivers/video/omap2/dss/hdmi.h>

#define DEBUG		/* Comment to remove debug printk */
/*#define DSS_INACTIVITY	 Un-comment when DSS inactivity will be supported */

#include "hdcp.h"

enum hdcp_states {
	HDCP_DISABLED,
	HDCP_ENABLED
};

enum hdmi_states {
	HDMI_STOPPED,
	HDMI_STARTED
};

struct hdcp_delayed_work {
	struct delayed_work work;
	int event;
};

static struct hdcp {
	void __iomem *hdmi_wp_base_addr;
	void __iomem *deshdcp_base_addr;
	struct mutex lock;
	struct mutex cb_lock;
	struct hdcp_enable_control *en_ctrl;
	dev_t dev_id;
	struct class *hdcp_class;
	enum hdmi_states hdmi_state;
	enum hdcp_states hdcp_state;
	enum hdcp_states request;
	struct delayed_work *pending_start;
	int retry_cnt;
	int auth_fail_restart;
	int auth_fail;
	int auth_done;
} hdcp;

/*-----------------------------------------------------------------------------
 * Function: hdcp_check_aksv
 *-----------------------------------------------------------------------------
 */
static int hdcp_check_aksv(void)
{
	uint8_t aksv[5];
	int i, j;
	int zero = 0, one = 0;

	for(i = 0; i < 5; i++) {
		aksv[i] =
			(RD_REG_32(hdcp.hdmi_wp_base_addr +
				   HDMI_IP_CORE_SYSTEM,
		   		   HDMI_IP_CORE_SYSTEM__AKSV0 +
				   i*sizeof(uint32_t))
			) & 0xFF;

		/* Count number of zero / one */
		for (j = 0; j < 8; j++) {
			if (aksv[i] & (0x01 << j))
				one++;
			else
				zero++;
		}
	}

	printk("AKSV: %x %x %x %x %x 0:%d 1:%d", aksv[0], aksv[1], aksv[2], aksv[3], aksv[4], zero, one);
	printk("AKSV: RETURN 0/n");

		return 0;
#if 0
	if (one == zero)
		return 0;
	else
		return -1;
#endif
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_load_key_3des
 *-----------------------------------------------------------------------------
 */
static int hdcp_load_key_3des(uint32_t *deshdcp_encrypted_key)
{
	int counter = 0, status = HDCP_OK;

	printk("Loading HDCP keys...");

	hdcp_request_dss();

	/* Set decryption mode in DES control register */
  	WR_FIELD_32(hdcp.deshdcp_base_addr,
		    DESHDCP__DHDCP_CTRL,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		    0x0);

	/* Write encrypted data */
	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			  deshdcp_encrypted_key[counter]);
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			  deshdcp_encrypted_key[counter + 1]);

		/* Wait for output bit at '1' */
		while(
			RD_FIELD_32(hdcp.deshdcp_base_addr,
				    DESHDCP__DHDCP_CTRL,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L
			) != 0x1) {};

		/* Dummy read (indeed data are transfered directly into
		 * key memory)
		 */
		if (RD_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L) !=
									0x0) {
			status = HDCP_ERROR;
			printk(KERN_ERR "DESHDCP dummy read error\n");
		}
		if (RD_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H) !=
									0x0) {
			status = HDCP_ERROR;
			printk(KERN_ERR "DESHDCP dummy read error\n");
		}

		counter+=2;
	}

	hdcp_release_dss();

	return status;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_enable
 *-----------------------------------------------------------------------------
 */
static void hdcp_enable(uint32_t *deshdcp_encrypted_key)
{
	uint32_t hsa_ctrl = 0;

	printk("hdcp_enable()\n");

	/* Load DESHDCP keys */
	if (hdcp_load_key_3des(deshdcp_encrypted_key) == HDCP_ERROR)
		return;

	hdcp_request_dss();

	/* FIXME: Clear HPD to get HDCP enabled - should be handled by
	 * HDMI handler when hot plug modify will be supported
	 */
	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  0x1c4,
		  0x40);

	RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  0x1c4);

	/* Enable HSA */
	printk("HSA enabling...");

	/* Release HDCP reset */
	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__HDCP_CTRL,
		  0x4);

	/* Enable encryption */
	hsa_ctrl |= HSA_CTRL__COPP_PROTLEVEL_SET;

	/* Set HSA */
	hsa_ctrl &= ~HSA_CTRL__HSA_SW_SET;

	/* Always set sink type to HDMI since DVI mode is not working */
	hsa_ctrl |= HSA_CTRL__SINK_TYPE_HDMI_SET;

	/* Write R0 calc time depending on VSYNC */
	/* 0.1 * vsync in Hz */
	hsa_ctrl |= 7 << HSA_CTRL__R0_CALC_TIME_POS_L;

	printk("HSA_CTRL=%x",hsa_ctrl);

	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_GAMUT,
		  HDMI_IP_CORE_GAMUT__HSA_CTRL1,
		  hsa_ctrl);

	if (hdcp_check_aksv() < 0) {
		printk("AKSV error (number of 0 and 1 not equal)."
		    "Disabling HDCP...");
		hdcp_disable();
	}

	hdcp_release_dss();

	/* Enable delayed work to check authentication status */
	hdcp_submit_work(HDCP_CHECK_AUTH_STATUS, HDCP_AUTH_DELAY);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_disable
 *-----------------------------------------------------------------------------
 */
static void hdcp_disable(void)
{
	uint32_t hsa_ctrl;

	/* Disable HDCP */
	printk("hdcp_disable() %u", jiffies_to_msecs(jiffies));

	hdcp_request_dss();

	hsa_ctrl = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_GAMUT,
			     HDMI_IP_CORE_GAMUT__HSA_CTRL1);

	/* Disable encryption */
	hsa_ctrl &= ~HSA_CTRL__COPP_PROTLEVEL_SET;

	/* Reset HSA */
	hsa_ctrl |= HSA_CTRL__HSA_SW_SET;

	printk("HSA_CTRL=%x",hsa_ctrl);

	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_GAMUT,
		  HDMI_IP_CORE_GAMUT__HSA_CTRL1,
		  hsa_ctrl);

	/* Set HDCP reset */
	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__HDCP_CTRL,
		  0x0);

	hdcp_release_dss();
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_check_auth_status
 *-----------------------------------------------------------------------------
 */
static void hdcp_check_auth_status(void)
{
	enum hsa_state status;

	hdcp_request_dss();

	status = (enum hsa_state)
		 (RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_GAMUT,
				HDMI_IP_CORE_GAMUT__HSA_STATUS))
				>> HSA_STATUS__HW_CS_POS_L;

	hdcp_release_dss();

	if ((status < HSA_STATE_1ST_STEP_AUTH) ||
	    (status == HSA_STATE_AUTH_FAILURE)) {
		/* Restart HDMI */
		if (hdcp.retry_cnt > 0)
		{
			printk("Authentication failure - Restarting HDMI. Attempt"
			    " %d, HW status: %x",
				hdcp.retry_cnt,
				status);
			hdcp.auth_done = 0;
			hdcp.auth_fail_restart = 1;
			hdmi_restart();
			hdcp.retry_cnt--;
		}
		else
		{
			printk("Authentication failure - All attempts FAILED - "
			    "HW status: %x",
				status);
			hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
			hdcp.auth_done = 0;
			hdcp.auth_fail_restart = 0;
			hdcp.auth_fail = 1;
		}
	}
	else if (status == HSA_STATE_LINK_INTEGRITY_CHECK) {
		/* Status is ok */
		printk("Authentication success\n");
		hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
		hdcp.auth_fail_restart = 0;
		hdcp.auth_done = 1;
	}
	else {
		printk("Wait for authentication completion - HW status: %x",
				status);

		/* Wait for another 200 ms
		 * (2nd step authentication may take up to 5s)
		 */
		hdcp_submit_work(HDCP_CHECK_AUTH_STATUS, HDCP_AUTH_DELAY);
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
	enum hdcp_transition {
		TR_NONE,
		TR_HDCP_ENABLE,
		TR_HDCP_DISABLE
	} transition;

	mutex_lock(&hdcp.lock);

	transition = TR_NONE;
	
	printk("hdcp_work_queue() - START - %u req=%d hdmi=%d hdcp=%d cmd=%d",
		jiffies_to_msecs(jiffies),
		hdcp.request,
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		event);

	switch (event) {
	case HDCP_ENABLE_REQ:
		printk("workqueue: HDCP enable request");

		hdcp.request = HDCP_ENABLED; 
		hdcp.retry_cnt = hdcp.en_ctrl->nb_retry;
		hdcp.auth_fail_restart = 0;
			
		if ((hdcp.hdmi_state == HDMI_STARTED) &&
		    (hdcp.hdcp_state == HDCP_DISABLED))
			transition = TR_HDCP_ENABLE;

		break;

	case HDCP_DISABLE_REQ:
		printk("workqueue: HDCP disable request");

		if (hdcp.en_ctrl) {
			kfree(hdcp.en_ctrl);
			hdcp.en_ctrl = 0;
		}

		hdcp.request = HDCP_DISABLED;

		if (hdcp.hdcp_state == HDCP_ENABLED)
			transition = TR_HDCP_DISABLE;

		break;

	case HDCP_START_FRAME_EVENT:
		printk("workqueue: HDCP start frame event");

		mutex_lock(&hdcp.cb_lock);
		hdcp.pending_start = 0;
		mutex_unlock(&hdcp.cb_lock);
		hdcp.hdmi_state = HDMI_STARTED;

		if (hdcp.request == HDCP_ENABLED)
			transition = TR_HDCP_ENABLE;

		break;

	case HDCP_STOP_FRAME_EVENT:
		printk("workqueue: HDCP stop frame event\n");

		hdcp.hdmi_state = HDMI_STOPPED;

		if (hdcp.hdcp_state == HDCP_ENABLED)
			transition = TR_HDCP_DISABLE;

		break;

	case HDCP_CHECK_AUTH_STATUS:
		printk("workqueue: HDCP authentication check\n");

		if (hdcp.hdcp_state == HDCP_ENABLED)
			hdcp_check_auth_status();

		break;

	default:
		printk(KERN_ERR "HDCP: Unknow workqueue event\n");
	break;
	}

	switch(transition) {
	case TR_NONE:
		/* Do nothing */
		break;

	case TR_HDCP_ENABLE:
		hdcp.auth_fail = 0;
		hdcp.auth_done = 0;

		if (hdcp.en_ctrl) {
			hdcp_enable(hdcp.en_ctrl->key);
			hdcp.hdcp_state = HDCP_ENABLED;
		}
		else
			printk(KERN_ERR "HDCP: no key available to "
					"enable HDCP !\n");
		break;

	case TR_HDCP_DISABLE:
		hdcp.auth_fail = 0;
		hdcp.auth_done = 0;

		hdcp_disable();
		hdcp.hdcp_state = HDCP_DISABLED;
		break;

	default:
		printk(KERN_ERR "HDCP: unknow HW transition\n");
		break;
	}

	kfree(hdcp_w);

	printk("hdcp_work_queue() - END - %u transition=%d req=%d hdmi=%d hdcp=%d"
	    " cmd=%d",
		jiffies_to_msecs(jiffies),
		transition,
		hdcp.request,
		hdcp.hdmi_state,
		hdcp.hdcp_state,
		event);

	mutex_unlock(&hdcp.lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_submit_work
 *-----------------------------------------------------------------------------
 */
static struct delayed_work *hdcp_submit_work(int event, int delay)
{
	struct hdcp_delayed_work *work;
	work = kmalloc(sizeof(struct hdcp_delayed_work), GFP_KERNEL);

	if (work) {
		work->event = event;
		INIT_DELAYED_WORK(&work->work, hdcp_work_queue);
		schedule_delayed_work(&work->work, msecs_to_jiffies(delay));
	}
	else {
		printk(KERN_ERR "HDCP: Cannot allocate memory to create work");
		return 0;
	}

	return &work->work;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_3des_enc_key
 *-----------------------------------------------------------------------------
 */
static void hdcp_3des_enc_key(struct hdcp_encrypt_control *enc_ctrl,
			      uint32_t out_key[DESHDCP_KEY_SIZE])
{
	int counter = 0;

	printk("Encrypting HDCP keys...");

	hdcp_request_dss();

	/* Reset encrypted key array */
	for (counter = 0; counter < DESHDCP_KEY_SIZE; counter++)
		out_key[counter] = 0;

	/* Set encryption mode in DES control register */
  	WR_FIELD_32(hdcp.deshdcp_base_addr,
		    DESHDCP__DHDCP_CTRL,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		    0x1);

  	/* Write raw data and read encrypted data */
  	counter = 0;

	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			  enc_ctrl->in_key[counter]);
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			  enc_ctrl->in_key[counter + 1]);

		/* Wait for output bit at '1' */
		while(
			RD_FIELD_32(hdcp.deshdcp_base_addr,
				    DESHDCP__DHDCP_CTRL,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L
			) != 0x1) {};

    		/* Read enrypted data */
    		out_key[counter]     = RD_REG_32(hdcp.deshdcp_base_addr,
						 DESHDCP__DHDCP_DATA_L);
    		out_key[counter + 1] = RD_REG_32(hdcp.deshdcp_base_addr,
						 DESHDCP__DHDCP_DATA_H);

		counter+=2;
	}

	hdcp_release_dss();
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_start_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_start_frame_cb(void)
{
	int ret = 0;
	printk("hdcp_start_frame_cb() %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.cb_lock);

	if (hdcp.pending_start) {
		ret = cancel_delayed_work(hdcp.pending_start);
		if(ret <= 0) {
			ret = cancel_work_sync(&(hdcp.pending_start->work));
			printk(KERN_ERR"%s() cancel_work_sync ret = %x",__func__, ret);
		}
		if(hdcp.pending_start) {
			kfree(hdcp.pending_start);
			hdcp.pending_start = 0;
		}
	}

	hdcp.pending_start = hdcp_submit_work(HDCP_START_FRAME_EVENT,
					      HDCP_ENABLE_DELAY);
	mutex_unlock(&hdcp.cb_lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_stop_frame_cb
 *-----------------------------------------------------------------------------
 */
static void hdcp_stop_frame_cb(void)
{
	int ret = 0;
	printk("hdcp_stop_frame_cb() %u", jiffies_to_msecs(jiffies));

	/* Cancel any pending "Start Frame" work */
	mutex_lock(&hdcp.cb_lock);

	if (hdcp.pending_start) {
		ret = cancel_delayed_work(hdcp.pending_start);
		if(ret <= 0) {
			ret = cancel_work_sync(&(hdcp.pending_start->work));
			printk(KERN_ERR"%s() cancel_work_sync ret = %x",__func__, ret);
		}
		if(hdcp.pending_start) {
			kfree(hdcp.pending_start);
			hdcp.pending_start = 0;
		}
	}

	hdcp_submit_work(HDCP_STOP_FRAME_EVENT, 0);

	mutex_unlock(&hdcp.cb_lock);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_ioctl
 *-----------------------------------------------------------------------------
 */
long hdcp_ioctl(struct file *fd, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	printk("hdcp_ioctl()\n");

	switch (cmd) {
	case HDCP_ENABLE:
		printk("hdcp_ioctl() - ENABLE but just return \n");
		return 0;
		if (hdcp.en_ctrl == 0) {
			hdcp.en_ctrl =
				kmalloc(sizeof(struct hdcp_enable_control),
								GFP_KERNEL);

			if (hdcp.en_ctrl == 0) {
				printk(KERN_ERR
					"HDCP: Cannot allocate memory for HDCP"
					" enable control struct");
				return -EFAULT;
			}
		}
		
		if (copy_from_user(hdcp.en_ctrl, argp,
				   sizeof(struct hdcp_enable_control))) {
			printk(KERN_ERR "HDCP: Error copying from user space "
					"- enable ioctl");
			return -EFAULT;
		}

		/* Post event to workqueue */
		if (hdcp_submit_work(HDCP_ENABLE_REQ, 0) == 0)
			return -EFAULT;

		break;

	case HDCP_DISABLE:
		printk("hdcp_ioctl() - DISABLE but just return\n");

		/* Post event to workqueue */
		if (hdcp_submit_work(HDCP_DISABLE_REQ, 0) == 0)
			return -EFAULT;

		break;

	case HDCP_ENCRYPT_KEY: {
		struct hdcp_encrypt_control *ctrl;
		uint32_t *out_key;

		printk("hdcp_ioctl() - HDCP_ENCRYPT_KEY \n");

		mutex_lock(&hdcp.lock);

		if (hdcp.hdcp_state == HDCP_ENABLED) {
			printk(KERN_ERR "HDCP: Cannot encrypt keys while HDCP "
					"is enabled");
			return -EFAULT;
		}

		mutex_unlock(&hdcp.lock);

		/* Encryption happens in ioctl / user context */
		ctrl = kmalloc(sizeof(struct hdcp_encrypt_control),
			       GFP_KERNEL);

		if (ctrl == 0) {
			printk(KERN_ERR "HDCP: Cannot allocate memory for HDCP"
					" encryption control struct");
			return -EFAULT;
		}

		out_key = kmalloc(sizeof(uint32_t) *
						DESHDCP_KEY_SIZE, GFP_KERNEL);

		if (out_key == 0) {
			printk(KERN_ERR "HDCP: Cannot allocate memory for HDCP "
					"encryption output key");
			kfree(ctrl);	
			return -EFAULT;
		}

		if (copy_from_user(ctrl, argp,
					sizeof(struct hdcp_encrypt_control))) {
			printk(KERN_ERR "HDCP: Error copying from user space"
					" - encrypt ioctl");
			goto hdcp_encrypt_error;
		}

		/* Call encrypt function */
		hdcp_3des_enc_key(ctrl, out_key);

		/* Store output data to output pointer */
		if (copy_to_user(ctrl->out_key, out_key,
					sizeof(uint32_t)*DESHDCP_KEY_SIZE)) {
			printk(KERN_ERR "HDCP: Error copying to user space -"
					" encrypt ioctl");
			goto hdcp_encrypt_error;
		}

		kfree(ctrl);
		kfree(out_key);
		return 0;

hdcp_encrypt_error:
		kfree(ctrl);
		kfree(out_key);
		return -EFAULT;
	}
	break;

	case HDCP_QUERY_STATUS: {
		uint32_t *status = (uint32_t *)arg;
		int auth_fail = 0, auth_fail_restart = 0, auth_done = 0;
		uint32_t map_hsa_state[HSA_NB_STATES] = {
			HDCP_STATE_DISABLED,	  /* HSA_STATE_IDLE */
			HDCP_STATE_INIT,	  /* HSA_STATE_INIT */
			HDCP_STATE_DISABLED,	  /* HSA_STATE_POWER_DOWN */
			HDCP_STATE_INIT,	  /* HSA_STATE_CONFIG */
			HDCP_STATE_INIT,	  /* HSA_STATE_PREPARATION */
			HDCP_STATE_AUTH_1ST_STEP, /* HSA_STATE_1ST_STEP_AUTH */
			HDCP_STATE_AUTH_2ND_STEP, /* HSA_STATE_2ND_STEP_AUTH */
			HDCP_STATE_AUTH_3RD_STEP, /* HSA_STATE_LINK_INTEGRITY
								      _CHECK */
			HDCP_STATE_AUTH_FAILURE	  /* HSA_STATE_AUTH_FAILURE */
		};

		printk("hdcp_ioctl() - QUERY %u", jiffies_to_msecs(jiffies));

		mutex_lock(&hdcp.lock);
		/* Read SW authentication status (used to bypass HW status) */
		auth_fail_restart = hdcp.auth_fail_restart;
		auth_fail = hdcp.auth_fail;
		auth_done = hdcp.auth_done;
		mutex_unlock(&hdcp.lock);

		/* Authentication failed - ignore HSA status */
		if (auth_fail)
			*status = HDCP_STATE_AUTH_FAILURE;
		/* HDMI restart ongoing - ignore HSA status */
		else if (auth_fail_restart)
			*status = HDCP_STATE_AUTH_FAIL_RESTARTING;
		else {
			uint32_t hw_state;

			hdcp_request_dss();
			hw_state =
				RD_REG_32(hdcp.hdmi_wp_base_addr +
					  HDMI_IP_CORE_GAMUT,
					  HDMI_IP_CORE_GAMUT__HSA_STATUS);

			hdcp_release_dss();

			*status = map_hsa_state[hw_state
						>> HSA_STATUS__HW_CS_POS_L];

			/*
			 * When authentication has been succesful (i.e. link
                         * integrity check) HSA status HSA_STATE_INIT,
			 * HSA_STATE_CONFIG, HSA_STATE_PREPARATION are
			 * reported as authentication failure
			 */
			if ((hdcp.auth_done)&&(*status == HDCP_STATE_INIT))
				*status = HDCP_STATE_AUTH_FAILURE;
				
			printk("HDCP status hw: %x sw: %d\n", hw_state, *status);
		}
	
		break;
	}

	default:
		return -ENOTTY;
	} /* End switch */

	return 0;
}

#if 0
static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hdcp_ioctl,
};

static struct cdev hdcp_cdev;
#else


static struct file_operations hdcp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hdcp_ioctl,
};
                 
static struct miscdevice hdcp_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "hdcp",
    .fops   = &hdcp_fops,
};
#endif
/*-----------------------------------------------------------------------------
 * Function: hdcp_init
 *-----------------------------------------------------------------------------
 */

static int __init hdcp_init(void)
{
	printk( "HDCP: hdcp_init\n");

	/* Map HDMI WP address */
	hdcp.hdmi_wp_base_addr = ioremap(HDMI_WP, 0x1000);

	if (!hdcp.hdmi_wp_base_addr) {
		printk(KERN_ERR "HDCP: HDMI WP IOremap error\n");
		return -EFAULT;
	}

	/* Map DESHDCP in kernel address space */
	hdcp.deshdcp_base_addr = ioremap(DSS_SS_FROM_L3__DESHDCP, 0x34);

	if (!hdcp.deshdcp_base_addr) {
		printk(KERN_ERR "HDCP: DESHDCP IOremap error\n");
		goto err_map_deshdcp;
	}

	mutex_init(&hdcp.lock);
	mutex_init(&hdcp.cb_lock);
#if 0
	/* Get the major number for this module */
	if (alloc_chrdev_region(&hdcp.dev_id, 0, 1, "hdcp")) {
		printk(KERN_ERR "HDCP: Cound not register character device\n");
		goto err_register;
	}
	printk("HDCP: hdcp.dev_id =%d\n", MAJOR(hdcp.dev_id));

	/* Initialize character device */
	cdev_init(&hdcp_cdev, &hdcp_fops);
	hdcp_cdev.owner = THIS_MODULE;
	hdcp_cdev.ops = &hdcp_fops;

	/* add char driver */
	if (cdev_add(&hdcp_cdev, hdcp.dev_id, 1)) {
		printk(KERN_ERR "HDCP: Could not add character driver\n");
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
	hdcp.request = HDCP_DISABLED;
	hdcp.pending_start = 0;
	hdcp.retry_cnt = 0;
	hdcp.auth_fail_restart = 0;
	hdcp.auth_fail = 0;
	hdcp.auth_done = 0;
	
	/* Register HDCP callbacks to HDMI library */
	if (hdmi_register_hdcp_callbacks(&hdcp_start_frame_cb,
					 &hdcp_stop_frame_cb))
		hdcp.hdmi_state = HDMI_STARTED;
	else
		hdcp.hdmi_state = HDMI_STOPPED;

	mutex_unlock(&hdcp.lock);

	return 0;

err_add_driver:
	//cdev_del(&hdcp_cdev);

	//unregister_chrdev_region(hdcp.dev_id, 1);

err_register:
	mutex_destroy(&hdcp.lock);
	mutex_destroy(&hdcp.cb_lock);

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
	printk("hdcp_exit() %u", jiffies_to_msecs(jiffies));

	mutex_lock(&hdcp.lock);

	if (hdcp.en_ctrl)
		kfree(hdcp.en_ctrl);

	/* Un-register HDCP callbacks to HDMI library */
	hdmi_register_hdcp_callbacks(0, 0);

	/* Unmap HDMI WP / DESHDCP */
	iounmap(hdcp.hdmi_wp_base_addr);
	iounmap(hdcp.deshdcp_base_addr);

	/* Unregister char device */
	//cdev_del(&hdcp_cdev);
	unregister_chrdev_region(hdcp.dev_id, 1);

	mutex_unlock(&hdcp.lock);

	mutex_destroy(&hdcp.lock);
	mutex_destroy(&hdcp.cb_lock);
}

/*-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
module_init(hdcp_init);
module_exit(hdcp_exit);
MODULE_LICENSE("BSD");
MODULE_DESCRIPTION("OMAP HDCP kernel module");
MODULE_AUTHOR("Fabrice Olivero");

