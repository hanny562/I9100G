/**
 * Samsung Virtual Network driver using IpcHsi device
 *
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
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
 */

//#define DEBUG

//#define FORMAT_TX_DUMP
//#define RAW_TX_DUMP
//#define RFS_TX_DUMP
//#define ALL_TX_DUMP

//#define FORMAT_RX_DUMP
//#define RAW_RX_DUMP
//#define RFS_RX_DUMP
//#define ALL_RX_DUMP

//#define RAW_TX_RX_LENGTH_DUMP
//#define RFS_TX_RX_LENGTH_DUMP

//#define LOOP_BACK_TEST
//#define LOOP_BACK_TEST_TX_ONLY
//#define LOOP_BACK_TEST_RX_ONLY


#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/phone_svn/ipc_hsi.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include <linux/vmalloc.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/in.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/pm_qos_params.h>
#include <plat/omap-pm.h>
#include <linux/hsi_driver_if.h>
#include <mach/sec_debug.h>

#define USE_WAKELOCK_TO_CTRL_WAKELINE
#ifdef USE_WAKELOCK_TO_CTRL_WAKELINE
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
static struct wake_lock ipc_hsi_wlock;
#endif
#endif

//static struct pm_qos_request_list *pm_qos_handle;
#define DRVNAME "onedram"

#define ONEDRAM_REG_OFFSET	0xFFF800
#define ONEDRAM_REG_SIZE		0x800

#define MB_VALID					0x0080
#define MB_COMMAND				0x0040

#define MB_CMD( x )					( MB_VALID | MB_COMMAND | x )
#define MB_DATA( x )				( MB_VALID | x )

#define MBD_SEND_FMT			0x0002
#define MBD_SEND_RAW			0x0001
#define MBD_SEND_RFS				0x0100
#define MBD_REQ_ACK_FMT		0x0020
#define MBD_REQ_ACK_RAW		0x0010
#define MBD_REQ_ACK_RFS		0x0400
#define MBD_RES_ACK_FMT		0x0008
#define MBD_RES_ACK_RAW		0x0004
#define MBD_RES_ACK_RFS			0x0200

#define HSI_CONTROL_CHANNEL		0
#define HSI_FLASHLESS_CHANNEL		0
#define HSI_FMT_CHANNEL		1
#define HSI_RAW_CHANNEL		2
#define HSI_RFS_CHANNEL		3
#define HSI_CMD_CHANNEL		4
#define HSI_LOOPBACK_CHANNEL		4
#define HSI_NUM_OF_USE_CHANNELS		5

#define HSI_MAX_CHANNELS		16
#define CHANNEL_MASK		0xFF
#define HSI_LL_INVALID_CHANNEL		0xFF

#define HSI_CHANNEL_TX_STATE_UNAVAIL		( 1 << 0 )
#define HSI_CHANNEL_TX_STATE_WRITING		( 1 << 1 )
#define HSI_CHANNEL_RX_STATE_UNAVAIL		( 1 << 0 )
#define HSI_CHANNEL_RX_STATE_READING		( 1 << 1 )

#define HSI_READ_TIMEOUT_DISABLE		0
#define HSI_READ_TIMEOUT_ENABLE		1
#define HSI_READ_DONE_TIMEOUT		( HZ )
#define HSI_WRITE_DONE_TIMEOUT		( HZ )
#define HSI_ACK_DONE_TIMEOUT		( HZ )
#define HSI_CLOSE_CONN_DONE_TIMEOUT		( HZ )
#define HSI_ACWAKE_DOWN_TIMEOUT		( HZ / 2 )


enum {
	MBC_NONE = 0,
	MBC_INIT_START,				// 0x0001
	MBC_INIT_END,				// 0x0002
	MBC_REQ_ACTIVE,			// 0x0003
	MBC_RES_ACTIVE,			// 0x0004
	MBC_TIME_SYNC,				// 0x0005
	MBC_POWER_OFF,			// 0x0006
	MBC_RESET,					// 0x0007
	MBC_PHONE_START,			// 0x0008
	MBC_ERR_DISPLAY,			// 0x0009
	MBC_POWER_SAVE,			// 0x000A
	MBC_NV_REBUILD,			// 0x000B
	MBC_EMER_DOWN,			// 0x000C
	MBC_REQ_SEM,				// 0x000D
	MBC_RES_SEM,				// 0x000E
	MBC_MAX						// 0x000F
};
#define MBC_MASK					0xFF

enum {
	HSI_INIT_MODE_NORMAL		= 0,
	HSI_INIT_MODE_FLASHLESS_BOOT,
	HSI_INIT_MODE_CP_RAMDUMP,
};

enum {
	HSI_LL_MSG_BREAK		= 0x00,
	HSI_LL_MSG_ECHO		= 0x01,
	HSI_LL_MSG_INFO_REQ		= 0x02,
	HSI_LL_MSG_INFO		= 0x03,
	HSI_LL_MSG_CONFIGURE		= 0x04,
	HSI_LL_MSG_ALLOCATE_CH		= 0x05,
	HSI_LL_MSG_RELEASE_CH		= 0x06,
	HSI_LL_MSG_OPEN_CONN		= 0x07,
	HSI_LL_MSG_CONN_READY		= 0x08,
	HSI_LL_MSG_CONN_CLOSED		= 0x09,
	HSI_LL_MSG_CANCEL_CONN		= 0x0A,
	HSI_LL_MSG_ACK		= 0x0B,
	HSI_LL_MSG_NAK		= 0x0C,
	HSI_LL_MSG_CONF_RATE		= 0x0D,
	HSI_LL_MSG_OPEN_CONN_OCTET		= 0x0E,
	HSI_LL_MSG_INVALID		= 0xFF,
};

enum
{
	HSI_LL_TX_STATE_UNDEF,
	HSI_LL_TX_STATE_CLOSED,
	HSI_LL_TX_STATE_IDLE,
	HSI_LL_TX_STATE_POWER_DOWN,
	HSI_LL_TX_STATE_ERROR,
	HSI_LL_TX_STATE_SEND_OPEN_CONN,
	HSI_LL_TX_STATE_WAIT_FOR_ACK,
	HSI_LL_TX_STATE_NACK,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_READY,
	HSI_LL_TX_STATE_SEND_CONF_RATE,
	HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK,
	HSI_LL_TX_STATE_TX,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED,
	HSI_LL_TX_STATE_TO_OPEN_CONN,
	HSI_LL_TX_STATE_TO_ACK,
	HSI_LL_TX_STATE_TO_READY,
	HSI_LL_TX_STATE_TO_CONF,
	HSI_LL_TX_STATE_TO_CONF_ACK,
	HSI_LL_TX_STATE_TO_TX,
	HSI_LL_TX_STATE_TO_CLOSE,
	HSI_LL_TX_STATE_SEND_BREAK,
};

enum {
	HSI_LL_RX_STATE_UNDEF,
	HSI_LL_RX_STATE_CLOSED,
	HSI_LL_RX_STATE_IDLE,
	HSI_LL_RX_STATE_POWER_DOWN,
	HSI_LL_RX_STATE_ERROR,
	HSI_LL_RX_STATE_BLOCKED,
	HSI_LL_RX_STATE_SEND_ACK,
	HSI_LL_RX_STATE_SEND_NACK,
	HSI_LL_RX_STATE_SEND_CONN_READY,
	HSI_LL_RX_STATE_RX,
	HSI_LL_RX_STATE_SEND_CONN_CLOSED,
	HSI_LL_RX_STATE_SEND_CONN_CANCEL,
	HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK,
	HSI_LL_RX_STATE_SEND_CONF_ACK,
	HSI_LL_RX_STATE_SEND_CONF_NACK,
	HSI_LL_RX_STATE_TO_RX,
	HSI_LL_RX_STATE_TO_ACK,
	HSI_LL_RX_STATE_TO_NACK,
	HSI_LL_RX_STATE_TO_CONN_READY,
	HSI_LL_RX_STATE_TO_CONN_CLOSED,
	HSI_LL_RX_STATE_TO_CONN_CANCEL,
	HSI_LL_RX_STATE_TO_CONN_CANCEL_ACK,
	HSI_LL_RX_STATE_TO_CONF_ACK,
	HSI_LL_RX_STATE_SEND_BREAK,
};


struct onedram_reg_mapped {
	u32 sem;
	u32 reserved1[ 7 ];
	u32 mailbox_AB;  // CP write, AP read
	u32 reserved2[ 7 ];
	u32 mailbox_BA;  // AP write, CP read
	u32 reserved3[ 23 ];
	u32 check_AB;    // can't read
	u32 reserved4[ 7 ];
	u32 check_BA;    // 0: CP read, 1: CP don't read
};

struct ipc_hsi_handler {
	struct list_head list;
	void *data;
	void ( *handler )( u32, void * );
};

struct ipc_hsi_handler_head {
	struct list_head list;
	u32 len;
	spinlock_t lock;
};
static struct ipc_hsi_handler_head h_list;

static struct resource ipc_hsi_resource = {
	.name = DRVNAME,
	.start = 0,
	.end = -1,
	.flags = IORESOURCE_MEM,
};

struct ipc_hsi {
	struct class *class;
	struct device *dev;
	struct cdev cdev;
	dev_t devid;

	wait_queue_head_t waitq;
	struct fasync_struct *async_queue;
	u32 mailbox;

	unsigned long base;
	unsigned long size;
	void __iomem *mmio;

	int irq;

	struct completion comp;
	atomic_t ref_sem;
	unsigned long flags;

	const struct attribute_group *group;

	struct onedram_reg_mapped *reg;
};
struct ipc_hsi *ipc_hsi;

struct workqueue_struct* ipc_hsi_wq;
typedef struct ipc_hsi_send_modem_bin_workq_data {
	struct ipc_hsi *od;
	
	struct work_struct send_modem_w;
} ipc_hsi_send_modem_bin_workq_data_t;
struct ipc_hsi_send_modem_bin_workq_data *ipc_hsi_send_modem_work_data;

typedef struct ipc_hsi_data_rec {
	u8 *buf;

	u32 channel;
	u32 len;
	u8 more;
} ipc_hsi_data;


static int hsi_init_handshake( int mode );
static int if_hsi_read( int ch, u32 *data, unsigned int size, const char *func, int time_out_flag );
static int if_hsi_write( int ch, u32 *data, unsigned int size, const char *func );
static int hsi_protocol_read( int ch, u32 *data, u32 *size );
static int hsi_protocol_write( int ch, u32 *data, unsigned int size );
static void hsi_acwake_down_timer_func( unsigned long data );
static struct timer_list hsi_acwake_down_timer;

static DEFINE_SPINLOCK( ipc_hsi_lock );

static unsigned long hw_tmp; /* for hardware */
static inline int _read_sem( struct ipc_hsi *od );
static inline void _write_sem( struct ipc_hsi *od, int v );


struct task_struct *tx_thread = NULL;
struct completion tx_thread_start;
struct task_struct *fmt_read_thread = NULL;
struct completion fmt_read_thread_start;
struct task_struct *raw_read_thread = NULL;
struct completion raw_read_thread_start;
struct task_struct *rfs_read_thread = NULL;
struct completion rfs_read_thread_start;
struct task_struct *cmd_read_thread = NULL;
struct completion cmd_read_thread_start;

#ifdef LOOP_BACK_TEST
struct task_struct *loopback_read_thread = NULL;
struct completion loopback_read_thread_start;
#endif

struct task_struct *write_cmd_thread = NULL;
struct completion write_cmd_thread_start;
struct task_struct *read_cmd_thread = NULL;
struct completion read_cmd_thread_start;

struct semaphore transfer_event_sem;
static int transfer_thread_waiting = 0;
static int cp_flow_control_stop_transfer = 0;
int cp_flashless_boot_done = 0;
EXPORT_SYMBOL( cp_flashless_boot_done );
static int cp_restart = 0;

volatile static void __iomem *p_virtual_buff = 0;

static unsigned long recv_cnt;
static unsigned long send_cnt;
static unsigned long nak_count = 0;
static ssize_t show_debug( struct device *d, struct device_attribute *attr, char *buf )
{
	char *p = buf;
	struct ipc_hsi *od = dev_get_drvdata( d );

	if( !od )
		return 0;

	p += sprintf( p, "Semaphore: %d (%d)\n", _read_sem( od ), ( char )hw_tmp );
	p += sprintf( p, "Mailbox: %x\n", od->reg->mailbox_AB );
	p += sprintf( p, "Reference count: %d\n", atomic_read( &od->ref_sem ) );
	p += sprintf( p, "Mailbox send: %lu\n", send_cnt );
	p += sprintf( p, "Mailbox recv: %lu\n", recv_cnt );

	p += sprintf( p, "NAK count: %lu\n", nak_count );

	return p - buf;
}

static DEVICE_ATTR(debug, S_IRUGO, show_debug, NULL);

static struct attribute *ipc_hsi_attributes[] = {
	&dev_attr_debug.attr,
	NULL
};

static const struct attribute_group ipc_hsi_group = {
	.attrs = ipc_hsi_attributes,
};

static inline void _write_sem( struct ipc_hsi *od, int v )
{
	//od->reg->sem = v;
	od->reg->sem = 1;
	
	hw_tmp = od->reg->sem; /* for hardware */
}

static inline int _read_sem( struct ipc_hsi *od )
{
	od->reg->sem = 1;
	
	return od->reg->sem;
}

extern void kernel_restart( char *cmd );
static void cp_ramdump_hsi( struct ipc_hsi *od, int order );
static inline int _send_cmd( struct ipc_hsi *od, u32 cmd )
{
	u32 tmp_cmd;
	char cmd_string[] = "CP Crash";
	
	if (!od) {
		printk(KERN_ERR "[%s]ipcspi: Dev is NULL, but try to access\n",__func__);
		return -EFAULT;
	}

	if (!od->reg) {
		dev_err( od->dev, "(%d) Failed to send cmd, not initialized\n", __LINE__ );
		
		return -EFAULT;
	}

//	dev_dbg( od->dev, "(%d) send %x\n", __LINE__, cmd );

	if( cmd == 0x45674567 ) {
		dev_dbg( od->dev, "(%d) modem image loaded.\n", __LINE__ );
		dev_dbg( od->dev, "(%d) start to send modem binary to cp.\n", __LINE__ );

		ipc_hsi_send_modem_work_data->od = od;
		//schedule_work( &ipc_hsi_send_modem_work_data->send_modem_w );
		if( !queue_work( ipc_hsi_wq, &ipc_hsi_send_modem_work_data->send_modem_w ) ) { // enqueue work			
			dev_err( od->dev, "(%d) already exist w-q\n", __LINE__ );		
		}

		return 0;
	}
	else if( cmd == 0x12340000 ) {
		printk( "[CP FLASHLESS BOOT] : CP POWER ON DONE.\n" );
		return 0;
	}
	else if( cmd == 0x12340001 ) {
		printk( "[CP FLASHLESS BOOT] : XMIT PSI DONE.\n" );
		return 0;
	}
	else if( cmd == 0x12340002 ) {
		printk( "[CP FLASHLESS BOOT] : GET PSI RUNNING ACK DONE.\n" );
		return 0;
	}
	else if( cmd == 0x12340003 ) {
		printk( "[CP FLASHLESS BOOT] : XMIT EBL DONE.\n" );
		return 0;
	}
	else if( cmd == 0x12340004 ) {
		printk( "[CP FLASHLESS BOOT] : SET EBL ADDR DONE.\n" );
		return 0;
	}
	else if( cmd == 0xDEAD0000 ) {
		cp_ramdump_hsi( od, 0 );
		return 0;
	}
	else if( cmd == 0xDEAD0101 ) {
		cp_ramdump_hsi( od, 1 );
		return 0;
	}
	else if( cmd == 0xDEAD0202 ) {
		cp_ramdump_hsi( od, 2 );
		return 0;
	}
	else if( cmd == 0xDEAD0303 ) {
		cp_ramdump_hsi( od, 3 );
		return 0;
	}
	else if( cmd == 0xDEAD0404 ) {
		cp_ramdump_hsi( od, 4 );
		return 0;
	}
	else if( cmd == 0xF0F0F0F0 ) {
		dev_err( od->dev, "(%d) CP RamDump Done. cmd : %x\n", __LINE__, cmd );
		panic( cmd_string );

		return 0;
	}

	if( cmd & MB_VALID ) {
		if( cmd & MB_COMMAND ) {
			tmp_cmd = ( cmd & MBC_MASK ) & ~( MB_CMD( 0 ) );
//			dev_dbg( od->dev, "(%d) command : %x\n", __LINE__, tmp_cmd );

			switch( tmp_cmd ) {
				case MBC_INIT_END : // 0x0002
					dev_info( od->dev, "(%d) send ril init cmd : %x\n", __LINE__, cmd );
					
					send_cnt++;
					od->reg->mailbox_BA = cmd;

					if( transfer_thread_waiting ) {
						//transfer_thread_waiting = 0;
						up( &transfer_event_sem );
					}

					complete_all( &tx_thread_start );
					
					break;
					
				default :
//					dev_dbg( od->dev, "(%d) ignore command ( 0x%x )\n", __LINE__, tmp_cmd );
					
					break;
			}
		}
		else {
			dev_dbg( od->dev, "(%d) =>send data ( 0x%x )\n", __LINE__, cmd );
			
			if( transfer_thread_waiting ) {
				//transfer_thread_waiting = 0;
				up( &transfer_event_sem );
			}
		}
	}
	else {
		dev_err( od->dev, "(%d) send invalid cmd : 0x%x\n", __LINE__, cmd );
	}
	
	return 0;
}

static inline int _recv_cmd( struct ipc_hsi *od, u32 *cmd )
{
	if( !cmd )
		return -EINVAL;

	if( !od ) {
		printk( KERN_ERR "[%s]ipcspi: Dev is NULL, but try to access\n",__func__ );
		return -EFAULT;
	}

	if( !od->reg ) {
		dev_err( od->dev, "(%d) Failed to read cmd, not initialized\n", __LINE__ );
		
		return -EFAULT;
	}

	recv_cnt++;
	*cmd = od->reg->mailbox_AB;
	
	return 0;
}

static inline int _get_auth( struct ipc_hsi *od, u32 cmd )
{
	unsigned long timeleft;
	int retry = 0;

	/* send cmd every 20m seconds */
	while( 1 ) {
		_send_cmd( od, cmd );

		timeleft = wait_for_completion_timeout( &od->comp, HZ/50 );
#if 0		
		if( timeleft )
			break;
#endif
		if( _read_sem( od ) )
			break;

		retry++;
		if( retry > 50 ) { /* time out after 1 seconds */
			dev_err( od->dev, "(%d) get authority time out\n", __LINE__ );
			
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int get_auth( struct ipc_hsi *od, u32 cmd )
{
	int r;

	if( !od ) {
		printk( KERN_ERR "[%s]ipcspi: Dev is NULL, but try to access\n",__func__ );
		return -EFAULT;
	}

	if( !od->reg ) {
		dev_err( od->dev, "(%d) Failed to get authority\n", __LINE__ );
		
		return -EFAULT;
	}

	//atomic_inc( &od->ref_sem );

	if( _read_sem( od ) )
		return 0;

	if( cmd )
		r = _get_auth( od, cmd );
	else
		r = -EACCES;

	//if( r < 0 )
	//	atomic_dec( &od->ref_sem );

	return r;
}

static int put_auth( struct ipc_hsi *od, int release )
{
	if( !od ) {
		printk( KERN_ERR "[%s]onedram: Dev is NULL, but try to access\n",__func__ );
		return -EFAULT;
	}

	if( !od->reg ) {
		dev_err( od->dev, "(%d) Failed to put authority\n", __LINE__ );
		
		return -EFAULT;
	}

	//if( release )
	//	set_bit( 0, &od->flags );

	//if( atomic_dec_and_test( &od->ref_sem ) && test_and_clear_bit( 0, &od->flags ) ) {
		//INIT_COMPLETION( od->comp );
		//_write_sem( od, 0 );
		//dev_dbg( od->dev, "rel_sem: %d\n", _read_sem( od ) );
	//}

	return 0;
}

static int rel_sem( struct ipc_hsi *od )
{
	if( !od ) {
		printk( KERN_ERR "[%s]onedram: Dev is NULL, but try to access\n",__func__ );
		return -EFAULT;
	}

	if( !od->reg ) {
		dev_err( od->dev, "(%d) Failed to put authority\n", __LINE__ );
		
		return -EFAULT;
	}

	//if( atomic_read( &od->ref_sem ) )
	//	return -EBUSY;

	//INIT_COMPLETION( od->comp );
	//clear_bit( 0, &od->flags );
	//_write_sem( od, 0 );
	//dev_dbg( od->dev, "rel_sem: %d\n", _read_sem( od ) );
	
	return 0;
}

int onedram_read_mailbox( u32 *mb )
{
	return _recv_cmd( ipc_hsi, mb );
}
EXPORT_SYMBOL( onedram_read_mailbox );

int onedram_write_mailbox( u32 mb )
{
	return _send_cmd( ipc_hsi, mb );
}
EXPORT_SYMBOL(onedram_write_mailbox);

void onedram_init_mailbox( void )
{
	int r = 0;
	/* flush mailbox before registering onedram irq */
	r = ipc_hsi->reg->mailbox_AB;

	/* Set nINT_ONEDRAM_CP to low */
	ipc_hsi->reg->mailbox_BA=0x0;
}
EXPORT_SYMBOL( onedram_init_mailbox );

int onedram_get_auth( u32 cmd )
{
	return get_auth( ipc_hsi, cmd );
}
EXPORT_SYMBOL( onedram_get_auth );

int onedram_put_auth( int release )
{
	return put_auth( ipc_hsi, release );
}
EXPORT_SYMBOL( onedram_put_auth );

int onedram_rel_sem( void )
{
	return rel_sem( ipc_hsi );
}
EXPORT_SYMBOL( onedram_rel_sem );

int onedram_read_sem( void )
{
	return _read_sem( ipc_hsi );
}
EXPORT_SYMBOL(onedram_read_sem);

void onedram_get_vbase( void** vbase )
{
	*vbase = (void*)ipc_hsi->mmio;
}
EXPORT_SYMBOL( onedram_get_vbase );

static void ipc_hsi_make_data_interrupt( struct ipc_hsi *od, u32 cmd )
{
	struct list_head *l;
	unsigned long flags;
	u32 mailbox;

	mailbox = cmd;
	dev_dbg( od->dev, "(%d) <=make data int : 0x%08x\n", __LINE__, mailbox );

	if( h_list.len ) {
		spin_lock_irqsave( &h_list.lock, flags );
		list_for_each( l, &h_list.list ) {
			struct ipc_hsi_handler *h = list_entry( l, struct ipc_hsi_handler, list );

			if( h->handler ) h->handler( mailbox, h->data );
		}
		spin_unlock_irqrestore( &h_list.lock, flags );

		spin_lock( &ipc_hsi_lock );
		od->mailbox = mailbox;
		spin_unlock( &ipc_hsi_lock );
	} else {
		od->mailbox = mailbox;
	}

	dev_dbg( od->dev, "(%d) <=send data int cmd event\n", __LINE__ );
	
	wake_up_interruptible( &od->waitq );
	kill_fasync( &od->async_queue, SIGIO, POLL_IN );
}


#if 0
static void ipc_hsi_vm_close( struct vm_area_struct *vma )
{
	struct ipc_hsi *od = vma->vm_private_data;
	unsigned long offset;
	unsigned long size;

	put_auth( od, 0 );

	offset = ( vma->vm_pgoff << PAGE_SHIFT ) - od->base;
	size = vma->vm_end - vma->vm_start;
	dev_dbg( od->dev, "Rel region: 0x%08lx 0x%08lx\n", offset, size );
	
	onedram_release_region( offset, size );
}
#endif

static int ipc_hsi_vm_pagefault( struct vm_area_struct *vma, struct vm_fault *vmf )
{
	struct ipc_hsi *od = vma->vm_private_data;
	
//	dev_dbg( od->dev, "(%d) ipc_hsi_vm_pagefault.\n", __LINE__ );

	vmf->page = vmalloc_to_page( od->mmio + ( vmf->pgoff << PAGE_SHIFT ) );
	get_page( vmf->page );
	
//	dev_dbg( od->dev, "(%d) ipc_hsi_vm_pagefault Done.\n", __LINE__ );
	
	return 0;
}

static struct vm_operations_struct ipc_hsi_vm_ops = {
	//.close = ipc_hsi_vm_close,
	.fault = ipc_hsi_vm_pagefault,
};

static int ipc_hsi_open( struct inode *inode, struct file *filp )
{
	struct cdev *cdev = inode->i_cdev;
	struct ipc_hsi *od = container_of( cdev, struct ipc_hsi, cdev );

	filp->private_data = od;
	return 0;
}

static int ipc_hsi_release( struct inode *inode, struct file *filp )
{
	filp->private_data = NULL;
	return 0;
}

static unsigned int ipc_hsi_poll( struct file *filp, poll_table *wait )
{
	struct ipc_hsi *od;
	u32 data;

	od = filp->private_data;

	poll_wait( filp, &od->waitq, wait );

	spin_lock_irq( &ipc_hsi_lock );
	data = od->mailbox;
	spin_unlock_irq( &ipc_hsi_lock );

	if( data )
		return POLLIN | POLLRDNORM;

	return 0;
}

static ssize_t ipc_hsi_read( struct file *filp, char __user *buf, size_t count, loff_t *ppos )
{
	DECLARE_WAITQUEUE( wait, current );
	u32 data;
	ssize_t retval;
	struct ipc_hsi *od;

	od = filp->private_data;

	if( count < sizeof( u32 ) )
		return -EINVAL;

	add_wait_queue( &od->waitq, &wait );

	while( 1 ) {
		set_current_state( TASK_INTERRUPTIBLE );

		spin_lock_irq( &ipc_hsi_lock );
		data = od->mailbox;
		od->mailbox = 0;
		spin_unlock_irq( &ipc_hsi_lock );

		if( data )
			break;
		else if( filp->f_flags & O_NONBLOCK ) {
			retval = -EAGAIN;
			goto out;
		}
		else if( signal_pending( current ) ) {
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	}

	retval = put_user( data, ( u32 __user * )buf );
	if( !retval )
		retval = sizeof( u32 );
	
out:
	__set_current_state( TASK_RUNNING );
	remove_wait_queue( &od->waitq, &wait );

	return retval;
}

static ssize_t ipc_hsi_write( struct file *filp, const char __user *buf, size_t count, loff_t *ppos )
{
	struct ipc_hsi *od;

	od = filp->private_data;

	if( count ) {
		u32 data;

		if( get_user( data, ( u32 __user * )buf ) )
			return -EFAULT;

		_send_cmd( od, data );
	}

	return count;
}

static int ipc_hsi_fasync(int fd, struct file *filp, int on)
{
	struct ipc_hsi *od;

	od = filp->private_data;

	return fasync_helper( fd, filp, on, &od->async_queue );
}

static int ipc_hsi_mmap( struct file *filp, struct vm_area_struct *vma )
{
	struct ipc_hsi *od;
	unsigned long size;
	unsigned long offset;

	od = filp->private_data;
	if( !od || !vma )
		return -EFAULT;

	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;

	dev_dbg( od->dev, "(%d) Req region: 0x%08lx 0x%08lx\n", __LINE__, offset, size );
	
	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &ipc_hsi_vm_ops;
	vma->vm_private_data = od;

	dev_dbg( od->dev, "(%d) ipc_hsi_mmap Done.\n", __LINE__ );
	
	return 0;
}

static int ipc_hsi_ioctl( struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg )
{
	struct cdev *cdev = inode->i_cdev;
	struct ipc_hsi *od = container_of( cdev, struct ipc_hsi, cdev );
	int r;

	switch( cmd ) {
		case ONEDRAM_GET_AUTH :
			r = get_auth( od, arg );
			break;
		case ONEDRAM_PUT_AUTH :
			r = put_auth( od, 0 );
			break;
		case ONEDRAM_REL_SEM:
			r = rel_sem( od );
			break;
		default:
			r = -ENOIOCTLCMD;
			break;
	}

	return r;
}

static const struct file_operations ipc_hsi_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = ipc_hsi_read,
	.write = ipc_hsi_write,
	.poll = ipc_hsi_poll,
	.fasync = ipc_hsi_fasync,
	.open = ipc_hsi_open,
	.release = ipc_hsi_release,
	.mmap = ipc_hsi_mmap,
	.ioctl = ipc_hsi_ioctl,
};

static int _register_chrdev( struct ipc_hsi *od )
{
	int r;
	dev_t devid;

	od->class = class_create( THIS_MODULE, DRVNAME );
	if( IS_ERR( od->class ) ) {
		r = PTR_ERR( od->class );
		od->class = NULL;
		return r;
	}

	r = alloc_chrdev_region( &devid, 0, 1, DRVNAME );
	if( r )
		return r;

	cdev_init( &od->cdev, &ipc_hsi_fops );

	r = cdev_add( &od->cdev, devid, 1 );
	if( r ) {
		unregister_chrdev_region( devid, 1 );
		return r;
	}
	od->devid = devid;

	od->dev = device_create( od->class, NULL, od->devid, od, DRVNAME );
	if( IS_ERR( od->dev ) ) {
		r = PTR_ERR( od->dev );
		od->dev = NULL;
		return r;
	}
	dev_set_drvdata( od->dev, od );

	return 0;
}

static inline int _request_mem( struct ipc_hsi *od, struct platform_device *pdev )
{
	od->mmio = ( void __iomem * )vmalloc( od->size );
	if( !od->mmio ) {
		dev_err( &pdev->dev, "(%d) Failed to vmalloc size : %lu\n", __LINE__, od->size );
		
		return -EBUSY;
	}
	else {
		dev_dbg( &pdev->dev, "(%d) vmalloc Done. mmio : 0x%08x\n", __LINE__, ( u32 )od->mmio );
	}

	memset( ( void * )od->mmio, 0, od->size );

	od->reg = (struct onedram_reg_mapped *)(
			(char *)od->mmio + ONEDRAM_REG_OFFSET);
	dev_dbg( &pdev->dev, "(%d) Onedram semaphore: %d\n", __LINE__, _read_sem( od ) );

	od->reg->sem = 1;
	dev_dbg( &pdev->dev, "(%d) force set sem to 1 : %d\n", __LINE__, od->reg->sem );

	od->reg->mailbox_AB = 0x000000C8; // Only for Latona2
	od->reg->mailbox_BA = 0;
	dev_dbg( &pdev->dev, "(%d) force set mailbox to 0 : AB=0x%x, BA=0x%x\n", __LINE__, od->reg->mailbox_AB, od->reg->mailbox_BA );
	
	ipc_hsi_resource.start = ( resource_size_t )od->mmio;
	ipc_hsi_resource.end = ( resource_size_t )od->mmio + od->size - 1;

	p_virtual_buff = od->mmio;

	return 0;
}

static void _release( struct ipc_hsi *od )
{
	if( !od )
		return;

	if( od->irq )
		free_irq( od->irq, od );

	if( od->group )
		sysfs_remove_group( &od->dev->kobj, od->group );

	if( od->dev )
		device_destroy( od->class, od->devid );

	if( od->devid ) {
		cdev_del( &od->cdev );
		unregister_chrdev_region( od->devid, 1 );
	}

	if( od->mmio ) {
		od->reg = NULL;
		
		vfree( ( void * )od->mmio );
		
		ipc_hsi_resource.start = 0;
		ipc_hsi_resource.end = -1;
	}

	if( od->class )
		class_destroy( od->class );

	kfree( od );
}

struct resource* onedram_request_region( resource_size_t start, resource_size_t n, const char *name )
{
	struct resource *res;

	start += ipc_hsi_resource.start;
	res = __request_region( &ipc_hsi_resource, start, n, name, 0 );
	if( !res )
		return NULL;

	return res;
}
EXPORT_SYMBOL( onedram_request_region );

void onedram_release_region( resource_size_t start, resource_size_t n )
{
	start += ipc_hsi_resource.start;
	__release_region( &ipc_hsi_resource, start, n );
}
EXPORT_SYMBOL( onedram_release_region );

int onedram_register_handler( void ( *handler )( u32, void * ), void *data )
{
	unsigned long flags;
	struct ipc_hsi_handler *hd;

	if( !handler )
		return -EINVAL;

	hd = kzalloc( sizeof( struct ipc_hsi_handler ), GFP_KERNEL );
	if( !hd )
		return -ENOMEM;

	hd->data = data;
	hd->handler = handler;

	spin_lock_irqsave( &h_list.lock, flags );
	list_add_tail( &hd->list, &h_list.list );
	h_list.len++;
	spin_unlock_irqrestore( &h_list.lock, flags );

	return 0;
}
EXPORT_SYMBOL( onedram_register_handler );

int onedram_unregister_handler( void ( *handler )( u32, void * ) )
{
	unsigned long flags;
	struct list_head *l, *tmp;

	if( !handler )
		return -EINVAL;

	spin_lock_irqsave( &h_list.lock, flags );
	list_for_each_safe( l, tmp, &h_list.list ) {
		struct ipc_hsi_handler *hd = list_entry( l, struct ipc_hsi_handler, list );

		if( hd->handler == handler ) {
			list_del( &hd->list );
			h_list.len--;
			kfree( hd );
		}
	}
	spin_unlock_irqrestore( &h_list.lock, flags );

	return 0;
}
EXPORT_SYMBOL( onedram_unregister_handler );

static void _unregister_all_handlers( void )
{
	unsigned long flags;
	struct list_head *l, *tmp;

	spin_lock_irqsave( &h_list.lock, flags );
	list_for_each_safe( l, tmp, &h_list.list ) {
		struct ipc_hsi_handler *hd = list_entry( l, struct ipc_hsi_handler, list );

		list_del( &hd->list );
		h_list.len--;
		kfree( hd );
	}
	spin_unlock_irqrestore( &h_list.lock, flags );
}

static void _init_data( struct ipc_hsi *od )
{
	init_completion( &od->comp );
	atomic_set( &od->ref_sem, 0 );
	INIT_LIST_HEAD( &h_list.list );
	spin_lock_init( &h_list.lock );
	h_list.len = 0;
	init_waitqueue_head( &od->waitq );
}


#define FMT_OUT	0x0FE000
#define FMT_IN		0x10E000
#define FMT_SZ		0x10000   /* 65536 bytes */

#define RAW_OUT	0x11E000
#define RAW_IN		0x21E000
#define RAW_SZ		0x100000 /* 1 MB */

#define RFS_OUT	0x31E000
#define RFS_IN		0x41E000
#define RFS_SZ		0x100000 /* 1 MB */

static u32 ipc_hsi_get_send_vbuff_command( struct ipc_hsi *od )
{
	u32 cmd = 0;

	memcpy( ( void * )&cmd, ( void * )( p_virtual_buff + ONEDRAM_REG_OFFSET + 64 ), sizeof( cmd ) ); // mailbox_BA
//	dev_dbg( od->dev, "(%d) get mailbox_BA cmd : 0x%x\n", __LINE__, cmd );
	
	return cmd;
}

static void ipc_hsi_set_send_vbuff_command_clear( struct ipc_hsi *od )
{
	u32 cmd = 0;

	memcpy( ( void * )( p_virtual_buff + ONEDRAM_REG_OFFSET + 64 ), ( void * )&cmd, sizeof( cmd ) ); // mailbox_BA
//	dev_dbg( od->dev, "(%d) clear mailbox_BA cmd : 0x%x\n", __LINE__, cmd );
}

static inline void ipc_hsi_get_pointer_of_vbuff_format_tx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 16 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 20 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get FMT tx_head : %d, tx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_get_pointer_of_vbuff_format_rx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 16 + 8 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 20 + 8 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get FMT rx_head : %d, rx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_get_pointer_of_vbuff_raw_tx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 32 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 36 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get RAW tx_head : %d, tx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_get_pointer_of_vbuff_raw_rx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 32 + 8 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 36 + 8 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get RAW rx_head : %d, rx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_get_pointer_of_vbuff_rfs_tx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 48 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 52 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get RFS tx_head : %d, tx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_get_pointer_of_vbuff_rfs_rx( struct ipc_hsi *od, u32 *head, u32 *tail )
{
	memcpy( ( void * )head, ( void * )( p_virtual_buff + 48 + 8 ), sizeof( *head ) );
	memcpy( ( void * )tail, ( void * )( p_virtual_buff + 52 + 8 ), sizeof( *tail ) );
//	dev_dbg( od->dev, "(%d) get RFS rx_head : %d, rx_tail : %d\n", __LINE__, *head, *tail );
}

static inline void ipc_hsi_update_tail_of_vbuff_format_tx( struct ipc_hsi *od, u32 u_tail )
{
	memcpy( ( void * )( p_virtual_buff + 20 ), ( void * )&u_tail, sizeof( u_tail ) );
//	dev_dbg( od->dev, "(%d) update FMT tx_tail : %d\n", __LINE__, u_tail );
}

static inline void ipc_hsi_update_head_of_vbuff_format_rx( struct ipc_hsi *od, u32 u_head )
{
	memcpy( ( void * )( p_virtual_buff + 20 + 4 ), ( void * )&u_head, sizeof( u_head ) );
//	dev_dbg( od->dev, "(%d) update FMT rx_head : %d\n", __LINE__, u_head );
}

static inline void ipc_hsi_update_tail_of_vbuff_raw_tx( struct ipc_hsi *od, u32 u_tail )
{
	memcpy( ( void * )( p_virtual_buff + 36 ), ( void * )&u_tail, sizeof( u_tail ) );
//	dev_dbg( od->dev, "(%d) update RAW tx_tail : %d\n", __LINE__, u_tail );
}

static inline void ipc_hsi_update_head_of_vbuff_raw_rx( struct ipc_hsi *od, u32 u_head )
{
	memcpy( ( void * )( p_virtual_buff + 36 + 4 ), ( void * )&u_head, sizeof( u_head ) );
//	dev_dbg( od->dev, "(%d) update RAW rx_head : %d\n", __LINE__, u_head );
}

static inline void ipc_hsi_update_tail_of_vbuff_rfs_tx( struct ipc_hsi *od, u32 u_tail )
{
	memcpy( ( void * )( p_virtual_buff + 52 ), ( void * )&u_tail, sizeof( u_tail ) );
//	dev_dbg( od->dev, "(%d) update RFS tx_tail : %d\n", __LINE__, u_tail );
}

static inline void ipc_hsi_update_head_of_vbuff_rfs_rx( struct ipc_hsi *od, u32 u_head )
{
	memcpy( ( void * )( p_virtual_buff + 52 + 4 ), ( void * )&u_head, sizeof( u_head ) );
//	dev_dbg( od->dev, "(%d) update RFS rx_head : %d\n", __LINE__, u_head );
}

static void ipc_hsi_clear_all_vbuff( struct ipc_hsi *od )
{
	u32 head, tail;

	ipc_hsi_get_pointer_of_vbuff_format_tx( od, &head, &tail );
	ipc_hsi_update_tail_of_vbuff_format_tx( od, head );
	dev_dbg( od->dev, "(%d) Remove Format TX.\n", __LINE__ );
	
	ipc_hsi_get_pointer_of_vbuff_format_rx( od, &head, &tail );
	ipc_hsi_update_head_of_vbuff_format_rx( od, tail );
	dev_dbg( od->dev, "(%d) Remove Format RX.\n", __LINE__ );

	ipc_hsi_get_pointer_of_vbuff_raw_tx( od, &head, &tail );
	ipc_hsi_update_tail_of_vbuff_raw_tx( od, head );
	dev_dbg( od->dev, "(%d) Remove Raw TX.\n", __LINE__ );
	
	ipc_hsi_get_pointer_of_vbuff_raw_rx( od, &head, &tail );
	ipc_hsi_update_head_of_vbuff_raw_rx( od, tail );
	dev_dbg( od->dev, "(%d) Remove Raw RX.\n", __LINE__ );

	ipc_hsi_get_pointer_of_vbuff_rfs_tx( od, &head, &tail );
	ipc_hsi_update_tail_of_vbuff_rfs_tx( od, head );
	dev_dbg( od->dev, "(%d) Remove Rfs TX.\n", __LINE__ );
	
	ipc_hsi_get_pointer_of_vbuff_rfs_rx( od, &head, &tail );
	ipc_hsi_update_head_of_vbuff_rfs_rx( od, tail );
	dev_dbg( od->dev, "(%d) Remove Rfs RX.\n", __LINE__ );

	dev_err( od->dev, "(%d) Remove all vbuff.\n", __LINE__ );
}

static u32 ipc_hsi_get_length_vbuff_format_tx( struct ipc_hsi *od, u32 *out_head, u32 *out_tail )
{
	u32 len = 0;

	ipc_hsi_get_pointer_of_vbuff_format_tx( od, out_head, out_tail );
	
	if( *out_head >= *out_tail ) {
		len = *out_head - *out_tail;
	}
	else {
		len = FMT_SZ - *out_tail + *out_head;
	}
//	dev_dbg( od->dev, "(%d) get FMT tx_len : %d\n", __LINE__, len );

	return len;
}

static u32 ipc_hsi_get_length_vbuff_raw_tx( struct ipc_hsi *od, u32 *out_head, u32 *out_tail )
{
	u32 len = 0;

	ipc_hsi_get_pointer_of_vbuff_raw_tx( od, out_head, out_tail );

	if( *out_head >= *out_tail ) {
		len = *out_head - *out_tail;
	}
	else {
		len = RAW_SZ - *out_tail + *out_head;
	}
//	dev_dbg( od->dev, "(%d) get RAW tx_len : %d\n", __LINE__, len );

	if( cp_flow_control_stop_transfer ) {
		dev_dbg( od->dev, "(%d) ipc_hsi_get_length_vbuff_raw_tx : CP Flow Control.\n", __LINE__ );

		return 0;
	}

	return len;
}

static u32 ipc_hsi_get_length_vbuff_rfs_tx( struct ipc_hsi *od, u32 *out_head, u32 *out_tail )
{
	u32 len = 0;

	ipc_hsi_get_pointer_of_vbuff_rfs_tx( od, out_head, out_tail );

	if( *out_head >= *out_tail ) {
		len = *out_head - *out_tail;
	}
	else {
		len = RFS_SZ - *out_tail + *out_head;
	}
//	dev_dbg( od->dev, "(%d) get RFS tx_len : %d\n", __LINE__, len );

	return len;
}

static u32 ipc_hsi_get_space_vbuff_format_rx( struct ipc_hsi *od, u32 *in_head, u32 *in_tail )
{
	u32 space = 0;
	
	ipc_hsi_get_pointer_of_vbuff_format_rx( od, in_head, in_tail );

	//check the memory space remained
	if( *in_tail <= *in_head ) {
		space = FMT_SZ - *in_head + *in_tail;
	}
	else{
		space = *in_tail - *in_head;
	}
//	dev_dbg( od->dev, "(%d) get FMT rx space : %d\n", __LINE__, space );

	return space;
}

static u32 ipc_hsi_get_space_vbuff_raw_rx( struct ipc_hsi *od, u32 *in_head, u32 *in_tail )
{
	u32 space = 0;
	
	ipc_hsi_get_pointer_of_vbuff_raw_rx( od, in_head, in_tail );

	//check the memory space remained
	if( *in_tail <= *in_head ) {
		space = RAW_SZ - *in_head + *in_tail;
	}
	else{
		space = *in_tail - *in_head;
	}
//	dev_dbg( od->dev, "(%d) get RAW rx space : %d\n", __LINE__, space );

	return space;
}

static u32 ipc_hsi_get_space_vbuff_rfs_rx( struct ipc_hsi *od, u32 *in_head, u32 *in_tail )
{
	u32 space = 0;
	
	ipc_hsi_get_pointer_of_vbuff_rfs_rx( od, in_head, in_tail );

	//check the memory space remained
	if( *in_tail <= *in_head ) {
		space = RFS_SZ - *in_head + *in_tail;
	}
	else{
		space = *in_tail - *in_head;
	}
//	dev_dbg( od->dev, "(%d) get RFS rx space : %d\n", __LINE__, space );

	return space;
}

static int ipc_hsi_check_send_data( struct ipc_hsi *od )
{
	int retval = 0;
	u32 out_head = 0;
	u32 out_tail = 0;

	if( ipc_hsi_get_send_vbuff_command( od ) ) {
//		dev_dbg( od->dev, "(%d) exist CMD data\n", __LINE__ );
		retval = 1;
	}

	ipc_hsi_get_pointer_of_vbuff_format_tx( od, &out_head, &out_tail );
	if( out_head != out_tail ) {
//		dev_dbg( od->dev, "(%d) exist FMT data\n", __LINE__ );
		retval = 1;
	}

	ipc_hsi_get_pointer_of_vbuff_raw_tx( od, &out_head, &out_tail );
	if( out_head != out_tail ) {
//		dev_dbg( od->dev, "(%d) exist RAW data\n", __LINE__ );
		if( cp_flow_control_stop_transfer ) {
			dev_dbg( od->dev, "(%d) ipc_hsi_check_send_data : CP Flow Control.\n", __LINE__ );
		}
		else {
			retval = 1;
		}
	}

	ipc_hsi_get_pointer_of_vbuff_rfs_tx( od, &out_head, &out_tail );
	if( out_head != out_tail ) {
//		dev_dbg( od->dev, "(%d) exist RFS data\n", __LINE__ );
		retval = 1;
	}

	return retval;
}

static void ipc_hsi_copy_from_vbuff_format_tx( struct ipc_hsi *od, void *p_des, u32 offset_vbuff, u32 copy_len )
{
	if( ( offset_vbuff + copy_len ) <= FMT_SZ ) {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + FMT_OUT + offset_vbuff ), copy_len );
	}
	else {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + FMT_OUT + offset_vbuff ), FMT_SZ - offset_vbuff );
		memcpy( ( void * )( p_des + ( FMT_SZ - offset_vbuff ) ), ( void * )( p_virtual_buff + FMT_OUT ), copy_len -( FMT_SZ - offset_vbuff ) );
	}
}

static void ipc_hsi_copy_from_vbuff_raw_tx( struct ipc_hsi *od, void *p_des, u32 offset_vbuff, u32 copy_len )
{
	if( ( offset_vbuff + copy_len ) <= RAW_SZ ) {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + RAW_OUT + offset_vbuff ), copy_len );
	}
	else {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + RAW_OUT + offset_vbuff ), RAW_SZ - offset_vbuff );
		memcpy( ( void * )( p_des + ( RAW_SZ - offset_vbuff ) ), ( void * )( p_virtual_buff + RAW_OUT ), copy_len -( RAW_SZ - offset_vbuff ) );
	}
}

static void ipc_hsi_copy_from_vbuff_rfs_tx( struct ipc_hsi *od, void *p_des, u32 offset_vbuff, u32 copy_len )
{
	if( ( offset_vbuff + copy_len ) <= RFS_SZ ) {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + RFS_OUT + offset_vbuff ), copy_len );
	}
	else {
		memcpy( ( void * )p_des, ( void * )( p_virtual_buff + RFS_OUT + offset_vbuff ), RFS_SZ - offset_vbuff );
		memcpy( ( void * )( p_des + ( RFS_SZ - offset_vbuff ) ), ( void * )( p_virtual_buff + RFS_OUT ), copy_len -( RFS_SZ - offset_vbuff ) );
	}
}

static int ipc_hsi_copy_to_vbuff_format_rx( struct ipc_hsi *od, void *data, u16 len )
{
	u32 head = 0, tail = 0, new_head = 0, space = 0;
	int copy_retry_count = 0;

	dev_dbg( od->dev, "(%d) <=copy data to FMT vbuff, len : %d\n", __LINE__, len );

COPY_TO_VBUFF_FMT_RETRY :
	
	space = ipc_hsi_get_space_vbuff_format_rx( od, &head, &tail );
	if( space < len ) {
		dev_err( od->dev, "(%d) FMT vbuff is full, space : %d, len : %d\n", __LINE__, space, len );

		copy_retry_count++;
		if( copy_retry_count > 20 ) {
			dev_err( od->dev, "(%d) FMT vbuff is full. copy fail.\n", __LINE__ );
			
			return -ENOMEM;
		}

		msleep( 5 );
		goto COPY_TO_VBUFF_FMT_RETRY;
	}

	if( head + len <= FMT_SZ ) {
		memcpy( ( void * )( p_virtual_buff + FMT_IN + head ), data, len );
	}
	else {
		memcpy( ( void * )( p_virtual_buff + FMT_IN + head ), data, FMT_SZ - head );
		memcpy( ( void * )( p_virtual_buff + FMT_IN ), ( void * )( data + ( FMT_SZ - head ) ), len - ( FMT_SZ - head ) );
	}

	new_head = ( head + len ) % FMT_SZ;
	ipc_hsi_update_head_of_vbuff_format_rx( od, new_head );

	dev_dbg( od->dev, "(%d) <=copy data to FMT vbuff done.\n", __LINE__ );

	return 0;
}

static int ipc_hsi_copy_to_vbuff_raw_rx( struct ipc_hsi *od, void *data, u32 len )
{
	u32 head = 0, tail = 0, new_head = 0, space = 0;
	int copy_retry_count = 0;

	dev_dbg( od->dev, "(%d) <=copy data to RAW vbuff, len : %d\n", __LINE__, len );

COPY_TO_VBUFF_RAW_RETRY :

	space = ipc_hsi_get_space_vbuff_raw_rx( od, &head, &tail );
	if( space < len ) {
		dev_err( od->dev, "(%d) RAW vbuff is full, space : %d, len : %d\n", __LINE__, space, len );

		copy_retry_count++;
		if( copy_retry_count > 20 ) {
			dev_err( od->dev, "(%d) RAW vbuff is full. copy fail.\n", __LINE__ );
			
			return -ENOMEM;
		}

		msleep( 5 );
		goto COPY_TO_VBUFF_RAW_RETRY;
	}

	if( head + len <= RAW_SZ ) {
		memcpy( ( void * )( p_virtual_buff + RAW_IN + head ), data, len );
	}
	else {
		memcpy( ( void * )( p_virtual_buff + RAW_IN + head ), data, RAW_SZ - head );
		memcpy( ( void * )( p_virtual_buff + RAW_IN ), ( void * )( data + ( RAW_SZ - head ) ), len - ( RAW_SZ - head ) );
	}

	new_head = ( head + len ) % RAW_SZ;
	ipc_hsi_update_head_of_vbuff_raw_rx( od, new_head );

	dev_dbg( od->dev, "(%d) <=copy data to RAW vbuff done.\n", __LINE__ );

	return 0;
}

static int ipc_hsi_copy_to_vbuff_rfs_rx( struct ipc_hsi *od, void *data, u32 len )
{
	u32 head = 0, tail = 0, new_head = 0, space = 0;
	int copy_retry_count = 0;

	dev_dbg( od->dev, "(%d) <=copy data to RFS vbuff, len : %d\n", __LINE__, len );

COPY_TO_VBUFF_RFS_RETRY :

	space = ipc_hsi_get_space_vbuff_rfs_rx( od, &head, &tail );
	if( space < len ) {
		dev_err( od->dev, "(%d) RFS vbuff is full, space : %d, len : %d\n", __LINE__, space, len );

		copy_retry_count++;
		if( copy_retry_count > 20 ) {
			dev_err( od->dev, "(%d) RFS vbuff is full. copy fail.\n", __LINE__ );
			
			return -ENOMEM;
		}

		msleep( 5 );
		goto COPY_TO_VBUFF_RFS_RETRY;
	}

	if( head + len <= RFS_SZ ) {
		memcpy( ( void * )( p_virtual_buff + RFS_IN + head ), data, len );
	}
	else {
		memcpy( ( void * )( p_virtual_buff + RFS_IN + head ), data, RFS_SZ - head );
		memcpy( ( void * )( p_virtual_buff + RFS_IN ), ( void * )( data + ( RFS_SZ - head ) ), len - ( RFS_SZ - head ) );
	}

	new_head = ( head + len ) % RFS_SZ;
	ipc_hsi_update_head_of_vbuff_rfs_rx( od, new_head );

	dev_dbg( od->dev, "(%d) <=copy data to RFS vbuff done.\n", __LINE__ );

	return 0;
}


#define IPC_HSI_TX_BUF_SIZE		( 6 * 1024 )
#define IPC_HSI_FMT_RX_BUF_SIZE		( 255 * 1024 )
#define IPC_HSI_FMT_RX_SAVE_BUF_SIZE		( 10 * 1024 )
#define IPC_HSI_RAW_RX_BUF_SIZE		( 255 * 1024 )
#define IPC_HSI_RAW_RX_SAVE_BUF_SIZE		( 10 * 1024 )
#define IPC_HSI_RFS_RX_BUF_SIZE		( 255 * 1024 )
#define IPC_HSI_RFS_RX_SAVE_BUF_SIZE		( 10 * 1024 )
#define IPC_HSI_CMD_RX_BUF_SIZE		( 255 * 1024 )
#define IPC_HSI_LOOPBACK_RX_BUF_SIZE		( 255 * 1024 )

static void ipc_hsi_prepare_tx_data( struct ipc_hsi *od, ipc_hsi_data *tx_data )
{
	u32 len = 0;
	u32 cmd = 0;
	u8 cmd_8 = 0;
	u32 p_send_data_h = 0, p_send_data_t = 0;
	u8 bof = 0, eof = 0;
	u32 packet_len = 0, copy_size = 0;
	u16 packet_len_16 = 0;
	int i;

	dev_dbg( od->dev, "(%d) =>ipc_hsi_prepare_tx_data\n", __LINE__ );

	i = 0;
	tx_data->channel = 0;
	tx_data->len = 0;
	tx_data->more = 0;

	cmd = ipc_hsi_get_send_vbuff_command( od );
	if( cmd ) {
		cmd_8 = cmd & 0xFF;
		dev_dbg( od->dev, "(%d) =>exist CMD tx cmd_8 : %x\n", __LINE__, cmd_8 );

		memcpy( ( void * )tx_data->buf, ( void * )&cmd_8, sizeof( cmd_8 ) );
		ipc_hsi_set_send_vbuff_command_clear( od );
		dev_dbg( od->dev, "(%d) =>copy CMD tx data. cmd_8 : %x\n", __LINE__, cmd_8 );

		tx_data->channel = HSI_CMD_CHANNEL;
		tx_data->len = 1;

		goto PREP_TX_DONE;
	}
	
	len = ipc_hsi_get_length_vbuff_format_tx( od, &p_send_data_h, &p_send_data_t );
	if( len ) {
		dev_dbg( od->dev, "(%d) =>exist FMT tx data. len : %d, in : %d, out : %d\n", __LINE__, len, p_send_data_t, p_send_data_h );

		if( len > IPC_HSI_TX_BUF_SIZE ) {
			dev_dbg( od->dev, "(%d) =>FMT tx data len is over. len : %d\n", __LINE__, len );

			while( copy_size < len ) {
				ipc_hsi_copy_from_vbuff_format_tx( od, ( void * )&packet_len_16, p_send_data_t + 1, sizeof( packet_len_16 ) );
				if( packet_len_16 + 2 > IPC_HSI_TX_BUF_SIZE - copy_size ) {
					dev_dbg( od->dev, "(%d) =>FMT packet_len_16 : %d, remain size : %d.\n", __LINE__, packet_len_16, len - copy_size );
					break;
				}
				dev_dbg( od->dev, "(%d) =>FMT packet size : %d.\n", __LINE__, packet_len_16 );

				ipc_hsi_copy_from_vbuff_format_tx( od, ( void * )&bof, p_send_data_t, sizeof( bof ) );
				if( bof != 0x7F ) {
					dev_err( od->dev, "(%d) =>FMT bof error. bof : %x\n", __LINE__, bof );

					ipc_hsi_update_tail_of_vbuff_format_tx( od, p_send_data_h );
					dev_err( od->dev, "(%d) =>discard FMT tx data.\n", __LINE__ );

					tx_data->channel = 0;
					tx_data->len = 0;

					goto PREP_TX_DONE;
				}

				ipc_hsi_copy_from_vbuff_format_tx( od, ( void * )( tx_data->buf + copy_size ), p_send_data_t, packet_len_16 + sizeof ( bof ) + sizeof ( eof ) );
				dev_dbg( od->dev, "(%d) =>copy FMT tx data. len : %d\n", __LINE__, packet_len_16 + sizeof ( bof ) + sizeof ( eof ) );

				copy_size += packet_len_16 + sizeof ( bof ) + sizeof ( eof );

				p_send_data_t += packet_len_16 + sizeof ( bof ) + sizeof ( eof );
				p_send_data_t %= FMT_SZ;
				ipc_hsi_update_tail_of_vbuff_format_tx( od, p_send_data_t );
				dev_dbg( od->dev, "(%d) =>update FMT tx data in pointer. in : %d\n", __LINE__, p_send_data_t );

				tx_data->channel = HSI_FMT_CHANNEL;
				tx_data->len = copy_size;
			}
		}
		else {
			ipc_hsi_copy_from_vbuff_format_tx( od, ( void * )tx_data->buf, p_send_data_t, len );
			dev_dbg( od->dev, "(%d) =>copy FMT tx data. len : %d\n", __LINE__, len );
			
			p_send_data_t += len;
			p_send_data_t %= FMT_SZ;
			ipc_hsi_update_tail_of_vbuff_format_tx( od, p_send_data_t );
			dev_dbg( od->dev, "(%d) =>update FMT tx data in pointer. in : %d, out : %d\n", __LINE__, p_send_data_t, p_send_data_h );

			tx_data->channel = HSI_FMT_CHANNEL;
			tx_data->len = len;
		}

#ifdef FORMAT_TX_DUMP
		printk( "[IPC_HSI => FMT TX :" );
		for( i = 0; i < ( ( tx_data->len > 10 )? 10 : tx_data->len ); i++ ) {
			printk( " %02x", *( ( u8 * )( tx_data->buf + i ) ) );
		}
		printk( "]\n" );
#endif // FORMAT_TX_DUMP

		goto PREP_TX_DONE;
	}

	len = ipc_hsi_get_length_vbuff_raw_tx( od, &p_send_data_h, &p_send_data_t );
	if( len ) {
		dev_dbg( od->dev, "(%d) =>exist RAW tx data\n", __LINE__ );

		if( len > IPC_HSI_TX_BUF_SIZE ) {
			dev_dbg( od->dev, "(%d) =>RAW tx data len is over. len : %d\n", __LINE__, len );

			while( copy_size < len ) {
				ipc_hsi_copy_from_vbuff_raw_tx( od, ( void * )&packet_len, p_send_data_t + 1, sizeof( packet_len ) );
				if( packet_len + 2 > IPC_HSI_TX_BUF_SIZE - copy_size ) {
					dev_dbg( od->dev, "(%d) =>RAW packet_len : %d, remain size : %d.\n", __LINE__, packet_len, len - copy_size );
					break;
				}
				dev_dbg( od->dev, "(%d) =>RAW packet size : %d.\n", __LINE__, packet_len );

				ipc_hsi_copy_from_vbuff_raw_tx( od, ( void * )&bof, p_send_data_t, sizeof( bof ) );
				if( bof != 0x7F ) {
					dev_err( od->dev, "(%d) =>RAW bof error. bof : %x\n", __LINE__, bof );

					ipc_hsi_update_tail_of_vbuff_raw_tx( od, p_send_data_h );
					dev_err( od->dev, "(%d) =>discard RAW tx data.\n", __LINE__ );

					tx_data->channel = 0;
					tx_data->len = 0;

					goto PREP_TX_DONE;
				}

				ipc_hsi_copy_from_vbuff_raw_tx( od, ( void * )( tx_data->buf + copy_size ), p_send_data_t, packet_len + sizeof ( bof ) + sizeof ( eof ) );
				dev_dbg( od->dev, "(%d) =>copy RAW tx data. len : %d\n", __LINE__, packet_len + sizeof ( bof ) + sizeof ( eof ) );

				copy_size += packet_len + sizeof ( bof ) + sizeof ( eof );

				p_send_data_t += packet_len + sizeof ( bof ) + sizeof ( eof );
				p_send_data_t %= RAW_SZ;
				ipc_hsi_update_tail_of_vbuff_raw_tx( od, p_send_data_t );
				dev_dbg( od->dev, "(%d) =>update RAW tx data in pointer. in : %d\n", __LINE__, p_send_data_t );

				tx_data->channel = HSI_RAW_CHANNEL;
				tx_data->len = copy_size;
			}
		}
		else {
			ipc_hsi_copy_from_vbuff_raw_tx( od, ( void * )tx_data->buf, p_send_data_t, len );
			dev_dbg( od->dev, "(%d) =>copy RAW tx data. len : %d\n", __LINE__, len );
			
			p_send_data_t += len;
			p_send_data_t %= RAW_SZ;
			ipc_hsi_update_tail_of_vbuff_raw_tx( od, p_send_data_t );
			dev_dbg( od->dev, "(%d) =>update RAW tx data in pointer. in : %d\n", __LINE__, p_send_data_t );

			tx_data->channel = HSI_RAW_CHANNEL;
			tx_data->len = len;
		}

#ifdef RAW_TX_DUMP
		printk( "[IPC_HSI => RAW TX :" );
		for( i = 0; i < ( ( tx_data->len > 20 )? 20 : tx_data->len ); i++ ) {
			printk( " %02x", *( ( u8 * )( tx_data->buf + i ) ) );
		}
		printk( "]\n" );
#endif // RAW_TX_DUMP

#ifdef RAW_TX_RX_LENGTH_DUMP
		printk( "[IPC_HSI => RAW TX : %d]\n", tx_data->len );
#endif // RAW_TX_RX_LENGTH_DUMP

		goto PREP_TX_DONE;
	}

	len = ipc_hsi_get_length_vbuff_rfs_tx( od, &p_send_data_h, &p_send_data_t );
	if( len ) {
		dev_dbg( od->dev, "(%d) =>exist RFS tx data\n", __LINE__ );

		if( len > IPC_HSI_TX_BUF_SIZE ) {
			dev_dbg( od->dev, "(%d) =>RFS tx data len is over. len : %d\n", __LINE__, len );

			while( copy_size < len ) {
				ipc_hsi_copy_from_vbuff_rfs_tx( od, ( void * )&packet_len, p_send_data_t + 1, sizeof( packet_len ) );
				if( packet_len + 2 > IPC_HSI_TX_BUF_SIZE - copy_size ) {
					dev_dbg( od->dev, "(%d) =>RFS packet_len : %d, remain size : %d.\n", __LINE__, packet_len, len - copy_size );
					break;
				}
				dev_dbg( od->dev, "(%d) =>RFS packet size : %d.\n", __LINE__, packet_len );

				ipc_hsi_copy_from_vbuff_rfs_tx( od, ( void * )&bof, p_send_data_t, sizeof( bof ) );
				if( bof != 0x7F ) {
					dev_err( od->dev, "(%d) =>RFS bof error. bof : %x\n", __LINE__, bof );

					ipc_hsi_update_tail_of_vbuff_rfs_tx( od, p_send_data_h );
					dev_err( od->dev, "(%d) =>discard RFS tx data.\n", __LINE__ );

					tx_data->channel = 0;
					tx_data->len = 0;

					goto PREP_TX_DONE;
				}

				ipc_hsi_copy_from_vbuff_rfs_tx( od, ( void * )( tx_data->buf + copy_size ), p_send_data_t, packet_len + sizeof ( bof ) + sizeof ( eof ) );
				dev_dbg( od->dev, "(%d) =>copy RFS tx data. len : %d\n", __LINE__, packet_len + sizeof ( bof ) + sizeof ( eof ) );

				copy_size += packet_len + sizeof ( bof ) + sizeof ( eof );

				p_send_data_t += packet_len + sizeof ( bof ) + sizeof ( eof );
				p_send_data_t %= RFS_SZ;
				ipc_hsi_update_tail_of_vbuff_rfs_tx( od, p_send_data_t );
				dev_dbg( od->dev, "(%d) =>update RFS tx data in pointer. in : %d\n", __LINE__, p_send_data_t );

				tx_data->channel = HSI_RFS_CHANNEL;
				tx_data->len = copy_size;
			}
		}
		else {
			ipc_hsi_copy_from_vbuff_rfs_tx( od, ( void * )tx_data->buf, p_send_data_t, len );
			dev_dbg( od->dev, "(%d) =>copy RFS tx data. len : %d\n", __LINE__, len );
			
			p_send_data_t += len;
			p_send_data_t %= RFS_SZ;
			ipc_hsi_update_tail_of_vbuff_rfs_tx( od, p_send_data_t );
			dev_dbg( od->dev, "(%d) =>update RFS tx data in pointer. in : %d\n", __LINE__, p_send_data_t );

			tx_data->channel = HSI_RFS_CHANNEL;
			tx_data->len = len;
		}

#ifdef RFS_TX_DUMP
		printk( "[IPC_HSI => RFS TX :" );
		for( i = 0; i < ( ( tx_data->len > 50 )? 50 : tx_data->len ); i++ ) {
			printk( " %02x", *( ( u8 * )( tx_data->buf + i ) ) );
		}
		printk( "]\n" );
#endif // RFS_TX_DUMP

#ifdef RFS_TX_RX_LENGTH_DUMP
		printk( "[IPC_HSI => RFS TX : %d]\n", tx_data->len );
#endif // RFS_TX_RX_LENGTH_DUMP

		goto PREP_TX_DONE;
	}

PREP_TX_DONE :

#ifdef ALL_TX_DUMP
	printk( "[IPC_HSI => ALL TX :" );
	for( i = 0; i < ( ( tx_data->len > 20 )? 20 : tx_data->len ); i++ ) {
		printk( " %02x", *( ( u8 * )( tx_data->buf + i ) ) );
	}
	printk( "]\n" );
#endif // ALL_TX_DUMP

	dev_dbg( od->dev, "(%d) =>ipc_hsi_prepare_tx_data done. ch : %d, len : %d\n", __LINE__, tx_data->channel, tx_data->len );
	
}

#ifdef LOOP_BACK_TEST

#ifdef LOOP_BACK_TEST_TX_ONLY
u8 tx_prep_count = 0;
#endif

#ifdef LOOP_BACK_TEST_RX_ONLY
u8 rx_prep_count = 0;
#endif


static void ipc_hsi_prepare_loopback_tx_data( struct ipc_hsi *od, ipc_hsi_data *tx_data )
{
//	int i;
	
	dev_dbg( od->dev, "(%d) =>ipc_hsi_prepare_loopback_tx_data\n", __LINE__ );

#ifdef LOOP_BACK_TEST_RX_ONLY
	if( !rx_prep_count ) {
		rx_prep_count = 1;

		tx_data->channel = HSI_LOOPBACK_CHANNEL;
		tx_data->len = 1;
		tx_data->buf[ 0 ] = 0xC2;
	}
	else {
		tx_data->channel = 0;
		tx_data->len = 0;
		tx_data->more = 0;
	}
#else

#ifdef LOOP_BACK_TEST_TX_ONLY
	tx_data->channel = 2;
#else
	tx_data->channel = HSI_LOOPBACK_CHANNEL;
#endif

	tx_data->len = 1558;
	tx_data->more = 0;

	tx_data->buf[ 0 ] = 0x7F;
	*( u32 * )( tx_data->buf + 1 ) = tx_data->len - 2;
	tx_data->buf[ 5 ] = 30;
	tx_data->buf[ 6 ] = 0;

#ifdef LOOP_BACK_TEST_TX_ONLY
	tx_prep_count++;
	if( tx_prep_count > 200 ) tx_prep_count = 0;
	*( u8 * )( tx_data->buf + 7 ) = tx_prep_count;
#else
	tx_data->buf[ 7 ] = 0;
#endif

	tx_data->buf[ tx_data->len - 1 ] = 0x7E;

#endif // LOOP_BACK_TEST_RX_ONLY

//	printk( "[IPC_HSI => LOOP TX :" );
//	for( i = 0; i < 5; i++ ) {
//		printk( " %02x", *( ( u8 * )( tx_data->buf + i ) ) );
//	}
//	printk( "]\n" );

	dev_dbg( od->dev, "(%d) =>ipc_hsi_prepare_loopback_tx_data done. ch : %d, len : %d\n", __LINE__, tx_data->channel, tx_data->len );
}
#endif // LOOP_BACK_TEST

u32 fmt_rx_save_buf_p = 0;
static void ipc_hsi_fmt_rx_process( struct ipc_hsi *od, ipc_hsi_data *rx_data, u8 *rx_save_buf )
{
	int retval = 0;
	u32 read_size = 0;
	u32 int_cmd = 0;
	u8 bof = 0, eof = 0;
	u16 packet_len = 0;
	int i;

	dev_dbg( od->dev, "(%d) <=ipc_hsi_fmt_rx_process. channel : %d, len : %d\n", __LINE__, rx_data->channel, rx_data->len );

	i = 0;

	if( rx_data->channel != 1 ) {
		dev_err( od->dev, "(%d) <=FMT Wrong channel : %d\n", __LINE__, rx_data->channel );
		return;
	}
	dev_dbg( od->dev, "(%d) <=FMT channel : %d\n", __LINE__, rx_data->channel );

	if( rx_data->len > IPC_HSI_FMT_RX_BUF_SIZE ) {
		dev_err( od->dev, "(%d) <=FMT Wrong len : %d\n", __LINE__, rx_data->len );
		return;
	}
	dev_dbg( od->dev, "(%d) <=FMT len : %d\n", __LINE__, rx_data->len );
	
	while( rx_data->len > read_size ) {
		dev_dbg( od->dev, "(%d) <=rx process, got FMT rx data.\n", __LINE__ );
		
		if( fmt_rx_save_buf_p ) {
			dev_dbg( od->dev, "(%d) <=FMT not complete packet. bof : 0x%x, eof : 0x%x\n", __LINE__, *( u8 * )rx_data->buf , *( u8 * )( rx_data->buf + rx_data->len - 1 ));

			if( fmt_rx_save_buf_p < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, FMT small saved size : %d\n", __LINE__, fmt_rx_save_buf_p );
				
				memcpy( ( void * )( rx_save_buf + fmt_rx_save_buf_p ), ( void * )rx_data->buf, sizeof( bof ) + sizeof( packet_len ) - fmt_rx_save_buf_p );
				dev_err( od->dev, "(%d) <=rx process, FMT small saved packet len.\n", __LINE__ );
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_save_buf + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_FMT_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );
				return;
			}

			memcpy( ( void * )( rx_save_buf + fmt_rx_save_buf_p ), ( void * )rx_data->buf, packet_len + sizeof( bof ) + sizeof( eof ) - fmt_rx_save_buf_p );

			memcpy( ( void * )&bof, ( void * )rx_save_buf, sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_save_buf + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_format_rx( od, ( void * )rx_save_buf, packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, FMT read Done. len : %d\n", __LINE__, rx_data->len );
			}

#ifdef FORMAT_RX_DUMP
			printk( "[IPC_HSI <= FMT RX(%d) :", packet_len );
			for( i = 0; i < ( ( packet_len > 10 )? 10 : packet_len ); i++ ) {
				printk( " %02x", *( ( u8 * )( rx_save_buf + i ) ) );
			}
			printk( "]\n" );
#endif // FORMAT_RX_DUMP

			read_size += packet_len + sizeof( bof ) + sizeof( eof ) - fmt_rx_save_buf_p;
			dev_dbg( od->dev, "(%d) <=rx process, FMT update read_size : %d\n", __LINE__, read_size );

			fmt_rx_save_buf_p = 0;
		}
		else {
			fmt_rx_save_buf_p = 0;
			
			if( rx_data->len - read_size < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT remain len : %d\n", __LINE__, rx_data->len - read_size );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				fmt_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_data->buf + read_size + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_FMT_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );
				return;
			}

			memcpy( ( void * )&bof, ( void * )( rx_data->buf + read_size ), sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			if( packet_len + sizeof( bof ) + sizeof( eof ) > rx_data->len - read_size ) {
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT remain len : %d, packet_len : %d\n", __LINE__, rx_data->len - read_size, packet_len );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				fmt_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_data->buf + read_size + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap FMT eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_format_rx( od, ( void * )( rx_data->buf + read_size ), packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap FMT memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, FMT read Done. len : %d\n", __LINE__, packet_len + sizeof( bof ) + sizeof( eof ) );
			}

#ifdef FORMAT_RX_DUMP
			printk( "[IPC_HSI <= FMT RX(%d) :", packet_len );
			for( i = 0; i < ( ( packet_len > 10 )? 10 : packet_len ); i++ ) {
				printk( " %02x", *( ( u8 * )( rx_data->buf + read_size + i ) ) );
			}
			printk( "]\n" );
#endif // FORMAT_RX_DUMP

			read_size += packet_len + sizeof( bof ) + sizeof( eof );
			dev_dbg( od->dev, "(%d) <=rx process, FMT update read_size : %d\n", __LINE__, read_size );
		}

		int_cmd = MB_DATA( MBD_SEND_FMT );
		ipc_hsi_make_data_interrupt( od, int_cmd );
		dev_dbg( od->dev, "(%d) <=rx process, FMT send event : 0x%x\n", __LINE__, int_cmd );

		dev_dbg( od->dev, "(%d) ===== rx process FMT loop. total_len : %d, read_size : %d\n", __LINE__, rx_data->len, read_size );
	}
}

u32 raw_rx_save_buf_p = 0;
static void ipc_hsi_raw_rx_process( struct ipc_hsi *od, ipc_hsi_data *rx_data, u8 *rx_save_buf )
{
	int retval = 0;
	u32 read_size = 0;
	u32 int_cmd = 0;
	u8 bof = 0, eof = 0;
	u32 packet_len = 0;
	int i;

	dev_dbg( od->dev, "(%d) <=ipc_hsi_raw_rx_process. channel : %d, len : %d\n", __LINE__, rx_data->channel, rx_data->len );

	 i =0;

	if( rx_data->channel != 2 ) {
		dev_err( od->dev, "(%d) <=RAW Wrong channel : %d\n", __LINE__, rx_data->channel );
		return;
	}
	dev_dbg( od->dev, "(%d) <=RAW channel : %d\n", __LINE__, rx_data->channel );

	if( rx_data->len > IPC_HSI_RAW_RX_BUF_SIZE ) {
		dev_err( od->dev, "(%d) <=RAW Wrong len : %d\n", __LINE__, rx_data->len );
		return;
	}
	dev_dbg( od->dev, "(%d) <=RAW len : %d\n", __LINE__, rx_data->len );
	
	while( rx_data->len > read_size ) {
		dev_dbg( od->dev, "(%d) <=rx process, got RAW rx data.\n", __LINE__ );

		if( raw_rx_save_buf_p ) {
			dev_dbg( od->dev, "(%d) <=RAW not complete packet. bof : 0x%x, eof : 0x%x\n", __LINE__, *( u8 * )rx_data->buf , *( u8 * )( rx_data->buf + rx_data->len - 1 ));

			if( raw_rx_save_buf_p < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, RAW small saved size : %d\n", __LINE__, raw_rx_save_buf_p );
				
				memcpy( ( void * )( rx_save_buf + raw_rx_save_buf_p ), ( void * )rx_data->buf, sizeof( bof ) + sizeof( packet_len ) - raw_rx_save_buf_p );
				dev_err( od->dev, "(%d) <=rx process, RAW small saved packet len.\n", __LINE__ );
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_save_buf + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_RAW_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );				
				return;
			}

			memcpy( ( void * )( rx_save_buf + raw_rx_save_buf_p ), ( void * )rx_data->buf, packet_len + sizeof( bof ) + sizeof( eof ) - raw_rx_save_buf_p );

			memcpy( ( void * )&bof, ( void * )rx_save_buf, sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_save_buf + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_raw_rx( od, ( void * )rx_save_buf, packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, RAW read Done. len : %d\n", __LINE__, rx_data->len );
			}

			read_size += packet_len + sizeof( bof ) + sizeof( eof ) - raw_rx_save_buf_p;
			dev_dbg( od->dev, "(%d) <=rx process, RAW update read_size : %d\n", __LINE__, read_size );

			raw_rx_save_buf_p = 0;

#ifdef RAW_RX_DUMP
			printk( "[IPC_HSI <= RAW RX :" );
			for( i = 0; i < ( ( packet_len > 20 )? 20 : packet_len ); i++ ) {
				printk( " %02x", *( ( u8 * )( rx_save_buf + i ) ) );
			}
			printk( "]\n" );
#endif // RAW_RX_DUMP
	
#ifdef RAW_TX_RX_LENGTH_DUMP
			printk( "[IPC_HSI <= RAW RX : %d]\n", packet_len );
#endif // RAW_TX_RX_LENGTH_DUMP

		}
		else {
			raw_rx_save_buf_p = 0;
			
			if( rx_data->len - read_size < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW remain len : %d\n", __LINE__, rx_data->len - read_size );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				raw_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_data->buf + read_size + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_RAW_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );				
				return;
			}

			memcpy( ( void * )&bof, ( void * )( rx_data->buf + read_size ), sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			if( packet_len + sizeof( bof ) + sizeof( eof ) > rx_data->len - read_size ) {
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW remain len : %d, packet_len : %d\n", __LINE__, rx_data->len - read_size, packet_len );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				raw_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_data->buf + read_size + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RAW eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_raw_rx( od, ( void * )( rx_data->buf + read_size ), packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RAW memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, RAW read Done. len : %d\n", __LINE__, packet_len + sizeof( bof ) + sizeof( eof ) );
			}

			read_size += packet_len + sizeof( bof ) + sizeof( eof );
			dev_dbg( od->dev, "(%d) <=rx process, RAW update read_size : %d\n", __LINE__, read_size );

#ifdef RAW_RX_DUMP
			printk( "[IPC_HSI <= RAW RX :" );
			for( i = 0; i < ( ( packet_len > 20 )? 20 : packet_len ); i++ ) {
				printk( " %02x", *( ( u8 * )( rx_data->buf + read_size + i ) ) );
			}
			printk( "]\n" );
#endif // RAW_RX_DUMP
		
#ifdef RAW_TX_RX_LENGTH_DUMP
				printk( "[IPC_HSI <= RAW RX : %d]\n", packet_len );
#endif // RAW_TX_RX_LENGTH_DUMP

		}

		int_cmd = MB_DATA( MBD_SEND_RAW );
		ipc_hsi_make_data_interrupt( od, int_cmd );
		dev_dbg( od->dev, "(%d) <=rx process, RAW send event : 0x%x\n", __LINE__, int_cmd );

		dev_dbg( od->dev, "(%d) ===== rx process RAW loop. total_len : %d, read_size : %d\n", __LINE__, rx_data->len, read_size );
	}
}

u32 rfs_rx_save_buf_p = 0;
static void ipc_hsi_rfs_rx_process( struct ipc_hsi *od, ipc_hsi_data *rx_data, u8 *rx_save_buf )
{
	int retval = 0;
	u32 read_size = 0;
	u32 int_cmd = 0;
	u8 bof = 0, eof = 0;
	u32 packet_len = 0;
	int i;

	dev_dbg( od->dev, "(%d) <=ipc_hsi_rfs_rx_process. channel : %d, len : %d\n", __LINE__, rx_data->channel, rx_data->len );

	i = 0;

	if( rx_data->channel != 3 ) {
		dev_err( od->dev, "(%d) <=RFS Wrong channel : %d\n", __LINE__, rx_data->channel );
		return;
	}
	dev_dbg( od->dev, "(%d) <=RFS channel : %d\n", __LINE__, rx_data->channel );

	if( rx_data->len > IPC_HSI_RFS_RX_BUF_SIZE ) {
		dev_err( od->dev, "(%d) <=RFS Wrong len : %d\n", __LINE__, rx_data->len );
		return;
	}
	dev_dbg( od->dev, "(%d) <=RFS len : %d\n", __LINE__, rx_data->len );
	
	while( rx_data->len > read_size ) {
		dev_dbg( od->dev, "(%d) <=rx process, got RFS rx data.\n", __LINE__ );
		
		if( rfs_rx_save_buf_p ) {
			dev_dbg( od->dev, "(%d) <=RFS not complete packet. bof : 0x%x, eof : 0x%x\n", __LINE__, *( u8 * )rx_data->buf , *( u8 * )( rx_data->buf + rx_data->len - 1 ));

			if( rfs_rx_save_buf_p < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, RFS small saved size : %d\n", __LINE__, rfs_rx_save_buf_p );
				
				memcpy( ( void * )( rx_save_buf + rfs_rx_save_buf_p ), ( void * )rx_data->buf, sizeof( bof ) + sizeof( packet_len ) - rfs_rx_save_buf_p );
				dev_err( od->dev, "(%d) <=rx process, RFS small saved packet len.\n", __LINE__ );
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_save_buf + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_RFS_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );
				return;
			}

			memcpy( ( void * )( rx_save_buf + rfs_rx_save_buf_p ), ( void * )rx_data->buf, packet_len + sizeof( bof ) + sizeof( eof ) - rfs_rx_save_buf_p );

			memcpy( ( void * )&bof, ( void * )rx_save_buf, sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_save_buf + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_rfs_rx( od, ( void * )rx_save_buf, packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, RFS read Done. len : %d\n", __LINE__, rx_data->len );
			}

			read_size += packet_len + sizeof( bof ) + sizeof( eof ) - rfs_rx_save_buf_p;
			dev_dbg( od->dev, "(%d) <=rx process, RFS update read_size : %d\n", __LINE__, read_size );

			rfs_rx_save_buf_p = 0;

#ifdef RFS_RX_DUMP
			printk( "[IPC_HSI <= RFS RX :" );
			for( i = 0; i < ( ( packet_len > 20 )? 20 : packet_len ); i++ ) {
				printk( " %02x", *( ( u8 * )( rx_save_buf + i ) ) );
			}
			printk( "]\n" );
#endif // RFS_RX_DUMP
	
#ifdef RFS_TX_RX_LENGTH_DUMP
			printk( "[IPC_HSI <= RFS RX : %d]\n", packet_len );
#endif // RFS_TX_RX_LENGTH_DUMP

		}
		else {
			rfs_rx_save_buf_p = 0;
			
			if( rx_data->len - read_size < sizeof( bof ) + sizeof( packet_len ) ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS remain len : %d\n", __LINE__, rx_data->len - read_size );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				rfs_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&packet_len, ( void * )( rx_data->buf + read_size + sizeof( bof ) ), sizeof( packet_len ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS packet_len : %d\n", __LINE__, packet_len );

			if( packet_len > IPC_HSI_RFS_RX_BUF_SIZE ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong Packet Len : %d, read_size : %d!!!.\n", __LINE__, packet_len, read_size );
				return;
			}

			memcpy( ( void * )&bof, ( void * )( rx_data->buf + read_size ), sizeof( bof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS bof : 0x%x\n", __LINE__, bof );

			if( bof != 0x7F ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong bof : 0x%x!!!.\n", __LINE__, bof );
				return;
			}

			if( packet_len + sizeof( bof ) + sizeof( eof ) > rx_data->len - read_size ) {
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS remain len : %d, packet_len : %d\n", __LINE__, rx_data->len - read_size, packet_len );

				memcpy( ( void * )rx_save_buf, ( void * )( rx_data->buf + read_size ), rx_data->len - read_size );
				dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS copy to save buf. len : %d\n", __LINE__, rx_data->len - read_size );

				rfs_rx_save_buf_p = rx_data->len - read_size;
				return;
			}

			memcpy( ( void * )&eof, ( void * )( rx_data->buf + read_size + sizeof( bof ) + packet_len ), sizeof( eof ) );
			dev_dbg( od->dev, "(%d) <=rx process, cp -> ap RFS eof : 0x%x\n", __LINE__, eof );

			if( eof != 0x7E ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS Wrong eof : 0x%x!!!.\n", __LINE__, eof );
				return;
			}

			retval = ipc_hsi_copy_to_vbuff_rfs_rx( od, ( void * )( rx_data->buf + read_size ), packet_len + sizeof( bof ) + sizeof( eof ) );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) <=rx process, cp -> ap RFS memory FULL!!!.\n", __LINE__ );
				return;
			}
			else {
				dev_dbg( od->dev, "(%d) <=rx process, RFS read Done. len : %d\n", __LINE__, packet_len + sizeof( bof ) + sizeof( eof ) );
			}

			read_size += packet_len + sizeof( bof ) + sizeof( eof );
			dev_dbg( od->dev, "(%d) <=rx process, RFS update read_size : %d\n", __LINE__, read_size );

#ifdef RFS_RX_DUMP
				printk( "[IPC_HSI <= RFS RX :" );
				for( i = 0; i < ( ( packet_len > 20 )? 20 : packet_len ); i++ ) {
					printk( " %02x", *( ( u8 * )( rx_data->buf + read_size + i ) ) );
				}
				printk( "]\n" );
#endif // RFS_RX_DUMP
		
#ifdef RFS_TX_RX_LENGTH_DUMP
				printk( "[IPC_HSI <= RFS RX : %d]\n", packet_len );
#endif // RFS_TX_RX_LENGTH_DUMP

		}

		int_cmd = MB_DATA( MBD_SEND_RFS );
		ipc_hsi_make_data_interrupt( od, int_cmd );
		dev_dbg( od->dev, "(%d) <=rx process, RFS send event : 0x%x\n", __LINE__, int_cmd );

		dev_dbg( od->dev, "(%d) ===== rx process RFS loop. total_len : %d, read_size : %d\n", __LINE__, rx_data->len, read_size );
	}
}

static void ipc_hsi_cmd_rx_process( struct ipc_hsi *od, ipc_hsi_data *rx_data )
{
	u32 read_size = 0;
	u16 cmd_16 = 0;

	dev_dbg( od->dev, "(%d) <=ipc_hsi_cmd_rx_process. channel : %d, len : %d\n", __LINE__, rx_data->channel, rx_data->len );

	if( rx_data->channel != 4 ) {
		dev_err( od->dev, "(%d) <=CMD Wrong channel : %d\n", __LINE__, rx_data->channel );
		return;
	}
	dev_dbg( od->dev, "(%d) <=CMD channel : %d\n", __LINE__, rx_data->channel );

	if( rx_data->len > IPC_HSI_CMD_RX_BUF_SIZE ) {
		dev_err( od->dev, "(%d) <=CMD Wrong len : %d\n", __LINE__, rx_data->len );
		return;
	}
	dev_dbg( od->dev, "(%d) <=CMD len : %d\n", __LINE__, rx_data->len );
	
	while( rx_data->len > read_size ) {
		dev_dbg( od->dev, "(%d) <=rx process, got CMD rx data.\n", __LINE__ );

		memcpy( ( void * )&cmd_16, ( void * )rx_data->buf, sizeof( cmd_16 ) );
		dev_dbg( od->dev, "(%d) <=rx process, CMD read Done. cmd : 0x%x, len : %d\n", __LINE__, cmd_16, rx_data->len );

		if( ( cmd_16 & 0xF ) == 0xA ) { // ONEDRAM_CMD_SUSPEND_REQ
			printk( "[IPC_HSI] Got ONEDRAM_CMD_SUSPEND_REQ\n" );
			cp_flow_control_stop_transfer = 1;
		}
		else if( ( cmd_16 & 0xF ) == 0xB ) { // ONEDRAM_CMD_RESUME_REQ
			printk( "[IPC_HSI] Got ONEDRAM_CMD_RESUME_REQ\n" );
			cp_flow_control_stop_transfer = 0;
		}

		read_size += rx_data->len;
		dev_dbg( od->dev, "(%d) <=rx process, CMD update read_size : %d\n", __LINE__, read_size );

		dev_dbg( od->dev, "(%d) ===== rx process CMD loop. total_len : %d, read_size : %d\n", __LINE__, rx_data->len, read_size );
	}
}

#ifdef LOOP_BACK_TEST
static void ipc_hsi_loopback_rx_process( struct ipc_hsi *od, ipc_hsi_data *rx_data )
{
	u32 read_size = 0;
//	int i;

	dev_dbg( od->dev, "(%d) <=ipc_hsi_loopback_rx_process. channel : %d, len : %d\n", __LINE__, rx_data->channel, rx_data->len );

	if( rx_data->channel != 4 ) {
		dev_err( od->dev, "(%d) <=LOOP Wrong channel : %d\n", __LINE__, rx_data->channel );
		return;
	}
	dev_dbg( od->dev, "(%d) <=LOOP channel : %d\n", __LINE__, rx_data->channel );

	if( rx_data->len > IPC_HSI_LOOPBACK_RX_BUF_SIZE ) {
		dev_err( od->dev, "(%d) <=LOOP Wrong len : %d\n", __LINE__, rx_data->len );
		return;
	}
	dev_dbg( od->dev, "(%d) <=LOOP len : %d\n", __LINE__, rx_data->len );
	
	while( rx_data->len > read_size ) {

//		printk( "[IPC_HSI => LOOP RX :" );
//		for( i = 0; i < 5; i++ ) {
//			printk( " %02x", *( ( u8 * )( rx_data->buf + i ) ) );
//		}
//		printk( "]\n" );

		read_size += rx_data->len;
		dev_dbg( od->dev, "(%d) <=rx process, LOOPBACK update read_size : %d\n", __LINE__, read_size );

		dev_dbg( od->dev, "(%d) ===== rx process LOOPBACK loop. total_len : %d, read_size : %d\n", __LINE__, rx_data->len, read_size );
	}
}
#endif // LOOP_BACK_TEST

static int ipc_hsi_fmt_rx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;

	ipc_hsi_data *rx_data = NULL;
	u8 *rx_save_buf = NULL;

	printk( "(%d) ipc_hsi_fmt_rx_thread : start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) ipc_hsi_fmt_rx_thread : p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	rx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !rx_data ) {
		dev_err( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_data kmalloc done.\n", __LINE__ );

	rx_data->buf = kmalloc( IPC_HSI_FMT_RX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_data->buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data->buf, 0, IPC_HSI_FMT_RX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_data->buf vmalloc done.\n", __LINE__ );

	rx_save_buf = kmalloc( IPC_HSI_FMT_RX_SAVE_BUF_SIZE, GFP_ATOMIC );
	if( !rx_save_buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_save_buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_save_buf, 0, IPC_HSI_FMT_RX_SAVE_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : rx_save_buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait fmt_read_thread_start complete.\n", __func__ );
	wait_for_completion( &fmt_read_thread_start );
	printk( "[%s] fmt_read_thread_start completed.\n", __func__ );
	
	while( !kthread_should_stop() ) {
		rx_data->channel = HSI_FMT_CHANNEL;
		rx_data->len = 0;
		rx_data->more = 0;
		
		dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : hsi_protocol_read start\n", __LINE__ );
		retval = hsi_protocol_read( rx_data->channel, ( u32 * )rx_data->buf, &rx_data->len );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) fmt_read_thread CP Restart.\n", __LINE__ );
			
			init_completion( &fmt_read_thread_start );
			goto SILENT_RESET;
		}
		
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_fmt_rx_thread : hsi_protocol_read error. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
		else {
			dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : hsi_protocol_read done. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );

			ipc_hsi_fmt_rx_process( od, rx_data, rx_save_buf );
		}

		dev_dbg( od->dev, "(%d) ipc_hsi_fmt_rx_thread : check rx more : %d\n", __LINE__, rx_data->more );
		dev_dbg( od->dev, "(%d) ===== ipc_hsi_fmt_rx_thread rx loop =====\n", __LINE__ );
	}

exit :
	printk( "(%d) ipc_hsi_fmt_rx_thread : thread stop.\n", __LINE__ );

	return retval;
}

static int ipc_hsi_raw_rx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;

	ipc_hsi_data *rx_data = NULL;
	u8 *rx_save_buf = NULL;

	printk( "(%d) ipc_hsi_raw_rx_thread : start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) ipc_hsi_raw_rx_thread : p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	rx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !rx_data ) {
		dev_err( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_data kmalloc done.\n", __LINE__ );

	rx_data->buf = kmalloc( IPC_HSI_RAW_RX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_data->buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data->buf, 0, IPC_HSI_RAW_RX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_data->buf vmalloc done.\n", __LINE__ );

	rx_save_buf = kmalloc( IPC_HSI_RAW_RX_SAVE_BUF_SIZE, GFP_ATOMIC );
	if( !rx_save_buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_save_buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_save_buf, 0, IPC_HSI_RAW_RX_SAVE_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : rx_save_buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait raw_read_thread_start complete.\n", __func__ );
	wait_for_completion( &raw_read_thread_start );
	printk( "[%s] raw_read_thread_start completed.\n", __func__ );
	
	while( 1 ) {
		rx_data->channel = HSI_RAW_CHANNEL;
		rx_data->len = 0;
		rx_data->more = 0;
		
		dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : hsi_protocol_read start\n", __LINE__ );
		retval = hsi_protocol_read( rx_data->channel, ( u32 * )rx_data->buf, &rx_data->len );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) raw_read_thread CP Restart.\n", __LINE__ );
			
			init_completion( &raw_read_thread_start );
			goto SILENT_RESET;
		}
		
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_raw_rx_thread : hsi_protocol_read error. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
		else {
			dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : hsi_protocol_read done. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );

			ipc_hsi_raw_rx_process( od, rx_data, rx_save_buf );
		}

		dev_dbg( od->dev, "(%d) ipc_hsi_raw_rx_thread : check rx more : %d\n", __LINE__, rx_data->more );
		dev_dbg( od->dev, "(%d) ===== ipc_hsi_raw_rx_thread rx loop =====\n", __LINE__ );
	}

exit :
	printk( "(%d) ipc_hsi_raw_rx_thread : thread stop.\n", __LINE__ );

	return retval;
}

static int ipc_hsi_rfs_rx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;

	ipc_hsi_data *rx_data = NULL;
	u8 *rx_save_buf = NULL;
	
	printk( "(%d) ipc_hsi_rfs_rx_thread : start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) ipc_hsi_rfs_rx_thread : p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	rx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !rx_data ) {
		dev_err( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_data kmalloc done.\n", __LINE__ );

	rx_data->buf = kmalloc( IPC_HSI_RFS_RX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_data->buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data->buf, 0, IPC_HSI_RFS_RX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_data->buf vmalloc done.\n", __LINE__ );
	
	rx_save_buf = kmalloc( IPC_HSI_RFS_RX_SAVE_BUF_SIZE, GFP_ATOMIC );
	if( !rx_save_buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_save_buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_save_buf, 0, IPC_HSI_RFS_RX_SAVE_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : rx_save_buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait rfs_read_thread_start complete.\n", __func__ );
	wait_for_completion( &rfs_read_thread_start );
	printk( "[%s] rfs_read_thread_start completed.\n", __func__ );
	
	while( !kthread_should_stop() ) {
		rx_data->channel = HSI_RFS_CHANNEL;
		rx_data->len = 0;
		rx_data->more = 0;
		
		dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : hsi_protocol_read start\n", __LINE__ );
		retval = hsi_protocol_read( rx_data->channel, ( u32 * )rx_data->buf, &rx_data->len );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) rfs_read_thread_start CP Restart.\n", __LINE__ );
			
			init_completion( &rfs_read_thread_start );
			goto SILENT_RESET;
		}
		
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_rfs_rx_thread : hsi_protocol_read error. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
		else {
			dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : hsi_protocol_read done. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );

			ipc_hsi_rfs_rx_process( od, rx_data, rx_save_buf );
		}

		dev_dbg( od->dev, "(%d) ipc_hsi_rfs_rx_thread : check rx more : %d\n", __LINE__, rx_data->more );
		dev_dbg( od->dev, "(%d) ===== ipc_hsi_rfs_rx_thread rx loop =====\n", __LINE__ );
	}

exit :
	printk( "(%d) ipc_hsi_rfs_rx_thread : thread stop.\n", __LINE__ );

	return retval;
}

static int ipc_hsi_cmd_rx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;

	ipc_hsi_data *rx_data = NULL;

	printk( "(%d) ipc_hsi_cmd_rx_thread : start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) ipc_hsi_cmd_rx_thread : p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	rx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !rx_data ) {
		dev_err( od->dev, "(%d) ipc_hsi_cmd_rx_thread : rx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) ipc_hsi_cmd_rx_thread : rx_data kmalloc done.\n", __LINE__ );

	rx_data->buf = kmalloc( IPC_HSI_CMD_RX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_data->buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_cmd_rx_thread : rx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data->buf, 0, IPC_HSI_CMD_RX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_cmd_rx_thread : rx_data->buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait cmd_read_thread_start complete.\n", __func__ );
	wait_for_completion( &cmd_read_thread_start );
	printk( "[%s] cmd_read_thread_start completed.\n", __func__ );

	while( !kthread_should_stop() ) {
		rx_data->channel = HSI_CMD_CHANNEL;
		rx_data->len = 0;
		rx_data->more = 0;
		
		dev_dbg( od->dev, "(%d) ipc_hsi_cmd_rx_thread : hsi_protocol_read start\n", __LINE__ );
		retval = hsi_protocol_read( rx_data->channel, ( u32 * )rx_data->buf, &rx_data->len );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) cmd_read_thread_start CP Restart.\n", __LINE__ );
			
			init_completion( &cmd_read_thread_start );
			goto SILENT_RESET;
		}
		
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_cmd_rx_thread : hsi_protocol_read error. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
		else {
			dev_dbg( od->dev, "(%d) ipc_hsi_cmd_rx_thread : hsi_protocol_read done. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );

			ipc_hsi_cmd_rx_process( od, rx_data );
		}
		
		dev_dbg( od->dev, "(%d) ipc_hsi_cmd_rx_thread : check rx more : %d\n", __LINE__, rx_data->more );
		dev_dbg( od->dev, "(%d) ===== ipc_hsi_cmd_rx_thread rx loop =====\n", __LINE__ );
	}

exit :
	printk( "(%d) ipc_hsi_cmd_rx_thread : thread stop.\n", __LINE__ );

	return retval;
}

#ifdef LOOP_BACK_TEST
static int ipc_hsi_loopback_rx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;
	struct timespec time_check_before, time_check_now;
	u32 total_read_len = 0;

#ifdef LOOP_BACK_TEST_RX_ONLY
	int i = 0, rx_count = 0;
	struct timespec time_check_start, time_check_end;
#endif

	ipc_hsi_data *rx_data = NULL;

	printk( "(%d) ipc_hsi_loopback_rx_thread : start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) ipc_hsi_loopback_rx_thread : p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	rx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !rx_data ) {
		dev_err( od->dev, "(%d) ipc_hsi_loopback_rx_thread : rx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : rx_data kmalloc done.\n", __LINE__ );

	rx_data->buf = kmalloc( IPC_HSI_LOOPBACK_RX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_data->buf ) {
		dev_err( od->dev, "(%d) ipc_hsi_loopback_rx_thread : rx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )rx_data->buf, 0, IPC_HSI_LOOPBACK_RX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : rx_data->buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait loopback_read_thread_start complete.\n", __func__ );
	wait_for_completion( &loopback_read_thread_start );
	printk( "[%s] loopback_read_thread_start completed.\n", __func__ );

	time_check_before = CURRENT_TIME;
	time_check_now = CURRENT_TIME;

	while( !kthread_should_stop() ) {
		
#ifdef LOOP_BACK_TEST_RX_ONLY
		rx_data->channel = 2;
#else
		rx_data->channel = HSI_LOOPBACK_CHANNEL;
#endif

		rx_data->len = 0;
		rx_data->more = 0;
		
		dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : hsi_protocol_read start\n", __LINE__ );
		retval = hsi_protocol_read( rx_data->channel, ( u32 * )rx_data->buf, &rx_data->len );
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_loopback_rx_thread : hsi_protocol_read error. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
		else {
			dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : hsi_protocol_read done. len : %d, ret : %d\n", __LINE__, rx_data->len, retval );
		}
//		up( &transfer_event_sem );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) loopback_read_thread_start CP Restart.\n", __LINE__ );
			
			init_completion( &loopback_read_thread_start );
			goto SILENT_RESET;
		}

#if !( defined( LOOP_BACK_TEST_TX_ONLY ) || defined( LOOP_BACK_TEST_RX_ONLY ) )
		ipc_hsi_loopback_rx_process( od, rx_data );
		dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : ipc_hsi_loopback_rx_process done. len : %d\n", __LINE__, rx_data->len );

		total_read_len += rx_data->len;
		time_check_now = CURRENT_TIME;
		if( time_check_now.tv_sec - time_check_before.tv_sec ) {
			time_check_before = CURRENT_TIME;
			
			printk( "[SPEED RX] %d\n", total_read_len );

			total_read_len = 0;
		}
#endif

#ifdef LOOP_BACK_TEST_RX_ONLY
		printk( "(%d) %02x %02x %02x\n", 
			rx_data->len, *( ( u8 * )( rx_data->buf + rx_data->len -3 ) ), *( ( u8 * )( rx_data->buf + rx_data->len -2 ) ), *( ( u8 * )( rx_data->buf + rx_data->len -1 ) ) );

		if( rx_count == 0 ){
			time_check_start = CURRENT_TIME;
			total_read_len = 0;
			rx_count = 1;
		}

		total_read_len += rx_data->len;

		if( total_read_len >= ( 1558 * 198 ) ){
			time_check_end = CURRENT_TIME;
			
			printk( "200 packet RX Done. (%d) => init time : %lu.%09lu, end time : %lu.%09lu\n", 
						rx_count, time_check_start.tv_sec, time_check_start.tv_nsec, time_check_end.tv_sec, time_check_end.tv_nsec );
			printk( "===================\n" );
		}
#endif

		dev_dbg( od->dev, "(%d) ipc_hsi_loopback_rx_thread : check rx more : %d\n", __LINE__, rx_data->more );
		dev_dbg( od->dev, "(%d) ===== ipc_hsi_loopback_rx_thread rx loop =====\n", __LINE__ );
	}

exit :
	printk( "(%d) ipc_hsi_loopback_rx_thread : thread stop.\n", __LINE__ );

	return retval;
}
#endif // LOOP_BACK_TEST

static int ipc_hsi_tx_thread( void *data )
{
	int retval = 0;
	struct ipc_hsi *od = ( struct ipc_hsi * )data;

#ifdef LOOP_BACK_TEST
#ifdef LOOP_BACK_TEST_TX_ONLY
	int tx_count = 0;
	struct timespec init_time, end_time;
#endif
#endif // LOOP_BACK_TEST

	ipc_hsi_data *tx_data = NULL;

	printk( "(%d) ipc_hsi_tx_thread start.\n", __LINE__ );

	if( !p_virtual_buff ) {
		dev_err( od->dev, "(%d) p_virtual_buff is NULL.\n", __LINE__);

		retval = -ENODEV;
		goto exit;
	}

	tx_data = kmalloc( sizeof( ipc_hsi_data ), GFP_ATOMIC );
	if( !tx_data ) {
		dev_err( od->dev, "(%d) tx_data kmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )tx_data, 0, sizeof( ipc_hsi_data ) );
	dev_dbg( od->dev, "(%d) tx_data kmalloc done.\n", __LINE__ );

	tx_data->buf = kmalloc( IPC_HSI_TX_BUF_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !tx_data->buf ) {
		dev_err( od->dev, "(%d) tx_data->buf vmalloc fail.", __LINE__ );
		retval = -ENOMEM;
		goto exit;
	}
	memset( ( void * )tx_data->buf, 0, IPC_HSI_TX_BUF_SIZE );
	dev_dbg( od->dev, "(%d) tx_data->buf vmalloc done.\n", __LINE__ );

SILENT_RESET :

	printk( "[%s] wait tx_thread_start complete.\n", __func__ );
	wait_for_completion( &tx_thread_start );
	printk( "[%s] tx_thread_start completed.\n", __func__ );

	cp_restart = 0;

	retval = hsi_init_handshake( HSI_INIT_MODE_NORMAL );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) hsi_init_handshake fail :%d\n", __LINE__, retval );
		goto exit;
	}
	dev_dbg( od->dev, "(%d) hsi_init_handshake Done :%d\n", __LINE__, retval );

	sema_init( &transfer_event_sem, 0 );

	complete_all( &fmt_read_thread_start );
	complete_all( &raw_read_thread_start );
	complete_all( &rfs_read_thread_start );
	complete_all( &cmd_read_thread_start );
	
#ifdef LOOP_BACK_TEST
	complete_all( &loopback_read_thread_start );
#endif

	dev_err( od->dev, "(%d) wait 2 sec...\n", __LINE__ );
	msleep( 2000 );
	dev_err( od->dev, "(%d) Start TX Thread.\n", __LINE__ );
	
	while( !kthread_should_stop() ) {

#ifndef LOOP_BACK_TEST
		if( !ipc_hsi_check_send_data( od ) ) {
			transfer_thread_waiting = 1;

			dev_dbg( od->dev, "(%d) no data and no sem, wait tx event.\n", __LINE__ );

			down( &transfer_event_sem );
			transfer_thread_waiting = 0;

			dev_dbg( od->dev, "(%d) got tx event.\n", __LINE__ );
		}
		else {
			dev_dbg( od->dev, "(%d) send data exist\n", __LINE__ );
		}
#endif // LOOP_BACK_TEST

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) tx_thread CP Restart.\n", __LINE__ );
			
			init_completion( &tx_thread_start );
			goto SILENT_RESET;
		}

#ifdef LOOP_BACK_TEST
#ifdef LOOP_BACK_TEST_TX_ONLY
		if( !tx_count ) {
			//printk( "200 packet TX Start. (%d)\n", tx_count );

			init_time = CURRENT_TIME;
		}
#endif
		ipc_hsi_prepare_loopback_tx_data( od, tx_data );
#else
		ipc_hsi_prepare_tx_data( od, tx_data );
#endif // LOOP_BACK_TEST

		if( !tx_data->len || !tx_data->channel ) {
			dev_dbg( od->dev, "(%d) ZERO len : %d, channel : %d\n", __LINE__, tx_data->len, tx_data->channel );
		}
		else {
			dev_dbg( od->dev, "(%d) start hsi_protocol_write. len : %d\n", __LINE__, tx_data->len );
			
			retval = hsi_protocol_write( tx_data->channel, ( u32 * )tx_data->buf, tx_data->len );
			if( retval < 0 ) {
				dev_err( od->dev, "(%d) start hsi_protocol_write error : %d\n", __LINE__, retval );
			}
			else {
				dev_dbg( od->dev, "(%d) start hsi_protocol_write Done : %d\n", __LINE__, retval );

#ifdef LOOP_BACK_TEST_TX_ONLY
				tx_count++;
				if( tx_count >= 200 ) {
					end_time = CURRENT_TIME;
					tx_count = 0;
					
					printk( "200 packet TX Done. (%d) => init time : %lu.%09lu, end time : %lu.%09lu\n", 
							tx_count, init_time.tv_sec, init_time.tv_nsec, end_time.tv_sec, end_time.tv_nsec );
					printk( "===================\n" );
					msleep( 3000 );
				}
#endif

#ifdef LOOP_BACK_TEST_RX_ONLY
			printk( "WaitWaitWait...\n" );
			while( 1 );
#endif

			}
		}

		dev_dbg( od->dev, "(%d) ===== tx loop =====\n", __LINE__ );

#ifdef LOOP_BACK_TEST
//		down( &transfer_event_sem );
#endif // LOOP_BACK_TEST

	}

exit :
	printk( "(%d) thread stop.\n", __LINE__ );

	return retval;
}


//========================================================//
//                ++ Flashless Boot. ++                   //
//========================================================//

typedef struct hsi_protocol_header_rec {
	unsigned long length:32;
} hsi_protocol_header;

struct ipc_hsi_send_modem_bin_header {
	u16 sot;		
	u16 type;		// Request, Response
	u16 length;
};

struct ipc_hsi_send_modem_bin_footer {
	u16 crc;
	u16 eot;
};

enum image_type {
	MODEM_PSI,
	MODEM_EBL,
	MODEM_MAIN,
	MODEM_NV,
};

struct image_buf {
	u32 length;
	u8 *buf;
};

#define HSI_SEND_BLOCK_SIZE		63 * 1024   //2048 (byte)
#define EBL_PACKET_SIZE			HSI_SEND_BLOCK_SIZE + 24

#define PSI_OFFSET			0
#define EBL_OFFSET			0xF000
#define MAIN_OFFSET			0x28000
#define SWVER_OFFSET		0x9FF000
#define SECURE_OFFSET		0x9FF800
#define NV_OFFSET			0xa00000

#define PSI_LEN			( EBL_OFFSET - PSI_OFFSET ) // 0xF058
#define EBL_LEN			( MAIN_OFFSET - EBL_OFFSET ) // 0x13F98
#define MAIN_LEN		( NV_OFFSET - MAIN_OFFSET ) // 0x84B04D
#define SECURE_LEN		2048
#define NV_LEN			( 2 * 1024 * 1024 )


#define MAIN_OFFSET_VM		0
#define NV_OFFSET_VM		0xD80000

#define ReqSecStart				0x0204
#define ReqSecEnd				0x0205
#define ReqForceHwReset			0x0208
#define ReqFlashSetAddress		0x0802
#define ReqFlashWriteBlock		0x0804

//#define CONFIG_DEBUG_FLASHLESS_BOOT
#ifdef CONFIG_DEBUG_FLASHLESS_BOOT
#define flashless_dbg(format,...) printk ("[ (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#else
#define flashless_dbg(format,...)
#endif

static u16 ipc_hsi_send_modem_bin_make_crc( u8 *buf )
{
	u16 crc = 0;
	int i;
	struct ipc_hsi_send_modem_bin_header *header = ( struct ipc_hsi_send_modem_bin_header * )( buf + 4 );

	crc += header->type;
	crc += header->length;

	buf += 4;
	buf += sizeof( struct ipc_hsi_send_modem_bin_header );
	for( i = 0 ; i < header->length ; i++ )
		crc += *buf++;

	return crc;
}

static int ipc_hsi_send_modem_bin_receive_ack(void *data)
{
	int ret = 0;

	u32 *rx_b = (u32*)data;
	u32 mipi_length;

	flashless_dbg("++  \n");

	// 1. Read mipi header to know mipi length
	flashless_dbg("Try read mpip length..  \n");
	ret = if_hsi_read( HSI_FLASHLESS_CHANNEL, rx_b, 4, __func__, HSI_READ_TIMEOUT_ENABLE );
	if( ret < 0 ) {
		printk("(%s) can not get mipi length ! \n", __func__);
		goto read_fail;
	}
	mipi_length = *rx_b;
	flashless_dbg("Read done (mipi length: %d)..  \n", mipi_length);
	
	// 2. Read payload
	mipi_length += (4 - mipi_length % 4) % 4;
	ret = if_hsi_read( HSI_FLASHLESS_CHANNEL, ++rx_b, mipi_length, __func__, HSI_READ_TIMEOUT_ENABLE );
	if( ret < 0 ) {
		printk("(%s) can not get payload data ! \n", __func__);
		goto read_fail;
	}

read_fail:


	flashless_dbg("-- \n ");

	return ret;
}

static int ipc_hsi_send_modem_bin_execute_cmd( struct ipc_hsi *od, u16 type, u32 len, void *data, 
															u8 *tx_buf, u8 *rx_buf )
{
	int retval = 0;
	u8 *tx_b = tx_buf;
	u8 *rx_b = rx_buf;
	u8 *cur_p_tx_b = NULL;
	u8 *cur_p_rx_b = NULL;
	int i;


	hsi_protocol_header *tx_hsi_header = NULL;
	hsi_protocol_header *rx_hsi_header = NULL;
	struct ipc_hsi_send_modem_bin_header *tx_header = NULL;
	struct ipc_hsi_send_modem_bin_footer *tx_footer = NULL;
	struct ipc_hsi_send_modem_bin_header *rx_header = NULL;
	int tx_padding_size = 0;

	flashless_dbg("++\n");

	//
	// # 1. Init tx, rx buffer
	//
	cur_p_tx_b = tx_b;

	tx_hsi_header = ( hsi_protocol_header * )cur_p_tx_b;
	cur_p_tx_b += sizeof( hsi_protocol_header );

	tx_header = ( struct ipc_hsi_send_modem_bin_header * )( cur_p_tx_b );
	cur_p_tx_b += sizeof( struct ipc_hsi_send_modem_bin_header );

	cur_p_rx_b = rx_b;
	
	rx_hsi_header = ( hsi_protocol_header * )cur_p_rx_b;
	cur_p_rx_b += sizeof( hsi_protocol_header );
	
	rx_header = ( struct ipc_hsi_send_modem_bin_header * ) cur_p_rx_b ;
	cur_p_rx_b += sizeof( struct ipc_hsi_send_modem_bin_header );

	//
	// # 2. set header value
	//
	tx_header->sot = 0x0002;
	tx_header->type = type;
	
	//
	// # 3. set payload data
	//
	if ( len > 0 ) {
		memcpy( ( void * )cur_p_tx_b, data, len );
		cur_p_tx_b += len;
	}
	tx_header->length = len;  // payload_length
	
	//
	// # 4. set footer value
	//
	tx_footer = ( struct ipc_hsi_send_modem_bin_footer * ) cur_p_tx_b;
	cur_p_tx_b += sizeof( struct ipc_hsi_send_modem_bin_footer );

	tx_footer->crc = ipc_hsi_send_modem_bin_make_crc( tx_b );
	tx_footer->eot = 0x0003;

	//
	// # 5. set padding if we need
	//
	tx_padding_size = (4 - (cur_p_tx_b - tx_b) % 4) % 4;

	for( i = 0; i < tx_padding_size; i++ )
	{
		*(cur_p_tx_b + i) = 0xEA; 
	}
	cur_p_tx_b += tx_padding_size;

	tx_hsi_header->length = cur_p_tx_b - tx_b - sizeof( hsi_protocol_header ) - tx_padding_size;
	
	/*
	dev_dbg( od->dev, "[SPI DUMP] tx : [%02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ... %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", 
		tx_b[ 0 ], tx_b[ 1 ], tx_b[ 2 ], tx_b[ 3 ], tx_b[ 4 ], 
		tx_b[ 5 ], tx_b[ 6 ], tx_b[ 7 ], tx_b[ 8 ], tx_b[ 9 ], 
		tx_b[ 10 ], tx_b[ 11 ], tx_b[ 12 ], tx_b[ 13 ], tx_b[ 14 ], 
		tx_b[ 15 ], tx_b[ 16 ], tx_b[ 17 ], tx_b[ 18 ], tx_b[ 19 ],
		tx_b[ EBL_PACKET_SIZE - 10 ], tx_b[ EBL_PACKET_SIZE - 9 ], tx_b[ EBL_PACKET_SIZE - 8 ], tx_b[ EBL_PACKET_SIZE - 7 ], tx_b[ EBL_PACKET_SIZE - 6 ],
		tx_b[ EBL_PACKET_SIZE - 5 ], tx_b[ EBL_PACKET_SIZE - 4 ], tx_b[ EBL_PACKET_SIZE - 3 ], tx_b[ EBL_PACKET_SIZE - 2 ], tx_b[ EBL_PACKET_SIZE - 1 ] );
	*/


#if 0	//TX DATA DUMP
	u16 *t16;
	t16 = tx_b;
	printk("%d, length of byte : %d\n", __LINE__, cur_p_tx_b - tx_b );

	printk("\n\nWrite DATA:");

   	for( i = 0; i < (cur_p_tx_b - tx_b)/2; i++) {
		printk("%04x ", *(t16+i));
	}

	printk("\n\n");

#endif

	//
	// # 6. WRITE packet
	//
	retval = if_hsi_write( HSI_FLASHLESS_CHANNEL, ( u32 * )tx_b, cur_p_tx_b - tx_b, __func__ );
	if(retval < 0){
		printk( "(%s, %d) if_hsi_write Fail : %d\n", __func__, __LINE__, retval );
		goto err;
	}
	flashless_dbg("\n   waiting for ACK ....\n");

	if(type != ReqForceHwReset) {
		retval = ipc_hsi_send_modem_bin_receive_ack( rx_b );
		if(retval < 0){
			printk("(%s, %d) ACK Error!!\n", __func__, __LINE__);
			goto err;
		}	
	}

#if 0
	t16 = rx_b + sizeof(rx_hsi_header);
	printk("Received DATA: Length %lu \n", rx_hsi_header->length);
	for(i = 0; i < (rx_hsi_header->length)/2; i++) {
		printk("%04x ", *(t16+i));
	}
	printk("\n\n Read Done!!!!!\n\n"
);
#endif

	if( rx_header->type != type && type != ReqForceHwReset ) {
		dev_err( od->dev, "(%d) execute cmd ack error : 0x%x(0x%x)\n", __LINE__, rx_header->type, type );

		dev_err( od->dev, "[HSI DUMP] rx : [%02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ... %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", 
			rx_b[ 0 ], rx_b[ 1 ], rx_b[ 2 ], rx_b[ 3 ], rx_b[ 4 ], 
			rx_b[ 5 ], rx_b[ 6 ], rx_b[ 7 ], rx_b[ 8 ], rx_b[ 9 ], 
			rx_b[ 10 ], rx_b[ 11 ], rx_b[ 12 ], rx_b[ 13 ], rx_b[ 14 ], 
			rx_b[ 15 ], rx_b[ 16 ], rx_b[ 17 ], rx_b[ 18 ], rx_b[ 19 ],
			rx_b[ EBL_PACKET_SIZE - 10 ], rx_b[ EBL_PACKET_SIZE - 9 ], rx_b[ EBL_PACKET_SIZE - 8 ], rx_b[ EBL_PACKET_SIZE - 7 ], rx_b[ EBL_PACKET_SIZE - 6 ],
			rx_b[ EBL_PACKET_SIZE - 5 ], rx_b[ EBL_PACKET_SIZE - 4 ], rx_b[ EBL_PACKET_SIZE - 3 ], rx_b[ EBL_PACKET_SIZE - 2 ], rx_b[ EBL_PACKET_SIZE - 1 ] );
		
		retval = -1;
	}
	else {
		//		dev_dbg( od->dev, "(%d) execute cmd ack Done.\n", __LINE__ );
	}	

	/*
	dev_dbg( od->dev, "[SPI DUMP] rx : [%02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x ... %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", 
		rx_b[ 0 ], rx_b[ 1 ], rx_b[ 2 ], rx_b[ 3 ], rx_b[ 4 ], 
		rx_b[ 5 ], rx_b[ 6 ], rx_b[ 7 ], rx_b[ 8 ], rx_b[ 9 ], 
		rx_b[ 10 ], rx_b[ 11 ], rx_b[ 12 ], rx_b[ 13 ], rx_b[ 14 ], 
		rx_b[ 15 ], rx_b[ 16 ], rx_b[ 17 ], rx_b[ 18 ], rx_b[ 19 ],
		rx_b[ EBL_PACKET_SIZE - 10 ], rx_b[ EBL_PACKET_SIZE - 9 ], rx_b[ EBL_PACKET_SIZE - 8 ], rx_b[ EBL_PACKET_SIZE - 7 ], rx_b[ EBL_PACKET_SIZE - 6 ],
		rx_b[ EBL_PACKET_SIZE - 5 ], rx_b[ EBL_PACKET_SIZE - 4 ], rx_b[ EBL_PACKET_SIZE - 3 ], rx_b[ EBL_PACKET_SIZE - 2 ], rx_b[ EBL_PACKET_SIZE - 1 ] );
	*/

	if( type == 0x0208 ) // ReqForceHwReset
		return 0;

err:

	flashless_dbg("--\n");

	return retval;
}


static int ipc_hsi_send_modem_bin_xmit_img( struct ipc_hsi *od, enum image_type type, u32 *address, 
														u8 *tx_buf, u8 *rx_buf )
{
	int retval = 0;
	struct image_buf img;
	u32 data_size;
	u32 send_size = 0;
	u32 rest_size = 0;
	u8 *ptr;
	int i;

	flashless_dbg("type: %d\n", type);

	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img type : %d.\n", __LINE__, type );
	switch( type ) {
		
		case MODEM_MAIN:
			img.buf = ( u8 * )( p_virtual_buff + MAIN_OFFSET_VM );
			img.length = MAIN_LEN;
			dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img save MAIN to img.\n", __LINE__ );
				
			break;
			
		case MODEM_NV:
			img.buf = ( u8 * )( p_virtual_buff + NV_OFFSET_VM );
			img.length = NV_LEN;
			dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img save NV to img.\n", __LINE__ );
			
			break;
			
		default:
			dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img wrong : %d.", __LINE__, type );
			return -1;
	}

	//Command : ReqFlashSetAddress( 0x0802 )
	retval = ipc_hsi_send_modem_bin_execute_cmd( od, ReqFlashSetAddress, sizeof( u32 ), address, tx_buf, rx_buf );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd fail : %d", __LINE__, retval );
		return -1;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd Done.\n", __LINE__ );
	dev_dbg( od->dev, "(%d) Start send img. size : %d\n", __LINE__, img.length );

	flashless_dbg("send address (%x)  \n", *address);


	ptr = img.buf;
	data_size = HSI_SEND_BLOCK_SIZE;
	rest_size = img.length;

	for( i = 0 ; send_size < img.length ; i++ ) {
//		(i % 100) ?  : printk("\n%d\n", i);
		( i % 10 ) ?  : printk( "." );
		if( rest_size < HSI_SEND_BLOCK_SIZE )
			data_size = rest_size;

		//Command : ReqFlashWriteBlock( 0x0804 )
		retval = ipc_hsi_send_modem_bin_execute_cmd( od, ReqFlashWriteBlock, data_size, ptr, tx_buf, rx_buf );
		if( retval < 0 ) {
			dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd fail : %d", __LINE__, retval );
			return -1;
		}

		send_size += data_size;
		rest_size -= data_size;
		ptr += data_size;

		if( !( i % 100 ) )
			dev_dbg( od->dev, "(%d) [%d] 0x%x size done, rest size: 0x%x\n", __LINE__, i, send_size, rest_size );
	}
	printk( "\n" );
	
	return retval;
}


static void ipc_hsi_send_modem_bin( struct work_struct *send_modem_w )
{
	int retval = 0;
	u32 int_cmd = 0xABCDABCD;
	u32 int_cmd_fail = 0xDCBADCBA;
	u32 type = 0;
	struct ipc_hsi_send_modem_bin_workq_data *smw 
		= container_of( send_modem_w, struct ipc_hsi_send_modem_bin_workq_data, send_modem_w );
	struct ipc_hsi *od = smw->od;

	u32 modem_addr = 0x60300000;//0x60300000;0x61580000;0x61600000
	//u32 nvm_static_fix_addr = 0x60e80000;//0x60e80000;0x61F80000;0x61E80000
	//u32 nvm_static_cal_addr = 0x60f00000;//0x60f00000;0x61F00000;0x61F00000
	//u32 nvm_dynamic_addr = 0x60f80000;//0x60f80000;0x61E80000;0x61F80000
	u32 nvm_addr = 0x60E80000;//0x60D00000;//0x60b80000;//0x60C00000;
	u16 sec_end = 0x0000;
	u32 force_hw_reset = 0x00111001;
	
	u8 *sec_start = NULL;

	u8 *tx_b = NULL;
	u8 *rx_b = NULL;

	tx_b = kmalloc( EBL_PACKET_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !tx_b ) {
		dev_err( od->dev, "(%d) tx_b kmalloc fail.", __LINE__ );
		return; // -ENOMEM;
	}
	memset( tx_b, 0, EBL_PACKET_SIZE);

	rx_b = kmalloc( EBL_PACKET_SIZE, GFP_DMA | GFP_ATOMIC );
	if( !rx_b ) {
		dev_err( od->dev, "(%d) rx_b kmalloc fail.", __LINE__ );
		return; // -ENOMEM;
	}
	memset( rx_b, 0, EBL_PACKET_SIZE);

	retval = hsi_init_handshake( HSI_INIT_MODE_FLASHLESS_BOOT );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) hsi_init_handshake fail :%d\n", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) hsi_init_handshake Done :%d\n", __LINE__, retval );

	printk( "[HSI DUMP] mb : [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", 
		*( u8 * )( p_virtual_buff ), *( u8 * )( p_virtual_buff + 1 ), *( u8 * )( p_virtual_buff + 2 ), *( u8 * )( p_virtual_buff + 3 ), *( u8 * )( p_virtual_buff + 4 ), 
		 *( u8 * )( p_virtual_buff + 5 ),  *( u8 * )( p_virtual_buff + 6 ),  *( u8 * )( p_virtual_buff + 7 ),  *( u8 * )( p_virtual_buff + 8 ),  *( u8 * )( p_virtual_buff + 9 ), 
		*( u8 * )( p_virtual_buff + 10 ),  *( u8 * )( p_virtual_buff + 11 ),  *( u8 * )( p_virtual_buff + 12 ),  *( u8 * )( p_virtual_buff + 13 ),  *( u8 * )( p_virtual_buff + 14 ), 
		*( u8 * )( p_virtual_buff + 15 ),  *( u8 * )( p_virtual_buff + 16 ),  *( u8 * )( p_virtual_buff + 17 ),  *( u8 * )( p_virtual_buff + 18 ),  *( u8 * )( p_virtual_buff + 19 ) );

	//Command : ReqSecStart( 0x0204 )
	type = ReqSecStart;
	sec_start = kmalloc( 2048, GFP_ATOMIC );
	if( !sec_start ) {
		dev_err( od->dev, "(%d) sec_start kmalloc fail.", __LINE__ );
		goto err;
	}
	
#if 1 //XMM6260_11A 
	memcpy( sec_start, ( ( u8 * )p_virtual_buff + SECURE_OFFSET - MAIN_OFFSET ), 2048);	//send secure
#else
	memset( sec_start, 0, 2048 );
#endif

	retval = ipc_hsi_send_modem_bin_execute_cmd( od, type, 2048, ( void * )sec_start, tx_b, rx_b );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd fail : %d", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd Done.\n", __LINE__ );
//	printk("\n\n\n(%s)(%d)  1st step. DONE  \n\n\n\n\n", __func__, __LINE__);
	
	retval = ipc_hsi_send_modem_bin_xmit_img( od, MODEM_MAIN, &modem_addr, tx_b, rx_b );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img fail : %d", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img MODEM_MAIN Done.\n", __LINE__ );
//	printk("\n\n\n(%s)(%d)  2nd step. DONE  \n\n\n\n\n", __func__, __LINE__);
	printk( "[CP FLASHLESS BOOT] : XMIT MAIN DONE.\n" );

	printk( "[HSI NV DUMP] mb : [%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x]\n", 
		*( u8 * )( p_virtual_buff + NV_OFFSET_VM ), *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 1 ), *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 2 ), *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 3 ), *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 4 ), 
		 *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 5 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 6 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 7 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 8 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 9 ), 
		*( u8 * )( p_virtual_buff + NV_OFFSET_VM + 10 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 11 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 12 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 13 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 14 ), 
		*( u8 * )( p_virtual_buff + NV_OFFSET_VM + 15 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 16 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 17 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 18 ),  *( u8 * )( p_virtual_buff + NV_OFFSET_VM + 19 ) );

	retval = ipc_hsi_send_modem_bin_xmit_img( od, MODEM_NV, &nvm_addr, tx_b, rx_b );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img fail : %d", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_xmit_img MODEM_NV:nvm_addr Done.\n", __LINE__ );
//	printk("\n\n\n(%s)(%d)  3rd step. DONE  \n\n\n\n\n", __func__, __LINE__);
	printk( "[CP FLASHLESS BOOT] : XMIT NV DATA DONE.\n" );

	//Command : ReqSecEnd( 0x0205 )
	retval = ipc_hsi_send_modem_bin_execute_cmd( od, ReqSecEnd, sizeof( u16 ), &sec_end, tx_b, rx_b );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd fail : %d", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd; ReqSecEnd Done.\n", __LINE__ );
//	printk("\n\n\n(%s)(%d)  4th step. DONE  \n\n\n\n\n", __func__, __LINE__);

	//Command : ReqForceHwReset( 0x0208 )
	retval = ipc_hsi_send_modem_bin_execute_cmd( od, ReqForceHwReset, sizeof( u32 ), &force_hw_reset, tx_b, rx_b );
	if( retval < 0 ) {
		dev_err( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd fail : %d", __LINE__, retval );
		goto err;
	}
	dev_dbg( od->dev, "(%d) ipc_hsi_send_modem_bin_execute_cmd; ReqForceHwReset Done.\n", __LINE__ );

//	printk("\n\n\n(%s)(%d)  5th step. DONE  \n\n\n\n\n", __func__, __LINE__);
	printk( "[CP FLASHLESS BOOT] : ALL DONE.\n" );

	kfree( sec_start );

	// make data interrupt cmd
	ipc_hsi_make_data_interrupt( od, int_cmd );

	cp_flashless_boot_done = 1;

	kfree(tx_b);
	kfree(rx_b);
	return;

err :
	kfree(tx_b);
	kfree(rx_b);
	
	// make data interrupt cmd
	ipc_hsi_make_data_interrupt( od, int_cmd_fail );
}


//========================================================//
//                ++ CP Ramdump. ++                   //
//========================================================//

#define DUMP_PACKET_SIZE 	( 48 * 1024 )
#define DUMP_ERR_INFO_SIZE 	( 150 )

static void cp_ramdump_hsi( struct ipc_hsi *od, int order )
{
	int retval = 0;
	int i = 0;
	u32 int_cmd = 0xDEADAB00;
	u32 tx_data = 0x53534150;
	u8 *rx_data = NULL;

	rx_data = kmalloc( DUMP_PACKET_SIZE + 4, GFP_DMA | GFP_ATOMIC );
	if( !rx_data ) {
		printk( "[%s](%d) rx_data kmalloc fail.\n", __func__, __LINE__ );
		return;
	}
	memset( ( void * )rx_data, 0, DUMP_PACKET_SIZE );
//	printk( "[%s](%d) rx_data kmalloc Done.\n", __func__, __LINE__ );

	if( order == 0 ) {
		retval = hsi_init_handshake( HSI_INIT_MODE_CP_RAMDUMP );
		if( retval < 0 ) {
			printk( "[%s](%d) hsi_init_handshake fail : %d\n", __func__, __LINE__, retval );
		}
		else {
			printk( "[%s](%d) hsi_init_handshake Done : %d\n", __func__, __LINE__, retval );
		}
	}

	printk( "[CP RAMDUMP] : START. (%d)\n", order );

	if( order == 0 ) {
		retval = if_hsi_read( HSI_FLASHLESS_CHANNEL, ( void * )rx_data, DUMP_ERR_INFO_SIZE + 4, __func__, HSI_READ_TIMEOUT_DISABLE );
		if( retval < 0 ) {
			printk( "[%s](%d) if_hsi_read fail : %d\n", __func__, __LINE__, retval );
			return;
		}
		else {
			printk( "[%s](%d) if_hsi_read Done.\n", __func__, __LINE__ );

			memcpy( (void *)p_virtual_buff, ( void * )( rx_data + 4 ), DUMP_ERR_INFO_SIZE );
			printk( "[%s](%d) dump file copy Done.\n", __func__, __LINE__ );
		}
	}
	else {
		if( order == 1 ) {
			for( i = 0; i < 2; i++ ) {
				retval = if_hsi_write( HSI_FLASHLESS_CHANNEL, ( void * )&tx_data, sizeof( u32 ), __func__ );
				if( retval < 0 ) {
					printk( "[%s](%d) if_hsi_write fail : %d\n", __func__, __LINE__, retval );
					return;
				}
				else {
					printk( "[%s](%d) if_hsi_write SEND PASS Done.\n", __func__, __LINE__ );
				}
			}
		}
		
		for( i = 0; i < 256; i++ ) {
			( i % 10 ) ?  : printk( "." );
			retval = if_hsi_read( HSI_FLASHLESS_CHANNEL, ( void * )rx_data, DUMP_PACKET_SIZE + 4, __func__, HSI_READ_TIMEOUT_DISABLE );
			if( retval < 0 ) {
				printk( "[%s](%d) if_hsi_read fail : %d\n", __func__, __LINE__, retval );
				return;
			}
			else {
//				printk( "[%s](%d) if_hsi_read Done. i : %d\n", __func__, __LINE__, i );

				memcpy( (void *)( p_virtual_buff + ( i * DUMP_PACKET_SIZE ) ), ( void * )( rx_data + 4 ), DUMP_PACKET_SIZE );
//				printk( "[%s](%d) dump file copy Done. i : %d\n", __func__, __LINE__, i );
			}
		}
		printk( "\n" );

		if( order == 4 ) {
			retval = if_hsi_write( HSI_FLASHLESS_CHANNEL, ( void * )&tx_data, sizeof( u32 ), __func__ );
			if( retval < 0 ) {
				printk( "[%s](%d) if_hsi_write fail : %d\n", __func__, __LINE__, retval );
				return;
			}
			else {
				printk( "[%s](%d) if_hsi_write SEND PASS Done.\n", __func__, __LINE__ );
			}
		}
	}

	printk( "[CP RAMDUMP] : DUMP DONE. (%d)\n", order );

	kfree( rx_data );

	// make data interrupt cmd
	ipc_hsi_make_data_interrupt( od, int_cmd );
}

#if 0
extern void ( *onedram_cp_force_crash ) ( void );
static void ipc_hsi_cp_force_crash( void )
{
	u32 int_cmd = 0;
	
	printk( "[ipc_hsi_cp_force_crash]\n" );

	if( ipc_hsi ) {
// Silent Reset
#if 0
		// make data interrupt cmd
		int_cmd = MB_CMD( MBC_ERR_DISPLAY );
		
		ipc_hsi->reg->mailbox_BA = int_cmd;

		if( transfer_thread_waiting ) {
			//transfer_thread_waiting = 0;
			up( &transfer_event_sem );
		}

		printk( "Send CP Fatal command.\n" );
#endif
	}
	else {
		printk( "error ipc_hsi null.\n" );
	}
	
}
#endif

static int write_command_thread( void *data );
static int read_command_thread( void *data );
static int ipc_hsi_start_all_thread( void )
{
	int retval = 0;

	init_completion( &tx_thread_start );
	init_completion( &fmt_read_thread_start );
	init_completion( &raw_read_thread_start );
	init_completion( &rfs_read_thread_start );
	init_completion( &cmd_read_thread_start );

#ifdef LOOP_BACK_TEST
	init_completion( &loopback_read_thread_start );
#endif

	init_completion( &write_cmd_thread_start );
	init_completion( &read_cmd_thread_start );

	if( tx_thread ) {
		printk( "[%s](%d) kthread_stop() - tx_thread start.\n", __func__, __LINE__ );

		if( transfer_thread_waiting ) {
			//transfer_thread_waiting = 0;
			up( &transfer_event_sem );
		}
		
		kthread_stop( tx_thread );
		printk( "[%s](%d) kthread_stop() - tx_thread Done.\n", __func__, __LINE__ );
	}
	
	tx_thread = kthread_create( ipc_hsi_tx_thread, ( void * )ipc_hsi, "ipc_hsi_tx_thread" );
	if( IS_ERR( tx_thread ) ) {
		printk( "[%s](%d) kthread_create() - ipc_hsi_tx_thread failed.\n", __func__, __LINE__ );

		retval = IS_ERR( tx_thread );
		goto err;
	}
	else {
		kthread_bind( tx_thread , 0 );
		wake_up_process( tx_thread );

		printk( "[%s](%d) kthread_create() - ipc_hsi_tx_thread Done.\n", __func__, __LINE__ );
	}
	
	fmt_read_thread = kthread_create( ipc_hsi_fmt_rx_thread, ( void * )ipc_hsi, "ipc_hsi_fmt_rx_thread" );
	if( IS_ERR( fmt_read_thread ) ) {
		printk( "[%s](%d) kthread_create() - ipc_hsi_fmt_rx_thread failed.\n", __func__, __LINE__ );

		retval = IS_ERR( fmt_read_thread );
		goto err;
	}
	else {
		kthread_bind( fmt_read_thread , 0 );
		wake_up_process( fmt_read_thread );
		
		printk( "[%s](%d) kthread_create() - ipc_hsi_fmt_rx_thread Done.\n", __func__, __LINE__ );
	}
	
	raw_read_thread = kthread_create( ipc_hsi_raw_rx_thread, ( void * )ipc_hsi, "ipc_hsi_raw_rx_thread" );
	if( IS_ERR( raw_read_thread ) ) {
		printk( "[%s](%d) kthread_create() - ipc_hsi_raw_rx_thread failed.\n", __func__, __LINE__ );

		retval = IS_ERR( raw_read_thread );
		goto err;
	}
	else {
		kthread_bind( raw_read_thread , 0 );
		wake_up_process( raw_read_thread );
		
		printk( "[%s](%d) kthread_create() - ipc_hsi_raw_rx_thread Done.\n", __func__, __LINE__ );
	}

	rfs_read_thread = kthread_create( ipc_hsi_rfs_rx_thread, ( void * )ipc_hsi, "ipc_hsi_rfs_rx_thread" );
	if( IS_ERR( rfs_read_thread ) ) {
		printk( "[%s](%d) kthread_create() - ipc_hsi_rfs_rx_thread  failed.\n", __func__, __LINE__ );

		retval = IS_ERR( rfs_read_thread );
		goto err;
	}
	else {
		kthread_bind( rfs_read_thread , 0 );
		wake_up_process( rfs_read_thread );

		printk( "[%s](%d) kthread_create() - ipc_hsi_rfs_rx_thread Done.\n", __func__, __LINE__ );
	}

	cmd_read_thread = kthread_create( ipc_hsi_cmd_rx_thread, ( void * )ipc_hsi, "ipc_hsi_cmd_rx_thread" );
	if( IS_ERR( cmd_read_thread ) ) {
		printk( "[%s](%d) kthread_create() - ipc_hsi_cmd_rx_thread  failed.\n", __func__, __LINE__ );

		retval = IS_ERR( cmd_read_thread );
		goto err;
	}
	else {
		kthread_bind( cmd_read_thread , 0 );
		wake_up_process( cmd_read_thread );

		printk( "[%s](%d) kthread_create() - ipc_hsi_cmd_rx_thread Done.\n", __func__, __LINE__ );
	}

#ifdef LOOP_BACK_TEST
	loopback_read_thread = kthread_create( ipc_hsi_loopback_rx_thread, ( void * )ipc_hsi, "ipc_hsi_loopback_rx_thread" );
	if( IS_ERR( loopback_read_thread ) ) {
		dev_err( od->dev, "kthread_create() - ipc_hsi_loopback_rx_thread failed.\n" );

		retval = IS_ERR( loopback_read_thread );
		goto err;
	}
	else {
		kthread_bind( loopback_read_thread , 0 );
		wake_up_process( loopback_read_thread );

		dev_dbg( od->dev, "kthread_create() - ipc_hsi_loopback_rx_thread Done.\n" );
	}
#endif // LOOP_BACK_TEST

	write_cmd_thread = kthread_create( write_command_thread, NULL, "write_command_thread" );
	if( IS_ERR( write_cmd_thread ) ) {
		printk( "[%s](%d) kthread_create() - write_cmd_thread failed.\n", __func__, __LINE__ );

		retval = IS_ERR( write_cmd_thread );
		goto err;
	}
	else {
		kthread_bind( write_cmd_thread , 0 );
		wake_up_process( write_cmd_thread );
		
		printk( "[%s](%d) kthread_create() - write_cmd_thread Done.\n", __func__, __LINE__ );
	}

	read_cmd_thread = kthread_create( read_command_thread, NULL, "read_command_thread" );
	if( IS_ERR( read_cmd_thread ) ) {
		printk( "[%s](%d) kthread_create() - read_cmd_thread failed.\n", __func__, __LINE__ );

		retval = IS_ERR( read_cmd_thread );
		goto err;
	}
	else {
		kthread_bind( read_cmd_thread , 0 );
		wake_up_process( read_cmd_thread );
		
		printk( "[%s](%d) kthread_create() - read_cmd_thread Done.\n", __func__, __LINE__ );
	}

err :
	return retval;
}

static int __devinit ipc_hsi_platform_probe( struct platform_device *pdev )
{
	int r;
	struct ipc_hsi *od = NULL;
	struct ipc_hsi_platform_data *pdata;

	printk( "[%s]\n",__func__ );
	
	pdata = pdev->dev.platform_data;
	if( !pdata || !pdata->cfg_gpio ) {
		dev_err( &pdev->dev, "No platform data\n" );
		r = -EINVAL;
		goto err;
	}

	od = kzalloc( sizeof( struct ipc_hsi ), GFP_KERNEL );
	if (!od) {
		dev_err( &pdev->dev, "(%d) failed to allocate device\n", __LINE__ );
		
		r = -ENOMEM;
		goto err;
	}
	ipc_hsi = od;
	
	dev_dbg( &pdev->dev, "(%d) IpcHsi dev: %p\n", __LINE__, od );

	od->base = 0;
	od->size = 0x1000000; // 16M
	r = _request_mem(od, pdev);
	if (r)
		goto err;

	/* init mailbox state before registering irq handler */
	onedram_init_mailbox();

	_init_data( od );

	pdata->cfg_gpio();

	// Init work structure
	ipc_hsi_send_modem_work_data = kmalloc( sizeof( struct ipc_hsi_send_modem_bin_workq_data ), GFP_ATOMIC );
	if( !ipc_hsi_send_modem_work_data ) {
		dev_err( &pdev->dev, "(%d) memory alloc fail\n", __LINE__ );

		r = -ENOMEM;
		goto err;
	}
	INIT_WORK( &ipc_hsi_send_modem_work_data->send_modem_w, ipc_hsi_send_modem_bin );

	r = _register_chrdev( od );
	if( r ) {
		dev_err( &pdev->dev, "(%d) Failed to register chrdev\n", __LINE__ );
		goto err;
	}

	r = sysfs_create_group( &od->dev->kobj, &ipc_hsi_group );
	if( r ) {
		dev_err( &pdev->dev, "(%d) Failed to create sysfs files\n", __LINE__ );
		goto err;
	}
	od->group = &ipc_hsi_group;

	platform_set_drvdata( pdev, od );

	r = ipc_hsi_start_all_thread();
	if( r ) {
		dev_err( &pdev->dev, "(%d) ipc_hsi_start_all_thread FAIL : %d\n", __LINE__, r );
		goto err;
	}
	dev_err( &pdev->dev, "(%d) ipc_hsi_start_all_thread Done.\n", __LINE__ );	
	
//	onedram_cp_force_crash = ipc_hsi_cp_force_crash;

	cp_flashless_boot_done = 0;

	dev_err( &pdev->dev, "(%d) platform probe Done.\n", __LINE__ );

	return 0;

err:
	_release( od );
	
	return r;
}

static int __devexit ipc_hsi_platform_remove( struct platform_device *pdev )
{
	struct ipc_hsi *od = platform_get_drvdata( pdev );

	/* TODO: need onedram_resource clean? */
	_unregister_all_handlers();
	platform_set_drvdata( pdev, NULL );
	ipc_hsi = NULL;
	_release( od );

	// Free work queue data
	kfree( ipc_hsi_send_modem_work_data );

	return 0;
}

#ifdef CONFIG_PM
static int ipc_hsi_platform_suspend( struct platform_device *pdev, pm_message_t state )
{
//	struct onedram *od = platform_get_drvdata(pdev);

	return 0;
}

static int ipc_hsi_platform_resume( struct platform_device *pdev )
{
//	struct onedram *od = platform_get_drvdata(pdev);

	return 0;
}
#else
#  define ipc_hsi_platform_suspend NULL
#  define ipc_hsi_platform_resume NULL
#endif

static struct platform_driver ipc_hsi_platform_driver = {
	.probe = ipc_hsi_platform_probe,
	.remove = __devexit_p( ipc_hsi_platform_remove ),
	.suspend = ipc_hsi_platform_suspend,
	.resume = ipc_hsi_platform_resume,
	.driver = {
		.name = DRVNAME,
	},
};


struct if_hsi_channel {
	struct hsi_device *dev;
	unsigned int channel_id;
	
	u32 *tx_data;
	unsigned int tx_count;
	u32 *rx_data;
	unsigned int rx_count;
	
	unsigned int opened;
	
	unsigned int tx_state;
	unsigned int rx_state;
	spinlock_t tx_state_sem;
	spinlock_t rx_state_sem;

	struct semaphore read_done;
	struct semaphore write_done;

	unsigned int tx_step;
	unsigned int rx_step;
	spinlock_t tx_step_sem;
	spinlock_t rx_step_sem;

	int got_nack;
	
	struct semaphore ack_done;
	struct semaphore close_conn_done;
	struct semaphore open_ack_done;
};
struct if_hsi_channel hsi_channles[ HSI_MAX_CHANNELS ];

static u32 write_cmd[ 100 ];
static int write_cmd_in = 0, write_cmd_out = 0;
static struct semaphore write_cmd_sem;
spinlock_t write_cmd_lock;

unsigned long long rx_save_cmd_time[ 30 ], tx_save_cmd_time[ 30 ];
u32 rx_save_cmd[ 30 ], tx_save_cmd[ 30 ];
int rx_save_cmd_p = 0, tx_save_cmd_p = 0;

unsigned long long rx_data_time[ 5 ][ 30 ], tx_data_time[ 5 ][ 30 ];
int rx_data_time_p[ 5 ] = {0,0,0,0,0}, tx_data_time_p[ 5 ] = {0,0,0,0,0};

int cawake_level = 0;

void ipc_hsi_dump_cmd(void);
extern void modemctl_force_silent_reset( void );
static int __devinit if_hsi_probe( struct hsi_device *dev );
static int __devexit if_hsi_remove( struct hsi_device *dev );
static struct hsi_device_driver if_hsi_driver = {
	.ctrl_mask = ANY_HSI_CONTROLLER,
	.probe = if_hsi_probe,
	.remove = __devexit_p( if_hsi_remove ),
	.driver = {
	.name = "if_hsi_driver"
	},
};

static int if_hsi_set_wakeline( u32 ch, unsigned int state )
{
	int retval = 0;

#ifdef USE_WAKELOCK_TO_CTRL_WAKELINE
#ifdef CONFIG_HAS_WAKELOCK
	if( state ) {
		if( !wake_lock_active( &ipc_hsi_wlock ) ) {
			//omap_pm_set_max_mpu_wakeup_lat( &pm_qos_handle, 7 );
			
			wake_lock( &ipc_hsi_wlock );
//			printk( "[%s](%d) WAKE LOCK setting done.\n",__func__, __LINE__ );
		}
	}
#endif
#endif

	retval = hsi_ioctl( hsi_channles[ ch ].dev, state ? HSI_IOCTL_ACWAKE_UP : HSI_IOCTL_ACWAKE_DOWN, NULL );
	if( retval < 0 ) {
		if( retval != -EPERM )
			printk( "[%s](%d) hsi_ioctl ACWAKE(%d) setting fail : %d\n",__func__, __LINE__, state, retval );
		else
			return 0;
	}
	else {
//		printk( "[%s](%d) hsi_ioctl ACWAKE(%d) setting Done(ch:%d).\n",__func__, __LINE__, state, ch );
	}
	
	return retval;
}

static int if_hsi_openchannel( struct if_hsi_channel *channel )
{
	int retval = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	if( channel->opened ) {
		printk( "[%s](%d) channel is already opened(ch:%d)\n",__func__, __LINE__, channel->channel_id );
		return retval;
	}

	if( !channel->dev ) {
		printk( "[%s](%d) channel's dev fail(ch:%d)\n",__func__, __LINE__, channel->channel_id );
		return -ENODEV;
	}

	retval = hsi_open( channel->dev );
	if( retval < 0 ) {
		printk( "[%s](%d) channel open fail(ch:%d) : %d\n",__func__, __LINE__, channel->channel_id, retval );
		return retval;
	}
	printk( "[%s](%d) channel open Done (ch:%d)\n",__func__, __LINE__, channel->channel_id );

	channel->opened = 1;
	
	spin_lock( &channel->tx_step_sem );
	channel->tx_step = HSI_LL_TX_STATE_IDLE;
	spin_unlock( &channel->tx_step_sem );

	spin_lock( &channel->rx_step_sem );
	channel->rx_step = HSI_LL_RX_STATE_TO_CONN_READY;
	spin_unlock( &channel->rx_step_sem );

	return retval;
}

static int if_hsi_closechannel( struct if_hsi_channel *channel )
{
	int retval = 0;
	u32 tx_state = 0, rx_state = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	if( !channel->opened ) {
//		printk( "[%s](%d) channel is already closed(ch:%d)\n",__func__, __LINE__, channel->channel_id );
		return retval;
	}

	if( !channel->dev ) {
		printk( "[%s](%d) channel's dev fail(ch:%d)\n",__func__, __LINE__, channel->channel_id );
		return -ENODEV;
	}

	if( if_hsi_set_wakeline( channel->channel_id, 0 ) < 0 ) {
//		printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
	}

	spin_lock( &channel->rx_state_sem );
	rx_state = channel->rx_state;
	spin_unlock( &channel->rx_state_sem );

	if( rx_state & HSI_CHANNEL_RX_STATE_READING ) {
		spin_lock( &channel->rx_state_sem );
		channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
		spin_unlock( &channel->rx_state_sem );
		
		hsi_read_cancel( channel->dev );
	}

	spin_lock( &channel->tx_state_sem );
	tx_state = channel->tx_state;
	spin_unlock( &channel->tx_state_sem );
	
	if( tx_state & HSI_CHANNEL_TX_STATE_WRITING ) {
		spin_lock( &channel->tx_state_sem );
		channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
		spin_unlock( &channel->tx_state_sem );
		
		hsi_write_cancel( channel->dev );
	}
//	printk( "[%s](%d) read/write canceled.\n", __func__, __LINE__ );

	retval = hsi_ioctl( channel->dev, HSI_IOCTL_SW_RESET, NULL );
	if( retval < 0 ) {
//		printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET fail : %d\n", __func__, __LINE__, retval );
		retval = 0;
	}
	else {
		printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET Done : %d\n", __func__, __LINE__, retval );
	}

	hsi_close( channel->dev );
	printk( "[%s](%d) ch : %d hsi_close Done.\n", __func__, __LINE__, channel->channel_id );

	channel->opened = 0;

	spin_lock( &channel->tx_step_sem );
	channel->tx_step = HSI_LL_TX_STATE_CLOSED;
	spin_unlock( &channel->tx_step_sem );

	spin_lock( &channel->rx_step_sem );
	channel->rx_step = HSI_LL_RX_STATE_CLOSED;
	spin_unlock( &channel->rx_step_sem );
	
	return retval;
}

static int if_hsi_read( int ch, u32 *data, unsigned int size, const char *func, int time_out_flag )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	u32 rx_state = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ ch ];
//	printk( "[%s](%d) ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, size );

	channel->rx_data = data;
	if( size % 4 )
		size += ( 4 - ( size % 4 ) );
	channel->rx_count = size;
//	printk( "[%s](%d) ch : %d, count : %d\n", __func__, __LINE__, channel->channel_id, channel->rx_count );

	spin_lock( &channel->rx_state_sem );
	rx_state = channel->rx_state;
	spin_unlock( &channel->rx_state_sem );

	if( rx_state & HSI_CHANNEL_RX_STATE_READING ) {
		printk( "[%s](%d) Read Still Peding. ch : %d, rx_state : %08x\n", __func__, __LINE__, channel->channel_id, rx_state );
		return -EBUSY;
	}

	spin_lock( &channel->rx_state_sem );
	channel->rx_state |= HSI_CHANNEL_RX_STATE_READING;
	spin_unlock( &channel->rx_state_sem );

	retval = hsi_read( channel->dev, channel->rx_data, channel->rx_count / 4 );
	if( retval < 0 ) {
		printk( "[%s](%d) hsi_read(ch:%d) fail : %d\n", __func__, __LINE__, channel->channel_id, retval );

		spin_lock( &channel->rx_state_sem );
		rx_state = channel->rx_state;
		spin_unlock( &channel->rx_state_sem );

		if( rx_state & HSI_CHANNEL_RX_STATE_READING ) {
			hsi_read_cancel( channel->dev );
			printk( "[%s](%d) hsi_read_cancel Done\n", __func__, __LINE__ );
		}

		spin_lock( &channel->rx_state_sem );
		channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
		spin_unlock( &channel->rx_state_sem );
		
		return retval;
	}
//	printk( "[%s](%d) hsi_read(ch:%d) Done : %d\n", __func__, __LINE__, channel->channel_id, retval );

//	printk( "[%s](%d) Wait Read_Done... ch : %d\n", __func__, __LINE__, channel->channel_id );
	if( time_out_flag ) {
		if( down_timeout( &channel->read_done, HSI_READ_DONE_TIMEOUT ) ) {
			printk( "[%s][%s](%d) Error TimeOut Read_Done... ch : %d\n", __func__, func, __LINE__, channel->channel_id );

			spin_lock( &channel->rx_state_sem );
			channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
			spin_unlock( &channel->rx_state_sem );
			
			return -ETIMEDOUT;
		}
	}
	else {
		down( &channel->read_done );
	}
//	printk( "[%s](%d) Got  Read_Done... ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->rx_count );

	retval = channel->rx_count;

//	printk( "[%s](%d) Done. ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->rx_count );

	return retval;
}

static void if_hsi_read_done( struct hsi_device *dev, unsigned int size )
{
	struct if_hsi_channel *channel = NULL;
	
//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ dev->n_ch ];
//	printk( "[%s](%d) ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, size );

	spin_lock( &channel->rx_state_sem );
	channel->rx_state &= ~HSI_CHANNEL_RX_STATE_READING;
	spin_unlock( &channel->rx_state_sem );
	
	channel->rx_count = 4 * size;
	
//	printk( "[%s](%d) Call event ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->rx_count );

	up( &channel->read_done );
}

static int if_hsi_write( int ch, u32 *data, unsigned int size, const char *func )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	u32 tx_state = 0;
	
//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ ch ];
//	printk( "[%s](%d) ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, size );

	channel->tx_data = data;
	if( size % 4 )
		size += ( 4 - ( size % 4 ) );
	channel->tx_count = size;
//	printk( "[%s](%d) ch : %d, count : %d\n", __func__, __LINE__, channel->channel_id, channel->tx_count );

	spin_lock( &channel->tx_state_sem );
	tx_state = channel->tx_state;
	spin_unlock( &channel->tx_state_sem );

	if( tx_state & HSI_CHANNEL_TX_STATE_WRITING ) {
		printk( "[%s](%d) Write Still Peding. ch : %d, tx_state : %08x\n", __func__, __LINE__, channel->channel_id, tx_state );
		return -EBUSY;
	}

	spin_lock( &channel->tx_state_sem );
	channel->tx_state |= HSI_CHANNEL_TX_STATE_WRITING;
	spin_unlock( &channel->tx_state_sem );

	retval = hsi_write( channel->dev, channel->tx_data, channel->tx_count / 4 );
	if( retval < 0 ) {
		printk( "[%s](%d) hsi_write(ch:%d) fail : %d\n", __func__, __LINE__, channel->channel_id, retval );

		spin_lock( &channel->tx_state_sem );
		channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
		spin_unlock( &channel->tx_state_sem );
		
		return retval;
	}
//	printk( "[%s](%d) hsi_write(ch:%d) Done : %d\n", __func__, __LINE__, channel->channel_id, retval );

//	printk( "[%s](%d) Wait Write_Done... ch : %d\n", __func__, __LINE__, channel->channel_id );
	if( down_timeout( &channel->write_done, HSI_WRITE_DONE_TIMEOUT ) ) {
		printk( "[%s][%s](%d) Error TimeOut Write_Done... ch : %d\n", __func__, func, __LINE__, channel->channel_id );

		spin_lock( &channel->tx_state_sem );
		tx_state = channel->tx_state;
		spin_unlock( &channel->tx_state_sem );

		if( tx_state & HSI_CHANNEL_TX_STATE_WRITING ) {
			hsi_write_cancel( channel->dev );
			printk( "[%s](%d) hsi_write_cancel Done\n", __func__, __LINE__ );
		}

		spin_lock( &channel->tx_state_sem );
		channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;		
		spin_unlock( &channel->tx_state_sem );
		
		return -ETIMEDOUT;
	}
//	printk( "[%s](%d) Got  Write_Done... ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->tx_count );
	
	retval = channel->tx_count;

//	printk( "[%s](%d) Done. ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->tx_count );

	return retval;
}

static void if_hsi_write_done( struct hsi_device *dev, unsigned int size )
{
	struct if_hsi_channel *channel = NULL;
	
//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ dev->n_ch ];
//	printk( "[%s](%d) ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, size );

	spin_lock( &channel->tx_state_sem );
	channel->tx_state &= ~HSI_CHANNEL_TX_STATE_WRITING;
	spin_unlock( &channel->tx_state_sem );
	
	channel->tx_count = 4 * size;
	
//	printk( "[%s](%d) Call event ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->tx_count );

	up( &channel->write_done );
}

static void if_hsi_port_event( struct hsi_device *dev, unsigned int event, void *arg )
{
	int retval = 0;
	int acwake_level = 0;
//	printk( "[%s](%d) event : %d\n", __func__, __LINE__, event );

	switch( event ) {
		case HSI_EVENT_BREAK_DETECTED :
			printk( "[%s](%d) HSI_EVENT_BREAK_DETECTED : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );
			
			break;

		case HSI_EVENT_HSR_DATAAVAILABLE :
			printk( "[%s](%d) HSI_EVENT_HSR_DATAAVAILABLE : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );
			
			break;

		case HSI_EVENT_CAWAKE_UP :
			cawake_level = 1;

//			if( hsi_channles[ dev->n_ch ].opened ) {
//				if( if_hsi_set_wakeline( dev->n_ch, 1 ) < 0 ) {
//					printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
//				}
//			}

			if( dev->n_ch == HSI_CONTROL_CHANNEL ) {
//				printk( "[%s](%d) HSI_EVENT_CAWAKE_UP : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );

#ifdef USE_WAKELOCK_TO_CTRL_WAKELINE
#ifdef CONFIG_HAS_WAKELOCK
				if( !wake_lock_active( &ipc_hsi_wlock ) ) {
					//omap_pm_set_max_mpu_wakeup_lat( &pm_qos_handle, 7 );

					wake_lock( &ipc_hsi_wlock );
//					printk( "[%s](%d) WAKE LOCK setting done.\n",__func__, __LINE__ );
				}
#endif
#endif

#if 0
//				omap_pm_set_max_mpu_wakeup_lat( &pm_qos_handle, 7 );
				if( hsi_channles[ HSI_CONTROL_CHANNEL ].opened ) {
					retval = hsi_ioctl( hsi_channles[ HSI_CONTROL_CHANNEL ].dev, 
					                    HSI_IOCTL_SET_ACREADY_NORMAL, NULL );
					if( retval < 0 ) {
						printk( "[%s](%d) HSI_IOCTL_SET_ACREADY_NORMAL setting fail : %d\n",
							    __func__, __LINE__, retval );
					}
				}
#endif

			}
			
			break;

		case HSI_EVENT_CAWAKE_DOWN :
			cawake_level = 0;
			
			if( dev->n_ch == HSI_CONTROL_CHANNEL ) {
//				printk( "[%s](%d) HSI_EVENT_CAWAKE_DOWN : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );

				if( hsi_channles[ HSI_CONTROL_CHANNEL ].opened ) {
					retval = hsi_ioctl( hsi_channles[ HSI_CONTROL_CHANNEL ].dev, HSI_IOCTL_GET_ACWAKE, &acwake_level );
					if( retval < 0 ) {
						printk( "[%s](%d) HSI_IOCTL_GET_ACWAKE(%d) setting fail : %d\n",__func__, __LINE__, acwake_level, retval );
					}
					else {
						if( !acwake_level ) {
//							printk( "[%s](%d) HSI_IOCTL_GET_ACWAKE = %d\n",__func__, __LINE__, acwake_level );

#if 0
							retval = hsi_ioctl( hsi_channles[ HSI_CONTROL_CHANNEL ].dev, 
							                    HSI_IOCTL_SET_ACREADY_SAFEMODE, NULL );
							if( retval < 0 ) {
								printk( "[%s](%d) HSI_IOCTL_SET_ACREADY_SAFEMODE setting fail : %d\n",
									    __func__, __LINE__, retval );
							}
//							omap_pm_set_max_mpu_wakeup_lat( &pm_qos_handle, -1 );
#endif
			
#ifdef USE_WAKELOCK_TO_CTRL_WAKELINE
#ifdef CONFIG_HAS_WAKELOCK
							if( wake_lock_active( &ipc_hsi_wlock ) ) {
								//omap_pm_set_max_mpu_wakeup_lat( &pm_qos_handle, -1 );

								wake_unlock( &ipc_hsi_wlock );
//								printk( "[%s](%d) WAKE UNLOCK setting done.\n",__func__, __LINE__ );
							}
#endif
#endif
						}
					}
				}
			}
		
			break;

		case HSI_EVENT_ERROR :
			printk( "[%s](%d) HSI_EVENT_ERROR : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );

			if( hsi_channles[ dev->n_ch ].opened )
				printk( "[%s](%d) dump ch = %d, data = 0x%08x\n", __func__, __LINE__, dev->n_ch, *hsi_channles[ dev->n_ch ].rx_data );
			
			break;

		default :
			printk( "[%s](%d) Unknown event : %d, ch : %d\n", __func__, __LINE__, event, dev->n_ch );
			
			break;
	}
}

static void hsi_acwake_down_timer_func( unsigned long data )
{
	int i = 0;
	struct if_hsi_channel *channel = NULL;
	u32 tx_step = 0, rx_step = 0;
	u32 tx_state = 0;
	
//	printk( "[%s](%d)\n", __func__, __LINE__ );

	for( i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++ ) {
		channel = &hsi_channles[ i ];

		spin_lock( &channel->tx_step_sem );
		tx_step = channel->tx_step;
		spin_unlock( &channel->tx_step_sem );
		if( i == HSI_CONTROL_CHANNEL )
			tx_step = HSI_LL_TX_STATE_IDLE;

		spin_lock( &channel->rx_step_sem );
		rx_step = channel->rx_step;
		spin_unlock( &channel->rx_step_sem );
		if( i == HSI_CONTROL_CHANNEL )
			rx_step = HSI_LL_RX_STATE_IDLE;

		spin_lock( &channel->tx_state_sem );
		tx_state = channel->tx_state;
		spin_unlock( &channel->tx_state_sem );
		
		if( !( tx_state & HSI_CHANNEL_TX_STATE_WRITING ) && ( tx_step == HSI_LL_TX_STATE_IDLE ) 
					&& ( rx_step == HSI_LL_RX_STATE_IDLE ) ) {
			if( if_hsi_set_wakeline( channel->channel_id, 0 ) < 0 ) {
				printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
			}
		}
		else {
			if( timer_pending( &hsi_acwake_down_timer ) )
				del_timer( &hsi_acwake_down_timer );
			mod_timer( &hsi_acwake_down_timer, jiffies + HSI_ACWAKE_DOWN_TIMEOUT );
		}
	}
}

static int hsi_decode_command( u32 *cmd_data, u32 *cmd, u32 *ch, u32 *param )
{
	int retval = 0;
	u32 data = *cmd_data;
	u8 lrc_cal, lrc_act;
	u8 val1, val2, val3;

	*cmd = ( ( data & 0xF0000000 ) >> 28 );

//	printk( "[%s](%d) cmd : %08x\n", __func__, __LINE__, *cmd );
	
	switch( *cmd ) {
		case HSI_LL_MSG_BREAK :
			printk("[%s](%d) Command MSG_BREAK Received.\n",
						__func__, __LINE__);


			ipc_hsi_dump_cmd();
			
			retval = -1;
			break;
	
		case HSI_LL_MSG_OPEN_CONN :
			*ch = ( ( data & 0x0F000000 ) >> 24 );
			*param = ( ( data & 0x00FFFF00 ) >> 8 );

			val1 = ( ( data & 0xFF000000 ) >> 24 );
			val2 = ( ( data & 0x00FF0000 ) >> 16 );
			val3 = ( ( data & 0x0000FF00 ) >>  8 );
			lrc_act = ( data & 0x000000FF );
			lrc_cal = val1 ^ val2 ^ val3;
			if( lrc_cal != lrc_act ) {
				printk( "[%s](%d) CAL is broken.\n", __func__, __LINE__ );
				retval = -1;
			}
			
			break;

		case HSI_LL_MSG_CONN_READY :
		case HSI_LL_MSG_CONN_CLOSED :
		case HSI_LL_MSG_CANCEL_CONN :
		case HSI_LL_MSG_NAK :
			*ch = ( ( data & 0x0F000000 ) >> 24 );
			
			break;

		case HSI_LL_MSG_ACK :
			*ch = ( ( data & 0x0F000000) >> 24 );
			*param = ( data & 0x00FFFFFF );
			
			break;
	
		case HSI_LL_MSG_CONF_RATE :
			*ch = ( ( data & 0x0F000000 ) >> 24 );
			*param = ( ( data & 0x0F000000 ) >> 24 );
			
			break;

		case HSI_LL_MSG_OPEN_CONN_OCTET :
			*ch = ( ( data & 0x0F000000 ) >> 24 );
			*param = ( data & 0x00FFFFFF );
			
			break;

		case HSI_LL_MSG_ECHO :
		case HSI_LL_MSG_INFO_REQ :
		case HSI_LL_MSG_INFO :
		case HSI_LL_MSG_CONFIGURE :
		case HSI_LL_MSG_ALLOCATE_CH :
		case HSI_LL_MSG_RELEASE_CH :
		case HSI_LL_MSG_INVALID :
			printk( "[%s](%d) Invalid command received : %08x\n", __func__, __LINE__, *cmd );
			*cmd = HSI_LL_MSG_INVALID;
			*ch  = HSI_LL_INVALID_CHANNEL;
			retval = -1;
			
			break;
	}
	return retval;
}

static int hsi_protocol_send_command( u32 cmd, int ch, u32 param );
static int hsi_rx_stm( u32 cmd, u32 ch, u32 param )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	u32 tx_step = 0, rx_step = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ ch ];
//	printk( "[%s](%d) ch : %d\n", __func__, __LINE__, channel->channel_id );

	switch( cmd ){
		case HSI_LL_MSG_OPEN_CONN :
			printk( "[%s](%d) ERROR... OPEN_CONN Not supported. Should use OPEN_CONN_OCTECT instead.\n", __func__, __LINE__ );
			
			break;
	
		case HSI_LL_MSG_ECHO :
			printk( "[%s](%d) ERROR... HSI_LL_MSG_ECHO not supported.\n", __func__, __LINE__ ); 
			
			break;

		case HSI_LL_MSG_CONN_CLOSED :
			spin_lock( &channel->tx_step_sem );
			tx_step = channel->tx_step;
			spin_unlock( &channel->tx_step_sem );
			
			switch( tx_step ) {
				case HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED :
//					printk( "[%s](%d) Received CONN_CLOSED. ch-> %d\n", __func__, __LINE__, ch );

					up( &channel->close_conn_done );
					
					break;

				default:
					printk( "[%s](%d) Wrong STATE for CONN_CLOSED. tx_step : %d\n", __func__, __LINE__, tx_step );

					break;
			}
			
			break;
	
		case HSI_LL_MSG_CANCEL_CONN :
			printk( "[%s](%d) Received CANCEL_CONN\n", __func__, __LINE__ );
			
			break;

		case HSI_LL_MSG_ACK:
			spin_lock(&channel->tx_step_sem);
			tx_step = channel->tx_step;
			spin_unlock(&channel->tx_step_sem);
			
			switch (tx_step) {
			case HSI_LL_TX_STATE_WAIT_FOR_ACK:
			case HSI_LL_TX_STATE_NACK:
				/* printk( "[%s](%d) ACK received. ch : %d\n",
					__func__, __LINE__, channel->channel_id ); */

				spin_lock(&channel->tx_step_sem);
				channel->tx_step = HSI_LL_TX_STATE_TX;
				spin_unlock(&channel->tx_step_sem);

				channel->got_nack = 0;
				up(&channel->ack_done);

				break;

			default:
				printk("[%s](%d) Wrong STATE for HSI_LL_MSG_ACK. tx_step : %d\n",
						__func__, __LINE__, tx_step);
				break;
			}
			break;

		case HSI_LL_MSG_NAK:
			spin_lock(&channel->tx_step_sem);
			tx_step = channel->tx_step;
			spin_unlock(&channel->tx_step_sem);
			
			switch (tx_step) {
			case HSI_LL_TX_STATE_WAIT_FOR_ACK:
				/* printk("[%s](%d) NACK received. ch : %d\n",
					__func__, __LINE__, channel->channel_id); */

				spin_lock(&channel->tx_step_sem);
				channel->tx_step = HSI_LL_TX_STATE_NACK;
				spin_unlock(&channel->tx_step_sem);

				channel->got_nack = 1;
				up(&channel->ack_done);

				break;

			default:
				printk("[%s](%d) Wrong STATE for HSI_LL_MSG_NAK. tx_step : %d\n",
					__func__, __LINE__, tx_step);
				break;
			}
			break;

		case HSI_LL_MSG_CONF_RATE:
			// TODO: Set Conf Rate
			
			printk( "[%s](%d) CONF_RATE Received.\n", __func__, __LINE__ );
			
			break;

		case HSI_LL_MSG_OPEN_CONN_OCTET :
			spin_lock(&channel->rx_step_sem);
			rx_step = channel->rx_step;
			spin_unlock(&channel->rx_step_sem);
			
			switch (rx_step) {
			case HSI_LL_RX_STATE_IDLE:
				/* printk("[%s](%d) OPEN_CONN_OCTET received, ch-> %d\n",
						__func__, __LINE__, channel->channel_id); */

				retval = hsi_protocol_send_command(
					HSI_LL_MSG_ACK,channel->channel_id, param);
				if (retval < 0) {
					printk("[%s](%d) hsi_protocol_send_command HSI_LL_MSG_ACK fail : %d\n", 
						__func__, __LINE__, retval);

					spin_lock(&channel->rx_step_sem);
					channel->rx_step = HSI_LL_RX_STATE_IDLE;
					spin_unlock(&channel->rx_step_sem);

					return retval;
				}

				channel->rx_count = param;
				/* printk("[%s](%d) hsi_protocol_send_command HSI_LL_MSG_ACK Done : %d, ch : %d, len : %d\n",
					__func__, __LINE__, retval, channel->channel_id,
							channel->rx_count); */

				spin_lock(&channel->rx_step_sem);
				channel->rx_step = HSI_LL_RX_STATE_RX;
				spin_unlock(&channel->rx_step_sem);

				up(&channel->open_ack_done);
				break;

			case HSI_LL_RX_STATE_SEND_CONN_CLOSED:
				/* printk("[%s](%d) OPEN_CONN_OCTET in invalid state, ch : %d, Current State -> %d\n", 
					__func__, __LINE__, channel->channel_id, rx_step); */
				
				retval = hsi_protocol_send_command(
					HSI_LL_MSG_NAK, channel->channel_id, param);
				if (retval < 0) {
					printk("[%s](%d) hsi_protocol_send_command HSI_LL_MSG_NAK fail : %d\n",
						__func__, __LINE__, retval);
					return retval;
				}
				nak_count++;
				
				/* printk("[%s](%d) hsi_protocol_send_command HSI_LL_MSG_NAK Done : %d, ch : %d\n",
					__func__, __LINE__, retval, channel->channel_id); */
				break;

			default:
				printk( "[%s](%d) OPEN_CONN_OCTET received in wrong state : %d\n",
						__func__, __LINE__, rx_step);
				break;
			}
			
			break;
		
		default :
			printk( "[%s](%d) Invalid Command encountered in rx_state() : %x\n", __func__, __LINE__, cmd );

			break;
	}

	return retval;
}

static int read_command_thread( void *data )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	u32 command = 0;
	u32 cmd, ch, param;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	cmd = 0;
	ch = 0;
	param = 0;

	channel = &hsi_channles[ HSI_CONTROL_CHANNEL ];
	printk( "[%s](%d) ch : %d\n", __func__, __LINE__, channel->channel_id );

SILENT_RESET :

	printk( "[%s] wait read_cmd_thread_start complete.\n", __func__ );
	wait_for_completion( &read_cmd_thread_start );
	printk( "[%s] read_cmd_thread_start completed.\n", __func__ );

	while( !kthread_should_stop() ) {
		
		retval = if_hsi_read( HSI_CONTROL_CHANNEL, &command, 4, __func__, HSI_READ_TIMEOUT_DISABLE );
		if( retval < 0 ) {
			printk( "[%s](%d) if_hsi_read fail : %d\n", __func__, __LINE__, retval );			
			continue;
		}

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) read_cmd_thread_start CP Restart.\n", __LINE__ );
			
			init_completion( &read_cmd_thread_start );
			goto SILENT_RESET;
		}

		rx_save_cmd[ rx_save_cmd_p ] = command;
		rx_save_cmd_time[ rx_save_cmd_p ] = sched_clock();
		rx_save_cmd_p++;
		if( rx_save_cmd_p >= 30 ) rx_save_cmd_p = 0;

//		printk( "[%s](%d) if_hsi_read done : %08x\n", __func__, __LINE__, command );

		retval = hsi_decode_command( &command, &cmd, &ch, &param );
		if( retval < 0 ) {
			printk( "[%s](%d) hsi_decode_command fail : %d, cmd : %08x\n", __func__, __LINE__, retval, cmd );
		}
		else {
			retval = hsi_rx_stm( cmd, ch, param );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_rx_stm fail : %d, cmd : %08x\n", __func__, __LINE__, retval, cmd );
			}
		}
		
	}

	return retval;
}

static int write_command_thread( void *data )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	int temp_in = 0;
	u32 command = 0;
	int fail_count = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ HSI_CONTROL_CHANNEL ];
	printk( "[%s](%d) ch : %d\n", __func__, __LINE__, channel->channel_id );

SILENT_RESET :

	printk( "[%s] wait write_cmd_thread_start complete.\n", __func__ );
	wait_for_completion( &write_cmd_thread_start );
	printk( "[%s] write_cmd_thread_start completed.\n", __func__ );

	sema_init( &write_cmd_sem, 0 );
	spin_lock_init( &write_cmd_lock );
	write_cmd_in = 0;
	write_cmd_out = 0;

	while( !kthread_should_stop() ) {

		down( &write_cmd_sem );
//		printk( "[%s](%d) Got write_cmd_sem event.\n", __func__, __LINE__ );

		if( cp_restart ) {
//			printk( "[IPC_HSI] (%d) write_cmd_thread_start CP Restart.\n", __LINE__ );
			
			init_completion( &write_cmd_thread_start );
			goto SILENT_RESET;
		}

		if( if_hsi_set_wakeline( HSI_CONTROL_CHANNEL, 1 ) < 0 ) {
			printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
		}

		if( timer_pending( &hsi_acwake_down_timer ) )
			del_timer( &hsi_acwake_down_timer );
		mod_timer( &hsi_acwake_down_timer, jiffies + HSI_ACWAKE_DOWN_TIMEOUT );

		spin_lock( &write_cmd_lock );
		temp_in = write_cmd_in;
		spin_unlock( &write_cmd_lock );
		
		while( temp_in != write_cmd_out ) {
			command = write_cmd[ write_cmd_out ];
//			printk( "[%s](%d) write_cmd exist : %d, %08x\n", __func__, __LINE__, temp_count, command );

			retval = if_hsi_write( HSI_CONTROL_CHANNEL, &command, 4, __func__ );
			if( retval < 0 ) {
				printk( "[%s](%d) if_hsi_write fail : %d\n", __func__, __LINE__, retval );

				if( cp_restart ) {
//					printk( "[IPC_HSI] (%d) write_cmd_thread_start CP Restart.\n", __LINE__ );
					
					init_completion( &write_cmd_thread_start );
					goto SILENT_RESET;
				}

				fail_count++;
				if( fail_count > 5 ) {
					if( write_cmd_out >= 99 ) write_cmd_out = 0;
					else write_cmd_out++;
					fail_count = 0;
				}
			}
			else {
				if( write_cmd_out >= 99 ) write_cmd_out = 0;
				else write_cmd_out++;
				fail_count = 0;
				
				tx_save_cmd[ tx_save_cmd_p ] = command;
				tx_save_cmd_time[ tx_save_cmd_p ] = sched_clock();
				tx_save_cmd_p++;
				if( tx_save_cmd_p >= 30 ) tx_save_cmd_p = 0;
				
//				printk( "[%s](%d) write_cmd Done : %08x\n", __func__, __LINE__, command );
			}
		}

	}

	return retval;
}

static int hsi_init_handshake( int mode )
{
	int retval = 0, i = 0;
	struct hst_ctx tx_config;
	struct hsr_ctx rx_config;

	printk( "[%s](%d) mode : %d\n", __func__, __LINE__, mode );

	switch( mode ) {
		case HSI_INIT_MODE_NORMAL :
			if( timer_pending( &hsi_acwake_down_timer ) )
				del_timer( &hsi_acwake_down_timer );
			
			for( i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++ ) {
				sema_init( &hsi_channles[ i ].read_done, 0 );
				sema_init( &hsi_channles[ i ].write_done, 0 );
				sema_init( &hsi_channles[ i ].ack_done, 0 );
				sema_init( &hsi_channles[ i ].close_conn_done, 0 );
				sema_init( &hsi_channles[ i ].open_ack_done, 0 );

				spin_lock_init( &hsi_channles[ i ].tx_state_sem );
				spin_lock_init( &hsi_channles[ i ].rx_state_sem );

				spin_lock_init( &hsi_channles[ i ].tx_step_sem );
				spin_lock_init( &hsi_channles[ i ].rx_step_sem );

				hsi_channles[ i ].rx_state = 0;
				hsi_channles[ i ].tx_state = 0;
				hsi_channles[ i ].got_nack = 0;

#if 0
				if( hsi_channles[ i ].opened ) {
					retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_SW_RESET, NULL );
					if( retval < 0 ) {
						printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET fail : %d\n", __func__, __LINE__, retval );
						return retval;
					}
//					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET Done : %d\n", __func__, __LINE__, retval );
				}
#endif

				retval = if_hsi_closechannel( &hsi_channles[ i ] );
				if( retval < 0 ) {
					printk( "[%s](%d) if_hsi_closechannel fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) if_hsi_closechannel Done : %d\n", __func__, __LINE__, retval );

				retval = if_hsi_openchannel( &hsi_channles[ i ] );
				if( retval < 0 ) {
					printk( "[%s](%d) if_hsi_openchannel fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) if_hsi_openchannel Done : %d\n", __func__, __LINE__, retval );

				retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_GET_TX, &tx_config );
				if( retval < 0 ) {
					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX Done : %d\n", __func__, __LINE__, retval );

				tx_config.mode = 2;
				tx_config.divisor = 0;
				tx_config.channels = HSI_MAX_CHANNELS;

				retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_SET_TX, &tx_config );
				if( retval < 0 ) {
					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX Done : %d\n", __func__, __LINE__, retval );

				retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_GET_RX, &rx_config );
				if( retval < 0 ) {
					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX Done : %d\n", __func__, __LINE__, retval );

				rx_config.mode = 2;
				rx_config.divisor = 0;
				rx_config.channels = HSI_MAX_CHANNELS;

				retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_SET_RX, &rx_config );
				if( retval < 0 ) {
					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
//				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX Done : %d\n", __func__, __LINE__, retval );
			}

			complete_all( &write_cmd_thread_start );
			complete_all( &read_cmd_thread_start );

			printk( "[%s](%d) num of channels : %d, Handshake Done : %d\n", __func__, __LINE__, i, retval );
			
			break;

		case HSI_INIT_MODE_FLASHLESS_BOOT :
			sema_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].read_done, 0 );
			sema_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].write_done, 0 );

			spin_lock_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].tx_state_sem );
			spin_lock_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].rx_state_sem );
			hsi_channles[ HSI_FLASHLESS_CHANNEL ].rx_state = 0;
			hsi_channles[ HSI_FLASHLESS_CHANNEL ].tx_state = 0;

			for( i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++ ) {
				
#if 0
				if( hsi_channles[ i ].opened ) {
					retval = hsi_ioctl( hsi_channles[ i ].dev, HSI_IOCTL_SW_RESET, NULL );
					if( retval < 0 ) {
						printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET fail : %d\n", __func__, __LINE__, retval );
						return retval;
					}
//					printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SW_RESET Done : %d\n", __func__, __LINE__, retval );
				}
#endif

				retval = if_hsi_closechannel( &hsi_channles[ i ] );
				if( retval < 0 ) {
					printk( "[%s](%d) if_hsi_closechannel fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
	//			printk( "[%s](%d) if_hsi_closechannel Done : %d\n", __func__, __LINE__, retval );
			}
			
			retval = if_hsi_openchannel( &hsi_channles[ HSI_FLASHLESS_CHANNEL ] );
			if( retval < 0 ) {
				printk( "[%s](%d) if_hsi_openchannel fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) if_hsi_openchannel Done : %d\n", __func__, __LINE__, retval );

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_GET_TX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX Done : %d\n", __func__, __LINE__, retval );

			tx_config.mode = 2;
			tx_config.channels = 1;
			tx_config.divisor = 0;

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_SET_TX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX Done : %d\n", __func__, __LINE__, retval );

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_GET_RX, &rx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX Done : %d\n", __func__, __LINE__, retval );

			rx_config.mode = 2;
			rx_config.channels = 1;
			rx_config.divisor = 0;

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_SET_RX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX Done : %d\n", __func__, __LINE__, retval );

			if( if_hsi_set_wakeline( HSI_FLASHLESS_CHANNEL, 1 ) < 0 ) {
				printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
			}
			
			break;

		case HSI_INIT_MODE_CP_RAMDUMP :
			sema_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].read_done, 0 );
			sema_init( &hsi_channles[ HSI_FLASHLESS_CHANNEL ].write_done, 0 );

			for( i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++ ) {
				retval = if_hsi_closechannel( &hsi_channles[ i ] );
				if( retval < 0 ) {
					printk( "[%s](%d) if_hsi_closechannel fail : %d\n", __func__, __LINE__, retval );
					return retval;
				}
	//			printk( "[%s](%d) if_hsi_closechannel Done : %d\n", __func__, __LINE__, retval );
			}
			
			retval = if_hsi_openchannel( &hsi_channles[ HSI_FLASHLESS_CHANNEL ] );
			if( retval < 0 ) {
				printk( "[%s](%d) if_hsi_openchannel fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) if_hsi_openchannel Done : %d\n", __func__, __LINE__, retval );

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_GET_TX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_TX Done : %d\n", __func__, __LINE__, retval );

			tx_config.mode = 2;
			tx_config.channels = 1;
			tx_config.divisor = 0;

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_SET_TX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_TX Done : %d\n", __func__, __LINE__, retval );

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_GET_RX, &rx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_GET_RX Done : %d\n", __func__, __LINE__, retval );

			rx_config.mode = 2;
			rx_config.channels = 1;
			rx_config.divisor = 0;

			retval = hsi_ioctl( hsi_channles[ HSI_FLASHLESS_CHANNEL ].dev, HSI_IOCTL_SET_RX, &tx_config );
			if( retval < 0 ) {
				printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX fail : %d\n", __func__, __LINE__, retval );
				return retval;
			}
//			printk( "[%s](%d) hsi_ioctl HSI_IOCTL_SET_RX Done : %d\n", __func__, __LINE__, retval );

			if( if_hsi_set_wakeline( HSI_FLASHLESS_CHANNEL, 1 ) < 0 ) {
				printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
			}
			
			break;
	}

	return retval;
}

static u32 hsi_protocol_create_cmd( u32 cmd_type, int channel, void *arg )
{
	u32 command = 0;
	unsigned int size = 0, lcr = 0, role = 0, echo_params = 0, baud_rate = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	switch( cmd_type ) {
		case HSI_LL_MSG_BREAK :
			command = 0;

			break;

		case HSI_LL_MSG_OPEN_CONN :
			size = *( unsigned int * )arg;
			
			command = ( ( HSI_LL_MSG_OPEN_CONN & 0x0000000F ) << 28 ) |
					( ( channel & 0x000000FF ) << 24 ) |( ( size & 0x0000FFFF ) << 8 );

			lcr = ( ( command & 0xFF000000 ) >> 24 ) ^ ( ( command & 0x00FF0000 ) >> 16 ) ^ ( ( command & 0x0000FF00 ) >>  8 );

			command = command | ( lcr & 0x000000FF );
			
			break;

		case HSI_LL_MSG_CONN_READY :
			command = ( ( HSI_LL_MSG_CONN_READY & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 );
			
			break;

		case HSI_LL_MSG_CONN_CLOSED :
			command = ( ( HSI_LL_MSG_CONN_CLOSED & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 );

			break;

		case HSI_LL_MSG_CANCEL_CONN :
			role = *( unsigned int * )arg;

			command = ( ( HSI_LL_MSG_CANCEL_CONN & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 ) |
					( ( role & 0x000000FF) << 16 );
			
			break;

		case HSI_LL_MSG_ACK :
			echo_params = *( unsigned int * )arg;

			command = ( ( HSI_LL_MSG_ACK & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 ) |
					( ( echo_params & 0x00FFFFFF ) );

			break;

		case HSI_LL_MSG_NAK :
			command = ( ( HSI_LL_MSG_NAK & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 );

			break;

		case HSI_LL_MSG_CONF_RATE :
			baud_rate = *(unsigned int*)arg;

			command = ( ( HSI_LL_MSG_CONF_RATE & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF) << 24 ) |
					( ( baud_rate & 0x00FFFFFF ) );

			break;

		case HSI_LL_MSG_OPEN_CONN_OCTET :
			size = *( unsigned int * )arg;

			command = ( ( HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F ) << 28 ) |( ( channel & 0x000000FF ) << 24 ) |
					( ( size & 0x00FFFFFF ) );

			break;

		case HSI_LL_MSG_ECHO :
		case HSI_LL_MSG_INFO_REQ :
		case HSI_LL_MSG_INFO :
		case HSI_LL_MSG_CONFIGURE :
		case HSI_LL_MSG_ALLOCATE_CH :
		case HSI_LL_MSG_RELEASE_CH :
		case HSI_LL_MSG_INVALID :
			printk( "[%s](%d) Invalid command : %08x\n", __func__, __LINE__, cmd_type );
			command = 0xFFFFFFFF;
			
			break;
	}

//	printk( "[%s](%d) hsi_protocol_create_cmd Done : %08x\n", __func__, __LINE__, command );

	return command;
}


static int hsi_protocol_send_command( u32 cmd, int ch, u32 param )
{
	int retval = 0;
	u32 command = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	command = hsi_protocol_create_cmd( cmd, ch, &param );
	if( command == 0xFFFFFFFF ) {
		printk( "[%s](%d) hsi_protocol_create_cmd fail : %08x\n", __func__, __LINE__, command );
		return -EIO;
	}
//	printk( "[%s](%d) hsi_protocol_create_cmd Done : %08x\n", __func__, __LINE__, command );

#if 0
	if( if_hsi_set_wakeline( HSI_CONTROL_CHANNEL, 1 ) < 0 ) {
		printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
	}

	if( timer_pending( &hsi_acwake_down_timer ) )
		del_timer( &hsi_acwake_down_timer );
	mod_timer( &hsi_acwake_down_timer, jiffies + HSI_ACWAKE_DOWN_TIMEOUT );
	
	retval = if_hsi_write( HSI_CONTROL_CHANNEL, &command, 4, __func__ );
	if( retval < 0 ) {
		printk( "[%s](%d) if_hsi_write fail : %d\n", __func__, __LINE__, retval );
	}
	else {
		tx_save_cmd[ tx_save_cmd_p ] = command;
		tx_save_cmd_time[ tx_save_cmd_p ] = sched_clock();
		tx_save_cmd_p++;
		if( tx_save_cmd_p >= 30 ) tx_save_cmd_p = 0;

		return retval;
	}
#endif

	spin_lock( &write_cmd_lock );
	write_cmd[ write_cmd_in ] = command;
	if( write_cmd_in >= 99 ) write_cmd_in = 0;
	else write_cmd_in++;
	spin_unlock( &write_cmd_lock );

//	printk( "[%s](%d) save command Done.\n", __func__, __LINE__ );

	up( &write_cmd_sem );	

	return retval;
}

static int hsi_protocol_read( int ch, u32 *data, u32 *size )
{
	int retval = 0, read_size = 0;
	struct if_hsi_channel *channel = NULL;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ ch ];
//	printk( "[%s](%d) ch : %d\n", __func__, __LINE__, channel->channel_id );

	channel->rx_data = data;

	spin_lock( &channel->rx_step_sem );
	channel->rx_step = HSI_LL_RX_STATE_IDLE;
	spin_unlock( &channel->rx_step_sem );

//	printk( "[%s](%d) Wait OPEN ACK...\n", __func__, __LINE__ );
	down( &channel->open_ack_done );
//	printk( "[%s](%d) Got OPEN ACK... ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, channel->rx_count );

#if 0
	spin_lock( &channel->rx_step_sem );
	channel->rx_step = HSI_LL_RX_STATE_RX;
	spin_unlock( &channel->rx_step_sem );
#endif

	*size = channel->rx_count;

	retval = if_hsi_read( channel->channel_id, channel->rx_data, channel->rx_count, __func__, HSI_READ_TIMEOUT_ENABLE );
	if( retval < 0 ) {
		printk("[%s](%d) ch : %d, if_hsi_read fail : %d\n", __func__, __LINE__,
					channel->channel_id, retval);		
		return retval;
	}

	rx_data_time[ channel->channel_id ][ rx_data_time_p[ channel->channel_id ] ] = sched_clock();
	rx_data_time_p[ channel->channel_id ]++;
	if( rx_data_time_p[ channel->channel_id ] >= 30 ) rx_data_time_p[ channel->channel_id ] = 0;
	
//	printk( "[%s](%d) ch : %d, if_hsi_read Done : %d\n", __func__, __LINE__, channel->channel_id, retval );
	read_size = retval;

	spin_lock( &channel->rx_step_sem );
	channel->rx_step = HSI_LL_RX_STATE_SEND_CONN_CLOSED;
	spin_unlock( &channel->rx_step_sem );

	retval = hsi_protocol_send_command( HSI_LL_MSG_CONN_CLOSED, channel->channel_id, channel->rx_count );
	if( retval < 0 ) {
		printk( "[%s](%d) hsi_protocol_send_command HSI_LL_MSG_CONN_CLOSED fail : %d\n", 
			__func__, __LINE__, retval );
		
		return retval;
	}
//	printk( "[%s](%d) hsi_protocol_send_command HSI_LL_MSG_CONN_CLOSED Done : %d\n", __func__, __LINE__, retval );

//	printk( "[%s](%d) ch : %d, hsi_protocol_read Done : %d\n", __func__, __LINE__, channel->channel_id, read_size );

	return read_size;
}

static int hsi_protocol_write( int ch, u32 *data, unsigned int size )
{
	int retval = 0;
	struct if_hsi_channel *channel = NULL;
	u32 tx_step = 0;
	int error_retry_cnt = 0;
	
//	printk( "[%s](%d)\n", __func__, __LINE__ );

	channel = &hsi_channles[ ch ];
//	printk( "[%s](%d) ch : %d, len : %d\n", __func__, __LINE__, channel->channel_id, size );

	spin_lock( &channel->tx_step_sem );
	tx_step = channel->tx_step;
	spin_unlock( &channel->tx_step_sem );

	if( tx_step != HSI_LL_TX_STATE_IDLE ) {
		printk( "[%s](%d) channel : %d is stil writing. tx_step : %d\n", __func__, __LINE__, channel->channel_id, tx_step );
		return -EBUSY;
	}

	if( if_hsi_set_wakeline( channel->channel_id, 1 ) < 0 ) {
		printk( "[%s](%d) if_hsi_set_wakeline fail.\n", __func__, __LINE__ );
	}

	if( timer_pending( &hsi_acwake_down_timer ) )
		del_timer( &hsi_acwake_down_timer );
	mod_timer( &hsi_acwake_down_timer, jiffies + HSI_ACWAKE_DOWN_TIMEOUT );

	spin_lock( &channel->tx_step_sem );
	channel->tx_step = HSI_LL_TX_STATE_WAIT_FOR_ACK;
	spin_unlock( &channel->tx_step_sem );
	
	channel->tx_data = data;
	channel->tx_count = size;

RESEND_OPEN_CMD :

	channel->got_nack = 0;
	
	retval = hsi_protocol_send_command( HSI_LL_MSG_OPEN_CONN_OCTET, channel->channel_id, channel->tx_count );
	if( retval < 0 ) {
		printk( "[%s](%d) hsi_protocol_send_command HSI_LL_MSG_OPEN_CONN_OCTET fail : %d\n", 
			__func__, __LINE__, retval );

		if_hsi_set_wakeline(channel->channel_id, 0);

		spin_lock(&channel->tx_step_sem);
		channel->tx_step = HSI_LL_TX_STATE_IDLE;
		spin_unlock(&channel->tx_step_sem);

		return retval;
	}
//	printk( "[%s](%d) hsi_protocol_send_command HSI_LL_MSG_OPEN_CONN_OCTET Done : %d\n", __func__, __LINE__, retval );

//	printk( "[%s](%d) Wait ACK...\n", __func__, __LINE__ );
	if( down_timeout( &channel->ack_done, HSI_ACK_DONE_TIMEOUT ) ) {
		printk("[%s](%d) Error TimeOut Ack_Done... ch : %d, now time : %llu\n",
			__func__, __LINE__, channel->channel_id, sched_clock());

#if 0
		printk("MPU_ENABLE_IRQ0 0x%0x\n",omap_readl(0x4A058808));
		printk("MPU_STATUS_IRQ0 0x%0x\n",omap_readl(0x4A05880C));
		printk("HSR_BUFSTATE_P1 0x%0x\n",omap_readl(0x4A05A810));
		printk("HSR_RXSTATE_P1 0x%0x\n",omap_readl(0x4A05A80c));
		printk("HSR_MAPPING0 0x%0x\n",omap_readl(0x4A05A900));
#endif

		if_hsi_set_wakeline(channel->channel_id, 0);

		if (!cp_restart) {
			error_retry_cnt++;
			if (error_retry_cnt < 10) {
				if_hsi_set_wakeline(channel->channel_id, 1);
				goto RESEND_OPEN_CMD;
			}
			else {
				cp_restart = 1;
				ipc_hsi_clear_all_vbuff(ipc_hsi);
				ipc_hsi_dump_cmd();
				
				printk("[%s](%d) force phone reset.\n", __func__, __LINE__);
				modemctl_force_silent_reset();
			}
		}

		spin_lock(&channel->tx_step_sem);
		channel->tx_step = HSI_LL_TX_STATE_IDLE;
		spin_unlock(&channel->tx_step_sem);

		return -ETIMEDOUT;
	}
//	printk( "[%s](%d) Got ACK...\n", __func__, __LINE__ );

	if( channel->got_nack ) {
//		printk( "[%s](%d) Got NACK : %d\n", __func__, __LINE__, channel->got_nack );

		msleep( 1 );
//		printk( "[%s](%d) Resend Open Cmd.\n", __func__, __LINE__ );

		goto RESEND_OPEN_CMD;
	}
//	printk( "[%s](%d) Got ACK : %d\n", __func__, __LINE__, channel->got_nack );

	spin_lock( &channel->tx_step_sem );
	channel->tx_step = HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED;
	spin_unlock( &channel->tx_step_sem );

	retval = if_hsi_write( channel->channel_id, channel->tx_data, channel->tx_count, __func__ );
	if( retval < 0 ) {
		printk("[%s](%d) if_hsi_write fail : %d\n", __func__, __LINE__, retval);

		if_hsi_set_wakeline(channel->channel_id, 0);

		spin_lock(&channel->tx_step_sem);
		channel->tx_step = HSI_LL_TX_STATE_IDLE;
		spin_unlock(&channel->tx_step_sem);

		return retval;
	}

	tx_data_time[ channel->channel_id ][ tx_data_time_p[ channel->channel_id ] ] = sched_clock();
	tx_data_time_p[ channel->channel_id ]++;
	if( tx_data_time_p[ channel->channel_id ] >= 30 ) tx_data_time_p[ channel->channel_id ] = 0;
	
//	printk( "[%s](%d) if_hsi_write Done : %d\n", __func__, __LINE__, retval );

//	printk( "[%s](%d) Wait CLOSE CONN...\n", __func__, __LINE__ );
	if( down_timeout( &channel->close_conn_done, HSI_CLOSE_CONN_DONE_TIMEOUT ) ) {
		printk("[%s](%d) Error TimeOut Close_Conn_Done... ch : %d, now time : %llu\n",
			__func__, __LINE__, channel->channel_id, sched_clock());

		if_hsi_set_wakeline(channel->channel_id, 0);
		ipc_hsi_dump_cmd();

		spin_lock(&channel->tx_step_sem);
		channel->tx_step = HSI_LL_TX_STATE_IDLE;
		spin_unlock(&channel->tx_step_sem);

		return -ETIMEDOUT;
	}
//	printk( "[%s](%d) Got CLOSE CONN...\n", __func__, __LINE__ );

	spin_lock( &channel->tx_step_sem );
	channel->tx_step = HSI_LL_TX_STATE_IDLE;
	spin_unlock( &channel->tx_step_sem );
	
	return retval;
}

void ipc_hsi_dump_cmd( void )
{
	int i;
	
	printk( "======= dump of history command =======\n" );
	printk( "index : rx_cmd : rx_cmd_time : tx_cmd : tx_cmd_time : rx_data_time : tx_data_time\n" );
	for( i = 0; i < 30; i++ ) {
		printk( "%02d : %08x : %llu : %08x : %llu : %llu : %llu\n",
			i, rx_save_cmd[ i ], rx_save_cmd_time[ i ], tx_save_cmd[ i ], tx_save_cmd_time[ i ], rx_data_time[ 1 ][ i ], tx_data_time[ 1 ][ i ] );
	}
	printk( "rx_cmd_p : %d, tx_cmd_p : %d, rx_data_time_p : %d, tx_data_time_p : %d\n", 
		rx_save_cmd_p, tx_save_cmd_p, rx_data_time_p[ 1 ], tx_data_time_p[ 1 ] );
	printk( "======= dump of history command =======\n" );
}
EXPORT_SYMBOL( ipc_hsi_dump_cmd );

void  ipc_hsi_restart_hsi( void )
{
	int i = 0;
	
	printk( "Phone Restart HSI Init.\n" );

	if( cp_flashless_boot_done ) {
		cp_restart = 1;
		cp_flashless_boot_done = 0;
		cp_flow_control_stop_transfer = 0;

		up( &transfer_event_sem );
		up( &write_cmd_sem );

		for( i = 0; i < HSI_NUM_OF_USE_CHANNELS; i++ ) {
			up( &hsi_channles[ i ].open_ack_done );
			up( &hsi_channles[ i ].read_done );
		}

		ipc_hsi_clear_all_vbuff( ipc_hsi );
	}
}
EXPORT_SYMBOL( ipc_hsi_restart_hsi );

static int __devinit if_hsi_probe( struct hsi_device *dev )
{
	int retval = 0;
	int port = 0;
	unsigned long *address;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	for( port = 0; port < HSI_MAX_PORTS; port++ ) {
		if( if_hsi_driver.ch_mask[ port ] )
			break;
	}
	address = &if_hsi_driver.ch_mask[ port ];
//	printk( "[%s](%d) take ch_mask(%d) : %lx\n", __func__, __LINE__, port, if_hsi_driver.ch_mask[ port ] );

	if( test_bit( dev->n_ch, address ) && ( dev->n_p == port ) ) {
//		printk( "[%s](%d) Regestering callback functions\n", __func__, __LINE__ );
		
		hsi_set_read_cb( dev, if_hsi_read_done );
		hsi_set_write_cb( dev, if_hsi_write_done );
		hsi_set_port_event_cb( dev, if_hsi_port_event );

		hsi_channles[ dev->n_ch ].dev = dev;
		hsi_channles[ dev->n_ch ].rx_state = 0;
		hsi_channles[ dev->n_ch ].tx_state = 0;
		hsi_channles[ dev->n_ch ].rx_step = HSI_LL_RX_STATE_CLOSED;
		hsi_channles[ dev->n_ch ].tx_step = HSI_LL_TX_STATE_CLOSED;
		hsi_channles[ dev->n_ch ].rx_count = 0;
		hsi_channles[ dev->n_ch ].tx_count = 0;
		hsi_channles[ dev->n_ch ].rx_data = NULL;
		hsi_channles[ dev->n_ch ].tx_data = NULL;

		retval = 0;
//		printk( "[%s](%d) Probe Done.\n", __func__, __LINE__ );
	}

	return retval;
}

static int __devexit if_hsi_remove( struct hsi_device *dev )
{
	int retval = 0;
	int port = 0;
	unsigned long *address;

	printk( "[%s](%d)\n", __func__, __LINE__ );

	for( port = 0; port < HSI_MAX_PORTS; port++ ) {
		if( if_hsi_driver.ch_mask[ port ] )
			break;
	}
	address = &if_hsi_driver.ch_mask[ port ];
	printk( "[%s](%d) take ch_mask(%d) : %lx\n", __func__, __LINE__, port, if_hsi_driver.ch_mask[ port ] );

	if( test_bit( dev->n_ch, address ) && ( dev->n_p == port ) ) {
		printk( "[%s](%d) Delete callback functions\n", __func__, __LINE__ );
		
		hsi_set_read_cb( dev, NULL );
		hsi_set_write_cb( dev, NULL );
		hsi_set_port_event_cb( dev, NULL );

		hsi_channles[ dev->n_ch ].dev = NULL;
		hsi_channles[ dev->n_ch ].rx_state = HSI_CHANNEL_RX_STATE_UNAVAIL;
		hsi_channles[ dev->n_ch ].tx_state = HSI_CHANNEL_TX_STATE_UNAVAIL;
		
		retval = 0;
		printk( "[%s](%d) Remove Done.\n", __func__, __LINE__ );
	}

	return retval;
}

static int __init if_hsi_init( void )
{
	int retval = 0;
	int i = 0;

//	printk( "[%s](%d)\n", __func__, __LINE__ );

	for( i = 0; i < HSI_MAX_PORTS; i++ )
		if_hsi_driver.ch_mask[ i ] = 0;
	printk( "[%s](%d) ch_mask setting Done.\n", __func__, __LINE__ );

	for( i = 0; i < HSI_MAX_CHANNELS; i++ ) {
		hsi_channles[ i ].dev = NULL;
		hsi_channles[ i ].opened = 0;
		hsi_channles[ i ].tx_state = HSI_CHANNEL_TX_STATE_UNAVAIL;
		hsi_channles[ i ].rx_state = HSI_CHANNEL_RX_STATE_UNAVAIL;
		hsi_channles[ i ].tx_state = HSI_LL_TX_STATE_CLOSED;
		hsi_channles[ i ].rx_state = HSI_LL_RX_STATE_CLOSED;
		hsi_channles[ i ].channel_id = i;
	}
	printk( "[%s](%d) hsi_channles setting Done.\n", __func__, __LINE__ );

	if_hsi_driver.ch_mask[ 0 ] = CHANNEL_MASK;

	setup_timer( &hsi_acwake_down_timer, hsi_acwake_down_timer_func, 0 );
	printk( "[%s](%d) hsi_acwake_down_timer setup Done.\n", __func__, __LINE__ );

	retval = hsi_register_driver( &if_hsi_driver );
	if( retval ) {
		printk( "[%s](%d) hsi_register_driver ERROR : %d\n", __func__, __LINE__, retval );
		return retval;
	}
	printk( "[%s](%d) hsi_register_driver setting Done.\n", __func__, __LINE__ );

	return retval;
}

static void __devexit if_hsi_exit( void )
{
	int i = 0;
	
	for( i = 0; i < HSI_MAX_CHANNELS; i++ ) {
		if( hsi_channles[ i ].opened ) {
			if_hsi_closechannel( &hsi_channles[ i ] );
			if_hsi_set_wakeline( i, 0 );
		}
	}

	hsi_unregister_driver( &if_hsi_driver );

	printk( "[%s](%d) if_hsi_exit Done.\n",__func__, __LINE__ );
}

static int __init ipc_hsi_init( void )
{
	int retval = 0;
	
	printk("[%s]\n",__func__);

	retval = platform_driver_register( &ipc_hsi_platform_driver );
	if( retval < 0 ) {
		printk( "[%s] platform_driver_register ERROR : %d\n", __func__, retval );

		goto exit;
	}

	// creat work queue thread
	ipc_hsi_wq = create_singlethread_workqueue( "ipc_hsi_wq" );
	//ipc_hsi_wq = create_workqueue( "ipc_hsi_wq" );
	if( !ipc_hsi_wq ) {
		printk( "[%s] get workqueue thread fail\n", __func__ );

		retval = -ENOMEM;
		goto exit;
	}

	retval = if_hsi_init();
	if( retval < 0 ) {
		printk( "[%s](%d) if_hsi_init fail : %d\n", __func__, __LINE__, retval );
		goto exit;
	}
	printk( "[%s](%d) if_hsi_init Done.\n", __func__, __LINE__ );
	
#ifdef USE_WAKELOCK_TO_CTRL_WAKELINE
	wake_lock_init( &ipc_hsi_wlock, WAKE_LOCK_SUSPEND, "samsung-ipc-hsi" );
#endif

	printk( "[%s](%d) init Done.\n", __func__, __LINE__ );

	return 0;

exit :
	return retval;
}

static void __exit ipc_hsi_exit( void )
{
	printk( "[%s]\n", __func__ );
	
	platform_driver_unregister( &ipc_hsi_platform_driver );

	if( ipc_hsi_wq ) {
		destroy_workqueue( ipc_hsi_wq );
	}

	if_hsi_exit();
}

module_init( ipc_hsi_init );
module_exit( ipc_hsi_exit );

MODULE_LICENSE( "GPL" );
MODULE_AUTHOR( "Wonhee Seo <wonhee48.seo@samsung.com>" );
MODULE_DESCRIPTION( "IpcHsi driver" );
