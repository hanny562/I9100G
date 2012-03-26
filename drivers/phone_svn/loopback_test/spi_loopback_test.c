/* MIPI TEST CODE  */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/phone_svn/mipi_hsi.h>
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
#include <linux/hsi_driver_if.h>


// test
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <asm/mach-types.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/hsi_driver_if.h>
// test

#define OMAP4430_GPIO_MIPI_HSI_IPC_SLAVE_WAKEUP	53
#define OMAP4430_GPIO_MIPI_HSI_IPC_HOST_WAKEUP		52
#define OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST		56

static unsigned gpio_ipc_host_wakeup;
static unsigned gpio_ipc_slave_wakeup;
static unsigned gpio_suspend_request;

/*onedram functions */

struct resource* onedram_request_region(resource_size_t start,
		resource_size_t n, const char *name)
{
	return 0;
}
EXPORT_SYMBOL(onedram_request_region);

void onedram_release_region(resource_size_t start, resource_size_t n)
{
	return 0;
}
EXPORT_SYMBOL(onedram_release_region);

int onedram_register_handler(void (*handler)(u32, void *), void *data)
{

	return 0;
}
EXPORT_SYMBOL(onedram_register_handler);

int onedram_unregister_handler(void (*handler)(u32, void *))
{
	return 0;
}
EXPORT_SYMBOL(onedram_unregister_handler);

int onedram_read_mailbox(u32 *mb)
{
	return 0;
}
EXPORT_SYMBOL(onedram_read_mailbox);

int onedram_write_mailbox(u32 mb)
{
	return 0;
}
EXPORT_SYMBOL(onedram_write_mailbox);

int onedram_get_auth(u32 cmd)
{
	return 0;
}
EXPORT_SYMBOL(onedram_get_auth);

int onedram_put_auth(int release)
{
	return 0;
}
EXPORT_SYMBOL(onedram_put_auth);

int onedram_rel_sem(void)
{
	return 0;
}
EXPORT_SYMBOL(onedram_rel_sem);

void onedram_get_vbase(void** vbase)
{
	return 0;
}
EXPORT_SYMBOL(onedram_get_vbase);

/*onedram functions */


static void mipi_test_gpio_init( void )
{
	int err;
	err = gpio_request( gpio_ipc_host_wakeup, "IPC_HOST_WAKEUP" );
	if( err ) {
		printk( "mipi_hsi_cfg_gpio - fail to request gpio %s : %d\n", "IPC_HOST_WAKEUP", err );
	}
	else {
		gpio_direction_input( gpio_ipc_host_wakeup );
	}

	err = gpio_request( gpio_ipc_slave_wakeup, "IPC_SLAVE_WAKEUP" );
	if( err ) {
		printk( "mipi_hsi_cfg_gpio - fail to request gpio %s : %d\n", "IPC_SLAVE_WAKEUP", err );
	}
	else {
		gpio_direction_output( gpio_ipc_slave_wakeup, 0 );
	}

	err = gpio_request( gpio_suspend_request, "SUSPEND_REQUEST" );
	if( err ) {
		printk( "mipi_hsi_cfg_gpio - fail to request gpio %s : %d\n", "SUSPEND_REQUEST", err );
	}
	else {
		gpio_direction_input( gpio_suspend_request );
	}
	printk( "[MIPI_TEST](%d): %s done...\n", __LINE__, __func__ );
}



#if 0
struct pdp_header {
	u32 len;		//Data length
	u8 id;		//Channel ID
	u8 control;	//Control field
} __attribute__ ( ( packed ) );

static u32 rx_data_count = 0;
#endif

/* prot types */
struct mipi_test_channel {
	struct hsi_device *dev;
	unsigned int channel_id;
	u32 *tx_data;
	unsigned int tx_count;
	u32 *rx_data;
	unsigned int rx_count;
	unsigned int opened;
	unsigned int state;
	spinlock_t lock; /* Serializes access to channel data */
};
	
struct mipi_test_iface {
	struct mipi_test_channel channel;
	int bootstrap;
	spinlock_t lock; /* Serializes access to HSI functional interface */
};

static struct mipi_test_iface mipi_piface;

/* Test */
u32 *rx_buf, *tx_buf;
int mipi_read_test_running, mipi_write_test_running;
int hsi_test_read_resched;
int hsi_test_read_on, hsi_test_write_on;

#define MIPI_TEST_COUNT     20000



/* Read */
static int mipi_read_test(void)
{
	int ret;

	hsi_test_read_on = 1;
	ret = hsi_read( ( void * )mipi_piface.channel.dev,  rx_buf, MIPI_TEST_COUNT/4);
	if(ret < 0 ){
		printk( "[MIPI_TEST](%d): %s hsi_read error...\n", __LINE__, __func__ );		
	}
	printk( "[MIPI_TEST](%d): %s done...\n", __LINE__, __func__ );

	return 0;
}

static int hsi_test_read_stop(void)
{
	hsi_close(mipi_piface.channel.dev);
	kfree(rx_buf);

	//dev_dbg(&hsi_piface.channel.dev->device, "Ending read test.\n");
	printk( "[MIPI_TEST](%d): %s Ending read test...\n", __LINE__, __func__ );
	
	mipi_read_test_running = 0;

	return 0;
}

static int hsi_test_read_cb(void)
{
	hsi_test_read_on = 0;
	if (hsi_test_read_resched) {
		mipi_read_test();
	}
	else {
		hsi_test_read_stop();
	}
	return 0;
}


/* Write */
static int mipi_write_test(void)
{
	int ret;

	hsi_test_write_on = 1;
	ret = hsi_write( ( void * )mipi_piface.channel.dev, tx_buf, MIPI_TEST_COUNT/4 );
	if(ret < 0 ){
		printk( "[MIPI_TEST](%d): %s hsi_write error...\n", __LINE__, __func__ );		
	}
	printk( "[MIPI_TEST](%d): %s done...\n", __LINE__, __func__ );

	return 0;
}

static int hsi_test_write_stop(void)
{
	hsi_close(mipi_piface.channel.dev);
	kfree(tx_buf);

	//dev_dbg(&hsi_piface.channel.dev->device, "Ending read test.\n");
	printk( "[MIPI_TEST](%d): %s Ending write test...\n", __LINE__, __func__ );
	
	mipi_write_test_running = 0;

	return 0;
}

static int hsi_test_write_cb(void)
{
	hsi_test_read_on = 0;
	if (hsi_test_read_resched) {
		mipi_read_test();
	}
	else {
		hsi_test_read_stop();
	}
	return 0;
}

/* Test */
static int mipi_test_start(void)
{
	unsigned int i;
	int ret;

	mipi_read_test_running = 1;
	mipi_write_test_running = 1;
	
	if (tx_buf == NULL) {
		printk( "[MIPI_TEST](%d): tx_buf is NULL\n", __LINE__ );
		goto err;
	}
	
	for (i = 0; i < MIPI_TEST_COUNT/4; i++)
		tx_buf[i] = 0xaaaa0000 | (i & 0xffff);

	
	if (rx_buf == NULL) {
		printk( "[MIPI_TEST](%d): rx_buf is NULL\n", __LINE__ );
		goto err;
	}

	ret = hsi_open( (void *)mipi_piface.channel.dev);
	if ( ret < 0 ){

		printk( "[MIPI_TEST](%d): Could not open HSI channel 0\n", __LINE__ );
		goto err;
	}

	mipi_write_test();
	mipi_read_test();

err:
	return 0;

}

static int mipi_test_stop( void )
{
	hsi_close( ( void * )mipi_piface.channel.dev );
	kfree(tx_buf);
	kfree(rx_buf);
	printk( "[MIPI_TEST](%d): mipi_test end... 0\n", __LINE__ );
	mipi_read_test_running = 0;
	mipi_write_test_running = 0;
	
	return 0;
}



static int mipi_test_thread( void *data )
{
//	int ret = 0, i = 0;
	
	daemonize( "mipi_test_thread" );

	printk( "[MIPI_TEST](%d): %s start...\n", __LINE__, __func__ );

	msleep( 3000 );

	//prepare data
	tx_buf = kmalloc(MIPI_TEST_COUNT, GFP_ATOMIC);
	rx_buf = kmalloc(MIPI_TEST_COUNT, GFP_ATOMIC);
	printk( "[MIPI_TEST](%d): tx_buf, rx_buf memory allocated...\n", __LINE__ );

	memset( tx_buf, 10, MIPI_TEST_COUNT );
	memset( rx_buf, 10, MIPI_TEST_COUNT );
	printk( "[MIPI_TEST](%d): %s data initialization...\n", __LINE__, __func__ );

	// Send First Data( Handshake )


//	mod_timer( &check_speed_timer, jiffies + HZ );

	while( 1 ) {
		
		mipi_test_start();
		printk( "[MIPI_TEST](%d): %s start...\n", __LINE__, __func__ );
		msleep(500);
		msleep(500);
		msleep(500);
		msleep(500);
		msleep(500);
		

//LOOP :



//AGAIN :

		
	}
}

/*
static void mipi_test_check_speed_timer_func( unsigned long data )
{
	printk( "[MIPI_TEST] SPEED : %lu BytesPerSec.\n", rx_data_count );

	rx_data_count = 0;
	
	mod_timer( &check_speed_timer, jiffies + HZ );
}
*/


/* HSI driver required functions */

static void mipi_read_test_done(struct hsi_device *dev, unsigned int size) 
{ 
	dev_dbg(&mipi_piface.channel.dev->device, "Read complete: %d\n", size);

	if (mipi_read_test_running == 1) 
		hsi_test_read_cb();
}

static void mipi_write_test_done(struct hsi_device *dev, unsigned int size) 
{ 
	dev_dbg(&mipi_piface.channel.dev->device, "Write complete: %d\n", size);

	if (mipi_write_test_running == 1) 
		hsi_test_write_cb();
}

static void hsi_test_port_event(struct hsi_device *dev, unsigned int event, void *arg)
{
 	//dev_dbg(&hsi_piface.channel.dev->device, "Event triggered: %d\n", event);
 	printk("[MIPI_TEST](%d): %s - Event triggered: %d\n", __LINE__, __func__, event);
}

static int mipi_test_platform_probe(struct platform_device * pdev)
{
#if 0
	int r;
	struct resource *res;
	struct mipi_hsi_platform_data *pdata;

	mipi_test_gpio_init();

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	r = kernel_thread( mipi_test_thread, NULL, 0 );
	if( r < 0 ) {
		dev_err( &pdev->dev, "kernel_thread() failed : %d\n", r );
		
		goto err;
	}

	printk("[MIPI_TEST](%d): %s Done...\n", __LINE__, __func__);
	return 0;

err:
	return r;
#endif
	int r;
	int irq;
	struct mipi_hsi_platform_data *pdata;
	struct resource *res;

	printk("[%s]\n",__func__);
	pdata = pdev->dev.platform_data;
	if (!pdata || !pdata->cfg_gpio) {
		dev_err(&pdev->dev, "No platform data\n");
		r = -EINVAL;
		goto err;
	}

	gpio_ipc_host_wakeup= pdata->gpio_ipc_host_wakeup;
	gpio_ipc_slave_wakeup= pdata->gpio_ipc_slave_wakeup;
	gpio_suspend_request= pdata->gpio_suspend_request;
	dev_dbg( &pdev->dev, "(%d) gpio_ipc_host_wakeup : %d\n", __LINE__, gpio_get_value(gpio_ipc_host_wakeup) );
	dev_dbg( &pdev->dev, "(%d) gpio_ipc_slave_wakeup : %d\n", __LINE__, gpio_get_value(gpio_ipc_slave_wakeup) );
	dev_dbg( &pdev->dev, "(%d) gpio_ipc_suspend_request : %d\n", __LINE__, gpio_get_value(gpio_suspend_request) );

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err( &pdev->dev, "(%d) failed to get irq number\n", __LINE__ );
		
		r = -EINVAL;
		goto err;
	}
	irq = res->start;

	pdata->cfg_gpio();
	
/*
	r = request_irq( irq, ipc_spi_irq_handler, IRQF_TRIGGER_RISING, "IpcSpi", od );
	if (r) {
		dev_err( &pdev->dev, "(%d) Failed to allocate an interrupt: %d\n", __LINE__, irq );
		
		goto err;
	}
	od->irq = irq;

	// Init work structure
	ipc_spi_send_modem_work_data = kmalloc( sizeof( struct ipc_spi_send_modem_bin_workq_data ), GFP_ATOMIC );
	if( !ipc_spi_send_modem_work_data ) {
		dev_err( &pdev->dev, "(%d) memory alloc fail\n", __LINE__ );

		r = -ENOMEM;
		goto err;
	}

	INIT_WORK( &ipc_spi_send_modem_work_data->send_modem_w, ipc_spi_send_modem_bin );
*/
/*
	r = _register_chrdev(od);
	printk("[mipi_hsi_probe]: (%d) - debug\n", __LINE__);	//remove
	if (r) {
		dev_err( &pdev->dev, "(%d) Failed to register chrdev\n", __LINE__ );
		
		goto err;
	}

	r = sysfs_create_group(&od->dev->kobj, &ipc_spi_group);
	if (r) {
		dev_err( &pdev->dev, "(%d) Failed to create sysfs files\n", __LINE__ );
		
		goto err;
	}
	od->group = &ipc_spi_group;
*/
//	platform_set_drvdata(pdev, od);


	r = kernel_thread( mipi_test_thread, NULL, 0 );
	if( r < 0 ) {
		dev_err( &pdev->dev, "kernel_thread() failed : %d\n", r );
		
		goto err;
	}

	dev_info( &pdev->dev, "(%d) platform probe Done.\n", __LINE__ );

	printk("[%s Done...]\n",__func__);

	return 0;

err:
//	_release(od);
	return r;

}

static int mipi_test_platform_remove( struct platform_device *pdev )
{
	struct mipi_test_iface *od = platform_get_drvdata( pdev );

	platform_set_drvdata(pdev, NULL);

	// Free work queue data
	mipi_test_stop();

	return 0;
}


static struct platform_driver mipi_test_platform_driver = {
	.probe = mipi_test_platform_probe,
	.remove = __devexit_p( mipi_test_platform_remove ),
//	.suspend = ipc_spi_platform_suspend,
//	.resume = ipc_spi_platform_resume,
	.driver = {
		.name = "onedram",
	},
};


static int __devinit mipi_test_probe(struct hsi_device * dev)
{
	
       struct mipi_test_channel *channel;
	int ret = -ENXIO;

	printk("[MIPI_TEST](%d): %s - exec...\n", __LINE__, __func__);
	
//	setup_timer( &check_speed_timer, spi_loopback_test_check_speed_timer_func, 0 );
//	setup_timer( &srdy_timeout_timer, spi_loopback_test_srdy_timeout_timer_func, 0 );

	if ((dev->n_ch == 0) && (dev->n_p == 0)) {
		hsi_set_read_cb(dev, mipi_read_test_done);
		hsi_set_write_cb(dev, mipi_write_test_done);
		hsi_set_port_event_cb(dev, hsi_test_port_event);
		channel = &mipi_piface.channel;
		channel->dev = dev;
		channel->state = 0;
		ret = 0;
		printk("[MIPI_TEST](%d): %s - Binding to port 0 / channel 0\n", __LINE__, __func__);
	}

//	dev = mipi_piface.channel.dev;

	printk( "[MIPI_TEST](%d): %s done...\n", __LINE__, __func__ );

	return 0;
}

static int __devexit mipi_test_remove(struct hsi_device *dev)
{
	return 0;	
}

static struct hsi_device_driver mipi_test_device_driver = {
	.ctrl_mask = -1,
	.probe = mipi_test_probe,
	.remove = __devexit_p(mipi_test_remove),
	.driver = {
//		.name = "mipi_test"
		.name = "hsi_test"
	},
};

static int mipi_test_register(void)
{
	int ret, i;

	ret = platform_driver_register( &mipi_test_platform_driver );
	if( ret < 0 ) {
		printk( "[%s] platform_driver_register ERROR : %d\n", __func__, ret );

		goto exit;
	}
	printk("[MIPI_TEST](%d): %s-platform_driver_register Done...\n", __LINE__, __func__);

	for (i = 0; i < HSI_MAX_PORTS; i++)
		mipi_test_device_driver.ch_mask[i] = 0;

	/* select channel 0 */
	mipi_test_device_driver.ch_mask[0] = 1;

	ret = hsi_register_driver( &mipi_test_device_driver );

	if ( ret != 0) {
		printk( "[MIPI_TEST](%d): Error while registering HSI driver %d\n", __LINE__, ret );
		goto exit;
	}

	printk("[MIPI_TEST](%d): %s Done...\n", __LINE__, __func__);
	return 0;

exit:
	return ret;
}

static int __init mipi_test_init( void )
{
	int ret = 0;

	ret = mipi_test_register();

	if (ret)
		return ret;

	printk("[MIPI_TEST](%d): %s Done...\n", __LINE__, __func__);
	
	mipi_read_test_running = 0;
	mipi_write_test_running = 0;
	hsi_test_read_resched = 0;

	return 0;
}

static void __exit mipi_test_exit( void )
{
	return hsi_unregister_driver( &mipi_test_device_driver );
}

module_init( mipi_test_init );
module_exit( mipi_test_exit );
MODULE_LICENSE( "GPL" );

