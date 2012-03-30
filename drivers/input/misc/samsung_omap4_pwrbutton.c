/*
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/board.h>
#include <plat/mux.h>
#include <mach/sec_debug.h>

#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#define OMAP_GPIO_KEY_PWRON	3
#define OMAP_GPIO_KEY_HOME	176
#endif

extern struct class *sec_class;

static int __devinit power_key_driver_probe(struct platform_device *plat_dev);
static irqreturn_t powerkey_press_handler(int irq_num, void * dev);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
static irqreturn_t homekey_press_handler(int irq_num, void * dev);
int home_key_press_status = 0;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
static irqreturn_t vol_upkey_press_handler(int irq_num, void * dev);
static irqreturn_t vol_dnkey_press_handler(int irq_num, void * dev);
#endif

ssize_t sec_power_key_pressed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned int key_press_status=0;
  key_press_status = gpio_get_value(OMAP_GPIO_KEY_PWRON);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  key_press_status &= ~0x2;
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
  if(system_rev >= 5) {
    key_press_status |= ((!gpio_get_value(OMAP_GPIO_KEY_HOME)) << 1);
  }
  else {
    key_press_status |= ((!gpio_get_value(OMAP_GPIO_LEGACY_KEY_HOME)) << 1);
  }
#else
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_KEY_HOME)) << 1);
#endif
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  key_press_status &= ~0xC;
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_KEY_VOL_UP)) << 2);
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_KEY_VOL_DOWN)) << 3);
#endif

  return sprintf(buf, "%u\n", key_press_status);
}
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
static DEVICE_ATTR(sec_key_pressed, S_IRUGO, sec_power_key_pressed_show, NULL);
#else
static DEVICE_ATTR(sec_power_key_pressed, S_IRUGO, sec_power_key_pressed_show, NULL);
#endif
  
static irqreturn_t powerkey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status=0; 

  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = gpio_get_value(OMAP_GPIO_KEY_PWRON);

  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  
  input_report_key(ip_dev,KEY_POWER,key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*  dev_dbg(ip_dev->dev.parent,"Sent KEY_POWER event = %d\n",key_press_status);
  printk("[PWR-KEY] KEY_POWER event = %d\n",key_press_status); */
#endif
  return IRQ_HANDLED;
}

#ifdef CONFIG_INPUT_GPIO_HOME_KEY
static irqreturn_t homekey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }

#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
  if(system_rev >= 5) {
    home_key_press_status = !gpio_get_value(OMAP_GPIO_KEY_HOME);
  }
  else {
    home_key_press_status = !gpio_get_value(OMAP_GPIO_LEGACY_KEY_HOME);
  }
#else
  home_key_press_status = !gpio_get_value(OMAP_GPIO_KEY_HOME);
#endif
  
  if( home_key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  input_report_key(ip_dev,KEY_HOME,home_key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*  dev_dbg(ip_dev->dev.parent,"Sent KEY_HOME event = %d\n",home_key_press_status);
  printk("Sent KEY_HOME event = %d\n",home_key_press_status); */
#endif
  return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
static irqreturn_t vol_upkey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status=0;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = !gpio_get_value(OMAP_GPIO_KEY_VOL_UP);
  
  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  input_report_key(ip_dev, KEY_VOLUMEUP, key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*  dev_dbg(ip_dev->dev.parent,"Sent KEY_VOL_UP event = %d\n", key_press_status);
  printk("Sent KEY_VOL_UP event = %d\n", key_press_status); */
  if(key_press_status && home_key_press_status)
  {
    printk(KERN_ERR "%s : Force Crash by keypad\n", __func__);
    /* FIXME: sec_debug_check_crash_key is not implemented completely, yet!! */
    sec_debug_check_crash_key(KEY_VOLUMEUP, key_press_status);
  }
#endif
  return IRQ_HANDLED;
}

static irqreturn_t vol_dnkey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status=0;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = !gpio_get_value(OMAP_GPIO_KEY_VOL_DOWN);
  
  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  input_report_key(ip_dev, KEY_VOLUMEDOWN, key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*  dev_dbg(ip_dev->dev.parent,"Sent KEY_VOL_DN event = %d\n", key_press_status);
  printk("Sent KEY_VOL_DN event = %d\n", key_press_status); */
#endif
  return IRQ_HANDLED;
}
#endif

static int __devinit power_key_driver_probe(struct platform_device *plat_dev)
{
  struct input_dev *power_key=NULL;
  int pwr_key_irq=-1, err=0;
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  int home_key_irq = -1;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  int vol_up_key_irq = -1;
  int vol_dn_key_irq = -1;
#endif
  struct device *sec_power_key;

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
#ifdef CONFIG_MACH_SAMSUNG_T1
  if(system_rev >= 3)  {
    sec_power_key= device_create(sec_class, NULL, 0, NULL, "sec_key");
    if (!sec_power_key) {
      printk("Failed to create sysfs(sec_key)!\n");
      return -ENOMEM;
    }
    if (device_create_file(sec_power_key, &dev_attr_sec_key_pressed)< 0)
      printk("Failed to create device file(%s)!\n", dev_attr_sec_key_pressed.attr.name);
  }
#else
  sec_power_key= device_create(sec_class, NULL, 0, NULL, "sec_key");
  if (!sec_power_key) {
    printk("Failed to create sysfs(sec_key)!\n");
    return -ENOMEM;
  }
  if (device_create_file(sec_power_key, &dev_attr_sec_key_pressed)< 0)
    printk("Failed to create device file(%s)!\n", dev_attr_sec_key_pressed.attr.name);
#endif
#else
  sec_power_key= device_create(sec_class, NULL, 0, NULL, "sec_power_key");
  if (!sec_power_key) {
    printk("Failed to create sysfs(sec_power_key)!\n");
    return -ENOMEM;
  }
  if (device_create_file(sec_power_key, &dev_attr_sec_power_key_pressed)< 0)
    printk("Failed to create device file(%s)!\n", dev_attr_sec_power_key_pressed.attr.name);
#endif

  gpio_set_debounce(OMAP_GPIO_KEY_PWRON, 7936);
  pwr_key_irq = platform_get_irq(plat_dev, 0);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
  if(system_rev >= 5) {
    gpio_set_debounce(OMAP_GPIO_KEY_HOME, 7936);
  }
  else {
    gpio_set_debounce(OMAP_GPIO_LEGACY_KEY_HOME, 7936);
  }
#else
  gpio_set_debounce(OMAP_GPIO_KEY_HOME, 7936);
#endif
  home_key_irq = platform_get_irq(plat_dev, 1);
  if(pwr_key_irq <= 0 || home_key_irq <= 0){
    dev_err(&plat_dev->dev,"failed to map the power key to an IRQ %d & %d\n",pwr_key_irq, home_key_irq);
    err = -ENXIO;
    return err;
  }
#else
  if(pwr_key_irq <= 0 ){
    dev_err(&plat_dev->dev,"failed to map the power key to an IRQ %d\n",pwr_key_irq);
    err = -ENXIO;
    return err;
  }
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  gpio_set_debounce(OMAP_GPIO_KEY_VOL_UP, 7936);
  gpio_set_debounce(OMAP_GPIO_KEY_VOL_DOWN, 7936);
  vol_up_key_irq = platform_get_irq(plat_dev, 2);
  vol_dn_key_irq = platform_get_irq(plat_dev, 3);
  if(vol_up_key_irq <= 0 || vol_dn_key_irq <= 0){
    dev_err(&plat_dev->dev,"failed to map the volume keys to an IRQ %d & %d\n",vol_up_key_irq, vol_dn_key_irq);
    err = -ENXIO;
    return err;
  }
#endif
  power_key = input_allocate_device();
  if(!power_key)
  {
    dev_err(&plat_dev->dev,"failed to allocate an input devd %d \n",pwr_key_irq);
    err = -ENOMEM;
    return err;
  }
  err = request_irq(pwr_key_irq, &powerkey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                    "sec_power_key",power_key);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
  err |= request_irq(home_key_irq, &homekey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                    "sec_power_key",power_key);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
  if(system_rev >= 5) {
    err |= request_irq(home_key_irq, &homekey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                      "sec_power_key",power_key);
  }
  else {
    err |= request_irq(home_key_irq, &homekey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                      "sec_power_key",power_key);
  }
#else
  err |= request_irq(home_key_irq, &homekey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    "sec_power_key",power_key);
#endif
#endif
  if(err) {
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
    dev_err(&plat_dev->dev,"failed to request an IRQ handler for num %d & %d\n",pwr_key_irq, home_key_irq);
#else
    dev_err(&plat_dev->dev,"failed to request an IRQ handler for num %d\n",pwr_key_irq);
#endif
    goto free_input_dev;
  }

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  err = request_irq(vol_up_key_irq, &vol_upkey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                    "sec_power_key",power_key);
  err |= request_irq(vol_dn_key_irq, &vol_dnkey_press_handler ,IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
                    "sec_power_key",power_key);
  if(err) {
    dev_err(&plat_dev->dev,"failed to request an IRQ handler for num %d & %d\n",vol_up_key_irq, vol_dn_key_irq);
    goto free_input_dev;
  }
#endif

  dev_dbg(&plat_dev->dev,"\n Power Key Drive:Assigned IRQ num %d SUCCESS \n",pwr_key_irq);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  dev_dbg(&plat_dev->dev,"\n HOME Key Drive:Assigned IRQ num %d SUCCESS \n",home_key_irq);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  dev_dbg(&plat_dev->dev,"\n VOL_UP Key Drive:Assigned IRQ num %d SUCCESS \n",vol_up_key_irq);
  dev_dbg(&plat_dev->dev,"\n VOL_DN Key Drive:Assigned IRQ num %d SUCCESS \n",vol_dn_key_irq);
#endif

  /* register the input device now */
  input_set_capability(power_key, EV_KEY, KEY_POWER);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  input_set_capability(power_key, EV_KEY, KEY_HOME);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  input_set_capability(power_key, EV_KEY, KEY_VOLUMEUP);
  input_set_capability(power_key, EV_KEY, KEY_VOLUMEDOWN);
#endif

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  power_key->name = "sec_key";
  power_key->phys = "sec_key/input0";
#else
  power_key->name = "sec_power_key";
  power_key->phys = "sec_power_key/input0";
#endif
  power_key->dev.parent = &plat_dev->dev;
  platform_set_drvdata(plat_dev, power_key);

  err = input_register_device(power_key);
  if (err) {
    dev_err(&plat_dev->dev, "power key couldn't be registered: %d\n", err);
    goto release_irq_num;
  }
 
  return 0;

release_irq_num:
  free_irq(pwr_key_irq,NULL); //pass devID as NULL as device registration failed 
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  free_irq(home_key_irq, NULL);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  free_irq(vol_up_key_irq, NULL);
  free_irq(vol_dn_key_irq, NULL);
#endif

free_input_dev:
  input_free_device(power_key);

return err;

}

static int __devexit power_key_driver_remove(struct platform_device *plat_dev)
{
  struct input_dev *ip_dev= platform_get_drvdata(plat_dev);
  int pwr_key_irq=0;
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  int home_key_irq=0;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  int vol_up_key_irq=0;
  int vol_dn_key_irq=0;
#endif

  pwr_key_irq = platform_get_irq(plat_dev,0);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  home_key_irq = platform_get_irq(plat_dev,1);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  vol_up_key_irq = platform_get_irq(plat_dev,2);
  vol_dn_key_irq = platform_get_irq(plat_dev,3);
#endif

  free_irq(pwr_key_irq,ip_dev);
#ifdef CONFIG_INPUT_GPIO_HOME_KEY
  free_irq(home_key_irq,ip_dev);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  free_irq(vol_up_key_irq,ip_dev);
  free_irq(vol_dn_key_irq,ip_dev);
#endif
  input_unregister_device(ip_dev);

  return 0;
}

struct platform_driver power_key_driver_t = {
	.probe		= &power_key_driver_probe,
	.remove		= __devexit_p(power_key_driver_remove),
	.driver		= {
		.name	= "sec_power_key",
		.owner	= THIS_MODULE,
	},
};

static int __init power_key_driver_init(void)
{
	printk("Registered POWER KEY Driver\n");
	return platform_driver_register(&power_key_driver_t);
}
module_init(power_key_driver_init);

static void __exit power_key_driver_exit(void)
{
	platform_driver_unregister(&power_key_driver_t);
}
module_exit(power_key_driver_exit);

MODULE_ALIAS("platform:power key driver");
MODULE_DESCRIPTION("Samsung OMAP4 power key driver");
MODULE_LICENSE("GPL");

