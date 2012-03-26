/*
 * LCD panel driver for Samsung D10E50T6332
 *
 * Copyright (C) 2010 Samsung Electronics
 * Author: Shankar Bandal (shankar.b@samsung.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License  as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/display.h>

extern int cmc623_init(void);

struct samsung_data {
	/* XXX This regulator should actually be in SDP board file, not here,
	 * as it doesn't actually power the LCD, but something else that
	 * affects the output to LCD (I think. Somebody clarify). It doesn't do
	 * harm here, as SDP is the only board using this currently */
	struct regulator *vusim_reg;
};

static struct omap_video_timings samsung_D10E50T6332_timings = {
	.x_res = 1280,
	.y_res = 800,
	.pixel_clock	= 68940,

	.hsw		= 48, 	// h_sync_width
	.hfp		= 16,	// h_front_porch
	.hbp		= 64, 	//h_back_porch

	.vsw		= 3, 	// v_sync_width
	.vfp		= 1,	// v_front_porch
	.vbp		= 12, 	// v_back_porch

	
	/* 
	 *	For Magna D10E50T6332 TFT Display timing controller:
	 *	Signal					Min. 			Typ.			Max
	 *	Frame Frequency(TV)			815			-			850
	 *	Vertical Active Display Term(TVD) 	- 			800			-
	 *	One Line Scanning Time(TH)		1350			1408			1460
	 *	Horizontal Active Display Term(THD) 	-			1280			-
	 *
	 *	Total clocks per line (TH) = hsw + hfp + columns (THD) + hbp
	 *			     1408  = 48  + 16  + 1280          + 64
	 *						  
	 *	Total LCD lines (TV) 	  = vsw + vfp + rows (TVD)  + vbp
	 *			816	  = 3   + 1   + 800	    + 12
	 *					
	 *	From this data,
	 *	- single line takes (48 + 16 + 1280 + 64) clocks = 1408 clocks/line
	 *	- full frame takes (3 + 1 + 800 + 12) lines = 816 lines/frame
	 *	- full frame in clocks = 1408 * 816 = 1148928 clocks/frame
	 *	- 20MHz, the LCD would refresh at 20M/1148928 = 17.4Hz
	 *	- 70MHz, the LCD would refresh at 68.94M/1148928 = 60Hz
	 */

};

static int samsung_D10E50T6332_panel_probe(struct omap_dss_device *dssdev)
{
	struct samsung_data *sd;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = samsung_D10E50T6332_timings;

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);
#if 0
	sd->vusim_reg = regulator_get(&dssdev->dev, "vusim");
	if (IS_ERR(sd->vusim_reg)) {
		kfree(sd);
		pr_err("failed to get VUSIM regulator\n");
		return PTR_ERR(sd->vusim_reg);
	}
#endif
	return 0;
}

static void samsung_D10E50T6332_panel_remove(struct omap_dss_device *dssdev)
{
	struct samsung_data *sd = dev_get_drvdata(&dssdev->dev);

	//regulator_put(sd->vusim_reg);

	kfree(sd);
}

static int samsung_D10E50T6332_panel_enable(struct omap_dss_device *dssdev)
{
//	struct samsung_data *sd = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	//msleep(1);
	//regulator_enable(sd->vusim_reg);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if(r)
		    return r;
	}
	r = omapdss_dpi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DPI\n");
		return r;
	}
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;


	/* Now enable image converter */
	if(cmc623_init()< 0)
	{
		printk("[SHANKAR] %s [%d] CMC623 init failed ", __func__, __LINE__);
		return -1;
	}
	return r;
}

static void samsung_D10E50T6332_panel_disable(struct omap_dss_device *dssdev)
{
//	struct samsung_data *sd = dev_get_drvdata(&dssdev->dev);
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	omapdss_dpi_display_disable(dssdev);


	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	//regulator_disable(sd->vusim_reg);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(1);
}

static int samsung_D10E50T6332_panel_suspend(struct omap_dss_device *dssdev)
{
	samsung_D10E50T6332_panel_disable(dssdev);
	return 0;
}

static int samsung_D10E50T6332_panel_resume(struct omap_dss_device *dssdev)
{
	return samsung_D10E50T6332_panel_enable(dssdev);
}

static void samsung_D10E50T6332_panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}


static void samsung_D10E50T6332_panel_set_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
        dpi_set_timings(dssdev, timings);
}

static void samsung_D10E50T6332_panel_get_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
        *timings = dssdev->panel.timings;
}

static int samsung_D10E50T6332_panel_check_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
        return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver samsung_D10E50T6332_driver = {
	.probe		= samsung_D10E50T6332_panel_probe,
	.remove		= samsung_D10E50T6332_panel_remove,

	.enable		= samsung_D10E50T6332_panel_enable,
	.disable	= samsung_D10E50T6332_panel_disable,
	.get_resolution = samsung_D10E50T6332_panel_get_resolution,
	.suspend	= samsung_D10E50T6332_panel_suspend,
	.resume		= samsung_D10E50T6332_panel_resume,

	.set_timings	= samsung_D10E50T6332_panel_set_timings,
	.get_timings	= samsung_D10E50T6332_panel_get_timings,
	.check_timings	= samsung_D10E50T6332_panel_check_timings,

	.driver         = {
		.name   = "samsung_D10E50T6332_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init samsung_D10E50T6332_panel_drv_init(void)
{
	return omap_dss_register_driver(&samsung_D10E50T6332_driver);
}

static void __exit samsung_D10E50T6332_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&samsung_D10E50T6332_driver);
}

module_init(samsung_D10E50T6332_panel_drv_init);
module_exit(samsung_D10E50T6332_panel_drv_exit);
MODULE_DESCRIPTION("Samsung D10E50T6332 LCD timing controller driver");
MODULE_AUTHOR("Shankar Bandal<shankar.b@samsung.com>");
MODULE_LICENSE("GPL");
