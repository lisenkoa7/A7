/*
 * AT070TNA2 LVDS panel support
 *
 * Copyright (C) 2011 Foxconn Technology Group
 * Author: James
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
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
#include <linux/platform_device.h>

#include <plat/gpio.h>
#include <plat/mux.h>
#include <asm/mach-types.h>
#include <plat/control.h>

#include <plat/display.h>

/*+++++20110902, JimmySu add board ID*/
#include <linux/wakelock.h>
#include <linux/share_region.h>
static int at070tna2_hw_version = 3;
/*-----20110902, JimmySu add board ID*/

#define LCD_XRES		1024
#define LCD_YRES		600

#define LCD_PIXCLOCK_MIN      	40800 /* Min. PIX Clock is 40.8MHz */
#define LCD_PIXCLOCK_TYP      	41000 /* Typ. PIX Clock is 51.2MHz */ /*20111122, JimmySu optimize panel parameter 43Mhz -> 41Mhz*/
#define LCD_PIXCLOCK_MAX      	67200 /* Max. PIX Clock is 67.2MHz */

#define LCD_PIXEL_CLOCK		LCD_PIXCLOCK_TYP

#define EDP_LCD_LVDS_SHTDN_GPIO 	37
#define EDP_LCD_PWR_EN					109
#define EDP_LCD_LVDS_CLKSEL			110

#define EDP_LCD_BL_SW					62

/*touch*/
#define EP5A_EVT_TOUCH_RST 	157
#define EP5A_EVT_TOUCH_EN 	159

struct delayed_work delay_work_enabletouch;
struct wake_lock touch_wake_lock;
/*touch*/

/*++++ 20110902, JimmySu add board ID*/
//&*&*&*BC1_110817: fix the issue that user can see screen switch when device resume
struct delayed_work delay_work_enablepanel;
struct wake_lock panel_wake_lock;
//&*&*&*BC2_110817: fix the issue that user can see screen switch when device resume

#ifdef CONFIG_LEDS_OMAP_DISPLAY
extern void omap_enable_pwm(void);
extern void omap_set_primary_brightness(u8 brightness); //20111101, JimmySu trun-on backlight when panel probe
#endif
/*----20110902, JimmySu add board ID*/
/*20111122, JimmySu optimize panel parameter for power consumption*/
static struct omap_video_timings at070tna2_panel_timings = {
	/* 1024 x 600 @ 60 Hz   */
	.x_res          = LCD_XRES,
	.y_res          = LCD_YRES,
	.pixel_clock    = LCD_PIXEL_CLOCK,
 	.hfp			= 10,
	.hbp			= 160,
	.hsw			= 20,
	.vfp			= 4,
	.vbp			= 10,
	.vsw			= 4,

};

//&*&*&*BC1_110817: fix the issue that user can see screen switch when device resume
static void omap_delay_work_enablepanel(struct work_struct *work)
{
	if (at070tna2_hw_version == BOARD_ID_EVT1){
		#ifdef CONFIG_LEDS_OMAP_DISPLAY
		omap_enable_pwm();
		#endif
	}else{
		gpio_direction_output(EDP_LCD_BL_SW, 1);	
	}
}
//&*&*&*BC2_110817: fix the issue that user can see screen switch when device resume

static void omap_delay_work_enabletouch(struct work_struct *work)
{
/*20111122, JimmySu fine tuning reset timing*/
	gpio_direction_output(EP5A_EVT_TOUCH_EN, 1);	
	gpio_direction_output(EP5A_EVT_TOUCH_RST, 0);	
	mdelay(10);
	gpio_direction_output(EP5A_EVT_TOUCH_RST, 1);	
	mdelay(300);
}

static int at070tna2_panel_probe(struct omap_dss_device *dssdev)
{
/*+++++20110902, JimmySu add board ID*/
	int board_id;

	board_id = ep_get_hardware_id();	
	switch(board_id)
	{ 
		case BOARD_ID_EVT1: 
			at070tna2_hw_version =3;
			break;
		default:
			at070tna2_hw_version =6;
			break;
	}
/*----20110902, JimmySu add board ID*/
	printk("----------[EP5a]: %s-----------\n", __func__);
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS ;
	dssdev->panel.timings = at070tna2_panel_timings;
	dssdev->ctrl.pixel_size = 24;

//&*&*&*BC1_110817: fix the issue that user can see screen switch when device resume
	INIT_DELAYED_WORK(&delay_work_enablepanel, omap_delay_work_enablepanel);
	wake_lock_init(&panel_wake_lock, WAKE_LOCK_SUSPEND, "panel_wakelock");
//&*&*&*BC2_110817: fix the issue that user can see screen switch when device resume	

	gpio_request(EDP_LCD_LVDS_CLKSEL, "lcd LVDS CLKSEL");
	gpio_request(EDP_LCD_LVDS_SHTDN_GPIO, "lcd LVDS SHTN");
	gpio_request(EDP_LCD_PWR_EN, "lcd_panel Power_EN");

	gpio_request(EDP_LCD_BL_SW, "lcd_panel backlight switch");
	
/*touch*/
	if (at070tna2_hw_version != BOARD_ID_EVT1){
		INIT_DELAYED_WORK(&delay_work_enabletouch, omap_delay_work_enabletouch);
		wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "panel_wakelock");
		gpio_request(EP5A_EVT_TOUCH_RST, 	"touch_rst");
		gpio_request(EP5A_EVT_TOUCH_EN,		"touch_en");
	}
/*touch*/
	gpio_direction_output(EDP_LCD_LVDS_CLKSEL, 0); //20110816, modify by JimmySu to fix panel color

	omap_set_primary_brightness(100);  //20111101, JimmySu trun-on backlight when panel probe
	return 0;
}

static void at070tna2_panel_remove(struct omap_dss_device *dssdev)
{
}

static int at070tna2_panel_enable(struct omap_dss_device *dssdev)
{
	 int r = 0;

	if (at070tna2_hw_version != BOARD_ID_EVT1){
		schedule_delayed_work(&delay_work_enabletouch, 1);
	}
	
	gpio_direction_output(EDP_LCD_PWR_EN, 1);//for vdd
	
	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* Delay recommended by panel DATASHEET */
	mdelay(4);
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}
//&*&*&*BC1_110817: fix the issue that user can see screen switch when device resume
	gpio_direction_output(EDP_LCD_LVDS_SHTDN_GPIO, 1);
	schedule_delayed_work(&delay_work_enablepanel, 10);  //20111117, JimmySu modify timing 6 -> 10, for earily suspend/resume
//&*&*&*BC2_110817: fix the issue that user can see screen switch when device resume
	
	return r;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void at070tna2_panel_disable(struct omap_dss_device *dssdev)
{

	if (at070tna2_hw_version != BOARD_ID_EVT1){
		gpio_direction_output(EDP_LCD_BL_SW, 0);	
		gpio_direction_output(EP5A_EVT_TOUCH_EN, 0);		
	}

	cancel_delayed_work(&delay_work_enablepanel); //20111117, JimmySu, add cancel delay for quickly press power button

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	 
	/* Delay recommended by panel DATASHEET */
	mdelay(4);

	omapdss_dpi_display_disable(dssdev);
}

static int at070tna2_panel_suspend(struct omap_dss_device *dssdev)
{
	at070tna2_panel_disable(dssdev);
	return 0;
}

static int at070tna2_panel_resume(struct omap_dss_device *dssdev)
{
	return at070tna2_panel_enable(dssdev);
}

static struct omap_dss_driver at070tna2_driver = {
	.probe		= at070tna2_panel_probe,
	.remove		= at070tna2_panel_remove,

	.enable		= at070tna2_panel_enable,
	.disable	= at070tna2_panel_disable,
	.suspend	= at070tna2_panel_suspend,
	.resume		= at070tna2_panel_resume,

	.driver         = {
		.name   = "at070tna2_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init at070tna2_panel_drv_init(void)
{
	return omap_dss_register_driver(&at070tna2_driver);
}

static void __exit at070tna2_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&at070tna2_driver);
}

module_init(at070tna2_panel_drv_init);
module_exit(at070tna2_panel_drv_exit);
MODULE_LICENSE("GPL");
