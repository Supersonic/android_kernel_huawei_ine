/* Copyright (c) 2017-2018, Huawei terminal Tech. Co., Ltd. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
* GNU General Public License for more details.
*
*/

#include "hisi_fb.h"
#include "lcd_kit_common.h"
#include "lcd_kit_disp.h"
#include "lcd_kit_dbg.h"
#include <huawei_platform/log/log_jank.h>
#include "global_ddr_map.h"
#include "lcd_kit_utils.h"
#include "lcd_kit_adapt.h"
#include "lcd_kit_power.h"

static int lcd_kit_set_fastboot(struct platform_device* pdev);
static int lcd_kit_on(struct platform_device* pdev);
static int lcd_kit_off(struct platform_device* pdev);
static int lcd_kit_remove(struct platform_device* pdev);
static int lcd_kit_set_backlight(struct platform_device* pdev, uint32_t bl_level);
static int lcd_kit_esd_check(struct platform_device* pdev);
static int lcd_kit_set_display_region(struct platform_device* pdev, struct dss_rect* dirty);
static int lcd_kit_fps_scence_handle(struct platform_device* pdev, uint32_t scence);
static int lcd_kit_fps_updt_handle(struct platform_device* pdev);
static ssize_t lcd_kit_ce_mode_store(struct platform_device* pdev, const char* buf, size_t count);
static ssize_t lcd_kit_rgbw_set_func(struct hisi_fb_data_type* hisifd);
static ssize_t lcd_kit_hbm_set_func(struct hisi_fb_data_type* hisifd);
static ssize_t lcd_kit_cabc_store(struct platform_device* pdev, const char* buf, size_t count);
static ssize_t lcd_kit_color_param_get_func(struct hisi_fb_data_type* hisifd);

/*variable declare*/
static struct lcd_kit_disp_info g_lcd_kit_disp_info;
/*******************************************************************************
**hisi panel data & pinfo
*/
static struct hisi_panel_info lcd_kit_pinfo = {0};
static struct hisi_fb_panel_data lcd_kit_data = {
	.panel_info = &lcd_kit_pinfo,
	.set_fastboot = lcd_kit_set_fastboot,
	.on = lcd_kit_on,
	.off = lcd_kit_off,
	.remove = lcd_kit_remove,
	.set_backlight = lcd_kit_set_backlight,
	.esd_handle = lcd_kit_esd_check,
	.set_display_region = lcd_kit_set_display_region,
	.lcd_fps_scence_handle = lcd_kit_fps_scence_handle,
	.lcd_fps_updt_handle = lcd_kit_fps_updt_handle,
	.lcd_ce_mode_store = lcd_kit_ce_mode_store,
	.lcd_rgbw_set_func = lcd_kit_rgbw_set_func,
	.lcd_hbm_set_func  = lcd_kit_hbm_set_func,
	.lcd_color_param_get_func = lcd_kit_color_param_get_func,
	.lcd_cabc_mode_store = lcd_kit_cabc_store,
};

struct lcd_kit_disp_info *lcd_kit_get_disp_info(void)
{
	return &g_lcd_kit_disp_info;
}

static int lcd_kit_on(struct platform_device* pdev)
{
	struct hisi_fb_data_type* hisifd = NULL;
	struct hisi_panel_info* pinfo = NULL;
	int ret = LCD_KIT_OK;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	if (hisifd->aod_function) {
		LCD_KIT_INFO("AOD mode, bypass disp_kit_panel_on! \n");
		return LCD_KIT_OK;
	}

	LCD_KIT_INFO("fb%d, +!\n", hisifd->index);

	pinfo = &(hisifd->panel_info);
	if (!pinfo) {
		LCD_KIT_ERR("panel_info is NULL!\n");
		return LCD_KIT_FAIL;
	}
	switch (pinfo->lcd_init_step) {
		case LCD_INIT_POWER_ON:
			if (common_ops->panel_power_on) {
				ret = common_ops->panel_power_on((void*)hisifd);
			}
			pinfo->lcd_init_step = LCD_INIT_MIPI_LP_SEND_SEQUENCE;
			break;
		case LCD_INIT_MIPI_LP_SEND_SEQUENCE:
			/*send mipi command by low power*/
			if (common_ops->panel_on_lp) {
				ret = common_ops->panel_on_lp((void*)hisifd);
			}
			lcd_kit_read_power_status(hisifd);
			pinfo->lcd_init_step = LCD_INIT_MIPI_HS_SEND_SEQUENCE;
			break;
		case LCD_INIT_MIPI_HS_SEND_SEQUENCE:
			/*send mipi command by high speed*/
			if (common_ops->panel_on_hs) {
				ret = common_ops->panel_on_hs((void*)hisifd);
			}			
			/*record panel on time*/
			lcd_kit_disp_on_record_time();
			break;
		case LCD_INIT_NONE:
			break;
		case LCD_INIT_LDI_SEND_SEQUENCE:
			break;
		default:
			break;
	}
	LCD_KIT_INFO("fb%d, -!\n", hisifd->index);
	return ret;
}

/*
*name:lcd_kit_off
*function:power off panel
*@pdev:platform device
*/
static int lcd_kit_off(struct platform_device* pdev)
{
	struct hisi_fb_data_type* hisifd = NULL;
	struct hisi_panel_info* pinfo = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	if (hisifd->aod_function) {
		LCD_KIT_INFO("AOD mode, bypass disp_kit_panel_off! \n");
		return LCD_KIT_OK;
	}

	LCD_KIT_INFO("fb%d, +!\n", hisifd->index);
	pinfo = &(hisifd->panel_info);
	if (!pinfo) {
		LCD_KIT_ERR("panel_info is NULL!\n");
		return LCD_KIT_FAIL;
	}
	switch (pinfo->lcd_uninit_step) {
		case LCD_UNINIT_MIPI_HS_SEND_SEQUENCE:
			if (common_ops->panel_off_hs) {
				common_ops->panel_off_hs(hisifd);
			}
			pinfo->lcd_uninit_step = LCD_UNINIT_MIPI_LP_SEND_SEQUENCE;
			break;
		case LCD_UNINIT_MIPI_LP_SEND_SEQUENCE:
			if (common_ops->panel_off_lp) {
				common_ops->panel_off_lp(hisifd);
			}
			pinfo->lcd_uninit_step = LCD_UNINIT_POWER_OFF;
			break;
		case LCD_UNINIT_POWER_OFF:
			if (common_ops->panel_power_off) {
				common_ops->panel_power_off(hisifd);
			}
			break;
		default:
			break;
	}
	LCD_KIT_INFO("fb%d, -!\n", hisifd->index);
	return LCD_KIT_OK;
}

/*
*name:lcd_kit_remove
*function:panel remove
*@pdev:platform device
*/
static int lcd_kit_remove(struct platform_device* pdev)
{
	struct hisi_fb_data_type* hisifd = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (!hisifd) {
		LCD_KIT_ERR("hisifd is NULL Point!\n");
		return LCD_KIT_OK;
	}
	lcd_kit_power_finit(pdev);
	return LCD_KIT_OK;
}

static int lcd_kit_set_backlight(struct platform_device* pdev, uint32_t bl_level)
{
	int ret = LCD_KIT_OK;
	struct hisi_fb_data_type* hisifd = NULL;
	static uint32_t jank_last_bl_level = 0;
	static uint32_t bl_type;
	struct hisi_panel_info* pinfo = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	if (hisifd->aod_mode && hisifd->aod_function) {
		LCD_KIT_INFO("It is in AOD mode and should bypass lcd_kit_set_backlight! \n");
		return LCD_KIT_OK;
	}
	pinfo = &(hisifd->panel_info);
	if (NULL == pinfo) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	if (disp_info->quickly_sleep_out.support) {
		if (disp_info->quickly_sleep_out.panel_on_tag) {
			lcd_kit_disp_on_check_delay();
		}
	}
	if (jank_last_bl_level == 0 && bl_level != 0) {
		LOG_JANK_D(JLID_KERNEL_LCD_BACKLIGHT_ON, "LCD_BACKLIGHT_ON,%u", bl_level);
		jank_last_bl_level = bl_level;
	} else if (bl_level == 0 && jank_last_bl_level != 0) {
		LOG_JANK_D(JLID_KERNEL_LCD_BACKLIGHT_OFF, "LCD_BACKLIGHT_OFF");
		jank_last_bl_level = bl_level;
	}
	bl_type = lcd_kit_get_bl_set_type(pinfo);
	switch (bl_type) {
		case BL_SET_BY_PWM:
			ret = hisi_pwm_set_backlight(hisifd, bl_level);
			break;
		case BL_SET_BY_BLPWM:
			ret = lcd_kit_blpwm_set_backlight(hisifd, bl_level);
			break;
		case BL_SET_BY_MIPI:
			ret = lcd_kit_mipi_set_backlight(hisifd, bl_level);
			break;
		default:
			LCD_KIT_ERR("not support bl_type\n");
			ret = -1;
			break;
	}
	LCD_KIT_INFO("bl_type = %d, bl_level = %d\n", bl_type, bl_level);
	return ret;
}

static int lcd_kit_esd_check(struct platform_device* pdev)
{
	int ret = LCD_KIT_OK;
	struct hisi_fb_data_type* hisifd = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	mutex_lock(&disp_info->mipi_lock);
	if (common_ops->esd_handle) {
		ret = common_ops->esd_handle(hisifd);
	}
	mutex_unlock(&disp_info->mipi_lock);
	return ret;
}

static int lcd_kit_set_fastboot(struct platform_device* pdev)
{
	struct hisi_fb_data_type* hisifd = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	// backlight on
	hisi_lcd_backlight_on(pdev);
	return LCD_KIT_OK;
}


static int lcd_kit_set_display_region(struct platform_device* pdev, struct dss_rect* dirty)
{
	int ret = LCD_KIT_OK;
	struct hisi_fb_data_type* hisifd = NULL;

	if (NULL == pdev) {
		LCD_KIT_ERR("pdev is null\n");
		return LCD_KIT_FAIL;
	}

	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("hisifd is null\n");
		return LCD_KIT_FAIL;
	}

	if (common_ops->dirty_region_handle) {
		mutex_lock(&disp_info->mipi_lock);
		ret = common_ops->dirty_region_handle(hisifd, (struct region_rect*)dirty);
		mutex_unlock(&disp_info->mipi_lock);
	}
	return ret;
}

static int lcd_kit_fps_scence_handle(struct platform_device* pdev, uint32_t scence)
{
	int ret = LCD_KIT_OK;

	if (disp_info->fps.support) {
		ret = lcd_kit_updt_fps_scence(pdev, scence);
	}
	return ret;
}

static int lcd_kit_fps_updt_handle(struct platform_device* pdev)
{
	int ret = LCD_KIT_OK;
	struct hisi_fb_data_type* hisifd = NULL;

	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("hisifd is null\n");
		return LCD_KIT_FAIL;
	}
	if (disp_info->fps.support) {
		if (is_mipi_cmd_panel(hisifd)) {
			ret = lcd_kit_updt_fps(pdev);
		}
	}
	return ret;
}

static ssize_t lcd_kit_ce_mode_store(struct platform_device* pdev, const char* buf, size_t count)
{
	int ret = LCD_KIT_OK;
	unsigned long mode = 0;
	struct hisi_fb_data_type* hisifd = NULL;

	if (pdev == NULL) {
		LCD_KIT_ERR("pdev is null\n");
		return LCD_KIT_FAIL;
	}
	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("hisifd is null\n");
		return LCD_KIT_FAIL;
	}
	ret = strict_strtoul(buf, 0, &mode);
	if (ret) {
		LCD_KIT_ERR("strict_strtoul error\n");
		return ret;
	}

	hisifd->user_scene_mode = (int)mode;
	if (common_ops->set_ce_mode) {
		ret = common_ops->set_ce_mode(hisifd, mode);
	}
	return count;
}

static ssize_t lcd_kit_rgbw_set_func(struct hisi_fb_data_type* hisifd)
{
	int ret = LCD_KIT_OK;

	if (disp_info->rgbw.support) {
		ret = lcd_kit_rgbw_set_handle(hisifd);
	}
	return ret;
}

static ssize_t lcd_kit_hbm_set_func(struct hisi_fb_data_type* hisifd)
{
	int ret = LCD_KIT_OK;

	if (NULL == hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return LCD_KIT_FAIL;
	}
	if (common_ops->hbm_set_handle) {
		ret = common_ops->hbm_set_handle(hisifd, hisifd->de_info.hbm_level);
	}
	return ret;
}

static ssize_t lcd_kit_cabc_store(struct platform_device* pdev, const char* buf, size_t count)
{
	ssize_t ret = LCD_KIT_OK;
	struct hisi_fb_data_type* hisifd = NULL;
	unsigned long mode = 0;

	hisifd = platform_get_drvdata(pdev);
	if (NULL == hisifd) {
		LCD_KIT_ERR("NULL Pointer\n");
		return LCD_KIT_FAIL;
	}
	ret = strict_strtoul(buf, 0, &mode);
	if (ret) {
		LCD_KIT_ERR("invalid data!\n");
		return ret;
	}
	if (common_ops->set_cabc_mode) {
		mutex_lock(&disp_info->mipi_lock);
		common_ops->set_cabc_mode(hisifd, mode);
		mutex_unlock(&disp_info->mipi_lock);
	}
	return count;
}

static ssize_t lcd_kit_color_param_get_func(struct hisi_fb_data_type* hisifd)
{
	return LCD_KIT_OK;
}

/*
*name:lcd_kit_probe
*function:panel driver probe
*@pdev:platform device
*/
static int lcd_kit_probe(struct platform_device* pdev)
{
	struct hisi_panel_info* pinfo = NULL;
	struct device_node* np = NULL;
	int ret = LCD_KIT_OK;

	np = pdev->dev.of_node;
	if (!np) {
		LCD_KIT_ERR("NOT FOUND device node\n");
		return LCD_KIT_FAIL;
	}
	LCD_KIT_INFO("enter probe!\n");
	pinfo = lcd_kit_data.panel_info;
	if (!pinfo) {
		LCD_KIT_ERR("pinfo is null\n");
		return LCD_KIT_FAIL;
	}
	memset(pinfo, 0, sizeof(struct hisi_panel_info));
	/*1.adapt init*/
	lcd_kit_adapt_init();
	/*2.common init*/
	if (common_ops->common_init) {
		common_ops->common_init(np);
	}
	/*3.utils init*/
	lcd_kit_utils_init(np, pinfo);
	/*4.init fnode*/
	lcd_kit_sysfs_init();
	/*5.init factory mode*/
	lcd_kit_factory_init(pinfo);
	/*6.power init*/
	lcd_kit_power_init(pdev);
	/*7.init panel ops*/
	lcd_kit_panel_init();
	/*8.probe driver*/
	if (hisi_fb_device_probe_defer(pinfo->type, pinfo->bl_set_type)) {
		goto err_probe_defer;
	}
	pdev->id = 1;
	ret = platform_device_add_data(pdev, &lcd_kit_data, sizeof(struct hisi_fb_panel_data));
	if (ret) {
		LCD_KIT_ERR("platform_device_add_data failed!\n");
		goto err_device_put;
	}
	hisi_fb_add_device(pdev);
	/*read project id*/
	if (lcd_kit_read_project_id()) {
		LCD_KIT_ERR("read project id error\n");
	}
	LCD_KIT_INFO("exit probe!\n");
	return LCD_KIT_OK;

err_device_put:
	platform_device_put(pdev);
err_probe_defer:
	return -EPROBE_DEFER;

	return ret;
}

/***********************************************************
*platform driver definition
***********************************************************/
/*
*probe match table
*/
static struct of_device_id lcd_kit_match_table[] = {
	{
		.compatible = "auo_otm1901a_5p2_1080p_video",
		.data = NULL,
	},
	{},
};

/*
*panel platform driver
*/
static struct platform_driver lcd_kit_driver = {
	.probe = lcd_kit_probe,
	.remove = NULL,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		.name = "lcd_kit_mipi_panel",
		.of_match_table = lcd_kit_match_table,
	},
};

static int __init lcd_kit_init(void)
{
	int ret = LCD_KIT_OK;
	int len = 0;
	struct device_node* np = NULL;

	if (!lcd_kit_support()) {
		LCD_KIT_INFO("not lcd_kit driver and return\n");
		return ret;
	}
	np = of_find_compatible_node(NULL, NULL, DTS_COMP_LCD_KIT_PANEL_TYPE);
	if (!np) {
		LCD_KIT_ERR("NOT FOUND device node %s!\n", DTS_COMP_LCD_KIT_PANEL_TYPE);
		ret = -1;
		return ret;
	}
	OF_PROPERTY_READ_U32_RETURN(np, "product_id", &disp_info->product_id);
	LCD_KIT_INFO("disp_info->product_id = %d", disp_info->product_id);
	disp_info->compatible = (char*)of_get_property(np, "lcd_panel_type", NULL);
	if (!disp_info->compatible) {
		LCD_KIT_ERR("can not get lcd kit compatible\n");
		return ret;
	}
	LCD_KIT_INFO("disp_info->compatible: %s\n", disp_info->compatible);
	len = strlen(disp_info->compatible);
	memset( (char*)lcd_kit_driver.driver.of_match_table->compatible, 0, LCD_KIT_PANEL_COMP_LENGTH);
	strncpy( (char*)lcd_kit_driver.driver.of_match_table->compatible, disp_info->compatible,
			 len > (LCD_KIT_PANEL_COMP_LENGTH - 1) ? (LCD_KIT_PANEL_COMP_LENGTH - 1) : len);
	/*register driver*/
	ret = platform_driver_register(&lcd_kit_driver);
	if (ret) {
		LCD_KIT_ERR("platform_driver_register failed, error=%d!\n", ret);
	}
	return ret;
}
module_init(lcd_kit_init);
