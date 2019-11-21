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

#ifndef __LCD_KIT_DBG_H_
#define __LCD_KIT_DBG_H_

#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/syscalls.h>
#include "lcd_kit_common.h"

/* define macro */
#define LCD_KIT_DCS_STR              ("dcs_")
#define LCD_KIT_GEN_STR              ("gen_")
#define LCD_KIT_READ_STR             ("read_")
#define LCD_KIT_WRITE_STR            ("write_")

//#define LCD_DEBUG_BUF             (1024)
//#define LCD_PARAM_BUF             (256)
#define LCD_KIT_MAX_PARAM_NUM        (25)

#define LCD_KIT_OPER_READ            (1)
#define LCD_KIT_OPER_WRITE           (2)
#define LCD_KIT_MIPI_PATH_OPEN       (1)
#define LCD_KIT_MIPI_PATH_CLOSE      (0)
#define LCD_KIT_MIPI_DCS_COMMAND     (1<<0)
#define LCD_KIT_MIPI_GEN_COMMAND     (4)

#define LCD_KIT_HEX_BASE ((char)16)
#define LCD_KIT_IS_VALID_CHAR(_ch) ((_ch >= '0' && _ch <= '9')?1:\
									(_ch >= 'a' && _ch <= 'f')?1:(_ch >= 'A' && _ch <= 'F'))?1:0
/* dcs read/write */
#define DTYPE_DCS_WRITE     0x05/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1    0x15/* short write, 1 parameter */
#define DTYPE_DCS_READ      0x06/* read */
#define DTYPE_DCS_LWRITE    0x39/* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE     0x03/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1    0x13    /* short write, 1 parameter */
#define DTYPE_GEN_WRITE2    0x23/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE    0x29/* long write */
#define DTYPE_GEN_READ      0x04/* long read, 0 parameter */
#define DTYPE_GEN_READ1     0x14/* long read, 1 parameter */
#define DTYPE_GEN_READ2     0x24/* long read, 2 parameter */

/*
 * Message printing priorities:
 * LEVEL 0 KERN_EMERG (highest priority)
 * LEVEL 1 KERN_ALERT
 * LEVEL 2 KERN_CRIT
 * LEVEL 3 KERN_ERR
 * LEVEL 4 KERN_WARNING
 * LEVEL 5 KERN_NOTICE
 * LEVEL 6 KERN_INFO
 * LEVEL 7 KERN_DEBUG (Lowest priority)
 */

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

//#define HW_LCD_DEBUG  1

#define VCC_LCD_KIT_BIAS_NAME                "vcc_lcdbias"
#define VCC_LCD_KIT_VSN_NAME                 "lcd_vsn"
#define VCC_LCD_KIT_VSP_NAME                 "lcd_vsp"

#define LCD_KIT_VREG_VDD_NAME                "vdd"
#define LCD_KIT_VREG_LAB_NAME                "lab"
#define LCD_KIT_VREG_IBB_NAME                "ibb"
#define LCD_KIT_VREG_BIAS_NAME               "bias"
#define LCD_KIT_VREG_VDDIO_NAME              "vddio"


#define LCD_KIT_INIT_TEST_PARAM          "/data/lcd_kit_init_param.txt"
#define LCD_KIT_CONFIG_TABLE_MAX_NUM     (2 * PAGE_SIZE)

enum lcd_kit_cmds_type {
	LCD_KIT_DBG_LEVEL_SET = 0,
	LCD_KIT_DBG_MIPI_CLK,
	LCD_KIT_DBG_REG_READ,
	LCD_KIT_DBG_REG_WRITE,
	LCD_KIT_DBG_INIT_CODE,
	LCD_KIT_DBG_PANEL_VSP_VSN,
	LCD_KIT_DBG_ESD_ENABLE,
	LCD_KIT_DBG_ESD_RECOVER_TEST,
	LCD_KIT_DBG_ESD_RESET,
	LCD_KIT_DBG_ESD_BL_ENABLE,
	LCD_KIT_DBG_ESD_BL_SET,
	LCD_KIT_DBG_ESD_CHECK_REG,
	LCD_KIT_DBG_ESD_CHECK_VALUE,
	LCD_KIT_DBG_NUM_MAX,
};


struct lcd_kit_dsi_ctrl_hdr {
	char dtype; /* data type */
	char last;  /* last in chain */
	char vc;    /* virtual chan */
	char ack;   /* ask ACK from peripheral */
	char wait;  /* ms */
	char waittype;
	char dlen;  /* 8 bits */
};

typedef struct {
	char type;
	char pstr[100];
} lcd_kit_dbg_cmds;

struct lcd_kit_esd_debug {
	int esd_enable;
	char esd_check_reg[8];
	char esd_reg_value[8];
	int esd_bl_enable;
	int esd_bl_set;
	int check_count;
	int esd_recover_test;
};

struct lcd_kit_debug {
	int lcd_kit_g_mipiclk;
	int lcd_kit_g_mipiclk_enable;
	int lcd_kit_g_initcode_enable;
	int g_initcode_flag;
	int lcd_kit_g_vsp_vsn_enable;
	int lcd_kit_panel_bias;
	int lcd_kit_panel_vsp;
	int lcd_kit_panel_vsn;

	int lcd_kit_ic_mipi_reg;    // read register
	int lcd_kit_ic_mipi_value;  // read value

	//struct platform_device*
	void* lcd_kit_ctrl_pdev;
	void* lcd_kit_ctrl_pdata;

	struct lcd_kit_esd_debug g_esd_debug;
	struct lcd_kit_dsi_panel_cmds panel_cmds;
};

//for extern

int lcd_kit_debugfs_init(void);
int is_lcd_kit_initcode_enable(void);
int is_lcd_kit_mipiclk_enable(void);
int is_lcd_kit_vsp_vsn_enable(void);
int get_lcd_kit_mipiclk_dbg(void);
void lcd_kit_debug_set_vsp_vsn(void* vcc_cmds, int cnt);
int lcd_kit_esd_debug(void* pdata);

//for debug
void lcd_kit_dump_buf(const char* buf, int cnt);
void lcd_kit_dump_buf_32(const u32* buf, int cnt);
void lcd_kit_dump_cmds_desc(struct lcd_kit_dsi_cmd_desc* desc);
void lcd_kit_dump_cmds(struct lcd_kit_dsi_panel_cmds* cmds);
void lcd_kit_dump_array_data(struct lcd_kit_array_data* array);
void lcd_kit_dump_arrays_data(struct lcd_kit_arrays_data* arrays);
#endif
