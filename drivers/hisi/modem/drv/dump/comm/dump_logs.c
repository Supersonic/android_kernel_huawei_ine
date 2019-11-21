/*
 * Copyright (C) Huawei Technologies Co., Ltd. 2012-2015. All rights reserved.
 * foss@huawei.com
 *
 * If distributed as part of the Linux kernel, the following license terms
 * apply:
 *
 * * This program is free software; you can redistribute it and/or modify
 * * it under the terms of the GNU General Public License version 2 and
 * * only version 2 as published by the Free Software Foundation.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * * GNU General Public License for more details.
 * *
 * * You should have received a copy of the GNU General Public License
 * * along with this program; if not, write to the Free Software
 * * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Otherwise, the following license terms apply:
 *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * * 1) Redistributions of source code must retain the above copyright
 * *    notice, this list of conditions and the following disclaimer.
 * * 2) Redistributions in binary form must reproduce the above copyright
 * *    notice, this list of conditions and the following disclaimer in the
 * *    documentation and/or other materials provided with the distribution.
 * * 3) Neither the name of Huawei nor the names of its contributors may
 * *    be used to endorse or promote products derived from this software
 * *    without specific prior written permission.
 *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/kernel.h> 
#include <asm/string.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include "osl_malloc.h"
#include "bsp_sram.h"
#include "bsp_shared_ddr.h"
#include "bsp_ddr.h"
#include "bsp_dump.h"
#include "bsp_coresight.h"
#include "dump_file.h"
#include "dump_logs.h"
#include "dump_config.h"
#include "dump_cp_agent.h"
#include "dump_apr.h"
#include "dump_area.h"
#undef	THIS_MODU
#define THIS_MODU mod_dump

u8*      g_modem_ddr_map_addr = NULL;

dump_log_ctrl_s g_log_notifier_ctrl;

/*****************************************************************************
* 函 数 名  : dump_save_log_notifier_init
* 功能描述  : 保存log回调初始化
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_log_notifier_init(void)
{
    if(true == g_log_notifier_ctrl.init_flag)
    {
        return ;
    }
    spin_lock_init(&g_log_notifier_ctrl.lock);

    INIT_LIST_HEAD(&g_log_notifier_ctrl.log_list);

    g_log_notifier_ctrl.init_flag = true;

    return ;

}

/*****************************************************************************
* 函 数 名  : bsp_dump_register_log_notifier
* 功能描述  : 注册异常时保存log的回调
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
/*lint -save -e429*/
s32 bsp_dump_register_log_notifier(u32 modem_type,log_save_fun save_fun,char* name)
{
}
/*lint -restore -e429*/
/*****************************************************************************
* 函 数 名  : bsp_dump_unregister_log_notifier
* 功能描述  : 取消注册函数
*
* 输入参数  :
             
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年4月21日10:47:55   lixiaofan  creat
*
*****************************************************************************/
s32 bsp_dump_unregister_log_notifier(log_save_fun save_fun)
{
 
     unsigned long flags;
     dump_log_notifier* log_notifier = NULL;
     dump_log_notifier* log_notifier_node = NULL;

    if(g_log_notifier_ctrl.init_flag != true)
    {
        return BSP_ERROR;
    }

    spin_lock_irqsave(&g_log_notifier_ctrl.lock, flags);
    list_for_each_entry(log_notifier, &g_log_notifier_ctrl.log_list, list)
    {   
        if(log_notifier->save_fun == save_fun)
        {
            log_notifier_node = log_notifier;
        }
    }
    if(!log_notifier_node)
    {
        spin_unlock_irqrestore(&g_log_notifier_ctrl.lock, flags);
        return BSP_ERROR;
    }
    list_del(&log_notifier_node->list);
    osl_free(log_notifier_node);
    spin_unlock_irqrestore(&g_log_notifier_ctrl.lock, flags);

    return BSP_OK;
}

/*****************************************************************************
* 函 数 名  : bsp_dump_log_notifer_callback
* 功能描述  : 执行异常log保存log的回调
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void bsp_dump_log_notifer_callback(u32 modem_type,char* path)
{
    struct list_head *p = NULL;
    struct list_head *n = NULL;
    dump_log_notifier* log_notifier = NULL;
    //unsigned long flags;

    if(false == g_log_notifier_ctrl.init_flag)
    {
        return ;
    }
    
    //spin_lock_irqsave(&g_log_notifier_ctrl.lock, flags);
    if(list_empty(&g_log_notifier_ctrl.log_list))
    {
        //spin_unlock_irqrestore(&g_log_notifier_ctrl.lock, flags);
        return;
    }

    list_for_each_safe(p,n,&g_log_notifier_ctrl.log_list)
    {
        log_notifier = list_entry(p,dump_log_notifier,list);
        if(log_notifier != NULL && log_notifier->save_fun != NULL && log_notifier->modem_type& modem_type)
        {
            log_notifier->save_fun(path);
        }
    }
    //spin_unlock_irqrestore(&g_log_notifier_ctrl.lock, flags);
}


/*****************************************************************************
* 函 数 名  : dump_memmap_modem_ddr
* 功能描述  : 映射modem ddr的内存，只在手机版本上使用，
                           mbb平台上在fastboot导出
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_map_mdm_ddr(void)
{
    dump_product_type_t type = dump_get_product_type();
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();

    dump_error("mdm_ddr= %d,type=%d\n",cfg->file_list.file_bits.mdm_ddr,type);

    if(cfg->file_list.file_bits.mdm_ddr == 1 
        && type == DUMP_PHONE 
        && (DUMP_ACCESS_MDD_DDR_NON_SEC == dump_get_access_mdmddr_type()))
    {
        g_modem_ddr_map_addr = (u8 *)ioremap_wc((phys_addr_t)(MDDR_FAMA(DDR_MCORE_ADDR)), (size_t)(DDR_MCORE_SIZE));

        if(g_modem_ddr_map_addr == NULL)
        {
            dump_error("map g_modem_ddr_map_addr fail\n");
        }
    }
    dump_error("dump_memmap_modem_ddr finish\n");
}

/*****************************************************************************
* 函 数 名  : dump_save_mdm_ddr_file
* 功能描述  : 保存modem的ddr
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_ddr_file(char* dir_name)
{
    dump_load_info_t dump_load = {0,};
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();

    if(cfg->file_list.file_bits.mdm_ddr == 1
        && (dump_get_product_type() == DUMP_PHONE)
        && (EDITION_INTERNAL_BETA == dump_get_edition_type())
        && (DUMP_ACCESS_MDD_DDR_NON_SEC == dump_get_access_mdmddr_type()))
    {

        /*coverity[secure_coding]*/
        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, sizeof(file_name), "%smodem_ddr.bin", dir_name);

        if(NULL == g_modem_ddr_map_addr)
        {
            dump_error("ioremap MODEM DDR fail\n");
        }
        else
        {
            if( BSP_OK == dump_get_load_info(&dump_load))
            {
                dump_save_file(file_name,(u8*) g_modem_ddr_map_addr+dump_load.mdm_ddr_saveoff, DDR_MCORE_SIZE-dump_load.mdm_ddr_saveoff);
            }
            else
            {
                dump_save_file(file_name,(u8*) g_modem_ddr_map_addr, DDR_MCORE_SIZE);
            }
            dump_error("[dump]: save %s finished\n", file_name);
        }
    }
}

/*****************************************************************************
* 函 数 名  : dump_save_mdm_ddr_file
* 功能描述  : 保存modem的ddr
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_dts_file(char* dir_name)
{
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();
    u8* addr = (u8 *)ioremap_wc((phys_addr_t)(MDDR_FAMA(DDR_MCORE_DTS_ADDR)), (size_t)(DDR_MCORE_DTS_SIZE));

    if(cfg->file_list.file_bits.mdm_dts == 1
        && (dump_get_product_type() == DUMP_PHONE)
        && (EDITION_INTERNAL_BETA == dump_get_edition_type())
        && (DUMP_ACCESS_MDD_DDR_NON_SEC== dump_get_access_mdmddr_type()))
    {

        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        snprintf_s(file_name, sizeof(file_name),(sizeof(file_name)-1), "%smodem_dts.bin", dir_name);

        if(NULL == addr)
        {
            dump_error("ioremap DDR_MCORE_DTS_ADDR fail\n");
        }
        else
        {
            dump_save_file(file_name, addr, DDR_MCORE_DTS_SIZE);
            dump_error("[dump]: save %s finished\n", file_name);
        }
    }
}



/*****************************************************************************
* 函 数 名  : dump_save_mdm_sram_file
* 功能描述  : 保存modem的sram
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_sram_file(char* dir_name)
{
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();
    if((cfg->file_list.file_bits.mdm_sram == 1)&&(EDITION_INTERNAL_BETA == dump_get_edition_type()))
    {
        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, (sizeof(file_name)-1), "%smodem_sram.bin", dir_name);
        dump_save_file(file_name, (u8 *)g_mem_ctrl.sram_virt_addr, g_mem_ctrl.sram_mem_size);
        dump_error("[dump]: save %s finished\n", file_name);
    }
}


/*****************************************************************************
* 函 数 名  : dump_save_mdm_share_file
* 功能描述  : 保存modem的共享内存
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_secshare_file(char* dir_name)
{
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();
    void* addr = NULL;
    /*lint -save -e835*/
    if(cfg->file_list.file_bits.mdm_secshare == 1  
        && (dump_get_product_type()== DUMP_PHONE) 
        && (EDITION_INTERNAL_BETA == dump_get_edition_type())
        && (DUMP_ACCESS_MDD_DDR_NON_SEC== dump_get_access_mdmddr_type()))
    {

        addr = ioremap_wc((phys_addr_t)(MDDR_FAMA(DDR_SEC_SHARED_ADDR)),(size_t)(DDR_SEC_SHARED_SIZE));
        if(addr == NULL)
        {
            return;
        }
        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, (sizeof(file_name)-1), "%smodem_secshared.bin", dir_name);
        dump_save_file(file_name, addr, DDR_SEC_SHARED_SIZE);
        dump_error("[dump]: save %s finished\n", file_name);
    }
    /*lint -restore +e835*/

}

/*****************************************************************************
* 函 数 名  : dump_save_mdm_ddr_file
* 功能描述  : 保存modem的共享内存
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_share_file(char* dir_name)
{
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();
    dump_load_info_t dump_load = {0,};
    u32 len = 0;
    if(cfg->file_list.file_bits.mdm_share == 1  
        && (dump_get_product_type()== DUMP_PHONE) 
        && (EDITION_INTERNAL_BETA == dump_get_edition_type())
        && (DUMP_ACCESS_MDD_DDR_NON_SEC== dump_get_access_mdmddr_type()))
    {

        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, (sizeof(file_name)-1), "%smodem_share.bin", dir_name);
        if( BSP_OK == dump_get_load_info(&dump_load))
        {
            len = g_mem_ctrl.sddr_mem_size > dump_load.mdm_share_ddr_size ? dump_load.mdm_share_ddr_size : g_mem_ctrl.sddr_mem_size;
        }
        else
        {
            len = g_mem_ctrl.sddr_mem_size;
        }
        dump_save_file(file_name, (u8 *)g_mem_ctrl.sddr_virt_addr, len);
        dump_error("[dump]: save %s finished\n", file_name);
    }
}

/*****************************************************************************
* 函 数 名  : dump_save_mdm_llram_file
* 功能描述  : 保存modem的共享内存
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mdm_llram_file(char* dir_name)
{

    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();
    void* addr = NULL;

    if(cfg->file_list.file_bits.llram_share == 1  
        && (dump_get_product_type()== DUMP_PHONE) 
        && (EDITION_INTERNAL_BETA == dump_get_edition_type()))
    {
    
        addr = ioremap_wc((phys_addr_t)(MDDR_FAMA(CCPU_LLRAM_BASE_ADDR)),(size_t)(CCPU_LLRAM_BASE_SIZE));
        if(addr == NULL)
        {
            return;
        }
        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, (sizeof(file_name)-1), "%smodem_llram.bin", dir_name);
        dump_save_file(file_name, (u8 *)addr, CCPU_LLRAM_BASE_SIZE);
        dump_error("[dump]: save %s finished\n", file_name);
    }

}



/*****************************************************************************
* 函 数 名  : dump_save_modem_bin
* 功能描述  : 保存modem_dump.bin
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mntn_bin(char* dir_name)
{
    struct dump_global_area_ctrl_s global_area = {0,};
    char file_name[MODEM_DUMP_FILE_NAME_LENGTH] = {0,};
    s32 ret;
    DUMP_FILE_CFG_STRU* cfg = dump_get_file_cfg();

    if(cfg->file_list.file_bits.mdm_dump == 1  && (dump_get_product_type()== DUMP_PHONE))
    {
        memset_s(file_name, sizeof(file_name), 0, sizeof(file_name));
        /*coverity[secure_coding]*/
        snprintf(file_name, (sizeof(file_name) - 1), "%smodem_dump.bin", dir_name);
        ret = dump_get_global_info(&global_area);
        if(ret == BSP_OK && global_area.virt_addr != NULL)
        {
            dump_save_file(file_name, (u8 *)global_area.virt_addr, global_area.length);
            dump_error("[dump]: save %s finished\n", file_name);
        }

    }
}

/*****************************************************************************
* 函 数 名  : dump_save_mandatory_logs
* 功能描述  :保存lr系统的必选文件
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_save_mandatory_logs(char* dir_name)
{
    bsp_coresight_save_cp_etb(dir_name);
    dump_save_mntn_bin(dir_name);

}
/*****************************************************************************
* 函 数 名  : dump_optional_log_init
* 功能描述  : 保存lr系统的可选文件
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void dump_optional_log_init(void)
{
}


/****************************************************************************
* 函 数 名  : bsp_om_save_reboot_log
* 功能描述  : 对外接口用于开关机接口保存开关机信息
*
* 输入参数  :
* 输出参数  :

* 返 回 值  :

*
* 修改记录  : 2016年1月4日17:05:33   lixiaofan  creat
*
*****************************************************************************/
void bsp_om_save_reboot_log(const char * func_name, const void* caller)
{

    struct timex txc = {0,};
    struct rtc_time tm = {0,};
    char log_buff[200] = {0};
    char temp[32] = {0};

    do_gettimeofday(&(txc.time));
    rtc_time_to_tm(txc.time.tv_sec, &tm);
    
    /*coverity[secure_coding]*/
    /* coverity[overrun-buffer-val] */
    snprintf_s(temp, sizeof(temp),(sizeof(temp)-1), "%d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    snprintf_s((char*)log_buff, sizeof(log_buff),(sizeof(log_buff)-1) ,"system reboot reason: NORMAL_RESET A CORE, FUNC:%s, caller:%pK, TIME:%s\n", func_name, caller, temp);
    dump_append_file(OM_DUMP_PATH, OM_RESET_LOG, (void*)log_buff, (u32)(strlen(log_buff)), OM_RESET_LOG_MAX);
    dump_error("bsp_om_save_reboot_log finish\n");

}
EXPORT_SYMBOL(bsp_om_save_reboot_log);

