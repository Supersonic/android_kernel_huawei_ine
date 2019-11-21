/*
 * =====================================================================================
 *
 *       Filename:  chrdrv.h
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  09/18/2014 02:53:55 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#ifdef __cpluscplus
	#if __cplusplus
	extern "C" {
	#endif
#endif

#ifndef __CHR_USER_H__
#define __CHR_USER_H__

/*
 * 1 Other Include Head File
 */
#include "chr_errno.h"

/*****************************************************************************
  1 其他头文件包含
*****************************************************************************/

/*****************************************************************************
  2 宏定义
*****************************************************************************/
/*芯片平台异常事件上报*/
#define CHR_PLATFORM_EXCEPTION_EVENTID 909060003

/*芯片平台连接失败事件上报*/
#define CHR_CHIP_CONNCET_FAIL_EVENTID 909060009

/*芯片平台扫描超时事件上报*/
#define CHR_CHIP_SCAN_FAIL_EVENTID 909060006

/*wifi上网失败*/
#define CHR_WIFI_WEB_FAIL_EVENTID 909002024

/*wifi上网慢*/
#define CHR_WIFI_WEB_SLOW_EVENTID 909002025

/*wifi无法连接*/
#define CHR_WIFI_CONNECT_FAIL_EVENTID 909002023

/*wifi异常断开*/
#define CHR_WIFI_ABNORMAL_DISCONNECT_EVENTID 909002022

/*wifi打开关闭失败*/
#define CHR_WIFI_OPEN_CLOSE_FAIL_EVENTID 909002021

/*wifi温控时间上报*/
#define CHR_WIFI_TEMP_PROTECT_EVENTID 909060009


/*****************************************************************************
  3 枚举定义
*****************************************************************************/


/*****************************************************************************
  4 全局变量声明
*****************************************************************************/


/*****************************************************************************
  5 消息头定义
*****************************************************************************/


/*****************************************************************************
  6 消息定义
*****************************************************************************/


/*****************************************************************************
  7 STRUCT定义
*****************************************************************************/
typedef struct chr_chip_excption_event_info_tag
{
   oal_uint32 excption_event_id;
   oal_uint32 sub_event_id;
   oal_uint32 reserve[2];

}chr_platform_exception_event_info_stru;


typedef struct chr_scan_exception_event_info_tag
{
   int    sub_event_id;
   char   module_name[10];
   char   error_code[20];

   /*通用信息*/

   /*空口链路质量信息*/

   /*共存状态信息*/

   /*低功耗状态信息*/

}chr_scan_exception_event_info_stru;

typedef struct chr_connect_exception_event_info_tag
{
   int    sub_event_id;
   char   platform_module_name[10];
   char   error_code[20];

}chr_connect_exception_event_info_stru;

typedef enum chr_LogPriority{
	CHR_LOG_DEBUG = 0,
	CHR_LOG_INFO,
	CHR_LOG_WARN,
	CHR_LOG_ERROR,
}CHR_LOGPRIORITY;

typedef enum chr_LogTag{
	CHR_LOG_TAG_PLAT = 0,
	CHR_LOG_TAG_WIFI,
	CHR_LOG_TAG_GNSS,
	CHR_LOG_TAG_BT,
	CHR_LOG_TAG_FM,
	CHR_LOG_TAG_NFC,
}CHR_LOG_TAG;

typedef enum chr_dev_index{
    CHR_INDEX_KMSG_PLAT = 0,
    CHR_INDEX_KMSG_WIFI,
    CHR_INDEX_APP_WIFI,
    CHR_INDEX_APP_GNSS,
    CHR_INDEX_APP_BT,
#ifdef CONFIG_CHR_OTHER_DEVS
    CHR_INDEX_APP_FM,
    CHR_INDEX_APP_NFC,
    CHR_INDEX_APP_IR,
#endif
    CHR_INDEX_MUTT,
}CHR_DEV_INDEX;

#ifdef _PRE_CONFIG_HW_CHR
#define CHR_ERR_DATA_MAX_NUM       0xa
#define CHR_ERR_DATA_MAX_LEN       (OAL_SIZEOF(oal_uint32)*CHR_ERR_DATA_MAX_NUM)
typedef struct
{
    oal_uint32  errno;
    oal_uint16  errlen;
    oal_uint16  flag:1;
    oal_uint16  resv:15;
}CHR_ERRNO_WITH_ARG_STRU;

typedef uint32 (*chr_get_wifi_info)(uint32);

extern int32 __chr_printLog(CHR_LOGPRIORITY prio, CHR_DEV_INDEX dev_index, const int8 *fmt,...);
extern int __chr_exception(uint32 errno);
extern void chr_dev_exception_callback(void *buff, uint16 len);
extern int32 __chr_exception_para(uint32 chr_errno, uint8 *chr_ptr, uint16 chr_len);
extern void chr_host_callback_register(chr_get_wifi_info pfunc);
extern void chr_host_callback_unregister(void);
extern void chr_test(void);


#define CHR_LOG(prio, tag, fmt...)                  __chr_printLog(prio, tag, ##fmt)
#define CHR_EXCEPTION(errno)                        __chr_exception(errno)
#define CHR_EXCEPTION_P(chr_errno,chr_ptr,chr_len)  __chr_exception_para(chr_errno,chr_ptr,chr_len)


#define CHR_EXCEPTION_REPORT(excption_event, excption_sub_event)\
{\
    chr_platform_exception_event_info_stru chr_platform_exception_event_info;\
    chr_platform_exception_event_info.excption_event_id = excption_event;\
    chr_platform_exception_event_info.sub_event_id = excption_sub_event;\
    CHR_EXCEPTION_P(excption_event, (oal_uint8 *)(&chr_platform_exception_event_info), OAL_SIZEOF(chr_platform_exception_event_info));\
}



#else
#define CHR_LOG(prio, tag, fmt,...)
#define CHR_EXCEPTION(chr_errno)
#define CHR_EXCEPTION_P(chr_errno,chr_ptr,chr_len)
#define CHR_EXCEPTION_REPORT(excption_event, excption_sub_event)
#endif
#endif

#ifdef __cpluscplus
	#if __cplusplus
		}
	#endif
#endif
