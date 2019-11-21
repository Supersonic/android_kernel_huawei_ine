

#ifndef __HMAC_DFX_H__
#define __HMAC_DFX_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

#ifdef _PRE_WLAN_1103_CHR
#include "mac_frame.h"
#include "dmac_ext_if.h"
#endif

#undef  THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_HMAC_DFX_H

/*****************************************************************************
  1 其他头文件包含
*****************************************************************************/

/*****************************************************************************
  2 宏定义
*****************************************************************************/


/*****************************************************************************
  3 枚举定义
*****************************************************************************/


/*****************************************************************************
  4 全局变量声明
*****************************************************************************/
#ifdef _PRE_WLAN_1103_CHR

typedef struct
{
    oal_uint8  uc_vap_state;
    oal_uint8  uc_vap_mode;
    oal_uint8  uc_p2p_mode;
    oal_uint16 us_user_nums;
    oal_uint8  uc_protocol;
    oal_uint8  uc_chnl_band;
    oal_uint8  uc_chnl_bandwidth;
    oal_uint8  uc_chnl_idx;
}hmac_chr_p2p_info_stru;

typedef struct
{
    oal_uint8 uc_vap_state;
    oal_uint8 uc_vap_num;
    mac_channel_stru  st_channel;
    oal_uint8 uc_protocol;
    oal_uint8 uc_vap_rx_nss;
    hmac_chr_p2p_info_stru  st_p2p_info;
    oal_uint8 uc_ap_protocol_mode;
    oal_uint8 uc_ap_spatial_stream_num;
    oal_uint8 bit_ampdu_active   : 1;
    oal_uint8 bit_amsdu_active   : 1;
    oal_uint8 bit_is_dbac_running: 1;
    oal_uint8 bit_is_dbdc_running: 1;
    oal_uint8 bit_sta_11ntxbf    : 1;
    oal_uint8 bit_ap_11ntxbf     : 1;
    oal_uint8 bit_ap_qos         : 1;
    oal_uint8 bit_ap_1024qam_cap : 1;
}hmac_chr_vap_info_stru;

typedef struct tag_hmac_chr_ba_info_stru
{
    oal_uint8 uc_ba_num;
    oal_uint8 uc_del_ba_tid;
    mac_reason_code_enum_uint16 en_del_ba_reason;
}hmac_chr_ba_info_stru;

typedef struct tag_hmac_chr_disasoc_reason_stru
{
    oal_uint16 us_user_id;
    dmac_disasoc_misc_reason_enum_uint16 en_disasoc_reason;
}hmac_chr_disasoc_reason_stru;
#endif
/*****************************************************************************
  5 消息头定义
*****************************************************************************/


/*****************************************************************************
  6 消息定义
*****************************************************************************/


/*****************************************************************************
  7 STRUCT定义
*****************************************************************************/


/*****************************************************************************
  8 UNION定义
*****************************************************************************/


/*****************************************************************************
  9 OTHERS定义
*****************************************************************************/


/*****************************************************************************
  10 函数声明
*****************************************************************************/
extern oal_uint32  hmac_dfx_init(void);
extern oal_uint32  hmac_dfx_exit(void);

#ifdef _PRE_WLAN_1103_CHR
oal_uint32 hmac_chr_get_vap_info(mac_vap_stru *pst_mac_vap, hmac_chr_vap_info_stru *pst_vap_info);
oal_void hmac_chr_set_ba_info(oal_uint8 uc_tid, oal_uint16 reason_id);
oal_void hmac_chr_get_ba_info(mac_vap_stru *pst_mac_vap, hmac_chr_ba_info_stru *pst_del_ba_reason);
oal_void hmac_chr_set_disasoc_reason(oal_uint16 user_id, oal_uint16 reason_id);
oal_void dmac_chr_get_disasoc_reason(hmac_chr_disasoc_reason_stru *pst_disasoc_reason);
oal_uint32  hmac_chr_get_web_fail_slow_info(oal_void);
hmac_chr_disasoc_reason_stru* hmac_chr_disasoc_reason_get_pointer(void);
oal_void hmac_chr_set_connect_code(oal_uint8 uc_vap_id, oal_uint16 connect_code);
hmac_chr_ba_info_stru* hmac_chr_ba_info_get_pointer(void);
oal_uint32  hmac_get_chr_info_event_hander(oal_uint32 chr_event_id);
#endif

#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

#endif /* end of hmac_dfx.h */
