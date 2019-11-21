/* MD5: 94b0888ed18138b481545bfe39669274*/
#if !defined(__PRODUCT_CONFIG_GUCPHY_H__)
#define __PRODUCT_CONFIG_GUCPHY_H__

#ifndef ZSP_DSP_CHIP_BB_TYPE		
#define ZSP_DSP_CHIP_BB_TYPE		 15 
#endif 

#ifndef NV_VERSION
#define NV_VERSION nv_boston 
#endif 

#ifndef ZSP_DSP_PRODUCT_FORM		
#define ZSP_DSP_PRODUCT_FORM		 4 
#endif 

#ifndef BOARD
#define BOARD SFT 
#endif 

#ifndef UPHY_BOARD_TYPE
#define UPHY_BOARD_TYPE 2 
#endif 

#ifndef FEATURE_TEMP_MULTI_MODE_LP
#define FEATURE_TEMP_MULTI_MODE_LP FEATURE_ON 
#endif 

#ifndef FEATURE_GSM_SDR
#define FEATURE_GSM_SDR FEATURE_ON 
#endif 

#ifndef FEATURE_GSM_SDR_DAIC
#define FEATURE_GSM_SDR_DAIC FEATURE_OFF 
#endif 

#ifndef FEATURE_GSM_WHOLE_SDR		
#define FEATURE_GSM_WHOLE_SDR		 FEATURE_OFF 
#endif 

#ifndef FEATURE_XBBE16_NEW_MAIL		
#define FEATURE_XBBE16_NEW_MAIL		 FEATURE_ON 
#endif 

#ifndef FEATURE_SRAM_400K
#define FEATURE_SRAM_400K FEATURE_OFF 
#endif 

#ifndef FEATURE_EXTERNAL_32K_CLK		
#define FEATURE_EXTERNAL_32K_CLK		 FEATURE_ON 
#endif 

#ifndef FEATURE_SOCP_ON_DEMAND			
#define FEATURE_SOCP_ON_DEMAND			 FEATURE_OFF 
#endif 

#ifndef FEATURE_TAS				
#define FEATURE_TAS				 FEATURE_ON 
#endif 

#ifndef FEATURE_DC_DPA			
#define FEATURE_DC_DPA			 FEATURE_ON 
#endif 

#ifndef FEATURE_RFIC_RESET_GPIO_ON		
#define FEATURE_RFIC_RESET_GPIO_ON		 FEATURE_OFF 
#endif 

#ifndef FEATURE_VIRTUAL_BAND			
#define FEATURE_VIRTUAL_BAND			 FEATURE_ON 
#endif 

#ifndef FEATURE_HI6363                		
#define FEATURE_HI6363                		 FEATURE_ON 
#endif 

#ifndef FEATURE_RTT_TEST
#define FEATURE_RTT_TEST FEATURE_ON 
#endif 

#ifndef FEATURE_RTT_RANDOM_TEST
#define FEATURE_RTT_RANDOM_TEST FEATURE_OFF 
#endif 

#ifndef FEATURE_GUTLC_ONE_DSP
#define FEATURE_GUTLC_ONE_DSP FEATURE_ON 
#endif 

#ifndef FEATURE_NX_CORE_OPEN
#define FEATURE_NX_CORE_OPEN FEATURE_ON 
#endif 

#ifndef FEATURE_CSDR
#define FEATURE_CSDR FEATURE_ON 
#endif 

#ifndef CPHY_PUB_DTCM_BASE	
#define CPHY_PUB_DTCM_BASE	 0xE3484000 
#endif 

#ifndef CPHY_PUB_ITCM_BASE	
#define CPHY_PUB_ITCM_BASE	 0xE3535800 
#endif 

#ifndef CPHY_PRV_DTCM_BASE	
#define CPHY_PRV_DTCM_BASE	 0xE3484000 
#endif 

#ifndef CPHY_PRV_ITCM_BASE	
#define CPHY_PRV_ITCM_BASE	 0xE3535800 
#endif 

#ifndef CPHY_PUB_DTCM_SIZE		
#define CPHY_PUB_DTCM_SIZE		 0x3a000 
#endif 

#ifndef CPHY_PUB_ITCM_SIZE		
#define CPHY_PUB_ITCM_SIZE		 0x40000 
#endif 

#ifndef CPHY_PRV_DTCM_SIZE		
#define CPHY_PRV_DTCM_SIZE		 0x3a000 
#endif 

#ifndef CPHY_PRV_ITCM_SIZE		
#define CPHY_PRV_ITCM_SIZE		 0x40000 
#endif 

#ifndef CPHY_PUB_DTCM_GLB_MINUS_LOCAL	
#define CPHY_PUB_DTCM_GLB_MINUS_LOCAL	 0x0 
#endif 

#ifndef XTENSA_CORE_X_CACHE
#define XTENSA_CORE_X_CACHE Atlanta_NX_SWUP 
#endif 

#ifndef LD_MAP_PATH
#define LD_MAP_PATH kirin710_gutcl_one_dsp_lsp 
#endif 

#ifndef XTENSA_CORE_X_SYSTEM
#define XTENSA_CORE_X_SYSTEM "RH-2017.2.3" 
#endif 

#ifndef PHY_CBT_ENABLED
#define PHY_CBT_ENABLED 
#endif 

#ifndef FEATURE_EASYRF
#define FEATURE_EASYRF FEATURE_OFF 
#endif 

#endif /*__PRODUCT_CONFIG_H__*/ 
