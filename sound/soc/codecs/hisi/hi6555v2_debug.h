#ifndef __HI6555V2_DEBUG_H__
#define __HI6555V2_DEBUG_H__

#include "hi6555v2_utility.h"
#include "hicodec_debug.h"

#define PAGE_SocCODEC_BASE_ADDR  0xe8052000
#define PAGE_PmuCODEC_BASE_ADDR  0x000
#define PAGE_PmuHKADC_BASE_ADDR  0x000
#define PAGE_PmuCTRL_BASE_ADDR   0x000

#define HI6555V2_DBG_SOC_CODEC_START (PAGE_SocCODEC_BASE_ADDR + HI6555V2_SoCCODEC_START)
#define HI6555V2_DBG_SOC_CODEC_END   (PAGE_SocCODEC_BASE_ADDR + HI6555V2_SoCCODEC_END)
#define HI6555V2_DBG_PMU_CODEC_START (PAGE_PmuCODEC_BASE_ADDR + 0x380)
#define HI6555V2_DBG_PMU_CODEC_END   (PAGE_PmuCODEC_BASE_ADDR + 0x3D3)
#define HI6555V2_DBG_PMU_HKADC_START (PAGE_PmuHKADC_BASE_ADDR + 0x23E)
#define HI6555V2_DBG_PMU_HKADC_END   (PAGE_PmuHKADC_BASE_ADDR + 0x24E)
#define HI6555V2_DBG_PMU_CTRL_CLASSD_START (PAGE_PmuCTRL_BASE_ADDR + 0x109)
#define HI6555V2_DBG_PMU_CTRL_CLASSD_END   (PAGE_PmuCTRL_BASE_ADDR + 0x10B)

static struct hicodec_dump_reg_entry hi6555v2_dump_table[] = {
	{"SOC CODEC", HI6555V2_DBG_SOC_CODEC_START, HI6555V2_DBG_SOC_CODEC_END, 4},
	{"PMU CODEC", HI6555V2_DBG_PMU_CODEC_START, HI6555V2_DBG_PMU_CODEC_END, 1},
	{"PMU HKADC", HI6555V2_DBG_PMU_HKADC_START, HI6555V2_DBG_PMU_HKADC_END, 1},
	{"PMU CLASSD", HI6555V2_DBG_PMU_CTRL_CLASSD_START, HI6555V2_DBG_PMU_CTRL_CLASSD_END, 1},
};

static struct hicodec_dump_reg_info hi6555v2_dump_info = {
	.entry = hi6555v2_dump_table,
	.count = sizeof(hi6555v2_dump_table) / sizeof(struct hicodec_dump_reg_entry),
};

#endif