/*
 * hisi_log.h
 *
 * Copyright (c) 2001-2021, Huawei Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __HISI_LOG_H__
#define __HISI_LOG_H__

#undef  pr_fmt
#define pr_fmt(fmt) "["HISI_LOG_TAG"]:" fmt

#define MNTN_DUMP_TAG "mntn_dump"

#define HISI_PMIC_TAG "pmic"
#define HISI_PMIC_MNTN_TAG "pmic_mntn"
#define HISI_PMIC_REGULATOR_TAG "pmic_regulator"
#define HISI_PMIC_REGULATOR_DEBUG_TAG "pmic_regulator_debug"
#define HISI_SPMI_TAG "spmi"
#define HISI_SPMI_DBGFS_TAG "spmi_dbgfs"

#endif /* end of hisi_log.h */
