#ifndef _HI_GLOBAL_MEM_MAP_INCLUDE_H_
#define _HI_GLOBAL_MEM_MAP_INCLUDE_H_ 
#define HISI_RESERVED_FASTBOOT_PHYMEM_BASE 0x3C000000
#define HISI_RESERVED_FASTBOOT_PHYMEM_SIZE (0x500000)
#define HISI_RESERVED_FASTBOOT_DTB_PHYMEM_BASE (HISI_RESERVED_FASTBOOT_PHYMEM_BASE + HISI_RESERVED_FASTBOOT_PHYMEM_SIZE)
#define HISI_RESERVED_FASTBOOT_DTB_PHYMEM_SIZE (0x4000000)
#define HISI_RESERVED_FASTBOOT_AVB_HEAP_PHYMEM_BASE 0x40000000
#define HISI_RESERVED_FASTBOOT_AVB_HEAP_PHYMEM_SIZE (0x50000000 - 0x40000000)
#define HISI_RESERVED_FASTBOOT_CMA_PHYMEM_BASE (0x3C000000)
#define HISI_RESERVED_FASTBOOT_CMA_PHYMEM_SIZE (0x4000000)
#define HISI_RESERVED_DTB_PHYMEM_BASE 0x07A00000
#define HISI_RESERVED_DTB_PHYMEM_SIZE (0x07C00000 - 0x07A00000)
#define HISI_RESERVED_BOOT_CAN_RUN_BASE 0
#define HISI_RESERVED_BOOT_CAN_RUN_END 0x10000000
#define HISI_RESERVED_MODEM_SHARE_PHYMEM_BASE 0x10000000
#define HISI_RESERVED_MODEM_SHARE_PHYMEM_SIZE (0xB00000)
#define HISI_RESERVED_DRM_PGTABLE_BASE 0x10C00000
#define HISI_RESERVED_DRM_PGTABLE_SIZE (0x200000)
#define HISI_RESERVED_MODEM_SOCP_PHYMEM_BASE 0x10E00000
#define HISI_RESERVED_MODEM_SOCP_PHYMEM_SIZE (0x2000000)
#define HISI_RESERVED_MODEM_MNTN_PHYMEM_BASE 0x12E00000
#define HISI_RESERVED_MODEM_MNTN_PHYMEM_SIZE (0x100000)
#define HISI_RESERVED_HHEE_PHYMEM_BASE 0x12F00000
#define HISI_RESERVED_HHEE_PHYMEM_SIZE (0x600000)
#define HISI_HHEE_LOG_SIZE (0x20000)
#define HISI_HHEE_LOG_ADDR (HISI_RESERVED_HHEE_PHYMEM_BASE + HISI_RESERVED_HHEE_PHYMEM_SIZE - HISI_HHEE_LOG_SIZE)
#define HISI_RESERVED_FAST_KER_AND_PHYMEM_BASE 0x13500000
#define HISI_RESERVED_FAST_KER_AND_PHYMEM_SIZE (0x40000)
#define HISI_SUB_RESERVED_FASTBOOT_LOG_PYHMEM_BASE 0x13540000
#define HISI_SUB_RESERVED_FASTBOOT_LOG_PYHMEM_SIZE (0x20000)
#define HISI_SUB_RESERVED_SCHARGE_PYHMEM_BASE 0x13560000
#define HISI_SUB_RESERVED_SCHARGE_PYHMEM_SIZE (0x1000)
#define HISI_SUB_RESERVED_BL31_SHARE_MEM_PHYMEM_BASE 0x13561000
#define HISI_SUB_RESERVED_BL31_SHARE_MEM_PHYMEM_SIZE (0x10000)
#define HISI_SUB_RESERVED_LCD_GAMMA_MEM_PHYMEM_BASE 0x13571000
#define HISI_SUB_RESERVED_LCD_GAMMA_MEM_PHYMEM_SIZE (0x1000)
#define HISI_SUB_RESERVED_BRIGHTNESS_CHROMA_MEM_PHYMEM_BASE 0x13572000
#define HISI_SUB_RESERVED_BRIGHTNESS_CHROMA_MEM_PHYMEM_SIZE (0x1000)
#define HISI_SUB_RESERVED_UNUSED_PHYMEM_BASE 0x13573000
#define HISI_SUB_RESERVED_UNUSED_PHYMEM_SIZE (0x135FF000 - 0x13573000)
#define HISI_SUB_RESERVED_MNTN_DUMP_PHYMEM_BASE 0x135FF000
#define HISI_SUB_RESERVED_MNTN_DUMP_PHYMEM_SIZE (0x1000)
#define HISI_RESERVED_IVP_PHYMEM_BASE 0x13600000
#define HISI_RESERVED_IVP_PHYMEM_SIZE (0x100000)
#define HISI_RESERVED_SENSORHUB_SHMEM_PHYMEM_BASE 0x13700000
#define HISI_RESERVED_SENSORHUB_SHMEM_PHYMEM_SIZE (0x40000)
#define HISI_RESERVED_SENSORHUB_SHARE_MEM_PHYMEM_BASE 0x13740000
#define HISI_RESERVED_SENSORHUB_SHARE_MEM_PHYMEM_SIZE (0x80000)
#define HISI_RESERVED_LPMX_CORE_PHYMEM_BASE 0x137C0000
#define HISI_RESERVED_LPMX_CORE_PHYMEM_SIZE (0x100000)
#define HISI_RESERVED_LPMCU_PHYMEM_BASE 0x138C0000
#define HISI_RESERVED_LPMCU_PHYMEM_SIZE (0x40000)
#define HISI_RESERVED_SENSORHUB_PHYMEM_BASE 0x13900000
#define HISI_RESERVED_SENSORHUB_PHYMEM_SIZE (0x700000)
#define HISI_RESERVED_BL31_PHYMEM_BASE 0x14000000
#define HISI_RESERVED_BL31_PHYMEM_SIZE (0x400000)
#define HISI_RESERVED_HIFI_PHYMEM_BASE 0x14400000
#define HISI_RESERVED_HIFI_PHYMEM_SIZE (0xC00000)
#define HISI_RESERVED_SECOS_PHYMEM_BASE 0x15000000
#define HISI_RESERVED_SECOS_PHYMEM_SIZE (0x3000000)
#define HISI_RESERVED_SEC_CAMERA_PHYMEM_BASE 0x18000000
#define HISI_RESERVED_SEC_CAMERA_PHYMEM_SIZE (0xc00000)
#define HISI_RESERVED_MODEM_PHYMEM_BASE 0x20000000
#define HISI_RESERVED_MODEM_PHYMEM_SIZE (0x9500000)
#define HISI_RESERVED_HIFI_DATA_PHYMEM_BASE 0x2D500000
#define HISI_RESERVED_HIFI_DATA_PHYMEM_SIZE (0x500000)
#define HISI_RESERVED_MNTN_PHYMEM_BASE 0x2DA00000
#define HISI_RESERVED_MNTN_PHYMEM_SIZE (0x800000)
#define HISI_RESERVED_PSTORE_PHYMEM_BASE 0x2E200000
#define HISI_RESERVED_PSTORE_PHYMEM_SIZE (0x100000)
#define HISI_RESERVED_DRM_CMA_BASE 0x40000000
#define HISI_RESERVED_DRM_CMA_SIZE (0x20000000)
#define HISI_RESERVED_KERNEL_DUMP_PROTECT_BASE 0x80000000
#define HISI_RESERVED_KERNEL_DUMP_PRORECT_SIZE (0x20000000)
#define HISI_RESERVED_DDR_TRAINING1_PHYMEM_BASE 0x07B00000
#define HISI_RESERVED_DDR_TRAINING1_PHYMEM_SIZE (0x07B08000 - 0x07B00000)
#define HISI_RESERVED_LPMX_CORE_PHYMEM_BASE_UNIQUE (HISI_RESERVED_LPMX_CORE_PHYMEM_BASE)
#define HISI_RESERVED_LPMCU_PHYMEM_BASE_UNIQUE (HISI_RESERVED_LPMCU_PHYMEM_BASE)
#define HISI_RESERVED_MNTN_PHYMEM_BASE_UNIQUE (HISI_RESERVED_MNTN_PHYMEM_BASE)
#endif
