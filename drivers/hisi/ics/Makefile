ifeq ($(es_low_freq),true)
EXTRA_CFLAGS += -DCONFIG_ES_VDEC_LOW_FREQ
endif

ifneq ($(choose_variant),user)
EXTRA_CFLAGS += -DNPU_DEBUGFS_TEST_ENABLE
endif

obj-$(CONFIG_HISI_ICS_IPU) += cambricon_ipu.o
obj-$(CONFIG_HISI_ICS_IPU) += ipu_smmu_drv.o
obj-$(CONFIG_HISI_ICS_IPU) += ipu_clock.o
obj-$(CONFIG_HISI_ICS_IPU) += ipu_mntn.o
ifneq ($(TARGET_BUILD_VARIANT),user)
obj-$(CONFIG_HISI_ICS_IPU) += ics_debug_proxy.o
obj-$(CONFIG_HISI_ICS_IPU) += ics_debug.o
endif

obj-$(CONFIG_HISI_NPU_V200) += npu_main.o
obj-$(CONFIG_HISI_NPU_V200) += npu_task.o
obj-$(CONFIG_HISI_NPU_V200) += npu_task_sswq.o
obj-$(CONFIG_HISI_NPU_V200) += npu_task_wq.o
obj-$(CONFIG_HISI_NPU_V200) += npu_power.o
obj-$(CONFIG_HISI_NPU_V200) += npu_regulator.o
obj-$(CONFIG_HISI_NPU_V200) += npu_clock.o
obj-$(CONFIG_HISI_NPU_V200) += npu_smmu.o
obj-$(CONFIG_HISI_NPU_V200) += npu_mntn.o
ifneq ($(choose_variant),user)
obj-$(CONFIG_HISI_NPU_V200) += npu_debugfs_stub.o
obj-$(CONFIG_HISI_NPU_V200) += npu_debugfs_proxy.o
obj-$(CONFIG_HISI_NPU_V200) += npu_debugfs.o
endif