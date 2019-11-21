# Audiokit driver

obj-$(CONFIG_HUAWEI_DSM_AUDIO)	+= dsm_audio/
obj-$(CONFIG_HUAWEI_DEVICEBOX_AUDIO)	+= deviceboxID/
ifneq ($(TARGET_BUILD_VARIANT),user)
	obj-y      += headset_debug/
endif
ifeq ($(CONFIG_HUAWEI_SMARTPAKIT_AUDIO), y)
       obj-y += smartpakit/
endif


