/*
 * hisi_pcm_hifi.c -- ALSA SoC HISI PCM HIFI driver
 *
 * Copyright (c) 2013 Hisilicon Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/sched/rt.h>


#include <linux/hisi/hilog.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#ifdef CONFIG_HIFI_MAILBOX
#include "drv_mailbox_cfg.h"
//#include "../../../drivers/hisi/hifi_mailbox/mailbox/drv_mailbox_cfg.h"
#endif

#ifdef CONFIG_HUAWEI_DSM
#include <dsm_audio/dsm_audio.h>
#endif

#ifdef CONFIG_HI6XXX_MAILBOX_MULTICORE
#include <../../../drivers/hisi/mailbox/hi6xxx_mailbox/drv_mailbox.h>
#endif

#include "hifi_lpp.h"
#include "hisi_pcm_hifi.h"
#include "hisi_snd_log.h"

/*lint -e750 -e785 -e838 -e749 -e747 -e611 -e570 -e647 -e574*/

#define UNUSED_PARAMETER(x) (void)(x)

#define HISI_PCM_HIFI "hi6210-hifi"
/*
 * PLAYBACK SUPPORT FORMATS
 * BITS : 8/16/24  18/20
 * LITTLE_ENDIAN / BIG_ENDIAN
 * MONO / STEREO
 * UNSIGNED / SIGNED
 */
#define HISI_PCM_PB_FORMATS  (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_U8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_U16_LE | \
		SNDRV_PCM_FMTBIT_U16_BE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_U24_LE | \
		SNDRV_PCM_FMTBIT_U24_BE)

/*
 * PLAYBACK SUPPORT RATES
 * 8/11.025/16/22.05/32/44.1/48/88.2/96kHz
 */
#define HISI_PCM_PB_RATES    (SNDRV_PCM_RATE_8000_48000 | \
		SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_88200 | \
		SNDRV_PCM_RATE_96000 | \
		SNDRV_PCM_RATE_176400 | \
		SNDRV_PCM_RATE_192000 | \
		SNDRV_PCM_RATE_384000)

#define HISI_PCM_PB_MIN_CHANNELS  ( 1 )
#define HISI_PCM_PB_MAX_CHANNELS  ( 2 )
/* Assume the FIFO size */
#define HISI_PCM_PB_FIFO_SIZE     ( 16 )

/* CAPTURE SUPPORT FORMATS : SIGNED 16/24bit */
#define HISI_PCM_CP_FORMATS  ( SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

/* CAPTURE SUPPORT RATES : 48/96kHz */
#define HISI_PCM_CP_RATES    ( SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 )

#define HISI_PCM_CP_MIN_CHANNELS  ( 1 )
#define HISI_PCM_CP_MAX_CHANNELS  ( 6 )
/* Assume the FIFO size */
#define HISI_PCM_CP_FIFO_SIZE     ( 32 )
#define HISI_PCM_MODEM_RATES      ( SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000)
#define HISI_PCM_BT_RATES         ( SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 )
#define HISI_PCM_FM_RATES         ( SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 )

#define HISI_PCM_MAX_BUFFER_SIZE  ( 192 * 1024 )    /* 0x30000 */
#define HISI_PCM_BUFFER_SIZE_MM   ( 32 * 1024 )
#define HISI_PCM_MIN_BUFFER_SIZE  ( 32 )
#define HISI_PCM_MAX_PERIODS      ( 32 )
#define HISI_PCM_MIN_PERIODS      ( 2 )
#define HISI_PCM_WORK_DELAY_1MS   ( 33 )    /* 33 equals 1ms */
#define HISI_PCM_CYC_SUB(Cur, Pre, CycLen)                    \
	(((Cur) < (Pre)) ? (((CycLen) - (Pre)) + (Cur)) : ((Cur) - (Pre)))

#ifndef OK
#define OK              0
#endif
#ifndef ERROR
#define ERROR           -1
#endif

#undef NULL
#define NULL ((void *)0)

#define ALSA_TIMEOUT_MILLISEC 40
#define CHECK_FLAG_TIMEOUT    (160)     /* 5ms */
#define CHECK_UPDATE_TIMEOUT  (350)     /* 10ms */

PCM_DMA_BUF_CONFIG  g_pcm_dma_buf_config[PCM_DEVICE_MAX][PCM_STREAM_MAX] =
{
	{{PCM_DMA_BUF_0_PLAYBACK_BASE,PCM_DMA_BUF_0_PLAYBACK_LEN},{PCM_DMA_BUF_0_CAPTURE_BASE,PCM_DMA_BUF_0_CAPTURE_LEN}},/*normal*/
	{{0,0},{0,0}},/*modem*/
	{{0,0},{0,0}},/*fm*/
	{{0,0},{0,0}},/*bt*/
	{{0,0},{0,0}},/*offload*/
	{{PCM_DMA_BUF_1_PLAYBACK_BASE,PCM_DMA_BUF_1_PLAYBACK_LEN},{PCM_DMA_BUF_1_CAPTURE_BASE,PCM_DMA_BUF_1_CAPTURE_LEN}},/*direct*/
	{{PCM_DMA_BUF_2_PLAYBACK_BASE,PCM_DMA_BUF_2_PLAYBACK_LEN},{PCM_DMA_BUF_2_CAPTURE_BASE,PCM_DMA_BUF_2_CAPTURE_LEN}} /*lowlatency*/
};

/* supported sample rates */
static const unsigned int freq[] = {
	8000,   11025,  12000,  16000,
	22050,  24000,  32000,  44100,
	48000,  88200,  96000,  176400,
	192000, 384000,
};

static const struct snd_soc_component_driver hisi_pcm_component = {
	.name   = HISI_PCM_HIFI,
};

static u64 hisi_pcm_dmamask           = (u64)(0xffffffff);

/* according to definition in pcm driver  */
enum pcm_device {
	PCM_DEVICE_NORMAL = 0,
	PCM_DEVICE_MODEM,
	PCM_DEVICE_FM,
	PCM_DEVICE_BT,
	PCM_DEVICE_OFFLOAD,
	PCM_DEVICE_DIRECT,
	PCM_DEVICE_LOW_LATENCY,
	PCM_DEVICE_TOTAL
};

static struct snd_soc_dai_driver hisi_pcm_dai[] =
{
	{
		.name = "hi6210-mm",
		.playback = {
			.stream_name  = "hi6210-mm Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_PB_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
		.capture = {
			.stream_name  = "hi6210-mm Capture",
			.channels_min = HISI_PCM_CP_MIN_CHANNELS,
			.channels_max = HISI_PCM_CP_MAX_CHANNELS,
			.rates        = HISI_PCM_CP_RATES,
			.formats      = HISI_PCM_CP_FORMATS
		},
	},
	{
		.name = "hi6210-modem",
		.playback = {
			.stream_name  = "hi6210-modem Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_MODEM_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
	},
	{
		.name = "hi6210-fm",
		.playback = {
			.stream_name  = "hi6210-fm Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_FM_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
	},
	{
		.name = "hi6210-bt",
		.playback = {
			.stream_name  = "hi6210-bt Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_BT_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
	},
	{
		.name = "hi6210-lpp",
		.playback = {
			.stream_name  = "hi6210-lpp Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_PB_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
	},
	{
		.name = "hi6210-direct",
		.playback = {
			.stream_name  = "hi6210-direct Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_PB_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
	},
	{
		.name = "hi6210-fast",
		.playback = {
			.stream_name  = "hi6210-fast Playback",
			.channels_min = HISI_PCM_PB_MIN_CHANNELS,
			.channels_max = HISI_PCM_PB_MAX_CHANNELS,
			.rates        = HISI_PCM_PB_RATES,
			.formats      = HISI_PCM_PB_FORMATS
		},
		.capture = {
			.stream_name  = "hi6210-fast Capture",
			.channels_min = HISI_PCM_CP_MIN_CHANNELS,
			.channels_max = HISI_PCM_CP_MAX_CHANNELS,
			.rates        = HISI_PCM_CP_RATES,
			.formats      = HISI_PCM_CP_FORMATS
		},
	},
};

/* define the capability of playback channel */
static const struct snd_pcm_hardware hisi_pcm_hardware_playback =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_NONINTERLEAVED
		| SNDRV_PCM_INFO_MMAP
		| SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_PAUSE,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min     = HISI_PCM_PB_MIN_CHANNELS,
	.channels_max     = HISI_PCM_PB_MAX_CHANNELS,
	.buffer_bytes_max = PCM_DMA_BUF_PLAYBACK_PRIMARY_LEN,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = PCM_DMA_BUF_PLAYBACK_PRIMARY_LEN,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_PB_FIFO_SIZE,
};

/* define the capability of capture channel */
static const struct snd_pcm_hardware hisi_pcm_hardware_capture =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.channels_min     = HISI_PCM_CP_MIN_CHANNELS,
	.channels_max     = HISI_PCM_CP_MAX_CHANNELS,
	.buffer_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_CP_FIFO_SIZE,
};

/* define the capability of playback channel for direct*/
static const struct snd_pcm_hardware hisi_pcm_hardware_direct_playback =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_NONINTERLEAVED
		| SNDRV_PCM_INFO_MMAP
		| SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_PAUSE,
	.formats          = SNDRV_PCM_FMTBIT_S24_LE,
	.channels_min     = HISI_PCM_PB_MIN_CHANNELS,
	.channels_max     = HISI_PCM_PB_MAX_CHANNELS,
	.buffer_bytes_max = PCM_DMA_BUF_PLAYBACK_DIRECT_LEN,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = PCM_DMA_BUF_PLAYBACK_DIRECT_LEN,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_PB_FIFO_SIZE,
};

/* define the capability of playback channel for Modem */
static const struct snd_pcm_hardware hisi_pcm_hardware_modem_playback =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_NONINTERLEAVED
		| SNDRV_PCM_INFO_BLOCK_TRANSFER
		| SNDRV_PCM_INFO_PAUSE,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min     = HISI_PCM_PB_MIN_CHANNELS,
	.channels_max     = HISI_PCM_PB_MAX_CHANNELS,
	.buffer_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_PB_FIFO_SIZE,
};

/* define the capability of playback channel for lowlatency */
static const struct snd_pcm_hardware hisi_pcm_hardware_lowlatency_playback =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED
		| SNDRV_PCM_INFO_NONINTERLEAVED
		| SNDRV_PCM_INFO_MMAP
		| SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_PAUSE,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min     = HISI_PCM_PB_MIN_CHANNELS,
	.channels_max     = HISI_PCM_PB_MAX_CHANNELS,
	.buffer_bytes_max = PCM_DMA_BUF_PLAYBACK_PRIMARY_LEN,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = PCM_DMA_BUF_PLAYBACK_PRIMARY_LEN,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_PB_FIFO_SIZE,
};

/* define the capability of capture channel for lowlatency */
static const struct snd_pcm_hardware hisi_pcm_hardware_lowlatency_capture =
{
	.info             = SNDRV_PCM_INFO_INTERLEAVED,
	.formats          = SNDRV_PCM_FMTBIT_S16_LE,
	.rates            = SNDRV_PCM_RATE_48000,
	.channels_min     = HISI_PCM_CP_MIN_CHANNELS,
	.channels_max     = HISI_PCM_CP_MAX_CHANNELS,
	.buffer_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.period_bytes_min = HISI_PCM_MIN_BUFFER_SIZE,
	.period_bytes_max = HISI_PCM_MAX_BUFFER_SIZE,
	.periods_min      = HISI_PCM_MIN_PERIODS,
	.periods_max      = HISI_PCM_MAX_PERIODS,
	.fifo_size        = HISI_PCM_CP_FIFO_SIZE,
};

typedef struct _pcm_hifi_share_data
{
	volatile uint32_t is_play_finished;     /* is hifi updated play buffer */
	volatile uint32_t play_buf_offset;      /* new frame data addr offset */
	volatile uint32_t play_buf_size;        /* new frame buffer size */
	volatile uint32_t is_capture_finished;  /* is hifi updated capture buffer */
	volatile uint32_t capture_buf_offset;   /* new frame data addr offset */
	volatile uint32_t capture_buf_size;     /* new frame buffer size */
	volatile uint32_t play_time;            /* when hifi updated play buffer */
	volatile uint32_t capture_time;         /* when hifi updated capture buffer */
	volatile uint32_t set_play_time;        /* when kernel set new play buffer */
	volatile uint32_t set_capture_time;     /* when kernel set new capture buffer */
	volatile uint32_t sleep_time;           /* thread begin sleep time */
	volatile uint32_t wake_time;            /* thread end sleep time */
	volatile uint32_t check_play_time;      /* thread check is_play_finished flag changed time */
	volatile uint32_t play_data_avail;      /* alsa queued space for playback */
	volatile uint32_t check_capture_time;   /* thread check is_capture_finished flag changed time */
	volatile uint32_t capture_data_avail;   /* alsa queued space for capture */
	volatile uint32_t last_play_time;       /* last update play buffer time */
	volatile uint32_t last_capture_time;    /* last update capture buffer time */
} pcm_hifi_share_data;

struct hisi_pcm_private_data
{
	u32 pcm_cp_status_open;
	u32 pcm_pb_status_open;
	u32 pcm_direct_pb_status_open;
	u32 pcm_fast_pb_status_open;
	struct hisi_pcm_runtime_data pcm_rtd_playback;
	struct hisi_pcm_runtime_data pcm_rtd_capture;
	struct hisi_pcm_runtime_data pcm_rtd_direct_playback;
	struct hisi_pcm_runtime_data pcm_rtd_fast_playback;
	struct task_struct *pcm_run_thread;   /* currently only lowlatency use */
	pcm_hifi_share_data *hifi_share_data; /* communication share mem with hifi */
};

static struct hisi_pcm_private_data pdata;

spinlock_t g_pcm_cp_open_lock;
spinlock_t g_pcm_pb_open_lock;

extern int mailbox_get_timestamp(void);
static int hisi_pcm_notify_set_buf(struct snd_pcm_substream *substream);
static irq_rt_t hisi_pcm_notify_recv_isr(void *usr_para, void *mail_handle, unsigned int mail_len);
static irq_rt_t hisi_pcm_mb_isr_handle(unsigned short pcm_mode, struct snd_pcm_substream *substream);
static irq_rt_t hisi_pcm_isr_handle(struct snd_pcm_substream *substream);

static bool _is_valid_pcm_device(int pcm_device)
{
	switch (pcm_device) {
	case PCM_DEVICE_NORMAL:
	case PCM_DEVICE_DIRECT:
	case PCM_DEVICE_LOW_LATENCY:
		return true;
	default:
		return false;
	}
}

static void _dump_thread_info(uint16_t pcm_mode, char *str)
{
	pcm_hifi_share_data *thread_share_data = pdata.hifi_share_data;

	if (thread_share_data == NULL) {
		loge("pcm mode: %d, thread_share_data is null!\n", pcm_mode);
		return;
	}

	if (pcm_mode == SNDRV_PCM_STREAM_PLAYBACK) {
	    logw("%s: is_play_finished-%d, play_time-%d, set_play_time:%d, last_play_time-%d, check_play_time-%d\n",
		    str,
		    thread_share_data->is_play_finished,
		    thread_share_data->play_time,
		    thread_share_data->set_play_time,
		    thread_share_data->last_play_time,
		    thread_share_data->check_play_time);
    } else {
		logw("%s: is_capture_finished-%d, capture_time-%d, set_capture_time:%d, last_capture_time-%d, check_capture_time-%d\n",
			str,
			thread_share_data->is_capture_finished,
			thread_share_data->capture_time,
			thread_share_data->set_capture_time,
			thread_share_data->last_capture_time,
			thread_share_data->check_capture_time);
	}
}

static bool is_one_frame_finished(struct snd_pcm_substream *substream)
{
	pcm_hifi_share_data *thread_share_data = pdata.hifi_share_data;
	int ret = false;

	if (thread_share_data == NULL) {
		loge("thread_share_data is null!\n");
		return ret;
	}

	if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
		if (thread_share_data->is_play_finished) {
			thread_share_data->is_play_finished = false;
			thread_share_data->check_play_time = mailbox_get_timestamp();

			if (thread_share_data->check_play_time - thread_share_data->play_time > CHECK_FLAG_TIMEOUT) {
				_dump_thread_info(SNDRV_PCM_STREAM_PLAYBACK, "thread check play timeout");
			}
			ret = true;
		}
	} else {
		if (thread_share_data->is_capture_finished) {
			thread_share_data->is_capture_finished = false;
			thread_share_data->check_capture_time = mailbox_get_timestamp();

			if (thread_share_data->check_capture_time - thread_share_data->capture_time > CHECK_FLAG_TIMEOUT) {
				_dump_thread_info(SNDRV_PCM_STREAM_CAPTURE, "thread check capture timeout");
			}
			ret = true;
		}
	}

	return ret;
}

static int pcm_check_frame_thread(void *data)
{
	struct hisi_pcm_private_data *priv_data = (struct hisi_pcm_private_data *)data;
	struct hisi_pcm_runtime_data *prtd = NULL;
	bool should_schedule;

	while (!kthread_should_stop()) {
		if (!priv_data->pcm_cp_status_open && !priv_data->pcm_fast_pb_status_open) {
			logi("set lowlatency check frame thread to sleep state!\n");
			set_current_state(TASK_INTERRUPTIBLE); /*lint !e446 !e666*/
			schedule();
		}

		should_schedule = true;

		prtd = &priv_data->pcm_rtd_fast_playback;
		if (priv_data->pcm_fast_pb_status_open && STATUS_RUNNING == prtd->status) {
			if (is_one_frame_finished(prtd->substream)) {
				hisi_pcm_isr_handle(prtd->substream);
				should_schedule = false;
			}
		}

		prtd = &priv_data->pcm_rtd_capture;
		if (priv_data->pcm_cp_status_open && STATUS_RUNNING == prtd->status) {
			if (is_one_frame_finished(prtd->substream)) {
				hisi_pcm_isr_handle(prtd->substream);
				should_schedule = false;
			}
		}

		if (should_schedule) {
			usleep_range(1000, 1100);
		}
	}

	return 0;
}

static void pcm_thread_stop(void)
{
	if (pdata.pcm_fast_pb_status_open || pdata.pcm_cp_status_open)
		return;

	if (pdata.pcm_run_thread) {
		kthread_stop(pdata.pcm_run_thread);
		pdata.pcm_run_thread = NULL;

		if (pdata.hifi_share_data) {
			iounmap(pdata.hifi_share_data);
			pdata.hifi_share_data = NULL;
		}

		logi("pcm thread has stopped.\n");
	}
	return;
}

static int pcm_thread_start(void)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };
	pcm_hifi_share_data *thread_share_data;
	int ret;

	if (pdata.pcm_run_thread) {
		logi("pcm thread has already running, not need init.\n");
		return 0;
	}

	/* share memory remap */
	thread_share_data = ioremap_wc(DRV_DSP_PCM_THREAD_DATA_ADDR, sizeof(pcm_hifi_share_data));
	if (!thread_share_data) {
		loge("pcm share mem iormap failed!\n");
		return -ENOMEM;
	}
	memset(thread_share_data, 0, sizeof(pcm_hifi_share_data));/* unsafe_function_ignore: memset */

	/* create pcm thread for communication with hifi */
	pdata.pcm_run_thread = kthread_create(pcm_check_frame_thread, (void*)&pdata, "pcm_run_thread");
	if (IS_ERR(pdata.pcm_run_thread)) {
		loge("create check frame thread failed!\n");
		pdata.pcm_run_thread = NULL;
		iounmap(thread_share_data);
		thread_share_data = NULL;
		return -ENOMEM;
	}

	pdata.hifi_share_data = thread_share_data;

	/* set highest rt prio */
	ret = sched_setscheduler(pdata.pcm_run_thread, SCHED_FIFO, &param);
	if (ret)
		loge("set thread schedule priority failed\n");

	/* do not wakeup process in this stage, wakeup when lowlatency stream start */
	logi("create pcm_run_thread success\n");

	return 0;
}

static int pcm_set_share_data(uint16_t pcm_mode, uint32_t data_addr, uint32_t data_len)
{
	pcm_hifi_share_data *thread_share_data = pdata.hifi_share_data;

	if (thread_share_data == NULL) {
		loge("pcm mode: %d, thread_share_data is null!\n", pcm_mode);
		return -EINVAL;
	}

	if (pcm_mode == SNDRV_PCM_STREAM_PLAYBACK) {
		thread_share_data->play_buf_offset = data_addr;
		thread_share_data->play_buf_size = data_len;
		thread_share_data->last_play_time = thread_share_data->set_play_time;
		thread_share_data->set_play_time = mailbox_get_timestamp();

		if (thread_share_data->set_play_time - thread_share_data->last_play_time > CHECK_UPDATE_TIMEOUT) {
			_dump_thread_info(pcm_mode, "set play addr timeout");
		}
	} else {
		thread_share_data->capture_buf_offset = data_addr;
		thread_share_data->capture_buf_size = data_len;
		thread_share_data->last_capture_time = thread_share_data->set_capture_time;
		thread_share_data->set_capture_time = mailbox_get_timestamp();

		if (thread_share_data->set_capture_time - thread_share_data->last_capture_time > CHECK_UPDATE_TIMEOUT) {
			_dump_thread_info(pcm_mode, "set capture addr timeout");
		}
	}

	return 0;
}

static irq_rt_t hisi_pcm_isr_handle(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct hisi_pcm_runtime_data *prtd = NULL;
	snd_pcm_uframes_t rt_period_size = 0;
	unsigned int num_period = 0;
	snd_pcm_uframes_t avail = 0;
	unsigned short pcm_mode = 0;

	IN_FUNCTION;

	if (NULL == substream) {
		loge("ISR substream is null, error\n");
		return IRQ_HDD_PTR;
	}

	if (NULL == substream->runtime) {
		loge("substream->runtime == NULL\n");
		return IRQ_HDD_PTR;
	}

	pcm_mode = (unsigned short)substream->stream;
	prtd    = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	rt_period_size  = substream->runtime->period_size;
	num_period      = substream->runtime->periods;

	if (NULL == prtd) {
		loge("prtd == NULL\n");
		return IRQ_HDD_PTR;
	}

	if (STATUS_RUNNING != prtd->status) {
		logd("dma status %d error\n", prtd->status);
		return IRQ_HDD_STATUS;
	}

	if (SNDRV_PCM_STREAM_CAPTURE == pcm_mode) {
		avail = (snd_pcm_uframes_t)snd_pcm_capture_hw_avail(substream->runtime);
	} else {
		avail = (snd_pcm_uframes_t)snd_pcm_playback_hw_avail(substream->runtime);
	}
	if (avail < rt_period_size) {
		logw("pcm_mode:%d: avail(%d) < rt_period_size(%d)\n", pcm_mode, (int)avail, (int)rt_period_size);
		return IRQ_HDD_SIZE;
	} else {
		++prtd->period_cur;
		prtd->period_cur = (prtd->period_cur) % num_period;

		snd_pcm_period_elapsed(substream);

		ret = hisi_pcm_notify_set_buf(substream);
		if(ret < 0) {
			loge("hisi_pcm_notify_pcm_set_buf(%d)\n", ret);
			return IRQ_HDD_ERROR;
		}

		prtd->period_next = (prtd->period_next + 1) % num_period;
	}

	OUT_FUNCTION;

	return IRQ_HDD;
}

static irq_rt_t hisi_pcm_mb_isr_handle(unsigned short pcm_mode,
		struct snd_pcm_substream *substream)
{
	irq_rt_t ret = IRQ_NH;

	if (NULL == substream) {
		loge("substream is NULL\n");
		return IRQ_HDD_PTRS;
	}

	switch (pcm_mode)
	{
	case SNDRV_PCM_STREAM_PLAYBACK:
		/* lock used to protect close while doing _intr_handle_pb */
		spin_lock_bh(&g_pcm_pb_open_lock);
		if ((0 == pdata.pcm_pb_status_open) && (0 == pdata.pcm_direct_pb_status_open) && (0 == pdata.pcm_fast_pb_status_open)) {
			logd("pcm playback closed\n");
			spin_unlock_bh(&g_pcm_pb_open_lock);
			return IRQ_HDD;
		}

		ret = hisi_pcm_isr_handle(substream);
		spin_unlock_bh(&g_pcm_pb_open_lock);
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		/* lock used to protect close while doing _intr_handle_cp */
		spin_lock_bh(&g_pcm_cp_open_lock);

		if (0 == pdata.pcm_cp_status_open) {
			logd("pcm capture closed\n");
			spin_unlock_bh(&g_pcm_cp_open_lock);;
			return IRQ_HDD;
		}
		ret = hisi_pcm_isr_handle(substream);
		spin_unlock_bh(&g_pcm_cp_open_lock);
		break;
	default:
		ret = IRQ_NH_MODE;
		loge("PCM Mode error(%d)\n", pcm_mode);
		break;
	}

	return ret;
}

static int hisi_pcm_mailbox_send_data(void *pmsg_body, unsigned int msg_len,
		unsigned int msg_priority)
{
	unsigned int ret = 0;
	static unsigned int err_count = 0;

	ret = DRV_MAILBOX_SENDMAIL(MAILBOX_MAILCODE_ACPU_TO_HIFI_AUDIO, pmsg_body, msg_len);
	if (MAILBOX_OK != ret) {
		if (err_count % 50 == 0) {
#ifdef CONFIG_HUAWEI_DSM
			audio_dsm_report_info(AUDIO_CODEC, DSM_PCM_DRV_UPDATE_PCM_BUFF_DELAY, "update pcm buffer delay");
#endif
			HiLOGE("audio", "Hi6210_pcm","mailbox ap to hifi fail,ret=%d, maybe ap is abnormal\n", ret);
		}
		err_count++;
	} else {
		err_count = 0;
	}

	return (int)ret;
}

static int hisi_pcm_notify_set_buf(struct snd_pcm_substream *substream)
{
	int ret = 0;
	unsigned int period_size;
	struct hifi_channel_set_buffer msg_body = {0};
	unsigned short pcm_mode = (unsigned short)substream->stream;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;

	IN_FUNCTION;

	if (NULL == prtd) {
		loge("prtd is null, error\n");
		return -EINVAL;
	}
	period_size = prtd->period_size;

	if ((SNDRV_PCM_STREAM_PLAYBACK != pcm_mode) && (SNDRV_PCM_STREAM_CAPTURE != pcm_mode)) {
		loge("pcm mode %d invalid\n", pcm_mode);
		return -EINVAL;
	}

	msg_body.msg_type   = (unsigned short)HI_CHN_MSG_PCM_SET_BUF;
	msg_body.pcm_mode   = pcm_mode;
	msg_body.pcm_device = (unsigned short)substream->pcm->device;
	msg_body.data_addr  = (substream->runtime->dma_addr + prtd->period_next * period_size) - PCM_DMA_BUF_0_PLAYBACK_BASE;
	msg_body.data_len   = period_size;

	if (STATUS_RUNNING != prtd->status) {
		logd("pcm status %d error \n", prtd->status);
		return -EINVAL;
	}

	if (substream->pcm->device == PCM_DEVICE_LOW_LATENCY) {
		ret = pcm_set_share_data(msg_body.pcm_mode, msg_body.data_addr, msg_body.data_len);
	} else {
		ret = hisi_pcm_mailbox_send_data( &msg_body, sizeof(struct hifi_channel_set_buffer), 0 );
	}
	if (OK != ret)
		ret = -EBUSY;

	/* trace_dot(APCM,"5",0); */
	OUT_FUNCTION;

	return ret;
}

static void print_pcm_timeout(unsigned int pre_time, const char *print_type, unsigned int time_delay)
{
	unsigned int  delay_time;
	unsigned int  curr_time;

	if (hifi_misc_get_platform_type() != HIFI_DSP_PLATFORM_ASIC) {
		return;
	}

	curr_time = (unsigned int)mailbox_get_timestamp();
	delay_time = curr_time - pre_time;

	if (delay_time > (HISI_PCM_WORK_DELAY_1MS * time_delay)) {
		logw("[%d]:%s, delaytime %u.\n", mailbox_get_timestamp(), print_type, delay_time);
	}
}

static long get_snd_current_millisec(void)
{
	struct timeval last_update;
	long curr_time;

	do_gettimeofday(&last_update);
	curr_time = last_update.tv_sec * 1000 + last_update.tv_usec / 1000;
	return curr_time;
}

void snd_pcm_print_timeout(struct snd_pcm_substream *substream, unsigned int timeout_type)
{
	long delay_time;
	long curr_time;
	static unsigned int timeout_count[SND_TIMEOUT_TYPE_MAX] = {0};
	const char *timeout_str[SND_TIMEOUT_TYPE_MAX] = {
		"pcm write interval timeout",
		"pcm write proc timeout",
		"pcm read interval timeout",
		"pcm read proc timeout"};

	if (hifi_misc_get_platform_type() != HIFI_DSP_PLATFORM_ASIC) {
		return;
	}

	if (substream == NULL) {
		loge("substream is null\n");
		return;
	}

	if (timeout_type >= SND_TIMEOUT_TYPE_MAX) {
		return;
	}

	curr_time = get_snd_current_millisec();
	delay_time = curr_time - substream->runtime->pre_time;

	if (delay_time > ALSA_TIMEOUT_MILLISEC && (substream->runtime->pre_time != 0)) {
		timeout_count[timeout_type]++;

		if ((timeout_count[timeout_type] == 1) || (timeout_count[timeout_type] % 20 == 0)) {
			logw("%s, delay time: %ld ms.\n", timeout_str[timeout_type], delay_time);
        }
	} else {
		timeout_count[timeout_type] = 0;
	}

	if (timeout_type == SND_TIMEOUT_TYPE_WRITE_INTERVAL
		|| timeout_type == SND_TIMEOUT_TYPE_READ_INTERVAL) {
		substream->runtime->pre_time = curr_time;
	}
}
EXPORT_SYMBOL(snd_pcm_print_timeout);

void snd_pcm_reset_pre_time(struct snd_pcm_substream *substream)
{
	if (substream == NULL) {
		loge("substream is null\n");
		return;
	}

	substream->runtime->pre_time = 0;
}
EXPORT_SYMBOL(snd_pcm_reset_pre_time);

static int hisi_pcm_check_substream(struct snd_pcm_substream * substream)
{
	if (NULL == substream) {
		loge("substream from hifi is NULL\n");
		return -1;
	}

	if (NULL == substream->runtime) {
		loge("substream runtime is NULL\n");
		return -1;
	}

	if (pdata.pcm_rtd_playback.substream != substream
		&& pdata.pcm_rtd_capture.substream != substream
		&& pdata.pcm_rtd_direct_playback.substream != substream
		&& pdata.pcm_rtd_fast_playback.substream != substream) {
		loge("substream from hifi is invalid\n");
		return -1;
	}

	return 0;
}

static irq_rt_t hisi_pcm_notify_recv_isr(void *usr_para, void *mail_handle, unsigned int mail_len)
{
	struct snd_pcm_substream * substream    = NULL;
	struct hisi_pcm_runtime_data *prtd        = NULL;
	struct hifi_chn_pcm_period_elapsed mail_buf;
	unsigned int mail_size          = mail_len;
	unsigned int ret_mail           = MAILBOX_OK;
	irq_rt_t ret                    = IRQ_NH;
	unsigned int start_time = 0;
	const char *print_type[2] = {"recv pcm msg timeout", "process pcm msg timeout"};

	UNUSED_PARAMETER(usr_para);

	start_time = (unsigned int)mailbox_get_timestamp();
	memset(&mail_buf, 0, sizeof(struct hifi_chn_pcm_period_elapsed));/* unsafe_function_ignore: memset */

	/*get the data from mailbox*/

	ret_mail = DRV_MAILBOX_READMAILDATA(mail_handle, (unsigned char*)&mail_buf, &mail_size);
	if ((ret_mail != MAILBOX_OK)
		|| (mail_size == 0)
			|| (mail_size > sizeof(struct hifi_chn_pcm_period_elapsed)))
	{
		loge("Empty point or data length error! size: %d  ret_mail:%d sizeof(struct hifi_chn_pcm_period_elapsed):%lu\n", mail_size, ret_mail, sizeof(struct hifi_chn_pcm_period_elapsed));
		HiLOGE("audio", "Hi6210_pcm", "Empty point or data length error! size: %d\n", mail_size);
		return IRQ_NH_MB;
	}

	substream = INT_TO_ADDR(mail_buf.substream_l32,mail_buf.substream_h32);
	if (hisi_pcm_check_substream(substream))
		return IRQ_NH_OTHERS;

	prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	if (NULL == prtd) {
		loge("prtd is NULL\n");
		return IRQ_NH_OTHERS;
	}
	if (STATUS_STOP == prtd->status) {
		logi("process has stopped\n");
		return IRQ_NH_OTHERS;
	}

	switch(mail_buf.msg_type) {
		case HI_CHN_MSG_PCM_PERIOD_ELAPSED:
			/* check if elapsed msg is timeout */
			print_pcm_timeout(mail_buf.msg_timestamp, print_type[0], 10);
			ret = hisi_pcm_mb_isr_handle(mail_buf.pcm_mode, substream);
			if (ret == IRQ_NH)
				loge("mb msg handle err, ret : %d\n", ret);
			break;
		case HI_CHN_MSG_PCM_PERIOD_STOP:
			if (STATUS_STOPPING == prtd->status) {
				prtd->status = STATUS_STOP;
				logi("device %d mode %d stop now !\n", substream->pcm->device, mail_buf.pcm_mode);
			}
			break;
		default:
			loge("msg_type 0x%x\n", mail_buf.msg_type);
			break;
	}
	/* check if isr proc is timeout */
	print_pcm_timeout(start_time, print_type[1], 20);

	return ret;
}

static int hisi_pcm_notify_isr_register(irq_hdl_t pisr)
{
	int ret                     = 0;
	unsigned int mailbox_ret    = MAILBOX_OK;

	if (NULL == pisr) {
		loge("pisr==NULL!\n");
		ret = ERROR;
	} else {
		mailbox_ret = DRV_MAILBOX_REGISTERRECVFUNC(MAILBOX_MAILCODE_HIFI_TO_ACPU_AUDIO, (void *)pisr, NULL);
		if (MAILBOX_OK != mailbox_ret) {
			ret = ERROR;
			loge("ret : %d,0x%x\n", ret, MAILBOX_MAILCODE_HIFI_TO_ACPU_AUDIO);
		}
	}

	return ret;
}

static int hisi_pcm_notify_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct hifi_chn_pcm_open open_msg = {0};
	struct hifi_chn_pcm_hw_params hw_params_msg  = {0};
	unsigned int params_value = 0;
	unsigned int infreq_index = 0;

	IN_FUNCTION;

	open_msg.msg_type = (unsigned short)HI_CHN_MSG_PCM_OPEN;
	hw_params_msg.msg_type = (unsigned short)HI_CHN_MSG_PCM_HW_PARAMS;
	hw_params_msg.pcm_mode = open_msg.pcm_mode = (unsigned short)substream->stream;
	hw_params_msg.pcm_device = open_msg.pcm_device = (unsigned short)substream->pcm->device;

	/* check channels  : mono or stereo */
	params_value = params_channels(params);
	if ((HISI_PCM_CP_MIN_CHANNELS <= params_value) && (HISI_PCM_CP_MAX_CHANNELS >= params_value)) {
		open_msg.config.channels = params_value;
		hw_params_msg.channel_num = params_value;
	} else {
		loge("DAC not support %d channels\n", params_value);
		return -EINVAL;
	}

	/* check samplerate */
	params_value = params_rate(params);
	logi("rate is %d\n", params_value);

	for (infreq_index = 0; infreq_index < ARRAY_SIZE(freq); infreq_index++) {
		if(params_value == freq[infreq_index])
			break;
	}

	if (ARRAY_SIZE(freq) <= infreq_index) {
		loge("rate %d not support\n", params_value);
		return -EINVAL;
	}

	open_msg.config.rate = params_value;

	hw_params_msg.sample_rate = params_value;

	/* check format */
	params_value = (unsigned int)params_format(params);
	if (params_value == SNDRV_PCM_FORMAT_S24_LE) {
		params_value = PCM_FORMAT_S24_LE_LA;
	} else {
		params_value = PCM_FORMAT_S16_LE;
	}

	hw_params_msg.format = params_value;

	open_msg.config.format = params_value;
	open_msg.config.period_size = params_period_size(params);
	open_msg.config.period_count = params_periods(params);

	ret = hisi_pcm_mailbox_send_data(&open_msg, sizeof(open_msg), 0);

	/* send hw_params */
	ret += hisi_pcm_mailbox_send_data(&hw_params_msg, sizeof(hw_params_msg), 0);

	OUT_FUNCTION;

	return ret;
}

static int hisi_pcm_notify_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;

	UNUSED_PARAMETER(substream);

	return ret;
}

static int hisi_pcm_notify_prepare(struct snd_pcm_substream *substream)
{
	int ret = OK;

	UNUSED_PARAMETER(substream);

	return ret;
}

static int hisi_pcm_notify_trigger(int cmd, struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct hifi_chn_pcm_trigger msg_body = {0};
	unsigned int period_size = 0;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;

	IN_FUNCTION;

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	period_size = prtd->period_size;

	msg_body.msg_type   	= (unsigned short)HI_CHN_MSG_PCM_TRIGGER;
	msg_body.pcm_mode   	= (unsigned short)substream->stream;
	msg_body.pcm_device   	= (unsigned short)substream->pcm->device;
	msg_body.tg_cmd     	= (unsigned short)cmd;
	msg_body.substream_l32  = GET_LOW32(substream);
	msg_body.substream_h32  = GET_HIG32(substream);

	if ((SNDRV_PCM_TRIGGER_START == cmd)
		|| (SNDRV_PCM_TRIGGER_RESUME == cmd)
		|| (SNDRV_PCM_TRIGGER_PAUSE_RELEASE == cmd)) {
		msg_body.data_addr = (substream->runtime->dma_addr + prtd->period_next * period_size) - PCM_DMA_BUF_0_PLAYBACK_BASE;
		msg_body.data_len  = period_size;
	}

	/* update share memory info */
	if ((msg_body.pcm_device == PCM_DEVICE_LOW_LATENCY) && (cmd == SNDRV_PCM_TRIGGER_START)) {
		ret = wake_up_process(pdata.pcm_run_thread);
		logi("lowlatency check frame thread wake up %s!\n", ret ? "success" : "fail");

		ret = pcm_set_share_data(msg_body.pcm_mode, msg_body.data_addr, msg_body.data_len);
		if (ret)
			loge("set share data fail, ret:%d!\n", ret);
	}

	ret = hisi_pcm_mailbox_send_data(&msg_body, sizeof(msg_body), 0);

	OUT_FUNCTION;

	return ret;
}

static int hisi_pcm_notify_open(struct snd_pcm_substream *substream)
{
	int ret = 0;

	UNUSED_PARAMETER(substream);

	return ret;
}

static int hisi_pcm_notify_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct hifi_chn_pcm_close msg_body  = {0};

	IN_FUNCTION;

	msg_body.msg_type = (unsigned short)HI_CHN_MSG_PCM_CLOSE;
	msg_body.pcm_mode = (unsigned short)substream->stream;
	msg_body.pcm_device = (unsigned short)substream->pcm->device;
	ret = hisi_pcm_mailbox_send_data(&msg_body, sizeof(msg_body), 0);
	if (ret)
		ret = -EBUSY;

	OUT_FUNCTION;

	return ret;
}

static int hisi_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	size_t bytes = params_buffer_bytes(params);
	int device = substream->pcm->device;

	if (!_is_valid_pcm_device(device)) {
		return ret;
	}

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	ret = snd_pcm_lib_malloc_pages(substream, bytes);
	if (ret < 0) {
		loge("snd_pcm_lib_malloc_pages ret : %d\n", ret);
		return ret;
	}

	spin_lock(&prtd->lock);
	prtd->period_size = params_period_bytes(params);
	prtd->period_next = 0;
	spin_unlock(&prtd->lock);

	ret = hisi_pcm_notify_hw_params(substream, params);
	if (ret < 0) {
		loge("pcm mode %d error\n", substream->stream);
		snd_pcm_lib_free_pages(substream);
	}

	return ret;
}

static int hisi_pcm_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;
	int i   = 0;
	struct hisi_pcm_runtime_data *prtd = NULL;
	int device = substream->pcm->device;

	prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;

	if (!_is_valid_pcm_device(device)) {
		return ret;
	}

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	for(i = 0; i < 30 ; i++) {  /* wait for dma ok */
		if (STATUS_STOP == prtd->status) {
			break;
		} else {
			msleep(10);
		}
	}
	if (30 == i) {
		logi("timeout for waiting for stop info from other\n");
	}

	ret = hisi_pcm_notify_hw_free(substream);
	if (ret < 0) {
		loge("free fail device %d\n", substream->pcm->device);
	}

	ret = snd_pcm_lib_free_pages(substream);

	return ret;
}

static int hisi_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	int device = substream->pcm->device;

	if (!_is_valid_pcm_device(device)) {
		return ret;
	}

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	/* init prtd */
	spin_lock(&prtd->lock);
	prtd->status        = STATUS_STOP;
	prtd->period_next   = 0;
	prtd->period_cur    = 0;
	spin_unlock(&prtd->lock);

	ret = hisi_pcm_notify_prepare(substream);

	return ret;
}

static int hisi_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	unsigned int num_periods = runtime->periods;
	int device = substream->pcm->device;

	if (!_is_valid_pcm_device(device)) {
		return ret;
	}

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	logi("device %d mode %d trigger %d \n", substream->pcm->device, substream->stream, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = hisi_pcm_notify_trigger(cmd, substream);
		if (ret < 0) {
			loge("trigger %d failed, ret : %d\n", cmd, ret);
		} else {
			spin_lock(&prtd->lock);
			prtd->status = STATUS_RUNNING;
			prtd->period_next = (prtd->period_next + 1) % num_periods;
			spin_unlock(&prtd->lock);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock(&prtd->lock);
		prtd->status = STATUS_STOPPING;
		spin_unlock(&prtd->lock);

		ret = hisi_pcm_notify_trigger(cmd, substream);
		if (ret < 0) {
			loge("hisi_pcm_notify_pcm_trigger ret : %d\n", ret);
		}

		break;

	default:
		loge("trigger cmd error : %d\n", cmd);
		ret = -EINVAL;
		break;

	}

	return ret;
}

static snd_pcm_uframes_t hisi_pcm_pointer(struct snd_pcm_substream *substream)
{
	long frame = 0L;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hisi_pcm_runtime_data *prtd = (struct hisi_pcm_runtime_data *)substream->runtime->private_data;
	int device = substream->pcm->device;

	if (!_is_valid_pcm_device(device)) {
		return (snd_pcm_uframes_t)frame;
	}

	if (NULL == prtd) {
		loge("prtd is null\n");
		return -EINVAL;
	}

	frame = bytes_to_frames(runtime, prtd->period_cur * prtd->period_size);
	if (frame >= runtime->buffer_size)
		frame = 0;

	return (snd_pcm_uframes_t)frame;
}

static int hisi_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct hisi_pcm_runtime_data *prtd = NULL;

	if ((substream->stream != SNDRV_PCM_STREAM_PLAYBACK) && (substream->stream != SNDRV_PCM_STREAM_CAPTURE)) {
		loge("pcm mode %d error\n", substream->stream);
		return -1;
	}

	switch (substream->pcm->device) {
	case PCM_DEVICE_NORMAL:
		if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_pb_status_open = (u32)1;
			spin_unlock_bh(&g_pcm_pb_open_lock);
			snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_playback);
			prtd = &pdata.pcm_rtd_playback;
			prtd->substream = substream;
		} else {
			spin_lock_bh(&g_pcm_cp_open_lock);
			pdata.pcm_cp_status_open = (u32)1;
			spin_unlock_bh(&g_pcm_cp_open_lock);
			snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_capture);
			prtd = &pdata.pcm_rtd_capture;
			prtd->substream = substream;
		}
		break;
	case PCM_DEVICE_DIRECT:
		if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_direct_pb_status_open = (u32)1;
			spin_unlock_bh(&g_pcm_pb_open_lock);
			snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_direct_playback);
			prtd = &pdata.pcm_rtd_direct_playback;
			prtd->substream = substream;
		}
		break;
	case PCM_DEVICE_LOW_LATENCY:
		if (SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_fast_pb_status_open = (u32)1;
			spin_unlock_bh(&g_pcm_pb_open_lock);
			snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_lowlatency_playback);
			prtd = &pdata.pcm_rtd_fast_playback;
			prtd->substream = substream;
		} else {
			spin_lock_bh(&g_pcm_cp_open_lock);
			pdata.pcm_cp_status_open = (u32)1;
			spin_unlock_bh(&g_pcm_cp_open_lock);
			snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_lowlatency_capture);
			prtd = &pdata.pcm_rtd_capture;
			prtd->substream = substream;
		}
		break;
	default:
		ret = snd_soc_set_runtime_hwparams(substream, &hisi_pcm_hardware_modem_playback);
		break;
	}

	if (prtd) {
		spin_lock(&prtd->lock);
		prtd->period_cur  = 0;
		prtd->period_next = 0;
		prtd->period_size = 0;
		prtd->status      = STATUS_STOP;
		substream->runtime->private_data = prtd;
		spin_unlock(&prtd->lock);

		ret = hisi_pcm_notify_open(substream);
	}

	return ret;
}

static int hisi_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	int device = substream->pcm->device;

	if (!_is_valid_pcm_device(device)) {
		return ret;
	}

	if (NULL == substream->runtime->private_data) {
		logi("prtd is null\n");
	}

	logi("device %d, mode %d close\n", substream->pcm->device, substream->stream);
	switch (substream->pcm->device) {
	case PCM_DEVICE_NORMAL:
		if(SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_pb_status_open = (u32)0;
			ret = hisi_pcm_notify_close(substream);
			spin_unlock_bh(&g_pcm_pb_open_lock);
		} else {
			spin_lock_bh(&g_pcm_cp_open_lock);
			pdata.pcm_cp_status_open = (u32)0;
			ret = hisi_pcm_notify_close(substream);
			spin_unlock_bh(&g_pcm_cp_open_lock);
		}
		break;
	case PCM_DEVICE_DIRECT:
		if(SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_direct_pb_status_open = (u32)0;
			ret = hisi_pcm_notify_close(substream);
			spin_unlock_bh(&g_pcm_pb_open_lock);
		}
		break;
	case PCM_DEVICE_LOW_LATENCY:
		if(SNDRV_PCM_STREAM_PLAYBACK == substream->stream) {
			spin_lock_bh(&g_pcm_pb_open_lock);
			pdata.pcm_fast_pb_status_open = (u32)0;
			ret = hisi_pcm_notify_close(substream);
			spin_unlock_bh(&g_pcm_pb_open_lock);
		} else {
			spin_lock_bh(&g_pcm_cp_open_lock);
			pdata.pcm_cp_status_open = (u32)0;
			ret = hisi_pcm_notify_close(substream);
			spin_unlock_bh(&g_pcm_cp_open_lock);
		}
		break;
	default:
		break;
	}

	substream->runtime->private_data = NULL;

	return ret;
}

/* define all pcm ops of hisi pcm */
static struct snd_pcm_ops hisi_pcm_ops = {
	.open       = hisi_pcm_open,
	.close      = hisi_pcm_close,
	.ioctl      = snd_pcm_lib_ioctl,
	.hw_params  = hisi_pcm_hw_params,
	.hw_free    = hisi_pcm_hw_free,
	.prepare    = hisi_pcm_prepare,
	.trigger    = hisi_pcm_trigger,
	.pointer    = hisi_pcm_pointer,
};


static int preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream 	*substream = pcm->streams[stream].substream;
	struct snd_dma_buffer 		*buf = &substream->dma_buffer;

	if ((pcm->device >= PCM_DEVICE_MAX) ||(stream >= PCM_STREAM_MAX)) {
		loge("Invalid argument  : device %d stream %d \n", pcm->device, stream);
		return -EINVAL;
	}

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->addr = g_pcm_dma_buf_config[pcm->device][stream].pcm_dma_buf_base;
	buf->bytes = g_pcm_dma_buf_config[pcm->device][stream].pcm_dma_buf_len;
	buf->area = ioremap(buf->addr, buf->bytes);

	if (!buf->area) {
		loge("dma buf area error\n");
		return -ENOMEM;
	}

	return 0;
}

static void free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	logd("Entered %s\n", __func__);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		iounmap(buf->area);

		buf->area = NULL;
		buf->addr = 0;
	}
}

static int hisi_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm  *pcm  = rtd->pcm;
	bool need_register_isr;

	IN_FUNCTION;

	if (!card->dev->dma_mask) {
		logi("dev->dma_mask not set\n");
		card->dev->dma_mask = &hisi_pcm_dmamask;
	}

	if (!card->dev->coherent_dma_mask) {
		logi("dev->coherent_dma_mask not set\n");
		card->dev->coherent_dma_mask = hisi_pcm_dmamask;
	}

	logi("PLATFORM machine set\n");

	need_register_isr = ((pcm->device == PCM_DEVICE_NORMAL)
		|| (pcm->device == PCM_DEVICE_DIRECT)
		|| (pcm->device == PCM_DEVICE_LOW_LATENCY));

	if (need_register_isr) {
		/* register callback */
		ret = hisi_pcm_notify_isr_register((void*)hisi_pcm_notify_recv_isr);
		if (ret) {
			loge("notify Isr register error : %d\n", ret);
			goto out;
		}
	} else {
		logi("We just alloc space for the the four device \n");
		goto out;
	}
	logi("pcm-device = %d\n", pcm->device);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = preallocate_dma_buffer(pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

	/* init lowlatency stream thread */
	if (pcm->device == PCM_DEVICE_LOW_LATENCY) {
		pcm_thread_start();
	}

out:
	OUT_FUNCTION;

	return ret;
}

static void hisi_pcm_free(struct snd_pcm *pcm)
{
	IN_FUNCTION;

	if (pcm->device == PCM_DEVICE_LOW_LATENCY) {
		pcm_thread_stop();
	}

	free_dma_buffers(pcm);

	OUT_FUNCTION;
}

struct snd_soc_platform_driver hisi_pcm_platform = {
	.ops      = &hisi_pcm_ops,
	.pcm_new  =  hisi_pcm_new,
	.pcm_free =  hisi_pcm_free,
};

static int  hisi_pcm_platform_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;

	IN_FUNCTION;

	memset(&pdata, 0, sizeof(pdata));/* unsafe_function_ignore: memset */

	ret = snd_soc_register_component(&pdev->dev, &hisi_pcm_component,
			hisi_pcm_dai, ARRAY_SIZE(hisi_pcm_dai));
	if (ret) {
		loge("snd_soc_register_dai return %d\n" ,ret);
		goto probe_failed;
	}

	/* register platform (name : hi6210-hifi) */
	dev_set_name(&pdev->dev, HISI_PCM_HIFI);
	ret = snd_soc_register_platform(&pdev->dev, &hisi_pcm_platform);
	if (ret) {
		loge("snd_soc_register_platform return %d\n", ret);
		snd_soc_unregister_component(&pdev->dev);
		goto probe_failed;
	}

	spin_lock_init(&pdata.pcm_rtd_playback.lock);
	spin_lock_init(&pdata.pcm_rtd_capture.lock);
	spin_lock_init(&pdata.pcm_rtd_direct_playback.lock);
	spin_lock_init(&pdata.pcm_rtd_fast_playback.lock);
	spin_lock_init(&g_pcm_cp_open_lock);
	spin_lock_init(&g_pcm_pb_open_lock);

	OUT_FUNCTION;

	return ret;

probe_failed:
	OUT_FUNCTION;
	return ret;
}

static int hisi_pcm_platform_remove(struct platform_device *pdev)
{
	memset(&pdata, 0, sizeof(pdata));/* unsafe_function_ignore: memset */

	snd_soc_unregister_platform(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}


static const struct of_device_id hisi_pcm_hifi_match_table[] =
{
	{.compatible = HISI_PCM_HIFI, },
	{ },
};
static struct platform_driver hisi_pcm_platform_driver = {
	.driver = {
		.name  = HISI_PCM_HIFI,
		.owner = THIS_MODULE,
		.of_match_table = hisi_pcm_hifi_match_table,
	},
	.probe  = hisi_pcm_platform_probe,
	.remove = hisi_pcm_platform_remove,
};

static int __init hisi_pcm_init(void)
{
	IN_FUNCTION;
	return platform_driver_register(&hisi_pcm_platform_driver);
}
module_init(hisi_pcm_init);

static void __exit hisi_pcm_exit(void)
{
	platform_driver_unregister(&hisi_pcm_platform_driver);
}
module_exit(hisi_pcm_exit);

MODULE_AUTHOR("S00212991");
MODULE_DESCRIPTION("HISI HIFI platform driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hifi");

