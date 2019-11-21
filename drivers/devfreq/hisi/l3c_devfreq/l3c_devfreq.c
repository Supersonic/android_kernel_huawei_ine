#include <linux/devfreq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include "governor.h"

#include <linux/hisi/hisi_mailbox.h>
#include <linux/hisi/hisi_rproc.h>
#include <linux/hisi/ipc_msg.h>
#include <linux/perf_event.h>
#include <linux/cpufreq.h>


#ifdef CONFIG_HISI_HW_VOTE_L3C_FREQ
#include <linux/hisi/hisi_hw_vote.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/l3c_devfreq.h>

#define L3C_DEVFREQ_PLATFORM_DEVICE_NAME			"l3c_devfreq"
#define L3C_DEVFREQ_GOVERNOR_NAME					"l3c_governor"

#define L3C_DEVFREQ_DEFAULT_POLLING_MS			60


enum ev_index {
	L3D_IDX,
	BUS_ACCESS_IDX,
	CYCLE_IDX,
	NUM_EVENTS
};

enum cluster_idx {
	LITTLE_CLUSTER = 0,
	MID_CLUSTER,
	BIG_CLUSTER,
	NUM_CLUSTERS
};

#define L3D_EV			0x2B
#define CYCLE_EV		0x11
#define BUS_ACCESS_EV	0x19



struct evt_count {
	unsigned long l3_count;
	unsigned long bus_access_count;
	unsigned long cycle_count;
};

struct event_data {
	struct perf_event *pevent;
	unsigned long prev_count;
};

struct l3c_hwmon_data {
	struct event_data events[NUM_EVENTS];
	bool active;
	struct mutex active_lock;
};

struct l3c_hwmon {
	struct evt_count count;
	struct l3c_hwmon_data l3c_hw_data;
};

struct l3c_devfreq_data {
	u32 freq_min;
	u32 freq_max;
};

struct l3c_devfreq {
	struct l3c_devfreq_data *l3c_data;
	struct devfreq *devfreq;
	struct platform_device *pdev;
	struct devfreq_dev_profile *devfreq_profile;
#ifdef CONFIG_HISI_HW_VOTE_L3C_FREQ
	struct hvdev *l3c_hvdev;
#endif
	u32 polling_ms;
	unsigned long initial_freq;
	u32 hv_supported;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	unsigned int *freq_table;
#else
	unsigned long *freq_table;
#endif
	unsigned int *volt_table;
	int freq_table_len;

	u64 *load_map;
	int load_map_len;

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	struct workqueue_struct *update_wq;
	struct work_struct update_handle;
#endif

	struct mutex lock;

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	struct hrtimer poll_timer;
#endif

	unsigned long cur_freq;
	unsigned long boost_freq;
	unsigned long lit_link_freq;
	unsigned long big_link_freq;
	unsigned long load_map_max;

	struct l3c_hwmon *hw;
	unsigned long l3c_bw;
	u64 l3c_bw_max;
	u64 l3_bus_ratio;

	/* Contains state for the algorithm. It is clean during resume */
	struct {
		unsigned long usec_delta;
		unsigned long last_update;
	} alg;

	struct notifier_block l3c_trans_notify;

	struct mutex allow_lock;
};


static struct l3c_devfreq_data device_data;

static unsigned int allow_fcm_boost;

static const struct of_device_id l3c_devfreq_id[] = {
	{.compatible = "hisi,l3c_devfreq"},
	{}
};


static int l3c_devfreq_set_target_freq_ipc(struct device *dev,
					 unsigned long freq)
{
	u32 msg[8] = {0};
	int rproc_id = HISI_RPROC_LPM3_MBX13;
	int ret = 0;

	msg[0] = (OBJ_AP << 24) | (OBJ_L3 << 16) | (CMD_SETTING << 8) | TYPE_FREQ;
	msg[1] = freq;
	msg[1] = msg[1]/1000;

	ret = RPROC_ASYNC_SEND((rproc_id_t)rproc_id, (mbox_msg_t *)msg, 8);
	if(ret) {
		dev_err(dev, "%s: mailbox send error\n", __func__);
		return -1;
	}

	return 0;
}

#ifdef CONFIG_HISI_HW_VOTE_L3C_FREQ
static int l3c_devfreq_set_target_freq_hv(struct device *dev,
					 unsigned long freq)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	int ret = 0;

	ret = hisi_hv_set_freq(l3c->l3c_hvdev, freq);
	if(ret) {
		dev_err(dev, "%s: failed to set freq by hw vote\n", __func__);
		return -1;
	}

	return 0;
}
#endif

static int l3c_devfreq_set_target_freq(struct device *dev, unsigned long freq)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);

#ifdef CONFIG_HISI_HW_VOTE_L3C_FREQ
	if(l3c->hv_supported){
		return l3c_devfreq_set_target_freq_hv(dev, freq);
	}else{
		return l3c_devfreq_set_target_freq_ipc(dev, freq);
	}
#else
	return l3c_devfreq_set_target_freq_ipc(dev, freq);
#endif
}


static int l3c_devfreq_target(struct device *dev,
				    unsigned long *freq, u32 flags)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	struct l3c_devfreq_data *data = l3c->l3c_data;
	int i = 0;
	int ret = 0;

	for(i = 0; i < l3c->freq_table_len; i++){
		if(l3c->freq_table[i] >= *freq){
			break;
		}
	}

	if(i < l3c->freq_table_len){
		*freq = l3c->freq_table[i];
	}else{
		*freq = data->freq_max;
	}

	if (*freq == l3c->cur_freq) {
		return 0;
	}

	trace_l3c_devfreq_target(*freq);

	/* Set requested freq */
	ret = l3c_devfreq_set_target_freq(dev, *freq);

	if(!ret){
		l3c->cur_freq = *freq;
	}

	return ret;
}


/* perf event counter */
#define MAX_COUNT_LIM 0xFFFFFFFFFFFFFFFF
static inline unsigned long read_event(struct event_data *event)
{
	unsigned long ev_count;
	u64 total, enabled, running;

	total = perf_event_read_value(event->pevent, &enabled, &running);
	//trace_l3c_devfreq_read_event(total);

	if (total >= event->prev_count)
		ev_count = total - event->prev_count;
	else
		ev_count = (MAX_COUNT_LIM - event->prev_count) + total;

	event->prev_count = total;

	return ev_count;
}

static void read_perf_counters(struct l3c_hwmon *hw)
{
	struct l3c_hwmon_data *hw_data = &(hw->l3c_hw_data);

	mutex_lock(&(hw_data->active_lock));
	if (false == hw_data->active){
		mutex_unlock(&(hw_data->active_lock));
		return;
	}

	hw->count.l3_count =
			read_event(&hw_data->events[L3D_IDX]);

	hw->count.bus_access_count =
			read_event(&hw_data->events[BUS_ACCESS_IDX]);

	hw->count.cycle_count =
			read_event(&hw_data->events[CYCLE_IDX]);

	mutex_unlock(&(hw_data->active_lock));
}

static void delete_events(struct l3c_hwmon_data *hw_data)
{
	int i;

	for (i = 0; i < NUM_EVENTS; i++) {
		hw_data->events[i].prev_count = 0;
		perf_event_release_kernel(hw_data->events[i].pevent);
	}
}

static void stop_hwmon(struct l3c_hwmon *hw)
{
	struct l3c_hwmon_data *hw_data = &(hw->l3c_hw_data);

	mutex_lock(&(hw_data->active_lock));
	if (false == hw_data->active){
		mutex_unlock(&(hw_data->active_lock));
		return;
	}
	hw_data->active = false;

	delete_events(hw_data);

	hw->count.l3_count = 0;
	hw->count.bus_access_count = 0;
	hw->count.cycle_count = 0;

	mutex_unlock(&(hw_data->active_lock));
}

/*lint -e429 */
static struct perf_event_attr *alloc_attr(void)
{
	struct perf_event_attr *attr;

	attr = kzalloc(sizeof(struct perf_event_attr), GFP_KERNEL);
	if (IS_ERR_OR_NULL(attr))
		return ERR_PTR(-ENOMEM);
/*lint -e613 */
	attr->type = PERF_TYPE_DSU;
	attr->size = sizeof(struct perf_event_attr);
	attr->pinned = 1;
/*lint +e613 */
	return attr;
}
/*lint +e429 */

static int set_events(struct l3c_hwmon_data *hw_data, int cpu)
{
	struct perf_event *pevent= NULL;
	struct perf_event_attr *attr = NULL;
	int err;

	/* Allocate an attribute for event initialization */
	attr = alloc_attr();
	if (IS_ERR(attr)){
		pr_err("%s alloc attr failed\n", __func__);
		return PTR_ERR(attr);
	}

	attr->config = L3D_EV;
	pevent = perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
	if (IS_ERR_OR_NULL(pevent)){
		pr_err("%s perf event create failed, config = 0x%x\n", __func__, (unsigned int)attr->config);
		goto err_out;
	}
	hw_data->events[L3D_IDX].pevent = pevent;
	perf_event_enable(hw_data->events[L3D_IDX].pevent);

	attr->config = BUS_ACCESS_EV;
	pevent = perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
	if (IS_ERR_OR_NULL(pevent)){
		pr_err("%s perf event create failed, config = 0x%x\n", __func__, (unsigned int)attr->config);
		goto err_l3d;
	}
	hw_data->events[BUS_ACCESS_IDX].pevent = pevent;
	perf_event_enable(hw_data->events[BUS_ACCESS_IDX].pevent);

	attr->config = CYCLE_EV;
	pevent = perf_event_create_kernel_counter(attr, cpu, NULL, NULL, NULL);
	if (IS_ERR_OR_NULL(pevent)){
		pr_err("%s perf event create failed, config = 0x%x\n", __func__, (unsigned int)attr->config);
		goto err_buss_acc;
	}
	hw_data->events[CYCLE_IDX].pevent = pevent;
	perf_event_enable(hw_data->events[CYCLE_IDX].pevent);

	kfree(attr);
	return 0;

err_buss_acc:
	perf_event_disable(hw_data->events[BUS_ACCESS_IDX].pevent);
	perf_event_release_kernel(hw_data->events[BUS_ACCESS_IDX].pevent);
err_l3d:
	perf_event_disable(hw_data->events[L3D_IDX].pevent);
	perf_event_release_kernel(hw_data->events[L3D_IDX].pevent);
err_out:
	err = PTR_ERR(pevent);
	kfree(attr);
	return err;
}

static int start_hwmon(struct l3c_hwmon *hw)
{
	int ret = 0;
	struct l3c_hwmon_data *hw_data = &(hw->l3c_hw_data);
	/* cpu must be 0*/
	int cpu = 0;

	mutex_lock(&(hw_data->active_lock));
	if (true == hw_data->active){
		goto exit;
	}

	ret = set_events(hw_data, cpu);
	if (ret) {
		pr_err("Perf event init failed on CPU%d\n", cpu);
		WARN_ON(1);
		goto exit;
	}

	hw_data->active = true;

exit:
	mutex_unlock(&(hw_data->active_lock));
	return ret;
}


static int l3c_devfreq_get_dev_status(struct device *dev,
					    struct devfreq_dev_status *stat)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	unsigned long const usec = ktime_to_us(ktime_get());
	unsigned long delta = 0;
	int ret = 0;

	read_perf_counters(l3c->hw);

	if (0 == l3c->hw->count.cycle_count) {
		stop_hwmon(l3c->hw);
		ret = start_hwmon(l3c->hw);
		if(ret){
			dev_err(dev, "%s start hwmon failed.\n", __func__);
			return -ENODEV;
		}
		read_perf_counters(l3c->hw);
	}

	delta = usec - l3c->alg.last_update;
	l3c->alg.last_update = usec;
	l3c->alg.usec_delta = delta;

	stat->current_frequency = l3c->cur_freq;

	/*
	 * Although this is not yet suitable for use with the simple ondemand
	 * governor we'll fill these usage statistics.
	 */
	stat->total_time = delta;

	return 0;
}

//static unsigned long load_map[] = {80, 100, 125, 1000};

static int l3c_get_freq_from_load(struct l3c_devfreq *l3c)
{
	int i = 0;

	for(i = 0; i < l3c->load_map_len; i++) {
		if (l3c->l3c_bw < l3c->load_map[i])
			break;
	}

	if (i == l3c->load_map_len){
		return l3c->l3c_data->freq_max;
	}

	if(i < l3c->freq_table_len){
		return l3c->freq_table[i];
	}else{
		return l3c->l3c_data->freq_max;
	}
}


static unsigned long l3c_devfreq_calc_next_freq(struct l3c_devfreq *l3c)
{
	unsigned long target_freq = l3c->cur_freq;
	struct l3c_devfreq_data *data = l3c->l3c_data;
	unsigned long tmp_target = data->freq_min;
	unsigned long l3c_bw = 0;
	unsigned long l3c_hit_bw = 0;

	// bw_rate = total_access_count / cycle / 2
	// curr_freq = cycle / time
	// normalized bw_rate = bw_rate X curr_freq / max_freq
	if (l3c->hw->count.cycle_count > 0
	&& l3c->hw->count.l3_count > 0
	&& l3c->hw->count.bus_access_count > 0
	&& l3c->alg.usec_delta > 0
	&& data->freq_max > 0) {
		l3c_bw = ((l3c->hw->count.l3_count + l3c->hw->count.bus_access_count) >> 1) * 1000
			  / l3c->alg.usec_delta * 1000 / data->freq_max;
	}
	else {
		l3c_bw = 0;
	}
	l3c->l3c_bw = l3c_bw;

	if(l3c->alg.usec_delta > 0 && data->freq_max > 0){
		l3c_hit_bw = (l3c->hw->count.l3_count >> 1) * 1000
		  / l3c->alg.usec_delta * 1000 / data->freq_max;
	}

	// check high L3 hit bw
	if (l3c->l3c_bw >= l3c->l3c_bw_max && l3c->hw->count.bus_access_count > 0) {
		mutex_lock(&l3c->allow_lock);
		if (allow_fcm_boost) {
		//if (l3c->hw->count.l3_count / l3c->hw->count.bus_access_count >= l3c->l3_bus_ratio) {
			tmp_target = l3c->boost_freq;
		}
		mutex_unlock(&l3c->allow_lock);
	}

	trace_l3c_devfreq_calc_next_freq(l3c->hw->count.l3_count, l3c->hw->count.bus_access_count, l3c->hw->count.cycle_count,
			l3c->alg.usec_delta, l3c->cur_freq, l3c_bw, l3c_hit_bw);

	target_freq = l3c_get_freq_from_load(l3c);
	target_freq = max(target_freq, tmp_target);

	//bw is invalid by FCM idle, assign min freq to L3
	if(l3c_bw > l3c->load_map_max || l3c_hit_bw > l3c->load_map_max){
		target_freq = data->freq_min;
	}

	return target_freq;
}

static int l3c_devfreq_governor_get_target_freq(struct devfreq *df,
						  unsigned long *freq)
{

	struct l3c_devfreq *l3c = dev_get_drvdata(df->dev.parent);
	struct l3c_devfreq_data *data = l3c->l3c_data;
	int err;
	err = devfreq_update_stats(df);
	if (err)
		return err;

	*freq = l3c_devfreq_calc_next_freq(l3c);

	if (*freq < data->freq_min) {
		*freq = data->freq_min;
	} else if (*freq > data->freq_max) {
		*freq = data->freq_max;
	}

	return 0;
}

static int l3c_devfreq_governor_event_handler(struct devfreq *devfreq,
					    unsigned int event, void *data)
{
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;

	case DEVFREQ_GOV_INTERVAL:
		devfreq_interval_update(devfreq, (unsigned int *)data);
		break;
	}

	return ret;
}

static struct devfreq_governor l3c_devfreq_governor = {
	.name		 = L3C_DEVFREQ_GOVERNOR_NAME,
	.get_target_freq = l3c_devfreq_governor_get_target_freq,
	.event_handler	 = l3c_devfreq_governor_event_handler,
};

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
static void l3c_devfreq_handle_update(struct work_struct *work)
{
	struct l3c_devfreq *l3c = container_of(work, struct l3c_devfreq,
					     update_handle);

	mutex_lock(&l3c->devfreq->lock);
	update_devfreq(l3c->devfreq);
	mutex_unlock(&l3c->devfreq->lock);
}

static enum hrtimer_restart l3c_devfreq_polling(struct hrtimer *hrtimer)
{
	struct l3c_devfreq *l3c = container_of(hrtimer, struct l3c_devfreq,
					     poll_timer);

	queue_work(l3c->update_wq, &l3c->update_handle);

	hrtimer_forward_now(&l3c->poll_timer,
			    ms_to_ktime(l3c->polling_ms));
	return HRTIMER_RESTART;
}
#endif

static int l3c_devfreq_reinit_device(struct device *dev)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	/* Clean the algorithm statistics and start from scrach */
	memset(&l3c->alg, 0, sizeof(l3c->alg));	/* unsafe_function_ignore: memset */
	l3c->alg.last_update = ktime_to_us(ktime_get());

	return 0;
}

static int l3c_devfreq_setup_devfreq_profile(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	struct devfreq_dev_profile *df_profile;

	l3c->devfreq_profile = devm_kzalloc(&pdev->dev,
			sizeof(struct devfreq_dev_profile), GFP_KERNEL);
	if (IS_ERR_OR_NULL(l3c->devfreq_profile)) {
		dev_err(&pdev->dev, "no memory.\n");
		return PTR_ERR(l3c->devfreq_profile);
	}

	df_profile = l3c->devfreq_profile;

	df_profile->target = l3c_devfreq_target;
	df_profile->get_dev_status = l3c_devfreq_get_dev_status;
	df_profile->freq_table = l3c->freq_table;
	df_profile->max_state = l3c->freq_table_len;
	df_profile->polling_ms = l3c->polling_ms;
	df_profile->initial_freq = l3c->initial_freq;

	return 0;
}


static int l3c_devfreq_parse_algo_dt(struct platform_device *pdev, struct device_node *np)
{
	const struct property *prop = NULL;
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	u32 tmp_boost_freq = 0;
	u32 tmp_lit_link_freq = 0;
	u32 tmp_big_link_freq = 0;
	int i = 0;
	int ret = 0;

	prop = of_find_property(np, "load-map", NULL);
	if (prop) {
		l3c->load_map_len = ((unsigned int)prop->length) / sizeof(*l3c->load_map);
		dev_dbg(&pdev->dev, "load_map_len = %d\n", l3c->load_map_len);

		l3c->load_map = devm_kcalloc(&pdev->dev, l3c->load_map_len,
						   sizeof(*l3c->load_map),
						   GFP_KERNEL);
		if (IS_ERR_OR_NULL(l3c->load_map)){
			dev_err(&pdev->dev, "load_map alloc fail\n");
			ret = -ENOMEM;
			goto err;
		}

		if (of_property_read_u64_array(np, "load-map", l3c->load_map, l3c->load_map_len) < 0){
			dev_err(&pdev->dev, "read load_map fail\n");
			ret = -ENODEV;
			goto err;
		}
	}else{
		dev_err(&pdev->dev, "no load-map\n");
		ret = -ENODEV;
		goto err;
	}
	l3c->load_map_max = l3c->load_map[l3c->load_map_len - 1];
	dev_dbg(&pdev->dev, "load_map_max = %lu\n", (unsigned long)l3c->load_map_max);

	for(i = 0; i < l3c->load_map_len; i++){
		dev_dbg(&pdev->dev, "load_map[%d] = %lu\n", i, (unsigned long)l3c->load_map[i]);
	}

	ret = of_property_read_u64(np, "l3c-bw-max", &l3c->l3c_bw_max);
	if(ret){
		dev_err(&pdev->dev, "no l3c-bw-max\n");
		ret = -ENODEV;
		goto err;
	}
	dev_dbg(&pdev->dev, "l3c_bw_max = %lu\n", (unsigned long)l3c->l3c_bw_max);

	ret = of_property_read_u64(np, "l3-bus-ratio", &l3c->l3_bus_ratio);
	if(ret){
		dev_err(&pdev->dev, "no l3-bus-ratio\n");
		ret = -ENODEV;
		goto err;
	}
	dev_dbg(&pdev->dev, "l3_bus_ratio = %lu\n", (unsigned long)l3c->l3_bus_ratio);

	ret = of_property_read_u32(np, "boost-freq", &tmp_boost_freq);
	if(ret){
		dev_err(&pdev->dev, "no boost-freq\n");
		ret = -ENODEV;
		goto err;
	}
	l3c->boost_freq = tmp_boost_freq;
	dev_dbg(&pdev->dev, "boost_freq = %lu\n", l3c->boost_freq);

	ret = of_property_read_u32(np, "little-link-freq", &tmp_lit_link_freq);
	if(ret){
		dev_err(&pdev->dev, "no little-link-freq\n");
		ret = -ENODEV;
		goto err;
	}
	l3c->lit_link_freq = tmp_lit_link_freq;
	dev_dbg(&pdev->dev, "lit_link_freq = %lu\n", l3c->lit_link_freq);

	ret = of_property_read_u32(np, "big-link-freq", &tmp_big_link_freq);
	if(ret){
		dev_err(&pdev->dev, "no big-link-freq\n");
		ret = -ENODEV;
		goto err;
	}
	l3c->big_link_freq = tmp_big_link_freq;
	dev_dbg(&pdev->dev, "big_link_freq = %lu\n", l3c->big_link_freq);

err:
	return ret;
}


static int l3c_devfreq_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	const struct property *prop = NULL;
	const __be32 *val = NULL;
	unsigned int volt = 0, freq = 0;
	u32 tmp_init_freq = 0;
	int nr = 0;
	int k = 0;
	int i = 0;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, "hisi,l3c_devfreq");
	if (!np) {
		dev_err(&pdev->dev, "hisi,l3c_devfreq No compatible node found\n");
		return -ENODEV;
	}

	prop = of_find_property(np, "operating-points", NULL);
	if (!prop){
		dev_err(&pdev->dev, "no operating-points\n");
		ret = -ENODEV;
		goto err;
	}

	if (!prop->value){
		dev_err(&pdev->dev, "prop->value is valid\n");
		ret = -ENODATA;
		goto err;
	}

	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = ((unsigned int)prop->length) / sizeof(u32);
	if (nr % 2) {
		dev_err(&pdev->dev, "Invalid OPP list\n");
		ret= -EINVAL;
		goto err;
	}

	l3c->freq_table_len = nr / 2;
	val = prop->value;

	l3c->freq_table = devm_kcalloc(&pdev->dev, l3c->freq_table_len,
				       sizeof(*(l3c->freq_table)),
				       GFP_KERNEL);
	if (IS_ERR_OR_NULL(l3c->freq_table)){
		dev_err(&pdev->dev, "freq_table alloc fail\n");
		ret = -ENOMEM;
		goto err;
	}

	l3c->volt_table = devm_kcalloc(&pdev->dev, l3c->freq_table_len,
				       sizeof(*(l3c->volt_table)),
				       GFP_KERNEL);
	if (IS_ERR_OR_NULL(l3c->volt_table)){
		dev_err(&pdev->dev, "volt_table alloc fail\n");
		ret = -ENOMEM;
		goto err;
	}

	while (nr) {
		freq = be32_to_cpup(val++);
		volt = be32_to_cpup(val++);
		l3c->freq_table[k] = freq;
		l3c->volt_table[k] = volt;
		dev_dbg(&pdev->dev, "the OPP k %d,freq %d\n", k, freq);
		nr -= 2;
		k++;
	}

	device_data.freq_min = l3c->freq_table[0];
	device_data.freq_max = l3c->freq_table[l3c->freq_table_len - 1];
	l3c->l3c_data = &device_data;

	for(i = 0; i < l3c->freq_table_len; i++){
		dev_dbg(&pdev->dev, "freq_table[%d] = %d volt_table[%d] = %d\n", i, l3c->freq_table[i], i, l3c->volt_table[i]);
	}

	ret = of_property_read_u32(np, "init-freq", &tmp_init_freq);
	if(ret){
		dev_err(&pdev->dev, "no init-freq\n");
		ret = -ENODEV;
		goto err;
	}
	l3c->initial_freq = tmp_init_freq;
	l3c->cur_freq = l3c->initial_freq;
	dev_dbg(&pdev->dev, "initial_freq = %lu\n", l3c->initial_freq);

	ret = of_property_read_u32(np, "hv-supported", &l3c->hv_supported);
	if(ret){
		dev_err(&pdev->dev, "no hv-supported\n");
		ret = -ENODEV;
		goto err;
	}
	dev_err(&pdev->dev, "hv_supported = %u\n", l3c->hv_supported);

	ret = of_property_read_u32(np, "polling", &l3c->polling_ms);
	if (ret){
		l3c->polling_ms = L3C_DEVFREQ_DEFAULT_POLLING_MS;
	}
	dev_dbg(&pdev->dev, "polling_ms = %d\n", l3c->polling_ms);

	ret = l3c_devfreq_parse_algo_dt(pdev, np);
	if(ret){
		dev_err(&pdev->dev, "parse algo dt fail\n");
		ret = -ENODEV;
		goto err;
	}

err:
	of_node_put(np);
	return ret;
}


static void l3c_devfreq_remove_opps(struct platform_device *pdev)
{
	struct dev_pm_opp *opp = NULL;
	int i, count;
	unsigned long freq;

	count = dev_pm_opp_get_opp_count(&pdev->dev);
	if (count <= 0)
		return;

	rcu_read_lock();
	for (i = 0, freq = 0; i < count; i++, freq++) {
		opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &freq);
		if (!IS_ERR_OR_NULL(opp))
			dev_pm_opp_remove(&pdev->dev, freq);
	}
	rcu_read_unlock();
}

static int l3c_devfreq_enable_opps(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	int i = 0, ret = 0;
	int opp_count = 0;

	if (l3c->freq_table_len <= 0)
		return -EINVAL;

	for (i = 0; i < l3c->freq_table_len; i++) {
		ret = dev_pm_opp_add(&pdev->dev, l3c->freq_table[i],
				     l3c->volt_table[i]);
		if (ret)
			dev_warn(&pdev->dev, "cannot add a new OPP.\n");
		else
			opp_count++;
	}

	if (opp_count == 0) {
		dev_err(&pdev->dev, "device has no OPP registered.\n");
		return -ENODEV;
	}

	return 0;
}

static int l3c_devfreq_setup(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	struct l3c_devfreq_data *data = l3c->l3c_data;
	int ret = 0;

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	l3c->update_wq = create_workqueue("hisi_l3c_devfreq_wq");
	if (IS_ERR_OR_NULL(l3c->update_wq)) {
		dev_err(&pdev->dev, "cannot create workqueue.\n");
		return PTR_ERR(l3c->update_wq);
	}
	INIT_WORK(&l3c->update_handle, l3c_devfreq_handle_update);
#endif

	mutex_init(&l3c->lock);

	mutex_init(&l3c->allow_lock);

	ret = l3c_devfreq_enable_opps(pdev);
	if (ret) {
		dev_err(&pdev->dev, "device setup failed.\n");
		return ret;
	}

	ret = l3c_devfreq_setup_devfreq_profile(pdev);
	if (ret) {
		dev_err(&pdev->dev, "device setup failed.\n");
		return ret;
	}

	l3c->alg.last_update = ktime_to_us(ktime_get());

	l3c->devfreq = devm_devfreq_add_device(&pdev->dev,
					       l3c->devfreq_profile,
					       L3C_DEVFREQ_GOVERNOR_NAME, NULL);

	if (IS_ERR_OR_NULL(l3c->devfreq)) {
		dev_err(&pdev->dev, "registering to devfreq failed.\n");
		return PTR_ERR(l3c->devfreq);
	}

	mutex_lock(&l3c->devfreq->lock);
	l3c->devfreq->min_freq = data->freq_min;
	l3c->devfreq->max_freq = data->freq_max;
	mutex_unlock(&l3c->devfreq->lock);

	return 0;
}


static int l3c_devfreq_init_device(struct platform_device *pdev)
{
#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
#endif

	l3c_devfreq_reinit_device(&pdev->dev);

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	hrtimer_init(&l3c->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	l3c->poll_timer.function = l3c_devfreq_polling;
	hrtimer_start(&l3c->poll_timer, ms_to_ktime(l3c->polling_ms),
		      HRTIMER_MODE_REL);
#endif

	return 0;
}

/*lint -e429 */
static int l3c_devfreq_hwmon_setup(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	struct l3c_hwmon *hw;
	int ret = 0;

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (IS_ERR_OR_NULL(hw)){
		return -ENOMEM;
	}

	mutex_init(&(hw->l3c_hw_data.active_lock)); //lint !e613

	ret = start_hwmon(hw);
	if(ret){
		dev_err(&pdev->dev, "start hwmon failed.\n");
		return -ENODEV;
	}

	l3c->hw = hw;

	return ret;
}
/*lint +e429 */

static void l3c_devfreq_hwmon_destory(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);

	stop_hwmon(l3c->hw);
}

static int l3c_devfreq_suspend(struct device *dev)
{
	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	//struct l3c_devfreq_data *data = l3c->l3c_data;
	int ret = 0;

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	if (hrtimer_active(&l3c->poll_timer))
	    hrtimer_cancel(&l3c->poll_timer);
#endif

	ret = devfreq_suspend_device(l3c->devfreq);
	if (ret < 0) {
		dev_err(dev, "failed to suspend devfreq device.\n");
		return ret;
	}

	stop_hwmon(l3c->hw);

	return ret;
}

static int l3c_devfreq_resume(struct device *dev)
{

	struct l3c_devfreq *l3c = dev_get_drvdata(dev);
	int ret = 0;

	l3c_devfreq_reinit_device(dev);

	ret = start_hwmon(l3c->hw);
	if(ret){
		dev_err(dev, "start hwmon failed in resume.\n");
		return -ENODEV;
	}

	ret = devfreq_resume_device(l3c->devfreq);
	if (ret < 0)
		dev_err(dev, "failed to resume devfreq device.\n");

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	if (!hrtimer_active(&l3c->poll_timer))
	    hrtimer_start(&l3c->poll_timer, ms_to_ktime(l3c->polling_ms),
			  HRTIMER_MODE_REL);
#endif

	return ret;
}

static SIMPLE_DEV_PM_OPS(l3c_devfreq_pm, l3c_devfreq_suspend, l3c_devfreq_resume);


#ifdef CONFIG_HISI_L3C_DEVFREQ_DEBUG
#define LOCAL_BUF_MAX       (128)
static ssize_t show_l3c_bw_max(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)l3c->l3c_bw_max);	/* unsafe_function_ignore: snprintf */
}

static ssize_t store_l3c_bw_max(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	unsigned long value = 0;
	int ret = 0;

	ret = sscanf(buf, "%lu", &value);	/* unsafe_function_ignore: sscanf */
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&l3c->devfreq->lock);
	if (value > 0) {
		l3c->l3c_bw_max = value;
		ret = count;
	}else{
		ret = -EINVAL;
	}
	mutex_unlock(&l3c->devfreq->lock);
	return ret;
}


static ssize_t show_boost_freq(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)l3c->boost_freq);	/* unsafe_function_ignore: snprintf */
}

static ssize_t store_boost_freq(struct device *dev,
                        struct device_attribute *attr, const char *buf,
                        size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	unsigned long value = 0;
	int ret = 0;

	ret = sscanf(buf, "%lu", &value);	/* unsafe_function_ignore: sscanf */
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&l3c->devfreq->lock);
	if (value > 0) {
		l3c->boost_freq = value;
		ret = count;
	}else{
		ret = -EINVAL;
	}
	mutex_unlock(&l3c->devfreq->lock);
	return ret;
}

static ssize_t show_lit_link_freq(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)l3c->lit_link_freq);	/* unsafe_function_ignore: snprintf */
}

static ssize_t store_lit_link_freq(struct device *dev,
                        struct device_attribute *attr, const char *buf,
                        size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	unsigned long value = 0;
	int ret = 0;

	ret = sscanf(buf, "%lu", &value);	/* unsafe_function_ignore: sscanf */
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&l3c->devfreq->lock);
	if (value > 0) {
		l3c->lit_link_freq = value;
		ret = count;
	}else{
		ret = -EINVAL;
	}
	mutex_unlock(&l3c->devfreq->lock);
	return ret;
}

static ssize_t show_big_link_freq(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);

	return snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long)l3c->big_link_freq);	/* unsafe_function_ignore: snprintf */
}

static ssize_t store_big_link_freq(struct device *dev,
                        struct device_attribute *attr, const char *buf,
                        size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	unsigned long value = 0;
	int ret = 0;

	ret = sscanf(buf, "%lu", &value);	/* unsafe_function_ignore: sscanf */
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&l3c->devfreq->lock);
	if (value > 0) {
		l3c->big_link_freq = value;
		ret = count;
	}else{
		ret = -EINVAL;
	}
	mutex_unlock(&l3c->devfreq->lock);
	return ret;
}

static ssize_t show_load_map(struct device *dev, struct device_attribute *attr,
                        char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	int count = 0;
	int ret = 0;
	int i = 0;

	for(i = 0; i < l3c->load_map_len; i++){
		ret = snprintf(buf + count, (size_t)(PAGE_SIZE - count), "load_map[%d] = %lu\n",	/* unsafe_function_ignore: snprintf */
				i, (unsigned long)l3c->load_map[i]);

		if (ret >= ((int)PAGE_SIZE - count) || ret < 0) {
			break;
		}
		count += ret;
	}

    return count;
}

static int cmd_parse(char *para_cmd, char *argv[], int max_args)
{
    int para_id = 0;

	while (*para_cmd != '\0')
	{
		if (para_id >= max_args)
			break;
		while (*para_cmd == ' ')
			para_cmd++;

		if ('\0' == *para_cmd)
			break;

		argv[para_id] = para_cmd;
		para_id++;

		while ((*para_cmd != ' ') && (*para_cmd != '\0'))
			para_cmd++;

		if ('\0' == *para_cmd)
			break;

		*para_cmd = '\0';
		para_cmd++;

	}

	return para_id;
}

static ssize_t store_load_map(struct device *dev,
                        struct device_attribute *attr, const char *buf,
                        size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct l3c_devfreq *l3c = dev_get_drvdata(devfreq->dev.parent);
	char local_buf[LOCAL_BUF_MAX] = {0};
	char *argv[2] = {0};
	unsigned int idx = 0, value = 0;
	int argc = 0, ret = 0;

	if (count >= sizeof(local_buf)) {
		return -ENOMEM;
	}

	strncpy(local_buf, buf, min_t(size_t, sizeof(local_buf) - 1, count)); /* unsafe_function_ignore: strncpy */

	argc = cmd_parse(local_buf, argv, sizeof(argv) / sizeof(argv[0]));
	if (2 != argc) {
		dev_err(dev, "error, only surport two para\n");
		ret = -EINVAL;
		goto err_handle;
	}

	ret = sscanf(argv[0], "%u", &idx); /* unsafe_function_ignore: sscanf */
	if (ret != 1) {
		dev_err(dev, "parse idx error %s\n", argv[0]);
		ret = -EINVAL;
		goto err_handle;
	}

	ret = sscanf(argv[1], "%u", &value);/* unsafe_function_ignore: sscanf */
	if (ret != 1) {
		dev_err(dev, "parse value error %s\n", argv[1]);
		ret = -EINVAL;
		goto err_handle;
	}

	mutex_lock(&l3c->devfreq->lock);
	if (idx < (unsigned int)l3c->load_map_len) {
		l3c->load_map[idx] = value;
		ret = count;
	}else{
		dev_err(dev, "invalid idx %d\n",idx);
		ret = -EINVAL;
	}
	mutex_unlock(&l3c->devfreq->lock);

err_handle:
    return ret;
}

#define L3C_DEVFREQ_ATTR_RW(_name) \
	static DEVICE_ATTR(_name, 0640, show_##_name, store_##_name)

L3C_DEVFREQ_ATTR_RW(l3c_bw_max);
/* L3C_DEVFREQ_ATTR_RW(l3_bus_ratio); */
L3C_DEVFREQ_ATTR_RW(boost_freq);
L3C_DEVFREQ_ATTR_RW(lit_link_freq);
L3C_DEVFREQ_ATTR_RW(big_link_freq);
L3C_DEVFREQ_ATTR_RW(load_map);


static struct attribute *dev_entries[] = {
	&dev_attr_l3c_bw_max.attr,
	/* &dev_attr_l3_bus_ratio.attr, */
	&dev_attr_boost_freq.attr,
	&dev_attr_lit_link_freq.attr,
	&dev_attr_big_link_freq.attr,
	&dev_attr_load_map.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.name   = "l3c_dvfs",
	.attrs  = dev_entries,
};
#endif


extern int hisi_test_fast_cpu(int cpu);
/*
 * when CPUFREQ core post cpufreq change notify, we will check whether
 * big/little core's frequency exceed or equal link freq.
 * If so, set bit/little's bit in allow flag, which is used to boost
 * L3's frequency in next target freq evaluation.

*/
static int l3c_cpufreq_transition(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs;
	struct l3c_devfreq *l3c = container_of(nb, struct l3c_devfreq,
					     l3c_trans_notify);

	freqs = (struct cpufreq_freqs *) data;

	if (val == CPUFREQ_POSTCHANGE) {
		mutex_lock(&l3c->allow_lock);
		if(hisi_test_fast_cpu(freqs->cpu)){
			if (freqs->new >= l3c->big_link_freq) {
				allow_fcm_boost |= BIT(BIG_CLUSTER);
			}
			else {
				allow_fcm_boost &= ~BIT(BIG_CLUSTER);
			}
		}else{  // little core
			if (freqs->new >= l3c->lit_link_freq) {
				allow_fcm_boost |= BIT(LITTLE_CLUSTER);
			}
			else {
				allow_fcm_boost &= ~BIT(LITTLE_CLUSTER);
			}
		}

		trace_l3c_cpufreq_transition(freqs->cpu, freqs->old, freqs->new, allow_fcm_boost);
		mutex_unlock(&l3c->allow_lock);
	}

	return 0;
}



/*lint -e429*/
static int l3c_devfreq_probe(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = NULL;
	int ret = 0;

	dev_err(&pdev->dev, "registering l3c devfreq.\n");

	l3c = devm_kzalloc(&pdev->dev, sizeof(*l3c), GFP_KERNEL);
	if (IS_ERR_OR_NULL(l3c))
		return -ENOMEM;

	platform_set_drvdata(pdev, l3c);
	l3c->pdev = pdev;	//lint !e613

	ret = l3c_devfreq_parse_dt(pdev);
	if (ret)
		goto failed;

	ret = l3c_devfreq_hwmon_setup(pdev);
	if (ret){
		dev_err(&pdev->dev, "%s: hwmon setup failed %d\n", __func__, ret);
		goto failed;
	}

	ret = l3c_devfreq_setup(pdev);
	if (ret)
		goto err_hwmon;

	ret = l3c_devfreq_init_device(pdev);
	if (ret)
		goto err_hwmon;
/*lint -e613 */

#ifdef CONFIG_HISI_HW_VOTE_L3C_FREQ
	if(l3c->hv_supported){
		l3c->l3c_hvdev = hisi_hvdev_register(&pdev->dev, "l3-freq", "vote-src-1");
		if (IS_ERR_OR_NULL(l3c->l3c_hvdev)) {
			dev_err(&pdev->dev, "%s: register hvdev fail!\n", __func__);
			ret = -ENODEV;
			goto err_hwmon;
		}
	}else{
		l3c->l3c_hvdev = NULL;
	}
#endif

	l3c->l3c_trans_notify.notifier_call = l3c_cpufreq_transition;
	ret = cpufreq_register_notifier(&l3c->l3c_trans_notify, CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		goto err_hwmon;

#ifdef CONFIG_HISI_L3C_DEVFREQ_DEBUG
	ret = sysfs_create_group(&l3c->devfreq->dev.kobj, &dev_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs create err %d\n", ret);
		goto err_unreg_trans;
	}
#endif

	dev_err(&pdev->dev, "probe success\n");
	return 0;

err_unreg_trans:
	cpufreq_unregister_notifier(&l3c->l3c_trans_notify, CPUFREQ_TRANSITION_NOTIFIER);
err_hwmon:
	l3c_devfreq_hwmon_destory(pdev);
failed:
	dev_err(&pdev->dev, "failed to register driver, err %d.\n", ret);
	return ret;
/*lint +e613 */
}
/*lint +e429*/


static int l3c_devfreq_remove(struct platform_device *pdev)
{
	struct l3c_devfreq *l3c = platform_get_drvdata(pdev);
	int ret = 0;

	dev_err(&pdev->dev, "unregistering l3c devfreq device.\n");

	ret = l3c_devfreq_set_target_freq(&pdev->dev, l3c->initial_freq);

#ifdef L3C_DEVFREQ_HRTIMER_ENABLE
	/* Cancel hrtimer */
	if (hrtimer_active(&l3c->poll_timer))
		hrtimer_cancel(&l3c->poll_timer);
	/* Wait for pending work */
	flush_workqueue(l3c->update_wq);
	/* Destroy workqueue */
	destroy_workqueue(l3c->update_wq);
#endif

#ifdef CONFIG_HISI_L3C_DEVFREQ_DEBUG
	sysfs_remove_group(&l3c->devfreq->dev.kobj, &dev_attr_group);
#endif

	cpufreq_unregister_notifier(&l3c->l3c_trans_notify, CPUFREQ_TRANSITION_NOTIFIER);

	devm_devfreq_remove_device(&pdev->dev, l3c->devfreq);
	l3c_devfreq_remove_opps(pdev);
	l3c_devfreq_hwmon_destory(pdev);

	return ret;
}

MODULE_DEVICE_TABLE(of, l3c_devfreq_id);

static struct platform_driver l3c_devfreq_driver = {
	.probe	= l3c_devfreq_probe,
	.remove = l3c_devfreq_remove,
	.driver = {
		.name = L3C_DEVFREQ_PLATFORM_DEVICE_NAME,
		.of_match_table = l3c_devfreq_id,
		.pm = &l3c_devfreq_pm,
		.owner = THIS_MODULE,
	},
};

static int __init l3c_devfreq_init(void)
{
	int ret = 0;

	ret = devfreq_add_governor(&l3c_devfreq_governor);
	if (ret) {
		pr_err("%s: failed to add governor: %d.\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&l3c_devfreq_driver);
	if (ret)
		devfreq_remove_governor(&l3c_devfreq_governor);

	return ret;
}

static void __exit l3c_devfreq_exit(void)
{
	int ret = 0;

	ret = devfreq_remove_governor(&l3c_devfreq_governor);
	if (ret)
		pr_err("%s: failed to remove governor: %d.\n", __func__, ret);

	platform_driver_unregister(&l3c_devfreq_driver);
}

late_initcall(l3c_devfreq_init)


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("L3cache devfreq driver");
MODULE_AUTHOR("HUAWEI Ltd.");
MODULE_VERSION("1.0");
