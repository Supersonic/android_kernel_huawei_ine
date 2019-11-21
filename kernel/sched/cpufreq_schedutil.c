/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <trace/events/power.h>

#include "sched.h"
#include "tune.h"
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
#include "walt.h"

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_schedutil.h>
#endif

unsigned long boosted_cpu_util(int cpu);

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
enum freq_reporting_policy {
	FREQ_REPORT_MAX_CPU_LOAD_TOP_TASK,
	FREQ_REPORT_CPU_LOAD,
	FREQ_REPORT_TOP_TASK,
	FREQ_REPORT_INVALID_POLICY
};

/*
 * This governs what load needs to be used when reporting CPU busy time
 * to the cpufreq governor.
 */
__read_mostly unsigned int sysctl_sched_freq_reporting_policy =
				FREQ_REPORT_MAX_CPU_LOAD_TOP_TASK;

__read_mostly unsigned int sched_io_is_busy = 0;

unsigned long
freq_boosted(unsigned long util, int cpu);

void sched_set_io_is_busy(int val)
{
        sched_io_is_busy = val;
}
#endif

/* Stub out fast switch routines present on mainline to reduce the backport
 * overhead. */
#define cpufreq_driver_fast_switch(x, y) 0
#define cpufreq_enable_fast_switch(x)
#define cpufreq_disable_fast_switch(x)
#define LATENCY_MULTIPLIER			(1000)
#define SUGOV_KTHREAD_PRIORITY	50

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 90
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
#define DEFAULT_RATE_LIMIT_US (79 * USEC_PER_MSEC)
static unsigned int default_above_hispeed_delay[] = {
		DEFAULT_RATE_LIMIT_US };
static unsigned int default_min_sample_time[] = {
		DEFAULT_RATE_LIMIT_US };
#endif

struct sugov_tunables {
	struct gov_attr_set attr_set;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	/* Hi speed to bump to from lo speed when load burst (default max) */
	unsigned int hispeed_freq;

	/* Go to hi speed when CPU load at or above this value. */
#define DEFAULT_GO_HISPEED_LOAD 99
	unsigned long go_hispeed_load;

	/* Target load. Lower values result in higher CPU speeds. */
	spinlock_t target_loads_lock;
	unsigned int *target_loads;
	int ntarget_loads;

	/*
	 * Wait this long before raising speed above hispeed, by default a
	 * single timer interval.
	*/
	spinlock_t above_hispeed_delay_lock;
	unsigned int *above_hispeed_delay;
	int nabove_hispeed_delay;

	/*
	 * The minimum amount of time to spend at a frequency before we can ramp
	 * down.
	 */
	spinlock_t min_sample_time_lock;
	unsigned int *min_sample_time;
	int nmin_sample_time;

	/* Non-zero means indefinite speed boost active */
	int boost;
	/* Duration of a boot pulse in usecs */
#define DEFAULT_BOOSTPULSE_DURATION	(80 * USEC_PER_MSEC)
	int boostpulse_duration;
	/* End time of boost pulse in ktime converted to usecs */
	u64 boostpulse_endtime;
	bool boosted;
	/* Minimun boostpulse interval */
#define DEFAULT_MIN_BOOSTPULSE_INTERVAL (500 * USEC_PER_MSEC)
	int boostpulse_min_interval;

	/*
	 * Max additional time to wait in idle, beyond timer_rate, at speeds
	 * above minimum before wakeup to reduce speed, or -1 if unnecessary.
	 */
#define DEFAULT_TIMER_SLACK (80 * USEC_PER_MSEC)
	int timer_slack_val;

	bool io_is_busy;
	bool fast_ramp_up;
	bool fast_ramp_down;
#endif

#ifdef CONFIG_SCHED_HISI_TOP_TASK
	unsigned int top_task_hist_size;
	unsigned int top_task_stats_policy;
	bool top_task_stats_empty_window;
#endif

	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
};

struct sugov_policy {
	struct cpufreq_policy *policy;

	struct sugov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;  /* For shared policies */
	u64 last_freq_update_time;
	s64 min_rate_limit_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	u64 time;
	/* protect slack timer */
	raw_spinlock_t timer_lock;
	/* policy slack timer */
	struct timer_list pol_slack_timer;
	unsigned int min_freq;    /* in kHz */
	/* remember last active cpu and set slack timer on it */
	unsigned int trigger_cpu;
	/* most busy cpu */
	unsigned int max_cpu;
	unsigned int iowait_boost;
	bool skip_min_sample_time;
	bool skip_hispeed_logic;
	bool util_changed;
	bool governor_enabled;
#ifdef CONFIG_HISI_CPU_FREQ_LOCK_DETECT
#define LONGER_MIN_SAMPLE_TIME_ELAPSED_DURATION (4 * NSEC_PER_SEC)
	ktime_t start_time;
	ktime_t end_time;
	bool locked;
#endif
#endif
	bool work_in_progress;

	bool need_freq_update;
};

struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	unsigned int iowait_boost_max;
	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;

	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	unsigned int cpu;
	bool enabled;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

static inline bool use_pelt(void)
{
	return use_pelt_freq();
}

/************************ Governor internals ***********************/
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static void update_min_rate_limit_us(struct sugov_policy *sg_policy);

static unsigned int freq_to_targetload(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads - 1 &&
	     freq >= tunables->target_loads[i + 1]; i += 2)
		;

	ret = tunables->target_loads[i];
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static unsigned int freq_to_above_hispeed_delay(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->nabove_hispeed_delay - 1 &&
	     freq >= tunables->above_hispeed_delay[i + 1]; i += 2)
		;

	ret = tunables->above_hispeed_delay[i];
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);
	return ret;
}

static unsigned int freq_to_min_sample_time(struct sugov_tunables *tunables,
				       unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;

	spin_lock_irqsave(&tunables->min_sample_time_lock, flags);

	for (i = 0; i < tunables->nmin_sample_time - 1 &&
	     freq >= tunables->min_sample_time[i + 1]; i += 2)
		;

	ret = tunables->min_sample_time[i];
	spin_unlock_irqrestore(&tunables->min_sample_time_lock, flags);
	return ret;
}

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */
static unsigned int choose_freq(struct sugov_policy *sg_policy,
				unsigned int loadadjfreq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	unsigned int prevfreq, freqmin = 0, freqmax = UINT_MAX, tl;
	unsigned int freq = policy->cur;
	int index;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(sg_policy->tunables, freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		if (cpufreq_frequency_table_target(
			    policy, freq_table, loadadjfreq / tl,
			    CPUFREQ_RELATION_L, &index))
			break;
		freq = freq_table[index].frequency;

		if (freq > prevfreq) {
			/* The previous frequency is too low */
			freqmin = prevfreq;

			if (freq < freqmax)
				continue;

			/* Find highest frequency that is less than freqmax */
			if (cpufreq_frequency_table_target(
				    policy, freq_table,
				    freqmax - 1, CPUFREQ_RELATION_H,
				    &index))
				break;
			freq = freq_table[index].frequency;

			if (freq == freqmin) {
				/*
				 * The first frequency below freqmax has already
				 * been found to be too low. freqmax is the
				 * lowest speed we found that is fast enough.
				 */
				freq = freqmax;
				break;
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq > freqmin)
				continue;

			/* Find lowest frequency that is higher than freqmin */
			if (cpufreq_frequency_table_target(
				    policy, freq_table,
				    freqmin + 1, CPUFREQ_RELATION_L,
				    &index))
				break;
			freq = freq_table[index].frequency;

			/*
			 * If freqmax is the first frequency above
			 * freqmin then we have already found that
			 * this speed is fast enough.
			 */
			if (freq == freqmax)
				break;
		}

		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

	return freq;
}

/* Re-evaluate load to see if a frequency change is required or not */
static unsigned int eval_target_freq(struct sugov_policy *sg_policy,
			unsigned long util, unsigned long max, unsigned int max_freq)
{
	u64 now;
	int cpu_load = 0;
	unsigned int new_freq;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct cpufreq_policy *policy = sg_policy->policy;

	now = ktime_to_us(ktime_get());
	tunables->boosted = tunables->boost ||
			    now < tunables->boostpulse_endtime;

	if (tunables->boosted && policy->cur < tunables->hispeed_freq) {
		new_freq = tunables->hispeed_freq;
	} else {
		cpu_load = util * 100 / capacity_curr_of(policy->cpu);
		new_freq = choose_freq(sg_policy, cpu_load * policy->cur);

		if ((cpu_load >= tunables->go_hispeed_load &&
		     new_freq < tunables->hispeed_freq) ||
		    (new_freq > tunables->hispeed_freq &&
		     policy->cur < tunables->hispeed_freq))
			new_freq = tunables->hispeed_freq;
	}

	new_freq = max(sg_policy->iowait_boost, new_freq);

	trace_cpufreq_schedutil_eval_target(sg_policy->max_cpu,
					    util, max, cpu_load,
					    policy->cur, new_freq);

	return new_freq;
}

static void sugov_slack_timer_resched(struct sugov_policy *sg_policy)
{
	u64 expires;

	raw_spin_lock(&sg_policy->timer_lock);
	if (!sg_policy->governor_enabled)
		goto unlock;

	del_timer(&sg_policy->pol_slack_timer);
	if (sg_policy->tunables->timer_slack_val >= 0 &&
	    sg_policy->next_freq > sg_policy->policy->min) {
		expires = jiffies + usecs_to_jiffies(sg_policy->tunables->timer_slack_val);
		sg_policy->pol_slack_timer.expires = expires;
		add_timer_on(&sg_policy->pol_slack_timer, sg_policy->trigger_cpu);
	}
unlock:
	raw_spin_unlock(&sg_policy->timer_lock);
}
#endif

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	if (sg_policy->work_in_progress)
		return false;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	if (!use_pelt())
		return true;

	if (sg_policy->skip_hispeed_logic || sg_policy->skip_min_sample_time)
		return true;

	delta_ns = time - sg_policy->time;
#else
	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}

	delta_ns = time - sg_policy->last_freq_update_time;
#endif

	/* No need to recalculate next freq for min_rate_limit_us at least */
	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - sg_policy->last_freq_update_time;

	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static void sugov_update_commit(struct sugov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	sg_policy->time = time;
#else
	if (sugov_up_down_rate_limit(sg_policy, time, next_freq)) {
		/* Reset cached freq as next_freq isn't changed */
		sg_policy->cached_raw_freq = 0;
		return;
	}

	if (sg_policy->next_freq == next_freq)
		return;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;
#endif

	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (next_freq == CPUFREQ_ENTRY_INVALID)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	freq = eval_target_freq(sg_policy, util, max, freq);
	return freq;
#else
	freq = (freq + (freq >> 2)) * util / max;

	if (freq == sg_policy->cached_raw_freq && sg_policy->next_freq != UINT_MAX)
		return sg_policy->next_freq;
	sg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
#endif
}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static inline unsigned long freq_policy_util(int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned int reporting_policy = sysctl_sched_freq_reporting_policy;

	switch (reporting_policy) {
	case FREQ_REPORT_MAX_CPU_LOAD_TOP_TASK:
		return max(cpu_util_freq(cpu), top_task_util(rq));
	case FREQ_REPORT_TOP_TASK:
		return top_task_util(rq);
	case FREQ_REPORT_CPU_LOAD:
		break;
	default:
		break;
	}

	return cpu_util_freq(cpu);
}

unsigned long boosted_freq_policy_util(int cpu)
{
	unsigned long util;

	util = freq_policy_util(cpu);

	return freq_boosted(util, cpu);
}

static void sugov_update_util(int cpu, u64 time,
		unsigned int early_detection)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap, rt, util;
	s64 delta;
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

	max_cap = arch_scale_cpu_capacity(NULL, cpu);

	if (early_detection) {
		util = max_cap;
	} else {
		util = boosted_freq_policy_util(cpu);

		if (unlikely(use_pelt())) {
			sched_avg_update(rq);
			delta = time - rq->age_stamp;
			if (unlikely(delta < 0))
				delta = 0;
			rt = div64_u64(rq->rt_avg, sched_avg_period() + delta);
			rt = (rt * max_cap) >> SCHED_CAPACITY_SHIFT;
			util = util + rt;
			util = min(util, max_cap);
		}
	}

	sg_cpu->util = util;
	sg_cpu->max = max_cap;
}
#endif

static void sugov_get_util(unsigned long *util, unsigned long *max, u64 time)
{
	int cpu = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap, rt;
	s64 delta;

	max_cap = arch_scale_cpu_capacity(NULL, cpu);

	sched_avg_update(rq);
	delta = time - rq->age_stamp;
	if (unlikely(delta < 0))
		delta = 0;
	rt = div64_u64(rq->rt_avg, sched_avg_period() + delta);
	rt = (rt * max_cap) >> SCHED_CAPACITY_SHIFT;

	*util = boosted_cpu_util(cpu);
	if (likely(use_pelt()))
		*util = *util + rt;

	*util = min(*util, max_cap);
	*max = max_cap;
}

static void sugov_set_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	if (flags & SCHED_CPUFREQ_IOWAIT) {
		if (sg_cpu->iowait_boost_pending)
			return;

		sg_cpu->iowait_boost_pending = true;

		if (sg_cpu->iowait_boost) {
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
			sg_cpu->iowait_boost += ((sg_cpu->iowait_boost_max - sg_cpu->iowait_boost) >> 1);
#else
			sg_cpu->iowait_boost <<= 1;
#endif
			if (sg_cpu->iowait_boost > sg_cpu->iowait_boost_max)
				sg_cpu->iowait_boost = sg_cpu->iowait_boost_max;
		} else {
			sg_cpu->iowait_boost = sg_cpu->sg_policy->policy->min;
		}
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;

		/* Clear iowait_boost if the CPU apprears to have been idle. */
		if (delta_ns > TICK_NSEC) {
			sg_cpu->iowait_boost = 0;
			sg_cpu->iowait_boost_pending = false;
		}
	}
}

static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned int boost_util, boost_max;

	if (!sg_cpu->iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost_pending = false;
	} else {
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
		unsigned int descent_step = (sg_cpu->iowait_boost > sg_cpu->sg_policy->policy->min) ?
			((sg_cpu->iowait_boost - sg_cpu->sg_policy->policy->min) >> 1) : sg_cpu->sg_policy->policy->min;

		if (sg_cpu->iowait_boost <= descent_step) {
			sg_cpu->iowait_boost = 0;
			return;
		}
		sg_cpu->iowait_boost -= descent_step;
#else
		sg_cpu->iowait_boost >>= 1;
#endif
		if (sg_cpu->iowait_boost < sg_cpu->sg_policy->policy->min) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	boost_util = sg_cpu->iowait_boost;
	boost_max = sg_cpu->iowait_boost_max;

	if (*util * boost_max < *max * boost_util) {
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
		sg_cpu->sg_policy->max_cpu = sg_cpu->cpu;
		sg_cpu->sg_policy->iowait_boost = boost_util;
#else
		*util = boost_util;
		*max = boost_max;
#endif
	}
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls();
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	busy = sugov_cpu_is_busy(sg_cpu);

	if (flags & SCHED_CPUFREQ_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		sugov_get_util(&util, &max, time);
		sugov_iowait_boost(sg_cpu, &util, &max);
		next_f = get_next_freq(sg_policy, util, max);
		/*
		 * Do not reduce the frequency if the CPU has not been idle
		 * recently, as the reduction is likely to be premature then.
		 */
		if (busy && next_f < sg_policy->next_freq) {
			next_f = sg_policy->next_freq;

			/* Reset cached freq as next_freq has changed */
			sg_policy->cached_raw_freq = 0;
		}
	}
	sugov_update_commit(sg_policy, time, next_f);
}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static void sugov_boost(struct gov_attr_set *attr_set)
{
	struct sugov_policy *sg_policy;
	u64 now;

	now = use_pelt() ? ktime_get_ns() : walt_ktime_clock();

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sugov_update_commit(sg_policy, now, sg_policy->tunables->hispeed_freq);
	}
}

static unsigned int sugov_next_freq_shared_policy(struct sugov_policy *sg_policy,
		u64 time, unsigned int *early_detection)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	int j, i = 0;

	sg_policy->max_cpu = cpumask_first(policy->cpus);
	sg_policy->iowait_boost = 0;
	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
		}

		sugov_update_util(j, time, early_detection[i]);

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
			sg_policy->max_cpu = j;
		}

		sugov_iowait_boost(j_sg_cpu, &util, &max);

		trace_cpufreq_schedutil_get_util(j, j_sg_cpu->util,
						 j_sg_cpu->max,
						 top_task_util(cpu_rq(j)),
						 j_sg_cpu->iowait_boost,
						 j_sg_cpu->flags, early_detection[i]);

		j_sg_cpu->flags = 0;

		i++;
	}

	return get_next_freq(sg_policy, util, max);
}
#endif

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
			continue;
		}
		if (j_sg_cpu->flags & SCHED_CPUFREQ_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		sugov_iowait_boost(j_sg_cpu, &util, &max);
	}

	return get_next_freq(sg_policy, util, max);
}

static void sugov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!raw_spin_trylock(&sg_policy->update_lock)) {
		trace_cpufreq_schedutil_notyet(sg_cpu->cpu,
					       "lock_contention", 0ULL, 0, 0);
		return;
	}

	if (!sg_policy->governor_enabled)
		goto unlock;

	if (unlikely(!sg_policy->util_changed)) {
		trace_cpufreq_schedutil_notyet(sg_cpu->cpu,
					       "util_nochange", 0ULL, 0, 0);
		goto unlock;
	}

	sg_policy->util_changed = false;
	sg_policy->trigger_cpu = sg_cpu->cpu;

	if (sugov_should_update_freq(sg_policy, time)) {
		next_f = 0;
		sugov_update_commit(sg_policy, time, next_f);
	}

unlock:
	raw_spin_unlock(&sg_policy->update_lock);
#else
	unsigned long util, max;

	sugov_get_util(&util, &max, time);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	sugov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (sugov_should_update_freq(sg_policy, time)) {
		if (flags & SCHED_CPUFREQ_DL)
			next_f = sg_policy->policy->cpuinfo.max_freq;
		else
			next_f = sugov_next_freq_shared(sg_cpu, time);

		sugov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
#endif
}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static bool sugov_time_limit(struct sugov_policy *sg_policy, unsigned int next_freq)
{
	u64 delta_ns;
	unsigned int min_sample_time;
	bool skip_min_sample_time = sg_policy->skip_min_sample_time;
	bool skip_hispeed_logic = sg_policy->skip_hispeed_logic;

	if (skip_min_sample_time) {
		sg_policy->skip_min_sample_time = false;
		skip_min_sample_time = sg_policy->tunables->fast_ramp_down;
	}
	if (skip_hispeed_logic) {
		sg_policy->skip_hispeed_logic = false;
		skip_hispeed_logic = sg_policy->tunables->fast_ramp_up;
	}

	if (!skip_hispeed_logic &&
	    next_freq > sg_policy->next_freq &&
	    sg_policy->next_freq >= sg_policy->tunables->hispeed_freq) {
		delta_ns = sg_policy->time - sg_policy->hispeed_validate_time;
		if (delta_ns < NSEC_PER_USEC *
		    freq_to_above_hispeed_delay(sg_policy->tunables, sg_policy->next_freq)) {
			trace_cpufreq_schedutil_notyet(sg_policy->max_cpu,
						       "above_hispeed_delay", delta_ns,
						       sg_policy->next_freq, next_freq);
			return true;
		}
	}

	sg_policy->hispeed_validate_time = sg_policy->time;
	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */

	if (next_freq < sg_policy->next_freq) {
		min_sample_time = freq_to_min_sample_time(sg_policy->tunables, sg_policy->next_freq);

#ifdef CONFIG_HISI_CPU_FREQ_LOCK_DETECT
		if (LONGER_MIN_SAMPLE_TIME_ELAPSED_DURATION > ktime_to_ns(ktime_sub(ktime_get(), sg_policy->end_time))) { //lint !e446
			min_sample_time = (min_sample_time > DEFAULT_RATE_LIMIT_US) ?
					    min_sample_time : DEFAULT_RATE_LIMIT_US;
			skip_min_sample_time = false;
		}
#endif

		if (!skip_min_sample_time) {
			delta_ns = sg_policy->time - sg_policy->floor_validate_time;
			if (delta_ns < NSEC_PER_USEC * min_sample_time) {
				trace_cpufreq_schedutil_notyet(sg_policy->max_cpu,
						       "min_sample_time", delta_ns,
						       sg_policy->next_freq, next_freq);
				return true;
			}
		}
	}

	if (!sg_policy->tunables->boosted ||
	    next_freq > sg_policy->tunables->hispeed_freq)
		sg_policy->floor_validate_time = sg_policy->time;

	return false;
}

void sugov_mark_util_change(int cpu, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_policy *sg_policy;

	if (!sg_cpu->enabled)
		return;

	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy)
		return;
	if (!sg_policy->governor_enabled)
		return;

	sg_cpu->flags |= flags;

	if (flags &
	    (INTER_CLUSTER_MIGRATION_SRC | CLEAR_ED_TASK))
		sg_policy->skip_min_sample_time = true;

	if (flags &
	    (INTER_CLUSTER_MIGRATION_DST | ADD_ED_TASK))
		sg_policy->skip_hispeed_logic = true;

	sg_policy->util_changed = true;
}

void sugov_check_freq_update(int cpu)
{
	struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
	struct sugov_policy *sg_policy;

	if (!sg_cpu->enabled)
		return;

	sg_policy = sg_cpu->sg_policy;
	if (!sg_policy)
		return;
	if (!sg_policy->governor_enabled)
		return;

	if (sg_policy->util_changed)
		cpufreq_update_util(cpu_rq(cpu), 0);
}
#endif /* CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL */

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int next_freq, cpu;
	struct rq *rq;
	int i = 0;
	const int cpus = cpumask_weight(policy->cpus);
	int early_detection[cpus];
#endif

	mutex_lock(&sg_policy->work_lock);

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	for_each_cpu(cpu, policy->cpus) {
		rq = cpu_rq(cpu);
		raw_spin_lock_irq(&rq->lock);

		early_detection[i] = (rq->ed_task != NULL);
		if (early_detection[i])
			sg_policy->skip_hispeed_logic = true;
		else
			walt_update_task_ravg(rq->curr, rq, TASK_UPDATE, walt_ktime_clock(), 0);

		raw_spin_unlock_irq(&rq->lock);
		i++;
	}
	/* Clear util changes marked above in walt update */
	sg_policy->util_changed = false;

	next_freq = sugov_next_freq_shared_policy(sg_policy, sg_policy->time, early_detection);

	if (sugov_time_limit(sg_policy, next_freq))
		goto out;

	if (next_freq == sg_policy->next_freq) {
		trace_cpufreq_schedutil_already(sg_policy->max_cpu,
						sg_policy->next_freq, next_freq);
		goto out;
	}

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = sg_policy->time;
#endif

	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
					CPUFREQ_RELATION_L);

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
out:
#endif
	mutex_unlock(&sg_policy->work_lock);

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	sugov_slack_timer_resched(sg_policy);
#endif
	sg_policy->work_in_progress = false;
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	/*
	 * For RT and deadline tasks, the schedutil governor shoots the
	 * frequency to maximum. Special care must be taken to ensure that this
	 * kthread doesn't result in the same behavior.
	 *
	 * This is (mostly) guaranteed by the work_in_progress flag. The flag is
	 * updated only at the end of the sugov_work() function and before that
	 * the schedutil governor rejects all other frequency scaling requests.
	 *
	 * There is a very rare case though, where the RT thread yields right
	 * after the work_in_progress flag is cleared. The effects of that are
	 * neglected for now.
	 */
	queue_kthread_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_us(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static ssize_t go_hispeed_load_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%lu\n", tunables->go_hispeed_load);
}

static ssize_t go_hispeed_load_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->go_hispeed_load = val;

	return count;
}

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->hispeed_freq = val;

	return count;
}

static ssize_t boost_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->boost);
}

static ssize_t boost_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->boost = val;

	if (tunables->boost) {
		trace_cpufreq_schedutil_boost("on");
		if (!tunables->boosted)
			sugov_boost(attr_set);
	} else {
		tunables->boostpulse_endtime = ktime_to_us(ktime_get());
		trace_cpufreq_schedutil_unboost("off");
	}

	return count;
}

static ssize_t boostpulse_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;
	u64 now;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	now = ktime_to_us(ktime_get());
	if (tunables->boostpulse_endtime + tunables->boostpulse_min_interval > now)
		return count;

	tunables->boostpulse_endtime = now + tunables->boostpulse_duration;
	trace_cpufreq_schedutil_boost("pulse");

	if (!tunables->boosted)
		sugov_boost(attr_set);

	return count;
}

static ssize_t boostpulse_duration_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->boostpulse_duration);
}

static ssize_t boostpulse_duration_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->boostpulse_duration = val;

	return count;
}

static ssize_t io_is_busy_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->io_is_busy);
}

static ssize_t io_is_busy_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->io_is_busy = val;
	sched_set_io_is_busy(val);

	return count;
}

static ssize_t fast_ramp_down_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->fast_ramp_down);
}

static ssize_t fast_ramp_down_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->fast_ramp_down = val;

	return count;
}

static ssize_t fast_ramp_up_show(struct gov_attr_set *attr_set,
		char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->fast_ramp_up);
}

static ssize_t fast_ramp_up_store(struct gov_attr_set *attr_set,
		const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->fast_ramp_up = val;

	return count;
}

#ifdef CONFIG_SCHED_HISI_TOP_TASK
static ssize_t top_task_hist_size_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_hist_size);
}

static ssize_t top_task_hist_size_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	/* Allowed range: [1, RAVG_HIST_SIZE_MAX] */
	if (val < 1 || val > RAVG_HIST_SIZE_MAX)
		return -EINVAL;

	tunables->top_task_hist_size = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_hist_size = val;
		}
	}

	return count;
}

static ssize_t top_task_stats_policy_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_stats_policy);
}

static ssize_t top_task_stats_policy_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->top_task_stats_policy = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_stats_policy = val;
		}
	}

	return count;
}

static ssize_t top_task_stats_empty_window_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->top_task_stats_empty_window);
}

static ssize_t top_task_stats_empty_window_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->top_task_stats_empty_window = val;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		for_each_cpu(cpu, sg_policy->policy->cpus) {
			cpu_rq(cpu)->top_task_stats_empty_window = val;
		}
	}

	return count;
}
#endif /* CONFIG_SCHED_HISI_TOP_TASK */

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1) /* [false alarm]:fortify */
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;
	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s", tunables->target_loads[i],
			       i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return ret;
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int *new_target_loads;
	unsigned long flags;
	int ntokens;

	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_target_loads))
		return PTR_ERR(new_target_loads);

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);
	tunables->target_loads = new_target_loads;
	tunables->ntarget_loads = ntokens;
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return count;
}

static ssize_t above_hispeed_delay_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->nabove_hispeed_delay; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s", tunables->above_hispeed_delay[i],
			       i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);

	return ret;
}

static ssize_t above_hispeed_delay_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int *new_above_hispeed_delay;
	unsigned long flags;
	int ntokens;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_above_hispeed_delay))
		return PTR_ERR(new_above_hispeed_delay);

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);
	if (tunables->above_hispeed_delay != default_above_hispeed_delay)
		kfree(tunables->above_hispeed_delay);
	tunables->above_hispeed_delay = new_above_hispeed_delay;
	tunables->nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);

	return count;
}

static ssize_t min_sample_time_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->min_sample_time_lock, flags);

	for (i = 0; i < tunables->nmin_sample_time; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%s", tunables->min_sample_time[i],
			       i & 0x1 ? ":" : " ");

	scnprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->min_sample_time_lock, flags);

	return ret;
}

static ssize_t min_sample_time_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int *new_min_sample_time;
	unsigned long flags;
	int ntokens;

	new_min_sample_time = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_min_sample_time))
		return PTR_ERR(new_min_sample_time);

	spin_lock_irqsave(&tunables->min_sample_time_lock, flags);
	if (tunables->min_sample_time != default_min_sample_time)
		kfree(tunables->min_sample_time);
	tunables->min_sample_time = new_min_sample_time;
	tunables->nmin_sample_time = ntokens;
	spin_unlock_irqrestore(&tunables->min_sample_time_lock, flags);

	return count;
}

static ssize_t timer_slack_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%d\n", tunables->timer_slack_val);
}

static ssize_t timer_slack_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	int ret;
	unsigned long val;
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	tunables->timer_slack_val = val;
	return count;
}
#endif

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}

	return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static struct governor_attr go_hispeed_load = __ATTR_RW(go_hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr boost = __ATTR_RW(boost);
static struct governor_attr target_loads = __ATTR_RW(target_loads);
static struct governor_attr above_hispeed_delay = __ATTR_RW(above_hispeed_delay);
static struct governor_attr min_sample_time = __ATTR_RW(min_sample_time);
static struct governor_attr boostpulse = __ATTR_WO(boostpulse);
static struct governor_attr boostpulse_duration = __ATTR_RW(boostpulse_duration);
static struct governor_attr io_is_busy = __ATTR_RW(io_is_busy);
static struct governor_attr timer_slack = __ATTR_RW(timer_slack);
static struct governor_attr fast_ramp_down = __ATTR_RW(fast_ramp_down);
static struct governor_attr fast_ramp_up = __ATTR_RW(fast_ramp_up);
#endif
#ifdef CONFIG_SCHED_HISI_TOP_TASK
static struct governor_attr top_task_hist_size = __ATTR_RW(top_task_hist_size);
static struct governor_attr top_task_stats_policy = __ATTR_RW(top_task_stats_policy);
static struct governor_attr top_task_stats_empty_window = __ATTR_RW(top_task_stats_empty_window);
#endif

static struct attribute *sugov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	&go_hispeed_load.attr,
	&hispeed_freq.attr,
	&boost.attr,
	&target_loads.attr,
	&above_hispeed_delay.attr,
	&min_sample_time.attr,
	&boostpulse.attr,
	&boostpulse_duration.attr,
	&io_is_busy.attr,
	&timer_slack.attr,
	&fast_ramp_down.attr,
	&fast_ramp_up.attr,
#endif
#ifdef CONFIG_SCHED_HISI_TOP_TASK
	&top_task_hist_size.attr,
	&top_task_stats_policy.attr,
	&top_task_stats_empty_window.attr,
#endif
	NULL
};

static struct kobj_type sugov_tunables_ktype = {
	.default_attrs = sugov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
};

/********************** cpufreq governor interface *********************/
#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
static
#endif
struct cpufreq_governor cpufreq_gov_schedutil;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	raw_spin_lock_init(&sg_policy->timer_lock);
#endif
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	init_kthread_work(&sg_policy->work, sugov_work);
	init_kthread_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
#ifndef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	kthread_bind_mask(thread, policy->related_cpus);
#endif
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	flush_kthread_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}

	return tunables;
}

static void sugov_tunables_free(struct sugov_tunables *tunables)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;

	kfree(tunables);
}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
static void sugov_nop_timer(unsigned long data)
{
}
#endif

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	/* initialize the tunable parameters */
	tunables->above_hispeed_delay = default_above_hispeed_delay;
	tunables->nabove_hispeed_delay =
		ARRAY_SIZE(default_above_hispeed_delay);
	tunables->min_sample_time = default_min_sample_time;
	tunables->nmin_sample_time =
		ARRAY_SIZE(default_min_sample_time);
	tunables->go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
	tunables->target_loads = default_target_loads;
	tunables->ntarget_loads = ARRAY_SIZE(default_target_loads);
	tunables->boostpulse_duration = DEFAULT_BOOSTPULSE_DURATION;
	tunables->boostpulse_min_interval = DEFAULT_MIN_BOOSTPULSE_INTERVAL;
	tunables->timer_slack_val = DEFAULT_TIMER_SLACK;

	spin_lock_init(&tunables->target_loads_lock);
	spin_lock_init(&tunables->above_hispeed_delay_lock);
	spin_lock_init(&tunables->min_sample_time_lock);
#endif

	if (policy->up_transition_delay_us && policy->down_transition_delay_us) {
		tunables->up_rate_limit_us = policy->up_transition_delay_us;
		tunables->down_rate_limit_us = policy->down_transition_delay_us;
	} else {
		unsigned int lat;

		tunables->up_rate_limit_us = LATENCY_MULTIPLIER;
		tunables->down_rate_limit_us = LATENCY_MULTIPLIER;
		lat = policy->cpuinfo.transition_latency / NSEC_PER_USEC;
		if (lat) {
			tunables->up_rate_limit_us *= lat;
			tunables->down_rate_limit_us *= lat;
		}
	}

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   cpufreq_gov_schedutil.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	policy->governor_data = NULL;
	sugov_tunables_free(tunables);

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static int sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_tunables_free(tunables);

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);

	cpufreq_disable_fast_switch(policy);
	return 0;
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	sg_policy->floor_validate_time = 0;
	sg_policy->hispeed_validate_time = 0;
	/* allow freq drop as soon as possible when policy->min restored */
	sg_policy->min_freq = policy->min;
	sg_policy->util_changed = false;
	sg_policy->time = 0;
#ifdef CONFIG_HISI_CPU_FREQ_LOCK_DETECT
	sg_policy->start_time = ktime_get();
	sg_policy->end_time = sg_policy->start_time;
	sg_policy->locked = false;
#endif
#endif

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_us(sg_policy);
	sg_policy->last_freq_update_time = 0;
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	sg_policy->next_freq = policy->cur;
	sg_policy->trigger_cpu = policy->cpu;
#else
	sg_policy->next_freq = UINT_MAX;
#endif
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;
	sg_policy->cached_raw_freq = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->flags = SCHED_CPUFREQ_DL;
		sg_cpu->iowait_boost_max = policy->cpuinfo.max_freq;
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
		sg_cpu->cpu = cpu;
		sg_cpu->enabled = true;
#endif
	}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	raw_spin_lock(&sg_policy->timer_lock);
	init_timer(&sg_policy->pol_slack_timer);
	sg_policy->pol_slack_timer.function = sugov_nop_timer;
	add_timer_on(&sg_policy->pol_slack_timer, policy->cpu);
	sg_policy->governor_enabled = true;
	raw_spin_unlock(&sg_policy->timer_lock);
#endif

	return 0;
}

static int sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
	raw_spin_lock(&sg_policy->timer_lock);
	sg_policy->governor_enabled = false;
	del_timer_sync(&sg_policy->pol_slack_timer);
	raw_spin_unlock(&sg_policy->timer_lock);
#endif

	for_each_cpu(cpu, policy->cpus) {
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);
		sg_cpu->enabled = false;
#endif
		cpufreq_remove_update_util_hook(cpu);
	}

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}

	return 0;
}

static int sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
#ifdef CONFIG_HISI_CPU_FREQ_LOCK_DETECT
	u64 delta;
	ktime_t now;
#endif

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
		sg_policy->next_freq = policy->cur;
#endif
		mutex_unlock(&sg_policy->work_lock);
	}

#ifdef CONFIG_HISI_CPU_FREQ_GOV_SCHEDUTIL
#ifdef CONFIG_HISI_CPU_FREQ_LOCK_DETECT
	if (policy->min == policy->cpuinfo.max_freq &&
	    policy->min > sg_policy->min_freq) {
		sg_policy->start_time = ktime_get();
		sg_policy->locked = true;
	} else if (sg_policy->locked && policy->min < policy->max) {
		now = ktime_get();
		delta = ktime_to_ns(ktime_sub(now, sg_policy->start_time));
		if (delta >= NSEC_PER_SEC)
			sg_policy->end_time = now;

		sg_policy->locked = false;
	}
#endif

	/* policy->min restored */
	if (sg_policy->min_freq > policy->min)
		sg_policy->skip_min_sample_time = true;

	sg_policy->min_freq = policy->min;
#endif
	sg_policy->need_freq_update = true;

	return 0;
}

static int cpufreq_schedutil_cb(struct cpufreq_policy *policy,
				unsigned int event)
{
	switch(event) {
	case CPUFREQ_GOV_POLICY_INIT:
		return sugov_init(policy);
	case CPUFREQ_GOV_POLICY_EXIT:
		return sugov_exit(policy);
	case CPUFREQ_GOV_START:
		return sugov_start(policy);
	case CPUFREQ_GOV_STOP:
		return sugov_stop(policy);
	case CPUFREQ_GOV_LIMITS:
		return sugov_limits(policy);
	default:
		BUG();
	}
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
static
#endif
struct cpufreq_governor cpufreq_gov_schedutil = {
	.name = "schedutil",
	.governor = cpufreq_schedutil_cb,
	.owner = THIS_MODULE,
};

static int __init sugov_register(void)
{
	return cpufreq_register_governor(&cpufreq_gov_schedutil);
}
fs_initcall(sugov_register);
