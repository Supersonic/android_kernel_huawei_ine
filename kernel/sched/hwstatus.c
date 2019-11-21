/*
 * kernel/sched/hwstatus.c
 *
 * Copyright(C) 2018, Huawei, Inc., hwfatboy
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/mempolicy.h>
#include <log/log_usertype/log-usertype.h>

#include "sched.h"

/*
 * Ease the printing of nsec fields:
 */
static long long nsec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
	if ((long long)nsec < 0)
		nsec = -nsec;

	return do_div(nsec, 1000000);
}

#define SEQ_printf(m, x...)			\
 do {						\
	if (m)					\
		seq_printf(m, x);		\
	else					\
		printk(x);			\
 } while (0)

#define SPLIT_NS(x) nsec_high(x), nsec_low(x)

#define PN(F) \
        SEQ_printf(m,"  .%-30s: %lld.%06ld\n",#F, SPLIT_NS((long long )F))

#define FGTASK_MAX 4
enum{
	FG_UI = 0,
	FG_RENDER,
	PREV_UI,
	PREV_RENDER
};

typedef struct sched_hwstatus_rsthead
{
	u64 ktime_last;
	u64 ktime_now;
}sched_hwstatus_rsthead;

typedef struct sched_hwstatus_rstbody
{
	pid_t pid;
	pid_t tgid;
	sched_hwstatus hwstatus;
}sched_hwstatus_rstbody;

typedef struct sched_hwstatus_rst
{
	sched_hwstatus_rsthead head;
	sched_hwstatus_rstbody body[FGTASK_MAX];
}sched_hwstatus_rst;

static pid_t fgtasks[FGTASK_MAX] = {0};
static u64 ktime_last = 0;

static void clearpid(pid_t pid)
{
	struct task_struct *taskp;
	struct sched_statistics *ssp;

	taskp = find_task_by_vpid(pid);
	if(!taskp){
		return;
	}

	get_task_struct(taskp);
	ssp = &(taskp->se.statistics);
	ssp->hwstatus.sum_exec_runtime_big   = 0;
	ssp->hwstatus.sum_exec_runtime_ltt   = 0;
	ssp->hwstatus.wait_sum            = ssp->wait_sum;
	ssp->hwstatus.sum_sleep_runtime   = ssp->sum_sleep_runtime;

	ssp->hwstatus.iowait_sum = ssp->iowait_sum;
	ssp->hwstatus.iowait_max    = 0;

	ssp->hwstatus.dstate_block_count = 0;
	ssp->hwstatus.wait_count = ssp->wait_count;
	ssp->hwstatus.iowait_count = ssp->iowait_count;
	ssp->hwstatus.sleep_count = 0;

	ssp->hwstatus.dstate_block_max   = 0;
	ssp->hwstatus.dstate_block_sum   = 0;
	ssp->hwstatus.last_jiffies = jiffies;
	put_task_struct(taskp);
}

static void sched_hwstatus_clear(pid_t pid)
{
	if(pid == 0) {
		clearpid(fgtasks[0]);
		clearpid(fgtasks[1]);
	} else {
		clearpid(pid);
	}
}

void sched_hwstatus_updatefg(pid_t pid, pid_t tgid)
{
	if(BETA_USER != get_logusertype_flag()) {
		return;
	}

	if(tgid != fgtasks[FG_UI]) {
		sched_hwstatus_clear(pid);
		if(pid != tgid) {
			fgtasks[PREV_UI]     = fgtasks[FG_UI];
			fgtasks[PREV_RENDER] = fgtasks[FG_RENDER];

			fgtasks[FG_UI]     = tgid;
			fgtasks[FG_RENDER] = pid;
		}
	}
	return;
}

static int sched_hwstatus_show(struct seq_file *m, void *v)
{
	int i;

	if(BETA_USER != get_logusertype_flag()) {
		return 0;
	}

	for(i=0;  i<FGTASK_MAX; i++){
		struct task_struct *taskp;
		sched_hwstatus *statusp;
		u64 wait_sum;
		u64 sum_sleep_runtime;
		u64 iowait_sum;
		u64 wait_count;
		u64 iowait_count;

		taskp = find_task_by_vpid(fgtasks[i]);
		if(!taskp){
			SEQ_printf(m, "### %s find_task_by_vpid err!\n", __FUNCTION__);
			return 0;
		}

		get_task_struct(taskp);
		statusp = &(taskp->se.statistics.hwstatus);
		SEQ_printf(m, "pid:%d,jiffies:%llu\n", fgtasks[i],jiffies - statusp->last_jiffies);
		wait_sum = taskp->se.statistics.wait_sum - statusp->wait_sum;
		sum_sleep_runtime = taskp->se.statistics.sum_sleep_runtime - statusp->sum_sleep_runtime;
		iowait_sum = taskp->se.statistics.iowait_sum - statusp->iowait_sum;
		PN(statusp->sum_exec_runtime_big);
		PN(statusp->sum_exec_runtime_ltt);
		PN(wait_sum);
		PN(sum_sleep_runtime);
		PN(iowait_sum);
		PN(statusp->iowait_max);

		PN(statusp->dstate_block_max);
		PN(statusp->dstate_block_sum);

		PN(statusp->dstate_block_count);
		wait_count = taskp->se.statistics.wait_count - statusp->wait_count;
		iowait_count = taskp->se.statistics.iowait_count - statusp->iowait_count;
		PN(wait_count);
		PN(iowait_count);
		PN(statusp->sleep_count);
		put_task_struct(taskp);
	}

	return 0;
}

static ssize_t sched_hwstatus_read(struct file* file, char __user *buf,
			size_t count, loff_t *ppos)
{
	int i;
	static unsigned long last_jiffies = 0;
	sched_hwstatus_rst hwstatus_rst;

	if(BETA_USER != get_logusertype_flag()) {
		return 0;
	}

	if(count != sizeof(hwstatus_rst))
	{
		return -1;
	}

	if((jiffies - last_jiffies)< HZ){
		return 0;
	}

	last_jiffies = jiffies;

	for(i=0;  i<FGTASK_MAX; i++){
		struct task_struct *taskp;
		struct sched_statistics *ssp;
		sched_hwstatus_rstbody  *rstp;
		taskp = find_task_by_vpid(fgtasks[i]);
		if(!taskp){
			return -1;
		}

		get_task_struct(taskp);
		ssp  = &(taskp->se.statistics);
		rstp = &hwstatus_rst.body[i];
		rstp->pid      = fgtasks[i];
		rstp->tgid     = fgtasks[(i>>1)<<1];
		rstp->hwstatus.last_jiffies  = jiffies - ssp->hwstatus.last_jiffies;
		rstp->hwstatus.sum_exec_runtime_big = ssp->hwstatus.sum_exec_runtime_big;
		rstp->hwstatus.sum_exec_runtime_ltt = ssp->hwstatus.sum_exec_runtime_ltt;
		rstp->hwstatus.wait_sum            = ssp->wait_sum - ssp->hwstatus.wait_sum;
		rstp->hwstatus.sum_sleep_runtime   = ssp->sum_sleep_runtime - ssp->hwstatus.sum_sleep_runtime;

		rstp->hwstatus.iowait_sum = ssp->iowait_sum - ssp->hwstatus.iowait_sum;
		rstp->hwstatus.iowait_max = ssp->hwstatus.iowait_max;

		rstp->hwstatus.dstate_block_max = ssp->hwstatus.dstate_block_max;
		rstp->hwstatus.dstate_block_sum = ssp->hwstatus.dstate_block_sum;

		rstp->hwstatus.wait_count   = ssp->wait_count - ssp->hwstatus.wait_count;
		rstp->hwstatus.sleep_count  = ssp->hwstatus.sleep_count;
		rstp->hwstatus.iowait_count = ssp->iowait_count - ssp->hwstatus.iowait_count;
		rstp->hwstatus.dstate_block_count = ssp->hwstatus.dstate_block_count;
		put_task_struct(taskp);
	}

	hwstatus_rst.head.ktime_last = ktime_last;
	hwstatus_rst.head.ktime_now  = ktime_get_ns();

	return simple_read_from_buffer(buf, count, ppos, &hwstatus_rst, sizeof(hwstatus_rst));
}



static ssize_t sched_hwstatus_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	pid_t pid = 0;
	int rv;
	static unsigned long last_jiffies = 0;

	if(BETA_USER != get_logusertype_flag()) {
		return 0;
	}

	if(count > 10) {
		return 0;
	}

	if((jiffies - last_jiffies)< HZ){
		return 0;
	}

	last_jiffies = jiffies;
	rv = kstrtos32_from_user(buf, count, 10, &pid);
	if (rv < 0)
		return rv;

	if(0 == pid) {
		sched_hwstatus_show(NULL,NULL);
		return count;
	}

	sched_hwstatus_clear(0);
	ktime_last = ktime_get_ns();

	return count;
}

static const struct file_operations sched_hwstatus_fops = {
	.read		= sched_hwstatus_read,
	.write		= sched_hwstatus_write,
	.llseek		= seq_lseek,
};

static int __init init_sched_hwstatus_procfs(void)
{
	struct proc_dir_entry *pe;

	pe = proc_create("sched_hw", 0440, NULL, &sched_hwstatus_fops);
	if (!pe)
		return -ENOMEM;
	return 0;
}

__initcall(init_sched_hwstatus_procfs);
