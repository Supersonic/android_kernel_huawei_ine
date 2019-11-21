#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <stdarg.h>

#include "tc_ns_log.h"
#include "securec.h"
#include "teek_ns_client.h"
#include "smc.h"
#include "teek_client_api.h"

static struct dentry *tz_dbg_dentry;

extern void tzdebug_archivelog(void);
extern void wakeup_tc_siq(void);

typedef void (*tzdebug_opt_func)(char* param);
struct opt_ops {
	char* name;
	tzdebug_opt_func func;
};

static void archivelog(char* param)
{
	(void)param;
	tzdebug_archivelog();
}
static void tzdump(char* param)
{
	(void)param;
	wakeup_tc_siq();
}
static struct opt_ops optArr[]={
	 {"archivelog",archivelog},
	 {"dump",tzdump},
};

static ssize_t tz_dbg_opt_write(struct file *filp,
                               const char __user *ubuf, size_t cnt,
                               loff_t *ppos)
{
	char buf[128] = {0};
	char* value;
	char* p;
	unsigned int i;
	if (!ubuf || !filp || !ppos)
		return -EINVAL;

	if (cnt >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = 0;
	if (cnt>0 && buf[cnt-1]=='\n') {
		buf[cnt-1] = 0;
	}
	value = buf;
	p = strsep(&value, ":");
	for( i=0;i< sizeof(optArr)/sizeof(struct opt_ops);i++) {
		if(!strncmp(p, optArr[i].name, strlen(optArr[i].name))) {
			optArr[i].func(value);
			return cnt;
		}
	}
	return -EFAULT;
}


static const struct file_operations tz_dbg_opt_fops = {
	.owner = THIS_MODULE,
	.write = tz_dbg_opt_write,
};

static int __init tzdebug_init(void)
{
	tz_dbg_dentry = debugfs_create_dir("tzdebug", NULL);
	if ( !tz_dbg_dentry)
		return 0;
	debugfs_create_file("opt", 0440, tz_dbg_dentry,NULL, &tz_dbg_opt_fops);
	return 0;
}
static void __exit tzdebug_exit(void)
{
	if (!tz_dbg_dentry)
		return;
	debugfs_remove_recursive(tz_dbg_dentry);
	tz_dbg_dentry = NULL;
}
module_init(tzdebug_init);
module_exit(tzdebug_exit);
