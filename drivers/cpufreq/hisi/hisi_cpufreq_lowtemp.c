/*
 * Hisilicon Platforms LOW TEMPERATURE cpu frequency set support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/hisi/hisi_rproc.h>
#include <linux/hisi/ipc_msg.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>


static DEFINE_MUTEX(cpu_lowtemp_mutex);
struct cpu_lowtemp {
	void __iomem *base;
	uint32_t lowtemp_reg;
	uint32_t reg_offset;
	uint8_t initialized;
	uint8_t frequp;
	struct delayed_work work;
#ifdef CONFIG_HISI_CPUFREQ_LOWTEMP_DEBUG
	uint32_t h_time;
	uint32_t l_time;
#endif
};
static struct cpu_lowtemp *cpu_lowtemp_data;
enum {
	CPUFREQ_LOWTEMP_LEVEL0,
	CPUFREQ_LOWTEMP_LEVEL1,
	CPUFREQ_LOWTEMP_LEVEL_MAX,
};

#define TIMEOUT_MAX (3000)
bool is_low_temprature(void)
{
	unsigned int temprature = 0;

	if (!cpu_lowtemp_data->initialized)
		return false;

	temprature = readl(cpu_lowtemp_data->base + cpu_lowtemp_data->lowtemp_reg);
	temprature = temprature >> cpu_lowtemp_data->reg_offset & 0x3;

	if (temprature)
		return true;
	else
		return false;
}

static int cpufreq_lowtemp_sendcmd(unsigned int cpufreq_lvl)
{
	unsigned int msg[2];
	rproc_id_t id = HISI_RPROC_LPM3_MBX13;
	int ret = 0;

	if (RPROC_IS_SUSPEND(id)) {
		pr_err("[%s] rproc in suspend\n", __func__);
		return -EBUSY;
	}

	msg[0] = IPC_CMD(OBJ_AP, OBJ_LITTLE_CLUSTER, CMD_SETTING, TYPE_UPLIMIT);
	msg[1] = cpufreq_lvl;

	ret = RPROC_ASYNC_SEND(id, (mbox_msg_t *)msg, 2);
	if (ret)
		pr_err("[%s] send cpufreq lowtemp error %d\n", __func__, ret);

#ifdef CONFIG_HISI_CPUFREQ_LOWTEMP_DEBUG
	if (!ret) {
		if (cpufreq_lvl == CPUFREQ_LOWTEMP_LEVEL1)
			cpu_lowtemp_data->h_time++;
		else
			cpu_lowtemp_data->l_time++;
	}
#endif

	return ret;
}

int cpufreq_lowtemp_request(const unsigned int timeout_ms)
{
	int ret = 0;

	if (!cpu_lowtemp_data->initialized)
		return -ENODEV;

	if (timeout_ms > TIMEOUT_MAX)
		return -EPERM;

	cancel_delayed_work_sync(&cpu_lowtemp_data->work);

	mutex_lock(&cpu_lowtemp_mutex);
	if (!cpu_lowtemp_data->frequp) {
		ret = cpufreq_lowtemp_sendcmd(CPUFREQ_LOWTEMP_LEVEL1);
		if (!ret)
			cpu_lowtemp_data->frequp = 1;
	}
	mutex_unlock(&cpu_lowtemp_mutex);

	schedule_delayed_work(&cpu_lowtemp_data->work, msecs_to_jiffies(timeout_ms));

	return ret;
}

static void cpufreq_lowtemp_release_check_work(struct work_struct *work)
{
	int ret = 0;

	if (cpu_lowtemp_data->frequp) {
		ret = cpufreq_lowtemp_sendcmd(CPUFREQ_LOWTEMP_LEVEL0);
		if (!ret)
			cpu_lowtemp_data->frequp = 0;
		else
			pr_err("[%s] release cpu0 lowtemp freq fail\n", __func__);
	}
}

static int __init cpufreq_lowtemp_init(void)
{
	int ret;
	struct device_node *np = NULL;
	uint32_t reg = 0, offset = 0;
	void __iomem *base;

	cpu_lowtemp_data = kzalloc(sizeof(struct cpu_lowtemp), GFP_KERNEL);
	if (!cpu_lowtemp_data)
		return -ENOMEM;

	np = of_find_compatible_node(NULL, NULL, "hisi,cpu-lowtemp");
	if (!np) {
		pr_err("[%s] no compatible node found.\n", __func__);
		ret =  -EPERM;
		goto out_find_node;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("[%s] base iomap fail\n", __func__);
		ret = -ENOMEM;
		goto out_iomap;
	}
	ret = of_property_read_u32(np, "hisi,lowtemp-reg", &reg);
	if (ret) {
		pr_err("[%s] get lowtemp reg failed.\n", __func__);
		ret = -EPERM;
		goto out;
	}

	ret = of_property_read_u32(np, "hisi,lowtemp-reg-offset", &offset);
	if (ret) {
		pr_err("[%s] get lowtemp reg offset failed.\n", __func__);
		ret = -EPERM;
		goto out;
	}
	cpu_lowtemp_data->base = base;
	cpu_lowtemp_data->lowtemp_reg = reg;
	cpu_lowtemp_data->reg_offset = offset;
	INIT_DELAYED_WORK(&cpu_lowtemp_data->work, cpufreq_lowtemp_release_check_work);
	cpu_lowtemp_data->initialized = 1;

	return 0;

out:
	iounmap(base);
out_iomap:
	of_node_put(np);
out_find_node:
	kfree(cpu_lowtemp_data);
	cpu_lowtemp_data = NULL;
	return ret;
}
module_init(cpufreq_lowtemp_init);

#ifdef CONFIG_HISI_CPUFREQ_LOWTEMP_DEBUG
static struct dentry *cpufreq_lowtemp_debug_dir;

static int cpufreq_lowtemp_debugfs_show(struct seq_file *s, void *data)
{
	bool lowtemp;

	if (!s) {
		pr_err("[%s] seq_file is null\n", __func__);
		return -EINVAL;
	}

	lowtemp = is_low_temprature();
	seq_printf(s, "lowtemp state: %d\n", lowtemp);
	seq_printf(s, "high freq times:%d, low freq times:%d\n",
			cpu_lowtemp_data->h_time, cpu_lowtemp_data->l_time);

	return 0;
}

static int cpufreq_lowtemp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpufreq_lowtemp_debugfs_show, inode->i_private);
}

static ssize_t cpulowtemp_debugfs_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	int timeout_ms = -1;
	int ret;

	ret = kstrtos32_from_user(buf, count, 0, &timeout_ms);
	if (ret)
		return ret;

	if ((timeout_ms > 0) && (timeout_ms <= TIMEOUT_MAX))
		cpufreq_lowtemp_request(timeout_ms);

	return count;
}

static const struct file_operations cpufreq_lowtemp_debugfs_fops = {
	.owner   = THIS_MODULE,
	.open    = cpufreq_lowtemp_debugfs_open,
	.read    = seq_read,
	.write   = cpulowtemp_debugfs_write,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int __init cpufreq_lowtemp_debug_init(void)
{
	int ret = -ENODEV;

	cpufreq_lowtemp_debug_dir = debugfs_create_dir("hisi_cpulowtemp_debug", NULL);
	if (IS_ERR_OR_NULL(cpufreq_lowtemp_debug_dir)) {
		pr_err("[%s] create cpulowtemp_debug_dir fail\n", __func__);
		goto out;
	}

	if (NULL == debugfs_create_file("lowtemp_debug", 0660, cpufreq_lowtemp_debug_dir, NULL, &cpufreq_lowtemp_debugfs_fops)) {
		debugfs_remove_recursive(cpufreq_lowtemp_debug_dir);
		goto out;
	}

	return 0;
out:
	cpufreq_lowtemp_debug_dir = NULL;
	return ret;
}
module_init(cpufreq_lowtemp_debug_init);

static void __exit cpufreq_lowtemp_debug_exit(void)
{
	debugfs_remove_recursive(cpufreq_lowtemp_debug_dir);
	cpufreq_lowtemp_debug_dir = NULL;
}
module_exit(cpufreq_lowtemp_debug_exit);

#endif
