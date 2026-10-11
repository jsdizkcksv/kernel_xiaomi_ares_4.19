// SPDX-License-Identifier: GPL-2.0
/*
 * perf_mode: runtime master switch for the MediaTek performance modules.
 *
 * Copyright (c) 2026
 *
 * In "daily" mode the boosting modules (fpsgo, ppm, boost_ctrl/CFP, gbe,
 * task_turbo) are switched off, so they behave as if the performance
 * patches had not been applied.  In "game" mode the values captured at
 * boot are written back.
 *
 * The mode is picked automatically from the GED game-aware-scheduling
 * state, which the GED HAL drives from the foreground application, or it
 * can be forced through /proc/perf_mode (daily / game / auto).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/seq_file.h>
#include <linux/delay.h>

#define PERF_MODE_DAILY		0
#define PERF_MODE_GAME		1
#define PERF_MODE_AUTO		2

#define PERF_POLL_MS		2000

/*
 * GED game-aware scheduling.  The values mirror
 * drivers/gpu/mediatek/ged/include/ged_type.h:
 *   GED_EVENT_GAS_MODE = 15, GAS_CATEGORY_GAME = 0
 * ged_query_info() is exported by the GED driver.
 */
#define PERF_GED_EVENT_GAS_MODE	15
#define PERF_GAS_CATEGORY_GAME	0
extern unsigned long ged_query_info(int eType);

struct perf_sw {
	const char *path;
	const char *off;	/* value written in daily mode */
	char saved[24];		/* boot-time value, written back in game mode */
	bool saved_ok;
};

static struct perf_sw perf_sws[] = {
	{ "/sys/kernel/fpsgo/common/fpsgo_enable",		"0", {0}, false },
	{ "/proc/ppm/enabled",					"0", {0}, false },
	{ "/proc/perfmgr/boost_ctrl/cpu_ctrl/cfp_enable",	"0", {0}, false },
	{ "/sys/kernel/gbe/gbe_enable1",			"0", {0}, false },
	{ "/sys/kernel/gbe/gbe_enable2",			"0", {0}, false },
	{ "/sys/module/task_turbo/parameters/feats",		"0", {0}, false },
};

static int perf_mode = PERF_MODE_AUTO;
static int perf_applied = -1;
static struct delayed_work perf_work;

static int perf_node_write(const char *path, const char *val)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);
	ret = kernel_write(f, val, strlen(val), &pos);
	filp_close(f, NULL);

	return ret;
}

static int perf_node_read(const char *path, char *buf, int sz)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);
	ret = kernel_read(f, buf, sz - 1, &pos);
	filp_close(f, NULL);
	if (ret < 0)
		return ret;
	buf[ret] = '\0';

	return 0;
}

/* Capture the boot-time value once, so game mode can restore it. */
static void perf_save_defaults(void)
{
	int i;
	char buf[24];

	for (i = 0; i < ARRAY_SIZE(perf_sws); i++) {
		if (perf_sws[i].saved_ok)
			continue;
		if (perf_node_read(perf_sws[i].path, buf, sizeof(buf)) < 0)
			continue;
		strim(buf);
		strscpy(perf_sws[i].saved, buf, sizeof(perf_sws[i].saved));
		perf_sws[i].saved_ok = true;
	}
}

static void perf_apply(int mode)
{
	int i;

	if (mode == perf_applied)
		return;

	perf_save_defaults();

	for (i = 0; i < ARRAY_SIZE(perf_sws); i++) {
		if (mode == PERF_MODE_DAILY)
			perf_node_write(perf_sws[i].path, perf_sws[i].off);
		else if (perf_sws[i].saved_ok)
			perf_node_write(perf_sws[i].path, perf_sws[i].saved);
	}

	perf_applied = mode;
	pr_info("perf_mode: applied %s\n",
		mode == PERF_MODE_DAILY ? "daily" : "game");
}

static void perf_poll(struct work_struct *work)
{
	int mode = perf_mode;

	if (mode == PERF_MODE_AUTO)
		mode = (ged_query_info(PERF_GED_EVENT_GAS_MODE)
			== PERF_GAS_CATEGORY_GAME) ?
			PERF_MODE_GAME : PERF_MODE_DAILY;

	perf_apply(mode);
	schedule_delayed_work(&perf_work, msecs_to_jiffies(PERF_POLL_MS));
}

static int perf_mode_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", perf_mode == PERF_MODE_DAILY ? "daily" :
		   perf_mode == PERF_MODE_GAME ? "game" : "auto");

	return 0;
}

static ssize_t perf_mode_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *pos)
{
	char buf[16];

	if (count == 0 || count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	strim(buf);

	if (!strcmp(buf, "daily"))
		perf_mode = PERF_MODE_DAILY;
	else if (!strcmp(buf, "game"))
		perf_mode = PERF_MODE_GAME;
	else if (!strcmp(buf, "auto"))
		perf_mode = PERF_MODE_AUTO;
	else
		return -EINVAL;

	perf_applied = -1;	/* force re-apply */
	schedule_delayed_work(&perf_work, 0);

	return count;
}

static int perf_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, perf_mode_show, NULL);
}

static const struct file_operations perf_mode_fops = {
	.owner		= THIS_MODULE,
	.open		= perf_mode_open,
	.read		= seq_read,
	.write		= perf_mode_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init perf_mode_init(void)
{
	proc_create("perf_mode", 0644, NULL, &perf_mode_fops);
	INIT_DELAYED_WORK(&perf_work, perf_poll);
	schedule_delayed_work(&perf_work, msecs_to_jiffies(PERF_POLL_MS));
	pr_info("perf_mode: ready (auto, GED driven)\n");

	return 0;
}
late_initcall(perf_mode_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Runtime master switch for MediaTek performance modules");
