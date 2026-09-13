/*
 * MTK frame boost scheduler (frame_boost)
 *
 * A self-contained, closed-loop CPU / GPU / DDR boost scheduler that fills
 * the hole left by the missing MediaTek performance stack on this 4.19
 * kernel (fpsgo.ko is a 4.14 artifact and fails to load, CONFIG_MTK_EMI /
 * CONFIG_MTK_DVFSRC are off, PPM is a dummy, no devfreq devices exist).
 *
 * The scheduler owns three actuators:
 *
 *   CPU   : per-cluster min-frequency floor published through the PPM
 *           sysboost kicker (mt_ppm_sysboost_set_freq_limit).  The generic
 *           PM QoS freq_qos(FREQ_QOS_MIN) request is kept as well, because
 *           cpufreq_set_policy() aggregates every FREQ_QOS_MIN request, so
 *           the driver cooperates with thermal/the governor instead of
 *           fighting them.  A plain scaling_min_freq write would be useless
 *           here: miuibooster overwrites it within milliseconds.
 *   GPU   : mt_gpufreq_target(idx, KIR_POLICY), re-asserted periodically
 *           because the GPU DVFS governor recomputes its target every tick.
 *   DDR   : MTK PM QoS MTK_PM_QOS_DDR_OPP / VCORE_OPP, consumed by the
 *           helio-dvfsrc driver.
 *
 * The boost level is the maximum over all active requests.  Requests come
 * from the input edge detector (a gesture started -> boost immediately) and
 * from userspace through /sys/kernel/frame_boost/request.
 *
 * Userspace is expected to be the frame-aware brain (only it can read real
 * SurfaceFlinger frame statistics); this driver is the low-latency actuator.
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0
 */

#define pr_fmt(fmt) "frame_boost: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/input.h>
#include <linux/thermal.h>
#include <linux/cpu.h>

#include <linux/soc/mediatek/mtk-pm-qos.h>
#include <mtk_gpufreq.h>
#include <mtk_ppm_api.h>

/* ------------------------------------------------------------------ */
/* Tunables and level table                                            */
/* ------------------------------------------------------------------ */

#define FB_MAX_LEVEL		3
#define FB_MAX_REQUESTS		12
#define FB_MAX_CLUSTERS		4
#define FB_DISCOVER_RETRIES	40

enum fb_tag {
	FB_TAG_USER = 0,	/* /sys/kernel/frame_boost/request */
	FB_TAG_TOUCH,		/* input edge detector            */
	FB_NUM_TAGS
};

/*
 * A boost level maps to concrete hardware floors.
 *
 * cpu_pct   : CPU floor as a percentage of each cluster's max frequency.
 *             0 releases the floor.
 * gpu_pct   : GPU floor as a percentage of the OPP segment's top frequency.
 *             0 releases the request.
 * ddr_opp   : MTK PM QoS DDR OPP.  Lower value == faster / more voltage.
 *             MTK_PM_QOS_DDR_OPP_DEFAULT_VALUE (16) means "no request".
 * vcore_opp : MTK PM QoS VCORE OPP, same convention.
 */
struct fb_level {
	u8 cpu_pct;
	u8 gpu_pct;
	u8 ddr_opp;
	u8 vcore_opp;
	const char *name;
};

static const struct fb_level fb_levels[FB_MAX_LEVEL + 1] = {
	[0] = {  0,  0, 16, 16, "idle"    },
	[1] = { 50, 30,  5,  8, "touch"   },
	[2] = { 70, 65,  3,  5, "sustain" },
	[3] = { 85, 90,  1,  2, "panic"   },
};

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

struct fb_cpu_slot {
	bool valid;
	unsigned int first_cpu;
	unsigned int max_freq;
	struct freq_qos_request qos;
	bool qos_added;
	unsigned int applied_freq;
};

struct fb_request {
	bool active;
	u8 level;
	u8 tag;
	unsigned long expires;	/* jiffies */
};

static DEFINE_MUTEX(fb_lock);

static struct fb_request fb_reqs[FB_MAX_REQUESTS];
static struct fb_cpu_slot fb_cpus[FB_MAX_CLUSTERS];
static int fb_num_clusters;
static int fb_discover_retries;

static struct delayed_work fb_work;
static struct kobject *fb_kobj;

static bool fb_enabled = true;
static bool fb_touch_enabled = true;
static u8 fb_effective_level;

static struct mtk_pm_qos_request fb_ddr_req;
static struct mtk_pm_qos_request fb_vcore_req;
static bool fb_qos_added;

static struct thermal_zone_device *fb_tz_cpu;
/*
 * mtktscpu idles around 45-60 C on this device and MTK only starts throttling
 * the AP near 85-95 C, so the guard has to sit well above the idle range.
 */
static int fb_thermal_limit = 85000;	/* milli-degree C */
static int fb_thermal_clear = 78000;
static bool fb_thermal_blocked;

static unsigned int fb_poll_ms = 200;		/* idle poll */
static unsigned int fb_gpu_reassert_ms = 20;	/* GPU re-assert while boosting */
static unsigned int fb_touch_hold_ms = 1200;	/* gesture boost hold */

/* stats */
static u64 fb_stat_level_ups;
static u64 fb_stat_touch_boosts;
static u64 fb_stat_user_boosts;
static u64 fb_stat_thermal_blocks;
static u64 fb_stat_boosted_ms;
static u8 fb_stat_max_level;
static unsigned int fb_last_delay_ms;

/* touch edge, written from atomic context */
static unsigned long fb_touch_last;
static bool fb_touch_seen;

void fb_discover_clusters(void);

/* ------------------------------------------------------------------ */
/* Actuators                                                           */
/* ------------------------------------------------------------------ */

/*
 * Pick the lowest supported frequency that is >= target.  Operators differ
 * in freq_table ordering, so scan every entry instead of assuming order.
 */
static unsigned int fb_pick_freq(struct cpufreq_policy *policy,
				 unsigned int target)
{
	unsigned int best = 0;
	int i;

	if (!policy->freq_table)
		return target;

	for (i = 0; policy->freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		unsigned int f = policy->freq_table[i].frequency;

		if (f == CPUFREQ_ENTRY_INVALID || f == 0)
			continue;
		if (f >= target && (!best || f < best))
			best = f;
	}

	return best ? best : target;
}

/*
 * PPM kicker used for the CPU floor.
 *
 * Writing cpufreq's scaling_min_freq from userspace does NOT survive on
 * HyperOS: miuibooster rewrites it within milliseconds (measured).  PPM's
 * sysboost kicker is the platform-native floor and does survive, which is
 * why it is the primary CPU actuator here.
 */
static int fb_ppm_user = BOOST_BY_PERFSERV;
module_param_named(ppm_user, fb_ppm_user, int, 0644);
MODULE_PARM_DESC(ppm_user, "PPM sysboost kicker id used for the CPU floor");

static void fb_apply_cpu(u8 level)
{
	const struct fb_level *lv = &fb_levels[level];
	int i;

	for (i = 0; i < fb_num_clusters; i++) {
		struct fb_cpu_slot *slot = &fb_cpus[i];
		unsigned int want;

		if (!slot->valid && !slot->qos_added)
			continue;

		if (!lv->cpu_pct) {
			want = 0;
		} else {
			struct cpufreq_policy *policy;
			unsigned int target;

			target = (unsigned int)((u64)slot->max_freq *
						lv->cpu_pct / 100);
			policy = cpufreq_cpu_get(slot->first_cpu);
			want = policy ? fb_pick_freq(policy, target) : target;
			if (policy)
				cpufreq_cpu_put(policy);
		}

		if (want == slot->applied_freq)
			continue;

		/*
		 * Primary: PPM sysboost kicker.  Survives miuibooster and is
		 * reflected in scaling_min_freq like any other floor.
		 */
		if (fb_ppm_user >= 0 && fb_ppm_user < NR_PPM_SYSBOOST_USER)
			mt_ppm_sysboost_set_freq_limit(
				(enum ppm_sysboost_user)fb_ppm_user, i,
				want ? (int)want : -1, -1);

		/* Fallback / belt-and-braces: generic PM QoS frequency floor. */
		if (slot->qos_added)
			freq_qos_update_request(&slot->qos, want);

		slot->applied_freq = want;
	}
}

static void fb_apply_gpu(u8 level)
{
	const struct fb_level *lv = &fb_levels[level];
	unsigned int num, offset;

	if (!lv->gpu_pct)
		/* No forced target: hand control back to the DVFS governor. */
		return;

	if (mt_gpufreq_not_ready())
		return;

	num = mt_gpufreq_get_dvfs_table_num();
	if (!num || num > 512)
		return;

	/*
	 * For KIR_POLICY the driver adds g_segment_max_opp_idx itself, so
	 * request_idx is an offset from the fastest OPP in the segment.
	 */
	offset = (num - 1) * (100 - lv->gpu_pct) / 100;

	mt_gpufreq_target(offset, KIR_POLICY);
}

static void fb_apply_ddr(u8 level)
{
	const struct fb_level *lv = &fb_levels[level];

	if (!fb_qos_added)
		return;

	mtk_pm_qos_update_request(&fb_ddr_req, lv->ddr_opp);
	mtk_pm_qos_update_request(&fb_vcore_req, lv->vcore_opp);
}

/* ------------------------------------------------------------------ */
/* Request bookkeeping                                                 */
/* ------------------------------------------------------------------ */

static u8 fb_compute_level(void)
{
	u8 best = 0;
	int i;

	for (i = 0; i < FB_MAX_REQUESTS; i++) {
		if (!fb_reqs[i].active)
			continue;
		if (time_after(jiffies, fb_reqs[i].expires)) {
			fb_reqs[i].active = false;
			continue;
		}
		if (fb_reqs[i].level > best)
			best = fb_reqs[i].level;
	}

	if (fb_thermal_blocked)
		best = 0;

	return best;
}

static int fb_request(u8 tag, u8 level, unsigned int timeout_ms)
{
	int i, free = -1;
	unsigned long expires;

	if (level > FB_MAX_LEVEL)
		level = FB_MAX_LEVEL;

	expires = jiffies + msecs_to_jiffies(timeout_ms ? timeout_ms : 1);

	/* Refresh this tag's slot if it has one, otherwise take a free slot. */
	for (i = 0; i < FB_MAX_REQUESTS; i++) {
		if (fb_reqs[i].active && fb_reqs[i].tag == tag) {
			if (level > fb_reqs[i].level)
				fb_reqs[i].level = level;
			if (time_after(expires, fb_reqs[i].expires))
				fb_reqs[i].expires = expires;
			return i;
		}
		if (!fb_reqs[i].active && free < 0)
			free = i;
	}

	if (free < 0)
		return -ENOSPC;

	fb_reqs[free].active = true;
	fb_reqs[free].tag = tag;
	fb_reqs[free].level = level;
	fb_reqs[free].expires = expires;

	return free;
}

/* Shortest time we may sleep before something needs attention. */
static unsigned int fb_next_delay_ms(void)
{
	unsigned int delay = fb_effective_level ? fb_gpu_reassert_ms
						: fb_poll_ms;
	unsigned long now = jiffies;
	int i;

	for (i = 0; i < FB_MAX_REQUESTS; i++) {
		unsigned int ms;

		if (!fb_reqs[i].active)
			continue;

		if (time_after(now, fb_reqs[i].expires))
			return 1;

		ms = jiffies_to_msecs(fb_reqs[i].expires - now);
		if (ms < delay)
			delay = ms;
	}

	return delay ? delay : 1;
}

/* ------------------------------------------------------------------ */
/* Main loop                                                           */
/* ------------------------------------------------------------------ */

static void fb_apply_locked(u8 level)
{
	fb_apply_cpu(level);
	fb_apply_gpu(level);
	fb_apply_ddr(level);

	if (level > fb_effective_level)
		fb_stat_level_ups++;
	if (level > fb_stat_max_level)
		fb_stat_max_level = level;

	fb_effective_level = level;
}

static void fb_thermal_check(void)
{
	int temp;

	if (!fb_tz_cpu)
		return;

	if (thermal_zone_get_temp(fb_tz_cpu, &temp))
		return;

	if (!fb_thermal_blocked && temp >= fb_thermal_limit) {
		fb_thermal_blocked = true;
		fb_stat_thermal_blocks++;
		pr_info("thermal guard: %d >= %d, boosting suspended\n",
			temp, fb_thermal_limit);
	}

	if (fb_thermal_blocked && temp <= fb_thermal_clear) {
		fb_thermal_blocked = false;
		pr_info("thermal guard cleared at %d\n", temp);
	}
}

static void fb_work_fn(struct work_struct *work)
{
	u8 level;

	mutex_lock(&fb_lock);

	/* Retry cpufreq policy discovery if the driver was not up yet. */
	if (fb_num_clusters == 0 && fb_discover_retries < FB_DISCOVER_RETRIES) {
		fb_discover_retries++;
		fb_discover_clusters();
	}

	/* Consume an edge reported by the input handler. */
	if (fb_touch_enabled && fb_touch_seen) {
		fb_touch_seen = false;
		fb_stat_touch_boosts++;
		fb_request(FB_TAG_TOUCH, 1, fb_touch_hold_ms);
	}

	fb_thermal_check();

	level = fb_enabled ? fb_compute_level() : 0;
	if (fb_effective_level)
		fb_stat_boosted_ms += fb_last_delay_ms;
	fb_apply_locked(level);

	if (fb_enabled) {
		fb_last_delay_ms = fb_next_delay_ms();
		schedule_delayed_work(&fb_work,
				      msecs_to_jiffies(fb_last_delay_ms));
	}

	mutex_unlock(&fb_lock);
}

/* ------------------------------------------------------------------ */
/* Input edge detector                                                 */
/* ------------------------------------------------------------------ */

static void fb_event(struct input_handle *handle, unsigned int type,
		     unsigned int code, int value)
{
	/* Ignore key release and the finger-up ABS chatter. */
	if (type == EV_KEY && !value)
		return;
	if (type != EV_ABS && type != EV_KEY)
		return;
	if (!READ_ONCE(fb_touch_enabled) || !READ_ONCE(fb_enabled))
		return;
	if (code == ABS_MT_TRACKING_ID && value < 0)
		return;

	WRITE_ONCE(fb_touch_last, jiffies);
	WRITE_ONCE(fb_touch_seen, true);

	/* Atomic context: only nudge the worker. */
	if (!work_pending(&fb_work.work))
		schedule_delayed_work(&fb_work, 0);
}

static const struct input_device_id fb_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{ },
};

static int fb_connect(struct input_handler *handler, struct input_dev *dev,
		      const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "frame_boost";

	error = input_register_handle(handle);
	if (error)
		goto err_free;

	error = input_open_device(handle);
	if (error)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return error;
}

static void fb_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static struct input_handler fb_handler = {
	.event		= fb_event,
	.connect	= fb_connect,
	.disconnect	= fb_disconnect,
	.name		= "frame_boost",
	.id_table	= fb_ids,
};

/* ------------------------------------------------------------------ */
/* Cluster discovery                                                   */
/* ------------------------------------------------------------------ */

void fb_discover_clusters(void)
{
	struct cpufreq_policy *seen[FB_MAX_CLUSTERS];
	int n = 0;
	int cpu, i;

	memset(seen, 0, sizeof(seen));

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy;
		bool dup = false;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		for (i = 0; i < n; i++)
			if (seen[i] == policy)
				dup = true;

		if (dup || n >= FB_MAX_CLUSTERS) {
			cpufreq_cpu_put(policy);
			continue;
		}
		seen[n++] = policy;
	}

	if (!n)
		return;

	fb_num_clusters = n;

	for (i = 0; i < n; i++) {
		struct cpufreq_policy *policy = seen[i];
		struct fb_cpu_slot *slot = &fb_cpus[i];
		int ret;

		slot->valid = true;
		slot->first_cpu = policy->cpu;
		slot->max_freq = policy->cpuinfo.max_freq;
		slot->applied_freq = FREQ_QOS_MIN_DEFAULT_VALUE;

		ret = freq_qos_add_request(&policy->constraints, &slot->qos,
					   FREQ_QOS_MIN,
					   FREQ_QOS_MIN_DEFAULT_VALUE);
		if (ret < 0) {
			pr_warn("freq_qos_add_request failed for cluster %d: %d\n",
				i, ret);
			slot->valid = false;
		} else {
			slot->qos_added = true;
		}

		pr_info("cluster %d: first_cpu=%u max=%u kHz\n",
			i, slot->first_cpu, slot->max_freq);

		cpufreq_cpu_put(policy);
	}
}

static void fb_qos_init(void)
{
	if (!IS_ENABLED(CONFIG_MTK_PMQOS))
		return;

	mtk_pm_qos_add_request(&fb_ddr_req, MTK_PM_QOS_DDR_OPP,
			       MTK_PM_QOS_DDR_OPP_DEFAULT_VALUE);
	mtk_pm_qos_add_request(&fb_vcore_req, MTK_PM_QOS_VCORE_OPP,
			       MTK_PM_QOS_VCORE_OPP_DEFAULT_VALUE);
	fb_qos_added = true;
}

static void fb_thermal_init(void)
{
	fb_tz_cpu = thermal_zone_get_zone_by_name("mtktscpu");
	if (IS_ERR(fb_tz_cpu))
		fb_tz_cpu = NULL;

	if (fb_tz_cpu)
		pr_info("thermal zone 'mtktscpu' registered\n");
	else
		pr_warn("thermal zone 'mtktscpu' not found, guard disabled\n");
}

/* ------------------------------------------------------------------ */
/* sysfs interface                                                     */
/* ------------------------------------------------------------------ */

static ssize_t fb_enable_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", fb_enabled ? 1 : 0);
}

static ssize_t fb_enable_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&fb_lock);
	fb_enabled = !!val;
	if (!fb_enabled) {
		memset(fb_reqs, 0, sizeof(fb_reqs));
		fb_apply_locked(0);
	} else {
		WRITE_ONCE(fb_touch_last, jiffies);
		schedule_delayed_work(&fb_work, 0);
	}
	mutex_unlock(&fb_lock);

	return count;
}

/*
 * Writing "<level> [timeout_ms]" (or just "<level>") raises the boost for the
 * given window.  This is the hook userspace uses.
 */
static ssize_t fb_request_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int level = -1, timeout = 0;

	if (sscanf(buf, "%d %d", &level, &timeout) < 1)
		return -EINVAL;
	if (level < 0 || level > FB_MAX_LEVEL)
		return -EINVAL;
	if (timeout < 0)
		timeout = 0;
	if (!timeout)
		timeout = level ? (int)fb_touch_hold_ms : 1;

	mutex_lock(&fb_lock);
	if (fb_request(FB_TAG_USER, (u8)level, (unsigned int)timeout) >= 0)
		fb_stat_user_boosts++;
	if (fb_enabled)
		schedule_delayed_work(&fb_work, 0);
	mutex_unlock(&fb_lock);

	return count;
}

static ssize_t fb_level_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	u8 level;

	mutex_lock(&fb_lock);
	level = fb_effective_level;
	mutex_unlock(&fb_lock);

	return scnprintf(buf, PAGE_SIZE, "%u %s\n", level, fb_levels[level].name);
}

static ssize_t fb_params_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
			 "poll_ms %u\ngpu_reassert_ms %u\ntouch_hold_ms %u\n"
			 "touch_boost %d\nthermal_limit %d\nthermal_clear %d\n"
			 "thermal_blocked %d\n",
			 fb_poll_ms, fb_gpu_reassert_ms, fb_touch_hold_ms,
			 fb_touch_enabled ? 1 : 0, fb_thermal_limit,
			 fb_thermal_clear, fb_thermal_blocked ? 1 : 0);
}

static ssize_t fb_params_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	char key[32];
	int val;

	if (sscanf(buf, "%31s %d", key, &val) != 2)
		return -EINVAL;

	if (!strcmp(key, "poll_ms") && val >= 10 && val <= 5000)
		fb_poll_ms = val;
	else if (!strcmp(key, "gpu_reassert_ms") && val >= 5 && val <= 1000)
		fb_gpu_reassert_ms = val;
	else if (!strcmp(key, "touch_hold_ms") && val >= 0 && val <= 20000)
		fb_touch_hold_ms = val;
	else if (!strcmp(key, "touch_boost"))
		fb_touch_enabled = !!val;
	else if (!strcmp(key, "thermal_limit"))
		fb_thermal_limit = val;
	else if (!strcmp(key, "thermal_clear"))
		fb_thermal_clear = val;
	else
		return -EINVAL;

	return count;
}

static ssize_t fb_stats_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	unsigned int gpu_ready = mt_gpufreq_not_ready() ? 0 : 1;
	int len = 0, i;

	mutex_lock(&fb_lock);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "level_ups %llu\ntouch_boosts %llu\nuser_boosts %llu\n"
			 "thermal_blocks %llu\nboosted_ms %llu\n"
			 "max_level %u\neffective_level %u\n"
			 "gpu_opp %u\ndvfs_table %u\nclusters %d\n",
			 fb_stat_level_ups, fb_stat_touch_boosts,
			 fb_stat_user_boosts, fb_stat_thermal_blocks,
			 fb_stat_boosted_ms, fb_stat_max_level,
			 fb_effective_level, gpu_ready,
			 gpu_ready ? mt_gpufreq_get_dvfs_table_num() : 0,
			 fb_num_clusters);

	for (i = 0; i < fb_num_clusters; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "cluster%d max=%u applied_floor=%u\n",
				 i, fb_cpus[i].max_freq,
				 fb_cpus[i].applied_freq);
	mutex_unlock(&fb_lock);

	return len;
}

static struct kobj_attribute fb_attr_enable =
	__ATTR(enable, 0644, fb_enable_show, fb_enable_store);
static struct kobj_attribute fb_attr_request =
	__ATTR(request, 0200, NULL, fb_request_store);
static struct kobj_attribute fb_attr_level =
	__ATTR(level, 0444, fb_level_show, NULL);
static struct kobj_attribute fb_attr_params =
	__ATTR(params, 0644, fb_params_show, fb_params_store);
static struct kobj_attribute fb_attr_stats =
	__ATTR(stats, 0444, fb_stats_show, NULL);

static struct attribute *fb_attrs[] = {
	&fb_attr_enable.attr,
	&fb_attr_request.attr,
	&fb_attr_level.attr,
	&fb_attr_params.attr,
	&fb_attr_stats.attr,
	NULL,
};

static struct attribute_group fb_attr_group = {
	.attrs = fb_attrs,
};

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

static int __init fb_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&fb_work, fb_work_fn);

	fb_qos_init();
	fb_discover_clusters();

	if (fb_num_clusters == 0)
		pr_warn("no cpufreq policies yet, will retry from the worker\n");

	fb_thermal_init();

	fb_kobj = kobject_create_and_add("frame_boost", kernel_kobj);
	if (!fb_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(fb_kobj, &fb_attr_group);
	if (ret) {
		kobject_put(fb_kobj);
		fb_kobj = NULL;
		return ret;
	}

	ret = input_register_handler(&fb_handler);
	if (ret)
		pr_warn("input handler registration failed: %d\n", ret);

	schedule_delayed_work(&fb_work, msecs_to_jiffies(fb_poll_ms));

	pr_info("initialised: %d clusters, touch boost %s, gpu %s\n",
		fb_num_clusters, fb_touch_enabled ? "on" : "off",
		mt_gpufreq_not_ready() ? "not ready" : "ready");

	return 0;
}
late_initcall(fb_init);
