// SPDX-License-Identifier: GPL-2.0
/*
 * mtk_boot_sysfs.c - minimal MediaTek boot info sysfs for CIP-based ares_4.19
 *
 * Problem
 * =======
 * Official MTK kernels expose /sys/class/BOOT/BOOT/boot/boot_type through the
 * MediaTek "boot" driver.  This kernel tree (4.19.325-cip + MTK driver drop)
 * does NOT ship that driver, so the mTEE tee-supplicant cannot detect the
 * boot storage device and logs:
 *
 *   tee-supplicant: ERR get_boot_type: fail to open:
 *                   /sys/class/BOOT/BOOT/boot/boot_type
 *   tee-supplicant: ERR mmc_rpmb_fd: Could not open /dev/mmcblk0rpmb
 *                   (No such file or directory)
 *
 * On UFS-only devices (mt6893 / ares / Redmi K40 Gaming) the supplicant
 * falls back to the eMMC RPMB path, the TA RPMB service never comes up, and
 * keymaster/gatekeeper (mitee, tee_type=1) fail with:
 *
 *   MiteeKeymaster_ipc: TEEC_Opensession failed with code 0xffff3024 origin 0x3
 *
 * This driver recreates the boot_type attribute (BOOTDEV_UFS = 2 on ares) so
 * the supplicant selects the UFS RPMB path /dev/rpmb0 (rpmb-mtk backend,
 * which already exists on this kernel - no config change needed there).
 *
 * Installation (kernel side)
 * ==========================
 * 1) cp this file to drivers/misc/mediatek/mboot_params/mtk_boot_sysfs.c
 * 2) append to drivers/misc/mediatek/mboot_params/Makefile:
 *        obj-y += mtk_boot_sysfs.o
 *    (mboot_params/ is already built via obj-$(CONFIG_ARCH_MEDIATEK) in
 *     drivers/misc/mediatek/Makefile line ~193)
 * 3) rebuild: make O=out ares_defconfig && make O=out -j$(nproc)
 * 4) verify after boot (adb shell / TWRP terminal):
 *        cat /sys/class/BOOT/BOOT/boot/boot_type    # expect: 2
 *        ls -l /dev/rpmb0                           # expect: present
 *     then logcat should show keymaster/gatekeeper registering instead of
 *     TEEC_Opensession 0xffff3024.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/kdev_t.h>

#define BOOT_DEV_NAME		"BOOT"
#define BOOT_SYSFS		"boot"
#define BOOT_TYPE_SYSFS_ATTR	"boot_type"

#define BOOTDEV_NAND		(0)
#define BOOTDEV_SDMMC		(1)
#define BOOTDEV_UFS		(2)

/* matches struct tag_bootmode used by ufs-mediatek / lk atags */
struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

static unsigned int sysfs_boot_type = BOOTDEV_UFS;

static unsigned int mtk_get_boot_type(void)
{
	struct tag_bootmode *tags = NULL;
	struct device_node *node = NULL;
	unsigned long size = 0;
	int ret = BOOTDEV_UFS; /* ares (mt6893) is UFS-only */

	node = of_find_node_by_path("/chosen");
	if (!node)
		node = of_find_node_by_path("/chosen@0");
	if (node) {
		tags = (struct tag_bootmode *)of_get_property(node,
				"atag,boot", (int *)&size);
		of_node_put(node);
	}

	if (tags) {
		ret = tags->boottype;
		if ((ret > BOOTDEV_UFS) || (ret < BOOTDEV_NAND))
			ret = BOOTDEV_SDMMC;
	}

	return ret;
}

static ssize_t boot_type_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sysfs_boot_type);
}

static struct kobj_attribute boot_type_attr =
	__ATTR(boot_type, 0444, boot_type_show, NULL);

static struct attribute *boot_sysfs_attrs[] = {
	&boot_type_attr.attr,
	NULL,
};

static struct attribute_group boot_sysfs_group = {
	.name = BOOT_SYSFS,
	.attrs = boot_sysfs_attrs,
};

static struct class *boot_class;
static struct device *boot_dev;

static int __init mtk_boot_sysfs_init(void)
{
	int ret;

	sysfs_boot_type = mtk_get_boot_type();

	boot_class = class_create(THIS_MODULE, BOOT_DEV_NAME);
	if (IS_ERR(boot_class))
		return PTR_ERR(boot_class);

	boot_dev = device_create(boot_class, NULL, MKDEV(0, 0), NULL,
				 BOOT_DEV_NAME);
	if (IS_ERR(boot_dev)) {
		ret = PTR_ERR(boot_dev);
		goto err_class;
	}

	ret = sysfs_create_group(&boot_dev->kobj, &boot_sysfs_group);
	if (ret)
		goto err_dev;

	pr_info("mtk_boot_sysfs: /sys/class/%s/%s/%s/%s = %u\n",
		BOOT_DEV_NAME, BOOT_DEV_NAME, BOOT_SYSFS,
		BOOT_TYPE_SYSFS_ATTR, sysfs_boot_type);

	return 0;

err_dev:
	device_destroy(boot_class, MKDEV(0, 0));
err_class:
	class_destroy(boot_class);
	return ret;
}
device_initcall(mtk_boot_sysfs_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek boot info sysfs (boot_type for mTEE tee-supplicant)");
