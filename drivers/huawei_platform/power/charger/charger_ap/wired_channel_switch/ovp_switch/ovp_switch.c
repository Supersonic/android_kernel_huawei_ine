#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <huawei_platform/log/hw_log.h>
#include <huawei_platform/power/wired_channel_switch.h>

static int use_ovp_cutoff_wired_channel = 0;
static int gpio_ovp_chsw_en = 0;
static int ovp_gpio_initialized = 0;

#define HWLOG_TAG ovp_channel_switch
HWLOG_REGIST();

static int ovp_chsw_set_wired_channel(int flag)
{
	int ret = 0;
	if (!ovp_gpio_initialized) {
		hwlog_err("%s: ovp channel switch not initialized!\n", __func__);
		return -ENODEV;
	}

	if (WIRED_CHANNEL_CUTOFF == flag) {
		hwlog_info("%s set ovp en high\n", __func__);
		ret = gpio_direction_output(gpio_ovp_chsw_en, 1);  //cutoff
	} else {
		hwlog_info("%s set ovp en low\n", __func__);
		ret = gpio_direction_input(gpio_ovp_chsw_en);  //restore
	}

	return ret;
}
static struct wired_chsw_device_ops chsw_ops = {
	.set_wired_channel = ovp_chsw_set_wired_channel,
};
static void ovp_chsw_parse_dts(struct device_node *np)
{
	int ret = 0;
	ret = of_property_read_u32(of_find_compatible_node(NULL, NULL, "huawei,wired_channel_switch"),
			"use_ovp_cutoff_wired_channel", &use_ovp_cutoff_wired_channel);
	if (ret) {
		hwlog_err("%s: get use_ovp_cutoff_wired_channel failed\n", __func__);
		use_ovp_cutoff_wired_channel = 0;
	}
	hwlog_info("%s: use_ovp_cutoff_wired_channel  = %d.\n", __func__, use_ovp_cutoff_wired_channel);
}
static int ovp_chsw_gpio_init(struct device_node *np)
{
	int ret = 0;
	gpio_ovp_chsw_en = of_get_named_gpio(np, "gpio_ovp_chsw_en", 0);
	hwlog_info("ovp switch gpio_ovp_chsw_en = %d\n", gpio_ovp_chsw_en);
	if (!gpio_is_valid(gpio_ovp_chsw_en)) {
		hwlog_err("gpio_ovp_chsw_en is not valid\n");
		return -EINVAL;
	}
	if (gpio_request(gpio_ovp_chsw_en, "gpio_ovp_chsw_en")) {
		hwlog_err("could not request gpio_ovp_chsw_en\n");
		return  -ENOMEM;
	}
	ret = gpio_direction_input(gpio_ovp_chsw_en);/*avoid ovp_en to hiz mode*/
	if (ret) {
		hwlog_err("could not set gpio_ovp_chsw_en input\n");
		gpio_free(gpio_ovp_chsw_en);
		return -1;
	}
	ovp_gpio_initialized = 1;
	return 0;
}

static int ovp_chsw_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = (&pdev->dev)->of_node;

	ovp_chsw_parse_dts(np);
	if (use_ovp_cutoff_wired_channel) {
		ret = ovp_chsw_gpio_init(np);
		if (ret) {
			hwlog_info("%s ovp channel switch gpio init fail\n", __func__);
			return -1;
		}
		ret = wired_chsw_ops_register(&chsw_ops);
		if (ret) {
			hwlog_err("register ovp switch ops failed!\n");
			gpio_free(gpio_ovp_chsw_en);
			return -1;
		}
		hwlog_info("%s ovp channel switch ops register success\n", __func__);
	}

	hwlog_info("ovp_chsw probe ok.\n");
	return 0;
}

static int ovp_chsw_remove(struct platform_device *pdev)
{
	hwlog_info("%s ++\n", __func__);
	if(!gpio_is_valid(gpio_ovp_chsw_en))
		gpio_free(gpio_ovp_chsw_en);
	hwlog_info("%s --\n", __func__);
	return 0;
}

static struct of_device_id ovp_chsw_match_table[] = {
	{
	 .compatible = "huawei,ovp_channel_switch",
	 .data = NULL,
	},
	{},
};

static struct platform_driver ovp_chsw_driver = {
	.probe = ovp_chsw_probe,
	.remove = ovp_chsw_remove,
	.driver = {
		.name = "huawei,ovp_channel_switch",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ovp_chsw_match_table),
	},
};
static int __init ovp_chsw_init(void)
{
	hwlog_info("ovp_chsw init ok.\n");

	return platform_driver_register(&ovp_chsw_driver);
}
static void __exit ovp_chsw_exit(void)
{
	platform_driver_unregister(&ovp_chsw_driver);
}

fs_initcall_sync(ovp_chsw_init);
module_exit(ovp_chsw_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ovp switch module driver");
MODULE_AUTHOR("HUAWEI Inc");
