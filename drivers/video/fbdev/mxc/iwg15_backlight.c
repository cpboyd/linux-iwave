/*
 * Copyright (c) 2014-2015 iWave Systems Technologies Pvt. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
 
/*
 * @file iwg15_backlight.c
 *
 * @brief Simple driver to control the LVDS port power and backlight
 *
 * @ingroup MXC
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

struct imx6_iwg15_backlight {
        int                     power_on_gpio;
        int                     backlght_on_gpio;
};

static int backlight_ctrl_state (struct imx6_iwg15_backlight *iwg15_backlight)
{
	int ret;
        if (gpio_is_valid(iwg15_backlight->backlght_on_gpio))
                ret = gpio_get_value(iwg15_backlight->backlght_on_gpio);
	return ret;
}

static void backlight_ctrl (struct imx6_iwg15_backlight *iwg15_backlight, int val)
{
	if (gpio_is_valid(iwg15_backlight->backlght_on_gpio))
		gpio_set_value(iwg15_backlight->backlght_on_gpio, val);
}

static int power_ctrl_state (struct imx6_iwg15_backlight *iwg15_backlight)
{
	int ret;
        if (gpio_is_valid(iwg15_backlight->power_on_gpio))
                ret = gpio_get_value(iwg15_backlight->power_on_gpio);
	return ret;

}

static void power_ctrl (struct imx6_iwg15_backlight *iwg15_backlight, int val)
{
	if (gpio_is_valid(iwg15_backlight->power_on_gpio)) 
		gpio_set_value(iwg15_backlight->power_on_gpio, val);

}

static ssize_t iwg15_backlight_ctrl_state(struct device *dev,
                struct device_attribute *attr, const char *buf)
{
        struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(dev);
        int state;

        state = backlight_ctrl_state(iwg15_backlight);

	return sprintf(buf, "%d\n", state);
}

static ssize_t iwg15_backlight_ctrl(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(dev);
        unsigned long value;
        int ret = 0;

        ret = kstrtoul(buf, 2, &value);
        if (ret)
                return ret;

	backlight_ctrl(iwg15_backlight, value);

        return count;
}

static DEVICE_ATTR(bklight_enable, S_IWUSR | S_IRUGO, iwg15_backlight_ctrl_state, iwg15_backlight_ctrl);

static ssize_t iwg15_backlight_power_ctrl_state(struct device *dev,
                struct device_attribute *attr, const char *buf)
{
        struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(dev);
        int state;

        state = power_ctrl_state(iwg15_backlight);

	return sprintf(buf, "%d\n", state);
}

static ssize_t iwg15_backlight_power_ctrl(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(dev);
        unsigned long value;
        int ret = 0;

        ret = kstrtoul(buf, 2, &value);
        if (ret)
                return ret;

	power_ctrl(iwg15_backlight, value);

        return count;
}

static DEVICE_ATTR(pwr_enable, S_IWUSR | S_IRUGO, iwg15_backlight_power_ctrl_state, iwg15_backlight_power_ctrl);

#ifdef CONFIG_PM
static int iwg15_backlight_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(&pdev->dev);

	backlight_ctrl(iwg15_backlight, 0);
	power_ctrl(iwg15_backlight, 0);

	return 0;
}

static int iwg15_backlight_resume(struct platform_device *pdev)
{
	struct imx6_iwg15_backlight *iwg15_backlight = dev_get_drvdata(&pdev->dev);

	backlight_ctrl(iwg15_backlight, 1);
	power_ctrl(iwg15_backlight, 1);

	return 0;
}
#endif

/* 
 * iwg15_backlight_probe - Probe method for the LVDS contolling GPIOs.
 * @np: pointer to device tree node
 *
 * This function probes the RainboW G15 control GPIOs in the device tree. It request GPIOs
 * as outptu. It returns 0, if the driver registered
 * or a negative value if there is an error.
 */
static int iwg15_backlight_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct imx6_iwg15_backlight *imx6_iwg15_backlight;
	int ret=0;

	imx6_iwg15_backlight = devm_kzalloc(&pdev->dev, sizeof(*imx6_iwg15_backlight), GFP_KERNEL);
	if (!imx6_iwg15_backlight)
		return -ENOMEM;

	/* Fetch GPIOs */
	imx6_iwg15_backlight->backlght_on_gpio = of_get_named_gpio(np, "backlgt-gpios", 0);
	if (gpio_is_valid(imx6_iwg15_backlight->backlght_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
				imx6_iwg15_backlight->backlght_on_gpio,
				GPIOF_OUT_INIT_HIGH,
				"lvds bklight");
		if (ret) {
			dev_err(&pdev->dev, "unable to get backlight gpio\n");
			goto err;
		}
	}

	imx6_iwg15_backlight->power_on_gpio = of_get_named_gpio(np, "poweron-gpios", 0);
	if (gpio_is_valid(imx6_iwg15_backlight->power_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
				imx6_iwg15_backlight->power_on_gpio,
				GPIOF_OUT_INIT_HIGH,
				"lvds power enable");
		if (ret) {
			dev_err(&pdev->dev, "unable to get power-on gpio\n");
			goto err;
		}
	}

	platform_set_drvdata(pdev, imx6_iwg15_backlight);

	/* Create the device attr */
	ret = device_create_file(&pdev->dev, &dev_attr_bklight_enable);
	if (ret < 0)
		dev_warn(&pdev->dev,
				"cound not create sys node for backlight state\n");

	ret = device_create_file(&pdev->dev, &dev_attr_pwr_enable);
	if (ret < 0)
		dev_warn(&pdev->dev,
				"cound not create sys node for power state\n");

err:
	return ret;
}

static int iwg15_backlight_remove(struct platform_device *pdev)
{
	/* Platform not registerd return silently */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id iwg15_backlight_match[] = {
	{.compatible = "iwave,iwg15-backlight"},
	{}
};
MODULE_DEVICE_TABLE(of, iwg15_backlight_match);
#else
#define iwg15_backlight_match NULL
#endif

static struct platform_driver iwg15_backlight_driver = {
	.driver = {
		.name   = "iwg15-backlight",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(iwg15_backlight_match),
	},
	.probe          = iwg15_backlight_probe,
	.remove         = iwg15_backlight_remove,
        .suspend 	= iwg15_backlight_suspend,
        .resume 	= iwg15_backlight_resume,
};

static int __init iwg15_backlight_init(void)
{
	return platform_driver_register(&iwg15_backlight_driver);
}
module_init(iwg15_backlight_init);

static void __exit iwg15_backlight_exit(void)
{
	platform_driver_unregister(&iwg15_backlight_driver);
}
module_exit(iwg15_backlight_exit);

MODULE_AUTHOR("iWave Systems Technologies Pvt.Ltd");
MODULE_DESCRIPTION("iWave LVDS power and backlight Driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL v2");
