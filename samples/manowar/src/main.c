/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/dt-bindings/pwm/pwm.h"
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(main);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec motor0_enable =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, motor0_enable_gpios);
static const struct gpio_dt_spec motor0_direction =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, motor0_direction_gpios);
static const struct gpio_dt_spec motors_enable_step_input =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, motors_enable_step_input_gpios);

struct gpio {
	const char *name;
	const struct gpio_dt_spec *spec;
};

static struct gpio gpios[] = {
	{.name = "User LED", .spec = &led},
	{.name = "Motor0 enable", .spec = &motor0_enable},
	{.name = "Motor0 direction", .spec = &motor0_direction},
	{.name = "Motors step input enable", .spec = &motors_enable_step_input}};

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));

static void init_gpios(void)
{
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(gpios); ++i) {
		if (!gpio_is_ready_dt(gpios[i].spec)) {
			LOG_ERR("GPIO pin %s is not ready!", gpios[i].name);
			continue;
		}
		if ((ret = gpio_pin_configure_dt(gpios[i].spec, GPIO_OUTPUT_INACTIVE)) < 0) {
			LOG_ERR("Error configuring GPIO pin %s: %d", gpios[i].name, ret);
		}
	}

	gpio_pin_set_dt(&motors_enable_step_input, 1);
}

int main(void)
{
	int ret;

	if ((ret = usb_enable(NULL)) < 0) {
		return ret;
	}

	init_gpios();

	if (!pwm_is_ready_dt(&pwm_led0)) {
		LOG_ERR("Error: PWM device %s is not ready\n", pwm_led0.dev->name);
	}

	if ((ret = pwm_set_dt(&pwm_led0, PWM_SEC(2), PWM_SEC(1))) < 0) {
		LOG_ERR("Error setting period for pwm_led0!");
	}

	for (size_t i = 0;; i++) {
		LOG_INF("%6zu: Hello, world!", i);
		gpio_pin_toggle_dt(&led);
		gpio_pin_toggle_dt(&motor0_enable);
		gpio_pin_toggle_dt(&motor0_direction);
		k_sleep(K_SECONDS(5));
	}
}
