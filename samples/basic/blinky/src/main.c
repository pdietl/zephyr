/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "config.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct gpio_dt_spec out1_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, output1_gpios);
const struct gpio_dt_spec out2_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, output2_gpios);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

#ifdef MANUAL_CLOCKING
const struct gpio_dt_spec sr_enable_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, sr_enable_gpios);
const struct gpio_dt_spec sr_clock_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, sr_clock_gpios);
const struct gpio_dt_spec sr_data_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, sr_data_gpios);
const struct gpio_dt_spec sr_load_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, sr_load_gpios);

void shift_reg_write(uint32_t val)
{
	for (uint32_t i = 0; i < 32; ++i) {
		gpio_pin_set_dt(&sr_data_gpio, val & (1 << (31 - i)));
		gpio_pin_set_dt(&sr_clock_gpio, 1);
		gpio_pin_set_dt(&sr_clock_gpio, 0);
	}

	gpio_pin_set_dt(&sr_load_gpio, 1);
	gpio_pin_set_dt(&sr_load_gpio, 0);
}

#else

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#endif

static const struct gpio_dt_spec led_comport = GPIO_DT_SPEC_GET(DT_NODELABEL(led_comport1), gpios);

int main(void)
{
	int ret;
	bool led_state = true;
#ifdef MANUAL_CLOCKING
	if (!gpio_is_ready_dt(&sr_enable_gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&sr_clock_gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&sr_data_gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&sr_load_gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&led_comport)) {
		return 0;
	}

#else
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
#endif
	if (!gpio_is_ready_dt(&out1_gpio)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&out2_gpio)) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&led_comport, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

#ifdef MANUAL_CLOCKING
	ret = gpio_pin_configure_dt(&sr_enable_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&sr_data_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&sr_clock_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&sr_load_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}
#else
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}
#endif
	ret = gpio_pin_configure_dt(&out1_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&out2_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	/*
#ifdef MANUAL_CLOCKING

	while (2) {
		k_msleep(2000);
		shift_reg_write(0xff);
		k_msleep(2000);
		shift_reg_write(0);
	}
#else
	while (1) {
		k_msleep(2000);
		gpio_port_set_bits(led.port, 0xff);
		k_msleep(2000);
		gpio_port_clear_bits(led.port, 0xff);
	}
#endif
*/

	for (unsigned i = 0;; i++) {
#ifndef MANUAL_CLOCKING
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
#endif
		ret = gpio_pin_toggle_dt(&led_comport);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_toggle_dt(&out1_gpio);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_toggle_dt(&out2_gpio);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\r\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
