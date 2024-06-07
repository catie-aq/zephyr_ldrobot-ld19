/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ldrobot_ld19

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include "ld19.h"

LOG_MODULE_REGISTER(LD19, CONFIG_SENSOR_LOG_LEVEL);

struct ld19_config {
	const struct device *uart_dev;
};

struct ld19_data {
};

static void ld19_uart_callback_handler(const struct device *dev, void *user_data)
{
}

static int ld19_attr_set(const struct device *dev, enum sensor_channel chan,
			 enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int ld19_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ld19_data *data = dev->data;
	const struct ld19_config *config = dev->config;

	return 0;
}

static int ld19_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	struct ld19_data *data = dev->data;

	// TODO: Update val with the sensor value
	val->val1 = 0;
	val->val2 = 0;

	return 0;
}

static const struct sensor_driver_api ld19_driver_api = {
	.attr_set = ld19_attr_set,
	.sample_fetch = ld19_sample_fetch,
	.channel_get = ld19_channel_get,
};

static void ld19_uart_flush(const struct device *dev)
{
	uint8_t c;

	while (uart_fifo_read(dev, &c, 1)) {
		continue;
	}
}

static int ld19_init(const struct device *dev)
{
	const struct ld19_config *config = dev->config;
	struct ld19_data *data = dev->data;

	if (!device_is_ready(config->uart_dev)) {
		LOG_ERR("UART device %s is not ready", config->uart_dev->name);
		return -ENODEV;
	}

	ld19_uart_flush(config->uart_dev);

	uart_irq_callback_user_data_set(config->uart_dev, ld19_uart_callback_handler, dev);

	uart_irq_rx_disable(config->uart_dev);
	uart_irq_tx_disable(config->uart_dev);

	return 0;
}

#define LD19_INIT(n)                                                                               \
	static struct ld19_data ld19_data_##n;                                                     \
                                                                                                   \
	static struct ld19_config ld19_config_##n = {                                              \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                         \
	};                                                                                         \
	static struct ld19_data ld19_data_##n;                                                     \
	DEVICE_DT_INST_DEFINE(n, ld19_init, NULL, &ld19_data_##n, &ld19_config_##n, POST_KERNEL,   \
			      CONFIG_SENSOR_INIT_PRIORITY, &ld19_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LD19_INIT)
