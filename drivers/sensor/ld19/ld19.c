/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ldrobot_ld19

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "ld19.h"

LOG_MODULE_REGISTER(LD19, CONFIG_SENSOR_LOG_LEVEL);

struct ld19_config {
};

struct ld19_data {
};

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

static int ld19_init(const struct device *dev)
{
	const struct ld19_config *config = dev->config;
	struct ld19_data *data = dev->data;

	return 0;
}

static const struct sensor_driver_api ld19_driver_api = {
	.attr_set = ld19_attr_set,
	.sample_fetch = ld19_sample_fetch,
	.channel_get = ld19_channel_get,
};

#define LD19_INIT(n)                                                                             \
	static struct ld19_config ld19_config_##n = {                                             \
	};                                                                                         \
	static struct ld19_data ld19_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, ld19_init, NULL, &ld19_data_##n, &ld19_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ld19_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LD19_INIT)
