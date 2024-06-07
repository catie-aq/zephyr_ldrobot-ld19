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

#define LD19_MAX_BUFFER_SIZE      128
#define LD19_PKG_HEADER           0x54
#define LD19_DATA_PKG_INFO        0x2C
#define LD19_HEALTH_PKG_INFO      0xE0
#define LD19_MANUFACT_PKG_INF     0x0F
#define LD19_DATA_LEN             12
#define LD19_DATA_HEALTH_LEN      12
#define LD19_DATA_MANUFACTURE_LEN 12

enum ld19_state {
	HEADER,
	VER_LEN,
	DATA,
	DATA_HEALTH,
	DATA_MANUFACTURE,
};

struct ld19_config {
	const struct device *uart_dev;
};

struct ld19_buf {
	uint8_t buf[LD19_MAX_BUFFER_SIZE];
	uint8_t index;
};

struct ld19_data {
	struct ld19_buf rx_buf;
	enum ld19_state state;
};

static void ld19_uart_callback_handler(const struct device *dev, void *user_data)
{
	struct ld19_data *data = ((struct device *)user_data)->data;
	int len = 0;
	uint8_t c;

	if ((uart_irq_update(dev) & UART_IRQ_RX_RDY) == 0) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		len = uart_fifo_read(dev, &c, 1);
		if (len < 0) {
			LOG_ERR("Failed to read data from UART");
			return;
		}

		swicth(state)
		{
		case HEADER:
			if (c == LD19_PKG_HEADER) {
				data->rx_buf.index = 0;
				data->rx_buf.buf[data->rx_buf.index++] = c;
				data->state = VER_LEN;
			}
			break;
		case VER_LEN: {
			if (c == LD19_DATA_PKG_INFO) {
				data->rx_buf.buf[data->rx_buf.index++] = c;
				data->state = DATA;
			} else if (c == LD19_HEALTH_PKG_INFO) {
				data->rx_buf.buf[data->rx_buf.index++] = c;
				data->state = DATA_HEALTH;
			} else if (c == LD19_MANUFACT_PKG_INF) {
				data->rx_buf.buf[data->rx_buf.index++] = c;
				data->state = DATA_MANUFACTURE;
			} else {
				data->state = HEADER;
			}
			break;
		}
		case DATA:
			data->rx_buf.buf[data->rx_buf.index++] = c;
			if (data->rx_buf.index == LD19_POINT_PER_PACK) {
				// TODO: CRC check
				data->state = HEADER;
			}
			break;
		case DATA_HEALTH:
			data->rx_buf.buf[data->rx_buf.index++] = c;
			if (data->rx_buf.index == LD19_DATA_HEALTH_LEN) {
				// TODO: CRC check
				data->state = HEADER;
			}
			break;
		case DATA_MANUFACTURE:
			data->rx_buf.buf[data->rx_buf.index++] = c;
			if (data->rx_buf.index == LD19_DATA_MANUFACTURE_LEN) {
				// TODO: CRC check
				data->state = HEADER;
			}
			break;
		default:
			data->state = HEADER;
			break;
		}
	}
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
