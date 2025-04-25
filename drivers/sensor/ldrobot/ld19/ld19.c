/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ldrobot_ld19

#include <zephyr/init.h>
#include <zephyr/kernel.h>

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
#define LD19_DATA_LEN             12 * 3 + 11
#define LD19_DATA_HEALTH_LEN      12
#define LD19_DATA_MANUFACTURE_LEN 12

static const uint8_t crc_table[256] = {
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11,
	0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f,
	0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94,
	0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
	0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56,
	0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28,
	0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3,
	0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
	0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f,
	0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1,
	0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a,
	0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
	0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8,
	0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6,
	0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d,
	0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
	0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5,
	0xa8};

enum ld19_state {
	HEADER,
	VER_LEN,
	DATA,
	DATA_HEALTH,
	DATA_MANUFACTURE,
};

struct ld19_config {
	const struct device *uart_dev;
#ifdef CONFIG_LD19_TRIGGER
	sensor_trigger_handler_t handler;
	const struct sensor_trigger *trig;
#endif
};

struct ld19_buf {
	uint8_t buf[LD19_MAX_BUFFER_SIZE];
	uint8_t index;
};

typedef struct __attribute__((packed)) {
	uint16_t distance;
	uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructType point[12];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
} LiDARMeasureDataType;

static LiDARMeasureDataType ld19_data;

struct ld19_data {
	struct ld19_buf rx_buf;
	enum ld19_state state;
};

static uint8_t ld19_crc8(const uint8_t *data, uint8_t data_len)
{
	uint8_t crc = 0;
	while (data_len--) {
		crc = crc_table[(crc ^ *data) & 0xff];
		data++;
	}
	return crc;
}

static void ld19_uart_callback_handler(const struct device *dev, void *user_data)
{
	struct ld19_data *data = ((struct device *)user_data)->data;
	const struct ld19_config *config = ((struct device *)user_data)->config;
	int len = 0;
	uint8_t c;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) < 0)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		len = uart_fifo_read(dev, &c, 1);

		if (len < 0) {
			LOG_ERR("Failed to read data from UART");
			return;
		}

		switch (data->state) {
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
			if (data->rx_buf.index >= LD19_DATA_LEN) {
				uint8_t crc = ld19_crc8(data->rx_buf.buf, LD19_DATA_LEN - 1);
				if (crc == data->rx_buf.buf[LD19_DATA_LEN - 1]) {
					memcpy((uint8_t *)&ld19_data, data->rx_buf.buf,
					       LD19_DATA_LEN);
#ifdef CONFIG_LD19_TRIGGER
					if (config->handler) {
						config->handler(dev, config->trig);
					}
#endif
				}

				data->state = HEADER;
			}
			break;
		case DATA_HEALTH:
			data->rx_buf.buf[data->rx_buf.index++] = c;
			if (data->rx_buf.index == LD19_DATA_HEALTH_LEN) {
				uint8_t crc = ld19_crc8(data->rx_buf.buf, LD19_DATA_HEALTH_LEN - 1);
				crc = 0; // unused for now
				data->state = HEADER;
			}
			break;
		case DATA_MANUFACTURE:
			data->rx_buf.buf[data->rx_buf.index++] = c;
			if (data->rx_buf.index == LD19_DATA_MANUFACTURE_LEN) {
				uint8_t crc =
					ld19_crc8(data->rx_buf.buf, LD19_DATA_MANUFACTURE_LEN - 1);
				crc = 0; // unused for now
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
	return 0;
}

static int ld19_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_DISTANCE:
		val->val1 = ld19_data.speed;
		val->val2 = ld19_data.timestamp;
		val++;
		val->val1 = ld19_data.start_angle;
		val->val2 = ld19_data.end_angle;
		val++;
		for (int i = 0; i < 12; i++) {
			val->val1 = ld19_data.point[i].distance;
			val->val2 = ld19_data.point[i].intensity;
			val++;
		}
		break;
	default:
		return -ENOTSUP;
		break;
	}

	return 0;
}

#ifdef CONFIG_LD19_TRIGGER
int ld19_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		     sensor_trigger_handler_t handler)
{
	struct ld19_config *config = (struct ld19_config *)dev->config;

	config->handler = handler;
	config->trig = trig;

	return 0;
}
#endif

static const struct sensor_driver_api ld19_driver_api = {
	.attr_set = ld19_attr_set,
	.sample_fetch = ld19_sample_fetch,
	.channel_get = ld19_channel_get,
#ifdef CONFIG_LD19_TRIGGER
	.trigger_set = ld19_trigger_set,
#endif
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

	if (!device_is_ready(config->uart_dev)) {
		LOG_ERR("UART device %s is not ready", config->uart_dev->name);
		return -ENODEV;
	}

	ld19_uart_flush(config->uart_dev);

	uart_irq_callback_user_data_set(config->uart_dev, ld19_uart_callback_handler, (void *)dev);

	uart_irq_rx_enable(config->uart_dev);

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
