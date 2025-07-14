// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pamir AI Signal Aggregation Module (SAM) Power Manager
 *
 * Power management functionality.
 *
 * Copyright (C) 2025 Pamir AI Incorporated - http://www.pamir.ai/
 */
#include "pamir-sam.h"

/**
 * process_power_packet() - Process power management packet
 * @priv: Private driver data
 * @packet: Received packet
 *
 * Handle power state reports and metrics from the RP2040.
 */
void process_power_packet(struct sam_protocol_data *priv,
			  const struct sam_protocol_packet *packet)
{
	uint8_t cmd = packet->type_flags & POWER_CMD_MASK;
	uint8_t param = packet->type_flags & 0x0F;
	uint8_t data1 = packet->data[0];
	uint8_t data2 = packet->data[1];
	uint16_t value;

	dev_dbg(&priv->serdev->dev,
	 "Power packet - Cmd: 0x%02x, Param: 0x%02x, Data: 0x%02x 0x%02x\n", 
	 cmd, param, data1, data2);

	switch (cmd) {
	case POWER_CMD_QUERY:
		/* RP2040 is querying power status */
		{
			struct sam_protocol_packet response;

			response.type_flags = TYPE_POWER | POWER_CMD_QUERY;
			response.data[0] = 0x01; /* Power state - 1: Running */
			response.data[1] = 0x00; /* Reserved */
			send_packet(priv, &response);
		}
		break;

	case POWER_CMD_CURRENT:
		/* Current draw report from RP2040 */
		value = data1 | (data2 << 8); /* Little-endian */
		mutex_lock(&priv->power_metrics_mutex);
		priv->power_metrics.current_ma = value;
		priv->power_metrics.last_update = jiffies;
		priv->power_metrics.metrics_valid = true;
		mutex_unlock(&priv->power_metrics_mutex);
		dev_dbg(&priv->serdev->dev, "Power metrics: Current = %u mA\n", value);
		break;

	case POWER_CMD_BATTERY:
		/* Battery percentage report from RP2040 */
		value = data1 | (data2 << 8);
		mutex_lock(&priv->power_metrics_mutex);
		priv->power_metrics.battery_percent = min_t(uint16_t, value, 100);
		priv->power_metrics.last_update = jiffies;
		priv->power_metrics.metrics_valid = true;
		mutex_unlock(&priv->power_metrics_mutex);
		dev_dbg(&priv->serdev->dev, "Power metrics: Battery = %u%%\n", 
			priv->power_metrics.battery_percent);
		break;

	case POWER_CMD_TEMP:
		/* Temperature report from RP2040 (signed, deci-celsius) */
		value = data1 | (data2 << 8);
		mutex_lock(&priv->power_metrics_mutex);
		priv->power_metrics.temperature_dc = (int16_t)value;
		priv->power_metrics.last_update = jiffies;
		priv->power_metrics.metrics_valid = true;
		mutex_unlock(&priv->power_metrics_mutex);
		dev_dbg(&priv->serdev->dev, "Power metrics: Temperature = %d.%d°C\n", 
			priv->power_metrics.temperature_dc / 10, 
			abs(priv->power_metrics.temperature_dc % 10));
		break;

	case POWER_CMD_VOLTAGE:
		/* Voltage report from RP2040 */
		value = data1 | (data2 << 8);
		mutex_lock(&priv->power_metrics_mutex);
		priv->power_metrics.voltage_mv = value;
		priv->power_metrics.last_update = jiffies;
		priv->power_metrics.metrics_valid = true;
		mutex_unlock(&priv->power_metrics_mutex);
		dev_dbg(&priv->serdev->dev, "Power metrics: Voltage = %u mV\n", value);
		break;

	case POWER_CMD_SET:
	case POWER_CMD_SLEEP:
	case POWER_CMD_SHUTDOWN:
	default:
		/* Handle other power commands */
		dev_info(&priv->serdev->dev,
			"Power command: 0x%02x, Param: 0x%02x\n", cmd, param);
		break;
	}
}

/**
 * send_power_metrics_request() - Request power metrics from RP2040
 * @priv: Private driver data
 *
 * Send a request for power metrics to the RP2040.
 *
 * Return: 0 on success, negative error code on failure
 */
int send_power_metrics_request(struct sam_protocol_data *priv)
{
	struct sam_protocol_packet packet;

	if (!priv || !priv->serdev) {
		return -ENODEV;
	}

	packet.type_flags = TYPE_POWER | POWER_CMD_REQUEST_METRICS;
	packet.data[0] = 0x00; /* Request all metrics */
	packet.data[1] = 0x00; /* Reserved */

	return send_packet(priv, &packet);
}

/**
 * power_poll_work_handler() - Work handler for power metrics polling
 * @work: Work structure
 *
 * Periodic work handler that requests power metrics from RP2040.
 */
void power_poll_work_handler(struct work_struct *work)
{
	struct sam_protocol_data *priv = container_of(work, struct sam_protocol_data, 
						      power_poll_work.work);
	int ret;
	unsigned long age_ms;

	if (!priv || !priv->serdev) {
		return;
	}

	/* Check if driver is being removed */
	if (priv->recovery_in_progress) {
		goto reschedule;
	}

	/* Send power metrics request */
	ret = send_power_metrics_request(priv);
	if (ret) {
		dev_warn(&priv->serdev->dev, "Failed to request power metrics: %d\n", ret);
		
		/* Mark metrics as potentially stale */
		mutex_lock(&priv->power_metrics_mutex);
		if (priv->power_metrics.metrics_valid) {
			age_ms = jiffies_to_msecs(jiffies - priv->power_metrics.last_update);
			if (age_ms > POWER_METRICS_TIMEOUT_MS) {
				priv->power_metrics.metrics_valid = false;
				dev_warn(&priv->serdev->dev, "Power metrics timeout (%lu ms)\n", age_ms);
			}
		}
		mutex_unlock(&priv->power_metrics_mutex);
	}

reschedule:
	/* Reschedule for next polling interval if polling is still enabled */
	if (priv->config.power_poll_interval_ms > 0) {
		schedule_delayed_work(&priv->power_poll_work,
				      msecs_to_jiffies(priv->config.power_poll_interval_ms));
	}
}

/* Sysfs attributes for power metrics */

static ssize_t current_ma_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint16_t current_ma;
	bool valid;

	if (!priv) {
		return snprintf(buf, PAGE_SIZE, "driver not available\n");
	}

	mutex_lock(&priv->power_metrics_mutex);
	current_ma = priv->power_metrics.current_ma;
	valid = priv->power_metrics.metrics_valid;
	mutex_unlock(&priv->power_metrics_mutex);

	if (!valid) {
		return snprintf(buf, PAGE_SIZE, "data not available\n");
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", current_ma);
}

static ssize_t battery_percent_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint16_t battery_percent;
	bool valid;

	if (!priv) {
		return snprintf(buf, PAGE_SIZE, "driver not available\n");
	}

	mutex_lock(&priv->power_metrics_mutex);
	battery_percent = priv->power_metrics.battery_percent;
	valid = priv->power_metrics.metrics_valid;
	mutex_unlock(&priv->power_metrics_mutex);

	if (!valid) {
		return snprintf(buf, PAGE_SIZE, "data not available\n");
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", battery_percent);
}

static ssize_t temperature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	int16_t temperature_dc;
	bool valid;

	if (!priv) {
		return snprintf(buf, PAGE_SIZE, "driver not available\n");
	}

	mutex_lock(&priv->power_metrics_mutex);
	temperature_dc = priv->power_metrics.temperature_dc;
	valid = priv->power_metrics.metrics_valid;
	mutex_unlock(&priv->power_metrics_mutex);

	if (!valid) {
		return snprintf(buf, PAGE_SIZE, "data not available\n");
	}

	return snprintf(buf, PAGE_SIZE, "%d.%d\n", temperature_dc / 10, abs(temperature_dc % 10));
}

static ssize_t voltage_mv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	uint16_t voltage_mv;
	bool valid;

	if (!priv) {
		return snprintf(buf, PAGE_SIZE, "driver not available\n");
	}

	mutex_lock(&priv->power_metrics_mutex);
	voltage_mv = priv->power_metrics.voltage_mv;
	valid = priv->power_metrics.metrics_valid;
	mutex_unlock(&priv->power_metrics_mutex);

	if (!valid) {
		return snprintf(buf, PAGE_SIZE, "data not available\n");
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", voltage_mv);
}

static ssize_t metrics_last_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sam_protocol_data *priv = g_sam_protocol_data;
	unsigned long last_update, age_ms;
	bool valid;

	if (!priv) {
		return snprintf(buf, PAGE_SIZE, "driver not available\n");
	}

	mutex_lock(&priv->power_metrics_mutex);
	last_update = priv->power_metrics.last_update;
	valid = priv->power_metrics.metrics_valid;
	mutex_unlock(&priv->power_metrics_mutex);

	if (!valid || last_update == 0) {
		return snprintf(buf, PAGE_SIZE, "never\n");
	}

	age_ms = jiffies_to_msecs(jiffies - last_update);
	return snprintf(buf, PAGE_SIZE, "%lu ms ago\n", age_ms);
}

static DEVICE_ATTR_RO(current_ma);
static DEVICE_ATTR_RO(battery_percent);
static DEVICE_ATTR_RO(temperature);
static DEVICE_ATTR_RO(voltage_mv);
static DEVICE_ATTR_RO(metrics_last_update);

static struct attribute *power_metrics_attrs[] = {
	&dev_attr_current_ma.attr,
	&dev_attr_battery_percent.attr,
	&dev_attr_temperature.attr,
	&dev_attr_voltage_mv.attr,
	&dev_attr_metrics_last_update.attr,
	NULL,
};

static const struct attribute_group power_metrics_group = {
	.name = "power_metrics",
	.attrs = power_metrics_attrs,
};

/**
 * setup_power_metrics_sysfs() - Create power metrics sysfs interface
 * @priv: Private driver data
 *
 * Create sysfs attribute group for power metrics.
 *
 * Return: 0 on success, negative error code on failure
 */
int setup_power_metrics_sysfs(struct sam_protocol_data *priv)
{
	int ret;

	if (!priv || !priv->serdev) {
		return -ENODEV;
	}

	ret = sysfs_create_group(&priv->serdev->dev.kobj, &power_metrics_group);
	if (ret) {
		dev_err(&priv->serdev->dev, "Failed to create power metrics sysfs group: %d\n", ret);
		return ret;
	}

	dev_info(&priv->serdev->dev, "Power metrics sysfs interface created\n");
	return 0;
}

/**
 * cleanup_power_metrics_sysfs() - Remove power metrics sysfs interface
 * @priv: Private driver data
 *
 * Remove sysfs attribute group for power metrics.
 */
void cleanup_power_metrics_sysfs(struct sam_protocol_data *priv)
{
	if (!priv || !priv->serdev) {
		return;
	}

	sysfs_remove_group(&priv->serdev->dev.kobj, &power_metrics_group);
	dev_info(&priv->serdev->dev, "Power metrics sysfs interface removed\n");
}
