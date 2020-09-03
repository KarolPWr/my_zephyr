#ifndef M_UTILS_H_
#define M_UTILS_H_

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>

#include <device.h>
#include <drivers/sensor.h>

#define SENSOR_1_NAME "Temperature Sensor 1"
#define SENSOR_3_NAME "Humidity Sensor"

/* Sensor Internal Update Interval [seconds] */
#define SENSOR_1_UPDATE_IVAL 5
// #define SENSOR_2_UPDATE_IVAL			12
#define SENSOR_3_UPDATE_IVAL 60

/* ESS error definitions */
#define ESS_ERR_WRITE_REJECT 0x80
#define ESS_ERR_COND_NOT_SUPP 0x81

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE 0x00
#define ESS_FIXED_TIME_INTERVAL 0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME 0x02
#define ESS_VALUE_CHANGED 0x03
#define ESS_LESS_THAN_REF_VALUE 0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE 0x05
#define ESS_GREATER_THAN_REF_VALUE 0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE 0x07
#define ESS_EQUAL_TO_REF_VALUE 0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE 0x09

/* Main sleep interval */
#define SLEEP_S 70U

/* Environmental Sensing Service Declaration */

struct es_measurement
{
	u16_t flags; /* Reserved for Future Use */
	u8_t sampling_func;
	u32_t meas_period;
	u32_t update_interval;
	u8_t application;
	u8_t meas_uncertainty;
};

struct temperature_sensor
{
	s32_t temp_value;

	/* Valid Range */
	s16_t lower_limit;
	s16_t upper_limit;

	/* ES trigger setting - Value Notification condition */
	u8_t condition;
	union {
		u32_t seconds;
		s16_t ref_val; /* Reference temperature */
	};

	struct es_measurement meas;
};

struct humidity_sensor
{
	s16_t humid_value;

	struct es_measurement meas;
};

struct bme280_readings
{
	struct sensor_value temperature;
	struct sensor_value humidity;
};

struct es_trigger_setting_seconds
{
	u8_t condition;
	u8_t sec[3];
} __packed;

struct es_trigger_setting_reference
{
	u8_t condition;
	s16_t ref_val;
} __packed;

#endif /* M_UTILS_H_ */