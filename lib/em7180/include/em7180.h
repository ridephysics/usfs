#ifndef EM7180_H
#define EM7180_H

#include <crossi2c.h>
#include <stdbool.h>

struct em7180 {
    struct crossi2c_bus *i2cbus;
};

// feature flags
enum {
    EM7180_FF_BAROMETER = 0x1,
    EM7180_FF_HUMIDITY_SENSOR = 0x2,
    EM7180_FF_TEMP_SENSOR = 0x4,
    EM7180_FF_CUSTOM_SENSOR_1 = 0x8,
    EM7180_FF_CUSTOM_SENSOR_2 = 0x10,
    EM7180_FF_CUSTOM_SENSOR_3 = 0x20,
    EM7180_FF_RESERVED1 = 0x40,
    EM7180_FF_RESERVED2 = 0x80,
};

// sentral status
enum {
    EM7180_SS_EEPROM_DETECTED = 0x1,
    EM7180_SS_EEPROM_CFG_UPLOADED = 0x2,
    EM7180_SS_EEPROM_CRC_ERR = 0x4,
    EM7180_SS_INITIALIZED = 0x8,
    EM7180_SS_NO_EEPROM = 0x10,
    EM7180_SS_RESERVED1 = 0x20,
    EM7180_SS_RESERVED2 = 0x40,
    EM7180_SS_RESERVED3 = 0x80,
};

// algorithm control
enum {
    EM7180_AC_STANDBY_ENABLE = 0x1,
    EM7180_AC_RAWDATA_ENABLE = 0x2,
    EM7180_AC_HPR_OUTPUT = 0x4,
    EM7180_AC_6AXIS_ENABLE = 0x8,
    EM7180_AC_RESERVED1 = 0x10,
    EM7180_AC_ENU_OUTPUT = 0x20,
    EM7180_AC_DIS_GYR_WHEN_STILL = 0x40,
    EM7180_AC_PARAM_TRANSFER = 0x80,
};

// algorithm status
enum {
    EM7180_AS_STANDBY = 0x1,
    EM7180_AS_SLOW = 0x2,
    EM7180_AS_STILLNESS = 0x4,
    EM7180_AS_CAL_STABLE = 0x8,
    EM7180_AS_MAG_TRANSIENT = 0x10,
    EM7180_AS_UNRELIABLE = 0x20,
    EM7180_AS_RESERVED1 = 0x40,
    EM7180_AS_RESERVED2 = 0x80,
};

// host control
enum {
    EM7180_HOSTCTRL_RUN_ENABLE = 0x1,
    EM7180_HOSTCTRL_ENABLE_UPLOAD = 0x2,
};

// enable events
enum {
    EM7180_EVENT_CPURESET = 0x1,
    EM7180_EVENT_ERROR = 0x2,
    EM7180_EVENT_QUAT_RES = 0x4,
    EM7180_EVENT_MAG_RES = 0x8,
    EM7180_EVENT_ACCEL_RES = 0x10,
    EM7180_EVENT_GYRO_RES = 0x20,
    EM7180_EVENT_BARO_RES = 0x40,
    EM7180_EVENT_RESERVED2 = 0x80,
};

// sensor status
enum {
    EM7180_SENSORSTATUS_MAG_NACK = 0x1,
    EM7180_SENSORSTATUS_ACCEL_NACK = 0x2,
    EM7180_SENSORSTATUS_GYRO_NACK = 0x4,
    EM7180_SENSORSTATUS_RESERVED1 = 0x8,
    EM7180_SENSORSTATUS_MAGID_ERROR = 0x10,
    EM7180_SENSORSTATUS_ACCELID_ERROR = 0x20,
    EM7180_SENSORSTATUS_GYROID_ERROR = 0x40,
    EM7180_SENSORSTATUS_RESERVED2 = 0x80,
};

// error register
enum em7180_error {
    EM7180_ERR_NONE = 0x00,
    EM7180_ERR_MAG_FAILURE = 0x11,
    EM7180_ERR_ACC_FAILURE = 0x12,
    EM7180_ERR_GYRO_FAILURE = 0x14,
    EM7180_ERR_MAG_INIT_FAILURE = 0x21,
    EM7180_ERR_ACC_INIT_FAILURE = 0x22,
    EM7180_ERR_GYRO_INIT_FAILURE = 0x24,
    EM7180_ERR_MATH = 0x30,
    EM7180_ERR_INVALID_SAMPLE_RATE = 0x80,
};

enum em7180_param {
    EM7180_PARAM_STILLNESS_ENABLED = 0x49,
    EM7180_PARAM_FS_MAG_ACC = 0x4a,
    EM7180_PARAM_FS_GYRO = 0x4b,
};

// init
int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus);
int em7180_destroy(struct em7180 *dev);
int em7180_init(struct em7180 *dev);

// info
int em7180_get_romversion(struct em7180 *dev, uint16_t *pversion);
int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion);
int em7180_get_product_id(struct em7180 *dev, uint8_t *pversion);
int em7180_get_revision_id(struct em7180 *dev, uint8_t *pversion);
int em7180_get_feature_flags(struct em7180 *dev, uint8_t *pflags);
int em7180_get_sentral_status(struct em7180 *dev, uint8_t *pstatus);
int em7180_get_error_register(struct em7180 *dev, enum em7180_error *perror);
int em7180_get_event_status(struct em7180 *dev, uint8_t *pstatus);

// config
int em7180_set_qrate_divisior(struct em7180 *dev, size_t d);
int em7180_set_mag_rate(struct em7180 *dev, size_t hz);
int em7180_set_accel_rate(struct em7180 *dev, size_t hz);
int em7180_set_gyro_rate(struct em7180 *dev, size_t hz);
int em7180_set_baro_rate(struct em7180 *dev, size_t hz);
int em7180_set_enabled_events(struct em7180 *dev, uint8_t events);

// data
int em7180_get_data_accelerometer(struct em7180 *dev, int16_t pacc[3]);
int em7180_get_data_gyroscope(struct em7180 *dev, int16_t pgyro[3]);
int em7180_get_data_magnetometer(struct em7180 *dev, int16_t pmag[3]);
int em7180_get_data_quaternion(struct em7180 *dev, uint32_t pquat[4]);
int em7180_get_data_barometer(struct em7180 *dev, int16_t *pbaro);
int em7180_get_data_temperature(struct em7180 *dev, int16_t *ptemp);

// debug
void em7180_print_feature_flags(uint8_t flags);
void em7180_print_sentral_status(uint8_t flags);
void em7180_print_algorithm_ctrl(uint8_t ctrl);
void em7180_print_algorithm_status(uint8_t status);
void em7180_print_event_status(uint8_t status);
void em7180_print_sensor_status(uint8_t status);
void em7180_print_error(enum em7180_error error);

// param
int em7180_param_read(struct em7180 *dev, enum em7180_param param, uint8_t data[4]);
int em7180_param_write(struct em7180 *dev, enum em7180_param param, uint8_t data[4]);
int em7180_param_write_u32(struct em7180 *dev, enum em7180_param param, uint32_t v);
int em7180_fs_read(struct em7180 *dev, uint16_t *pmag, uint16_t *pacc, uint16_t *pgyro);
int em7180_fs_write(struct em7180 *dev, uint16_t mag, uint16_t acc, uint16_t gyro);

// state
int em7180_set_standby(struct em7180 *dev, bool enabled);
int em7180_set_run_mode(struct em7180 *dev, bool enabled);
int em7180_passthrough_enter(struct em7180 *dev);
int em7180_passthrough_exit(struct em7180 *dev);

#endif /* EM7180_H */
