#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"
#include <string.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

#define EM7180_ADDRESS 0x28

int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;

    return 0;
}

int em7180_destroy(struct em7180 *dev) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}

int em7180_read(struct em7180 *dev, uint8_t reg, void *buf, size_t len) {
    return crossi2c_write_read(dev->i2cbus, EM7180_ADDRESS, &reg, 1, buf, len);
}

int em7180_write_byte(struct em7180 *dev, uint8_t reg, uint8_t value) {
    return crossi2c_write_byte(dev->i2cbus, EM7180_ADDRESS, reg, value);
}

int em7180_init(struct em7180 *dev) {
    int rc;
    uint16_t fs_mag;
    uint16_t fs_acc;
    uint16_t fs_gyro;
    uint8_t alg_status;
    uint8_t event_status;
    uint8_t sensor_status;

    rc = em7180_enter_config_mode(dev);
    if (rc) return rc;

    // 41Hz
    rc = em7180_write_byte(dev, EM7180_REG_ACC_LPF_BW, 0x03);
    if (rc) return rc;

    // 184Hz
    rc = em7180_write_byte(dev, EM7180_REG_GYRO_LPF_BW, 0x01);
    if (rc) return rc;

    rc = em7180_set_qrate_divisior(dev, 2);
    if (rc) return rc;

    rc = em7180_set_mag_rate(dev, 100);
    if (rc) return rc;

    rc = em7180_set_accel_rate(dev, 200);
    if (rc) return rc;

    rc = em7180_set_gyro_rate(dev, 200);
    if (rc) return rc;

    rc = em7180_set_baro_rate(dev, 50);
    if (rc) return rc;

    rc = em7180_set_standby(dev, false);
    if (rc) return rc;

    rc = em7180_set_enabled_events(dev, EM7180_EVENT_CPURESET|EM7180_EVENT_ERROR|EM7180_EVENT_QUAT_RES);
    if (rc) return rc;

    rc = em7180_enter_run_mode(dev);
    if (rc) return rc;

    rc = em7180_fs_read(dev, &fs_mag, &fs_acc, &fs_gyro);
    if (rc) return rc;
    CROSSLOGI("Full Scale Range: mag=%u acc=%u gyro=%u", fs_mag, fs_acc, fs_gyro);

    rc = em7180_param_write_u32(dev, EM7180_PARAM_STILLNESS_ENABLED, 0);
    if (rc) return rc;

    rc = em7180_fs_write(dev, 1000, 8, 2000);
    if (rc) return rc;

    rc = em7180_fs_read(dev, &fs_mag, &fs_acc, &fs_gyro);
    if (rc) return rc;
    CROSSLOGI("Full Scale Range: mag=%u acc=%u gyro=%u", fs_mag, fs_acc, fs_gyro);

    rc = em7180_read(dev, EM7180_REG_ALGORITHM_STATUS, &alg_status, 1);
    if (rc) {
        CROSSLOGE("can't read alg status");
        return -1;
    }
    em7180_print_algorithm_status(alg_status);

    rc = em7180_get_event_status(dev, &event_status);
    if (rc) return rc;
    em7180_print_event_status(event_status);

    rc = em7180_read(dev, EM7180_REG_SENSOR_STATUS, &sensor_status, 1);
    if (rc) {
        CROSSLOGE("can't read sensor status");
        return -1;
    }
    em7180_print_sensor_status(sensor_status);

    return 0;
}
