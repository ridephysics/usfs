#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"
#include <string.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

#define EM7180_ADDRESS 0x28

static int read_hwinfo(struct em7180 *dev) {
    int rc;
    uint16_t romversion;
    uint16_t ramversion;
    uint8_t productid;
    uint8_t revid;
    uint8_t flags;

    rc = em7180_get_romversion(dev, &romversion);
    if (rc) return -1;

    rc = em7180_get_ramversion(dev, &ramversion);
    if (rc) return -1;

    rc = em7180_get_product_id(dev, &productid);
    if (rc) return -1;

    rc = em7180_get_revision_id(dev, &revid);
    if (rc) return -1;

    CROSSLOGD("romversion: %04x ramversion: %04x productid: %02x revid: %02x",
        romversion, ramversion, productid, revid);

    rc = em7180_get_feature_flags(dev, &flags);
    if (rc) return -1;
    em7180_print_feature_flags(flags);

    return 0;
}

static int read_actual_rates(struct em7180 *dev) {
    int rc;
    uint8_t actual_mag;
    uint8_t actual_accel;
    uint8_t actual_gyro;
    uint8_t actual_baro;
    uint8_t actual_temp;

    rc = em7180_get_actual_rate_mag(dev, &actual_mag);
    if (rc) return -1;

    rc = em7180_get_actual_rate_accel(dev, &actual_accel);
    if (rc) return -1;

    rc = em7180_get_actual_rate_gyro(dev, &actual_gyro);
    if (rc) return -1;

    rc = em7180_get_actual_rate_baro(dev, &actual_baro);
    if (rc) return -1;

    rc = em7180_get_actual_rate_temp(dev, &actual_temp);
    if (rc) return -1;

    CROSSLOGD("actual-rates: mag=%u accel=%u gyro=%u baro=%u temp=%u",
        actual_mag, actual_accel, actual_gyro, actual_baro, actual_temp);

    return 0;
}

int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus) {
    int rc;

    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;

    rc = read_hwinfo(dev);
    if (rc) {
        CROSSLOGE("can't read hwinfo");
        return -1;
    }

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

int em7180_reset_request(struct em7180 *dev) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_RESET_REQUEST, 0x01);
    if (rc) {
        CROSSLOGE("can't request reset");
        return -1;
    }

    return 0;
}

int em7180_init(struct em7180 *dev) {
    int rc;
    uint16_t fs_mag;
    uint16_t fs_acc;
    uint16_t fs_gyro;
    uint8_t sentral_status;
    uint8_t sensor_status;

    rc = em7180_reset_request(dev);
    if (rc) return rc;

    rc = em7180_set_run_mode(dev, false);
    if (rc) return rc;

    rc = em7180_get_sentral_status(dev, &sentral_status);
    if (rc) return rc;
    em7180_print_sentral_status(sentral_status);

    if (sentral_status & EM7180_SS_EEPROM_CRC_ERR) {
        CROSSLOGE("EEPROM CONFIG ERROR");
        return -1;
    }

    // force initialization
    rc = em7180_set_run_mode(dev, true);
    if (rc) return rc;
    rc = em7180_set_run_mode(dev, false);
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

    rc = em7180_set_algorithm(dev, 0x00);
    if (rc) return rc;

    rc = em7180_set_enabled_events(dev, EM7180_EVENT_CPURESET|EM7180_EVENT_ERROR|EM7180_EVENT_QUAT_RES);
    if (rc) return rc;

    rc = em7180_set_run_mode(dev, true);
    if (rc) return rc;

    rc = em7180_fs_read(dev, &fs_mag, &fs_acc, &fs_gyro);
    if (rc) return rc;
    CROSSLOGD("Full Scale Range: mag=%u acc=%u gyro=%u", fs_mag, fs_acc, fs_gyro);

    rc = em7180_param_write_u32(dev, EM7180_PARAM_STILLNESS_ENABLED, 0);
    if (rc) return rc;

    rc = em7180_fs_write(dev, 1000, 8, 2000);
    if (rc) return rc;

    rc = em7180_fs_read(dev, &fs_mag, &fs_acc, &fs_gyro);
    if (rc) return rc;
    CROSSLOGD("Full Scale Range: mag=%u acc=%u gyro=%u", fs_mag, fs_acc, fs_gyro);

    rc = em7180_get_sensor_status(dev, &sensor_status);
    if (rc) return rc;
    em7180_print_sensor_status(sensor_status);

    rc = read_actual_rates(dev);
    if (rc) return rc;

    return 0;
}
