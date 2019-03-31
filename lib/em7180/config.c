#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

int em7180_set_qrate_divisior(struct em7180 *dev, size_t d) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_QRATE_DIVISOR, d);
    if (rc) {
        CROSSLOGE("can't set qrate divisior");
        return -1;
    }

    return 0;
}

int em7180_set_mag_rate(struct em7180 *dev, size_t hz) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_MAG_RATE, hz);
    if (rc) {
        CROSSLOGE("can't set mag rate");
        return -1;
    }

    return 0;
}

int em7180_set_accel_rate(struct em7180 *dev, size_t hz) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_ACCEL_RATE, hz / 10);
    if (rc) {
        CROSSLOGE("can't set accelerometer rate");
        return -1;
    }

    return 0;
}

int em7180_set_gyro_rate(struct em7180 *dev, size_t hz) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_GYRO_RATE, hz / 10);
    if (rc) {
        CROSSLOGE("can't set gyro rate");
        return -1;
    }

    return 0;
}

int em7180_set_baro_rate(struct em7180 *dev, size_t hz) {
    int rc;

    // set enable bit
    if (hz)
        hz |= 0x80;

    rc = em7180_write_byte(dev, EM7180_REG_BARO_RATE, hz);
    if (rc) {
        CROSSLOGE("can't set gyro rate");
        return -1;
    }

    return 0;
}

int em7180_set_enabled_events(struct em7180 *dev, uint8_t events) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_ENABLE_EVENTS, events);
    if (rc) {
        CROSSLOGE("can't set gyro rate");
        return -1;
    }

    return 0;
}

static int em7180_write_i16(struct em7180 *dev, uint8_t reg1, uint8_t reg2, int16_t v) {
    int rc;

    union {
        int16_t i16;
        uint8_t raw[2];
    } u;

    u.i16 = v;

    rc = em7180_write_byte(dev, reg1, u.raw[0]);
    if (rc) return rc;

    rc = em7180_write_byte(dev, reg2, u.raw[1]);
    if (rc) return rc;

    return 0;
}

static inline int64_t e7180_calib_accel_calc_a(int16_t min, int16_t max) {
    return (4096000000 / (max - min)) - 1000000;
}

static inline int64_t e7180_calib_accel_calc_b(int16_t min, int16_t max) {
    return (((2048 - max) + (-2048 - min))*100000) / 4096;
}

int em7180_set_calibration_accel(struct em7180 *dev, int16_t min[3], int16_t max[3]) {
    int rc;
    int64_t v;

    v = e7180_calib_accel_calc_a(min[0], max[0]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_A_X_0, EM7180_REG_CALIB_ACCEL_A_X_1, v);
    if (rc) return -1;

    v = e7180_calib_accel_calc_a(min[1], max[1]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_A_Y_0, EM7180_REG_CALIB_ACCEL_A_Y_1, v);
    if (rc) return -1;

    v = e7180_calib_accel_calc_a(min[2], max[2]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_A_Z_0, EM7180_REG_CALIB_ACCEL_A_Z_1, v);
    if (rc) return -1;

    v = e7180_calib_accel_calc_b(min[0], max[0]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_B_X_0, EM7180_REG_CALIB_ACCEL_B_X_1, v);
    if (rc) return -1;

    v = e7180_calib_accel_calc_b(min[1], max[1]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_B_Y_0, EM7180_REG_CALIB_ACCEL_B_Y_1, v);
    if (rc) return -1;

    v = e7180_calib_accel_calc_b(min[2], max[2]);
    rc = em7180_write_i16(dev, EM7180_REG_CALIB_ACCEL_B_Z_0, EM7180_REG_CALIB_ACCEL_B_Z_1, v);
    if (rc) return -1;

    return 0;
}

int em7180_reset_calibration_accel(struct em7180 *dev) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_X_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_X_1, 0);
    if (rc) return -1;

    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_Y_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_Y_1, 0);
    if (rc) return -1;

    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_Z_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_A_Z_1, 0);
    if (rc) return -1;


    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_X_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_X_1, 0);
    if (rc) return -1;

    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_Y_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_Y_1, 0);
    if (rc) return -1;

    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_Z_0, 0);
    if (rc) return -1;
    rc = em7180_write_byte(dev, EM7180_REG_CALIB_ACCEL_B_Z_1, 0);
    if (rc) return -1;

    return 0;
}
