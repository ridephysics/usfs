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
