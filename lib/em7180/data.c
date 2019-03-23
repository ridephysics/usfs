#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

int em7180_get_data_accelerometer(struct em7180 *dev, int16_t pacc[3], uint16_t *ptime) {
    int rc;
    uint8_t acc_raw[8];

    rc = em7180_read(dev, EM7180_REG_AX_0, acc_raw, sizeof(acc_raw));
    if (rc) {
        CROSSLOGE("can't acc data");
        return -1;
    }

    pacc[0] = ((int16_t)acc_raw[1] << 8) | acc_raw[0];
    pacc[1] = ((int16_t)acc_raw[3] << 8) | acc_raw[2];
    pacc[2] = ((int16_t)acc_raw[5] << 8) | acc_raw[4];

    if (ptime)
        *ptime = ((uint16_t)acc_raw[7] << 8) | acc_raw[6];

    return 0;
}

int em7180_get_data_gyroscope(struct em7180 *dev, int16_t pgyro[3], uint16_t *ptime) {
    int rc;
    uint8_t gyro_raw[8];

    rc = em7180_read(dev, EM7180_REG_GX_0, gyro_raw, sizeof(gyro_raw));
    if (rc) {
        CROSSLOGE("can't gyro data");
        return -1;
    }

    pgyro[0] = ((int16_t)gyro_raw[1] << 8) | gyro_raw[0];
    pgyro[1] = ((int16_t)gyro_raw[3] << 8) | gyro_raw[2];
    pgyro[2] = ((int16_t)gyro_raw[5] << 8) | gyro_raw[4];
    if (ptime)
        *ptime = ((uint16_t)gyro_raw[7] << 8) | gyro_raw[6];

    return 0;
}

int em7180_get_data_magnetometer(struct em7180 *dev, int16_t pmag[3], uint16_t *ptime) {
    int rc;
    uint8_t mag_raw[8];

    rc = em7180_read(dev, EM7180_REG_MX_0, mag_raw, sizeof(mag_raw));
    if (rc) {
        CROSSLOGE("can't mag data");
        return -1;
    }

    pmag[0] = ((int16_t)mag_raw[1] << 8) | mag_raw[0];
    pmag[1] = ((int16_t)mag_raw[3] << 8) | mag_raw[2];
    pmag[2] = ((int16_t)mag_raw[5] << 8) | mag_raw[4];
    if (ptime)
        *ptime = ((uint16_t)mag_raw[7] << 8) | mag_raw[6];

    return 0;
}

int em7180_get_data_quaternion(struct em7180 *dev, uint32_t pquat[4], uint16_t *ptime) {
    int rc;
    uint8_t quat_raw[18];

    rc = em7180_read(dev, EM7180_REG_QX_0, quat_raw, sizeof(quat_raw));
    if (rc) {
        CROSSLOGE("can't quat data");
        return -1;
    }

    pquat[0] = ((uint32_t)quat_raw[3] << 24) | ((uint32_t)quat_raw[2] << 16) |
        ((uint32_t)quat_raw[1] << 8) | quat_raw[0];
    pquat[1] = ((uint32_t)quat_raw[7] << 24) | ((uint32_t)quat_raw[6] << 16) |
        ((uint32_t)quat_raw[5] << 8) | quat_raw[4];
    pquat[2] = ((uint32_t)quat_raw[11] << 24) | ((uint32_t)quat_raw[10] << 16) |
        ((uint32_t)quat_raw[9] << 8) | quat_raw[8];
    pquat[3] = ((uint32_t)quat_raw[15] << 24) | ((uint32_t)quat_raw[14] << 16) |
        ((uint32_t)quat_raw[13] << 8) | quat_raw[12];
    if (ptime)
        *ptime = ((uint16_t)quat_raw[17] << 8) | quat_raw[16];

    return 0;
}

int em7180_get_data_barometer(struct em7180 *dev, int16_t *pbaro, uint16_t *ptime) {
    int rc;
    uint8_t baro_raw[4];

    rc = em7180_read(dev, EM7180_REG_BD_0, baro_raw, sizeof(baro_raw));
    if (rc) {
        CROSSLOGE("can't baro data");
        return -1;
    }

    *pbaro = ((int16_t)baro_raw[1] << 8) | baro_raw[0];
    if (ptime)
        *ptime = ((uint16_t)baro_raw[3] << 8) | baro_raw[2];

    return 0;
}

int em7180_get_data_temperature(struct em7180 *dev, int16_t *ptemp, uint16_t *ptime) {
    int rc;
    uint8_t temp_raw[4];

    rc = em7180_read(dev, EM7180_REG_TD_0, temp_raw, sizeof(temp_raw));
    if (rc) {
        CROSSLOGE("can't temp data");
        return -1;
    }

    *ptemp = ((int16_t)temp_raw[1] << 8) | temp_raw[0];
    if (ptime)
        *ptime = ((uint16_t)temp_raw[3] << 8) | temp_raw[2];

    return 0;
}
