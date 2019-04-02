#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

void em7180_parse_data_quaternion(uint8_t quat_raw[18], uint32_t quat[4], uint16_t *ptime) {
    quat[0] = ((uint32_t)quat_raw[3] << 24) | ((uint32_t)quat_raw[2] << 16) |
        ((uint32_t)quat_raw[1] << 8) | quat_raw[0];
    quat[1] = ((uint32_t)quat_raw[7] << 24) | ((uint32_t)quat_raw[6] << 16) |
        ((uint32_t)quat_raw[5] << 8) | quat_raw[4];
    quat[2] = ((uint32_t)quat_raw[11] << 24) | ((uint32_t)quat_raw[10] << 16) |
        ((uint32_t)quat_raw[9] << 8) | quat_raw[8];
    quat[3] = ((uint32_t)quat_raw[15] << 24) | ((uint32_t)quat_raw[14] << 16) |
        ((uint32_t)quat_raw[13] << 8) | quat_raw[12];

    if (ptime)
        *ptime = ((uint16_t)quat_raw[17] << 8) | quat_raw[16];
}

void em7180_parse_data_magnetometer(uint8_t mag_raw[8], int16_t mag[3], uint16_t *ptime) {
    mag[0] = ((int16_t)mag_raw[1] << 8) | mag_raw[0];
    mag[1] = ((int16_t)mag_raw[3] << 8) | mag_raw[2];
    mag[2] = ((int16_t)mag_raw[5] << 8) | mag_raw[4];

    if (ptime)
        *ptime = ((uint16_t)mag_raw[7] << 8) | mag_raw[6];
}

void em7180_parse_data_accelerometer(uint8_t acc_raw[8], int16_t acc[3], uint16_t *ptime) {
    acc[0] = ((int16_t)acc_raw[1] << 8) | acc_raw[0];
    acc[1] = ((int16_t)acc_raw[3] << 8) | acc_raw[2];
    acc[2] = ((int16_t)acc_raw[5] << 8) | acc_raw[4];

    if (ptime)
        *ptime = ((uint16_t)acc_raw[7] << 8) | acc_raw[6];
}

void em7180_parse_data_gyroscope(uint8_t gyro_raw[8], int16_t gyro[3], uint16_t *ptime) {
    gyro[0] = ((int16_t)gyro_raw[1] << 8) | gyro_raw[0];
    gyro[1] = ((int16_t)gyro_raw[3] << 8) | gyro_raw[2];
    gyro[2] = ((int16_t)gyro_raw[5] << 8) | gyro_raw[4];
    if (ptime)
        *ptime = ((uint16_t)gyro_raw[7] << 8) | gyro_raw[6];
}

void em7180_parse_data_barometer(uint8_t baro_raw[4], int16_t *baro, uint16_t *ptime) {
    *baro = ((int16_t)baro_raw[1] << 8) | baro_raw[0];
    if (ptime)
        *ptime = ((uint16_t)baro_raw[3] << 8) | baro_raw[2];
}

void em7180_parse_data_temperature(uint8_t temp_raw[4], int16_t *temp, uint16_t *ptime) {
    *temp = ((int16_t)temp_raw[1] << 8) | temp_raw[0];
    if (ptime)
        *ptime = ((uint16_t)temp_raw[3] << 8) | temp_raw[2];
}

int em7180_get_data_accelerometer(struct em7180 *dev, int16_t acc[3], uint16_t *ptime) {
    int rc;
    uint8_t acc_raw[8];

    rc = em7180_read(dev, EM7180_REG_AX_0, acc_raw, sizeof(acc_raw));
    if (rc) {
        CROSSLOGE("can't acc data");
        return -1;
    }

    em7180_parse_data_accelerometer(acc_raw, acc, ptime);

    return 0;
}

int em7180_get_data_gyroscope(struct em7180 *dev, int16_t gyro[3], uint16_t *ptime) {
    int rc;
    uint8_t gyro_raw[8];

    rc = em7180_read(dev, EM7180_REG_GX_0, gyro_raw, sizeof(gyro_raw));
    if (rc) {
        CROSSLOGE("can't gyro data");
        return -1;
    }

    em7180_parse_data_gyroscope(gyro_raw, gyro, ptime);

    return 0;
}

int em7180_get_data_magnetometer(struct em7180 *dev, int16_t mag[3], uint16_t *ptime) {
    int rc;
    uint8_t mag_raw[8];

    rc = em7180_read(dev, EM7180_REG_MX_0, mag_raw, sizeof(mag_raw));
    if (rc) {
        CROSSLOGE("can't mag data");
        return -1;
    }

    em7180_parse_data_magnetometer(mag_raw, mag, ptime);

    return 0;
}

int em7180_get_data_quaternion(struct em7180 *dev, uint32_t quat[4], uint16_t *ptime) {
    int rc;
    uint8_t quat_raw[18];

    rc = em7180_read(dev, EM7180_REG_QX_0, quat_raw, sizeof(quat_raw));
    if (rc) {
        CROSSLOGE("can't quat data");
        return -1;
    }

    em7180_parse_data_quaternion(quat_raw, quat, ptime);

    return 0;
}

int em7180_get_data_barometer(struct em7180 *dev, int16_t *baro, uint16_t *ptime) {
    int rc;
    uint8_t baro_raw[4];

    rc = em7180_read(dev, EM7180_REG_BD_0, baro_raw, sizeof(baro_raw));
    if (rc) {
        CROSSLOGE("can't baro data");
        return -1;
    }

    em7180_parse_data_barometer(baro_raw, baro, ptime);

    return 0;
}

int em7180_get_data_temperature(struct em7180 *dev, int16_t *temp, uint16_t *ptime) {
    int rc;
    uint8_t temp_raw[4];

    rc = em7180_read(dev, EM7180_REG_TD_0, temp_raw, sizeof(temp_raw));
    if (rc) {
        CROSSLOGE("can't temp data");
        return -1;
    }

    em7180_parse_data_temperature(temp_raw, temp, ptime);

    return 0;
}

int em7180_get_data_all_raw(struct em7180 *dev, uint8_t raw[50]) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_QX_0, raw, 50);
    if (rc) {
        CROSSLOGE("can't raw sensor data");
        return -1;
    }

    return 0;
}
