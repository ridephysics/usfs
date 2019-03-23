#include <em7180.h>
#include <em7180_regs.h>
#include <string.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

#define EM7180_ADDRESS 0x28

static int em7180_read(struct em7180 *dev, uint8_t reg, void *buf, size_t len) {
    return crossi2c_write_read(dev->i2cbus, EM7180_ADDRESS, &reg, 1, buf, len);
}

static int em7180_write_byte(struct em7180 *dev, uint8_t reg, uint8_t value) {
    return crossi2c_write_byte(dev->i2cbus, EM7180_ADDRESS, reg, value);
}

int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));

    dev->i2cbus = i2cbus;

    return 0;
}

int em7180_destroy(struct em7180 *dev) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}

int em7180_get_romversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_ROM_VERSION_0, &version_raw, sizeof(version_raw));
    if (rc) return -1;

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_RAM_VERSION_0, &version_raw, sizeof(version_raw));
    if (rc) return -1;

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_product_id(struct em7180 *dev, uint8_t *pversion) {
    return em7180_read(dev, EM7180_REG_PRODUCT_ID, pversion, sizeof(*pversion));
}

int em7180_get_revision_id(struct em7180 *dev, uint8_t *pversion) {
    return em7180_read(dev, EM7180_REG_REVISION_ID, pversion, sizeof(*pversion));
}

int em7180_get_feature_flags(struct em7180 *dev, uint8_t *pflags) {
    return em7180_read(dev, EM7180_REG_FEATURE_FLAGS, pflags, sizeof(*pflags));
}

int em7180_get_sentral_status(struct em7180 *dev, uint8_t *pstatus) {
    return em7180_read(dev, EM7180_REG_SENTRAL_STATUS, pstatus, sizeof(*pstatus));
}

int em7180_set_standby(struct em7180 *dev, bool enabled) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, enabled ? EM7180_AC_STANDBY_ENABLE : 0x00);
    if (rc) {
        CROSSLOGE("can't set standby mode");
        return -1;
    }

    usleep(5000);

    return 0;
}

int em7180_passthrough_enter(struct em7180 *dev) {
    int rc;
    uint8_t ptstatus;

    rc = em7180_set_standby(dev, true);
    if (rc) {
        CROSSLOGE("can't enter standby mode");
        return -1;
    }

    rc = em7180_read(dev, EM7180_REG_PASSTHROUGH_STATUS, &ptstatus, 1);
    if (rc) {
        CROSSLOGE("can't read pt status");
        return -1;
    }

    if (ptstatus & 0x01) {
        CROSSLOGD("we're in passthrough mode already");
        return 0;
    }

    CROSSLOGD("enter passthrough mode");

    rc = em7180_write_byte(dev, EM7180_REG_PASSTRHOUGH_CTRL, 0x01);
    if (rc) {
        CROSSLOGE("can't put device into passthrough mode");
        return -1;
    }

    do {
        usleep(5000);

        rc = em7180_read(dev, EM7180_REG_PASSTHROUGH_STATUS, &ptstatus, 1);
        if (rc) {
            CROSSLOGE("can't read passthrough status");
            return -1;
        }
    } while (!(ptstatus & 0x01));

    return 0;
}

int em7180_passthrough_exit(struct em7180 *dev) {
    int rc;
    uint8_t ptstatus;

    rc = em7180_read(dev, EM7180_REG_PASSTHROUGH_STATUS, &ptstatus, 1);
    if (rc) {
        CROSSLOGE("can't read pt status");
        return -1;
    }

    if (!(ptstatus & 0x01)) {
        CROSSLOGD("we're in passthrough mode already");
        return 0;
    }

    CROSSLOGD("exit passthrough mode");

    rc = em7180_write_byte(dev, EM7180_REG_PASSTRHOUGH_CTRL, 0x00);
    if (rc) {
        CROSSLOGE("can't put device out of passthrough mode");
        return -1;
    }

    do {
        usleep(5000);

        rc = em7180_read(dev, EM7180_REG_PASSTHROUGH_STATUS, &ptstatus, 1);
        if (rc) {
            CROSSLOGE("can't read passthrough status");
            return -1;
        }
    } while (ptstatus & 0x01);

    rc = em7180_set_standby(dev, false);
    if (rc) {
        CROSSLOGE("can't exit standby mode");
        return -1;
    }

    return 0;
}

int em7180_enter_config_mode(struct em7180 *dev) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_HOST_CTRL, 0x00);
    if (rc) {
        CROSSLOGE("can't enter config mode");
        return -1;
    }

    return 0;
}

int em7180_enter_run_mode(struct em7180 *dev) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_HOST_CTRL, EM7180_HOSTCTRL_RUN_ENABLE);
    if (rc) {
        CROSSLOGE("can't enter run mode");
        return -1;
    }

    return 0;
}

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

int em7180_param_read(struct em7180 *dev, enum em7180_param param, uint8_t data[4]) {
    int rc;
    uint8_t ack;

    rc = em7180_write_byte(dev, EM7180_REG_PARAM_REQUEST, param);
    if (rc) {
        CROSSLOGE("can't set param request");
        return -1;
    }

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, EM7180_AC_PARAM_TRANSFER);
    if (rc) {
        CROSSLOGE("can't start param transfer");
        return -1;
    }

    do {
        rc = em7180_read(dev, EM7180_REG_PARAM_ACK, &ack, 1);
        if (rc) {
            CROSSLOGE("can't read param ack");
            return -1;
        }
    } while (ack != param);

    rc = em7180_read(dev, EM7180_REG_SAVED_PARAM_BYTE0, data, 4);
    if (rc) {
        CROSSLOGE("can't read param data");
        return -1;
    }

    return 0;
}

int em7180_param_write(struct em7180 *dev, enum em7180_param param, uint8_t data[4]) {
    int rc;
    size_t i;
    uint8_t ack;
    uint8_t rawparam = param | 0x80;

    for (i = 0; i < 4; i++) {
        rc = em7180_write_byte(dev, EM7180_REG_LOAD_PARAM_BYTE0 + i, data[i]);
        if (rc) {
            CROSSLOGE("can't write param data");
            return -1;
        }
    }

    rc = em7180_write_byte(dev, EM7180_REG_PARAM_REQUEST, rawparam);
    if (rc) {
        CROSSLOGE("can't set param request");
        return -1;
    }

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, EM7180_AC_PARAM_TRANSFER);
    if (rc) {
        CROSSLOGE("can't start param transfer");
        return -1;
    }

    do {
        rc = em7180_read(dev, EM7180_REG_PARAM_ACK, &ack, 1);
        if (rc) {
            CROSSLOGE("can't read param ack");
            return -1;
        }
    } while (ack != rawparam);

    rc = em7180_write_byte(dev, EM7180_REG_PARAM_REQUEST, 0x00);
    if (rc) {
        CROSSLOGE("can't reset param request");
        return -1;
    }

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, 0x00);
    if (rc) {
        CROSSLOGE("can't reset param request");
        return -1;
    }

    return 0;
}

int em7180_param_write_u32(struct em7180 *dev, enum em7180_param param, uint32_t v) {
    uint8_t data[4];

    data[0] = v & 0xff;
    data[1] = (v >> 8) & 0xff;
    data[2] = (v >> 16) & 0xff;
    data[3] = (v >> 24) & 0xff;

    return em7180_param_write(dev, param, data);
}

int em7180_fs_read(struct em7180 *dev, uint16_t *pmag, uint16_t *pacc, uint16_t *pgyro) {
    int rc;
    uint8_t magacc[4];
    uint8_t gyro[4];

    rc = em7180_param_read(dev, EM7180_PARAM_FS_MAG_ACC, magacc);
    if (rc) {
        CROSSLOGE("can't read magacc fs param");
        return -1;
    }

    rc = em7180_param_read(dev, EM7180_PARAM_FS_GYRO, gyro);
    if (rc) {
        CROSSLOGE("can't read gyro fs param");
        return -1;
    }

    *pmag = (uint16_t)((magacc[1] << 8) | magacc[0]);
    *pacc = (uint16_t)((magacc[3] << 8) | magacc[2]);
    *pgyro = (uint16_t)((gyro[1] << 8) | gyro[0]);

    return 0;
}

int em7180_fs_write(struct em7180 *dev, uint16_t mag, uint16_t acc, uint16_t gyro) {
    uint8_t data[4];
    int rc;

    data[0] = mag & 0xff;
    data[1] = (mag >> 8) & 0xff;
    data[2] = acc & 0xff;
    data[3] = (acc >> 8) & 0xff;

    rc = em7180_param_write(dev, EM7180_PARAM_FS_MAG_ACC, data);
    if (rc) {
        CROSSLOGE("can't write magacc fs param");
        return -1;
    }

    data[0] = gyro & 0xff;
    data[1] = (gyro >> 8) & 0xff;
    data[2] = 0x00;
    data[3] = 0x00;

    rc = em7180_param_write(dev, EM7180_PARAM_FS_GYRO, data);
    if (rc) {
        CROSSLOGE("can't write magacc fs param");
        return -1;
    }

    return 0;
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

int em7180_get_error_register(struct em7180 *dev, enum em7180_error *perror) {
    int rc;
    uint8_t error;

    rc = em7180_read(dev, EM7180_REG_ERROR, &error, 1);
    if (rc) {
        CROSSLOGE("can't read error register");
        return -1;
    }

    *perror = error;

    return 0;
}

int em7180_get_event_status(struct em7180 *dev, uint8_t *pstatus) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_EVENT_STATUS, pstatus, 1);
    if (rc) {
        CROSSLOGE("can't read event status");
        return -1;
    }

    return 0;
}

int em7180_get_data_accelerometer(struct em7180 *dev, int16_t pacc[3]) {
    int rc;
    uint8_t acc_raw[6];

    rc = em7180_read(dev, EM7180_REG_AX_0, acc_raw, sizeof(acc_raw));
    if (rc) {
        CROSSLOGE("can't acc data");
        return -1;
    }

    pacc[0] = ((int16_t)acc_raw[1] << 8) | acc_raw[0];
    pacc[1] = ((int16_t)acc_raw[3] << 8) | acc_raw[2];
    pacc[2] = ((int16_t)acc_raw[5] << 8) | acc_raw[4];

    return 0;
}

int em7180_get_data_gyroscope(struct em7180 *dev, int16_t pgyro[3]) {
    int rc;
    uint8_t gyro_raw[6];

    rc = em7180_read(dev, EM7180_REG_GX_0, gyro_raw, sizeof(gyro_raw));
    if (rc) {
        CROSSLOGE("can't gyro data");
        return -1;
    }

    pgyro[0] = ((int16_t)gyro_raw[1] << 8) | gyro_raw[0];
    pgyro[1] = ((int16_t)gyro_raw[3] << 8) | gyro_raw[2];
    pgyro[2] = ((int16_t)gyro_raw[5] << 8) | gyro_raw[4];

    return 0;
}

int em7180_get_data_magnetometer(struct em7180 *dev, int16_t pmag[3]) {
    int rc;
    uint8_t mag_raw[6];

    rc = em7180_read(dev, EM7180_REG_MX_0, mag_raw, sizeof(mag_raw));
    if (rc) {
        CROSSLOGE("can't mag data");
        return -1;
    }

    pmag[0] = ((int16_t)mag_raw[1] << 8) | mag_raw[0];
    pmag[1] = ((int16_t)mag_raw[3] << 8) | mag_raw[2];
    pmag[2] = ((int16_t)mag_raw[5] << 8) | mag_raw[4];

    return 0;
}

int em7180_get_data_quaternion(struct em7180 *dev, uint32_t pquat[4]) {
    int rc;
    uint8_t quat_raw[16];

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

    return 0;
}

int em7180_get_data_barometer(struct em7180 *dev, int16_t *pbaro) {
    int rc;
    uint8_t baro_raw[2];

    rc = em7180_read(dev, EM7180_REG_BD_0, baro_raw, sizeof(baro_raw));
    if (rc) {
        CROSSLOGE("can't baro data");
        return -1;
    }

    *pbaro = ((int16_t)baro_raw[1] << 8) | baro_raw[0];

    return 0;
}

int em7180_get_data_temperature(struct em7180 *dev, int16_t *ptemp) {
    int rc;
    uint8_t temp_raw[2];

    rc = em7180_read(dev, EM7180_REG_TD_0, temp_raw, sizeof(temp_raw));
    if (rc) {
        CROSSLOGE("can't temp data");
        return -1;
    }

    *ptemp = ((int16_t)temp_raw[1] << 8) | temp_raw[0];

    return 0;
}

void em7180_print_feature_flags(uint8_t flags) {
    CROSSLOGD("features:");

    if (flags & EM7180_FF_BAROMETER)
        CROSSLOGD("\tbarometer");
    if (flags & EM7180_FF_HUMIDITY_SENSOR)
        CROSSLOGD("\thumidity sensor");
    if (flags & EM7180_FF_TEMP_SENSOR)
        CROSSLOGD("\ttemperature sensor");
    if (flags & EM7180_FF_CUSTOM_SENSOR_1)
        CROSSLOGD("\tcustom sensor 1");
    if (flags & EM7180_FF_CUSTOM_SENSOR_2)
        CROSSLOGD("\tcustom sensor 2");
    if (flags & EM7180_FF_CUSTOM_SENSOR_3)
        CROSSLOGD("\tcustom sensor 3");
    if (flags & EM7180_FF_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (flags & EM7180_FF_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_sentral_status(uint8_t status) {
    CROSSLOGD("sentral status:");

    if (status & EM7180_SS_EEPROM_DETECTED)
        CROSSLOGD("\teeprom detected");
    if (status & EM7180_SS_EEPROM_CFG_UPLOADED)
        CROSSLOGD("\teeprom config uploaded");
    if (status & EM7180_SS_EEPROM_CRC_ERR)
        CROSSLOGD("\teeprom crc error");
    if (status & EM7180_SS_INITIALIZED)
        CROSSLOGD("\tinitialized");
    if (status & EM7180_SS_NO_EEPROM)
        CROSSLOGD("\tno eeprom");
    if (status & EM7180_SS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_SS_RESERVED2)
        CROSSLOGD("\treserved 2");
    if (status & EM7180_SS_RESERVED3)
        CROSSLOGD("\treserved 3");
}

void em7180_print_algorithm_ctrl(uint8_t ctrl) {
    CROSSLOGD("algorithm control:");

    if (ctrl & EM7180_AC_STANDBY_ENABLE)
        CROSSLOGD("\tstandby enable");
    if (ctrl & EM7180_AC_RAWDATA_ENABLE)
        CROSSLOGD("\trawdata enable");
    if (ctrl & EM7180_AC_HPR_OUTPUT)
        CROSSLOGD("\thpr output");
    if (ctrl & EM7180_AC_6AXIS_ENABLE)
        CROSSLOGD("\t6-axis enable");
    if (ctrl & EM7180_AC_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (ctrl & EM7180_AC_ENU_OUTPUT)
        CROSSLOGD("\tenu output");
    if (ctrl & EM7180_AC_DIS_GYR_WHEN_STILL)
        CROSSLOGD("\tdisable gyro when still");
    if (ctrl & EM7180_AC_PARAM_TRANSFER)
        CROSSLOGD("\tparam transfer");
}

void em7180_print_algorithm_status(uint8_t status) {
    CROSSLOGD("algorithm status:");

    if (status & EM7180_AS_STANDBY)
        CROSSLOGD("\tstandby");
    if (status & EM7180_AS_SLOW)
        CROSSLOGD("\talgorithm slow");
    if (status & EM7180_AS_STILLNESS)
        CROSSLOGD("\tstillness");
    if (status & EM7180_AS_CAL_STABLE)
        CROSSLOGD("\tcal stable");
    if (status & EM7180_AS_MAG_TRANSIENT)
        CROSSLOGD("\tmag transient");
    if (status & EM7180_AS_UNRELIABLE)
        CROSSLOGD("\tunreliable sensor data");
    if (status & EM7180_AS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_AS_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_event_status(uint8_t status) {
    CROSSLOGD("event status:");

    if (status & EM7180_EVENT_CPURESET)
        CROSSLOGD("\tcpu reset");
    if (status & EM7180_EVENT_ERROR)
        CROSSLOGD("\terror");
    if (status & EM7180_EVENT_QUAT_RES)
        CROSSLOGD("\tquaternion result");
    if (status & EM7180_EVENT_MAG_RES)
        CROSSLOGD("\tmag result");
    if (status & EM7180_EVENT_ACCEL_RES)
        CROSSLOGD("\taccel result");
    if (status & EM7180_EVENT_GYRO_RES)
        CROSSLOGD("\tgyro result");
    if (status & EM7180_EVENT_BARO_RES)
        CROSSLOGD("\tbaro result");
    if (status & EM7180_EVENT_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_sensor_status(uint8_t status) {
    CROSSLOGD("sensor status:");

    if (status & EM7180_SENSORSTATUS_MAG_NACK)
        CROSSLOGD("\tmagnetometer NACK");
    if (status & EM7180_SENSORSTATUS_ACCEL_NACK)
        CROSSLOGD("\taccelerometer NACK");
    if (status & EM7180_SENSORSTATUS_GYRO_NACK)
        CROSSLOGD("\tgyroscope NACK");
    if (status & EM7180_SENSORSTATUS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_SENSORSTATUS_MAGID_ERROR)
        CROSSLOGD("\tmag id error");
    if (status & EM7180_SENSORSTATUS_ACCELID_ERROR)
        CROSSLOGD("\taccel id error");
    if (status & EM7180_SENSORSTATUS_GYROID_ERROR)
        CROSSLOGD("\tgyro id error");
    if (status & EM7180_SENSORSTATUS_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_error(enum em7180_error error) {
    switch (error) {
    case EM7180_ERR_NONE:
        break;
    case EM7180_ERR_MAG_FAILURE:
        CROSSLOGD("\tmagnetometer failure");
        break;
    case EM7180_ERR_ACC_FAILURE:
        CROSSLOGD("\taccelerometer failure");
        break;
    case EM7180_ERR_GYRO_FAILURE:
        CROSSLOGD("\tgyroscope failure");
        break;
    case EM7180_ERR_MAG_INIT_FAILURE:
        CROSSLOGD("\tmagnetometer init failure");
        break;
    case EM7180_ERR_ACC_INIT_FAILURE:
        CROSSLOGD("\taccelerometer init failure");
        break;
    case EM7180_ERR_GYRO_INIT_FAILURE:
        CROSSLOGD("\tgyroscope init failure");
        break;
    case EM7180_ERR_MATH:
        CROSSLOGD("\tmathematical error");
        break;
    case EM7180_ERR_INVALID_SAMPLE_RATE:
        CROSSLOGD("\tinvalid sample rate");
        break;
    default:
        CROSSLOGD("\tunknown(%02x)", error);
        break;
    }
}
