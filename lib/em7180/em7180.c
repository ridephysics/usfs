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

    rc = em7180_read(dev, EM7180_REG_ROM_VERSION, &version_raw, sizeof(version_raw));
    if (rc) return -1;

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_RAM_VERSION, &version_raw, sizeof(version_raw));
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

int em7180_standby_enter(struct em7180 *dev) {
    int rc;
    uint8_t algstatus;

    rc = em7180_read(dev, EM7180_REG_ALGORITHM_STATUS, &algstatus, 1);
    if (rc) {
        CROSSLOGE("can't read alg status");
        return -1;
    }

    if (algstatus & EM7180_AS_STANDBY) {
        CROSSLOGD("we're in standby mode already");
        return 0;
    }

    CROSSLOGD("enter standby mode");

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, EM7180_AC_STANDBY_ENABLE);
    if (rc) {
        CROSSLOGE("can't put device into standby mode");
        return -1;
    }

    // XXX: for some reason the standby status never changes
#if 0
    do {
        usleep(5000);

        rc = em7180_read(dev, EM7180_REG_ALGORITHM_STATUS, &algstatus, 1);
        if (rc) {
            CROSSLOGE("can't read alg status");
            return -1;
        }
    } while (!(algstatus & EM7180_AS_STANDBY));
#else
    usleep(5000);
#endif

    return 0;
}

int em7180_standby_exit(struct em7180 *dev) {
    int rc;
    uint8_t algstatus;

    rc = em7180_read(dev, EM7180_REG_ALGORITHM_STATUS, &algstatus, 1);
    if (rc) {
        CROSSLOGE("can't read alg status");
        return -1;
    }

    if (!(algstatus & EM7180_AS_STANDBY)) {
        CROSSLOGD("we're not in standby mode");
        return 0;
    }

    CROSSLOGD("exit standby mode");

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, 0x00);
    if (rc) {
        CROSSLOGE("can't put device out of standby mode");
        return -1;
    }

    do {
        usleep(5000);

        rc = em7180_read(dev, EM7180_REG_ALGORITHM_STATUS, &algstatus, 1);
        if (rc) {
            CROSSLOGE("can't read alg status");
            return -1;
        }
    } while (algstatus & EM7180_AS_STANDBY);

    return 0;
}

int em7180_passthrough_enter(struct em7180 *dev) {
    int rc;
    uint8_t ptstatus;

    rc = em7180_standby_enter(dev);
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

    rc = em7180_standby_exit(dev);
    if (rc) {
        CROSSLOGE("can't exit standby mode");
        return -1;
    }

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
        CROSSLOGD("\treserved 1")
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
        CROSSLOGD("\treserved 1")
    if (status & EM7180_SS_RESERVED2)
        CROSSLOGD("\treserved 2")
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
        CROSSLOGD("\tenu output")
    if (ctrl & EM7180_AC_DIS_GYR_WHEN_STILL)
        CROSSLOGD("\tdisable gyro when still")
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
        CROSSLOGD("\tunreliable sensor data")
    if (status & EM7180_AS_RESERVED1)
        CROSSLOGD("\treserved 1")
    if (status & EM7180_AS_RESERVED2)
        CROSSLOGD("\treserved 2");
}
