#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

int em7180_get_romversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_ROM_VERSION_0, &version_raw, sizeof(version_raw));
    if (rc) {
        CROSSLOGE("can't read rom version");
        return -1;
    }

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion) {
    int rc;
    uint8_t version_raw[2];

    rc = em7180_read(dev, EM7180_REG_RAM_VERSION_0, &version_raw, sizeof(version_raw));
    if (rc) {
        CROSSLOGE("can't read ram version");
        return -1;
    }

    *pversion = (version_raw[0] & 0xff) << 8 | (version_raw[1] & 0xff);

    return 0;
}

int em7180_get_product_id(struct em7180 *dev, uint8_t *pversion) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_PRODUCT_ID, pversion, sizeof(*pversion));
    if (rc) {
        CROSSLOGE("can't read product id");
        return -1;
    }

    return 0;
}

int em7180_get_revision_id(struct em7180 *dev, uint8_t *pversion) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_REVISION_ID, pversion, sizeof(*pversion));
    if (rc) {
        CROSSLOGE("can't read revision id");
        return -1;
    }

    return 0;
}

int em7180_get_feature_flags(struct em7180 *dev, uint8_t *pflags) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_FEATURE_FLAGS, pflags, sizeof(*pflags));
    if (rc) {
        CROSSLOGE("can't read feature flags");
        return -1;
    }

    return 0;
}

int em7180_get_sentral_status(struct em7180 *dev, uint8_t *pstatus) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_SENTRAL_STATUS, pstatus, sizeof(*pstatus));
    if (rc) {
        CROSSLOGE("can't read sentral status");
        return -1;
    }

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

int em7180_get_sensor_status(struct em7180 *dev, uint8_t *pstatus) {
    int rc;

    rc = em7180_read(dev, EM7180_REG_SENSOR_STATUS, pstatus, 1);
    if (rc) {
        CROSSLOGE("can't read sensor status");
        return -1;
    }

    return 0;
}
