#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"
#include <usfs.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

int em7180_set_algorithm(struct em7180 *dev, uint8_t v) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_ALGORITHM_CTRL, v);
    if (rc) {
        CROSSLOGE("can't set algorithm control");
        return -1;
    }

    return 0;
}

int em7180_passthrough_enter(struct em7180 *dev) {
    int rc;
    uint8_t ptstatus;

    rc = em7180_set_algorithm(dev, EM7180_AC_STANDBY_ENABLE);
    if (rc) return -1;

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
        usfs_usleep(5000);

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
        CROSSLOGD("we're not in passthrough mode");
        return 0;
    }

    CROSSLOGD("exit passthrough mode");

    rc = em7180_write_byte(dev, EM7180_REG_PASSTRHOUGH_CTRL, 0x00);
    if (rc) {
        CROSSLOGE("can't put device out of passthrough mode");
        return -1;
    }

    do {
        usfs_usleep(5000);

        rc = em7180_read(dev, EM7180_REG_PASSTHROUGH_STATUS, &ptstatus, 1);
        if (rc) {
            CROSSLOGE("can't read passthrough status");
            return -1;
        }
    } while (ptstatus & 0x01);

    rc = em7180_set_algorithm(dev, 0x00);
    if (rc) return -1;

    return 0;
}

int em7180_set_run_mode(struct em7180 *dev, bool enabled) {
    int rc;

    rc = em7180_write_byte(dev, EM7180_REG_HOST_CTRL, enabled ? EM7180_HOSTCTRL_RUN_ENABLE : 0);
    if (rc) {
        CROSSLOGE("can't enter run mode");
        return -1;
    }

    return 0;
}
