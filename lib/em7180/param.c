#include <em7180.h>
#include <em7180_regs.h>
#include "em7180_p.h"

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

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
