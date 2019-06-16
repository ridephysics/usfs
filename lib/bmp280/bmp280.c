#include <usfs/bmp280.h>
#include <string.h>
#include <usfs.h>

#define CROSSLOG_TAG "bmp280"
#include <crosslog.h>

static void bmp280_delay_ms(uint32_t ms) {
    usfs_usleep(ms * 1000);
}

static int8_t bmp280_i2c_reg_write(void *ctx, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    struct usfs_bmp280 *dev = ctx;
    uint8_t buf[8];
    size_t buflen = 0;

    if (length > sizeof(buf) - 1)
        return BMP280_E_INVALID_LEN;

    buf[buflen++] = reg_addr;
    memcpy(buf + buflen, reg_data, length);
    buflen += length;

    if (crossi2c_write(dev->i2cbus, i2c_addr, buf, buflen))
        return BMP280_E_COMM_FAIL;

    return BMP280_OK;
}

static int8_t bmp280_i2c_reg_read(void *ctx, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    struct usfs_bmp280 *dev = ctx;

    if (crossi2c_write_read(dev->i2cbus, i2c_addr, &reg_addr, 1, reg_data, length))
        return BMP280_E_COMM_FAIL;

    return BMP280_OK;
}

int usfs_bmp280_create(struct usfs_bmp280 *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));
    dev->i2cbus = i2cbus;

    dev->bmp.userctx = dev;
    dev->bmp.delay_ms = bmp280_delay_ms;
    dev->bmp.dev_id = BMP280_I2C_ADDR_PRIM;
    dev->bmp.intf = BMP280_I2C_INTF;
    dev->bmp.read = bmp280_i2c_reg_read;
    dev->bmp.write = bmp280_i2c_reg_write;

    return 0;
}

int usfs_bmp280_init(struct usfs_bmp280 *dev) {
    int8_t brc;
    struct bmp280_config conf;

    brc = bmp280_init(&dev->bmp);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_init: %d", brc);
        return -1;
    }

    brc = bmp280_get_config(&conf, &dev->bmp);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_config: %d", brc);
        return -1;
    }

    conf.filter = BMP280_FILTER_COEFF_2;
    conf.os_temp = BMP280_OS_4X;
    conf.os_pres = BMP280_OS_4X;
    conf.odr = BMP280_ODR_1000_MS;

    brc = bmp280_set_config(&conf, &dev->bmp);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_config: %d", brc);
        return -1;
    }

    brc = bmp280_set_power_mode(BMP280_NORMAL_MODE, &dev->bmp);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_power_mode: %d", brc);
        return -1;
    }

    return 0;
}

int usfs_bmp280_destroy(struct usfs_bmp280 *dev) {
    memset(dev, 0, sizeof(*dev));
    return 0;
}
