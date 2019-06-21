#include <usfs/bmp280.h>
#include <string.h>
#include <usfs.h>

#define CROSSLOG_TAG "bmp280"
#include <crosslog.h>

static void bmp280_delay_ms(uint32_t ms) {
    usfs_usleep(ms * 1000);
}

static int8_t bmp280_i2c_reg_write(void *ctx, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    struct crossi2c_bus *i2cbus = ctx;
    uint8_t buf[8];
    size_t buflen = 0;

    if (length > sizeof(buf) - 1)
        return BMP280_E_INVALID_LEN;

    buf[buflen++] = reg_addr;
    memcpy(buf + buflen, reg_data, length);
    buflen += length;

    if (crossi2c_write(i2cbus, i2c_addr, buf, buflen))
        return BMP280_E_COMM_FAIL;

    return BMP280_OK;
}

static int8_t bmp280_i2c_reg_read(void *ctx, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    struct crossi2c_bus *i2cbus = ctx;

    if (crossi2c_write_read(i2cbus, i2c_addr, &reg_addr, 1, reg_data, length))
        return BMP280_E_COMM_FAIL;

    return BMP280_OK;
}

int usfs_bmp280_create(struct bmp280_dev *dev, struct crossi2c_bus *i2cbus) {
    memset(dev, 0, sizeof(*dev));

    dev->userctx = i2cbus;
    dev->delay_ms = bmp280_delay_ms;
    dev->dev_id = BMP280_I2C_ADDR_PRIM;
    dev->intf = BMP280_I2C_INTF;
    dev->read = bmp280_i2c_reg_read;
    dev->write = bmp280_i2c_reg_write;

    return 0;
}
