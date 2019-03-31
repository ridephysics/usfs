#include <crossi2c/linux.h>
#include <em7180.h>

#define CROSSLOG_TAG "test"
#include <crosslog.h>

static struct crossi2c_bus i2cbus;
static struct em7180 em7180;

int main(void) {
    int rc;
    int ret = -1;
    uint16_t romversion;
    uint16_t ramversion;
    uint8_t productid;
    uint8_t revid;
    uint8_t flags;
    uint8_t status = 0;
    size_t count;

    rc = crossi2c_linux_create(&i2cbus, "/dev/i2c-9");
    if (rc) return -1;

    rc = em7180_create(&em7180, &i2cbus);
    if (rc) goto out_i2sbus_destroy;


    rc = em7180_get_romversion(&em7180, &romversion);
    if (rc) goto out_em7180_destroy;

    rc = em7180_get_ramversion(&em7180, &ramversion);
    if (rc) goto out_em7180_destroy;

    rc = em7180_get_product_id(&em7180, &productid);
    if (rc) goto out_em7180_destroy;

    rc = em7180_get_revision_id(&em7180, &revid);
    if (rc) goto out_em7180_destroy;

    CROSSLOGI("romversion: %04x ramversion: %04x productid: %02x revid: %02x",
        romversion, ramversion, productid, revid);

    rc = em7180_get_feature_flags(&em7180, &flags);
    if (rc) goto out_em7180_destroy;

    em7180_print_feature_flags(flags);

    for (count = 0; !status && count < 10; count++) {
        rc = em7180_get_sentral_status(&em7180, &status);
        if (rc) goto out_em7180_destroy;

        em7180_print_sentral_status(status);
    }

    if (status & EM7180_SS_EEPROM_CRC_ERR)
        CROSSLOGE("EEPROM CONFIG ERROR");

    ret = 0;


out_em7180_destroy:
    rc = em7180_destroy(&em7180);
    if (rc) {
        CROSSLOGW("can't destroy em7180 dev");
    }

out_i2sbus_destroy:
    rc = crossi2c_destroy(&i2cbus);
    if (rc) {
        CROSSLOGW("can't destroy i2cbus");
    }

    if (ret) {
        CROSSLOGE("EXIT WITH ERROR");
    }

    return ret;
}
