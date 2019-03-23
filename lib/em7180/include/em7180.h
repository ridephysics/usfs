#ifndef EM7180_H
#define EM7180_H

#include <crossi2c.h>

struct em7180 {
    struct crossi2c_bus *i2cbus;
};

enum {
    EM7180_FF_BAROMETER = 0x1,
    EM7180_FF_HUMIDITY_SENSOR = 0x2,
    EM7180_FF_TEMP_SENSOR = 0x4,
    EM7180_FF_CUSTOM_SENSOR_1 = 0x8,
    EM7180_FF_CUSTOM_SENSOR_2 = 0x10,
    EM7180_FF_CUSTOM_SENSOR_3 = 0x20,
    EM7180_FF_RESERVED1 = 0x40,
    EM7180_FF_RESERVED2 = 0x80,
};

enum {
    EM7180_SS_EEPROM_DETECTED = 0x1,
    EM7180_SS_EEPROM_CFG_UPLOADED = 0x2,
    EM7180_SS_EEPROM_CRC_ERR = 0x4,
    EM7180_SS_INITIALIZED = 0x8,
    EM7180_SS_NO_EEPROM = 0x10,
    EM7180_SS_RESERVED1 = 0x20,
    EM7180_SS_RESERVED2 = 0x40,
    EM7180_SS_RESERVED3 = 0x80,
};



int em7180_create(struct em7180 *dev, struct crossi2c_bus *i2cbus);
int em7180_destroy(struct em7180 *dev);

int em7180_get_romversion(struct em7180 *dev, uint16_t *pversion);
int em7180_get_ramversion(struct em7180 *dev, uint16_t *pversion);
int em7180_get_product_id(struct em7180 *dev, uint8_t *pversion);
int em7180_get_revision_id(struct em7180 *dev, uint8_t *pversion);
int em7180_get_feature_flags(struct em7180 *dev, uint8_t *pflags);
int em7180_get_sentral_status(struct em7180 *dev, uint8_t *pstatus);

void em7180_print_feature_flags(uint8_t flags);
void em7180_print_sentral_status(uint8_t flags);

#endif /* EM7180_H */
