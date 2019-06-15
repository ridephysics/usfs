#include <inv_mpu.h>

#include "inv_mpu_p.h"

/**
 *  @brief      Read biases to the accel bias 6500 registers.
 *  This function reads from the MPU6500 accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP 
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int mpu_read_6500_accel_bias(struct mpu_state_s *st, int32_t *accel_bias) {
    uint8_t data[6];
    if (i2c_read(st, st->hw->addr, 0x77, 2, &data[0]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x7A, 2, &data[2]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x7D, 2, &data[4]))
        return -1;
    accel_bias[0] = ((int32_t)data[0]<<8) | data[1];
    accel_bias[1] = ((int32_t)data[2]<<8) | data[3];
    accel_bias[2] = ((int32_t)data[4]<<8) | data[5];
    return 0;
}

/**
 *  @brief      Read biases to the accel bias 6050 registers.
 *  This function reads from the MPU6050 accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP 
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int mpu_read_6050_accel_bias(struct mpu_state_s *st, int32_t *accel_bias) {
    uint8_t data[6];
    if (i2c_read(st, st->hw->addr, 0x06, 2, &data[0]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x08, 2, &data[2]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x0A, 2, &data[4]))
        return -1;
    accel_bias[0] = ((int32_t)data[0]<<8) | data[1];
    accel_bias[1] = ((int32_t)data[2]<<8) | data[3];
    accel_bias[2] = ((int32_t)data[4]<<8) | data[5];
    return 0;
}

int mpu_read_6500_gyro_bias(struct mpu_state_s *st, int32_t *gyro_bias) {
    uint8_t data[6];
    if (i2c_read(st, st->hw->addr, 0x13, 2, &data[0]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x15, 2, &data[2]))
        return -1;
    if (i2c_read(st, st->hw->addr, 0x17, 2, &data[4]))
        return -1;
    gyro_bias[0] = ((int32_t)data[0]<<8) | data[1];
    gyro_bias[1] = ((int32_t)data[2]<<8) | data[3];
    gyro_bias[2] = ((int32_t)data[4]<<8) | data[5];
    return 0;
}

/**
 *  @brief      Push biases to the gyro bias 6500/6050 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-1000dps format.
 *  @param[in]  gyro_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_gyro_bias_reg(struct mpu_state_s *st, int32_t *gyro_bias)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int i=0;
    for(i=0;i<3;i++) {
        gyro_bias[i]= (-gyro_bias[i]);
    }
    data[0] = (gyro_bias[0] >> 8) & 0xff;
    data[1] = (gyro_bias[0]) & 0xff;
    data[2] = (gyro_bias[1] >> 8) & 0xff;
    data[3] = (gyro_bias[1]) & 0xff;
    data[4] = (gyro_bias[2] >> 8) & 0xff;
    data[5] = (gyro_bias[2]) & 0xff;
    if (i2c_write(st, st->hw->addr, 0x13, 2, &data[0]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x15, 2, &data[2]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x17, 2, &data[4]))
        return -1;
    return 0;
}

/**
 *  @brief      Push biases to the accel bias 6050 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias_6050_reg(struct mpu_state_s *st, const int32_t *accel_bias) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t accel_reg_bias[3] = {0, 0, 0};

    if(mpu_read_6050_accel_bias(st, accel_reg_bias))
        return -1;

    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

    if (i2c_write(st, st->hw->addr, 0x06, 2, &data[0]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x08, 2, &data[2]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x0A, 2, &data[4]))
        return -1;

    return 0;
}

/**
 *  @brief      Push biases to the accel bias 6500 registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias_6500_reg(struct mpu_state_s *st, const int32_t *accel_bias) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t accel_reg_bias[3] = {0, 0, 0};

    if(mpu_read_6500_accel_bias(st, accel_reg_bias))
        return -1;

    // Preserve bit 0 of factory value (for temperature compensation)
    accel_reg_bias[0] -= (accel_bias[0] & ~1);
    accel_reg_bias[1] -= (accel_bias[1] & ~1);
    accel_reg_bias[2] -= (accel_bias[2] & ~1);

    data[0] = (accel_reg_bias[0] >> 8) & 0xff;
    data[1] = (accel_reg_bias[0]) & 0xff;
    data[2] = (accel_reg_bias[1] >> 8) & 0xff;
    data[3] = (accel_reg_bias[1]) & 0xff;
    data[4] = (accel_reg_bias[2] >> 8) & 0xff;
    data[5] = (accel_reg_bias[2]) & 0xff;

    if (i2c_write(st, st->hw->addr, 0x77, 2, &data[0]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x7A, 2, &data[2]))
        return -1;
    if (i2c_write(st, st->hw->addr, 0x7D, 2, &data[4]))
        return -1;

    return 0;
}
