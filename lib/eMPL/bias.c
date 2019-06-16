#include <inv_mpu.h>

#include "inv_mpu_p.h"

int mpu_read_gyro_bias(struct mpu_state_s *st, int32_t *gyro_bias) {
    uint8_t data[6];
    if (i2c_read(st, st->hw->addr, st->reg->gyro_offset_x_h, 2, &data[0]))
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->gyro_offset_y_h, 2, &data[2]))
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->gyro_offset_z_h, 2, &data[4]))
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
int mpu_set_gyro_bias_reg(struct mpu_state_s *st, const int32_t *gyro_bias)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};

    data[0] = ((-gyro_bias[0]) >> 8) & 0xff;
    data[1] = (-gyro_bias[0]) & 0xff;
    data[2] = ((-gyro_bias[1]) >> 8) & 0xff;
    data[3] = (-gyro_bias[1]) & 0xff;
    data[4] = ((-gyro_bias[2]) >> 8) & 0xff;
    data[5] = (-gyro_bias[2]) & 0xff;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_offset_x_h, 2, &data[0]))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_offset_y_h, 2, &data[2]))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_offset_z_h, 2, &data[4]))
        return -1;
    return 0;
}

/**
 *  @brief      Read biases to the accel bias registers.
 *  This function reads from the accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP 
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
int mpu_read_accel_bias(struct mpu_state_s *st, int32_t *accel_bias) {
    uint8_t data[6];
    if (i2c_read(st, st->hw->addr, st->reg->accel_offset_x_h, 2, &data[0]))
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->accel_offset_y_h, 2, &data[2]))
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->accel_offset_z_h, 2, &data[4]))
        return -1;
    accel_bias[0] = ((int32_t)data[0]<<8) | data[1];
    accel_bias[1] = ((int32_t)data[2]<<8) | data[3];
    accel_bias[2] = ((int32_t)data[4]<<8) | data[5];
    return 0;
}

/**
 *  @brief      Push biases to the accel bias registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values. Bias inputs are LSB
 *  in +-16G format.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias_reg(struct mpu_state_s *st, const int32_t *accel_bias) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t accel_reg_bias[3] = {0, 0, 0};

    if(mpu_read_accel_bias(st, accel_reg_bias))
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

    if (i2c_write(st, st->hw->addr, st->reg->accel_offset_x_h, 2, &data[0]))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->accel_offset_y_h, 2, &data[2]))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->accel_offset_z_h, 2, &data[4]))
        return -1;

    return 0;
}
