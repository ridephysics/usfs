#include <inv_mpu.h>

#include "inv_mpu_p.h"

/**
 *  @brief      Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(struct mpu_state_s *st, uint16_t *rate)
{
    if (st->magtype == MAG_TYPE_NONE) {
        rate[0] = 0;
        return -1;
    }

    rate[0] = st->chip_cfg.ak89xx.compass_sample_rate;
    return 0;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(struct mpu_state_s *st, uint16_t rate)
{
    uint8_t div;

    if (st->magtype == MAG_TYPE_NONE) {
        return -1;
    }

    if (!rate || rate > st->chip_cfg.sample_rate || rate > AK89XX_MAX_COMPASS_SAMPLE_RATE)
        return -1;

    div = st->chip_cfg.sample_rate / rate - 1;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s4_ctrl, 1, &div))
        return -1;
    st->chip_cfg.ak89xx.compass_sample_rate = st->chip_cfg.sample_rate / (div + 1);
    return 0;
}

/* This initialization is similar to the one in ak8975.c. */
int _ak89xx_setup_compass(struct mpu_state_s *st)
{
    uint8_t data[4], akm_addr;

    mpu_set_bypass(st, 1);

    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
        int result;
        result = i2c_read(st, akm_addr, AKM_REG_WHOAMI, 1, data);
        if (!result && (data[0] == AKM_WHOAMI))
            break;
    }

    if (akm_addr > 0x0F) {
        /* TODO: Handle this case in all compass-related functions. */
        log_e("Compass not found.");
        return -1;
    }

    st->chip_cfg.ak89xx.compass_addr = akm_addr;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    data[0] = AKM_FUSE_ROM_ACCESS;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    /* Get sensitivity adjustment data from fuse ROM. */
    if (i2c_read(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_ASAX, 3, data))
        return -1;
    st->chip_cfg.ak89xx.mag_sens_adj[0] = (int32_t)data[0] + 128;
    st->chip_cfg.ak89xx.mag_sens_adj[1] = (int32_t)data[1] + 128;
    st->chip_cfg.ak89xx.mag_sens_adj[2] = (int32_t)data[2] + 128;

    data[0] = AKM_POWER_DOWN;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, data))
        return -1;
    delay_ms(1);

    mpu_set_bypass(st, 0);

    /* Set up master mode, master clock, and ES bit. */
    data[0] = 0x40;
    if (i2c_write(st, st->hw->addr, st->reg->i2c_mst, 1, data))
        return -1;

    /* Slave 0 reads from AKM data registers. */
    data[0] = BIT_I2C_READ | st->chip_cfg.ak89xx.compass_addr;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s0_addr, 1, data))
        return -1;

    /* Compass reads start at this register. */
    data[0] = AKM_REG_ST1;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s0_reg, 1, data))
        return -1;

    /* Enable slave 0, 8-byte reads. */
    data[0] = BIT_SLAVE_EN | 8;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s0_ctrl, 1, data))
        return -1;

    /* Slave 1 changes AKM measurement mode. */
    data[0] = st->chip_cfg.ak89xx.compass_addr;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s1_addr, 1, data))
        return -1;

    /* AKM measurement mode register. */
    data[0] = AKM_REG_CNTL;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s1_reg, 1, data))
        return -1;

    /* Enable slave 1, 1-byte writes. */
    data[0] = BIT_SLAVE_EN | 1;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s1_ctrl, 1, data))
        return -1;

    /* Set slave 1 data. */
    data[0] = AKM_SINGLE_MEASUREMENT;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s1_do, 1, data))
        return -1;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data[0] = 0x03;
    if (i2c_write(st, st->hw->addr, st->reg->ak89xx.i2c_delay_ctrl, 1, data))
        return -1;

    if (st->mputype == MPU_TYPE_MPU6050) {
        /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
        data[0] = BIT_I2C_MST_VDDIO;
        if (i2c_write(st, st->hw->addr, st->reg->ak89xx.yg_offs_tc, 1, data))
            return -1;
    }

    return 0;
}

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in microseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp)
{
    uint8_t tmp[9];

    if (st->magtype == MAG_TYPE_NONE) {
        return -1;
    }

    if (!(st->chip_cfg.sensors & INV_XYZ_COMPASS))
        return -1;

    if (st->mag_bypass) {
        if (i2c_read(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_ST1, 8, tmp))
            return -1;
        tmp[8] = AKM_SINGLE_MEASUREMENT;
        if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, tmp+8))
            return -1;
    }
    else {
        if (i2c_read(st, st->hw->addr, st->reg->ak89xx.raw_compass, 8, tmp))
            return -1;
    }

    if (st->magtype == MAG_TYPE_AK8975) {
        /* AK8975 doesn't have the overrun error bit. */
        if (!(tmp[0] & AKM_DATA_READY))
            return -2;
        if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
            return -3;
    }
    else if (st->magtype == MAG_TYPE_AK8963) {
        /* AK8963 doesn't have the data read error bit. */
        if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
            return -2;
        if (tmp[7] & AKM_OVERFLOW)
            return -3;
    }

    data[0] = (tmp[2] << 8) | tmp[1];
    data[1] = (tmp[4] << 8) | tmp[3];
    data[2] = (tmp[6] << 8) | tmp[5];

    data[0] = ((int32_t)data[0] * st->chip_cfg.ak89xx.mag_sens_adj[0]) >> 8;
    data[1] = ((int32_t)data[1] * st->chip_cfg.ak89xx.mag_sens_adj[1]) >> 8;
    data[2] = ((int32_t)data[2] * st->chip_cfg.ak89xx.mag_sens_adj[2]) >> 8;

    if (timestamp)
        *timestamp = usfs_get_us();
    return 0;
}

/**
 *  @brief      Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(struct mpu_state_s *st, uint16_t *fsr)
{
    if (st->magtype == MAG_TYPE_NONE) {
        return -1;
    }

    fsr[0] = st->hw->ak89xx.compass_fsr;
    return 0;
}
