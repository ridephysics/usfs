#include <inv_mpu.h>

#include "inv_mpu_p.h"

/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
int _mpu_set_int_enable(struct mpu_state_s *st, uint8_t enable)
{
    uint8_t tmp;

    if (st->chip_cfg.dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, &tmp))
            return -1;
        st->chip_cfg.int_enable = tmp;
    } else {
        if (!st->chip_cfg.sensors)
            return -1;
        if (enable && st->chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, &tmp))
            return -1;
        st->chip_cfg.int_enable = tmp;
    }
    return 0;
}

/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(struct mpu_state_s *st, int16_t *status)
{
    uint8_t tmp[2];
    if (!st->chip_cfg.sensors)
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->dmp_int_status, 2, tmp))
        return -1;
    status[0] = (tmp[0] << 8) | tmp[1];
    return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(struct mpu_state_s *st, uint8_t active_low)
{
    st->chip_cfg.active_low_int = active_low;
    return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(struct mpu_state_s *st, uint8_t enable)
{
    uint8_t tmp;
    if (st->chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (st->chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (st->chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    if (i2c_write(st, st->hw->addr, st->reg->int_pin_cfg, 1, &tmp))
        return -1;
    st->chip_cfg.latched_int = enable;
    return 0;
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *  The behaviour of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data and detects any large change
 *  over a short period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(struct mpu_state_s *st, uint16_t thresh, uint8_t time,
    uint16_t lpa_freq)
{
    uint8_t data[3], temp[1];;

    if (lpa_freq) {
        if (!time)
            /* Minimum duration must be 1ms. */
            time = 1;

        if (st->mputype == MPU_TYPE_MPU6500) {
            if (lpa_freq > 640)
                /* At this point, the chip has not been re-configured, so the
                 * function can safely exit.
                 */
                return -1;
        }

        if (!st->chip_cfg.int_motion_only) {
            /* Store current settings for later. */
            if (st->chip_cfg.dmp_on) {
                mpu_set_dmp_state(st, 0);
                st->chip_cfg.cache.dmp_on = 1;
            } else
                st->chip_cfg.cache.dmp_on = 0;
            mpu_get_gyro_fsr(st, &st->chip_cfg.cache.gyro_fsr);
            mpu_get_accel_fsr(st, &st->chip_cfg.cache.accel_fsr);
            mpu_get_lpf(st, &st->chip_cfg.cache.lpf);
            mpu_get_sample_rate(st, &st->chip_cfg.cache.sample_rate);
            st->chip_cfg.cache.sensors_on = st->chip_cfg.sensors;
            mpu_get_fifo_config(st, &st->chip_cfg.cache.fifo_sensors);
        }

        if (st->mputype == MPU_TYPE_MPU6500) {
            uint8_t thresh_hw;

            /* 1LSb = 4mg. */
            if (thresh > 1020)
                thresh_hw = 255;
            else if (thresh < 4)
                thresh_hw = 1;
            else
                thresh_hw = thresh >> 2;

            /* Disable hardware interrupts. */
            _mpu_set_int_enable(st, 0);

            /* Enter full-power accel-only mode, no FIFO/DMP. */
            data[0] = 0;
            data[1] = 0;
            data[2] = BIT_STBY_XYZG;
            if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 3, data))
                goto lp_int_restore;

            /* Set motion threshold. */
            data[0] = thresh_hw;
            if (i2c_write(st, st->hw->addr, st->reg->motion_thr, 1, data))
                goto lp_int_restore;

            /* Set wake frequency. */
            if (lpa_freq == 1)
                data[0] = INV_LPA_MPU6500_1_25HZ;
            else if (lpa_freq == 2)
                data[0] = INV_LPA_MPU6500_2_5HZ;
            else if (lpa_freq <= 5)
                data[0] = INV_LPA_MPU6500_5HZ;
            else if (lpa_freq <= 10)
                data[0] = INV_LPA_MPU6500_10HZ;
            else if (lpa_freq <= 20)
                data[0] = INV_LPA_MPU6500_20HZ;
            else if (lpa_freq <= 40)
                data[0] = INV_LPA_MPU6500_40HZ;
            else if (lpa_freq <= 80)
                data[0] = INV_LPA_MPU6500_80HZ;
            else if (lpa_freq <= 160)
                data[0] = INV_LPA_MPU6500_160HZ;
            else if (lpa_freq <= 320)
                data[0] = INV_LPA_MPU6500_320HZ;
            else
                data[0] = INV_LPA_MPU6500_640HZ;
            if (i2c_write(st, st->hw->addr, st->reg->lp_accel_odr, 1, data))
                goto lp_int_restore;

            /* Enable motion interrupt (MPU6500 version). */
            data[0] = BITS_WOM_EN;
            if (i2c_write(st, st->hw->addr, st->reg->accel_intel, 1, data))
                goto lp_int_restore;
            /* Enable LP ACCEL mode, ACCEL_FCHOICE_B=1*/
            if (i2c_read(st, st->hw->addr, st->reg->accel_cfg2, 1, temp))
                return -1;

            /*Bypass accel DLPF. */
            data[0] = BIT_MPU6500_ACCL_FC_B | temp[0];
            if (i2c_write(st, st->hw->addr, st->reg->accel_cfg2, 1, data))
                goto lp_int_restore;

            /* Enable interrupt. */
            data[0] = BIT_MOT_INT_EN;
            if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, data))
                goto lp_int_restore;

            /* Enable cycle mode. */
            data[0] = BIT_LPA_CYCLE;
            if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, data))
                goto lp_int_restore;
            st->chip_cfg.int_motion_only = 1;
            return 0;
        }
    } else {
        /* Don't "restore" the previous state if no state has been saved. */
        size_t ii;
        char *cache_ptr = (char*)&st->chip_cfg.cache;
        for (ii = 0; ii < sizeof(st->chip_cfg.cache); ii++) {
            if (cache_ptr[ii] != 0)
                goto lp_int_restore;
        }
        /* If we reach this point, motion interrupt mode hasn't been used yet. */
        return -1;
    }
lp_int_restore:
    /* Set to invalid values to ensure no I2C writes are skipped. */
    st->chip_cfg.gyro_fsr = 0xFF;
    st->chip_cfg.accel_fsr = 0xFF;
    st->chip_cfg.lpf = 0xFF;
    st->chip_cfg.sample_rate = 0xFFFF;
    st->chip_cfg.sensors = 0xFF;
    st->chip_cfg.fifo_enable = 0xFF;
    st->chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_sensors(st, st->chip_cfg.cache.sensors_on);
    mpu_set_gyro_fsr(st, st->chip_cfg.cache.gyro_fsr);
    mpu_set_accel_fsr(st, st->chip_cfg.cache.accel_fsr);
    mpu_set_lpf(st, st->chip_cfg.cache.lpf);
    mpu_set_sample_rate(st, st->chip_cfg.cache.sample_rate);
    mpu_configure_fifo(st, st->chip_cfg.cache.fifo_sensors);

    if (st->chip_cfg.cache.dmp_on)
        mpu_set_dmp_state(st, 1);

    if (st->mputype == MPU_TYPE_MPU6500) {
        /* Disable motion interrupt (MPU6500 version). */
        data[0] = 0;
        if (i2c_write(st, st->hw->addr, st->reg->accel_intel, 1, data))
            goto lp_int_restore;
    }

    st->chip_cfg.int_motion_only = 0;
    return 0;
}
