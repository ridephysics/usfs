#include <inv_mpu.h>

#include "inv_mpu_p.h"

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(struct mpu_state_s *st)
{
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    data = 0;
    if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, &data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, &data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &data))
        return -1;

    if (st->chip_cfg.dmp_on) {
        data = BIT_FIFO_RST | BIT_DMP_RST;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &data))
            return -1;
        delay_ms(50);
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (st->chip_cfg.sensors & INV_XYZ_COMPASS)
            data |= BIT_AUX_IF_EN;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &data))
            return -1;
        if (st->chip_cfg.int_enable)
            data = BIT_DMP_INT_EN;
        else
            data = 0;
        if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, &data))
            return -1;
        data = 0;
        if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, &data))
            return -1;
    } else {
        data = BIT_FIFO_RST;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &data))
            return -1;
        if (st->chip_cfg.bypass_mode || !(st->chip_cfg.sensors & INV_XYZ_COMPASS))
            data = BIT_FIFO_EN;
        else
            data = BIT_FIFO_EN | BIT_AUX_IF_EN;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &data))
            return -1;
        delay_ms(50);
        if (st->chip_cfg.int_enable)
            data = BIT_DATA_RDY_EN;
        else
            data = 0;
        if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, &data))
            return -1;
        if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, &st->chip_cfg.fifo_enable))
            return -1;
    }
    return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(struct mpu_state_s *st, uint8_t *sensors)
{
    sensors[0] = st->chip_cfg.fifo_enable;
    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(struct mpu_state_s *st, uint8_t sensors)
{
    uint8_t prev;
    int result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    if (st->chip_cfg.dmp_on)
        return 0;
    else {
        if (!(st->chip_cfg.sensors))
            return -1;
        prev = st->chip_cfg.fifo_enable;
        st->chip_cfg.fifo_enable = sensors & st->chip_cfg.sensors;
        if (st->chip_cfg.fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors || st->chip_cfg.lp_accel_mode)
            _mpu_set_int_enable(st, 1);
        else
            _mpu_set_int_enable(st, 0);
        if (sensors) {
            if (mpu_reset_fifo(st)) {
                st->chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }

    return result;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in microseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(struct mpu_state_s *st, int16_t *gyro, int16_t *accel, uint64_t *timestamp,
        uint8_t *sensors, uint8_t *more)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    uint8_t data[MAX_PACKET_LENGTH];
    uint8_t packet_size = 0;
    uint16_t fifo_count, index = 0;

    if (st->chip_cfg.dmp_on)
        return -1;

    sensors[0] = 0;
    if (!st->chip_cfg.sensors)
        return -1;
    if (!st->chip_cfg.fifo_enable)
        return -1;

    if (st->chip_cfg.fifo_enable & INV_X_GYRO)
        packet_size += 2;
    if (st->chip_cfg.fifo_enable & INV_Y_GYRO)
        packet_size += 2;
    if (st->chip_cfg.fifo_enable & INV_Z_GYRO)
        packet_size += 2;
    if (st->chip_cfg.fifo_enable & INV_XYZ_ACCEL)
        packet_size += 6;

    if (i2c_read(st, st->hw->addr, st->reg->fifo_count_h, 2, data))
        return -1;
    fifo_count = (data[0] << 8) | data[1];
    if (fifo_count < packet_size)
        return 0;
//    log_i("FIFO count: %hd", fifo_count);
    if (fifo_count > (st->hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st, st->hw->addr, st->reg->int_status, 1, data))
            return -1;
        if (data[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo(st);
            return -2;
        }
    }
    *timestamp = usfs_get_us();

    if (i2c_read(st, st->hw->addr, st->reg->fifo_r_w, packet_size, data))
        return -1;
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if ((index != packet_size) && st->chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= INV_XYZ_ACCEL;
        index += 6;
    }
    if ((index != packet_size) && st->chip_cfg.fifo_enable & INV_X_GYRO) {
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_X_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st->chip_cfg.fifo_enable & INV_Y_GYRO) {
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Y_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st->chip_cfg.fifo_enable & INV_Z_GYRO) {
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Z_GYRO;
        index += 2;
    }

    return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(struct mpu_state_s *st, uint16_t length, uint8_t *data,
    uint8_t *more)
{
    uint8_t tmp[2];
    uint16_t fifo_count;
    if (!st->chip_cfg.dmp_on)
        return -1;
    if (!st->chip_cfg.sensors)
        return -1;

    if (i2c_read(st, st->hw->addr, st->reg->fifo_count_h, 2, tmp))
        return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < length) {
        more[0] = 0;
        return -1;
    }
    if (fifo_count > (st->hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st, st->hw->addr, st->reg->int_status, 1, tmp))
            return -1;
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo(st);
            return -2;
        }
    }

    if (i2c_read(st, st->hw->addr, st->reg->fifo_r_w, length, data))
        return -1;
    more[0] = fifo_count / length - 1;
    return 0;
}
