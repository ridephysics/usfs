#include <inv_mpu.h>

#include "inv_mpu_p.h"

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(struct mpu_state_s *st, uint16_t mem_addr, uint16_t length,
        uint8_t *data)
{
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!st->chip_cfg.sensors)
        return -1;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st->hw->bank_size)
        return -1;

    if (i2c_write(st, st->hw->addr, st->reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(struct mpu_state_s *st, uint16_t mem_addr, uint16_t length,
        uint8_t *data)
{
    uint8_t tmp[2];

    if (!data)
        return -1;
    if (!st->chip_cfg.sensors)
        return -1;

    tmp[0] = (uint8_t)(mem_addr >> 8);
    tmp[1] = (uint8_t)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st->hw->bank_size)
        return -1;

    if (i2c_write(st, st->hw->addr, st->reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_read(st, st->hw->addr, st->reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(struct mpu_state_s *st, uint16_t length, const uint8_t *firmware,
    uint16_t start_addr, uint16_t sample_rate)
{
    uint16_t ii;
    uint16_t this_write;
    /* Must divide evenly into st->hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
    uint8_t cur[LOAD_CHUNK], tmp[2];

    if (st->chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
        return -1;

    if (!firmware)
        return -1;
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (mpu_write_mem(st, ii, this_write, (uint8_t*)&firmware[ii]))
            return -1;
        if (mpu_read_mem(st, ii, this_write, cur))
            return -1;
        if (memcmp(firmware+ii, cur, this_write))
            return -2;
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (i2c_write(st, st->hw->addr, st->reg->prgm_start_h, 2, tmp))
        return -1;

    st->chip_cfg.dmp_loaded = 1;
    st->chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(struct mpu_state_s *st, uint8_t enable)
{
    uint8_t tmp;
    if (st->chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!st->chip_cfg.dmp_loaded)
            return -1;
        /* Disable data ready interrupt. */
        _mpu_set_int_enable(st, 0);
        /* Disable bypass mode. */
        mpu_set_bypass(st, 0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        mpu_set_sample_rate(st, st->chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        tmp = 0;
        i2c_write(st, st->hw->addr, 0x23, 1, &tmp);
        st->chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        _mpu_set_int_enable(st, 1);
        mpu_reset_fifo(st);
    } else {
        /* Disable DMP interrupt. */
        _mpu_set_int_enable(st, 0);
        /* Restore FIFO settings. */
        tmp = st->chip_cfg.fifo_enable;
        i2c_write(st, st->hw->addr, 0x23, 1, &tmp);
        st->chip_cfg.dmp_on = 0;
        mpu_reset_fifo(st);
    }
    return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(struct mpu_state_s *st, uint8_t *enabled)
{
    enabled[0] = st->chip_cfg.dmp_on;
    return 0;
}
