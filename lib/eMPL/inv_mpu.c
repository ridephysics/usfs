/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <string.h>
#include <inv_mpu.h>

#include "inv_mpu_p.h"

static const struct mpu_gyro_reg_s mpu6050_reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70,
    .gyro_offset_x_h    = 0x13,
    .gyro_offset_y_h    = 0x15,
    .gyro_offset_z_h    = 0x17,
    .accel_offset_x_h   = 0x06,
    .accel_offset_y_h   = 0x08,
    .accel_offset_z_h   = 0x0a,

    .ak89xx = {
        .raw_compass   = 0x49,
        .yg_offs_tc     = 0x01,
        .s0_addr        = 0x25,
        .s0_reg         = 0x26,
        .s0_ctrl        = 0x27,
        .s1_addr        = 0x28,
        .s1_reg         = 0x29,
        .s1_ctrl        = 0x2A,
        .s4_ctrl        = 0x34,
        .s0_do          = 0x63,
        .s1_do          = 0x64,
        .i2c_delay_ctrl = 0x67
    }
};

static const struct mpu_hw_s mpu6050_hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256,

    .ak89xx = {
        .compass_fsr    = AK8975_FSR
    }
};

static const struct mpu_test_s mpu6050_test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/16,
    .reg_rate_div   = 0,    /* 1kHz. */
    .reg_lpf        = 1,    /* 188Hz. */
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x18, /* 16g. */
    .wait_ms        = 50,
    .packet_thresh  = 5,    /* 5% */
    .min_dps        = 10.f,
    .max_dps        = 105.f,
    .max_gyro_var   = 0.14f,
    .min_g          = 0.3f,
    .max_g          = 0.95f,
    .max_accel_var  = 0.14f
};

static const struct mpu_gyro_reg_s mpu6500_reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .accel_cfg2     = 0x1D,
    .lp_accel_odr   = 0x1E,
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .accel_intel    = 0x69,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x77,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70,
    .gyro_offset_x_h    = 0x13,
    .gyro_offset_y_h    = 0x15,
    .gyro_offset_z_h    = 0x17,
    .accel_offset_x_h   = 0x77,
    .accel_offset_y_h   = 0x7a,
    .accel_offset_z_h   = 0x7d,

    .ak89xx = {
        .raw_compass   = 0x49,
        .s0_addr        = 0x25,
        .s0_reg         = 0x26,
        .s0_ctrl        = 0x27,
        .s1_addr        = 0x28,
        .s1_reg         = 0x29,
        .s1_ctrl        = 0x2A,
        .s4_ctrl        = 0x34,
        .s0_do          = 0x63,
        .s1_do          = 0x64,
        .i2c_delay_ctrl = 0x67
    }
};
static const struct mpu_hw_s mpu6500_hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 128,
    .temp_sens      = 321,
    .temp_offset    = 0,
    .bank_size      = 256,
    .ak89xx = {
        .compass_fsr    = AK8963_FSR
    }
};

static const struct mpu_test_s mpu6500_test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/2,  //FSR = +-2G = 16384 LSB/G
    .reg_rate_div   = 0,    /* 1kHz. */
    .reg_lpf        = 2,    /* 92Hz low pass filter*/
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x0,  /* Accel FSR setting = 2g. */
    .wait_ms        = 200,   //200ms stabilization time
    .packet_thresh  = 200,    /* 200 samples */
    .min_dps        = 20.f,  //20 dps for Gyro Criteria C
    .max_dps        = 60.f, //Must exceed 60 dps threshold for Gyro Criteria B
    .max_gyro_var   = .5f, //Must exceed +50% variation for Gyro Criteria A
    .min_g          = .225f, //Accel must exceed Min 225 mg for Criteria B
    .max_g          = .675f, //Accel cannot exceed Max 675 mg for Criteria B
    .max_accel_var  = .5f,  //Accel must be within 50% variation for Criteria A

    .mpu6500 = {
        .max_g_offset   = .5f,   //500 mg for Accel Criteria C
        .sample_wait_ms = 10    //10ms sample time wait
    }
};

/**
 *  @brief      Register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(struct mpu_state_s *st)
{
    uint8_t ii;
    uint8_t data;

    for (ii = 0; ii < st->hw->num_reg; ii++) {
        if (ii == st->reg->fifo_r_w || ii == st->reg->mem_r_w)
            continue;
        if (i2c_read(st, st->hw->addr, ii, 1, &data))
            return -1;
        log_i("%#5x: %#5x", ii, data);
    }
    return 0;
}

/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
int mpu_read_reg(struct mpu_state_s *st, uint8_t reg, void *data)
{
    if (reg == st->reg->fifo_r_w || reg == st->reg->mem_r_w)
        return -1;
    if (reg >= st->hw->num_reg)
        return -1;
    return i2c_read(st, st->hw->addr, reg, 1, data);
}

int mpu_create(struct mpu_state_s *st, enum mpu_type_e mputype, enum mag_type_e magtype, struct crossi2c_bus *i2cbus) {
    memset(st, 0, sizeof(*st));

    st->mputype = mputype;
    st->magtype = magtype;
    st->mag_bypass = 0;
    st->i2cbus = i2cbus;

    if (mputype == MPU_TYPE_MPU6050) {
        st->reg = &mpu6050_reg;
        st->hw = &mpu6050_hw;
        st->test = &mpu6050_test;
    }
    else if (mputype == MPU_TYPE_MPU6500) {
        st->reg = &mpu6500_reg;
        st->hw = &mpu6500_hw;
        st->test = &mpu6500_test;
    }
    else {
        return -1;
    }

    return 0;
}

int mpu_create_nodev(struct mpu_state_s *st, struct mpu_cfg_dump *cfg) {
    int rc;

    rc = mpu_create(st, cfg->mputype, cfg->magtype, NULL);
    if (rc) return rc;

    st->chip_cfg.gyro_fsr = cfg->gyro_fsr;
    st->chip_cfg.accel_fsr = cfg->accel_fsr;

    st->chip_cfg.ak89xx.mag_sens_adj[0] = cfg->mag_sens_adj[0];
    st->chip_cfg.ak89xx.mag_sens_adj[1] = cfg->mag_sens_adj[1];
    st->chip_cfg.ak89xx.mag_sens_adj[2] = cfg->mag_sens_adj[2];

    return 0;
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @return     0 if successful.
 */
int mpu_init(struct mpu_state_s *st)
{
    uint8_t data[6];

    if (i2c_read(st, st->hw->addr, st->reg->who_am_i, 1, data))
        return -1;

    if (data[0] != 0x71) {
        log_e("unsupported device: 0x%02x", data[0]);
        return -1;
    }

    /* Reset device. */
    data[0] = BIT_RESET;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, data))
        return -1;
    delay_ms(100);

    /* Wake up chip. */
    data[0] = 0x00;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, data))
        return -1;

    if (st->mputype == MPU_TYPE_MPU6500) {
        /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
         * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
         */
        data[0] = BIT_FIFO_SIZE_1024;
        if (i2c_write(st, st->hw->addr, st->reg->accel_cfg2, 1, data))
            return -1;
    }

    /* Set to invalid values to ensure no I2C writes are skipped. */
    st->chip_cfg.sensors = 0xFF;
    st->chip_cfg.gyro_fsr = 0xFF;
    st->chip_cfg.accel_fsr = 0xFF;
    st->chip_cfg.lpf = 0xFF;
    st->chip_cfg.sample_rate = 0xFFFF;
    st->chip_cfg.fifo_enable = 0xFF;
    st->chip_cfg.bypass_mode = 0xFF;
    if (st->magtype != MAG_TYPE_NONE) {
        st->chip_cfg.ak89xx.compass_sample_rate = 0xFFFF;
    }
    /* mpu_set_sensors always preserves this setting. */
    st->chip_cfg.clk_src = INV_CLK_PLL;
    /* Handled in next call to mpu_set_bypass. */
    st->chip_cfg.active_low_int = 1;
    st->chip_cfg.latched_int = 0;
    st->chip_cfg.int_motion_only = 0;
    st->chip_cfg.lp_accel_mode = 0;
    memset(&st->chip_cfg.cache, 0, sizeof(st->chip_cfg.cache));
    st->chip_cfg.dmp_on = 0;
    st->chip_cfg.dmp_loaded = 0;
    st->chip_cfg.dmp_sample_rate = 0;

    if (mpu_set_gyro_fsr(st, 2000))
        return -1;
    if (mpu_set_accel_fsr(st, 2))
        return -1;
    if (mpu_set_lpf(st, 42))
        return -1;
    if (mpu_set_sample_rate(st, 50))
        return -1;
    if (mpu_configure_fifo(st, 0))
        return -1;

    if (st->magtype == MAG_TYPE_NONE) {
        /* Already disabled by setup_compass. */
        if (mpu_set_bypass(st, 0))
            return -1;
    }
    else {
        _ak89xx_setup_compass(st);
        if (mpu_set_compass_sample_rate(st, 10))
            return -1;
    }

    mpu_set_sensors(st, 0);
    return 0;
}

void mpu_get_cfg(struct mpu_state_s *st, struct mpu_cfg_dump *cfg) {
    cfg->mputype = st->mputype;
    cfg->magtype = st->magtype;
    cfg->gyro_fsr = st->chip_cfg.gyro_fsr;
    cfg->accel_fsr = st->chip_cfg.accel_fsr;
    cfg->mag_sens_adj[0] = st->chip_cfg.ak89xx.mag_sens_adj[0];
    cfg->mag_sens_adj[1] = st->chip_cfg.ak89xx.mag_sens_adj[1];
    cfg->mag_sens_adj[2] = st->chip_cfg.ak89xx.mag_sens_adj[2];
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int mpu_lp_accel_mode(struct mpu_state_s *st, uint16_t rate)
{
    uint8_t tmp[2];

    if (rate > 40)
        return -1;

    if (!rate) {
        mpu_set_int_latched(st, 0);
        tmp[0] = 0;
        tmp[1] = BIT_STBY_XYZG;
        if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 2, tmp))
            return -1;
        st->chip_cfg.lp_accel_mode = 0;
        return 0;
    }
    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    mpu_set_int_latched(st, 1);

    if (st->mputype == MPU_TYPE_MPU6050) {
        tmp[0] = BIT_LPA_CYCLE;
        if (rate == 1) {
            tmp[1] = INV_LPA_MPU6050_1_25HZ;
            mpu_set_lpf(st, 5);
        } else if (rate <= 5) {
            tmp[1] = INV_LPA_MPU6050_5HZ;
            mpu_set_lpf(st, 5);
        } else if (rate <= 20) {
            tmp[1] = INV_LPA_MPU6050_20HZ;
            mpu_set_lpf(st, 10);
        } else {
            tmp[1] = INV_LPA_MPU6050_40HZ;
            mpu_set_lpf(st, 20);
        }
        tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
        if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 2, tmp))
            return -1;
    }
    else if (st->mputype == MPU_TYPE_MPU6500) {
        /* Set wake frequency. */
        if (rate == 1)
            tmp[0] = INV_LPA_MPU6500_1_25HZ;
        else if (rate == 2)
            tmp[0] = INV_LPA_MPU6500_2_5HZ;
        else if (rate <= 5)
            tmp[0] = INV_LPA_MPU6500_5HZ;
        else if (rate <= 10)
            tmp[0] = INV_LPA_MPU6500_10HZ;
        else if (rate <= 20)
            tmp[0] = INV_LPA_MPU6500_20HZ;
        else if (rate <= 40)
            tmp[0] = INV_LPA_MPU6500_40HZ;
        else if (rate <= 80)
            tmp[0] = INV_LPA_MPU6500_80HZ;
        else if (rate <= 160)
            tmp[0] = INV_LPA_MPU6500_160HZ;
        else if (rate <= 320)
            tmp[0] = INV_LPA_MPU6500_320HZ;
        else
            tmp[0] = INV_LPA_MPU6500_640HZ;
        if (i2c_write(st, st->hw->addr, st->reg->lp_accel_odr, 1, tmp))
            return -1;
        /* Enable LP ACCEL mode, ACCEL_FCHOICE_B=1*/
        if (i2c_read(st, st->hw->addr, st->reg->accel_cfg2, 1, tmp))
               return -1;

        tmp[0] = BIT_MPU6500_ACCL_FC_B | tmp[0];
        if (i2c_write(st, st->hw->addr, st->reg->accel_cfg2, 1, tmp))
            return -1;
        tmp[0] = BIT_LPA_CYCLE;
        if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, tmp))
            return -1;
    }

    st->chip_cfg.sensors = INV_XYZ_ACCEL;
    st->chip_cfg.clk_src = 0;
    st->chip_cfg.lp_accel_mode = 1;
    mpu_configure_fifo(st, 0);

    return 0;
}

/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in microseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp)
{
    uint8_t tmp[6];

    if (!(st->chip_cfg.sensors & INV_XYZ_GYRO))
        return -1;

    if (i2c_read(st, st->hw->addr, st->reg->raw_gyro, 6, tmp))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        *timestamp = usfs_get_us();
    return 0;
}

/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in microseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp)
{
    uint8_t tmp[6];

    if (!(st->chip_cfg.sensors & INV_XYZ_ACCEL))
        return -1;

    if (i2c_read(st, st->hw->addr, st->reg->raw_accel, 6, tmp))
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1];
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        *timestamp = usfs_get_us();
    return 0;
}

/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in microseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(struct mpu_state_s *st, int32_t *data, uint64_t *timestamp)
{
    uint8_t tmp[2];
    int16_t raw;

    if (!(st->chip_cfg.sensors))
        return -1;

    if (i2c_read(st, st->hw->addr, st->reg->temp, 2, tmp))
        return -1;
    raw = (tmp[0] << 8) | tmp[1];
    if (timestamp)
        *timestamp = usfs_get_us();

    data[0] = (int32_t)((35 + ((raw - (float)st->hw->temp_offset) / st->hw->temp_sens)) * 65536L);
    return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(struct mpu_state_s *st, uint16_t *fsr)
{
    switch (st->chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        fsr[0] = 250;
        break;
    case INV_FSR_500DPS:
        fsr[0] = 500;
        break;
    case INV_FSR_1000DPS:
        fsr[0] = 1000;
        break;
    case INV_FSR_2000DPS:
        fsr[0] = 2000;
        break;
    default:
        fsr[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(struct mpu_state_s *st, uint16_t fsr)
{
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 250:
        data = INV_FSR_250DPS << 3;
        break;
    case 500:
        data = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        return -1;
    }

    if (st->chip_cfg.gyro_fsr == (data >> 3))
        return 0;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_cfg, 1, &data))
        return -1;
    st->chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(struct mpu_state_s *st, uint8_t *fsr)
{
    switch (st->chip_cfg.accel_fsr) {
    case INV_FSR_2G:
        fsr[0] = 2;
        break;
    case INV_FSR_4G:
        fsr[0] = 4;
        break;
    case INV_FSR_8G:
        fsr[0] = 8;
        break;
    case INV_FSR_16G:
        fsr[0] = 16;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(struct mpu_state_s *st, uint8_t fsr)
{
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 2:
        data = INV_FSR_2G << 3;
        break;
    case 4:
        data = INV_FSR_4G << 3;
        break;
    case 8:
        data = INV_FSR_8G << 3;
        break;
    case 16:
        data = INV_FSR_16G << 3;
        break;
    default:
        return -1;
    }

    if (st->chip_cfg.accel_fsr == (data >> 3))
        return 0;
    if (i2c_write(st, st->hw->addr, st->reg->accel_cfg, 1, &data))
        return -1;
    st->chip_cfg.accel_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int mpu_get_lpf(struct mpu_state_s *st, uint16_t *lpf)
{
    switch (st->chip_cfg.lpf) {
    case INV_FILTER_188HZ:
        lpf[0] = 188;
        break;
    case INV_FILTER_98HZ:
        lpf[0] = 98;
        break;
    case INV_FILTER_42HZ:
        lpf[0] = 42;
        break;
    case INV_FILTER_20HZ:
        lpf[0] = 20;
        break;
    case INV_FILTER_10HZ:
        lpf[0] = 10;
        break;
    case INV_FILTER_5HZ:
        lpf[0] = 5;
        break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(struct mpu_state_s *st, uint16_t lpf)
{
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    if (lpf >= 188)
        data = INV_FILTER_188HZ;
    else if (lpf >= 98)
        data = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data = INV_FILTER_20HZ;
    else if (lpf >= 10)
        data = INV_FILTER_10HZ;
    else
        data = INV_FILTER_5HZ;

    if (st->chip_cfg.lpf == data)
        return 0;
    if (i2c_write(st, st->hw->addr, st->reg->lpf, 1, &data))
        return -1;

    if (st->mputype == MPU_TYPE_MPU6500) {
        data = BIT_FIFO_SIZE_1024 | data;

        if (i2c_write(st, st->hw->addr, st->reg->accel_cfg2, 1, &data))
                return -1;
    }
    st->chip_cfg.lpf = data;
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(struct mpu_state_s *st, uint16_t *rate)
{
    if (st->chip_cfg.dmp_on)
        return -1;
    else
        rate[0] = st->chip_cfg.sample_rate;
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(struct mpu_state_s *st, uint16_t rate)
{
    uint8_t data;

    if (!(st->chip_cfg.sensors))
        return -1;

    if (st->chip_cfg.dmp_on)
        return -1;
    else {
        if (st->chip_cfg.lp_accel_mode) {
            if (rate && (rate <= 40)) {
                /* Just stay in low-power accel mode. */
                mpu_lp_accel_mode(st, rate);
                return 0;
            }
            /* Requested rate exceeds the allowed frequencies in LP accel mode,
             * switch back to full-power mode.
             */
            mpu_lp_accel_mode(st, 0);
        }
        if (rate < 4)
            rate = 4;
        else if (rate > 1000)
            rate = 1000;

        data = 1000 / rate - 1;
        if (i2c_write(st, st->hw->addr, st->reg->rate_div, 1, &data))
            return -1;

        st->chip_cfg.sample_rate = 1000 / (1 + data);

        if (st->magtype != MAG_TYPE_NONE) {
            mpu_set_compass_sample_rate(st, min(st->chip_cfg.ak89xx.compass_sample_rate, AK89XX_MAX_COMPASS_SAMPLE_RATE));
        }

        /* Automatically set LPF to 1/2 sampling rate. */
        mpu_set_lpf(st, st->chip_cfg.sample_rate >> 1);
        return 0;
    }
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(struct mpu_state_s *st, float *sens)
{
    switch (st->chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        sens[0] = 131.f;
        break;
    case INV_FSR_500DPS:
        sens[0] = 65.5f;
        break;
    case INV_FSR_1000DPS:
        sens[0] = 32.8f;
        break;
    case INV_FSR_2000DPS:
        sens[0] = 16.4f;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(struct mpu_state_s *st, uint16_t *sens)
{
    switch (st->chip_cfg.accel_fsr) {
    case INV_FSR_2G:
        sens[0] = 16384;
        break;
    case INV_FSR_4G:
        sens[0] = 8192;
        break;
    case INV_FSR_8G:
        sens[0] = 4096;
        break;
    case INV_FSR_16G:
        sens[0] = 2048;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(struct mpu_state_s *st, uint8_t *power_on)
{
    if (st->chip_cfg.sensors)
        power_on[0] = 1;
    else
        power_on[0] = 0;
    return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(struct mpu_state_s *st, uint8_t sensors)
{
    uint8_t data;
    uint8_t user_ctrl;

    if (sensors & INV_XYZ_GYRO)
        data = INV_CLK_PLL;
    else if (sensors)
        data = 0;
    else
        data = BIT_SLEEP;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, &data)) {
        st->chip_cfg.sensors = 0;
        return -1;
    }
    st->chip_cfg.clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_2, 1, &data)) {
        st->chip_cfg.sensors = 0;
        return -1;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL))
        /* Latched interrupts only used in LP accel mode. */
        mpu_set_int_latched(st, 0);

    if (st->magtype != MAG_TYPE_NONE) {
        if (st->mag_bypass) {
            if (sensors & INV_XYZ_COMPASS)
                mpu_set_bypass(st, 1);
            else
                mpu_set_bypass(st, 0);
        }
        else {
            if (i2c_read(st, st->hw->addr, st->reg->user_ctrl, 1, &user_ctrl))
                return -1;
            /* Handle AKM power management. */
            if (sensors & INV_XYZ_COMPASS) {
                data = AKM_SINGLE_MEASUREMENT;
                user_ctrl |= BIT_AUX_IF_EN;
            } else {
                data = AKM_POWER_DOWN;
                user_ctrl &= ~BIT_AUX_IF_EN;
            }
            if (st->chip_cfg.dmp_on)
                user_ctrl |= BIT_DMP_EN;
            else
                user_ctrl &= ~BIT_DMP_EN;
            if (i2c_write(st, st->hw->addr, st->reg->ak89xx.s1_do, 1, &data))
                return -1;
            /* Enable/disable I2C master mode. */
            if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &user_ctrl))
                return -1;
        }
    }

    st->chip_cfg.sensors = sensors;
    st->chip_cfg.lp_accel_mode = 0;
    delay_ms(50);
    return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(struct mpu_state_s *st, uint8_t bypass_on)
{
    uint8_t tmp;

    if (st->chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        if (i2c_read(st, st->hw->addr, st->reg->user_ctrl, 1, &tmp))
            return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st->chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (st->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st, st->hw->addr, st->reg->int_pin_cfg, 1, &tmp))
            return -1;
    } else {
        /* Enable I2C master mode if compass is being used. */
        if (i2c_read(st, st->hw->addr, st->reg->user_ctrl, 1, &tmp))
            return -1;
        if (st->chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        if (st->chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (st->chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st, st->hw->addr, st->reg->int_pin_cfg, 1, &tmp))
            return -1;
    }
    st->chip_cfg.bypass_mode = bypass_on;
    return 0;
}

/**
 *  @}
 */

