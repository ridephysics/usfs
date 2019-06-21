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
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

#include <crossi2c.h>
#include <stdint.h>

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

enum mpu_type_e {
    MPU_TYPE_MPU6050,
    MPU_TYPE_MPU6500,
};

enum mag_type_e {
    MAG_TYPE_NONE,
    MAG_TYPE_AK8975,
    MAG_TYPE_AK8963,
};


/* Hardware registers needed by driver. */
struct mpu_gyro_reg_s {
    uint8_t who_am_i;
    uint8_t rate_div;
    uint8_t lpf;
    uint8_t prod_id;
    uint8_t user_ctrl;
    uint8_t fifo_en;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;
    uint8_t accel_cfg2;
    uint8_t lp_accel_odr;
    uint8_t motion_thr;
    uint8_t motion_dur;
    uint8_t fifo_count_h;
    uint8_t fifo_r_w;
    uint8_t raw_gyro;
    uint8_t raw_accel;
    uint8_t temp;
    uint8_t int_enable;
    uint8_t dmp_int_status;
    uint8_t int_status;
    uint8_t accel_intel;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    uint8_t int_pin_cfg;
    uint8_t mem_r_w;
    uint8_t accel_offs;
    uint8_t i2c_mst;
    uint8_t bank_sel;
    uint8_t mem_start_addr;
    uint8_t prgm_start_h;
    uint8_t gyro_offset_x_h;
    uint8_t gyro_offset_y_h;
    uint8_t gyro_offset_z_h;
    uint8_t accel_offset_x_h;
    uint8_t accel_offset_y_h;
    uint8_t accel_offset_z_h;

    struct {
        uint8_t s0_addr;
        uint8_t s0_reg;
        uint8_t s0_ctrl;
        uint8_t s1_addr;
        uint8_t s1_reg;
        uint8_t s1_ctrl;
        uint8_t s4_ctrl;
        uint8_t s0_do;
        uint8_t s1_do;
        uint8_t i2c_delay_ctrl;
        uint8_t raw_compass;
        /* The I2C_MST_VDDIO bit is in this register. */
        uint8_t yg_offs_tc;
    } ak89xx;
};

/* Information specific to a particular device. */
struct mpu_hw_s {
    uint8_t addr;
    uint16_t max_fifo;
    uint8_t num_reg;
    uint16_t temp_sens;
    int16_t temp_offset;
    uint16_t bank_size;

    struct {
        uint16_t compass_fsr;
    } ak89xx;
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct mpu_motion_int_cache_s {
    uint16_t gyro_fsr;
    uint8_t accel_fsr;
    uint16_t lpf;
    uint16_t sample_rate;
    uint8_t sensors_on;
    uint8_t fifo_sensors;
    uint8_t dmp_on;
};

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct mpu_chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    uint8_t gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    uint8_t accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    uint8_t sensors;
    /* Matches config register. */
    uint8_t lpf;
    uint8_t clk_src;
    /* Sample rate, NOT rate divider. */
    uint16_t sample_rate;
    /* Matches fifo_en register. */
    uint8_t fifo_enable;
    /* Matches int enable register. */
    uint8_t int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    uint8_t bypass_mode;
    /* 1 if device in low-power accel-only mode. */
    uint8_t lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    uint8_t int_motion_only;
    struct mpu_motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    uint8_t active_low_int;
    /* 1 for latched interrupts. */
    uint8_t latched_int;
    /* 1 if DMP is enabled. */
    uint8_t dmp_on;
    /* Ensures that DMP will only be loaded once. */
    uint8_t dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    uint16_t dmp_sample_rate;

    struct {
        /* Compass sample rate. */
        uint16_t compass_sample_rate;
        uint8_t compass_addr;
        int16_t mag_sens_adj[3];
    } ak89xx;
};

/* Information for self-test. */
struct mpu_test_s {
    uint32_t gyro_sens;
    uint32_t accel_sens;
    uint8_t reg_rate_div;
    uint8_t reg_lpf;
    uint8_t reg_gyro_fsr;
    uint8_t reg_accel_fsr;
    uint16_t wait_ms;
    uint8_t packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;

    struct {
        float max_g_offset;
        uint16_t sample_wait_ms;
    } mpu6500;
};

/* Gyro driver state variables. */
struct mpu_state_s {
    struct crossi2c_bus *i2cbus;

    enum mpu_type_e mputype;
    enum mag_type_e magtype;
    int mag_bypass;

    const struct mpu_gyro_reg_s *reg;
    const struct mpu_hw_s *hw;
    struct mpu_chip_cfg_s chip_cfg;
    const struct mpu_test_s *test;
};

struct mpu_cfg_dump {
    uint8_t mputype;
    uint8_t magtype;
    uint8_t gyro_fsr;
    uint8_t accel_fsr;
    int16_t mag_sens_adj[3];
} __attribute__((packed));

/* Set up APIs */
int mpu_create(struct mpu_state_s *st, enum mpu_type_e mputype, enum mag_type_e magtype, struct crossi2c_bus *i2cbus);
int mpu_create_nodev(struct mpu_state_s *st, struct mpu_cfg_dump *cfg);
int mpu_init(struct mpu_state_s *st);
int mpu_set_bypass(struct mpu_state_s *st, uint8_t bypass_on);

/* Configuration APIs */
int mpu_lp_accel_mode(struct mpu_state_s *st, uint16_t rate);
int mpu_lp_motion_interrupt(struct mpu_state_s *st, uint16_t thresh, uint8_t time,
    uint16_t lpa_freq);
int mpu_set_int_level(struct mpu_state_s *st, uint8_t active_low);
int mpu_set_int_latched(struct mpu_state_s *st, uint8_t enable);

int mpu_set_dmp_state(struct mpu_state_s *st, uint8_t enable);
int mpu_get_dmp_state(struct mpu_state_s *st, uint8_t *enabled);

int mpu_get_lpf(struct mpu_state_s *st, uint16_t *lpf);
int mpu_set_lpf(struct mpu_state_s *st, uint16_t lpf);

int mpu_get_gyro_fsr(struct mpu_state_s *st, uint16_t *fsr);
int mpu_set_gyro_fsr(struct mpu_state_s *st, uint16_t fsr);

int mpu_get_accel_fsr(struct mpu_state_s *st, uint8_t *fsr);
int mpu_set_accel_fsr(struct mpu_state_s *st, uint8_t fsr);

int mpu_get_compass_fsr(struct mpu_state_s *st, uint16_t *fsr);

int mpu_get_gyro_sens(struct mpu_state_s *st, float *sens);
int mpu_get_accel_sens(struct mpu_state_s *st, uint16_t *sens);

int mpu_get_sample_rate(struct mpu_state_s *st, uint16_t *rate);
int mpu_set_sample_rate(struct mpu_state_s *st, uint16_t rate);
int mpu_get_compass_sample_rate(struct mpu_state_s *st, uint16_t *rate);
int mpu_set_compass_sample_rate(struct mpu_state_s *st, uint16_t rate);

int mpu_get_fifo_config(struct mpu_state_s *st, uint8_t *sensors);
int mpu_configure_fifo(struct mpu_state_s *st, uint8_t sensors);

int mpu_get_power_state(struct mpu_state_s *st, uint8_t *power_on);
int mpu_set_sensors(struct mpu_state_s *st, uint8_t sensors);

int mpu_read_gyro_bias(struct mpu_state_s *st, int32_t *gyro_bias);
int mpu_set_gyro_bias_reg(struct mpu_state_s *st, const int32_t *gyro_bias);
int mpu_read_accel_bias(struct mpu_state_s *st, int32_t *accel_bias);
int mpu_set_accel_bias_reg(struct mpu_state_s *st, const int32_t *accel_bias);

/* Data getter/setter APIs */
int mpu_get_gyro_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp);
int mpu_get_accel_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp);
int mpu_get_compass_reg(struct mpu_state_s *st, int16_t *data, uint64_t *timestamp);
int mpu_get_temperature(struct mpu_state_s *st, int32_t *data, uint64_t *timestamp);

#define MPU_RAWSZ (3*2 + 1*2 + 3*2 + 4*2)
void mpu_get_cfg(struct mpu_state_s *st, struct mpu_cfg_dump *cfg);
int mpu_get_all_data(struct mpu_state_s *st, uint8_t data[MPU_RAWSZ], uint64_t *timestamp);
uint8_t mpu_parse_all_data(struct mpu_state_s *st, const uint8_t data[MPU_RAWSZ],
    int16_t *accel, int16_t *gyro, int16_t *compass);

int mpu_get_int_status(struct mpu_state_s *st, int16_t *status);
int mpu_read_fifo(struct mpu_state_s *st, int16_t *gyro, int16_t *accel, uint64_t *timestamp,
    uint8_t *sensors, uint8_t *more);
int mpu_read_fifo_stream(struct mpu_state_s *st, uint16_t length, uint8_t *data,
    uint8_t *more);
int mpu_reset_fifo(struct mpu_state_s *st);

int mpu_write_mem(struct mpu_state_s *st, uint16_t mem_addr, uint16_t length,
    uint8_t *data);
int mpu_read_mem(struct mpu_state_s *st, uint16_t mem_addr, uint16_t length,
    uint8_t *data);
int mpu_load_firmware(struct mpu_state_s *st, uint16_t length, const uint8_t *firmware,
    uint16_t start_addr, uint16_t sample_rate);

int mpu_reg_dump(struct mpu_state_s *st);
int mpu_read_reg(struct mpu_state_s *st, uint8_t reg, void *data);
int mpu_run_self_test(struct mpu_state_s *st, int32_t *gyro, int32_t *accel, int debug);
int mpu_register_tap_cb(struct mpu_state_s *st, void (*func)(uint8_t, uint8_t));

#endif  /* #ifndef _INV_MPU_H_ */

