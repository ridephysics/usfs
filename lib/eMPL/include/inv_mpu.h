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
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;

    struct {
        unsigned char s0_addr;
        unsigned char s0_reg;
        unsigned char s0_ctrl;
        unsigned char s1_addr;
        unsigned char s1_reg;
        unsigned char s1_ctrl;
        unsigned char s4_ctrl;
        unsigned char s0_do;
        unsigned char s1_do;
        unsigned char i2c_delay_ctrl;
        unsigned char raw_compass;
        /* The I2C_MST_VDDIO bit is in this register. */
        unsigned char yg_offs_tc;
    } ak89xx;
};

/* Information specific to a particular device. */
struct mpu_hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;

    struct {
        unsigned short compass_fsr;
    } ak89xx;
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct mpu_motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};

/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct mpu_chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    unsigned char sensors;
    /* Matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* Sample rate, NOT rate divider. */
    unsigned short sample_rate;
    /* Matches fifo_en register. */
    unsigned char fifo_enable;
    /* Matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct mpu_motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if DMP is enabled. */
    unsigned char dmp_on;
    /* Ensures that DMP will only be loaded once. */
    unsigned char dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    unsigned short dmp_sample_rate;

    struct {
        /* Compass sample rate. */
        unsigned short compass_sample_rate;
        unsigned char compass_addr;
        short mag_sens_adj[3];
    } ak89xx;
};

/* Information for self-test. */
struct mpu_test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;

    struct {
        float max_g_offset;
        unsigned short sample_wait_ms;
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

/* Set up APIs */
int mpu_init(struct mpu_state_s *st, enum mpu_type_e mputype, enum mag_type_e magtype, struct crossi2c_bus *i2cbus);
int mpu_set_bypass(struct mpu_state_s *st, unsigned char bypass_on);

/* Configuration APIs */
int mpu_lp_accel_mode(struct mpu_state_s *st, unsigned short rate);
int mpu_lp_motion_interrupt(struct mpu_state_s *st, unsigned short thresh, unsigned char time,
    unsigned short lpa_freq);
int mpu_set_int_level(struct mpu_state_s *st, unsigned char active_low);
int mpu_set_int_latched(struct mpu_state_s *st, unsigned char enable);

int mpu_set_dmp_state(struct mpu_state_s *st, unsigned char enable);
int mpu_get_dmp_state(struct mpu_state_s *st, unsigned char *enabled);

int mpu_get_lpf(struct mpu_state_s *st, unsigned short *lpf);
int mpu_set_lpf(struct mpu_state_s *st, unsigned short lpf);

int mpu_get_gyro_fsr(struct mpu_state_s *st, unsigned short *fsr);
int mpu_set_gyro_fsr(struct mpu_state_s *st, unsigned short fsr);

int mpu_get_accel_fsr(struct mpu_state_s *st, unsigned char *fsr);
int mpu_set_accel_fsr(struct mpu_state_s *st, unsigned char fsr);

int mpu_get_compass_fsr(struct mpu_state_s *st, unsigned short *fsr);

int mpu_get_gyro_sens(struct mpu_state_s *st, float *sens);
int mpu_get_accel_sens(struct mpu_state_s *st, unsigned short *sens);

int mpu_get_sample_rate(struct mpu_state_s *st, unsigned short *rate);
int mpu_set_sample_rate(struct mpu_state_s *st, unsigned short rate);
int mpu_get_compass_sample_rate(struct mpu_state_s *st, unsigned short *rate);
int mpu_set_compass_sample_rate(struct mpu_state_s *st, unsigned short rate);

int mpu_get_fifo_config(struct mpu_state_s *st, unsigned char *sensors);
int mpu_configure_fifo(struct mpu_state_s *st, unsigned char sensors);

int mpu_get_power_state(struct mpu_state_s *st, unsigned char *power_on);
int mpu_set_sensors(struct mpu_state_s *st, unsigned char sensors);

int mpu_read_6500_accel_bias(struct mpu_state_s *st, long *accel_bias);
int mpu_read_6500_gyro_bias(struct mpu_state_s *st, long *gyro_bias);
int mpu_set_gyro_bias_reg(struct mpu_state_s *st, long * gyro_bias);
int mpu_set_accel_bias_6500_reg(struct mpu_state_s *st, const long *accel_bias);
int mpu_read_6050_accel_bias(struct mpu_state_s *st, long *accel_bias);
int mpu_set_accel_bias_6050_reg(struct mpu_state_s *st, const long *accel_bias);

/* Data getter/setter APIs */
int mpu_get_gyro_reg(struct mpu_state_s *st, short *data, unsigned long *timestamp);
int mpu_get_accel_reg(struct mpu_state_s *st, short *data, unsigned long *timestamp);
int mpu_get_compass_reg(struct mpu_state_s *st, short *data, unsigned long *timestamp);
int mpu_get_temperature(struct mpu_state_s *st, long *data, unsigned long *timestamp);

int mpu_get_int_status(struct mpu_state_s *st, short *status);
int mpu_read_fifo(struct mpu_state_s *st, short *gyro, short *accel, unsigned long *timestamp,
    unsigned char *sensors, unsigned char *more);
int mpu_read_fifo_stream(struct mpu_state_s *st, unsigned short length, unsigned char *data,
    unsigned char *more);
int mpu_reset_fifo(struct mpu_state_s *st);

int mpu_write_mem(struct mpu_state_s *st, unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_read_mem(struct mpu_state_s *st, unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_load_firmware(struct mpu_state_s *st, unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

int mpu_reg_dump(struct mpu_state_s *st);
int mpu_read_reg(struct mpu_state_s *st, unsigned char reg, unsigned char *data);
int mpu_run_self_test(struct mpu_state_s *st, long *gyro, long *accel, int debug);
int mpu_register_tap_cb(struct mpu_state_s *st, void (*func)(unsigned char, unsigned char));

#endif  /* #ifndef _INV_MPU_H_ */

