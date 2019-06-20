#ifndef INV_MPU_P_H
#define INV_MPU_P_H

#include <usfs.h>
#include <math.h>

#define CROSSLOG_TAG "eMPL"
#include <crosslog.h>

static inline int i2c_write(struct mpu_state_s *st, uint8_t addr, uint8_t reg, size_t len, const void *buf) {
    return crossi2c_burst_write(st->i2cbus, addr, reg, buf, len);
}

static inline int i2c_read(struct mpu_state_s *st, uint8_t addr, uint8_t reg, size_t len, void *buf) {
    return crossi2c_burst_read(st->i2cbus, addr, reg, buf, len);
}

static inline void delay_ms(uint64_t ms) {
    usfs_usleep(ms * 1000);
}

#define log_i(fmt, ...)     CROSSLOGI(fmt, ##__VA_ARGS__)
#define log_e(fmt, ...)     CROSSLOGE(fmt, ##__VA_ARGS__)

/* labs is already defined by TI's toolchain. */
/* fabs is for doubles. fabsf is for floats. */
#define fabs        fabsf
#define min(a,b) ((a<b)?a:b)

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_MPU6050_1_25HZ = 0,
    INV_LPA_MPU6050_5HZ,
    INV_LPA_MPU6050_20HZ,
    INV_LPA_MPU6050_40HZ,

    INV_LPA_MPU6500_0_3125HZ = 0,
    INV_LPA_MPU6500_0_625HZ,
    INV_LPA_MPU6500_1_25HZ,
    INV_LPA_MPU6500_2_5HZ,
    INV_LPA_MPU6500_5HZ,
    INV_LPA_MPU6500_10HZ,
    INV_LPA_MPU6500_20HZ,
    INV_LPA_MPU6500_40HZ,
    INV_LPA_MPU6500_80HZ,
    INV_LPA_MPU6500_160HZ,
    INV_LPA_MPU6500_320HZ,
    INV_LPA_MPU6500_640HZ
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

#define BIT_MPU6500_ACCL_FC_B        (0x08)

#define AK8975_SUPPORTS_HIGH_SENS   (0x00)
#define AK8975_FSR                  (9830)
#define AK8963_SUPPORTS_HIGH_SENS   (0x10)
#define AK8963_FSR                  (4915)

#define SUPPORTS_AK89xx_HIGH_SENS ((st->magtype == MAG_TYPE_AK8975) ? AK8975_SUPPORTS_HIGH_SENS : AK8963_SUPPORTS_HIGH_SENS)

#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define AK89XX_MAX_COMPASS_SAMPLE_RATE (100)

#define MAX_PACKET_LENGTH (12)

int _mpu_set_int_enable(struct mpu_state_s *st, uint8_t enable);
int _ak89xx_setup_compass(struct mpu_state_s *st);

#endif /* INV_MPU_P_H */
