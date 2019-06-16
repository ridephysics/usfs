#include <inv_mpu.h>
#include <inttypes.h>

#include "inv_mpu_p.h"

#define MPU6500_HWST_MAX_PACKET_LENGTH (512)

static int get_6050_accel_prod_shift(struct mpu_state_s *st, float *st_shift)
{
    uint8_t tmp[4], shift_code[3], ii;

    if (i2c_read(st, st->hw->addr, 0x0D, 4, tmp))
        return 0x07;

    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);
    for (ii = 0; ii < 3; ii++) {
        if (!shift_code[ii]) {
            st_shift[ii] = 0.f;
            continue;
        }
        /* Equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[ii] = 0.34f;
        while (--shift_code[ii])
            st_shift[ii] *= 1.034f;
    }
    return 0;
}

static int accel_6050_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    int jj, result = 0;
    float st_shift[3], st_shift_cust, st_shift_var;
    float accel_max_z_bias, accel_max_xy_bias;

    get_6050_accel_prod_shift(st, st_shift);
    for(jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (st_shift[jj]) {
            st_shift_var = st_shift_cust / st_shift[jj] - 1.f;
            if (fabs(st_shift_var) > st->test->max_accel_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < st->test->min_g) ||
            (st_shift_cust > st->test->max_g))
            result |= 1 << jj;
    }

    accel_max_z_bias = (.23f * 65535.f);
    accel_max_xy_bias = (.18f * 65535.f);

    if(debug) {
        log_i("ACCEL Bias Test:");
        log_i("Initial Biases:%"PRId32", %"PRId32", %"PRId32, bias_regular[0], bias_regular[1], bias_regular[2]);
    }

    if(result == 0) {
        if(bias_regular[0]>accel_max_xy_bias) {
            log_e("ACCEL FAIL X");
            result |= 1;
        }
        if(bias_regular[1]>accel_max_xy_bias) {
            log_e("ACCEL FAIL Y");
            result |= 2;
        }
        if(bias_regular[2]>accel_max_z_bias) {
            log_e("ACCEL FAIL Z");
            result |= 4;
        }
    }

    return result;
}

static int gyro_6050_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    int jj, result = 0;
    uint8_t tmp[3];
    float st_shift, st_shift_cust, st_shift_var;
    float gyro_max_bias;

    if (i2c_read(st, st->hw->addr, 0x0D, 3, tmp))
        return 0x07;

    tmp[0] &= 0x1F;
    tmp[1] &= 0x1F;
    tmp[2] &= 0x1F;

    for (jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f;
        if (tmp[jj]) {
            st_shift = 3275.f / st->test->gyro_sens;
            while (--tmp[jj])
                st_shift *= 1.046f;
            st_shift_var = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var) > st->test->max_gyro_var)
                result |= 1 << jj;
        } else if ((st_shift_cust < st->test->min_dps) ||
            (st_shift_cust > st->test->max_dps))
            result |= 1 << jj;
    }

    gyro_max_bias = (20.f * 65535.f);

    if(debug) {
        log_i("GYRO Bias Test:");
        log_i("Initial Biases:%"PRId32", %"PRId32", %"PRId32, bias_regular[0], bias_regular[1], bias_regular[2]);
    }

    if(result == 0) {
        if(bias_regular[0]>gyro_max_bias) {
            log_e("GYRO FAIL X");
            result |= 1;
        }
        if(bias_regular[1]>gyro_max_bias) {
            log_e("GYRO FAIL Y");
            result |= 2;
        }
        if(bias_regular[2]>gyro_max_bias) {
            log_e("GYRO FAIL Z");
            result |= 4;
        }
    }

    return result;
}

static int ak89xx_compass_self_test(struct mpu_state_s *st)
{
    uint8_t tmp[6];
    uint8_t tries = 10;
    int result = 0x07;
    int16_t data;

    mpu_set_bypass(st, 1);

    tmp[0] = AKM_POWER_DOWN;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, tmp))
        return 0x07;
    tmp[0] = AKM_BIT_SELF_TEST;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_ASTC, 1, tmp))
        goto AKM_restore;
    tmp[0] = AKM_MODE_SELF_TEST;
    if (i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, tmp))
        goto AKM_restore;

    do {
        delay_ms(10);
        if (i2c_read(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_ST1, 1, tmp))
            goto AKM_restore;
        if (tmp[0] & AKM_DATA_READY)
            break;
    } while (tries--);
    if (!(tmp[0] & AKM_DATA_READY))
        goto AKM_restore;

    if (i2c_read(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_HXL, 6, tmp))
        goto AKM_restore;

    result = 0;
    if (st->mputype == MPU_TYPE_MPU6050) {
        data = (int16_t)(tmp[1] << 8) | tmp[0];
        if ((data > 100) || (data < -100))
            result |= 0x01;
        data = (int16_t)(tmp[3] << 8) | tmp[2];
        if ((data > 100) || (data < -100))
            result |= 0x02;
        data = (int16_t)(tmp[5] << 8) | tmp[4];
        if ((data > -300) || (data < -1000))
            result |= 0x04;
    }
    else if (st->mputype == MPU_TYPE_MPU6500) {
        data = (int16_t)(tmp[1] << 8) | tmp[0];
        if ((data > 200) || (data < -200))  
            result |= 0x01;
        data = (int16_t)(tmp[3] << 8) | tmp[2];
        if ((data > 200) || (data < -200))  
            result |= 0x02;
        data = (int16_t)(tmp[5] << 8) | tmp[4];
        if ((data > -800) || (data < -3200))  
            result |= 0x04;
    }

AKM_restore:
    tmp[0] = 0 | SUPPORTS_AK89xx_HIGH_SENS;
    i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_ASTC, 1, tmp);
    tmp[0] = SUPPORTS_AK89xx_HIGH_SENS;
    i2c_write(st, st->chip_cfg.ak89xx.compass_addr, AKM_REG_CNTL, 1, tmp);
    mpu_set_bypass(st, 0);
    return result;
}

static int get_st_6050_biases(struct mpu_state_s *st, int32_t *gyro, int32_t *accel, uint8_t hw_test, int debug)
{
    uint8_t data[MAX_PACKET_LENGTH];
    uint8_t packet_count, ii;
    uint16_t fifo_count;

    data[0] = 0x01;
    data[1] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
    data[0] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->i2c_mst, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;
    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    data[0] = st->test->reg_lpf;
    if (i2c_write(st, st->hw->addr, st->reg->lpf, 1, data))
        return -1;
    data[0] = st->test->reg_rate_div;
    if (i2c_write(st, st->hw->addr, st->reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        data[0] = st->test->reg_gyro_fsr | 0xE0;
    else
        data[0] = st->test->reg_gyro_fsr;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_cfg, 1, data))
        return -1;

    if (hw_test)
        data[0] = st->test->reg_accel_fsr | 0xE0;
    else
        data[0] = st->test->reg_accel_fsr;
    if (i2c_write(st, st->hw->addr, st->reg->accel_cfg, 1, data))
        return -1;
    if (hw_test)
        delay_ms(200);

    /* Fill FIFO for test->wait_ms milliseconds. */
    data[0] = BIT_FIFO_EN;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;

    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;
    delay_ms(st->test->wait_ms);
    data[0] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;

    if (i2c_read(st, st->hw->addr, st->reg->fifo_count_h, 2, data))
        return -1;

    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MAX_PACKET_LENGTH;
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    if(debug)
        log_i("Starting Bias Loop Reads");

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_cur[3], gyro_cur[3];
        if (i2c_read(st, st->hw->addr, st->reg->fifo_r_w, MAX_PACKET_LENGTH, data))
            return -1;
        accel_cur[0] = ((int16_t)data[0] << 8) | data[1];
        accel_cur[1] = ((int16_t)data[2] << 8) | data[3];
        accel_cur[2] = ((int16_t)data[4] << 8) | data[5];
        accel[0] += (int32_t)accel_cur[0];
        accel[1] += (int32_t)accel_cur[1];
        accel[2] += (int32_t)accel_cur[2];
        gyro_cur[0] = (((int16_t)data[6] << 8) | data[7]);
        gyro_cur[1] = (((int16_t)data[8] << 8) | data[9]);
        gyro_cur[2] = (((int16_t)data[10] << 8) | data[11]);
        gyro[0] += (int32_t)gyro_cur[0];
        gyro[1] += (int32_t)gyro_cur[1];
        gyro[2] += (int32_t)gyro_cur[2];
    }

    if(debug)
        log_i("Samples: %u", ii);

#ifdef EMPL_NO_64BIT
    gyro[0] = (int32_t)(((float)gyro[0]*65536.f) / st->test->gyro_sens / packet_count);
    gyro[1] = (int32_t)(((float)gyro[1]*65536.f) / st->test->gyro_sens / packet_count);
    gyro[2] = (int32_t)(((float)gyro[2]*65536.f) / st->test->gyro_sens / packet_count);
    if (has_accel) {
        accel[0] = (int32_t)(((float)accel[0]*65536.f) / st->test->accel_sens /
            packet_count);
        accel[1] = (int32_t)(((float)accel[1]*65536.f) / st->test->accel_sens /
            packet_count);
        accel[2] = (int32_t)(((float)accel[2]*65536.f) / st->test->accel_sens /
            packet_count);
        /* Don't remove gravity! */
        accel[2] -= 65536L;
    }
#else
    gyro[0] = (int32_t)(((int64_t)gyro[0]<<16) / st->test->gyro_sens / packet_count);
    gyro[1] = (int32_t)(((int64_t)gyro[1]<<16) / st->test->gyro_sens / packet_count);
    gyro[2] = (int32_t)(((int64_t)gyro[2]<<16) / st->test->gyro_sens / packet_count);
    accel[0] = (int32_t)(((int64_t)accel[0]<<16) / st->test->accel_sens /
        packet_count);
    accel[1] = (int32_t)(((int64_t)accel[1]<<16) / st->test->accel_sens /
        packet_count);
    accel[2] = (int32_t)(((int64_t)accel[2]<<16) / st->test->accel_sens /
        packet_count);
    /* Don't remove gravity! */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;
#endif

    if(debug) {
        log_i("Accel offset data HWST bit=%d: %7.4f %7.4f %7.4f", hw_test, accel[0]/65536.f, accel[1]/65536.f, accel[2]/65536.f);
        log_i("Gyro offset data HWST bit=%d: %7.4f %7.4f %7.4f", hw_test, gyro[0]/65536.f, gyro[1]/65536.f, gyro[2]/65536.f);
    }

    return 0;
}

#define REG_6500_XG_ST_DATA     0x0
#define REG_6500_XA_ST_DATA     0xD
static const uint16_t mpu_6500_st_tb[256] = {
    2620,2646,2672,2699,2726,2753,2781,2808, //7
    2837,2865,2894,2923,2952,2981,3011,3041, //15
    3072,3102,3133,3165,3196,3228,3261,3293, //23
    3326,3359,3393,3427,3461,3496,3531,3566, //31
    3602,3638,3674,3711,3748,3786,3823,3862, //39
    3900,3939,3979,4019,4059,4099,4140,4182, //47
    4224,4266,4308,4352,4395,4439,4483,4528, //55
    4574,4619,4665,4712,4759,4807,4855,4903, //63
    4953,5002,5052,5103,5154,5205,5257,5310, //71
    5363,5417,5471,5525,5581,5636,5693,5750, //79
    5807,5865,5924,5983,6043,6104,6165,6226, //87
    6289,6351,6415,6479,6544,6609,6675,6742, //95
    6810,6878,6946,7016,7086,7157,7229,7301, //103
    7374,7448,7522,7597,7673,7750,7828,7906, //111
    7985,8065,8145,8227,8309,8392,8476,8561, //119
    8647,8733,8820,8909,8998,9088,9178,9270,
    9363,9457,9551,9647,9743,9841,9939,10038,
    10139,10240,10343,10446,10550,10656,10763,10870,
    10979,11089,11200,11312,11425,11539,11654,11771,
    11889,12008,12128,12249,12371,12495,12620,12746,
    12874,13002,13132,13264,13396,13530,13666,13802,
    13940,14080,14221,14363,14506,14652,14798,14946,
    15096,15247,15399,15553,15709,15866,16024,16184,
    16346,16510,16675,16842,17010,17180,17352,17526,
    17701,17878,18057,18237,18420,18604,18790,18978,
    19167,19359,19553,19748,19946,20145,20347,20550,
    20756,20963,21173,21385,21598,21814,22033,22253,
    22475,22700,22927,23156,23388,23622,23858,24097,
    24338,24581,24827,25075,25326,25579,25835,26093,
    26354,26618,26884,27153,27424,27699,27976,28255,
    28538,28823,29112,29403,29697,29994,30294,30597,
    30903,31212,31524,31839,32157,32479,32804,33132
};
static int accel_6500_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    int i, result = 0, otp_value_zero = 0;
    float accel_st_al_min, accel_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
    uint8_t regs[3];
    if (i2c_read(st, st->hw->addr, REG_6500_XA_ST_DATA, 3, regs)) {
        if(debug)
            log_i("Reading OTP Register Error.");
        return 0x07;
    }
    if(debug)
        log_i("Accel OTP:%d, %d, %d", regs[0], regs[1], regs[2]);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
            ct_shift_prod[i] *= 65536.f;
            ct_shift_prod[i] /= st->test->accel_sens;
        }
        else {
            ct_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }
    if(otp_value_zero == 0) {
        if(debug)
            log_i("ACCEL:CRITERIA A");
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];
            if(debug) {
                log_i("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f",
                        st_shift_cust[i]/1.f, bias_regular[i]/1.f,
                        bias_st[i]/1.f);
                log_i("OTP value: %7.4f", ct_shift_prod[i]/1.f);
            }

            st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i] - 1.f;

            if(debug)
                log_i("ratio=%7.4f, threshold=%7.4f", st_shift_ratio[i]/1.f,
                            st->test->max_accel_var/1.f);

            if (fabs(st_shift_ratio[i]) > st->test->max_accel_var) {
                if(debug)
                    log_i("ACCEL Fail Axis = %d", i);
                result |= 1 << i;    //Error condition
            }
        }
    }
    else {
        /* Self Test Pass/Fail Criteria B */
        accel_st_al_min = st->test->min_g * 65536.f;
        accel_st_al_max = st->test->max_g * 65536.f;

        if(debug) {
            log_i("ACCEL:CRITERIA B");
            log_i("Min MG: %7.4f", accel_st_al_min/1.f);
            log_i("Max MG: %7.4f", accel_st_al_max/1.f);
        }

        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];

            if(debug)
                log_i("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
            if(st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
                if(debug)
                    log_i("Accel FAIL axis:%d <= 225mg or >= 675mg", i);
                result |= 1 << i;    //Error condition
            }
        }
    }

    if(result == 0) {
    /* Self Test Pass/Fail Criteria C */
        accel_offset_max = st->test->mpu6500.max_g_offset * 65536.f;
        if(debug)
            log_i("Accel:CRITERIA C: bias less than %7.4f", accel_offset_max/1.f);
        for (i = 0; i < 3; i++) {
            if(fabs(bias_regular[i]) > accel_offset_max) {
                if(debug)
                    log_i("FAILED: Accel axis:%d = %"PRId32" > 500mg", i, bias_regular[i]);
                result |= 1 << i;    //Error condition
            }
        }
    }

    return result;
}

static int gyro_6500_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    int i, result = 0, otp_value_zero = 0;
    float gyro_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
    uint8_t regs[3];

    if (i2c_read(st, st->hw->addr, REG_6500_XG_ST_DATA, 3, regs)) {
        if(debug)
            log_i("Reading OTP Register Error.");
        return 0x07;
    }

    if(debug)
        log_i("Gyro OTP:%d, %d, %d", regs[0], regs[1], regs[2]);

    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
            ct_shift_prod[i] *= 65536.f;
            ct_shift_prod[i] /= st->test->gyro_sens;
        }
        else {
            ct_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    if(otp_value_zero == 0) {
        if(debug)
            log_i("GYRO:CRITERIA A");
        /* Self Test Pass/Fail Criteria A */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];

            if(debug) {
                log_i("Bias_Shift=%7.4f, Bias_Reg=%7.4f, Bias_HWST=%7.4f",
                        st_shift_cust[i]/1.f, bias_regular[i]/1.f,
                        bias_st[i]/1.f);
                log_i("OTP value: %7.4f", ct_shift_prod[i]/1.f);
            }

            st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];

            if(debug)
                log_i("ratio=%7.4f, threshold=%7.4f", st_shift_ratio[i]/1.f,
                            st->test->max_gyro_var/1.f);

            if (fabs(st_shift_ratio[i]) < st->test->max_gyro_var) {
                if(debug)
                    log_i("Gyro Fail Axis = %d", i);
                result |= 1 << i;    //Error condition
            }
        }
    }
    else {
        /* Self Test Pass/Fail Criteria B */
        gyro_st_al_max = st->test->max_dps * 65536.f;

        if(debug) {
            log_i("GYRO:CRITERIA B");
            log_i("Max DPS: %7.4f", gyro_st_al_max/1.f);
        }

        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = bias_st[i] - bias_regular[i];

            if(debug)
                log_i("Bias_shift=%7.4f, st=%7.4f, reg=%7.4f", st_shift_cust[i]/1.f, bias_st[i]/1.f, bias_regular[i]/1.f);
            if(st_shift_cust[i] < gyro_st_al_max) {
                if(debug)
                    log_i("GYRO FAIL axis:%d greater than 60dps", i);
                result |= 1 << i;    //Error condition
            }
        }
    }

    if(result == 0) {
    /* Self Test Pass/Fail Criteria C */
        gyro_offset_max = st->test->min_dps * 65536.f;
        if(debug)
            log_i("Gyro:CRITERIA C: bias less than %7.4f", gyro_offset_max/1.f);
        for (i = 0; i < 3; i++) {
            if(fabs(bias_regular[i]) > gyro_offset_max) {
                if(debug)
                    log_i("FAILED: Gyro axis:%d = %"PRId32" > 20dps", i, bias_regular[i]);
                result |= 1 << i;    //Error condition
            }
        }
    }
    return result;
}

static int get_st_6500_biases(struct mpu_state_s *st, int32_t *gyro, int32_t *accel, uint8_t hw_test, int debug)
{
    uint8_t data[MPU6500_HWST_MAX_PACKET_LENGTH];
    uint8_t packet_count, ii;
    uint16_t fifo_count;
    int s = 0, read_size = 0, ind;

    data[0] = 0x01;
    data[1] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
    data[0] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->int_enable, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->pwr_mgmt_1, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->i2c_mst, 1, data))
        return -1;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;
    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    data[0] = st->test->reg_lpf;
    if (i2c_write(st, st->hw->addr, st->reg->lpf, 1, data))
        return -1;
    data[0] = st->test->reg_rate_div;
    if (i2c_write(st, st->hw->addr, st->reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        data[0] = st->test->reg_gyro_fsr | 0xE0;
    else
        data[0] = st->test->reg_gyro_fsr;
    if (i2c_write(st, st->hw->addr, st->reg->gyro_cfg, 1, data))
        return -1;

    if (hw_test)
        data[0] = st->test->reg_accel_fsr | 0xE0;
    else
        data[0] = st->test->reg_accel_fsr;
    if (i2c_write(st, st->hw->addr, st->reg->accel_cfg, 1, data))
        return -1;

    delay_ms(st->test->wait_ms);  //wait 200ms for sensors to stabilize

    /* Enable FIFO */
    data[0] = BIT_FIFO_EN;
    if (i2c_write(st, st->hw->addr, st->reg->user_ctrl, 1, data))
        return -1;
    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;

    //initialize the bias return values
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    if(debug)
        log_i("Starting Bias Loop Reads");

    //start reading samples
    while (s < st->test->packet_thresh) {
        delay_ms(st->test->mpu6500.sample_wait_ms); //wait 10ms to fill FIFO
        if (i2c_read(st, st->hw->addr, st->reg->fifo_count_h, 2, data))
            return -1;
        fifo_count = (data[0] << 8) | data[1];
        packet_count = fifo_count / MAX_PACKET_LENGTH;
        if ((st->test->packet_thresh - s) < packet_count)
            packet_count = st->test->packet_thresh - s;
        read_size = packet_count * MAX_PACKET_LENGTH;

        if (read_size > 64)
            read_size = 64;

        //burst read from FIFO
        if (i2c_read(st, st->hw->addr, st->reg->fifo_r_w, read_size, data))
            return -1;
        ind = 0;
        for (ii = 0; ii < packet_count; ii++) {
            int16_t accel_cur[3], gyro_cur[3];
            accel_cur[0] = ((int16_t)data[ind + 0] << 8) | data[ind + 1];
            accel_cur[1] = ((int16_t)data[ind + 2] << 8) | data[ind + 3];
            accel_cur[2] = ((int16_t)data[ind + 4] << 8) | data[ind + 5];
            accel[0] += (int32_t)accel_cur[0];
            accel[1] += (int32_t)accel_cur[1];
            accel[2] += (int32_t)accel_cur[2];
            gyro_cur[0] = (((int16_t)data[ind + 6] << 8) | data[ind + 7]);
            gyro_cur[1] = (((int16_t)data[ind + 8] << 8) | data[ind + 9]);
            gyro_cur[2] = (((int16_t)data[ind + 10] << 8) | data[ind + 11]);
            gyro[0] += (int32_t)gyro_cur[0];
            gyro[1] += (int32_t)gyro_cur[1];
            gyro[2] += (int32_t)gyro_cur[2];
            ind += MAX_PACKET_LENGTH;
        }
        s += packet_count;
    }

    if(debug)
        log_i("Samples: %d", s);

    //stop FIFO
    data[0] = 0;
    if (i2c_write(st, st->hw->addr, st->reg->fifo_en, 1, data))
        return -1;

    gyro[0] = (int32_t)(((int64_t)gyro[0]<<16) / st->test->gyro_sens / s);
    gyro[1] = (int32_t)(((int64_t)gyro[1]<<16) / st->test->gyro_sens / s);
    gyro[2] = (int32_t)(((int64_t)gyro[2]<<16) / st->test->gyro_sens / s);
    accel[0] = (int32_t)(((int64_t)accel[0]<<16) / st->test->accel_sens / s);
    accel[1] = (int32_t)(((int64_t)accel[1]<<16) / st->test->accel_sens / s);
    accel[2] = (int32_t)(((int64_t)accel[2]<<16) / st->test->accel_sens / s);
    /* remove gravity from bias calculation */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;


    if(debug) {
        log_i("Accel offset data HWST bit=%d: %7.4f %7.4f %7.4f", hw_test, accel[0]/65536.f, accel[1]/65536.f, accel[2]/65536.f);
        log_i("Gyro offset data HWST bit=%d: %7.4f %7.4f %7.4f", hw_test, gyro[0]/65536.f, gyro[1]/65536.f, gyro[2]/65536.f);
    }

    return 0;
}

static int get_st_biases(struct mpu_state_s *st, int32_t *gyro, int32_t *accel, uint8_t hw_test, int debug)
{
    if (st->mputype == MPU_TYPE_MPU6050)
        return get_st_6050_biases(st, gyro, accel, hw_test, debug);

    if (st->mputype == MPU_TYPE_MPU6500)
        return get_st_6500_biases(st, gyro, accel, hw_test, debug);

    return -1;
}

static int accel_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    if (st->mputype == MPU_TYPE_MPU6050)
        return accel_6050_self_test(st, bias_regular, bias_st, debug);

    if (st->mputype == MPU_TYPE_MPU6500)
        return accel_6500_self_test(st, bias_regular, bias_st, debug);

    return -1;
}

static int gyro_self_test(struct mpu_state_s *st, int32_t *bias_regular, int32_t *bias_st, int debug)
{
    if (st->mputype == MPU_TYPE_MPU6050)
        return gyro_6050_self_test(st, bias_regular, bias_st, debug);

    if (st->mputype == MPU_TYPE_MPU6500)
        return gyro_6500_self_test(st, bias_regular, bias_st, debug);

    return -1;
}

/**
 *  @brief      Trigger gyro/accel/compass self-test for MPU6500/MPU9250
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @param[in]  debug       Debug flag used to print out more detailed logs. Must first set up logging in Motion Driver.
 *  @return     Result mask (see above).
 */
int mpu_run_self_test(struct mpu_state_s *st, int32_t *gyro, int32_t *accel, int debug)
{
    const uint8_t tries = 2;
    int32_t gyro_st[3], accel_st[3];
    uint8_t accel_result, gyro_result;
    uint8_t compass_result;
    int ii;
    int result;
    uint8_t accel_fsr, fifo_sensors, sensors_on;
    uint16_t gyro_fsr, sample_rate, lpf;
    uint8_t dmp_was_on;

    if (debug)
        log_i("Starting HWST!");

    if (st->chip_cfg.dmp_on) {
        mpu_set_dmp_state(st, 0);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* Get initial settings. */
    mpu_get_gyro_fsr(st, &gyro_fsr);
    mpu_get_accel_fsr(st, &accel_fsr);
    mpu_get_lpf(st, &lpf);
    mpu_get_sample_rate(st, &sample_rate);
    sensors_on = st->chip_cfg.sensors;
    mpu_get_fifo_config(st, &fifo_sensors);

    if(debug)
        log_i("Retrieving Biases");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(st, gyro, accel, 0, debug))
            break;
    if (ii == tries) {
        /* If we reach this point, we most likely encountered an I2C error.
         * We'll just report an error for all three sensors.
         */

        if(debug)
            log_i("Retrieving Biases Error - possible I2C error");

        result = 0;
        goto restore;
    }

    if(debug)
        log_i("Retrieving ST Biases");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(st, gyro_st, accel_st, 1, debug))
            break;
    if (ii == tries) {
        if(debug)
            log_i("Retrieving ST Biases Error - possible I2C error");

        /* Again, probably an I2C error. */
        result = 0;
        goto restore;
    }

    accel_result = accel_self_test(st, accel, accel_st, debug);
    if(debug)
        log_i("Accel Self Test Results: %d", accel_result);

    gyro_result = gyro_self_test(st, gyro, gyro_st, debug);
    if(debug)
        log_i("Gyro Self Test Results: %d", gyro_result);

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

    if (st->magtype == MAG_TYPE_NONE) {
        result |= 0x04;
    }
    else {
        compass_result = ak89xx_compass_self_test(st);
        if(debug)
            log_i("Compass Self Test Results: %d", compass_result);
        if (!compass_result)
            result |= 0x04;
    }

restore:
    if(debug)
        log_i("Exiting HWST");
    /* Set to invalid values to ensure no I2C writes are skipped. */
    st->chip_cfg.gyro_fsr = 0xFF;
    st->chip_cfg.accel_fsr = 0xFF;
    st->chip_cfg.lpf = 0xFF;
    st->chip_cfg.sample_rate = 0xFFFF;
    st->chip_cfg.sensors = 0xFF;
    st->chip_cfg.fifo_enable = 0xFF;
    st->chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_gyro_fsr(st, gyro_fsr);
    mpu_set_accel_fsr(st, accel_fsr);
    mpu_set_lpf(st, lpf);
    mpu_set_sample_rate(st, sample_rate);
    mpu_set_sensors(st, sensors_on);
    mpu_configure_fifo(st, fifo_sensors);

    if (dmp_was_on)
        mpu_set_dmp_state(st, 1);

    return result;
}
