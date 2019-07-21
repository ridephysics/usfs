#include <getopt.h>
#include <crossi2c/linux.h>
#include <em7180.h>
#include <usfs/bmp280.h>
#include <inv_mpu.h>
#include <usfs.h>
#include <fcntl.h>
#include <signal.h>
#include <math.h>

#define CROSSLOG_TAG "reader"
#include <crosslog.h>

enum informat {
    INFORMAT_INVALID = -1,
    INFORMAT_SENTRAL,
    INFORMAT_SENTRAL_PT,
};

enum outformat {
    OUTFORMAT_INVALID = -1,
    OUTFORMAT_RAW,
    OUTFORMAT_PROCESSED,
};

static enum informat arg_infmt = INFORMAT_INVALID;
static enum outformat arg_outfmt = OUTFORMAT_INVALID;
static const char *arg_src = NULL;
static bool arg_src_isi2c = false;
static double cal_mag_center[3] = {0, 0, 0};
static double cal_mag_tr[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};
static double bias_accel[3] = {0,0,0};
static double bias_gyro[3] = {0,0,0};

static volatile bool keep_running = true;
static struct crossi2c_bus i2cbus;
static struct em7180 em7180;
static struct bmp280_dev bmp280;
static struct mpu_state_s mpu9250;

static enum informat str2informat(const char *s) {
    if (!strcmp(s, "sentral"))
        return INFORMAT_SENTRAL;
    if (!strcmp(s, "sentral-pt"))
        return INFORMAT_SENTRAL_PT;

    return INFORMAT_INVALID;
}

static enum informat str2outformat(const char *s) {
    if (!strcmp(s, "raw"))
        return OUTFORMAT_RAW;
    if (!strcmp(s, "processed"))
        return OUTFORMAT_PROCESSED;

    return OUTFORMAT_INVALID;
}

static bool str_startswith(const char *pre, const char *str)
{
    size_t lenpre = strlen(pre),
           lenstr = strlen(str);
    return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

static ssize_t read_all(int fd, void *buf, size_t count) {
    ssize_t ret = 0;
    ssize_t nbytes;

    while (count) {
        nbytes = read(fd, buf, count);
        if (nbytes < 0) {
            if (errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN)
                continue;

            return -1;
        }

        if (nbytes == 0) {
            return ret;
        }

        buf += nbytes;
        count -= nbytes;
        ret += nbytes;
    }

    return ret;
}

static int load_cal_mag(const char *path) {
    int ret = -1;
    int fd;
    ssize_t nbytes;
    double tmp_cal_mag_center[3];
    double tmp_cal_mag_tr[3][3];

    fd = open(path, O_RDONLY, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("open");
        return -1;
    }

    nbytes = read_all(fd, tmp_cal_mag_center, sizeof(tmp_cal_mag_center));
    if (nbytes != sizeof(tmp_cal_mag_center)) {
        CROSSLOG_ERRNO("read_all");
        goto out_close;
    }

    nbytes = read_all(fd, tmp_cal_mag_tr, sizeof(tmp_cal_mag_tr));
    if (nbytes != sizeof(tmp_cal_mag_tr)) {
        CROSSLOG_ERRNO("read_all");
        goto out_close;
    }

    memcpy(cal_mag_center, tmp_cal_mag_center, sizeof(cal_mag_center));
    memcpy(cal_mag_tr, tmp_cal_mag_tr, sizeof(cal_mag_tr));

    CROSSLOGI("center = [%f, %f, %f]", cal_mag_center[0], cal_mag_center[1], cal_mag_center[2]);
    CROSSLOGI("TR = [\n\t[%f, %f, %f],\n\t[%f, %f, %f],\n\t[%f, %f, %f]\n]",
        cal_mag_tr[0][0], cal_mag_tr[0][1], cal_mag_tr[0][2],
        cal_mag_tr[1][0], cal_mag_tr[1][1], cal_mag_tr[1][2],
        cal_mag_tr[2][0], cal_mag_tr[2][1], cal_mag_tr[2][2]
    );

    ret = 0;

out_close:
    close(fd);
    return ret;
}

static int load_bias_ag(const char *path) {
    int ret = -1;
    int fd;
    ssize_t nbytes;
    int32_t tmp_accel[3];
    int32_t tmp_gyro[3];
    size_t i;

    fd = open(path, O_RDONLY, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("open");
        return -1;
    }

    nbytes = read_all(fd, tmp_accel, sizeof(tmp_accel));
    if (nbytes != sizeof(tmp_accel)) {
        CROSSLOG_ERRNO("read_all");
        goto out_close;
    }

    nbytes = read_all(fd, tmp_gyro, sizeof(tmp_gyro));
    if (nbytes != sizeof(tmp_gyro)) {
        CROSSLOG_ERRNO("read_all");
        goto out_close;
    }

    for (i=0; i<3; i++) {
        bias_accel[i] = ((double)tmp_accel[i]) / 65536.0f;
        bias_gyro[i] = ((double)tmp_gyro[i]) / 65536.0f;
    }

    CROSSLOGI("bias_accel = [%f, %f, %f]", bias_accel[0], bias_accel[1], bias_accel[2]);
    CROSSLOGI("bias_gyro = [%f, %f, %f]", bias_gyro[0], bias_gyro[1], bias_gyro[2]);

    ret = 0;

out_close:
    close(fd);
    return ret;
}

static void parse_args(int argc, char **argv) {
    int c;

    for (;;) {
        int option_index = 0;
        static struct option long_options[] = {
            {"infmt",   required_argument, 0, 0 },
            {"outfmt",  required_argument, 0, 0 },
            {"cal_mag",  required_argument, 0, 0 },
            {"bias_ag",  required_argument, 0, 0 },
            {0,         0,                 0, 0 }
        };

        c = getopt_long(argc, argv, "", long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {
        case 0: {
            struct option *opt = &long_options[option_index];
            const char *name = opt->name;
            if (!strcmp(name, "infmt")) {
                arg_infmt = str2informat(optarg);
                if (arg_infmt == INFORMAT_INVALID) {
                    fprintf(stderr, "invalid infmt: %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
            }
            else if (!strcmp(name, "outfmt")) {
                arg_outfmt = str2outformat(optarg);
                if (arg_outfmt == OUTFORMAT_INVALID) {
                    fprintf(stderr, "invalid infmt: %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
            }
            else if (!strcmp(name, "cal_mag")) {
                if (load_cal_mag(optarg)) {
                    fprintf(stderr, "can't load magnetometer calibration from %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
            }
            else if (!strcmp(name, "bias_ag")) {
                if (load_bias_ag(optarg)) {
                    fprintf(stderr, "can't load accel+gyro bias from %s\n", optarg);
                    exit(EXIT_FAILURE);
                }
            }
            else {
                fprintf(stderr, "unsupported option %s\n", name);
                exit(EXIT_FAILURE);
            }
            break;
        }

        case '?':
            exit(EXIT_FAILURE);
            break;

        default:
            fprintf(stderr, "?? getopt returned character code 0%o ??\n", c);
            break;
        }
    }

    if (arg_infmt == INFORMAT_INVALID) {
        fprintf(stderr, "missing argument: infmt\n");
        exit(EXIT_FAILURE);
    }

    if (arg_outfmt == OUTFORMAT_INVALID) {
        fprintf(stderr, "missing argument: outfmt\n");
        exit(EXIT_FAILURE);
    }

    if (argc - optind != 1) {
        fprintf(stderr, "invalid arguments after options. expected=1, got:%d\n", argc - optind);
        exit(EXIT_FAILURE);
    }

    arg_src = argv[optind];
    if (str_startswith("i2c:", arg_src)) {
        arg_src += 4;
        arg_src_isi2c = true;
    }
}

static int write_sentral(uint64_t t, uint8_t alg_status, uint8_t event_status, uint8_t data_raw[EM7180_RAWDATA_SZ]) {
    size_t i;
    double tmp;

    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t temp_raw;
    int16_t baro_raw;
    uint32_t quat_raw[4];

    double accelfp[3];
    double gyrofp[3];
    double magfp[3];
    double temperature;
    double pressure;
    double quat[4];

    if (arg_outfmt == OUTFORMAT_RAW) {
        // write time
        if (fwrite(&t, sizeof(t), 1, stdout) != 1) {
            CROSSLOGE("can't write time");
            return -1;
        }

        // write alg status
        if (fwrite(&alg_status, sizeof(alg_status), 1, stdout) != 1) {
            CROSSLOGE("can't algorithm status");
            return -1;
        }

        // write event status
        if (fwrite(&event_status, sizeof(event_status), 1, stdout) != 1) {
            CROSSLOGE("can't event status");
            return -1;
        }

        // write sensor data
        if (fwrite(data_raw, EM7180_RAWDATA_SZ, 1, stdout) != 1) {
            CROSSLOGE("can't write data");
            return -1;
        }

        return 0;
    }

    if (arg_outfmt != OUTFORMAT_PROCESSED) {
        return -1;
    }

    em7180_parse_data_accelerometer(&data_raw[EM7180_RAWDATA_OFF_A], acc_raw, NULL);
    em7180_parse_data_gyroscope(&data_raw[EM7180_RAWDATA_OFF_G], gyro_raw, NULL);
    em7180_parse_data_magnetometer(&data_raw[EM7180_RAWDATA_OFF_M], mag_raw, NULL);
    em7180_parse_data_temperature(&data_raw[EM7180_RAWDATA_OFF_T], &temp_raw, NULL);
    em7180_parse_data_barometer(&data_raw[EM7180_RAWDATA_OFF_B], &baro_raw, NULL);
    em7180_parse_data_quaternion(&data_raw[EM7180_RAWDATA_OFF_Q], quat_raw, NULL);

    float lsb_acc = 32.0f / 65536.0f;
    float lsb_gyro = 10000.0f / 65536.0f;
    float lsb_mag = 2000.0f / 65536.0f;

    for (i=0; i < 3; i++) {
        accelfp[i] = acc_raw[i] * lsb_acc;
        gyrofp[i] = gyro_raw[i] * lsb_gyro;
        magfp[i] = mag_raw[i] * lsb_mag;
    }

    for (i=0; i < 4; i++) {
        float f;
        memcpy(&f, &quat_raw[i], sizeof(float));
        quat[i] = f;
    }

    temperature = temp_raw * 0.01f;
    pressure = baro_raw * 0.01f + 1013.25f;

    // accel: inverted(NED) -> ENU
    tmp = accelfp[0];
    accelfp[0] = -accelfp[1];
    accelfp[1] = -tmp;

    // gyro: NED -> ENU
    tmp = gyrofp[0];
    gyrofp[0] = gyrofp[1];
    gyrofp[1] = tmp;
    gyrofp[2] = -gyrofp[2];

    // mag: NED -> ENU
    tmp = magfp[0];
    magfp[0] = magfp[1];
    magfp[1] = tmp;
    magfp[2] = -magfp[2];

    // quat: xyzw -> wxyz
    tmp = quat[0];
    quat[0] = quat[3];
    quat[3] = quat[2];
    quat[2] = quat[1];
    quat[1] = tmp;

    // quat: NED -> ENU
    tmp = quat[1];
    quat[0] = quat[0];
    quat[1] = quat[2];
    quat[2] = tmp;
    quat[3] = -quat[3];

    if (fwrite(&t, sizeof(t), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(accelfp, sizeof(accelfp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(gyrofp, sizeof(gyrofp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(magfp, sizeof(magfp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&t, sizeof(t), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&temperature, sizeof(temperature), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&pressure, sizeof(pressure), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(quat, sizeof(quat), 1, stdout)!= 1) {
        return -1;
    }

    return 0;
}

static int write_sentral_pt_hdr(const struct bmp280_calib_param *calib_param) {
    struct mpu_cfg_dump cfg;

    mpu_get_cfg(&mpu9250, &cfg);

    // in processed mode we don't need the header
    if (arg_outfmt == OUTFORMAT_PROCESSED) {
        return 0;
    }

    if (arg_outfmt != OUTFORMAT_RAW) {
        return -1;
    }

    if (fwrite(&cfg, sizeof(cfg), 1, stdout)!= 1) {
        CROSSLOGE("can't write mpu cfg");
        return -1;
    }

    if (fwrite(calib_param, sizeof(*calib_param), 1, stdout)!= 1) {
        CROSSLOGE("can't write bmp cfg");
        return -1;
    }

    return 0;
}

static int write_sentral_pt(struct bmp280_calib_param *calib_param, const uint8_t data[USFS_TOTAL_SAMPLESZ]) {
    const uint8_t *mpu_data = &data[0];
    uint64_t mpu_time = *((uint64_t*)&data[MPU_RAWSZ]);
    const uint8_t *bmp_data = &data[MPU_SAMPLESZ];
    uint64_t bmp_time = *((uint64_t*)&data[MPU_SAMPLESZ + BMP_RAWSZ]);

    static int16_t accel[3] = {0, 0, 0};
    static int16_t gyro[3] = {0, 0, 0};
    static int16_t mag[3] = {0, 0, 0};

    int16_t _accel[3];
    int16_t _gyro[3];
    int16_t _mag[3];
    uint8_t sensors;

    double accelfp[3];
    double gyrofp[3];
    double magfp[3];

    float gyro_sens;
    uint16_t accel_sens;
    uint16_t mag_fsr;
    double tmp;
    int rc;
    int8_t brc;
    size_t i;
    double quat[4] = {1, 0, 0, 0};

    struct bmp280_uncomp_data uncomp_data;
    double temperature = 0;
    double pressure = 0;

    if (arg_outfmt == OUTFORMAT_RAW) {
        if (fwrite(data, USFS_TOTAL_SAMPLESZ, 1, stdout)!= 1) {
            CROSSLOGE("can't write data to stdout");
            return -1;
        }
        return 0;
    }

    if (arg_outfmt != OUTFORMAT_PROCESSED) {
        return -1;
    }

    rc = mpu_get_gyro_sens(&mpu9250, &gyro_sens);
    if (rc) {
        CROSSLOGE("can't get gyro sens");
        return -1;
    }

    rc = mpu_get_accel_sens(&mpu9250, &accel_sens);
    if (rc) {
        CROSSLOGE("can't get accel sens");
        return -1;
    }

    rc = mpu_get_compass_fsr(&mpu9250, &mag_fsr);
    if (rc) {
        CROSSLOGE("can't get mag fsr");
        return -1;
    }

    brc = bmp280_raw_to_uncomp(&uncomp_data, bmp_data);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_raw_to_uncomp: %d", brc);
        return -1;
    }

    sensors = mpu_parse_all_data(&mpu9250, mpu_data, _accel, _gyro, _mag);
    if (sensors & INV_XYZ_ACCEL) {
        accel[0] = _accel[0];
        accel[1] = _accel[1];
        accel[2] = _accel[2];
    }
    if (sensors & INV_XYZ_GYRO) {
        gyro[0] = _gyro[0];
        gyro[1] = _gyro[1];
        gyro[2] = _gyro[2];
    }
    if (sensors & INV_XYZ_COMPASS) {
        mag[0] = _mag[0];
        mag[1] = _mag[1];
        mag[2] = _mag[2];
    }

    brc = bmp280_get_comp_temp_double(&temperature, uncomp_data.uncomp_temp, calib_param);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_comp_temp_double: %d", brc);
        return -1;
    }

    brc = bmp280_get_comp_pres_double(&pressure, uncomp_data.uncomp_press, calib_param);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_comp_pres_double: %d", brc);
        return -1;
    }

    for (i = 0; i < 3; i++) {
        accelfp[i] = ((double)accel[i]) / ((double)accel_sens);

        // calibrate
        accelfp[i] += bias_accel[i];
    }

    for (i = 0; i < 3; i++) {
        gyrofp[i] = ((double)gyro[i]) / ((double)gyro_sens);

        // calibrate
        gyrofp[i] += bias_gyro[i];
    }

    for (i = 0; i < 3; i++) {
        magfp[i] = ((double)mag[i]) / (32768.0 / ((double)mag_fsr));
    }

    // mag: convert to ENU
    tmp = magfp[0];
    magfp[0] = magfp[1];
    magfp[1] = tmp;
    magfp[2] = -magfp[2];

    // pressure: Pa -> hPa
    pressure /= 100.0;

    // calibrate: mag center
    for (i = 0; i < 3; i++) {
        magfp[i] -= cal_mag_center[i];
    }

    // calibrate: mag matrix
    {
        double mx = magfp[0];
        double my = magfp[1];
        double mz = magfp[2];

        for (i = 0; i < 3; i++) {
            magfp[i] = cal_mag_tr[i][0] * mx + cal_mag_tr[i][1] * my + cal_mag_tr[i][2] * mz;
        }
    }

    if (fwrite(&mpu_time, sizeof(mpu_time), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(accelfp, sizeof(accelfp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(gyrofp, sizeof(gyrofp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(magfp, sizeof(magfp), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&bmp_time, sizeof(bmp_time), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&temperature, sizeof(temperature), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(&pressure, sizeof(pressure), 1, stdout)!= 1) {
        return -1;
    }

    if (fwrite(quat, sizeof(quat), 1, stdout)!= 1) {
        return -1;
    }

    return 0;
}

static int handle_i2c_sentral_pt(void) {
    int rc;
    int8_t brc;
    struct bmp280_config conf;
    uint8_t data[USFS_TOTAL_SAMPLESZ];

    rc = usfs_bmp280_create(&bmp280, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create bmp280");
        return -1;
    }

    rc = mpu_create(&mpu9250, MPU_TYPE_MPU6500, MAG_TYPE_AK8963, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create mpu9250");
        return -1;
    }

    rc = em7180_reset_request(&em7180);
    if (rc) {
        CROSSLOGE("can't reset em7180");
        return -1;
    }

    rc = em7180_set_run_mode(&em7180, false);
    if (rc) {
        CROSSLOGE("can't disable em7180 run mode");
        return -1;
    }

    rc = em7180_passthrough_enter(&em7180);
    if (rc) {
        CROSSLOGE("can't enter em7180 passthrough mode");
        return -1;
    }

    brc = bmp280_init(&bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_init: %d", brc);
        return -1;
    }

    brc = bmp280_get_config(&conf, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_config: %d", brc);
        return -1;
    }

    conf.filter = BMP280_FILTER_COEFF_16;
    conf.os_temp = BMP280_OS_2X;
    conf.os_pres = BMP280_OS_16X;
    conf.odr = BMP280_ODR_0_5_MS;

    brc = bmp280_set_config(&conf, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_config: %d", brc);
        return -1;
    }

    brc = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_power_mode: %d", brc);
        return -1;
    }

    rc = mpu_init(&mpu9250);
    if (rc) {
        CROSSLOGE("can't init mpu9250");
        return -1;
    }

    rc = mpu_set_sensors(&mpu9250, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    if (rc) {
        CROSSLOGE("mpu_set_sensors failed");
        return -1;
    }

    rc = mpu_set_sample_rate(&mpu9250, 200);
    if (rc) {
        CROSSLOGE("mpu_set_sample_rate failed");
        return -1;
    }

    rc = mpu_set_accel_fsr(&mpu9250, 8);
    if (rc) {
        CROSSLOGE("mpu_set_accel_fsr failed");
        return -1;
    }

    rc = mpu_set_gyro_fsr(&mpu9250, 2000);
    if (rc) {
        CROSSLOGE("mpu_set_gyro_fsr failed");
        return -1;
    }

    rc = write_sentral_pt_hdr(&bmp280.calib_param);
    if (rc) {
        CROSSLOGE("can't write header");
        return -1;
    }

    while (keep_running) {
        rc = mpu_get_all_data(&mpu9250, &data[0], (uint64_t*)&data[MPU_RAWSZ]);
        if (rc) continue;

        brc = bmp280_get_raw_data(&data[MPU_SAMPLESZ], &bmp280);
        if (brc != BMP280_OK) continue;
        *((uint64_t*)&data[MPU_SAMPLESZ + BMP_RAWSZ]) = usfs_get_us();

        write_sentral_pt(&bmp280.calib_param, data);
    }

    return 0;
}

static int handle_i2c_sentral(void) {
    int rc;

    rc = em7180_init(&em7180);
    if (rc) {
        CROSSLOGE("can't init em7180");
        return -1;
    }

    while (keep_running) {
        uint8_t event_status;
        uint8_t alg_status;
        uint8_t sensor_status;
        uint8_t error_reg;
        uint8_t data_raw[EM7180_RAWDATA_SZ];

        rc = em7180_get_event_status(&em7180, &event_status);
        if (rc) {
            CROSSLOGE("Unable to get event status (err %d)", rc);
            continue;
        }

        // don't read any data if the error flag is set
        if (event_status & EM7180_EVENT_ERROR) {
            rc = em7180_get_error_register(&em7180, &error_reg);
            if (rc) {
                CROSSLOGE("Unable to get error register (err %d)", rc);
                continue;
            }

            em7180_print_error((enum em7180_error)error_reg);
            continue;
        }

        // update the current algorithm status
        rc = em7180_get_algorithm_status(&em7180, &alg_status);
        if (rc) {
            CROSSLOGE("Unable to get algorithm status (err %d)", rc);
            continue;
        }

        // print any sensor communication errors
        rc = em7180_get_sensor_status(&em7180, &sensor_status);
        if (rc) {
            CROSSLOGE("Unable to get sensor status (err %d)", rc);
            continue;
        }
        if (sensor_status) {
            em7180_print_sensor_status(sensor_status);
        }

        // if there are no events, don't do anything
        if (!event_status) {
            continue;
        }

        // get all sensor data
        rc = em7180_get_data_all_raw(&em7180, data_raw);
        if (rc) {
            CROSSLOGE("Unable to get raw data (err %d)", rc);
            continue;
        }

        uint64_t now = usfs_get_us();

        write_sentral(now, alg_status, event_status, data_raw);
    }

    return 0;
}

static int handle_i2c(void) {
    int rc;
    int ret = -1;

    rc = crossi2c_linux_create(&i2cbus, arg_src);
    if (rc) {
        CROSSLOGE("can't create crossi2c");
        goto out;
    }

    rc = em7180_create(&em7180, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create em7180");
        goto out_i2cbus_destroy;
    }

    switch (arg_infmt) {
    case INFORMAT_SENTRAL:
        ret = handle_i2c_sentral();
        break;

    case INFORMAT_SENTRAL_PT:
        ret = handle_i2c_sentral_pt();
        break;

    default:
        CROSSLOGE("invalid infmt: %d", arg_infmt);
        ret = -1;
    }

out_i2cbus_destroy:
    rc = crossi2c_destroy(&i2cbus);
    if (rc) {
        CROSSLOGW("can't destroy i2cbus");
    }
out:
    return ret;
}

static int handle_file_sentral_pt(int fd) {
    int rc;
    ssize_t nbytes;
    struct mpu_cfg_dump cfg;
    struct bmp280_calib_param calib_param;
    uint8_t data[USFS_TOTAL_SAMPLESZ];

    nbytes = read_all(fd, &cfg, sizeof(cfg));
    if (nbytes != sizeof(cfg)) {
        CROSSLOG_ERRNO("can't read mpu config from file. nbytes=%zd", nbytes);
        return -1;
    }

    nbytes = read_all(fd, &calib_param, sizeof(calib_param));
    if (nbytes != sizeof(calib_param)) {
        CROSSLOG_ERRNO("can't read bmp calib param from file. nbytes=%zd", nbytes);
        return -1;
    }

    rc = mpu_create_nodev(&mpu9250, &cfg);
    if (rc) {
        CROSSLOGE("can't create nodev mpu9250");
        return -1;
    }

    rc = write_sentral_pt_hdr(&calib_param);
    if (rc) {
        CROSSLOGE("can't write header");
        return -1;
    }

    for (;;) {
        nbytes = read_all(fd, data, USFS_TOTAL_SAMPLESZ);
        if (nbytes < 0) {
            CROSSLOG_ERRNO("read_all");
            return -1;
        }

        if (nbytes == 0) {
            break;
        }

        if (nbytes != USFS_TOTAL_SAMPLESZ) {
            CROSSLOGE("short read");
            return -1;
        }

        write_sentral_pt(&calib_param, data);
    }

    return 0;
}

static int handle_file_sentral(int fd) {
    ssize_t nbytes;
    uint8_t data[sizeof(uint64_t) + 1 + 1 + EM7180_RAWDATA_SZ];

    for (;;) {
        nbytes = read_all(fd, data, sizeof(data));
        if (nbytes < 0) {
            CROSSLOG_ERRNO("read_all");
            return -1;
        }

        if (nbytes == 0) {
            break;
        }

        if (nbytes != sizeof(data)) {
            CROSSLOGE("short read");
            return -1;
        }

        uint64_t t = *((uint64_t*)data);
        uint8_t alg_status = data[sizeof(uint64_t)];
        uint8_t event_status = data[sizeof(uint64_t) + 1];
        uint8_t *raw_data = &data[sizeof(uint64_t) + 1 + 1];

        write_sentral(t, alg_status, event_status, raw_data);
    }

    return 0;
}

static int handle_file(void) {
    int ret;
    int fd;

    if (!strcmp(arg_src, "-"))
        fd = 0;
    else
        fd = open(arg_src, O_RDONLY, 0);

    if (fd < 0) {
        CROSSLOG_ERRNO("can't open %s", arg_src);
        return -1;
    }

    switch (arg_infmt) {
    case INFORMAT_SENTRAL:
        ret = handle_file_sentral(fd);
        break;

    case INFORMAT_SENTRAL_PT:
        ret = handle_file_sentral_pt(fd);
        break;

    default:
        CROSSLOGE("invalid infmt: %d", arg_infmt);
        ret = -1;
    }

    if (fd != 0)
        close(fd);

    return ret;
}

static void handle_signal(int signo) {
    (void)(signo);

    if (keep_running) {
        CROSSLOGI("clean stop");
        keep_running = false;
    }
    else {
        CROSSLOGE("force quit");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv) {
    int rc;

    signal(SIGINT, handle_signal);

    parse_args(argc, argv);

    if (arg_src_isi2c) {
        rc = handle_i2c();
    }
    else {
        rc = handle_file();
    }

    if (rc) {
        CROSSLOGE("EXIT WITH ERROR");
    }
    else {
        CROSSLOGI("done");
    }
    return rc;
}
