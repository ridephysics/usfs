#include <getopt.h>
#include <crossi2c/linux.h>
#include <em7180.h>
#include <usfs/bmp280.h>
#include <inv_mpu.h>
#include <usfs.h>
#include <fcntl.h>
#include <signal.h>

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

static void parse_args(int argc, char **argv) {
    int c;

    for (;;) {
        int option_index = 0;
        static struct option long_options[] = {
            {"infmt",   required_argument, 0, 0 },
            {"outfmt",  required_argument, 0, 0 },
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

#if 0
static int write_sentral(void) {
    // TODO
    return -1;
}
#endif

#define BMP_RAWSZ (6)
#define MPU_SAMPLESZ (MPU_RAWSZ + sizeof(uint64_t))
#define BMP_SAMPLESZ (BMP_RAWSZ + sizeof(uint64_t))
#define TOTAL_SAMPLESZ (MPU_SAMPLESZ + BMP_SAMPLESZ)

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

static int write_sentral_pt(struct bmp280_calib_param *calib_param, const uint8_t data[TOTAL_SAMPLESZ]) {
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
        if (fwrite(data, TOTAL_SAMPLESZ, 1, stdout)!= 1) {
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

    if (fwrite(&mpu_time, sizeof(mpu_time), 1, stdout)!= 1) {
        return -1;
    }

    // TODO: calibrate
    for (i = 0; i < 3; i++) {
        accelfp[i] = ((double)accel[i]) / ((double)accel_sens);
    }

    // TODO: calibrate
    for (i = 0; i < 3; i++) {
        gyrofp[i] = ((double)gyro[i]) / ((double)gyro_sens);
    }

    // TODO: calibrate
    for (i = 0; i < 3; i++) {
        magfp[i] = ((double)mag[i]) / (32768.0 / ((double)mag_fsr));
    }

    // mag: convert to ENU
    tmp = magfp[0];
    magfp[0] = magfp[1];
    magfp[1] = tmp;
    magfp[2] = -magfp[2];

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

    if (fwrite(&quat, sizeof(quat), 1, stdout)!= 1) {
        return -1;
    }

    return 0;
}

static int handle_i2c_sentral_pt(void) {
    int rc;
    int8_t brc;
    struct bmp280_config conf;
    uint8_t data[TOTAL_SAMPLESZ];

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
    // TODO
    return -1;
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

static int handle_file(void) {
    int ret = -1;
    int rc;
    int fd;
    ssize_t nbytes;
    struct mpu_cfg_dump cfg;
    struct bmp280_calib_param calib_param;
    uint8_t data[TOTAL_SAMPLESZ];

    if (!strcmp(arg_src, "-"))
        fd = 0;
    else
        fd = open(arg_src, O_RDONLY, 0);

    if (fd < 0) {
        CROSSLOG_ERRNO("can't open %s", arg_src);
        return -1;
    }

    nbytes = read_all(fd, &cfg, sizeof(cfg));
    if (nbytes != sizeof(cfg)) {
        CROSSLOG_ERRNO("can't read mpu config from file. nbytes=%zd", nbytes);
        goto out_close;
    }

    nbytes = read_all(fd, &calib_param, sizeof(calib_param));
    if (nbytes != sizeof(calib_param)) {
        CROSSLOG_ERRNO("can't read bmp calib param from file. nbytes=%zd", nbytes);
        goto out_close;
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

    // TODO: handle sentral mode
    for (;;) {
        nbytes = read_all(fd, data, TOTAL_SAMPLESZ);
        if (nbytes < 0) {
            CROSSLOG_ERRNO("read_all");
            goto out_close;
        }

        if (nbytes == 0) {
            break;
        }

        if (nbytes != TOTAL_SAMPLESZ) {
            CROSSLOGE("short read");
            goto out_close;
        }

        write_sentral_pt(&calib_param, data);
    }

    ret = 0;

out_close:
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
