#include <unistd.h>
#include <fcntl.h>
#include <em7180.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static float u32_to_f(uint32_t v) {
    union {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 = v;

    return u.f;
}

int main(int argc, char **argv) {
    int fd;
    ssize_t nbytes;
    uint8_t data_raw[8 + 1 + 1 + 50];

    if (argc != 2) {
        CROSSLOGE("usage: %s FILENAME", argv[0]);
        return -1;
    }

    fd = open(argv[1], O_RDONLY, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("open");
        return -1;
    }

    uint64_t nsamples = 0;
    for (;;) {
        nbytes = read(fd, data_raw, sizeof(data_raw));
        if (nbytes < 0) {
            CROSSLOG_ERRNO("read");
            return -1;
        }

        if (nbytes == 0)
            break;

        if ((size_t)nbytes != sizeof(data_raw)) {
            CROSSLOGE("short read");
            return -1;
        }

        uint8_t *ptr = data_raw;

        uint64_t uptime = *((uint64_t*)ptr);
        ptr += 8;

        uint8_t alg_status = *ptr++;
        uint8_t event_status = *ptr++;
        uint8_t *imudata = ptr;

        CROSSLOGD("uptime=%lu", uptime);
        em7180_print_algorithm_status(alg_status);
        em7180_print_event_status(event_status);

        if (event_status & EM7180_EVENT_QUAT_RES)
        {
            uint32_t quat[4];
            uint16_t time;
            em7180_parse_data_quaternion(&imudata[EM7180_RAWDATA_OFF_Q], quat, &time);
            CROSSLOGD("[%lu:%u]quat: %f|%f|%f|%f", uptime, time, u32_to_f(quat[0]), u32_to_f(quat[1]),
                u32_to_f(quat[2]), u32_to_f(quat[3]));
        }

        nsamples++;
    }

    close(fd);

    CROSSLOGI("nsamples: %lu", nsamples);

    return 0;
}
