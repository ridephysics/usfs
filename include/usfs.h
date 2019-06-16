#ifndef USFS_H
#define USFS_H

#ifdef __ZEPHYR__
    #include <kernel.h>
    #define usfs_usleep(x) k_sleep((x))
#elif defined(__unix__)
    #include <unistd.h>
    #include <time.h>

    #define usfs_usleep(x) usleep((x))

    static inline uint64_t usfs_get_us(void) {
        int rc;
        struct timespec ts;

        rc = clock_gettime(CLOCK_MONOTONIC, &ts);
        if (rc) {
            return 0;
        }

        return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    }
#elif defined(ESP_PLATFORM)
    #include <esp32/rom/ets_sys.h>
    #define usfs_usleep(x) ets_delay_us((x))
#else
    #error "unsupported platform"
#endif

#endif /* USFS_H */
