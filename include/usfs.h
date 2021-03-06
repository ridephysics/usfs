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
    #include <esp_timer.h>

    #define usfs_usleep(x) ets_delay_us((x))

    static inline uint64_t usfs_get_us(void) {
	    int64_t now = esp_timer_get_time();
	    if (now <= 0)
		    return 0;
	    return (uint64_t) now;
    }
#else
    #error "unsupported platform"
#endif

#define BMP_RAWSZ (6)
#define MPU_SAMPLESZ (MPU_RAWSZ + sizeof(uint64_t))
#define BMP_SAMPLESZ (BMP_RAWSZ + sizeof(uint64_t))
#define USFS_TOTAL_SAMPLESZ (MPU_SAMPLESZ + BMP_SAMPLESZ)

#endif /* USFS_H */
