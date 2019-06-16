#ifndef USFS_H
#define USFS_H

#ifdef __ZEPHYR__
    #include <kernel.h>
    #define usfs_usleep(x) k_sleep((x))
#elif defined(__unix__)
    #include <unistd.h>
    #define usfs_usleep(x) usleep((x))
#elif defined(ESP_PLATFORM)
    #include <esp32/rom/ets_sys.h>
    #define usfs_usleep(x) ets_delay_us((x))
#else
    #error "unsupported platform"
#endif

#endif /* USFS_H */
