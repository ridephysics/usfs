#ifndef EM7180_P_H
#define EM7180_P_H

#ifdef __ZEPHYR__
    #include <kernel.h>
    #define em7180_usleep(x) k_sleep((x))
#elif defined(__unix__)
    #include <unistd.h>
    #define em7180_usleep(x) usleep((x))
#endif

int em7180_read(struct em7180 *dev, uint8_t reg, void *buf, size_t len);
int em7180_write_byte(struct em7180 *dev, uint8_t reg, uint8_t value);

#endif /* EM7180_P_H */
