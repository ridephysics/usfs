#ifndef CROSSLOG_H
#define CROSSLOG_H

#ifndef CROSSLOG_TAG
#define CROSSLOG_TAG "default"
#endif

#ifdef __unix__
    #include <stdio.h>
    #include <unistd.h>
    #include <string.h>
    #include <errno.h>

    #define CROSSLOG_COLOR_RESET "\x1B[0m"
    #define CROSSLOG_COLOR_PREFIX "\x1B[38;5;"
    #define CROSSLOG_COLOR_BLUE    CROSSLOG_COLOR_PREFIX "75m"
    #define CROSSLOG_COLOR_GREEN   CROSSLOG_COLOR_PREFIX "40m"
    #define CROSSLOG_COLOR_ORANGE  CROSSLOG_COLOR_PREFIX "202m"
    #define CROSSLOG_COLOR_RED     CROSSLOG_COLOR_PREFIX "196m"
    #define CROSSLOG_COLOR(x) (isatty(STDERR_FILENO) ? CROSSLOG_COLOR_ ## x : "")

    #define CROSSLOG_INTERNAL(prefix, color, fmt, ...) \
        fprintf(stderr, "%s" prefix " " CROSSLOG_TAG ": " fmt "%s\n", \
            CROSSLOG_COLOR(color), ##__VA_ARGS__, CROSSLOG_COLOR(RESET) \
        )

    #define CROSSLOGD(fmt, ...) CROSSLOG_INTERNAL("D", BLUE, fmt, ##__VA_ARGS__)
    #define CROSSLOGI(fmt, ...) CROSSLOG_INTERNAL("I", GREEN, fmt, ##__VA_ARGS__)
    #define CROSSLOGW(fmt, ...) CROSSLOG_INTERNAL("W", ORANGE, fmt, ##__VA_ARGS__)
    #define CROSSLOGE(fmt, ...) CROSSLOG_INTERNAL("E", RED, fmt, ##__VA_ARGS__)

    #define CROSSLOG_ERRNO(fmt, ...) CROSSLOGE(fmt ": %s(%d)", ##__VA_ARGS__, strerror(errno), errno)
#else
    #error "unsupported platform"
#endif

#endif /* CROSSLOG_H */
