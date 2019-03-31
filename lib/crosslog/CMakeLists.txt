set(CROSSLOG_SRCS_COMMON
    crosslog.c
)
set(CROSSLOG_INCS_COMMON
    include
)

if(PROJECT_USE_ZEPHYR)
zephyr_library_named(crosslog)
zephyr_library_sources(
    ${CROSSLOG_SRCS_COMMON}
)
target_include_directories(crosslog PUBLIC
    ${CROSSLOG_INCS_COMMON}
)
else()
cmake_minimum_required(VERSION 2.8)
project(crosslog)

add_library(crosslog
    ${CROSSLOG_SRCS_COMMON}
)
target_include_directories(crosslog PUBLIC
    ${CROSSLOG_INCS_COMMON}
)
endif()
