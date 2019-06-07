cmake_minimum_required(VERSION 2.8)
project(usfstools)

add_executable(usfs_test
    test.c
)
target_link_libraries(usfs_test
    usfs
    crosslog
    crossi2c
    em7180
)

add_executable(eeprom_dump
    eeprom_dump.c
)
target_link_libraries(eeprom_dump
    crosslog
    crossi2c
    em7180
    m24512
)

add_executable(rawdata_decoder
    rawdata_decoder.c
)
target_link_libraries(rawdata_decoder
    crosslog
    em7180
)
