cmake_minimum_required(VERSION 2.8)
project(usfstools)

add_executable(usfs_reader
    reader.c
)
target_link_libraries(usfs_reader
    usfs
    crosslog
    crossi2c
    em7180
    bmp280
    eMPL
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
