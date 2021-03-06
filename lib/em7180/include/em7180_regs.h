#ifndef EM7180_REGS_H
#define EM7180_REGS_H

// upload registers
#define EM7180_REG_UPLOAD_ADDR_0 0x94
#define EM7180_REG_UPLOAD_ADDR_1 0x95
#define EM7180_REG_UPLOAD_DATA 0x96
#define EM7180_REG_CRC_HOST_0 0x97
#define EM7180_REG_CRC_HOST_1 0x98
#define EM7180_REG_CRC_HOST_2 0x99
#define EM7180_REG_CRC_HOST_3 0x9a

// control registers
#define EM7180_REG_HOST_CTRL 0x34
#define EM7180_REG_RUN_STATUS 0x92
#define EM7180_REG_RESET_REQUEST 0x9b
#define EM7180_REG_ENABLE_EVENTS 0x33
#define EM7180_REG_EVENT_STATUS 0x35
#define EM7180_REG_ALGORITHM_CTRL 0x54
#define EM7180_REG_ALGORITHM_STATUS 0x38
#define EM7180_REG_PASSTRHOUGH_CTRL 0xa0
#define EM7180_REG_PASSTHROUGH_STATUS 0x9e

// parameter registers
#define EM7180_REG_MAG_RATE 0x55
#define EM7180_REG_ACCEL_RATE 0x56
#define EM7180_REG_GYRO_RATE 0x57
#define EM7180_REG_BARO_RATE 0x58
#define EM7180_REG_TEMP_RATE 0x59
#define EM7180_REG_ACTUAL_MAG_RATE 0x45
#define EM7180_REG_ACTUAL_ACCEL_RATE 0x46
#define EM7180_REG_ACTUAL_GYRO_RATE 0x47
#define EM7180_REG_ACTUAL_BARO_RATE 0x48
#define EM7180_REG_ACTUAL_TEMP_RATE 0x49
#define EM7180_REG_QRATE_DIVISOR 0x32
#define EM7180_REG_PARAM_REQUEST 0x64
#define EM7180_REG_LOAD_PARAM_BYTE0 0x60
#define EM7180_REG_LOAD_PARAM_BYTE1 0x61
#define EM7180_REG_LOAD_PARAM_BYTE2 0x62
#define EM7180_REG_LOAD_PARAM_BYTE3 0x63
#define EM7180_REG_SAVED_PARAM_BYTE0 0x3b
#define EM7180_REG_SAVED_PARAM_BYTE1 0x3c
#define EM7180_REG_SAVED_PARAM_BYTE2 0x3d
#define EM7180_REG_SAVED_PARAM_BYTE3 0x3e
#define EM7180_REG_PARAM_ACK 0x3a

// result registers
#define EM7180_REG_QX_0 0x00
#define EM7180_REG_QX_1 0x01
#define EM7180_REG_QX_2 0x02
#define EM7180_REG_QX_3 0x03
#define EM7180_REG_QY_0 0x04
#define EM7180_REG_QY_1 0x05
#define EM7180_REG_QY_2 0x06
#define EM7180_REG_QY_3 0x07
#define EM7180_REG_QZ_0 0x08
#define EM7180_REG_QZ_1 0x09
#define EM7180_REG_QZ_2 0x0a
#define EM7180_REG_QZ_3 0x0b
#define EM7180_REG_QW_0 0x0c
#define EM7180_REG_QW_1 0x0d
#define EM7180_REG_QW_2 0x0e
#define EM7180_REG_QW_3 0x0f
#define EM7180_REG_QT_0 0x10
#define EM7180_REG_QT_1 0x11

#define EM7180_REG_MX_0 0x12
#define EM7180_REG_MX_1 0x13
#define EM7180_REG_MY_0 0x14
#define EM7180_REG_MY_1 0x15
#define EM7180_REG_MZ_0 0x16
#define EM7180_REG_MZ_1 0x17
#define EM7180_REG_MT_0 0x18
#define EM7180_REG_MT_1 0x19

#define EM7180_REG_AX_0 0x1a
#define EM7180_REG_AX_1 0x1b
#define EM7180_REG_AY_0 0x1c
#define EM7180_REG_AY_1 0x1d
#define EM7180_REG_AZ_0 0x1e
#define EM7180_REG_AZ_1 0x1f
#define EM7180_REG_AT_0 0x20
#define EM7180_REG_AT_1 0x21

#define EM7180_REG_GX_0 0x22
#define EM7180_REG_GX_1 0x23
#define EM7180_REG_GY_0 0x24
#define EM7180_REG_GY_1 0x25
#define EM7180_REG_GZ_0 0x26
#define EM7180_REG_GZ_1 0x27
#define EM7180_REG_GT_0 0x28
#define EM7180_REG_GT_1 0x29

#define EM7180_REG_BD_0 0x2a
#define EM7180_REG_BD_1 0x2b
#define EM7180_REG_BT_0 0x2c
#define EM7180_REG_BT_1 0x2d

#define EM7180_REG_TD_0 0x2e
#define EM7180_REG_TD_1 0x2f
#define EM7180_REG_TT_0 0x30
#define EM7180_REG_TT_1 0x31

// troubleshooting registers
#define EM7180_REG_SENSOR_STATUS 0x36
#define EM7180_REG_SENTRAL_STATUS 0x37
#define EM7180_REG_ERROR 0x50

// device information registers
#define EM7180_REG_FEATURE_FLAGS 0x39
#define EM7180_REG_ROM_VERSION_0 0x70
#define EM7180_REG_ROM_VERSION_1 0x71
#define EM7180_REG_RAM_VERSION_0 0x72
#define EM7180_REG_RAM_VERSION_1 0x73
#define EM7180_REG_PRODUCT_ID  0x90
#define EM7180_REG_REVISION_ID 0x91

// GP
#define EM7180_REG_GP36 0x5b
#define EM7180_REG_GP37 0x5c
#define EM7180_REG_GP38 0x5d
#define EM7180_REG_GP39 0x5e
#define EM7180_REG_GP40 0x5f
#define EM7180_REG_GP50 0x69
#define EM7180_REG_GP51 0x6a
#define EM7180_REG_GP52 0x6b
#define EM7180_REG_GP53 0x6c
#define EM7180_REG_GP54 0x6d
#define EM7180_REG_GP55 0x6e
#define EM7180_REG_GP56 0x6f

// GP: named
#define EM7180_REG_ACC_LPF_BW EM7180_REG_GP36
#define EM7180_REG_GYRO_LPF_BW EM7180_REG_GP37
#define EM7180_REG_BARO_LPF_BW EM7180_REG_GP38

// XXX: the first three overlap with the LPF regs
#define EM7180_REG_CALIB_ACCEL_A_X_0 EM7180_REG_GP36
#define EM7180_REG_CALIB_ACCEL_A_X_1 EM7180_REG_GP37
#define EM7180_REG_CALIB_ACCEL_A_Y_0 EM7180_REG_GP38
#define EM7180_REG_CALIB_ACCEL_A_Y_1 EM7180_REG_GP39
#define EM7180_REG_CALIB_ACCEL_A_Z_0 EM7180_REG_GP40
#define EM7180_REG_CALIB_ACCEL_A_Z_1 EM7180_REG_GP50

#define EM7180_REG_CALIB_ACCEL_B_X_0 EM7180_REG_GP51
#define EM7180_REG_CALIB_ACCEL_B_X_1 EM7180_REG_GP52
#define EM7180_REG_CALIB_ACCEL_B_Y_0 EM7180_REG_GP53
#define EM7180_REG_CALIB_ACCEL_B_Y_1 EM7180_REG_GP54
#define EM7180_REG_CALIB_ACCEL_B_Z_0 EM7180_REG_GP55
#define EM7180_REG_CALIB_ACCEL_B_Z_1 EM7180_REG_GP56

#endif /* EM7180_REGS_H */
