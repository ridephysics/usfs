#include <em7180.h>

#define CROSSLOG_TAG "em7180"
#include <crosslog.h>

void em7180_print_feature_flags(uint8_t flags) {
    CROSSLOGD("features:");

    if (flags & EM7180_FF_BAROMETER)
        CROSSLOGD("\tbarometer");
    if (flags & EM7180_FF_HUMIDITY_SENSOR)
        CROSSLOGD("\thumidity sensor");
    if (flags & EM7180_FF_TEMP_SENSOR)
        CROSSLOGD("\ttemperature sensor");
    if (flags & EM7180_FF_CUSTOM_SENSOR_1)
        CROSSLOGD("\tcustom sensor 1");
    if (flags & EM7180_FF_CUSTOM_SENSOR_2)
        CROSSLOGD("\tcustom sensor 2");
    if (flags & EM7180_FF_CUSTOM_SENSOR_3)
        CROSSLOGD("\tcustom sensor 3");
    if (flags & EM7180_FF_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (flags & EM7180_FF_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_sentral_status(uint8_t status) {
    CROSSLOGD("sentral status:");

    if (status & EM7180_SS_EEPROM_DETECTED)
        CROSSLOGD("\teeprom detected");
    if (status & EM7180_SS_EEPROM_CFG_UPLOADED)
        CROSSLOGD("\teeprom config uploaded");
    if (status & EM7180_SS_EEPROM_CRC_ERR)
        CROSSLOGD("\teeprom crc error");
    if (status & EM7180_SS_INITIALIZED)
        CROSSLOGD("\tinitialized");
    if (status & EM7180_SS_NO_EEPROM)
        CROSSLOGD("\tno eeprom");
    if (status & EM7180_SS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_SS_RESERVED2)
        CROSSLOGD("\treserved 2");
    if (status & EM7180_SS_RESERVED3)
        CROSSLOGD("\treserved 3");
}

void em7180_print_algorithm_ctrl(uint8_t ctrl) {
    CROSSLOGD("algorithm control:");

    if (ctrl & EM7180_AC_STANDBY_ENABLE)
        CROSSLOGD("\tstandby enable");
    if (ctrl & EM7180_AC_RAWDATA_ENABLE)
        CROSSLOGD("\trawdata enable");
    if (ctrl & EM7180_AC_HPR_OUTPUT)
        CROSSLOGD("\thpr output");
    if (ctrl & EM7180_AC_6AXIS_ENABLE)
        CROSSLOGD("\t6-axis enable");
    if (ctrl & EM7180_AC_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (ctrl & EM7180_AC_ENU_OUTPUT)
        CROSSLOGD("\tenu output");
    if (ctrl & EM7180_AC_DIS_GYR_WHEN_STILL)
        CROSSLOGD("\tdisable gyro when still");
    if (ctrl & EM7180_AC_PARAM_TRANSFER)
        CROSSLOGD("\tparam transfer");
}

void em7180_print_algorithm_status(uint8_t status) {
    CROSSLOGD("algorithm status:");

    if (status & EM7180_AS_STANDBY)
        CROSSLOGD("\tstandby");
    if (status & EM7180_AS_SLOW)
        CROSSLOGD("\talgorithm slow");
    if (status & EM7180_AS_STILLNESS)
        CROSSLOGD("\tstillness");
    if (status & EM7180_AS_CAL_STABLE)
        CROSSLOGD("\tcal stable");
    if (status & EM7180_AS_MAG_TRANSIENT)
        CROSSLOGD("\tmag transient");
    if (status & EM7180_AS_UNRELIABLE)
        CROSSLOGD("\tunreliable sensor data");
    if (status & EM7180_AS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_AS_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_event_status(uint8_t status) {
    CROSSLOGD("event status:");

    if (status & EM7180_EVENT_CPURESET)
        CROSSLOGD("\tcpu reset");
    if (status & EM7180_EVENT_ERROR)
        CROSSLOGD("\terror");
    if (status & EM7180_EVENT_QUAT_RES)
        CROSSLOGD("\tquaternion result");
    if (status & EM7180_EVENT_MAG_RES)
        CROSSLOGD("\tmag result");
    if (status & EM7180_EVENT_ACCEL_RES)
        CROSSLOGD("\taccel result");
    if (status & EM7180_EVENT_GYRO_RES)
        CROSSLOGD("\tgyro result");
    if (status & EM7180_EVENT_BARO_RES)
        CROSSLOGD("\tbaro result");
    if (status & EM7180_EVENT_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_sensor_status(uint8_t status) {
    CROSSLOGD("sensor status:");

    if (status & EM7180_SENSORSTATUS_MAG_NACK)
        CROSSLOGD("\tmagnetometer NACK");
    if (status & EM7180_SENSORSTATUS_ACCEL_NACK)
        CROSSLOGD("\taccelerometer NACK");
    if (status & EM7180_SENSORSTATUS_GYRO_NACK)
        CROSSLOGD("\tgyroscope NACK");
    if (status & EM7180_SENSORSTATUS_RESERVED1)
        CROSSLOGD("\treserved 1");
    if (status & EM7180_SENSORSTATUS_MAGID_ERROR)
        CROSSLOGD("\tmag id error");
    if (status & EM7180_SENSORSTATUS_ACCELID_ERROR)
        CROSSLOGD("\taccel id error");
    if (status & EM7180_SENSORSTATUS_GYROID_ERROR)
        CROSSLOGD("\tgyro id error");
    if (status & EM7180_SENSORSTATUS_RESERVED2)
        CROSSLOGD("\treserved 2");
}

void em7180_print_error(enum em7180_error error) {
    switch (error) {
    case EM7180_ERR_NONE:
        break;
    case EM7180_ERR_MAG_FAILURE:
        CROSSLOGD("\tmagnetometer failure");
        break;
    case EM7180_ERR_ACC_FAILURE:
        CROSSLOGD("\taccelerometer failure");
        break;
    case EM7180_ERR_GYRO_FAILURE:
        CROSSLOGD("\tgyroscope failure");
        break;
    case EM7180_ERR_MAG_INIT_FAILURE:
        CROSSLOGD("\tmagnetometer init failure");
        break;
    case EM7180_ERR_ACC_INIT_FAILURE:
        CROSSLOGD("\taccelerometer init failure");
        break;
    case EM7180_ERR_GYRO_INIT_FAILURE:
        CROSSLOGD("\tgyroscope init failure");
        break;
    case EM7180_ERR_MATH:
        CROSSLOGD("\tmathematical error");
        break;
    case EM7180_ERR_INVALID_SAMPLE_RATE:
        CROSSLOGD("\tinvalid sample rate");
        break;
    default:
        CROSSLOGD("\tunknown(%02x)", error);
        break;
    }
}
