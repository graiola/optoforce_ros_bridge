#ifndef OMD__SENSORCONFIG_H
#define OMD__SENSORCONFIG_H

#include <stdint.h>

/* Types of sensor configuration elems */

/* Sensor state for checking correct functioning between DAQ and Sensor */
enum sensor_state:unsigned int { no_sensor = 0, overload_x, overload_y, overload_z, sensor_failure, sensor_ok, conn_faliure };

/* Sending data speed from DAQ */
enum sensor_speed:unsigned int { speed_1000hz = 0, speed_333hz, speed_100hz, speed_30hz };

/* Filter mode in sending of DAQ */
enum sensor_filter:unsigned int { no_filter = 0, filter_150hz, filter_50hz, filter_15hz };

/* Sending mode */
enum sensor_mode:unsigned int { mode_raw = 0, mode_comp };

/* Configuration datatype */

struct SensorConfig {
    sensor_mode mode: 1;
    sensor_filter filter: 2;
    sensor_speed speed: 2;
    sensor_state state: 3;
    SensorConfig (sensor_state st, sensor_speed sp, sensor_filter ft, sensor_mode rf);
    SensorConfig ();

    void set(sensor_state st, sensor_speed sp, sensor_filter ft, sensor_mode rf);
    SensorConfig null_sensor();
    /* Raw configuration reading from a 8bit data */
    static SensorConfig from_uint8_t (uint8_t c) {
        int i = c;
        return * (SensorConfig*) (&i);
    }
    static uint8_t to_uin8_t (SensorConfig c) {
        return *(uint8_t*)(&c);
    }
    uint8_t to_uin8_t () const;


    int getState ();
    int getSpeed ();
    int getFilter ();
    int getMode ();
    void setSpeed(int sp);
    void setFilter(int ft);
    void setMode(int md);

};

/*extern "C" OMD_EXPORT_OTHER void SensorConfig_setMODE(void* conf, sensor_mode mode);
extern "C" OMD_EXPORT_OTHER void SensorConfig_setSPEED(void* conf, sensor_speed speed);
extern "C" OMD_EXPORT_OTHER void SensorConfig_setFILTER(void* conf, sensor_filter filter);
extern "C" OMD_EXPORT_OTHER void SensorConfig_setSTATE(void* conf, sensor_state filter);

extern "C" OMD_EXPORT_OTHER int SensorConfig_getMODE(void* conf);
extern "C" OMD_EXPORT_OTHER int SensorConfig_getSPEED(void* conf);
extern "C" OMD_EXPORT_OTHER int SensorConfig_getFILTER(void* conf);
extern "C" OMD_EXPORT_OTHER int SensorConfig_getSTATE(void* conf);

extern "C" OMD_EXPORT_OTHER void* callSensorConfig_Create();
extern "C" OMD_EXPORT_OTHER void callSensorConfig_Dispose(void* conf);
extern "C" OMD_EXPORT_OTHER void callSet(void* conf, sensor_state state, sensor_speed speed, sensor_filter filter, sensor_mode mode);
extern "C" OMD_EXPORT_OTHER int callGetState(void* conf);
extern "C" OMD_EXPORT_OTHER int callGetSpeed(void* conf);
extern "C" OMD_EXPORT_OTHER int callGetState(void* conf);
extern "C" OMD_EXPORT_OTHER int callGetFilter(void* conf);
extern "C" OMD_EXPORT_OTHER void callSetSpeed(void* conf, int speed);
extern "C" OMD_EXPORT_OTHER void callSetMode(void* conf, int mode);
extern "C" OMD_EXPORT_OTHER void callSetFilter(void* conf, int filter);*/



#endif // OMD__SENSORCONFIG_H
