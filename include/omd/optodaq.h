#ifndef OMD__OPTODAQ_H
#define OMD__OPTODAQ_H

#include "omd/optopackage.h"
#include "omd/optopackage6d.h"
#include "omd/sensorconfig.h"
#include "omd/optoports.h"

class ReaderThread;
class OptoDAQ_private;


/* OptoDAQ state machine internal states */

class OptoDAQ{

private:

    OptoDAQ_private *d_ptr;


public:
    OptoDAQ ();
    ~OptoDAQ ();


    /* Returns the actual sensor configuration */
    /* You can find sensor_configuration struct definition in sensor_configuration.h */
    SensorConfig getConfig();


    /* If offset is cleared, read function returns raw data */
    bool unzero(int number=0);
    void unzeroAll();
    /* clearOffset for the four virtual sensor */
    //void unzeroVirtuals();
    //void unzeroVirtual(int number);

    /* An automatic calibration function for each sensor (virtual/non-virtual)
     * They set the offset to zero the actual signal
     *
     * For example: STATE 1: after calling a read: x=245 y=520 z=324 s1=5896 s2=6502 s3=6502 s4=6502
     *              STATE 3: calling calibrateSensor()
     *              STATE 2: after calling a read again: x=0 y=-1 z=0 s1=-1 s2=-1 s3=0 s4=0
     *
     * You can reset the raw signal using the clearOffset() functions */
    bool zero(int number=0);
    void zeroAll();
    /* Virtual sensor calibrators */
    //void zeroVirtuals();
    //void zeroVirtual(int number);

    /* The internal buffer's actual size in bytes */
    int getSize();

    /* Returns true if the checksum is valid (only for the _67 and the _94 type devices) */
    bool isChksumOK();

    /* Offset getters for the DAQ */
    /* Each one returns a package of offsets for all the signals coming from the sensor */

    /* Returns the actual sensor version */
    opto_version getVersion();

    OptoPackage getOffset(int sensor);

    /* Return number of bytes of one package size */
    int getBytesPerRead();

    /* Tries to open a port with a portname */
    /* Returns true if open was successful, else returns false */
    bool open (OPort port);

    /* Close the port (it it's opened) */
    void close ();

    /* Returns the actual portname in use */
    char* getPortName();

    /* Returns true if port is opened */
    bool isOpen ();

    /* Returns true if the sensor is virtual (four sensor instead of one) */
    bool isVirtual();

    int getSensorSize();

    /* Returns true once if there was a valid package */
    /* It can be true again after reopening the port */
    //bool isFirstValid();

    /* Read functions */

    /* Reading from the DAQ (open it first!)*/

    /* A simple read function, that gets a package as an argument */
    /* Returns  -1: buffer is full                   */
    /*           0: no bytes available               */
    /* buffer size: number of packages in the buffer */

    /*peek argument: if it's true it let's you reading the data without clearing the waiting queue of the port */
    int read(OptoPackage& package, int sensor, bool peek=false);

    /* This function needs an 8 integer size array as input and gives back the package values as int (x y z s1 s2 s3 s4 temp) */
    int readArray(int (&arr)[8], bool peek=false);

    /* Same as the previous one, except it reads all the four virtual sensor into a 32 int size buffer */
    /* S1(x y z s1 s2 s3 s4 temp) ... S4(x y z s1 s2 s3 s4 temp)  */
    int readVirtualArray(int (&virtArray)[128], bool peek=false);

    /* Reads four virtual package from the DAQ */
    int readVirtual(OptoPackage (&packArray)[16], bool peek=false);

    /* Reads four virtual buffer from the DAQ into a 4 sized package pointer array (pointers should be zeroed first to allocate correctly) */
    /* Pointers should be initialized to 0 at the first time you call this function */
    int readAll(OptoPackage *&buffer, bool peek=false);

    int read6D(OptoPackage6D& package, bool peek=false);

    int readAll6D(OptoPackage6D *&buffer, bool peek=false);

    /* Reads and calculates the six axis sensor actal values into a 6 integer size array (vectorx, vectory, vectorz, torquex, torquey, torquez) */
    int read6Axis(int (&axis6)[6], bool peek=false);

    /* Reads and calculates the 6 axis sensor data as a dynamic array */
    int readAll6Axis(int *&axis6Array, bool peek=false);

    /* Send a configuration to the DAQ */
    bool sendConfig (SensorConfig c);
};

// wrapper classes for other languages
/*extern "C" OMD_EXPORT_OTHER void* callOptoDAQ_Create();
extern "C" OMD_EXPORT_OTHER void callOptoDAQ_Dispose(void* daq);
extern "C" OMD_EXPORT_OTHER void callDeletePackageArray(void *&arr);
extern "C" OMD_EXPORT_OTHER void callDeleteIntArray(int *&arr);
extern "C" OMD_EXPORT_OTHER SensorConfig callGetConf(void* daq);
extern "C" OMD_EXPORT_OTHER int callGetSensorSize(void* daq);
extern "C" OMD_EXPORT_OTHER bool callUnzero(void* daq, int number);
extern "C" OMD_EXPORT_OTHER void callUnzeroAll(void* daq);
extern "C" OMD_EXPORT_OTHER bool callZero(void* daq, int number);
extern "C" OMD_EXPORT_OTHER void callZeroAll(void* daq);
extern "C" OMD_EXPORT_OTHER int callGetSize(void* daq);
extern "C" OMD_EXPORT_OTHER bool callIsChksumOK(void* daq);
extern "C" OMD_EXPORT_OTHER int callGetActualVersion(void* daq);
extern "C" OMD_EXPORT_OTHER int callGetBytesPerRead(void* daq);
extern "C" OMD_EXPORT_OTHER bool callOpen (void* daq, char* portname);
extern "C" OMD_EXPORT_OTHER void callClose (void* daq);
extern "C" OMD_EXPORT_OTHER char* callGetPort(void* daq);
extern "C" OMD_EXPORT_OTHER bool callIsOpen (void* daq);
extern "C" OMD_EXPORT_OTHER bool callIsVirtual(void* daq);
//extern "C" OMD_EXPORT_OTHER bool callIsFirstValid(void* daq);
extern "C" OMD_EXPORT_OTHER int callRead(void* daq, OptoPackage& package, bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadArray(void* daq, int (&arr)[8], bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadVirtualArray(void* daq, int (&virtArr)[32], bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadVirtual(void* daq, OptoPackage (&packArr)[16], bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadAll(void* daq, OptoPackage *&packDeq, bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadAllVirtual(void* daq, OptoPackage *&packDeqArr, bool peek=false);
extern "C" OMD_EXPORT_OTHER int callRead6Axis(void* daq, int (&axis6)[6], bool peek=false);
extern "C" OMD_EXPORT_OTHER int callReadAll6Axis(void* daq, int *&axis6ArrDeq, bool peek=false);
extern "C" OMD_EXPORT_OTHER void callSendConfig (void* daq, SensorConfig c);*/



#endif // OMD__OPTODAQ_H
