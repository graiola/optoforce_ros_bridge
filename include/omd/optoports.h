#ifndef OMD__OPTOPORTS_H
#define OMD__OPTOPORTS_H

#include <stddef.h>
#include <stdlib.h>

class OptoPorts_private;


struct OPort
{
    /*   A structure for containing an OptoForce port
     *   name - the name of the serial port
     *   deviceName - not defined now (every sensor's identical name) */

    char name[25];
    char deviceName[25];

    OPort();
    ~OPort();
};

class OptoPorts
{

    OptoPorts_private *d_ptr;
public:
    /* private class for embedding into a dll */

    OptoPorts();
    ~OptoPorts();

    /* Returns the port list as an allocated array of oPorts */
    /* Only the supported sensors are available in the list */
    OPort* listPorts(bool connectFilter);

    /* returns the actual size of the port list */
    int getSize(bool connectFilter);

    /* Returns the size, when listports called */
    /* For counting oPorts in the list */
    int getLastSize();

    /* Queries the list and returns true if a new port was found */
    bool isNewPort(bool connectFilter);

    /* Returns true if there is a lost port at the moment */
    bool isLostPort();

    /* Returns the port that is found when isNewPort was called */
    OPort getNewPort();

    /* Returns the port that is found when isLostPort was called */
    OPort getLostPort();


};

/*extern "C" OMD_EXPORT_OTHER void callDeletePortList(void* port);
extern "C" OMD_EXPORT_OTHER void* callOptoPorts_Create();
extern "C" OMD_EXPORT_OTHER void callOptoPorts_Dispose(void* ports);
extern "C" OMD_EXPORT_OTHER void* callListPorts(void* ports);
extern "C" OMD_EXPORT_OTHER int callGetSizePorts(void* ports);
extern "C" OMD_EXPORT_OTHER int callGetLastSize(void* ports);
extern "C" OMD_EXPORT_OTHER bool callIsNewPort(void* ports);
extern "C" OMD_EXPORT_OTHER bool callIsLostPort(void* ports);
extern "C" OMD_EXPORT_OTHER oPort callGetNewPort(void* ports);
extern "C" OMD_EXPORT_OTHER oPort callGetLostPort(void* ports);*/

#endif // OMD__OPTOPORTS_H
