#ifndef OMD__OPTOPACKAGE_H
#define OMD__OPTOPACKAGE_H

#include <stdint.h>
#include "omd/sensorconfig.h"
#include <stddef.h>
#include <stdlib.h>

/* versions of the packages */
enum opto_version {
    undefined_version = 0, _66, _67, _68, _94, _95
};

/* Representation of a one package */

struct OptoPackage {

  /* Raw datas */
  int s1;
  int s2;
  int s3;
  int s4;

  /* Compensated raw datas */
  int s1c;
  int s2c;
  int s3c;
  int s4c;

  /* Force vectors */
  int x;
  int y;
  int z;

  /* Compensated force vectors */
  int xc;
  int yc;
  int zc;

  /* Inconsistent raw signals */
  int is1;
  int is2;
  int is3;
  int is4;

  /* Inconsistent, compensated raw signals */
  int is1c;
  int is2c;
  int is3c;
  int is4c;

  /* Temp data */
  int temp;

  /* Configuration of DAQ */
  SensorConfig config;

  /* Version of data package */
  int vs;

  OptoPackage ();
  ~OptoPackage();
  OptoPackage (opto_version v);

  /* the package is correct */
  bool isCorrect ();

  /* implication of raw/force conditions */
  bool isRaw();

  /* probate the invariant of package */
  void setInvariant();

  void saveInconsistent();

  void saveAsInconsistent();

  /* probate the invariant of package with an offset */
  void setInvariant(const OptoPackage& offset);

  /* Helper functions for easy package operations */

  const OptoPackage& operator= (int pack);

  OptoPackage operator+(const OptoPackage& pack);

  OptoPackage operator-(const OptoPackage& pack);

  OptoPackage operator-(int value);

  OptoPackage operator*=(const OptoPackage& pack);

  OptoPackage operator/=(const int num);

  OptoPackage operator/=(const OptoPackage& pack);

};

/*extern "C" OMD_EXPORT_OTHER void* OptoPackage_Create();
extern "C" OMD_EXPORT_OTHER void OptoPackage_Dispose(void* pack);

extern "C" OMD_EXPORT_OTHER void OptoPackage_SetS1(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetS2(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetS3(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetS4(void* pack, int value);

extern "C" OMD_EXPORT_OTHER void OptoPackage_SetX(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetY(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetZ(void* pack, int value);

extern "C" OMD_EXPORT_OTHER void OptoPackage_SetIS1(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetIS2(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetIS3(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetIS4(void* pack, int value);

extern "C" OMD_EXPORT_OTHER void OptoPackage_SetTEMP(void* pack, int value);

extern "C" OMD_EXPORT_OTHER int OptoPackage_GetS1(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetS2(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetS3(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetS4(void* pack);

extern "C" OMD_EXPORT_OTHER int OptoPackage_GetX(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetY(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetZ(void* pack);

extern "C" OMD_EXPORT_OTHER int OptoPackage_GetIS1(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetIS2(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetIS3(void* pack);
extern "C" OMD_EXPORT_OTHER int OptoPackage_GetIS4(void* pack);

extern "C" OMD_EXPORT_OTHER int OptoPackage_GetTEMP(void* pack);

extern "C" OMD_EXPORT_OTHER int OptoPackage_GetVS(void* pack);
extern "C" OMD_EXPORT_OTHER SensorConfig OptoPackage_GetCONFIG(void* pack);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetVS(void* pack, int value);
extern "C" OMD_EXPORT_OTHER void OptoPackage_SetCONFIG(void* pack, SensorConfig value);

extern "C" OMD_EXPORT_OTHER bool callIsCorrect(void* pack);
extern "C" OMD_EXPORT_OTHER void callSetZero(void* pack);
extern "C" OMD_EXPORT_OTHER int callGetVersion(void* pack);
extern "C" OMD_EXPORT_OTHER void callSetVersion(void* pack, int version);
extern "C" OMD_EXPORT_OTHER bool callIsRaw(void* pack);
extern "C" OMD_EXPORT_OTHER void callSetInvariant(void* pack);
extern "C" OMD_EXPORT_OTHER void callSaveInconsistent(void* pack);
extern "C" OMD_EXPORT_OTHER void callSaveAsInconsistent(void* pack);*/

#endif // OMD__PACKAGE_H
