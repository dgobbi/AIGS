/*
  Program:   Logitech 3D Mouse C Interface Library
  Module:    $RCSfile: logitech3d.h,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C
  Author:    $Author: dgobbi $
  Date:      $Date: 2002/11/04 02:09:40 $
  Version:   $Revision: 1.1 $
*/
/*
  Copyright 2002 Atamai Inc.
  All rights reserved
*/

#ifndef LOGITECH3D_H
#define LOGITECH3D_H 1

#ifdef __cplusplus
extern "C" {
#endif

struct logitech3d;
typedef struct logitech3d logitech3d;

/*---------------------
  any-mode commands  */
								      
/* allocation/destruction */
logitech3d *ltNew();            /* create & initialize */
void ltDelete(logitech3d *lt);  /* delete */

/* open/close communication, the baud rate should be 19200 if the
   mouse is in 3D mode (the default) or 1200 if the mouse is in 2D mode */
/* the dimension should be LT_3D or LT_2D */
/* the sync_mode should be LT_NOTHREAD or LT_THREAD */
int ltOpen(logitech3d *lt, int port, int dimension, int sync_mode);
void ltClose(logitech3d *lt);

#define LT_2D_MODE  2
#define LT_3D_MODE  3

#define LT_NOTHREAD 0  /* synchronous mode, no multithreading */
#define LT_THREAD   1  /* spawn a streaming thread, improves performance */

/* perform hardware reset into LT_2D or LT_3D mode */
void ltHardReset(logitech3d *lt, int dimension);

/* error reporting */
int ltGetError(logitech3d *lt); /* get error code (zero if no error) */
char *ltGetErrorMessage(logitech3d *lt);  /* get text error message */
void ltSetErrorCallback(logitech3d *lt, void (*callback)(void *data),
                        void *data);

#define LT_ERROR_MODE          1 /* bad threading mode */           
#define LT_ERROR_OPEN          2 /* error opening serial port */
#define LT_ERROR_NO_DEVICE     3 /* no device on serial port */
#define LT_ERROR_RESOURCES     4 /* couldn't start thread */
#define LT_ERROR_BAUD          5 /* incorrect baud rate */
#define LT_ERROR_COM           6 /* comm port setup error */
#define LT_ERROR_READ          7 /* I/O error */
#define LT_ERROR_WRITE         8 /* I/O error */
#define LT_ERROR_TIMEOUT       9 /* timeout exceeded */
#define LT_ERROR_WRITE_TIMEOUT 10 /* timeout exceeded */
#define LT_ERROR_PHASE        11 /* bad data record */
#define LT_ERROR_DIAGNOSTICS  12 /* device failed diagnostic check */

/*---------------------
  3D mode commands   */

/* perform software reset, leaves the mouse in 3D mode */
void ltSoftReset(logitech3d *lt);   

/* set data reporting mode */ 
void ltDemandMode(logitech3d *lt);
void ltIncrementalMode(logitech3d *lt);
void ltStreamMode(logitech3d *lt);

/* in demand mode only, request a report */ 
void ltDemand(logitech3d *lt);

/* get a record, specify the number of millesecs to wait */
/* a return value of 1 means that a new record was received */
/* a timeout period of -1 will wait forever, if necessary */   
int ltReport(logitech3d *lt, int timeout);

/* after ltReport, use these to get info from the record */
void ltGetPosition(logitech3d *lt, int xyz[3]); /* *0.001 to get inches  */
void ltGetRotation(logitech3d *lt, int pyr[3]); /* *0.025 to get degrees */
void ltGetStatus(logitech3d *lt, int *status); /* status, see bit defs below */
void ltGetTime(logitech3d *lt, int time[2]); /* get timestamp, secs & msecs */ 

#define LT3D_STATUS_RIGHT     0x01
#define LT3D_STATUS_MIDDLE    0x02
#define LT3D_STATUS_LEFT      0x04
#define LT3D_STATUS_SUSPEND   0x08
#define LT3D_STATUS_STAND     0x10
#define LT3D_STATUS_OUT       0x20
#define LT3D_STATUS_FRINGE    0x40

/* diagnostic information, see bit defs below */
int ltGetDiagnostics(logitech3d *lt);

#define LT3D_DIAG_CONTROLLER  0x0001
#define LT3D_DIAG_PROCESSOR   0x0002
#define LT3D_DIAG_CHECKSUM    0x0004
#define LT3D_DIAG_RAM         0x0008
#define LT3D_DIAG_TRANSMITTER 0x0010
#define LT3D_DIAG_RECEIVER    0x0020
#define LT3D_DIAG_SERIAL      0x0100
#define LT3D_DIAG_EPROM       0x0200
#define LT3D_DIAG_ALL         0xbf3f

/* enable or disable the transmitter */
void ltEnableAudio(logitech3d *lt);
void ltDisableAudio(logitech3d *lt);

/* get loads of status information, see manual */
char *ltGetInformation(logitech3d *lt);

/*---------------------
  2D mode commands   */

/* set mode */
void ltMPlusMode(logitech3d *lt);
void ltMicrosoftMode(logitech3d *lt);

/* get copyright from device */
char *ltGetCopyright(logitech3d *lt);

/* get standard information */
char *ltGetConfiguration(logitech3d *lt);

/* get device type, see defs below */
int ltGetDeviceType(logitech3d *lt);

#define LT2D_TYPE_UNKNOWN   0x80
#define LT2D_TYPE_WIRELESS  0x81
#define LT2D_TYPE_3DMOUSE   0x82

/*---------------------
  low-level commands */

/* these command set error codes */
void ltSendCommand(logitech3d *lt, const char *command);
void ltReceiveReply(logitech3d *lt, char *reply, int length);
void ltSetBaudRate(logitech3d *lt, int rate);

/* these commands return zero if successful, or an error code if not */
int ltSendRaw(logitech3d *lt, const char *data, int length);
int ltReceiveRaw(logitech3d *lt, char *result, int length);

#ifdef __cplusplus
}
#endif

#endif /* LOGITECH3D_H */
