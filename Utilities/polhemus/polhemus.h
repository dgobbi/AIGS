/*=======================================================================

  Program:   Polhemus Fastrack C Interface Library
  Module:    $RCSfile: polhemus.h,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C
  Author:    $Author: dgobbi $
  Date:      $Date: 2005/06/27 13:47:41 $
  Version:   $Revision: 1.1 $

==========================================================================
Copyright 2005 Atamai, Inc.

Redistribution of this source code and/or any binary applications created
using this source code is prohibited without the expressed, written
permission of the copyright holders.  

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
==========================================================================*/

/** \file polhemus.h */

#ifndef POLHEMUS_H
#define POLHEMUS_H 1

#ifdef __cplusplus
extern "C" {
#endif

/** \name Fastrack Commands

Constants for the commands that are understood by the polhemus.
*/
/*\{*/
#define PH_ALIGNMENT           'A'
#define PH_BORESIGHT           'B'
#define PH_UNBORESIGHT         'b'
#define PH_CONTINUOUS          'C'
#define PH_NOCONTINUOUS        'c'
#define PH_METALCOMP           'D'
#define PH_NOMETALCOMP         'd'
#define PH_STYLUS              'e'
#define PH_ENABLE_ASCII        'F'
#define PH_ENABLE_BINARY       'f'
#define PH_BORESIGHT_ANGLES    'G'
#define PH_HEMISPHERE          'H'
#define PH_INCREMENT           'I'
#define PH_STATION_STATE       'l'
#define PH_TIP_OFFSETS         'N'
#define PH_OUTPUT_DATA_LIST    'O'
#define PH_SET_OUTPUT_PORT     'o'
#define PH_SINGLE              'P'
#define PH_ANGULAR_ENVELOPE    'Q'
#define PH_RESET_REFERENCE     'R'
#define PH_TRANSMITTER_FRAME   'r'
#define PH_SYSTEM_STATUS       'S'
#define PH_TEST                'T'
#define PH_IMPERIAL_UNITS      'U'
#define PH_METRIC_UNITS        'u'
#define PH_POSITION_ENVELOPE   'V'
#define PH_ATTITUTE_FILTER     'v'
#define PH_RESET_TO_DEFAULTS   'W'
#define PH_CONFIGURATION_DATA  'X'
#define PH_POSITION_FILTER     'x'
#define PH_SYNCH_MODE          'y'
#define PH_SAVE_CONFIGURATION  '\x0b' /* '^K' */
#define PH_XON                 '\x11' /* '^Q' */
#define PH_XOFF                '\x13' /* '^S' */
#define PH_REINITIALIZE_SYSTEM '\x19' /* '^Y' */
/*\}*/

/** \name Error Codes

The following error codes are returned by fbGetError():
*/
/*\{*/
#define PH_OPEN_ERROR     1     /* error opening serial port */
#define PH_COM_ERROR      2     /* error setting COM port parameters */
#define PH_IO_ERROR       3     /* some sort of I/O error */
#define PH_TIMEOUT_ERROR  4     /* communications timeout error */
#define PH_PARM_ERROR     5     /* bad examine/change parameter */
#define PH_COMMAND_ERROR  6     /* unrecognized bird command */
#define PH_ILLEGAL_ERROR  7     /* action is illegal in this state */ 
#define PH_PHASE_ERROR    8     /* phase error: comm port is dropping bytes */
#define PH_RESOURCE_ERROR 9     /* out of system resources */
#define PH_MODE_ERROR     10    /* specified bad mode when opening polhemus */
/*\}*/

/* miscellaneous */
#define PI             3.14159265359
#define TIMEOUT_PERIOD 5000     /* timeout period in milliseconds */

/** \name Baud Rates

The following baud rates are supported by fbOpen():
*/
/*\{*/
#define PH_1200   12          /* 1200 baud, etc, etc */
#define PH_2400   24
#define PH_4800   48
#define PH_9600   96
#define PH_19200  192
#define PH_38400  384
#define PH_57600  576
#define PH_115200 1152
/*\}*/

/** \name Synchronization Modes

The following synchronization modes are supported by fbOpen().
*/
/*\{*/
#define PH_NOTHREAD 0  /* synchronous mode, no multithreading */ 
#define PH_THREAD   1  /* spawn a streaming thread, improves performance */
#define PH_NONBLOCK 2  /* never block, i.e. allow duplicate readings */
/*\}*/

/** \name Data Reporting Formats

The following are the reporting modes supported by the polhemus.
*/
/*\{*/
#define PH_POSITION    0x0001
#define PH_MOTION      0x0002
#define PH_ANGLES      0x0004
#define PH_QUATERNION  0x0008
#define PH_XDIRCOS     0x0010
#define PH_YDIRCOS     0x0020
#define PH_ZDIRCOS     0x0040
#define PH_BUTTON      0x0080
#define PH_EXTENDED    0x1000
#define PH_BINARY16    0x2000
/*\}*/

/* the polhemus structure: NEVER modify any parameters directly */

struct polhemus;
typedef struct polhemus polhemus;
                                                                      
/* get a serial port device name, given an integer starting at zero */

char *phDeviceName(int i);

/* allocation/destruction of the polhemus structure */

polhemus *phNew();            /* create & initialize polhemus structure */
void phDelete(polhemus *ph);  /* delete polhemus */

/* set initial serial communications parameters */

int phSetInitialComm(polhemus *ph, int rate, int parity, int bits,
                     int handshake);

/* set whether threading is to be used */

int phSetThreadMode(polhemus *ph, int mode);

/* open/close communication with the polhemus */

int phOpen(polhemus *ph, const char *device);
void phClose(polhemus *ph);

/* do a hardware reset of the polhemus */ 

void phReset(polhemus *ph);

/* hemisphere control */

void phSetHemisphere(polhemus *ph, int station, double vx,
		     double vy, double vz);

/* set whether binary or ascii is used for data records */

void phSetBinary(polhemus *ph); /* use binary data records */
void phSetAscii(polhemus *ph);  /* use ascii data records */

/* set the data format for the records */

void phSetReplyFormat(polhemus *ph, int station, int format);

/* set whether the button can be used to send records */

void phSetButtonMode(polhemus *ph, int station, int mode);

/* standard data request methods */

void phPoint(polhemus *ph);              /* request only one data record */
void phStream(polhemus *ph);             /* start streaming acquisition */
void phEndStream(polhemus *ph);          /* end streaming acquisition */

/* standard data acquisition methods, use in and out of streaming mode */
/* phGetTime() returns 'unix' time, i.e. seconds since new year's 1970 */

int phUpdate(polhemus *ph);             /* get next data record */
void phGetPosition(polhemus *ph, float xyz[3]);
void phGetMotion(polhemus *ph, float xyz[3]);
void phGetAngles(polhemus *ph, float zyx[3]);
void phGetQuaternion(polhemus *ph, float q[4]);
void phGetXDirCos(polhemus *ph, float a[3]);
void phGetYDirCos(polhemus *ph, float a[3]);
void phGetZDirCos(polhemus *ph, float a[3]);
int phGetButton(polhemus *ph);           /* true if button pressed */
int phGetStation(polhemus *ph);          /* which station data is from */
double phGetTime(polhemus *ph);          /* when the data was acquired */

/* data morph methods: convert data from one form to another */

void phMatrixFromAngles(float a[9], const float zyx[3]);
void phAnglesFromMatrix(float zyx[3], const float a[9]);

/* low-level methods for sending data to the polhemus */

void phSendCommand(polhemus *ph, const char *command);
void phReceiveReply(polhemus *ph, char *reply, int maxlen);

/* error reporting */
/* if the error is PH_TIMEOUT_ERROR, the bird itself probably has an error
   which you can retrieve with phExamineValue(fb,PH_ERROR_CODE) */

int phGetError(polhemus *ph);          /* get error code (zero if no error) */
char *phGetErrorMessage(polhemus *ph); /* get text error message */
void phSetErrorCallback(polhemus *ph, void (*callback)(void *data), 
                        void *data);

/* utility functions for building commands */

void phAddCommandChar(char **cpp, int cval);
void phAddCommandStation(char **cpp, int station);
void phAddCommandParameterInt(char **cpp, int ival);
void phAddCommandParameterFloat(char **cpp, double fval);

/* utility functions for decoding replies */

float phAsciiToFloat(char **cpp); /* convert ascii to float */
float phAsciiExToFloat(char **cpp); /* convert ascii scientific to float */
float phBinaryToFloat(char **cpp); /* little endian float to native float */
int phBinary16ToInt(char **cpp); /* convert 16-bit binary to int */

/* ultra low-level interface, for use only for diagnostics  */

void phSendRaw(polhemus *ph, const char *text, int len);
void phReceiveRaw(polhemus *ph, char *reply, int len, int thread);


#ifdef __cplusplus
}
#endif

#endif /* POLHEMUS_H */
