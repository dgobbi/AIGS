/*=======================================================================

  Program:   Polhemus Fastrack C Interface Library
  Module:    $RCSfile: polhemus.c,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C
  Author:    $Author: kwang $
  Date:      $Date: 2005/12/06 16:30:12 $
  Version:   $Revision: 1.6 $

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

#include <time.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#include <winbase.h>
#include <sys/timeb.h>
#ifdef _MT
#include <process.h>
#define POLHEMUS_USE_THREADS 1
#endif /* _MT */

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#if defined(__APPLE__)
#include <dirent.h>
#endif
#include <errno.h>
#ifdef _POSIX_THREADS
#include <pthread.h>
#define POLHEMUS_USE_THREADS 1
#endif /* _POSIX_THREADS */
#endif /* unix */

#include "polhemus.h"

/* A short note on multithreading:

   Multithreading is enabled when POLHEMUS_USE_THREADS is defined, which
   happens when
   1) the _MT flag is set under Windows (i.e. when the /MT flag is given
      to CL.EXE or when multithreaded compilation is selected in
      Visual Studio)
   2) the _POSIX_THREADS flag is set under UNIX, i.e. if the operating
      system supports POSIX threads

   If threading is enabled and phOpen() is called with PH_THREAD or
   PH_NONBLOCK, then a thread will be spawned that listens for data
   records sent from the Polhemus.

   This thread continually checks for new data records from the polhemus,
   which come at a rate of 120Hz in the Polhemus's "Stream" mode.  Without
   the use of this thread to catch the records, the serial port buffer
   might overflow during Stream mode.
   
   If multithreading is not available on a particular operating system,
   then "Point" mode should be used instead of "Stream" mode.
*/


/* the polhemus structure: never meant to be used anywhere but in polhemus.c */

struct polhemus {
  /* stuff that needs to be set before the polhemus is opened */
  char device_name[256]; /* serial port device name */
  int baud_rate;         /* baud rate */
  int parity;            /* parity */
  int data_bits;         /* data bits */
  int handshake;         /* hardware handshaking */
  int asynchronous;      /* asynchronous mode */

  /* architecture dependent stuff */
#if defined(_WIN32) || defined(WIN32)
  HANDLE file;           /* windows file handle */
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  int file;              /* unix file handle */
#endif

  /* multithreading stuff */
#ifdef POLHEMUS_USE_THREADS
#if defined(_WIN32) || defined(WIN32)
  HANDLE file_mutex;        /* mutex lock on file handle */
  HANDLE data_event;        /* event is triggered when new data arrives */
  HANDLE data_mutex;        /* lock on data_buffer */
  HANDLE stream_thread;  /* tracking thread for asynchronous mode */
  HANDLE stream_mutex;   /* use to pause tracking thread */
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS
  pthread_mutex_t file_mutex;      /* mutex lock on file handle */
  pthread_cond_t  fresh_data_cond; /* condition to wait on for new data */
  pthread_mutex_t fresh_data_mutex;/* mutex lock on condition           */
  pthread_mutex_t data_mutex;      /* lock on data_buffer */
  pthread_t stream_thread;      /* tracking thread for asynchronous mode */
  pthread_mutex_t stream_mutex; /* use to pause tracking thread */
#endif
#endif /* unix */
  int async_station;           /* station counter for thread */
  int async_error;             /* error in asyncronous thread */
  char async_buffer[4][256]; /* buffer for use by the extra thread  */
  long async_timestamp_secs;   /* timestamp -- secs since 1970 */
  long async_timestamp_msecs;  /* timestamp -- millisecs */  
  int async_data_rate;            /* data rate (Hz) in asynchronous mode */
#endif /* POLHEMUS_USE_THREADS */

  /* error reporting */
  int error;             /* stores last error */
  char error_text[256];  /* stores text for last error */
  void (*error_handler)(void *); /* error callback function */
  void *error_handler_data;   /* error callback data */

  /* miscellaneous state information */
  int revision;            /* revision number (quite important) */
  int stream;              /* true if currently stream mode */
  int point;               /* true if currently point mode*/
  int station;             /* the station which this data set is for */
  int centimeters;         /* position is in centimetres, not inches */
  int binary;              /* true for binary mode */

  /* state information for each station */
  int station_active[4];   /* which stations are active? */
  int button_function[4];  /* button function for each station */
  int format_len[4];       /* format length for each station */
  unsigned char format[4][256];    /* data reply format for each station */

  /* storage of data etc. */
  char data_buffer[256]; /* place where data is stored after being read */
  int phase_leftovers;   /* leftover chars after a phase error */
  int fresh_data;        /* is the data still fresh? */
  long timestamp_secs;   /* timestamp -- secs since 1970 */
  long timestamp_msecs;  /* timestamp -- millisecs */  
};

/* number of bytes in each data element*/
static int record_len_table[256] = { 1, /* space */
                                     2, /* CR/LF */
                                     12, /* position */
                                     12, /* relative position */
                                     12, /* angles */
                                     12, /* x dircos */
                                     12, /* y dircos */
                                     12, /* z dircos */
                                     12, 12, 12,
                                     16, /* quaternion */
                                     4, 12, 12, 12,
                                     2,  /* stylus switch */
                                     0,
                                     6,  /* 16-bit position */
                                     6,  /* 16-bit angles */
                                     8,  /* 16-bit quaternion */
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,0,
                                     0,0,0,0,0,0,0,0,0,
                                     1, /* space */
                                     2, /* CR/LF */
                                     39, /* position */
                                     39, /* relative position */
                                     39, /* angles */
                                     39, /* x dircos */
                                     39, /* y dircos */
                                     39, /* z dircos */
                                     39, 39, 39,
                                     52, /* quaternion */
                                     13, 39, 39, 39,
                                     9,  /* stylus switch */
};

/* prototypes for static internal-use-only functions */

static int set_error(polhemus *ph, int error_code, const char *text);
static int convert_baud_rate(int baud);
static int set_comm_parameters(polhemus *ph);
static void set_timestamp(long *sec, long *msec);
static int record_item_length(polhemus *ph, int format);
static int record_length(polhemus *ph, int station);
static int record_offset(polhemus *ph, int station, unsigned char *try_format,
			 int try_len, int *found_format);
static int binary16_offset(polhemus *ph, int station);
static void read_format_list(const char *cp, unsigned char *dlist, int *len);

#ifdef POLHEMUS_USE_THREADS
#if defined(_WIN32) || defined(WIN32)
static void stream_thread(void *user_data);
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
static void *stream_thread(void *user_data);
#endif
static int start_stream_thread(polhemus *ph);
static void end_stream_thread(polhemus *ph);
#endif 

/*---------------------------------------------------------------------*/
/** \defgroup SetupMethods Connecting to the Polhemus

The polhemus must be connected to one of the serial ports on the
computer.  The device name for the serial ports are different for
different operating systems, the phDeviceName() function can be used
to generate an appropriate device name for the host OS.

The \p polhemus data structure stores all of the information that is needed
by the host computer in order to maintain communication with the polhemus.
This includes information about the polhemus status as well as copies of the
most recent data records from the polhemus.  The contents of this structure
cannot be accessed directly.
*/

/** \fn      char *phDeviceName(int i)
    \ingroup SetupMethods

This function returns a serial port device name that is appropriate
for whatever operating system this code was compiled on.

\param  i    an integer between 0 and 3
\return      a serial port device name, or NULL if \em i is out of range
*/

char *phDeviceName(int i)
{
#if defined(_WIN32) || defined(WIN32)
  static char *dev_names[] = { "COM1:", "COM2:", "COM3:", "COM4:", NULL };
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#if defined(linux)
  static char *dev_names[] = { "/dev/ttyS0", "/dev/ttyS1",
                               "/dev/ttyUSB0", "/dev/ttyUSB1", NULL }; 
#elif defined(__APPLE__)
   static char *dev_names[] = { NULL, NULL, NULL, NULL, NULL };
#elif defined(sgi)
  static char *dev_names[] = { "/dev/ttyd1", "/dev/ttyd2",
                               "/dev/ttyd3", "/dev/ttyd4", NULL }; 
#else
  static char *dev_names[] = { "/dev/ttya", "/dev/ttyb", NULL, NULL, NULL }; 
#endif /* sgi */
#endif /* unix */

  int j;

#if defined(__APPLE__)
  static char devicenames[4][255+6];
  DIR *dirp;
  struct dirent *ep;
  
  dirp = opendir("/dev/");
  if (dirp == NULL) {
    return NULL;
  }

  j = 0;
  while ((ep = readdir(dirp)) != NULL && j < 4) {
    if (ep->d_name[0] == 'c' && ep->d_name[1] == 'u' &&
        ep->d_name[2] == '.')
    {
      strncpy(devicenames[j],"/dev/",5);
      strncpy(devicenames[j]+5,ep->d_name,255);
      devicenames[j][255+5] == '\0';
      dev_names[j] = devicenames[j];
      j++;
    }
  }

  while (j < 4) {
    dev_names[j] = NULL;
    j++;
  }

  closedir(dirp);
#endif /* __APPLE__ */

  /* guard against negative values */
  if (i < 0) {
    return NULL;
  }

  /* guard against values greater than the dev_names array length */
  for (j = 0; j < i; j++) {
    if (dev_names[j] == NULL) {
      return NULL;
    }
  }

  return dev_names[i];
}

/** \fn      polhemus *phNew() 
    \ingroup SetupMethods

Allocate an initialized polhemus structure that can be used to
communicating with the polhemus.

\return   pointer to a new polhemus structure
*/

polhemus *phNew() 
{
  polhemus *ph;
  int i;

  ph = (polhemus *)malloc(sizeof(polhemus));
  ph->device_name[0] = '\0';
  ph->device_name[255] = '\0';
  ph->baud_rate = PH_9600;
  ph->parity = 'N';
  ph->data_bits = 8;
  ph->handshake = 0;
  ph->asynchronous = PH_NOTHREAD;

#if defined(_WIN32) || defined(WIN32)
  ph->file = INVALID_HANDLE_VALUE;
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  ph->file = -1;
#endif

  ph->error = 0;
  ph->error_text[0] = '\0';
  ph->error_handler = NULL;
  ph->error_handler_data = NULL;

  ph->revision = 0;
  ph->stream = 0;
  ph->point = 0;
  ph->station = 0;  /* start station numbering at zero */
  ph->centimeters = 0;
  ph->binary = 0;

  for (i = 0; i < 4; i++) {
    ph->button_function[i] = 0;
    ph->station_active[i] = 0;
    ph->format_len[i] = 3;
    ph->format[i][0] = 2; /* x,y,z */
    ph->format[i][1] = 4; /* azimuth, elevation, roll */
    ph->format[i][2] = 1; /* <CR><LF> */
  }

  memset(ph->data_buffer,0,256);
  ph->phase_leftovers = 0;
  ph->fresh_data = 1;
 
#ifdef POLHEMUS_USE_THREADS
  ph->async_station = 0;
  ph->async_error = 0;
  ph->async_data_rate = 0;
  ph->fresh_data = 0;
#endif

  return ph;
}

/** \fn      void phDelete(polhemus *ph)
    \ingroup SetupMethods

Release all computer resources associated with a polhemus structure.
This will terminate communication with the polhemus before the polhemus
structure is deallocated.

\param ph    pointer to a polhemus structure created with phNew()
*/

void phDelete(polhemus *ph)
{
#if defined(_WIN32) || defined(WIN32)
  if (ph->file != INVALID_HANDLE_VALUE) {
    phClose(ph);
  }
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  if (ph->file != -1) {
    phClose(ph);
  }
#endif
  free(ph);
}

/** \fn      int phSetInitialComm(polhemus *ph, int baud, int parity,
                                  int bits, int handshake) 
    \ingroup SetupMethods

Set the serial communication parameters to use when the computer first
attempts to communicate with the device, so that the computers
communication parameters match those set via the DIP switches on the
device.

\param ph      pointer to a polhemus structure
\param baud    the baud rate for the polhemus, either as the actual
               desired baud rate or as one of the following constants:
               PH_1200, PH_2400, PH_4800, PH_9600, PH_19200, PH_38400,
               PH_57600, PH_115200 (the factory default is 9600)
\param parity  'N', 'O', or 'E' for No parity, Odd parity, or Even parity
\param bits    7 or 8 data bits
\param handshake set to zero for no hardware handshaking (current models
               of the polhemus do not support hardware handshaking)

\return   zero if successful, or an error code if not
          (use phGetErrorMessage() to get a text error description)

The recommended DIP switch setting for the device are (9600,'N',8,0),
which is what phOpen() will be used by default if you do not call
phSetInitialComm().

If you want to use a higher baud rate than 9600, it is recommended
that you set the baud rate with the 'o' command rather than by
setting the dip switches.
*/

int phSetInitialComm(polhemus *ph, int baud, int parity, int bits,
                    int handshake) 
{
  ph->baud_rate = baud;
  ph->parity = parity;
  ph->data_bits = bits;
  ph->handshake = handshake;

  return 0;
}

/** \fn      int phSetThreadMode(polhemus *ph, int mode) 
    \ingroup SetupMethods

Use this method in order to multi-thread the communication with the
device.

\param mode    the threading mode, which should be one of:
               - PH_NOTHREAD: disable multithreading
               - PH_THREAD: enable threading
               - PH_NONBLOCK: enable threading and allow duplicate records

\return        zero if phSetThreadMode() was successful, or an error code
               otherwise (use phGetErrorMessage() to get a text error
               description)

The threading mode used is critical to the performance of communication
with the polhemus.  The polhemus is usually used in stream mode, which means
that it is continuously sending data record to the computer at a rate
of 120Hz.  The threading mode determines how the computer manages to
catch these data records and transfer the information to your application.

In PH_NOTHREAD mode, your application is responsibe for calling phUpdate()
at least 120 times per second during stream mode.  If the application is
not able to do so, then the serial port buffer will fill up and data
records will become stale or might even be corrupted.  It is, however,
safe to use PH_NOTHREAD mode if only point mode data collection is used.

In PH_THREAD mode, a separate thread is spawned that listens to the
serial port while your application runs.  Each time your application calls
phUpdate() the most recent data record received from the polhemus is returned.
If a new data record has not been recieved from the polhemus since the last
time phUpdate() was called, then phUpdate() will not return until a
fresh data record is transmitted from the polhemus.

The PH_NONBLOCK mode is almost identical to PH_THREAD mode, except that
a call to phUpdate() will always return immediately even if no new
information has been recieved since the last call to phUpdate().  This
means that a phUpdate() call might return exactly the same information as
the previous phUpdate().
*/

int phSetThreadMode(polhemus *ph, int mode) 
{
  ph->asynchronous = mode;
#ifndef _MT
  if (mode != PH_NOTHREAD) {
    return set_error(ph,PH_MODE_ERROR,"bad mode: threading not supported");
  }
#endif

  return 0;
}

/*
int convert_baud_rate(polhemus *ph, int rate)

Convert the baud rate into an operating-system specific form:
the baud rate must match the rate specified
by the bird's dip switches.

Valid values are PH_1200, PH_2400, PH_9600, PH_14400, PH_19200, PH_38400, 
PH_57600 and PH_115200. */

static int convert_baud_rate(int rate)
{
#if defined(_WIN32) || defined(WIN32)
  static int equiv[] = { CBR_1200,
                         CBR_2400,
                         CBR_4800,
                         CBR_9600,
                         CBR_19200,
                         CBR_38400,
                         CBR_57600,
                         CBR_115200 };
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#if defined(sgi) && defined(__NEW_MAX_BAUD)
  static int equiv[] = { 1200,
                         2400,
                         4800,
                         9600,
                         19200,
                         38400,
                         57600,
                         115200 };
#elif defined(B115200)
  static int equiv[] = { B1200,
                         B2400,
                         B4800,
                         B9600,
                         B19200,
                         B38400,
                         B57600,
                         B115200 };
#else
  static int equiv[] = { B1200,
                         B2400,
                         B4800,
                         B9600, 
                         B19200, 
                         B38400,
                         -1,
                         -1 };
#endif /* sgi __NEW_MAX_BAUD */
#endif /* unix */

  int idx = -1;
  switch (rate)
    {
    case 1200:
    case PH_1200:
      idx = 0;
      break;
    case 2400:
    case PH_2400:
      idx = 1;
      break;
    case 4800:
    case PH_4800:
      idx = 2;
      break;
    case 9600:
    case PH_9600:
      idx = 3;
      break;
    case 19200:
    case PH_19200:
      idx = 4;
      break;
    case 38400:
    case PH_38400:
      idx = 5;
      break;
    case 57600:
    case PH_57600:
      idx = 6;
      break;
    case 115200:
    case PH_115200:
      idx = 7;
      break;
    }
  if (idx < 0) {
    return -1;
  }
  return equiv[idx];
}

/*
int set_comm_parameters(polhemus *ph, int baud, int parity, int handshake)

Set the comm port to the specified communication parameters
*/
static int set_comm_parameters(polhemus *ph)
{
#if defined(_WIN32) || defined(WIN32)   /* start of WIN32 portion of code - */
  static COMMTIMEOUTS default_ctmo = { 0, 2, /* return every char */
                                       TIMEOUT_PERIOD, 
                                       2, 
                                       TIMEOUT_PERIOD };
  DCB comm_settings;

  if (GetCommState(ph->file,&comm_settings) == FALSE) {
    return -1;
  }

  comm_settings.fOutX = FALSE;             /* no S/W handshake on output */
  comm_settings.fInX = TRUE;               /* use XON/XOFF on input */
  comm_settings.XonChar = '\x11';
  comm_settings.XoffChar = '\x13';

  comm_settings.fAbortOnError = TRUE;      /* must clear errors */

  comm_settings.fOutxDsrFlow = TRUE;       /* allow modem-style handshaking*/
  comm_settings.fDtrControl = DTR_CONTROL_HANDSHAKE;  

  if (ph->handshake)
    {
    comm_settings.fOutxCtsFlow = TRUE;      /* use RTS/CTS handshake */
    comm_settings.fRtsControl = RTS_CONTROL_HANDSHAKE;
    }
  else
    {
    comm_settings.fOutxCtsFlow = FALSE;      /* no RTS/CTS handshake */
    comm_settings.fRtsControl = RTS_CONTROL_DISABLE;
    }

  comm_settings.ByteSize = ph->data_bits;
  if (ph->parity == 'N') {                 /* set parity */
    comm_settings.Parity = NOPARITY;
  }
  else if (ph->parity == 'O') {
    comm_settings.Parity = ODDPARITY;
  }
  else if (ph->parity == 'E') {
    comm_settings.Parity = EVENPARITY;
  }
  comm_settings.StopBits = ONESTOPBIT;

  comm_settings.BaudRate = convert_baud_rate(ph->baud_rate);  /* speed */

  if (SetCommState(ph->file,&comm_settings) == FALSE) {
    return -1;
  }

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
/* start of UNIX portion of code -------------------*/

  struct termios t;
  int baud;
  baud = convert_baud_rate(ph->baud_rate);

  if (tcgetattr(ph->file,&t) == -1) { /* get I/O information */
    return -1;
  }

  /* 9600 baud, 8 data bits, no parity, 1 stop bit, enable read */
#if defined(sgi) && defined (__NEW_MAX_BAUD)
  if (ph->data_bits == 8) {
    t.c_cflag = CS8 | CREAD | CLOCAL;
    }
  else {
    t.c_cflag = CS7 | CREAD | CLOCAL;
    }
  t.c_ospeed = baud;
#elif defined(__APPLE__)
  if (ph->data_bits == 8) {
    t.c_cflag = CS8 | CREAD | CLOCAL;
    }
  else {
    t.c_cflag = CS7 | CREAD | CLOCAL;
    }
  cfsetispeed(&t, baud);
  cfsetospeed(&t, baud);
#else
  if (ph->data_bits == 8) {
    t.c_cflag = baud | CS8 | CREAD | CLOCAL;
    }
  else {
    t.c_cflag = baud | CS7 | CREAD | CLOCAL;
    }
#endif
  if (ph->handshake) {
#ifdef sgi
    t.c_cflag |= CNEW_RTSCTS;       /* enable hardware handshake */
#else
    t.c_cflag |= CRTSCTS;           
#endif
  }
  else {
#ifdef sgi
    t.c_cflag &= ~CNEW_RTSCTS;       /* turn off hardware handshake */
#else
    t.c_cflag &= ~CRTSCTS;
#endif     
  } 

  /* clear everything specific to terminals */
  t.c_lflag = 0;
  t.c_iflag = IXOFF; /* send XOFF when buffer is full */
  t.c_oflag = 0;

  t.c_cc[VMIN] = 0;                    /* use constant, not interval timout */
  t.c_cc[VTIME] = TIMEOUT_PERIOD/100;  /* wait for 1 secs max */

  if (tcsetattr(ph->file,TCSANOW,&t) == -1) { /* set I/O information */
    return -1;
  }
#endif

  return 0;
}

/** \fn      int phOpen(polhemus *ph,
                        const char *device)
    \ingroup SetupMethods

Open communication between the computer and the polhemus.  This will
also probe the polhemus for configuration information.
The serial port is set up to do XON/XOFF handshaking.

\param ph      pointer to a polhemus structure
\param device  a valid serial port device name (see phDeviceName())

\return        zero if phOpen() was successful, or an error code otherwise
               (use phGetErrorMessage() to get a text error description)

If it is unknown which serial port the polhemus is plugged into or what
baud rate the polhemus is set to, then phOpen() can be called repeatedly
with different serial port names and baud rates in order to probe for
the polhemus.
*/

int phOpen(polhemus *ph, const char *device)
{
  int i;

#if defined(_WIN32) || defined(WIN32)
  DCB comm_settings;
  DWORD comm_bits;
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  int term_bits;
#endif

#if defined(_WIN32) || defined(WIN32)   /* start of WIN32 portion of code - */
  static COMMTIMEOUTS default_ctmo = { 0, 2, /* return every char */
                                       TIMEOUT_PERIOD, 
                                       2, 
                                       TIMEOUT_PERIOD };

  if (convert_baud_rate(ph->baud_rate) < 0) {
    return set_error(ph,PH_ILLEGAL_ERROR,"illegal baud rate");
  }

  if (device == NULL || strlen(device) > 255) {
    return set_error(ph,PH_ILLEGAL_ERROR,"illegal device name");
  }

  strncpy(ph->device_name,device,255);

  ph->file = CreateFile(ph->device_name,
                        GENERIC_READ|GENERIC_WRITE,
                        0,  /* not allowed to share ports */
                        0,  /* child-processes don't inherit handle */
                        OPEN_EXISTING, 
                        FILE_ATTRIBUTE_NORMAL,
                        NULL); /* no template file */

  if (ph->file == INVALID_HANDLE_VALUE) {
    return set_error(ph,PH_OPEN_ERROR,"couldn't open serial port");
  }

  if (SetupComm(ph->file,1024,1024) == FALSE ||
      SetCommTimeouts(ph->file,&default_ctmo) == FALSE) {
    CloseHandle(ph->file);
    return set_error(ph,PH_COM_ERROR,"couldn't set serial port parameters");
  }
  if (set_comm_parameters(ph) < 0)
    {
    CloseHandle(ph->file);
    return set_error(ph,PH_COM_ERROR,"couldn't set serial port parameters");
  }

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
/* start of UNIX portion of code -------------------*/

  static struct flock fl = { F_WRLCK, 0, 0, 0 }; /* for file locking */
  static struct flock fu = { F_UNLCK, 0, 0, 0 }; /* for file unlocking */
  int term_bits;

#ifndef _POSIX_THREADS
  if (mode != PH_NOTHREAD) {
    return set_error(ph,PH_MODE_ERROR,"bad mode: threading not supported");
  }
#endif
  if (convert_baud_rate(ph->baud_rate) < 0) {
    return set_error(ph,PH_ILLEGAL_ERROR,"illegal baud rate");
  }

  if (device == NULL || strlen(device) > 255) {
    return set_error(ph,PH_ILLEGAL_ERROR,"illegal device name");
  }

  strcpy(ph->device_name,device);

  /* port is readable/writable and is blocking */
  ph->file = open(ph->device_name,O_RDWR|O_NOCTTY|O_NDELAY);

  if (ph->file == -1) {
    char text[80];
    sprintf(text,"couldn't open serial port %s",ph->device_name);
    return set_error(ph,PH_OPEN_ERROR,text);
  }

  /* restore blocking now that the port is open (we just didn't want */
  /* the port to block while we were trying to open it) */
  fcntl(ph->file, F_SETFL, 0);

  /* get exclusive lock on the serial port */
  /* (on many unices, this has no effect for a device file but try anyway) */
#ifndef __APPLE__
  if (fcntl(ph->file, F_SETLK, &fl) == -1) {
    char text[80];
    sprintf(text,"serial port %s is in use",ph->device_name);
    close(ph->file);
    return set_error(ph,PH_OPEN_ERROR,text);
  }
#endif /* __APPLE__ */

  if (set_comm_parameters(ph) < 0) {
#ifndef __APPLE__
    fcntl(ph->file, F_SETLK, &fu);
#endif /* __APPLE__ */
    close(ph->file);
    return set_error(ph,PH_COM_ERROR,"couldn't set serial port parameters");
    }

#endif /* unix */

  /* start of system-independent portion of code ---------------*/

#if defined(_WIN32) || defined(WIN32)
  GetCommModemStatus(ph->file,&comm_bits);
  if ((comm_bits & MS_RLSD_ON) == 0) {  /* check for carrier signal */
    /* set_error(ph,PH_COM_ERROR,"no carrier"); */
  }

  PurgeComm(ph->file,PURGE_TXCLEAR);
  PurgeComm(ph->file,PURGE_RXCLEAR);

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  ioctl(ph->file, TIOCMGET, &term_bits);
  if ((term_bits & TIOCM_CAR) == 0) { /* check for carrier signal */
    /* set_error(ph,PH_COM_ERROR,"no carrier"); */
    /* fprintf(stderr, "no carrier %x\n", term_bits); */
  }

  tcflush(ph->file,TCIOFLUSH);
#endif /* unix */

#ifdef POLHEMUS_USE_THREADS
  if (ph->asynchronous != PH_NOTHREAD) {
    if (start_stream_thread(ph) == 0) {
      ph->asynchronous = PH_NOTHREAD;
      phClose(ph);
      return set_error(ph,PH_RESOURCE_ERROR,"couldn't open streaming thread");
    }
  }
#endif

  /* ========= code should be added here to get info about device ==== */

  if (ph->error) {
    phClose(ph);
  }
  return ph->error;
}

/** \fn      void phClose(polhemus *ph)
    \ingroup SetupMethods

Shut down the polhemus and close communication.

\param ph      pointer to a polhemus structure
*/

void phClose(polhemus *ph)
{
#if defined(_WIN32) || defined(WIN32)
  DCB comm_settings;
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  static struct polhemus fu = { F_UNLCK, 0, 0, 0 }; /* for file unlocking */
  int term_bits;
#endif

  if (ph->stream) {
    phEndStream(ph);
  }
#ifdef POLHEMUS_USE_THREADS
  if (ph->asynchronous != PH_NOTHREAD) {
    end_stream_thread(ph);
  }
#endif 

#if defined(_WIN32) || defined(WIN32)
  CloseHandle(ph->file);
  ph->file = INVALID_HANDLE_VALUE;

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  /* release our lock on the serial port */
#ifndef __APPLE__
  fcntl(ph->file, F_SETLK, &fu);
#endif /* __APPLE__ */
  close(ph->file);
  ph->file = -1;
#endif
}

/** \fn      void phReset(polhemus *ph)
    \ingroup SetupMethods

Reset the polhemus by sending the ^Y command to the device.  This will
reset the device to its power-on defaults.

\param ph         pointer to a polhemus structure
*/

void phReset(polhemus *ph)
{
  int i;
#if defined(_WIN32) || defined(WIN32)
  DCB comm_settings;
  DWORD comm_bits;
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  int term_bits;
#endif

  if (ph->stream) {
    phEndStream(ph);
  }
  
  /* send the reset command */
  phSendRaw(ph,"\x19\n",2);

#if defined(_WIN32) || defined(WIN32)
  set_comm_parameters(ph);
  Sleep(2000);    /* wait 2 seconds for device to wake up */

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  set_comm_parameters(ph);
  usleep(500000); /* wait for 2 sec for device to wake up */
  usleep(500000);
  usleep(500000);
  usleep(500000);
#endif /* unix */

  ph->error = 0;
  ph->error_text[0] = '\0';

  ph->stream = 0;
  ph->point = 0;
  ph->station = 0;  /* start station numbering at zero */
  ph->centimeters = 0;
  ph->binary = 0;

  for (i = 0; i < 4; i++) {
    ph->button_function[i] = 0;
    ph->format_len[i] = 3;
    ph->format[i][0] = 2; /* x,y,z */
    ph->format[i][1] = 4; /* azimuth, elevation, roll */
    ph->format[i][2] = 1; /* <CR><LF> */
  }

  memset(ph->data_buffer,0,256);
  ph->phase_leftovers = 0;
  ph->fresh_data = 1;
 
#ifdef POLHEMUS_USE_THREADS
  ph->async_station = 0;
  ph->async_error = 0;
  ph->async_data_rate = 0;
  ph->fresh_data = 0;
#endif

#if defined(_WIN32) || defined(WIN32)
  GetCommModemStatus(ph->file,&comm_bits);
  if ((comm_bits & MS_RLSD_ON) == 0) {  /* check for carrier signal */
    /* set_error(ph,PH_COM_ERROR,"no carrier"); */
  }

  PurgeComm(ph->file,PURGE_TXCLEAR);
  PurgeComm(ph->file,PURGE_RXCLEAR);

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  ioctl(ph->file, TIOCMGET, &term_bits);
  if ((term_bits & TIOCM_CAR) == 0) { /* check for carrier signal */
    /* set_error(ph,PH_COM_ERROR,"no carrier"); */
  }

  tcflush(ph->file,TCIOFLUSH);
#endif /* unix */
}  

/*---------------------------------------------------------------------*/
/** \defgroup ConfigureMethods Configuring the Data Format

These methods are used to configure the data records that will be sent
from the polhemus.
*/

/** \fn      void phSetBinary(polhemus *ph)
    \ingroup ConfigureMethods

Report data in binary format, which more compact and precise than ascii.
The default format is ascii.

\param ph         pointer to a polhemus structure
*/

void phSetBinary(polhemus *ph)
{
  phSendCommand(ph, "f\n");
}

/** \fn      void phSetAscii(polhemus *ph)
    \ingroup ConfigureMethods

Report data in ascii format.  This is the default.

\param ph         pointer to a polhemus structure
*/

void phSetAscii(polhemus *ph)
{
  phSendCommand(ph, "F\n");
}

/** \fn    void phSetHemisphere(polhemus *ph, int station, 
                                double vx, double vy, double vz)
    \ingroup ConfigureMethods

Set the tracking hemisphere for the polhemus by specifying a vector that
is normal to the plane that divides the two hemispheres.  The polhemus
will only correctly report the positions for receivers within the
hemisphere that the vector is pointing into.

If the special vector (0,0,0) is used, then the polhemus will continually
modify the tracking hemisphere so that the receiver is always correctly
tracked (note that the receiver must be in the correct hemisphere when
this special command is sent).

\param ph         pointer to a polhemus structure
\param station    the station: an integer between 1 and 4
\param vx         x component of the vector
\param vy         y component of the vector
\param vz         z component of the vector
*/

void phSetHemisphere(polhemus *ph, int station,
		     double vx, double vy, double vz)
{
  char command[256];
  char *cp = command;

  phAddCommandChar(&cp, 'H');
  phAddCommandStation(&cp, station);
  phAddCommandParameterFloat(&cp, vx);
  phAddCommandParameterFloat(&cp, vy);
  phAddCommandParameterFloat(&cp, vz);
  phAddCommandChar(&cp, '\r');
  phAddCommandChar(&cp, '\n');

  phSendCommand(ph, command);
}

/** \fn      void phSetReplyFormat(polhemus *ph, int station, int format)
    \ingroup ConfigureMethods

Set the data format that will be used by the polhemus.  This must
be set before data records are requested from the polhemus.

The most common formats are (PH_POSITION | PH_ANGLES) and
(PH_POSITION | PH_QUATERNION), if you want to check whether
the stylus button is pressed then add PH_BUTTON.

There are two special format qualifiers that can be added:
PH_EXTENDED provides ascii reporting at full IEEE float
precision, and PH_BINARY16 uses a compact 16-bit packed
format that provides the data in half the space required for
IEEE floats.

\param ph         pointer to a polhemus structure
\param station    the station: an integer between 1 and 4
\param format     a combination of the following: PH_POSITION, PH_ANGLES,
                  PH_QUATERNION, PH_MOTION, PH_XDIRCOS, PH_YDIRCOS,
                  PH_ZDIRCOS, PH_BUTTON, PH_EXTENDED, PH_BINARY16

The most common data format is PH_POSITION_ANGLES, which is the most
compact format for the full six degrees of freedom.
*/

void phSetReplyFormat(polhemus *ph, int station, int format)
{
  char command[256];
  char *cp = command;

  int extended = 0;

  int space = 0;
  int crlf = 1;
  int position = 2;
  int motion = 3;
  int angles = 4;
  int xdircos = 5;
  int ydircos = 6;
  int zdircos = 7;
  int quaternion = 11;
  int button = 16;  

  if (format & PH_EXTENDED) {
    extended = 50;
  }

  phAddCommandChar(&cp, 'O');
  phAddCommandStation(&cp, station);

  if (format & PH_BINARY16) {
    position = 18;
    angles = 19;
    quaternion = 20;
    
    if (format & PH_POSITION) {
      phAddCommandParameterInt(&cp, position);
    }
    if (format & PH_ANGLES) {
      phAddCommandParameterInt(&cp, angles);
    }
    if (format & PH_QUATERNION) {
      phAddCommandParameterInt(&cp, quaternion);
    }
    if (format & PH_BUTTON) {
      phAddCommandParameterInt(&cp, button);
    }
  }
  else {
    if (format & PH_POSITION) {
      phAddCommandParameterInt(&cp, extended + position);
    }
    if (format & PH_MOTION) {
      phAddCommandParameterInt(&cp, extended + motion);
    }
    if (format & PH_ANGLES) {
      phAddCommandParameterInt(&cp, extended + angles);
    }
    if (format & PH_XDIRCOS) {
      phAddCommandParameterInt(&cp, extended + xdircos);
    }
    if (format & PH_YDIRCOS) {
      phAddCommandParameterInt(&cp, extended + ydircos);
    }
    if (format & PH_ZDIRCOS) {
      phAddCommandParameterInt(&cp, extended + zdircos);
    }
    if (format & PH_QUATERNION) {
      phAddCommandParameterInt(&cp, extended + quaternion);
    }
    if (format & PH_BUTTON) {
      if (!extended) {  /* add space to avoid confusion */
	phAddCommandParameterInt(&cp, space);
      }
      phAddCommandParameterInt(&cp, button);
    }
    phAddCommandParameterInt(&cp, extended + crlf);
  }

  phAddCommandChar(&cp, '\r');
  phAddCommandChar(&cp, '\n');

  phSendCommand(ph, command);
}

/** \fn      void phSetButtonMode(polhemus *ph, int station
                                  int mode)
    \ingroup ConfigureMethods

Enable or disable the reporting of button information from the polhemus.

\param ph         pointer to a polhemus structure
\param station    the station: an integer between 1 and 4
\param mode       0 or 1 depending on whether button information is desired
*/

void phSetButtonMode(polhemus *ph, int station, int mode)
{
  char command[256];
  char *cp = command;

  phAddCommandChar(&cp, 'e');
  phAddCommandStation(&cp, station);
  phAddCommandParameterInt(&cp, mode);
  phAddCommandChar(&cp, '\r');
  phAddCommandChar(&cp, '\n');

  phSendCommand(ph, command);
}

/*---------------------------------------------------------------------*/
/** \defgroup RequestMethods Requesting Data from the Polhemus

The polhemus has two primary methods for sending data records to the 
host computer: stream mode and point mode.

In stream mode, the polhemus sends data records for all active stations
at the preselected data rate.  Stream mode continues indefinitely until
it is interrupted.

In point mode, the polhemus will only send exactly one data record per bird
each time that data is requested from the polhemus.

Also, unless phSetButtonMode() has been used to set the button mode to
zero, pressing the stylus button will cause the following to occur.
In point mode, pressing the button will cause a record to be sent.  In
continuous mode, pressing the button will turn continuous reporting on
and off.
*/

/** \fn      void phStream(polhemus *ph)
    \ingroup RequestMethods

Request the polhemus to begin streaming data records.

\param ph         pointer to a polhemus structure

Once the polhemus is in stream mode, phUpdate() can be used
to prepare a data record for access via phGetPosition() and
the other related functions.

Stream mode should only be used if phOpen() was called with
either the PH_THREAD or PH_NONBLOCK mode.  In PH_NOTHREAD mode,
it is the responsibility of the application to call phUpdate()
often enough to ensure that the serial port buffer does not
fill up.

Stream mode can be turned of by phEndStream().  Note that stream
mode is automatically terminated by any of the following functions:
phPoint().
*/

void phStream(polhemus *ph)
{
  if (ph->stream) {
    return;
  }
  ph->stream = 1;
  ph->station = 0;

  phSendRaw(ph, "C", 1);

#ifdef POLHEMUS_USE_THREADS
  if (ph->asynchronous != PH_NOTHREAD) {
    int i;
    for (i = 0; i < 4; i++) {
       memset(ph->async_buffer[i],'\0',256);
    }
    /* wake-up the asynchronous stream thread */
#if defined(_WIN32) || defined(WIN32)
#ifdef _MT
    ReleaseMutex(ph->stream_mutex);
#endif
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS
    pthread_mutex_unlock(&ph->stream_mutex);
#endif
#endif
  }
#endif
}

/** \fn      void phEndStream(polhemus *ph)
    \ingroup RequestMethods

Terminate streaming mode.

\param ph         pointer to a polhemus structure
*/

void phEndStream(polhemus *ph)
{
  if (!ph->stream) {
    return;
  }

  ph->stream = 0;
  phSendRaw(ph, "c", 1);

#ifdef POLHEMUS_USE_THREADS
  if (ph->asynchronous != PH_NOTHREAD) { 
    /* put the asynchronous stream thread to sleep */ 
#if defined(_WIN32) || defined(WIN32)
#ifdef _MT
    WaitForSingleObject(ph->stream_mutex,INFINITE);
#endif
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS
    pthread_mutex_lock(&ph->stream_mutex);
#endif
#endif
  }
#endif

#if defined(_WIN32) || defined(WIN32)
  PurgeComm(ph->file,PURGE_RXCLEAR);
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  tcflush(ph->file,TCIFLUSH);
#endif
}

/** \fn      void phPoint(polhemus *ph)
    \ingroup RequestMethods

Request a single data record from the polhemus.

\param ph         pointer to a polhemus structure

This requests a single data record from each active polhemus station.
The data records must then be retrieved by a call to phUpdate() for
each one of the active stations.

Using point mode to obtain information from the polhemus is not
as efficient as using stream mode.
*/

void phPoint(polhemus *ph)
{
  if (ph->stream) {
    phEndStream(ph);
  }

  phSendRaw(ph, "P\n", 2);

  if (!ph->error) {
    ph->point = 1;
  }
}

/*---------------------------------------------------------------------*/
/** \defgroup DataMethods Decoding Polhemus Data

After a data record has been sent by the polhemus, the phUpdate() method
can be used to retrieve it.  Data records can be requested from the
polhemus via either the phStream() or phPoint() methods, or by pressing
the button on the stylus.

After phUpdate() has been called, the various phGetXX() methods 
extract various pieces of information from the data record.  The
phGetStation() method should always be used to check which station the
data record is for.  You must call phUpdate() once for each
active station.
*/

/** \fn      int phUpdate(polhemus *ph)
    \ingroup DataMethods

This is the central function in the polhemus interface: it retrieves
a single data record from the polhemus.

\param ph         pointer to a polhemus structure

\return           only for PH_NONBLOCK mode: return value is 0 if no
                  new information has been received from the bird since
                  the last call (in PH_THREAD or PH_NOTHREAD mode the
                  return value is always 1)

If phPoint() is used to request data records from the polhemus, then
every call to phUpdate() must be preceeded by phPoint().

If phStream() is used to put the polhemus into stream mode, then
phUpdate() is used to obtain the most recent data record that was
sent from the polhemus.

In particular, if phOpen() was called with PH_THREAD set, then
phUpdate() will wait for the next data record to be sent from the
polhemus or will return immediately if there is already a data record
waiting.  If phOpen() was called with PH_NONBLOCK, then phUpdate()
will always return immediately and the return value will be 0 if
a new data record has not yet arrived.
*/

int phUpdate(polhemus *ph)
{
  int i;
  int len;

#ifdef POLHEMUS_USE_THREADS
  int fresh = 0;
  int async_error;

  if (ph->stream && ph->asynchronous != PH_NOTHREAD) {

#if defined(_WIN32) || defined(WIN32)
#ifdef _MT

    /* wait until a new data record arrives */
    if (ph->asynchronous != PH_NONBLOCK) {
      if (WaitForSingleObject(ph->data_event,TIMEOUT_PERIOD) == WAIT_TIMEOUT) {
        set_error(ph,PH_TIMEOUT_ERROR,
                  "timeout waiting for asynchronous data event");
        return fresh;
      }
    }

    /* get a lock on the data record */
    WaitForSingleObject(ph->data_mutex,INFINITE);

    memcpy(ph->data_buffer,ph->async_buffer[ph->station],256);
    ph->timestamp_secs = ph->async_timestamp_secs;
    ph->timestamp_msecs = ph->async_timestamp_msecs;
    fresh = ph->fresh_data;
    ph->fresh_data = 0;
    async_error = ph->async_error;
    ph->async_error = 0;

    /* free the lock on the data record */
    ReleaseMutex(ph->data_mutex);

#endif /* _MT */
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS

    /* wait until a new data record arrives */
    if (ph->asynchronous != PH_NONBLOCK) {
      struct timeval tv;
#ifndef PTHREAD_COND_TIMEDWAIT_USES_TIMEVAL
      struct timespec ts;
#endif
      pthread_mutex_lock(&ph->fresh_data_mutex);
      if (!ph->fresh_data) {
        /* all the time stuff is used to check for timeouts */
        gettimeofday(&tv,0);
        tv.tv_sec += TIMEOUT_PERIOD/1000; /* msec to sec */ 
        tv.tv_usec += (TIMEOUT_PERIOD % 1000)*1000; /* msec to usec */
        if (tv.tv_usec >= 1000000) { /* if usec overflow */
          tv.tv_usec -= 1000000;
          tv.tv_sec += 1;
        }
#ifdef PTHREAD_COND_TIMEDWAIT_USES_TIMEVAL
        if (pthread_cond_timedwait(&ph->fresh_data_cond,
                                   &ph->fresh_data_mutex,&tv) 
            == ETIMEDOUT) {
#else   /* convert timeval to timespec */
        ts.tv_sec = tv.tv_sec;
        ts.tv_nsec = tv.tv_usec * 1000; 
        if (pthread_cond_timedwait(&ph->fresh_data_cond,
                                   &ph->fresh_data_mutex,&ts) 
            == ETIMEDOUT) {
#endif
          pthread_mutex_unlock(&ph->fresh_data_mutex);
          set_error(ph,PH_TIMEOUT_ERROR,
                    "timeout waiting for asynchronous data event");
          return fresh;
        }
      }
      pthread_mutex_unlock(&ph->fresh_data_mutex);
    }

    /* get a lock on the data record */
    pthread_mutex_lock(&ph->data_mutex);

    memcpy(ph->data_buffer,ph->async_buffer[ph->station],256);
    ph->timestamp_secs = ph->async_timestamp_secs;
    ph->timestamp_msecs = ph->async_timestamp_msecs;
    fresh = ph->fresh_data;
    ph->fresh_data = 0;
    async_error = ph->async_error;
    ph->async_error = 0;

    /* release the lock on the data record */
    pthread_mutex_unlock(&ph->data_mutex);

#endif /* _POSIX_THREADS */
#endif /* unix */

    /* check for error code set by stream thread */
    if (async_error) {
      if (async_error == PH_TIMEOUT_ERROR) {
        set_error(ph,PH_TIMEOUT_ERROR,"timeout while waiting for data");
      }
      else if (async_error == PH_IO_ERROR) {
        set_error(ph,PH_IO_ERROR,"I/O error on serial port read");
      } 
      else if (async_error == PH_PHASE_ERROR) {
        set_error(ph,PH_PHASE_ERROR,"received malformed data record");
      }
    }
    else if (ph->data_buffer[1] - '1' != ph->station) {
      ph->station = ph->data_buffer[1] - '1';
      if (ph->station < 0 || ph->station > 3) {
	ph->station = 0;
      }
      set_error(ph,PH_PHASE_ERROR,"received malformed data record");
    } 

    /* find the next active station */
    for (i = 0; i < 4; i++) {
      ph->station++;
      if (ph->station > 3) {
	ph->station = 0;
      }
      if (ph->station_active[ph->station]) {
	break;
      }
    }

    return fresh;
  }    
#endif /* POLHEMUS_USE_THREADS */

  len = record_length(ph,ph->station);
  phReceiveRaw(ph,ph->data_buffer,len,0);
  set_timestamp(&ph->timestamp_secs,&ph->timestamp_msecs);

  if (phGetStation(ph) != ph->station + 1) {
      fprintf(stderr, "station mismatch %i %i\n", ph->data_buffer[1] - '1', ph->station);
    if (phGetStation(ph)) {
      ph->station = phGetStation(ph);
    }
    set_error(ph,PH_PHASE_ERROR,"received malformed data record");
  }

  /* find the next active station */
  for (i = 0; i < 4; i++) {
    ph->station++;
    if (ph->station > 3) {
      ph->station = 0;
      ph->point = 0;   /* finished point data */
    }
    if (ph->station_active[ph->station]) {
      break;
    }
  }

  return 1;
}

/** \fn      void phGetPosition(polhemus *ph,
                                float xyz[3])
    \ingroup DataMethods

Get the position returned in the last phUpdate() data record.

\param ph         pointer to a polhemus structure
\param xyz        storage space for the position to be returned in

The bird positions are only available if phSetReplyFormat() was called
with one of the following modes: PH_POSITION, PH_POSITION_ANGLES,
PH_POSITION_MATRIX, PH_POSITION_QUATERNION.
*/

void phGetPosition(polhemus *ph, float xyz[3])
{
  static unsigned char pos_formats[3] = { 2, 18, 52 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, pos_formats, 3, &format);

  if (i == 0) {
    xyz[0] = 0.0f;
    xyz[1] = 0.0f;
    xyz[2] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 18) { /* 16-bit */
    if (ph->centimeters) {
      for (i = 0; i < 3; i++) {
        xyz[i] = (float)(phBinary16ToInt(&cp)*0.018310546875f);
      }
    }
    else {
      for (i = 0; i < 3; i++) {
        xyz[i] = (float)(phBinary16ToInt(&cp)*0.0072088767224f);
      }
    }
  }
  else if (format == 52) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      xyz[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      xyz[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      xyz[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetMotion(polhemus *ph,
                                float xyz[3])
    \ingroup DataMethods

Get the motion returned in the last phUpdate() data record.

\param ph         pointer to a polhemus structure
\param xyz        storage space for the position to be returned in

The bird positions are only available if phSetReplyFormat() was called
with one of the following modes: PH_POSITION, PH_POSITION_ANGLES,
PH_POSITION_MATRIX, PH_POSITION_QUATERNION.
*/

void phGetMotion(polhemus *ph, float xyz[3])
{
  static unsigned char pos_formats[2] = { 3, 53 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, pos_formats, 2, &format);

  if (i == 0) {
    xyz[0] = 0.0f;
    xyz[1] = 0.0f;
    xyz[2] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 53) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      xyz[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      xyz[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      xyz[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetAngles(polhemus *ph,
                              float zyx[3])
    \ingroup DataMethods

Get the euler angles returned in the last phUpdate() data record.

\param ph         pointer to a polhemus structure
\param zyx        storage space for the angles to be returned in

The bird angles are only available if phSetReplyFormat() was called
with one of the following modes: PH_ANGLES, PH_POSITION_ANGLES.
*/

void phGetAngles(polhemus *ph, float zyx[3])
{
  static unsigned char angle_formats[3] = { 4, 19, 54 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, angle_formats, 3, &format);

  if (i == 0) {
    zyx[0] = 0.0f;
    zyx[1] = 0.0f;
    zyx[2] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 19) { /* 16-bit */
    for (i = 0; i < 3; i++) {
      zyx[i] = (float)(phBinary16ToInt(&cp)*0.010986328125f);
    }
  }
  else if (format == 54) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      zyx[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      zyx[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      zyx[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetXDirCos(polhemus *ph,
                               float a[3])
    \ingroup DataMethods

Get the components of the x direction cosine returned in the last
phUpdate() data record.

\param ph         pointer to a polhemus structure
\param a          storage space for the three components

The direction cosines are only available if phSetReplyFormat() was called
with one of the following modes: PH_MATRIX, PH_POSITION_MATRIX.
*/

void phGetXDirCos(polhemus *ph, float a[3])
{
  static unsigned char dircos_formats[3] = { 5, 55 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, dircos_formats, 3, &format);

  if (i == 0) {
    a[0] = 1.0f;
    a[1] = 0.0f;
    a[2] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 55) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      a[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetYDirCos(polhemus *ph,
                               float a[3])
    \ingroup DataMethods

Get the components of the y direction cosine returned in the last
phUpdate() data record.

\param ph         pointer to a polhemus structure
\param a          storage space for the three components

The direction cosines are only available if phSetReplyFormat() was called
with one of the following modes: PH_MATRIX, PH_POSITION_MATRIX.
*/

void phGetYDirCos(polhemus *ph, float a[3])
{
  static unsigned char dircos_formats[3] = { 6, 56 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, dircos_formats, 3, &format);

  if (i == 0) {
    a[0] = 0.0f;
    a[1] = 1.0f;
    a[2] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 55) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      a[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetZDirCos(polhemus *ph,
                               float a[3])
    \ingroup DataMethods

Get the components of the z direction cosine returned in the last
phUpdate() data record.

\param ph         pointer to a polhemus structure
\param a          storage space for the three components

The direction cosines are only available if phSetReplyFormat() was called
with one of the following modes: PH_MATRIX, PH_POSITION_MATRIX.
*/

void phGetZDirCos(polhemus *ph, float a[3])
{
  static unsigned char dircos_formats[2] = { 7, 57 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, dircos_formats, 2, &format);

  if (i == 0) {
    a[0] = 0.0f;
    a[1] = 0.0f;
    a[2] = 1.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 55) { /* extended ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 3; i++) {
      a[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 3; i++) {
      a[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      void phGetQuaternion(polhemus *ph,
                                  float q[4])
    \ingroup DataMethods

Get the quaternion returned in the last phUpdate() data record.

\param ph         pointer to a polhemus structure
\param q          storage space for the quaternion to be returned in

The bird quaternion is only available if phSetReplyFormat() was called
with one of the following modes: PH_QUATERNION, PH_POSITION_QUATERNION.
*/

void phGetQuaternion(polhemus *ph, float q[4])
{
  static unsigned char quat_formats[3] = { 11, 20, 61 };
  int station;
  int i, format;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, quat_formats, 3, &format);

  if (i == 0) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }

  cp = &ph->data_buffer[i];

  if (format == 20) { /* 16-bit */
    for (i = 0; i < 4; i++) {
      q[i] = (float)(phBinary16ToInt(&cp)*0.000030517578125f);
    }
  }
  else if (format == 61) { /* extended ascii */
    for (i = 0; i < 4; i++) {
      q[i] = phAsciiExToFloat(&cp);
    }
  }
  else if (ph->binary == 0) { /* normal ascii */
    for (i = 0; i < 4; i++) {
      q[i] = phAsciiToFloat(&cp);
    }
  }
  else { /* binary floating point */
    for (i = 0; i < 4; i++) {
      q[i] = phBinaryToFloat(&cp);
    }
  }
}

/** \fn      int phGetButton(polhemus *ph)
    \ingroup DataMethods

Get the button state returned in the last phUpdate() data record.

\param ph         pointer to a polhemus structure

\return           button state: 0 or 1

The return value is always zero unless phSetButtonMode() has been
used to turn on button reporting.
*/

int phGetButton(polhemus *ph)
{
  static unsigned char switch_formats[2] = { 6, 66 };
  int station;
  int i, format, bval;
  char *cp;

  station = ph->station;

  i = record_offset(ph, station, switch_formats, 2, &format);

  if (i == 0) {
    return 0;
  }

  cp = &ph->data_buffer[i];
  
  bval = 0;
  if (format == 6) {
    for (i = 0; i < 2; i++)
      if (cp[i] >= '0' && cp[i] <= '9') {
	bval = (bval << 3) + (bval << 1) + (cp[i] - '0');
      }
  }
  else {
    for (i = 0; i < 3; i++)
      if (cp[i] >= '0' && cp[i] <= '9') {
	bval = (bval << 3) + (bval << 1) + (cp[i] - '0');
      }
  }

  return bval;
}

/** \fn      int phGetStation(polhemus *ph)
    \ingroup DataMethods

Get the phb address of the bird for the data record obtained through
the last phUpdate().

\param ph         pointer to a polhemus structure

\return           a value between 1 and 4

A return value of zero indicates that a phase error or some
other communication problem occurred with the polhemus.
*/

int phGetStation(polhemus *ph)
{
  int station;

  if (ph->data_buffer[0] == '0' &&
      ph->data_buffer[1] >= '1' &&
      ph->data_buffer[1] <= '4') {
    station = ph->data_buffer[1] - '0';
  }
  else {
    station = 0;
  }

  return station;
}

/** \fn      double phGetTime(polhemus *ph)
    \ingroup DataMethods

Get the timestamp (in seconds since 1970) for the last phUpdate().

\param ph         pointer to a polhemus structure

\return           an absolute time value in seconds

The time is generated by the computer, not by the polhemus.  The time is
only accurate to within a few milliseconds.
*/

double phGetTime(polhemus *ph)
{
  return ph->timestamp_secs + 0.001*ph->timestamp_msecs;
}

/*---------------------------------------------------------------------*/
/** \defgroup ConversionMethods Data Format Conversion

These are helper functions that convert data from one format to
another in order to ease the decoding of data records sent by the polhemus.
*/

/** \fn      void phMatrixFromAngles(float a[9],
                                     const float zyx[3])
    \ingroup ConversionMethods

Convert euler angles into a 3x3 matrix.

\param a        the nine matrix elements are stored here, column by column
\param zyx      the three angles
*/

void phMatrixFromAngles(float a[9], const float zyx[3])
{
  double cx,sx,cy,sy,cz,sz;

  cz = cos((double)(zyx[0]*0.017453292519943295));
  sz = sin((double)(zyx[0]*0.017453292519943295));
  cy = cos((double)(zyx[1]*0.017453292519943295));
  sy = sin((double)(zyx[1]*0.017453292519943295));
  cx = cos((double)(zyx[2]*0.017453292519943295));
  sx = sin((double)(zyx[2]*0.017453292519943295));

  a[0] = (float)(cy*cz);
  a[3] = (float)(cy*sz);
  a[6] = (float)(-sy);

  a[1] = (float)(-cx*sz + sx*sy*cz);
  a[4] = (float)(cx*cz + sx*sy*sz);
  a[7] = (float)(sx*cy);

  a[2] = (float)(sx*sz + cx*sy*cz);
  a[5] = (float)(-sx*cz + cx*sy*sz);
  a[8] = (float)(cx*cy);
}

/** \fn      void phAnglesFromMatrix(float zyx[3],
                                     const float a[9])
    \ingroup ConversionMethods

Does the opposite of phMatrixFromAngles().

\param zyx        the three angles are stored here
\param a          the matrix
*/

void phAnglesFromMatrix(float zyx[3], const float a[9])
{
  double r,cz,sz;

  r = sqrt((double)(a[0]*a[0] + a[1]*a[1]));
  cz = a[0]/r;
  sz = a[3]/r;

  zyx[0] = (float)(atan2(sz, cz)*57.295779513082323);
  zyx[1] = (float)(atan2((double)(-a[2]),cz*a[0]+sz*a[1])*57.295779513082323);
  zyx[2] = (float)(atan2(sz*a[6]-cz*a[7],-sz*a[3]+cz*a[4])*57.295779513082323);
}

/** \fn      void phSendCommand(polhemus *ph, const char *command);
    \ingroup CommandMethods

Send a command to the polhemus.

\param ph         pointer to a polhemus structure
\param command    the command to send (a null-terminated string)
*/

void phSendCommand(polhemus *ph, const char *command)
{
  int station;
  int state;
  int i;
  char *cp;

  int n = 0;
  for (n = 0; n < 256; n++) {
    if (command[n] == '\0') {
      break;
    }
  }

  phSendRaw(ph, command, n);

  /* check for error */

  /* keep track of certain commands */
  switch (command[0])
    {
    case 'F':  /* enable ascii reporting */
      ph->binary = 0;
      break;
    case 'f':  /* enable binary reporting */ 
      ph->binary = 1;
      break;
    case 'U':  /* british units (inches) */
      ph->centimeters = 0;
      break;
    case 'u':  /* metric units (centimeters) */
      ph->centimeters = 1;
      break;
    case 'l':  /* set active station state */
      station = command[1] - '1';
      state = command[3] - '0';
      ph->station_active[station] = state;
    case 'e':  /* stylus button mode */
      station = command[1] - '1';
      state = command[3] - '0';
      ph->button_function[station] = state;
      break;
    case 'O':  /* set output data list */
      station = command[1] - '1';
      read_format_list(&command[3], ph->format[station], 
		       &ph->format_len[station]);
      break;
    case 'C':
    case 'c':
    case 'P':
      break;
    }
}

/** \fn      void phReceiveReply(polhemus *ph, char *reply, int n);
    \ingroup CommandMethods

Get a reply of length at most \em n from the polhemus, reading as
many bytes as possible until a \\r\\n is received.

\param ph         pointer to a polhemus structure
\param text       storage space for reply string
\param n          maximum number of characters to store
*/

void phReceiveReply(polhemus *ph, char *reply, int n)
{
  phReceiveRaw(ph, reply, n, 0);
}



/*---------------------------------------------------------------------*/
/** \defgroup RawMethods Internal Methods

These methods send raw data to the polhemus and read raw data from the polhemus.
They should only be used by someone who is very familiar both with the
polhemus of birds and with the driver code.
*/

/** \fn      void phSendRaw(polhemus *ph,
                            const char *text,
                            int len)
    \ingroup RawMethods

This function is meant primarily for internal use.  It sends a
raw stream of bytes to the polhemus.

\param ph         pointer to a polhemus structure
\param text       the bytes to send to the polhemus
\param len        the number of bytes to send

If a command is sent to the polhemus with this function that causes
the state of the polhemus to change, then communication with the polhemus
might be disrupted.  The phSendCommand() function should be
used instead of phSendRaw().
*/

void phSendRaw(polhemus *ph, const char *text, int len)
{
  int error;

#if defined(_WIN32) || defined(WIN32)
  DWORD m,n,dumb;

#ifdef _MT
  int asynchronous;
  asynchronous = ph->asynchronous;

  if (asynchronous != PH_NOTHREAD) {
    WaitForSingleObject(ph->file_mutex,INFINITE);
  }
#endif /* _MT */

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  int i,n;
  struct timeval start_time,tv;

#ifdef _POSIX_THREADS
  int asynchronous;
  asynchronous = ph->asynchronous;

  if (asynchronous != PH_NOTHREAD) {
    pthread_mutex_lock(&ph->file_mutex);
  }
#endif /* _POSIX_THREADS */
#endif /* unix */

  n = len;
  error = 0;
  
  if (text[0] != 'P') { 
    fprintf(stderr,"SEND: \'%.*s\'\n",len,text); /* debug line, print output */
  }

#if defined(_WIN32) || defined(WIN32)
  while (WriteFile(ph->file,text,n,&m,NULL) == FALSE) {
    if (GetLastError() == ERROR_OPERATION_ABORTED) {/* system cancelled us */
      ClearCommError(ph->file,&dumb,NULL); /* so clear error */
    }
    else {
      error = PH_IO_ERROR;
      break;
    }
  }
  if (!error && m != n) {  /* incomplete write: must have timed out */
    error = PH_TIMEOUT_ERROR;
  }

#ifdef _MT
  if (asynchronous != PH_NOTHREAD) {
    ReleaseMutex(ph->file_mutex);   /* release comm port */
  }
#endif

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  gettimeofday(&start_time,0);
  while ((i = write(ph->file,text,n)) != n) { 
    if (i == -1 && errno != EAGAIN) {
      error = PH_IO_ERROR;
      break;
    }
    n -= i;
    gettimeofday(&tv,0);
    tv.tv_sec -= start_time.tv_sec;
    tv.tv_usec -= start_time.tv_usec;
    if (tv.tv_sec*1000 + tv.tv_usec/1000 > TIMEOUT_PERIOD) {
      error = PH_TIMEOUT_ERROR;
      break;
    }
  }
#ifdef _POSIX_THREADS
  if (asynchronous != PH_NOTHREAD) {
    pthread_mutex_unlock(&ph->file_mutex);   /* release comm port */
  }
#endif
#endif /* unix */

  if (error == PH_IO_ERROR) {
    set_error(ph,PH_IO_ERROR,"I/O error on serial port write");
  }
  else if (error == PH_TIMEOUT_ERROR) {
    set_error(ph,PH_TIMEOUT_ERROR,"timeout on serial port write");
  }
}

/** \fn      void phReceiveRaw(polhemus *ph,
                               char *reply,
                               int len,
                               int thread)
    \ingroup RawMethods

This function is meant primarily for internal use.  It reads a
raw stream of bytes from the polhemus.  If the first byte read is '2'
then the data record will be terminated after the first carriage return,
line feed combination.

\param ph         pointer to a polhemus structure
\param reply      the bytes read from the polhemus
\param len        the number of bytes to read
\param thread     0 if the function was called from the application,
                  or 1 if the function was called from the streaming
                  thread
*/

void phReceiveRaw(polhemus *ph, char *reply, int len, int thread)
{
  int error;  /* error indicator */
  int binary_record = 0; /* expecting binary data */
  int phase_offset = 3; /* offset to phase byte for 16-bit data */
  int station;  /* if receiving a data record, expect it from this station */
  int i, j;

  /* WIN32 code ------------------------*/
#if defined(_WIN32) || defined(WIN32)
  DWORD m,n,dumb;

#ifdef _MT
  int asynchronous;
  asynchronous = ph->asynchronous;

  if (asynchronous != PH_NOTHREAD) { /* request comm port */
    WaitForSingleObject(ph->file_mutex,INFINITE);
  }
#endif

  /* unix code ------------------------*/
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  int m,n;

#ifdef _POSIX_THREADS
  int asynchronous;
  asynchronous = ph->asynchronous;

  if (asynchronous != PH_NOTHREAD) { /* request comm port */
    pthread_mutex_lock(&ph->file_mutex);
  }
#endif 
#endif /* unix */

  /* shared code ------------------------*/
  error = 0;
  n = len;
  i = 0;

  /* fprintf(stderr,"receiving: %d\n",n);  debug line - print output */

  if (ph->stream || ph->point) {
    /* correct for previous phase error */
    i = ph->phase_leftovers;
    n -= i;
  }
  ph->phase_leftovers = 0;
   
  /* check whether we are in 16-bit binary data mode*/
  station = ph->station;
#ifdef POLHEMUS_USE_THREADS
  if (thread) {
    station = ph->async_station;
  }
#endif
  phase_offset = binary16_offset(ph, station);
  /* check whether we are expecting either kind of binary data */
  if (ph->binary || phase_offset) {
    binary_record = 1;
  }

  /* WIN32 code ------------------------*/
#if defined(_WIN32) || defined(WIN32)
  /* code needs fixing as per UNIX code */
  while (ReadFile(ph->file,&reply[i],n,&m,NULL) == FALSE) {
    if (GetLastError() == ERROR_OPERATION_ABORTED) {/* cancelled */
      ClearCommError(ph->file,&dumb,NULL); /* so clear error */
      n -= m; /* number of chars read so far */
      i += m;
    }
    else {
      error = PH_IO_ERROR;
      break;
    }
    /* check for <CR><LF> if expecting it and terminate early */
    if (i >= 2 && !binary_record) {
      for (j = i-m+1; j <= i; j++) {
	if (reply[j-2] == '\r' && reply[j-1] == '\n') {
	  break;
	}
      }    
    }
  }
  if (!error && n != m) {
    error = PH_TIMEOUT_ERROR;
  }

#ifdef _MT
  if (asynchronous != PH_NOTHREAD) {
    ReleaseMutex(ph->file_mutex);   /* release comm port */
  }
#endif

  /* unix code ------------------------*/
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  /* fprintf(stderr,"reading i=%i, n=%i\n", i, n); */
  while (n > 0) {
    m = read(ph->file,&reply[i],1);
    /* fprintf(stderr,"m = %d, n = %d, i = %d, s=%.*s\n",m,n,i,i,reply); */
    if (m == -1 && errno != EAGAIN) {    /* if problem is not 'temporary,' */ 
      error = PH_IO_ERROR;
      break;
    }
    if (m == 0) {
      error = PH_TIMEOUT_ERROR;
      break;
    }
    i += m;
    n -= m;
    /* check for <CR><LF> if expecting it and terminate early */
    if (i >= 1 && !binary_record) {
      if (reply[i-1] == '\n') {
	break;
      }    
    }
  }  
  /* fprintf(stderr,"m = %d, n = %d, i = %d\n",m,n,i); */
  
#ifdef _POSIX_THREADS
  if (asynchronous != PH_NOTHREAD) {
    pthread_mutex_unlock(&ph->file_mutex);   /* release comm port */
  }
#endif

#endif /* unix */

  /* shared code ------------------------*/

  /* terminate string if possible */
  if (i < len) {
    reply[i] = '\0';
  }

  if (reply[0] != '0') {
    fprintf(stderr,"READ: \'%.*s\'\n",len,reply); /* debug line */
  }
  /* check for phase errors caused by noise in the serial cable */
  if ((ph->stream || ph->point) && !error) {
    /* look for reply formats corresponding to 16-bit binary */
    if (phase_offset > 0) {
      if (!(reply[phase_offset] & 0x80)) {
	error = PH_PHASE_ERROR;
	for (m = 0; m < i; m++) {
	  if (reply[m] & 0x80) { /* found the phase bit */
	    m -= phase_offset; /* move to the right position */
	    if (m < 0) {
	      m += i;
	    }
	    memmove(reply,&reply[m],i-m);
	    ph->phase_leftovers = i-m;
	    break;
	  }
	}
      }
    }
    /* if IEEE binary record, look for the trailing <CR><LF> */
    else if (binary_record) {
      if (i < 2 || reply[i-2] != '\r' || reply[i-1] != '\n') {
	for (m = i; m > 2; m--) {
	  if (reply[m-2] == '\r' && reply[m-1] == '\n') {
	    memmove(reply,&reply[m],i-m);
	    ph->phase_leftovers = i-m;
	    break;
	  }
	}
      }
    }
  }

  if (!thread) {
    if (error == PH_IO_ERROR) {
      set_error(ph,PH_IO_ERROR,"I/O error on serial port read");
    }
    else if (error == PH_TIMEOUT_ERROR) {
      set_error(ph,PH_TIMEOUT_ERROR,"timeout while waiting for data");
    }
    else if (error == PH_PHASE_ERROR) {
      set_error(ph,PH_PHASE_ERROR,"received malformed data record");
    }
  }
#ifdef POLHEMUS_USE_THREADS
  else if (ph->async_error == 0) {
    ph->async_error = error;
  }
#endif
}


/** \fn       int phAddCommandChar(const char **cp, int cval)
    \ingroup  UtilityMethods

A helper function for building commands: append a character to
a command string.  The resulting string will be null-terminated.

\param cp    pointer to a position in the command string
\param cval   the character to append
*/

void phAddCommandChar(char **cpp, int cval)
{
  *(*cpp)++ = cval;
  *(*cpp) = '\0';
}

/** \fn       int phAddCommandStation(const char **cp, int station)
    \ingroup  UtilityMethods

A helper function for building commands: append a station number to
a command string.  The resulting string will be null-terminated.

\param cp      pointer to a position in the command string
\param station  the station
*/

void phAddCommandStation(char **cpp, int station)
{
  *(*cpp)++ = (station % 10) + '0';
  *(*cpp) = '\0';
}

/** \fn       int phAddCommandParameterInt(const char **cp, int ival)
    \ingroup  UtilityMethods

A helper function for building commands: append a comma followed by
an integer value to a command string.  The resulting string will
be null-terminated.

\param cp      pointer to a position in the command string
\param ival     an integer value
*/

void phAddCommandParameterInt(char **cpp, int ival)
{
  *cpp += sprintf(*cpp, ",%i", ival);
  *(*cpp) = '\0';
}

/** \fn       int phAddCommandParameterFloat(const char **cp, double fval)
    \ingroup  UtilityMethods

A helper function for building commands: append a comma followed by
a float value to a command string.  The resulting string will be
null-terminated.

\param cp      pointer to a position in the command string
\param fval     a float value
*/

void phAddCommandParameterFloat(char **cpp, double fval)
{
  *cpp += sprintf(*cpp, ",%g", fval);
  *(*cpp) = '\0';
}

/** \fn       int phBinary16ToInt(char **cp)
    \ingroup  UtilityMethods

A helper function: unpack two characters sent from the polhemus into
a short integer, and advance the character pointer by two.  This
will convert the data from the polhemus's special 7-bit data record
encoding into conventional 8-bit data.

\param cp    pointer to a data string from the polhemus

\return      the unpacked data
*/

int phBinary16ToInt(char **cpp)
{
  unsigned char lsb;
  short msb;

  lsb = *(*cpp)++;
  msb = *(*cpp)++;
  lsb <<= 1;
  msb <<= 8;
  msb |= lsb;
  msb <<= 1;
  return msb;
}

/** \fn       float phAsciiToFloat(char **cp)
    \ingroup  RawMethods

A helper function: convert an ascii real number in the format
"+xxx.xx" to a floating point value.

\param cpp   pointer to a data string from the polhemus

\return      the unpacked data
*/

float phAsciiToFloat(char **cpp)
{
  static double mult_table[7] = { 
      0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1.0
  };
  int mantissa, exponent, sign, c;
  double multiplier = 1; 
  int i = 0;
  char *cp = *cpp;

  for (; i < 6; i++) {
    if (cp[i] != ' ') {
      break;
    }
  }

  sign = 1;
  c = cp[i];
  if (c == '+') {
    i++;
  }
  else if (c == '-') {
    sign = -1;
    i++;
  }

  mantissa = 0;
  exponent = 32767;
  for (; i < 7; i++) {
    c = cp[i];
    if (c == '.') {
      exponent = 0;
      continue;
    }
    else if (c < '0' || c > '9') {
       break;
    }
    exponent--;
    mantissa = (mantissa << 3) + (mantissa << 1) + (c - '0');
  }
  if (exponent < 0) {
    multiplier = mult_table[exponent + 6];
  }

  *cpp += 7;

  return (float)(sign*mantissa*multiplier);
}

/** \fn       float phAsciiExToFloat(char **cp)
    \ingroup  RawMethods

A helper function: convert an ascii real number in the format
"+x.xxxxxE+xx " to a floating point value.

\param cpp   pointer to a data string from the polhemus

\return      the unpacked data
*/

float phAsciiExToFloat(char **cpp)
{
  static double mult_table1[8] = { 
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7
  };
  static double mult_table2[8] = { 
    1e0, 1e8, 1e16, 1e24, 1e32, 1e40, 1e48, 1e56
  };
  static double mult_tablen1[8] = { 
    1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7
  };
  static double mult_tablen2[8] = { 
    1e0, 1e-8, 1e-16, 1e-24, 1e-32, 1e-40, 1e-48, 1e-56
  };
  int mantissa, exponent, sign, esign, c;
  double multiplier = 1.0;
  int i = 0;
  char *cp = *cpp;

  for (; i < 7; i++) {
    if (cp[i] != ' ') {
      break;
    }
  }

  sign = 1;
  c = cp[i];
  if (c == '+') {
    i++;
  }
  else if (c == '-') {
    sign = -1;
    i++;
  }

  mantissa = 0;
  exponent = 32767;
  for (; i < 8; i++) {
    c = cp[i];
    if (c == '.') {
      exponent = 0;
      continue;
    }
    else if (c < '0' || c > '9') {
       break;
    }
    exponent--;
    mantissa = (mantissa << 3) + (mantissa << 1) + (c - '0');
  }
  if (exponent < 0) {
    multiplier = mult_tablen1[-exponent];
  }

  c = cp[i];
  if (c == 'e' || c == 'E') {
    i++;
    c = cp[i];
    esign = 1;
    if (c == '+') {
      i++;
    }
    else if (c == '-') {
      esign = -1;
      i++;
    }
    exponent = 0;
    for (; i < 12; i++) {
      c = cp[i];
      if (c < '0' || c > '9') {
	break;
      }
      exponent = (exponent << 3) + (exponent << 1) + (c - '0');
    }
    if (exponent < 64) {
      if (esign < 0) {
	multiplier *=
	  mult_tablen1[exponent & 0x7] * mult_tablen2[(exponent & 0x38) >> 3];
      }
      else {
	multiplier *=
	  mult_table1[exponent & 0x7] * mult_table2[(exponent & 0x38) >> 3];
      }
    }
    else {
      if (esign < 0) {
	multiplier = 1e-64; /* force underflow */
      }
      else {
	multiplier = 1e64; /* force overflow */
      }
    }
  }

  *cpp += 13;  

  return (float)(sign*mantissa*multiplier);
}

/** \fn       float phBinaryToFloat(char **cp)
    \ingroup  UtilityMethods

A helper function: unpack an IEEE 32-bit float.

\param cpp   pointer to a data string from the polhemus

\return      the unpacked data
*/

float phBinaryToFloat(char **cpp)
{
  union {
    float fl;
    unsigned int ul;
  } uvalue; 

  uvalue.ul = (*(*cpp)++ & 0xff);
  uvalue.ul += (*(*cpp)++ & 0xff) << 4;
  uvalue.ul += (*(*cpp)++ & 0xff) << 16;
  uvalue.ul += (*(*cpp)++ & 0xff) << 24;

  return uvalue.fl;
}

/*---------------------------------------------------------------------*/
/** \defgroup ErrorMethods Error Checking

These methods are used to check whether an error occured as a result of
an attempt to communicate with the polhemus.
*/

/** \fn      int phGetError(polhemus *ph)
    \ingroup ErrorMethods

Return the last error code and clear the error indicator.

\param ph         pointer to a polhemus structure

\return           integer error code, or zero if no error

Note that the error codes are generated by the host computer,
not by the polhemus.  To check the error code for the polhemus,
use phExamineValue() to get the value of the PH_ERROR_CODE
parameter.

All of the polhemus functions can generate errors except for
the following:  phGetSwitch(), phGetPosition(), phGetAngles(),
phGetQuaternion(), phGetXDirCos(), phGetYDirCos(), phGetZDirCos().
*/

int phGetError(polhemus *ph)
{
  int error = ph->error;
  ph->error = 0;
  return error;
}

/** \fn      char *phGetErrorMessage(polhemus *ph)
    \ingroup ErrorMethods

Return some text that describes the last error.

\param ph         pointer to a polhemus structure

\return           text for the last error
*/

char *phGetErrorMessage(polhemus *ph)
{
  return ph->error_text;
}

/** \fn      void phSetErrorCallback(polhemus *ph,
                                     void (*callback)(void *data),
                                     void *data)
    \ingroup ErrorMethods

Set a callback function for error handling.  This function will
be called whenever there is a communications error with the polhemus.
it is not necessary to set an error callback.

\param ph         pointer to a polhemus structure
\param callback   pointer to callback function
\param data       pointer to data that will be sent to callback function
*/

void phSetErrorCallback(polhemus *ph, void (*callback)(void *data),
                        void *data)
{
  ph->error_handler_data = data;
  ph->error_handler = callback;
}

/*-------------------------------------------------------------
The following code is for internal use only
*/

/*-------------------------------------------
  data record parsing functions */

/* calculate the number of bytes for a particular data element */
static int record_item_length(polhemus *ph, int format)
{
  int l;

  if (format < 0 || format > 66) {
    return 0;
    }
  
  l = record_len_table[format];

  /* ascii numbers are 7 bytes, compared to 4 bytes for float */
  if (ph->binary == 0 && format > 1 && format < 16) {
    l += (l >> 2) + (l >> 1);  /* bit trick for l = l/4*7 */
  }

  return l;
}

/* given a list of format characters, calculate the total length of the
   output data records from the device */

static int record_length(polhemus *ph, int station)
{
  int l = 3;
  int i;
  unsigned char *format_list = ph->format[station];
  int format_len = ph->format_len[station];
  char test[256];

  for (i = 0; i < format_len; i++) {
    l += record_item_length(ph, format_list[i]);
    test[i] = '0' + format_list[i];
  }

  return l;
}

/* given a list of format characters, look for the first occurrence of one
   of the format characters in "try_format".  If one of the try_format
   characters is found, return that character in found_format, and return
   the offset in the data record where the data element that corresponds
   to the found_format is located. */

static int record_offset(polhemus *ph, int station, 
			 unsigned char *try_format, int try_len,
                         int *found_format)
{
  int l = 3;
  int i, j, c;
  unsigned char *format_list = ph->format[station];
  int format_len = ph->format_len[station];

  for (i = 0; i < format_len; i++) {
    c = format_list[i];
    for (j = 0; j < try_len; j++) {
      if (try_format[j] == c) {
        *found_format = c;
        return l;
      }
    }
    l += record_item_length(ph, c);
  }

  return 0;
}

/* get the offset to the first 16-bit packed binary record for
   the specified station, or zero if no 16-bit data is expected */

static int binary16_offset(polhemus *ph, int station)
{
  static unsigned char binary16_formats[] = { 18, 19, 20 };
  int format;

  return record_offset(ph, station, binary16_formats, 3, &format);
}

/* read the comma-separated list of data record item formats that
   is sent with the 'O' command */

static void read_format_list(const char *cp, unsigned char *dlist, int *len)
{
  int val;
  int i;

  for (i = 0; i < 256; i++) {
    val = 0;
    while (*cp >= '0' && *cp <= '9') {
      val *= 10;
      val += *cp++ - '0';
    }
    dlist[i] = val;
    *len = i+1;
    if (*cp++ != ',') {
      break;
    }
  }
}

/*-------------------------------------------
  error functions */

static int set_error(polhemus *ph, int error_code, const char *text)
{
  ph->error = error_code;
  strncpy(ph->error_text,text,255);
  ph->error_text[255] = '\0';
  if (ph->error_handler) {
    ph->error_handler(ph->error_handler_data);
    ph->error = 0;
  }
  return error_code;
}

static void set_timestamp(long *sec, long *msec)
{
#if defined(_WIN32) || defined(WIN32)
  /* The ftime() system time isn't precise enough, but the 
     QueryPerformanceCounter() function doesn't give the
     absolute time value that is desired.
     So we read them both once to get an offset to add to
     the QueryPerformanceCounter() value.
     (This is a quick-and-dirty type fix, it really should
     average the system time and it should correct for drift
     in the performance time) */
  static int perf_initialized = 0;
  static CRITICAL_SECTION perf_lock;
  static LARGE_INTEGER perf_freq;
  static LARGE_INTEGER perf_offset;
  static int has_perf = -1;
  LARGE_INTEGER perf_time;
  struct timeb curr_time;

  /* do this once only */
  if (!perf_initialized) {
    InitializeCriticalSection(&perf_lock);
    EnterCriticalSection(&perf_lock);
    /* re-check just in case another thread has already done it */
    if (!perf_initialized) {
      has_perf = QueryPerformanceFrequency(&perf_freq);
      if (has_perf) {
	QueryPerformanceCounter(&perf_offset);
	ftime(&curr_time);
	perf_offset.QuadPart = ((curr_time.time*perf_freq.QuadPart
				 + curr_time.millitm*perf_freq.QuadPart/1000) 
				- perf_offset.QuadPart);
      }
      perf_initialized = 1;
    }
    LeaveCriticalSection(&perf_lock);
  }
  if (has_perf) {
    QueryPerformanceCounter(&perf_time);
    perf_time.QuadPart = perf_time.QuadPart + perf_offset.QuadPart;
      *sec = (time_t)(perf_time.QuadPart
		      /perf_freq.QuadPart);
      *msec = (unsigned short)((perf_time.QuadPart
				%perf_freq.QuadPart)*1000
			       /perf_freq.QuadPart);
  }
  else {
    ftime(&curr_time);
    *sec = curr_time.time;
    *msec = curr_time.millitm;
  }
#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
  struct timeval curr_time;
  gettimeofday(&curr_time, 0);
  *sec = curr_time.tv_sec;
  *msec = curr_time.tv_usec/1000;
#endif /* unix */
}  

/*-------------------------------------------------------------
The following code is used only in asynchronous mode.  It is
non-portable and is only compiled if threading is available.
*/

/*-----
void stream_thread(void *user_data)

The stream thread starts up when phOpen() is called and
is not terminated until phClose() or phDelete() are called.
The thread waits around in a state of suspended animation
until phStream() is called.

This function contains the code that is run by the thread.
The thread ends when this function returns.  

The function consists of a loop that does the following:

1) Check ph->stream_mutex, which is unlocked only when the polhemus
   is put into stream mode by a '@' character e.g. by phStream().
   This ensures that the thread is only running if the Polhemus is
   actually streaming data.
2) Check ph->asyncronous to see whether the program wants the
   thread to stop, i.e. to see whether end_stream_thread() has
   been called.
3) Get the data record from the polhemus via the serial port.
4) Copy the data record into the polhemus structure, after the polhemus 
   structure has been locked so that the application thread cannot
   look at the data until the copy is complete
5) Inform the application that a new data record has arrived
   (use an Event under Windows and a 'cond' under UNIX)


The start_stream_thread() initializes all of the data locking structures
and starts the stream thread.

The end_stream_thread() signals for the stream thread to stop and, once
the thread has stopped, frees up all of the data locking stuctures.
*/

#ifdef POLHEMUS_USE_THREADS

#if defined(_WIN32) || defined(WIN32)
#ifdef _MT

static void stream_thread(void *user_data)
{
  long curr_time_sec, curr_time_msec;
  long old_time_sec, old_time_msec;
  int i,count,oldcount,len,check_station;
  char buffer[256];
  polhemus *ph;
  ph = (polhemus *)user_data;
  
  old_time_sec = 0;
  ph->async_station = -1;

  /* the stream-recieve loop */
  for (count = 0;; count++) {

    /* the stream_mutex is used to wake up/put to sleep this thread */
    WaitForSingleObject(ph->stream_mutex,INFINITE);
    ReleaseMutex(ph->stream_mutex);

    if (ph->asynchronous == PH_NOTHREAD) {
      break; /* no longer in asynchronous mode: terminate thread */ 
    }

    /* find the next active station */
    for (i = 0; i < 4; i++) {
      ph->async_station++;
      if (ph->async_station > 3) {
	ph->async_station = 0;
      }
      if (ph->station_active[ph->async_station]) {
	break;
      }
    }
    
    len = record_length(ph,ph->async_station);
    phReceiveRaw(ph,buffer,len,1);
    set_timestamp(&curr_time_sec,&curr_time_msec);

    /* make sure we are receiving data from the station we think we are */
    check_station = buffer[1] - '1';
    if (check_station >= 0 && check_station < 4) {
      ph->async_station = check_station;
    }

    WaitForSingleObject(ph->data_mutex,INFINITE);    
    memcpy(ph->async_buffer[ph->async_station],buffer,len);
    ph->fresh_data = 1;
    ph->async_timestamp_secs = curr_time_sec;
    ph->async_timestamp_msecs = curr_time_msec;
    ReleaseMutex(ph->data_mutex);
    SetEvent(ph->data_event);

    /* calculate the refresh rate every second */
    if (curr_time_sec > old_time_sec 
        && curr_time_msec > old_time_msec) { 
      if (old_time_sec != 0) { /* calc hertz */
        ph->async_data_rate = count - oldcount;
      }
      old_time_sec = curr_time_sec;
      old_time_msec = curr_time_msec;
      oldcount = count;
    }
  }
  // thread automatically terminates on return
}

static int start_stream_thread(polhemus *ph)
{
  ph->file_mutex = CreateMutex(0,FALSE,0);  /* lock on serial port */
  ph->data_mutex = CreateMutex(0,FALSE,0);  /* lock on ph->async_ elements */
  ph->data_event = CreateEvent(0,FALSE,FALSE,0); /* event for new data */
  ph->stream_mutex = CreateMutex(0,FALSE,0); /* only unlocked in stream mode */
  WaitForSingleObject(ph->stream_mutex,INFINITE);  
  ph->stream_thread = (HANDLE)_beginthread(&stream_thread,8*1024,ph);
  if (ph->stream_thread == INVALID_HANDLE_VALUE) {
    CloseHandle(ph->file_mutex);
    CloseHandle(ph->data_mutex);
    CloseHandle(ph->data_event);
    CloseHandle(ph->stream_mutex);
    return 0;
  }
  /* this thread spends most of its time just waiting on the serial port */
  SetThreadPriority(ph->stream_thread,THREAD_PRIORITY_TIME_CRITICAL);
  
  return 1;  /* success */ 
}

static void end_stream_thread(polhemus *ph)
{
  int async = ph->asynchronous;
  ph->asynchronous = PH_NOTHREAD;  /* this signals thread to stop */
  if (!ph->stream) { 
    ReleaseMutex(ph->stream_mutex); /* this wakes the thread up */
  }
  WaitForSingleObject(ph->stream_thread,INFINITE);
  ph->asynchronous = async;
  CloseHandle(ph->file_mutex);
  CloseHandle(ph->data_mutex);
  CloseHandle(ph->data_event);
  CloseHandle(ph->stream_mutex);
}

#endif /* _MT */

#elif defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS

static void *stream_thread(void *user_data)
{
  struct timeval curr_time;
  struct timeval old_time;
  int count,len,check_station;
  int oldcount = 0;
  char buffer[256];
  int i;
  polhemus *ph;
  ph = (polhemus *)user_data;
  
  old_time.tv_sec = 0;
  ph->async_station = -1;

  /* the stream-recieve loop */
  for (count = 0;; count++) {
    /* the stream_mutex is used to wake up/put to sleep this thread */
    pthread_mutex_lock(&ph->stream_mutex);
    pthread_mutex_unlock(&ph->stream_mutex);

    if (ph->asynchronous == PH_NOTHREAD) {
      break; /* no longer in asynchronous mode: terminate thread */ 
    }

    /* find the next active station */
    for (i = 0; i < 4; i++) {
      ph->async_station++;
      if (ph->async_station > 3) {
	ph->async_station = 0;
      }
      if (ph->station_active[ph->async_station]) {
	break;
      }
    }
 
    len = record_length(ph,ph->async_station);
    phReceiveRaw(ph,buffer,len,1);
    gettimeofday(&curr_time,0);

    /* make sure we are receiving data from the station we think we are */
    check_station = buffer[1] - '1';
    if (check_station >= 0 && check_station < 4) {
      ph->async_station = check_station;
    }

    /* lock the async data buffer and copy data to it */
    pthread_mutex_lock(&ph->data_mutex);    
    memcpy(ph->async_buffer[ph->async_station],buffer,len);
    ph->async_timestamp_secs = curr_time.tv_sec;
    ph->async_timestamp_msecs = curr_time.tv_usec/1000;
    pthread_mutex_lock(&ph->fresh_data_mutex);
    ph->fresh_data = 1;
    pthread_mutex_unlock(&ph->data_mutex);
    pthread_cond_signal(&ph->fresh_data_cond);
    pthread_mutex_unlock(&ph->fresh_data_mutex);

    /* calculate the refresh rate every second */
    if (curr_time.tv_sec > old_time.tv_sec 
        && curr_time.tv_usec > old_time.tv_usec) { 
      if (old_time.tv_sec != 0) {  /* calc hertz */
        ph->async_data_rate = (count-oldcount);
        /* fprintf(stderr,"hertz %d\n",ph->async_data_rate); */
      }
      old_time.tv_sec = curr_time.tv_sec;
      old_time.tv_usec = curr_time.tv_usec;
      oldcount = count;
    }
  }

  return 0;
}

static int start_stream_thread(polhemus *ph)
{
  pthread_mutex_init(&ph->file_mutex,0);   /* lock on serial port */
  pthread_mutex_init(&ph->data_mutex,0);   /* lock on ph->async_ elements */
  pthread_mutex_init(&ph->stream_mutex,0); /* only unlocked in stream mode */
  pthread_mutex_init(&ph->fresh_data_mutex,0); /* helper mutex for cond */
  pthread_cond_init(&ph->fresh_data_cond,0); /* signals arrival of new data */
  pthread_mutex_lock(&ph->stream_mutex);
  if (pthread_create(&ph->stream_thread,0,&stream_thread,ph)) {
    pthread_mutex_destroy(&ph->file_mutex);
    pthread_mutex_destroy(&ph->data_mutex);
    pthread_mutex_destroy(&ph->stream_mutex);
    pthread_mutex_destroy(&ph->fresh_data_mutex);
    pthread_cond_destroy(&ph->fresh_data_cond);
    return 0;
  }
  return 1;
} 

static void end_stream_thread(polhemus *ph)
{
  int async = ph->asynchronous;
  ph->asynchronous = PH_NOTHREAD;  /* this signals thread to stop */
  if (!ph->stream) { 
    pthread_mutex_unlock(&ph->stream_mutex); /* this wakes the thread up */
  }
  pthread_join(ph->stream_thread,0);
  ph->asynchronous = async;
  pthread_mutex_destroy(&ph->file_mutex);
  pthread_mutex_destroy(&ph->data_mutex);
  pthread_mutex_destroy(&ph->stream_mutex);
  pthread_mutex_destroy(&ph->fresh_data_mutex);
  pthread_cond_destroy(&ph->fresh_data_cond);
}

#endif /* _POSIX_THREADS */
#endif /* unix */

#endif /* POLHEMUS_USE_THREADS */





