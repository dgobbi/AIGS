/*
  Program:   Logitech 3D Mouse C Interface Library
  Module:    $RCSfile: logitech3d.c,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C
  Author:    $Author: dgobbi $
  Date:      $Date: 2005/10/31 02:07:16 $
  Version:   $Revision: 1.3 $
*/
/*
  Copyright 2002 Atamai Inc.
  All rights reserved
*/

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
#define LT3D_USE_THREADS 1
#endif /* _MT */
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#ifdef _POSIX_THREADS
#include <pthread.h>
#define LT3D_USE_THREADS 1
#endif /* _POSIX_THREADS */
#endif /* __unix__ */

#include "logitech3d.h"

/* set write timeout period to 1 second */
#define TIMEOUT_PERIOD 1000

/* the logitech3d structure: 
   never meant to be used anywhere but in this file */

struct logitech3d {
  /* stuff that needs to be set before the device is opened */
  int com_port;          /* comm port number, 1 or 2 */
  int baud_rate;         /* baud rate */
  int asynchronous;      /* asynchronous mode */

  /* miscellaneous state information */  
  char information[30];  /* Current Operating Information */
  char copyright[128];   /* copyright information buffer */
  char standard_config[4]; /* standard configuration */
  char specific_config[8]; /* specific configuration */

  /* storage of data etc. */
  char data_buffer[16];  /* place where record is stored after being read */
  int leftovers;         /* leftover chars after a phase error or timeout */
  int fresh_data;        /* is the data still fresh? */
  long timestamp_secs;   /* timestamp -- secs since 1970 */
  long timestamp_msecs;  /* timestamp -- millisecs */  

  /* error reporting */
  int error;             /* stores last error */
  char error_text[256];  /* stores text for last error */
  void (*error_handler)(void *); /* error callback function */
  void *error_handler_data;   /* error callback data */

  /* architecture dependent stuff */
#if defined(_WIN32) || defined(WIN32)
  HANDLE file;           /* windows file handle */
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  int file;              /* unix file handle */
#endif

  /* multithreading stuff */
#ifdef LT3D_USE_THREADS
#if defined(_WIN32) || defined(WIN32)
  HANDLE file_mutex;     /* mutex lock on file handle */
  HANDLE data_event;     /* event is triggered when new data arrives */
  HANDLE data_mutex;     /* lock on data_buffer */
  HANDLE stream_thread;  /* tracking thread for asynchronous mode */
  HANDLE stream_mutex;   /* use to pause tracking thread */
#endif /* _WIN32 */
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS
  pthread_mutex_t file_mutex;      /* mutex lock on file handle */
  pthread_cond_t  fresh_data_cond; /* condition to wait on for new data */
  pthread_mutex_t fresh_data_mutex;/* mutex lock on condition           */
  pthread_mutex_t data_mutex;      /* lock on data_buffer */
  pthread_t stream_thread;         /* tracking thread for asynchronous mode */
  pthread_mutex_t stream_mutex;    /* use to pause tracking thread */
#endif
#endif /* __unix__ */
  int stream_paused;
  int async_error;             /* error in asyncronous thread */
  char async_buffer[16];       /* buffer for use by the extra thread  */
  long async_timestamp_secs;   /* timestamp -- secs since 1970 */
  long async_timestamp_msecs;  /* timestamp -- millisecs */  
  int async_data_rate;         /* data rate (Hz) in asynchronous mode */
#endif /* LT3D_USE_THREADS */
};

/* prototypes for static internal-use-only functions */

static int set_error(struct logitech3d *lt, int error_code);
static int convert_baud_rate(int baud);
static void set_timestamp(struct logitech3d *lt);

#ifdef LT3D_USE_THREADS
#if defined(_WIN32) || defined(WIN32)
static void stream_thread(void *user_data);
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
static void *stream_thread(void *user_data);
#endif
static int start_stream_thread(struct logitech3d *lt);
static void end_stream_thread(struct logitech3d *lt);
static void pause_stream_thread(struct logitech3d *lt);
static void wake_stream_thread(struct logitech3d *lt);
#endif 

/*------
struct logitech3d *ltNew() 

returns an initialized logitech3d structure, which contains
state information about the mouse
*/

struct logitech3d *ltNew() 
{
  struct logitech3d *lt;

  lt = (struct logitech3d *)malloc(sizeof(struct logitech3d));
  lt->com_port = 0;
  lt->baud_rate = 19200;
  lt->asynchronous = LT_NOTHREAD;

  memset(lt->data_buffer,'\0',16);
  lt->leftovers = 0;
  lt->fresh_data = 0;
  lt->timestamp_secs = 0;
  lt->timestamp_msecs = 0;
  
  lt->error = 0;
  lt->error_text[0] = '\0';
  lt->error_handler = 0;
  lt->error_handler_data = 0;

  lt->file = 0;

#ifdef LT3D_USE_THREADS
  lt->async_error = 0;
  lt->async_data_rate = 0;
  lt->fresh_data = 0;
#endif

  return lt;
}

/*------
void ltDelete(struct logitech3d *lt)

release all resources used by the mouse
*/

void ltDelete(struct logitech3d *lt)
{
  if (lt->file) {
    ltClose(lt);
  }
  free(lt);
}

/*------
int convert_baud_rate(struct logitech3d *lt, int rate)

convert the baud rate into an operating-system specific form
*/

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
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
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
#endif
#endif

  switch (rate)
    {
    case 1200:
      rate = 0;
      break;
    case 19200:
      rate = 4;
      break;
    default:
      return -1;
    }

  return equiv[rate];
}

/*------
int ltOpen(struct logitech3d *lt, int port, int dimension, int mode)

open the serial port and begin communicating with the flock

valid ports are
  1,2,3,4 ...       COM1, ttyS0, ttya, ttyd1, etc, etc

valid dimensions are
  LT_2D_MODE
  LT_3D_MODE

valid modes are
  LT_NOTHREAD       synchronous mode, no multithreading 
  LT_THREAD         spawn a stream thread, improves performance

a non-zero return value signals that an error occured while opening

calling ltGetError() returns the error code, while ltGetErrorMessage()
returns a string describing the error. 
*/

int ltOpen(struct logitech3d *lt, int port, int dimension, int mode)
{
#if defined(_WIN32) || defined(WIN32)   /* start of WIN32 portion of code -------- */
  static COMMTIMEOUTS default_ctmo = { 0, 2, 
				       TIMEOUT_PERIOD, 
				       2, 
				       TIMEOUT_PERIOD };
  DCB comm_settings;
  char name[16];

  lt->asynchronous = mode;
#ifndef _MT
  if (mode != LT_NOTHREAD) {
    return set_error(lt,LT_ERROR_MODE);
  }
#endif

  lt->com_port = port;

  sprintf(name,"COM%d",lt->com_port);

  lt->file = CreateFile(name,
			GENERIC_READ|GENERIC_WRITE,
			0,  /* not allowed to share ports */
			0,  /* child-processes don't inherit handle */
			OPEN_EXISTING, 
			FILE_ATTRIBUTE_NORMAL,
			NULL); /* no template file */

  if (lt->file == INVALID_HANDLE_VALUE) {
    lt->file = 0;
    return set_error(lt,LT_ERROR_OPEN);
  }

  if (SetupComm(lt->file,1024,1024) == FALSE ||
      SetCommTimeouts(lt->file,&default_ctmo) == FALSE || 
      GetCommState(lt->file,&comm_settings) == FALSE) {
    set_error(lt,LT_ERROR_COM);
    CloseHandle(lt->file);
    return lt->error;
  }

  comm_settings.fOutX = FALSE;             /* no S/W handshake */
  comm_settings.fInX = FALSE;

  comm_settings.fAbortOnError = FALSE;     /* error recovery */

  comm_settings.fOutxDsrFlow = FALSE;      /* no modem-style handshaking */
  comm_settings.fOutxCtsFlow = FALSE;      /* no RTS/CTS handshake */
  comm_settings.fRtsControl = RTS_CONTROL_ENABLE;  /* set high */ 

  if (dimension == LT_3D_MODE) { /* 3D mode */
    comm_settings.ByteSize = 8;
    comm_settings.Parity = NOPARITY;
    comm_settings.StopBits = ONESTOPBIT;
    comm_settings.BaudRate = CBR_19200;  /* speed */
  } else {                 /* 2D mode */
    comm_settings.ByteSize = 7;
    comm_settings.Parity = NOPARITY;
    comm_settings.StopBits = ONESTOPBIT;
    comm_settings.BaudRate = CBR_1200;  /* speed */
  }

  if (SetCommState(lt->file,&comm_settings) == FALSE) {
    CloseHandle(lt->file);
    return set_error(lt,LT_ERROR_COM);
  }
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
/* start of UNIX portion of code -------------------*/

  char name[16];
  struct termios t;
  int term_bits;

  lt->asynchronous = mode;
#ifndef _POSIX_THREADS
  if (mode != LT_NOTHREAD) {
    return set_error(lt,LT_MODE_ERROR);
  }
#endif
  lt->com_port = port;

#ifdef linux
  sprintf(name,"/dev/ttyS%d",lt->com_port-1);
#elif defined(sgi)
  sprintf(name,"/dev/ttyd%d",lt->com_port);
#else
  sprintf(name,"/dev/tty%c",'a'+lt->com_port-1);
#endif

  /* port is readable/writable and is blocking */
  lt->file = open(name,O_RDWR|O_NOCTTY);

  if (lt->file == -1) {
    lt->file = 0;
    return set_error(lt,LT_ERROR_OPEN);
  }

  /* check that there is a device plugged in */
#ifdef linux
  ioctl(lt->file, TIOCMGET, &term_bits);
  if (!(term_bits & TIOCM_CTS)) {
    return set_error(lt,LT_ERROR_NO_DEVICE);
  }
#endif /* linux */

  if (tcgetattr(lt->file,&t) == -1) { /* get I/O information */
    close(lt->file);
    return set_error(lt,LT_ERROR_COM);
  }

#if defined(sgi) && defined (__NEW_MAX_BAUD)
  if (dimension == LT_3D_MODE) {
    t.c_cflag = CS8 | CREAD | CLOCAL;
    t.c_ospeed = 19200;
  } else {
    t.c_cflag = CS7 | CREAD | CLOCAL;
    t.c_ospeed = 1200;
  }    
#else
  if (dimension == LT_3D_MODE) {
    t.c_cflag = B19200 | CS8 | CREAD | CLOCAL;
  } else {
    t.c_cflag = B1200 | CS7 | CREAD | CLOCAL;
  }    
#endif

  /* clear everything specific to terminals */
  t.c_lflag = 0;
  t.c_iflag = 0;
  t.c_oflag = 0;

  t.c_cc[VMIN] = 0;                    /* use constant, not interval timout */
  t.c_cc[VTIME] = TIMEOUT_PERIOD/100;  /* wait for 1 secs max */

  if (tcsetattr(lt->file,TCSANOW,&t) == -1) { /* set I/O information */
    close(lt->file);
    return set_error(lt,LT_ERROR_COM);
  }
#endif /* __unix__ */

  /* start of system-independent portion of code ---------------*/

  /* perform a hard reset */
  ltHardReset(lt, dimension);

#ifdef LT3D_USE_THREADS
  if (!lt->error && lt->asynchronous != LT_NOTHREAD) {
    if (start_stream_thread(lt) == 0) {
      lt->asynchronous = LT_NOTHREAD;
      ltClose(lt);
      return set_error(lt,LT_ERROR_RESOURCES);
    }
  }
#endif

  if (lt->error) {
    ltClose(lt);
  }
  return lt->error;
}

/*------
void ltClose(struct logitech3d *lt)

close the serial port
*/

void ltClose(struct logitech3d *lt)
{
#ifdef LT3D_USE_THREADS
  if (lt->asynchronous != LT_NOTHREAD) {
    end_stream_thread(lt);
  }
#endif 

  if (lt->file) {
#if defined(_WIN32) || defined(WIN32)
    CloseHandle(lt->file);
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
    close(lt->file);
#endif
  }
  lt->file = 0;
}

/*------
void ltHardReset(struct logitech3d *lt, int dimension)

reset by toggling the RTS line
*/

void ltHardReset(struct logitech3d *lt, int dimension)
{
#if defined(_WIN32) || defined(WIN32)
  DCB comm_settings;
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  int term_bits;
#endif

  if (!lt->file) {
    return;
  }

  /* pause the stream thread, because we won't be streaming 3D data
     after the reset */
#ifdef LT3D_USE_THREADS
  pause_stream_thread(lt);
#endif

  lt->leftovers = 0;
  lt->error = 0;
  lt->error_text[0] = '\0';
 
#if defined(_WIN32) || defined(WIN32)
  GetCommState(lt->file,&comm_settings);
  /* set DTR line low for 3D, high for 2D */
  if (dimension == LT_3D_MODE) {
    comm_settings.fDtrControl = DTR_CONTROL_DISABLE;
  } else {
    comm_settings.fDtrControl = DTR_CONTROL_ENABLE;
  }
  Sleep(200); /* wait for 200 msec */
  /* set the RTS line low, then high */
  comm_settings.fRtsControl = RTS_CONTROL_DISABLE; /* set low */ 
  SetCommState(lt->file,&comm_settings);
  Sleep(200);                                      /* hold for 200 msec */
  comm_settings.fRtsControl = RTS_CONTROL_ENABLE;  /* set high */ 
  SetCommState(lt->file,&comm_settings);        
  Sleep(20);                                /* wait another 20 msec */

  /* do 3D specific stuff */
  if (dimension == LT_3D_MODE) {
    int diagnostics;
    ltSoftReset(lt);
    ltDemandMode(lt);
    if ((diagnostics = ltGetDiagnostics(lt)) != LT3D_DIAG_ALL) {
      set_error(lt,LT_ERROR_DIAGNOSTICS);
    }    
  } 
  /* do 2D specific stuff */
  else {
    Sleep(1000); /* wait for 1 sec for full reset */
    PurgeComm(lt->file,PURGE_TXCLEAR);
    PurgeComm(lt->file,PURGE_RXCLEAR);
  }

#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  /* the equivalent of this for other unices would be nice */
#ifdef linux
  /* set DTR line low for 3D, high for 2D */
  term_bits = TIOCM_DTR;
  if (dimension == LT_3D_MODE) {
    ioctl(lt->file, TIOCMBIC, &term_bits);
  } else {
    ioctl(lt->file, TIOCMBIS, &term_bits);
  }
  usleep(200000); /* wait for 200 msec */

  /* set the RTS line low, then high to reset */
  term_bits = TIOCM_RTS;
  ioctl(lt->file, TIOCMBIC, &term_bits);
  usleep(200000); /* low 200 msec to reset */
  ioctl(lt->file, TIOCMBIS, &term_bits);
  usleep(20000); /* wait for another 20 msec */
#endif

  /* do 3D specific stuff */
  if (dimension == LT_3D_MODE) {
    int diagnostics;
    ltSoftReset(lt);
    ltDemandMode(lt);
    if ((diagnostics = ltGetDiagnostics(lt)) != LT3D_DIAG_ALL) {
      set_error(lt,LT_ERROR_DIAGNOSTICS);
    }    
  } 
  /* do 2D specific stuff */
  else {
    usleep(1000000); /* wait for 1 sec for full reset */
    tcflush(lt->file,TCIOFLUSH);
  }

#endif /* __unix__ */

  if (dimension == LT_3D_MODE) {
    ltSetBaudRate(lt,19200);
  } else {
    ltSetBaudRate(lt,1200);
  }

#ifdef LT3D_USE_THREADS
  lt->async_error = 0;
  lt->async_data_rate = 0;
  lt->fresh_data = 0;
#endif
}  

/*------
int ltGetError(struct logitech3d *lt)

return the last error code, and clear the error indicator
*/

int ltGetError(struct logitech3d *lt)
{
  int error = lt->error;
  lt->error = 0;
  return error;
}

/*------
char *ltGetErrorMessage(struct logitech3d *lt)

return the last error in human-readable format
 */

char *ltGetErrorMessage(struct logitech3d *lt)
{
  return lt->error_text;
}

/*------
void ltSetErrorCallback(struct logitech3d *lt, void (*callback)(void *data),
			void *data)

set a callback for error handling
*/

void ltSetErrorCallback(struct logitech3d *lt, void (*callback)(void *data),
			void *data)
{
  lt->error_handler_data = data;
  lt->error_handler = callback;
}

/*------
void ltSoftReset(struct logitech3d *lt)

reset the mouse, it will end up in 3D mode

this only works in 3D mode
*/

void ltSoftReset(struct logitech3d *lt)
{
  if (!lt->file) {
    return;
  }

  ltSendCommand(lt,"*R");

  /* sleep for one second */
#if defined(_WIN32) || defined(WIN32)
  Sleep(1000);
#endif
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  usleep(1000000);
#endif

  /* pause the stream thread, because we won't be streaming
     after the reset */
#ifdef LT3D_USE_THREADS
  pause_stream_thread(lt);
#endif

#if defined(_WIN32) || defined(WIN32)
  PurgeComm(lt->file,PURGE_RXCLEAR);
#endif /* _WIN32 */
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  tcflush(lt->file,TCIFLUSH);
#endif /* __unix__ */
}

/*---------
void ltStreamMode(struct logitech3d *lt)

request stream output
*/

void ltStreamMode(struct logitech3d *lt)
{
  if (!lt->file) {
    return;
  }

  ltSendCommand(lt,"*S");

#ifdef LT3D_USE_THREADS
  wake_stream_thread(lt);
#endif
}

/*---------
void ltIncrementalMode(struct logitech3d *lt)

request incremental output
*/

void ltIncrementalMode(struct logitech3d *lt)
{
  if (!lt->file) {
    return;
  }

  ltSendCommand(lt,"*I");

#ifdef LT3D_USE_THREADS
  wake_stream_thread(lt);
#endif
}

/*---------
void ltDemandMode(struct logitech3d *lt)

request on-demand output
*/

void ltDemandMode(struct logitech3d *lt)
{
  if (!lt->file) {
    return;
  }

  ltSendCommand(lt,"*D");

#ifdef LT3D_USE_THREADS
  pause_stream_thread(lt);
#endif

  /* purge the receive buffer before continuing */
#if defined(_WIN32) || defined(WIN32)
  Sleep(300);
  PurgeComm(lt->file,PURGE_RXCLEAR);
#endif /* _WIN32 */
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  usleep(300000);
  tcflush(lt->file,TCIFLUSH);
#endif /* __unix__ */
}

/*---------
void ltDemand(struct logitech3d *lt)

request a data record (only works in demand mode)
*/

void ltDemand(struct logitech3d *lt)
{
  ltSendCommand(lt,"*d");
}

/*------
int ltReport(struct logitech3d *lt, int timeout)

this is the central function in the interface: it waits for a single
packet of information, and returns null if the timeout was exceeded
before the packet was returned 
*/

int ltReport(struct logitech3d *lt, int timeout)
{
#ifdef LT3D_USE_THREADS
  int fresh = 0;
  int async_error;

  if (lt->asynchronous != LT_NOTHREAD && !lt->stream_paused) {

#if defined(_WIN32) || defined(WIN32)
#ifdef _MT

    /* wait until a new data record arrives */
    if (timeout != 0) {
      if (WaitForSingleObject(lt->data_event,timeout) == WAIT_TIMEOUT) {
	return 0;
      }
    }

    /* get a lock on the data record */
    WaitForSingleObject(lt->data_mutex,INFINITE);
    memcpy(lt->data_buffer,lt->async_buffer,16);
    lt->timestamp_secs = lt->async_timestamp_secs;
    lt->timestamp_msecs = lt->async_timestamp_msecs;
    fresh = lt->fresh_data;
    lt->fresh_data = 0;
    async_error = lt->async_error;
    lt->async_error = 0;
    ReleaseMutex(lt->data_mutex);

#endif /* _MT */
#endif /* _WIN32 */
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS

    /* wait until a new data record arrives */
    if (timeout != 0) {
      struct timeval tv;
#ifndef PTHREAD_COND_TIMEDWAIT_USES_TIMEVAL
      struct timespec ts;
#endif
      pthread_mutex_lock(&lt->fresh_data_mutex);
      if (!lt->fresh_data) {
	/* all the time stuff is used to check for timeouts */
	gettimeofday(&tv,0);
	tv.tv_sec += timeout/1000; /* msec to sec */ 
	tv.tv_usec += (timeout % 1000)*1000; /* msec to usec */
	if (tv.tv_usec >= 1000000) { /* if usec overflow */
	  tv.tv_usec -= 1000000;
	  tv.tv_sec += 1;
	}
#ifdef PTHREAD_COND_TIMEDWAIT_USES_TIMEVAL
	if (pthread_cond_timedwait(&lt->fresh_data_cond,
				   &lt->fresh_data_mutex,&tv) 
	    == ETIMEDOUT)
#else   /* convert timeval to timespec */
        ts.tv_sec = tv.tv_sec;
        ts.tv_nsec = tv.tv_usec * 1000; 
	if (pthread_cond_timedwait(&lt->fresh_data_cond,
				   &lt->fresh_data_mutex,&ts) 
	    == ETIMEDOUT)
#endif
	{
	  pthread_mutex_unlock(&lt->fresh_data_mutex);
	  return 0;
	}
      }
      pthread_mutex_unlock(&lt->fresh_data_mutex);
    }

    /* get a lock on the data record */
    pthread_mutex_lock(&lt->data_mutex);
    memcpy(lt->data_buffer,lt->async_buffer,16);
    lt->timestamp_secs = lt->async_timestamp_secs;
    lt->timestamp_msecs = lt->async_timestamp_msecs;
    fresh = lt->fresh_data;
    lt->fresh_data = 0;
    async_error = lt->async_error;
    lt->async_error = 0;
    pthread_mutex_unlock(&lt->data_mutex);

#endif /* _POSIX_THREADS */
#endif /* __unix__ */

    /* check for error code set by stream thread */
    if (async_error) {
      set_error(lt,async_error);
    }
  }    
  else
#endif /* LT3D_USE_THREADS */

  /* this code is only executed if threading isn't enabled */
  {
    int old_timeout;
    int error;

#if defined(_WIN32) || defined(WIN32)
    COMMTIMEOUTS ctmo;

    GetCommTimeouts(lt->file,&ctmo);
    old_timeout =  ctmo.ReadTotalTimeoutConstant;
    ctmo.ReadTotalTimeoutConstant = timeout;
    SetCommTimeouts(lt->file,&ctmo); 

    error = ltReceiveRaw(lt,lt->data_buffer,16);

    ctmo.ReadTotalTimeoutConstant = old_timeout;
    SetCommTimeouts(lt->file,&ctmo); 
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
    struct termios t;

    tcgetattr(lt->file,&t);
    old_timeout = t.c_cc[VTIME];
    t.c_cc[VTIME] = ((timeout < 25500) ? timeout/100 : 255);
    tcsetattr(lt->file,TCSANOW,&t);

    error = ltReceiveRaw(lt,lt->data_buffer,16);
    
    t.c_cc[VTIME] = old_timeout;
    tcsetattr(lt->file,TCSANOW,&t);
#endif

    set_timestamp(lt);

    if (error) {
      if (error != LT_ERROR_TIMEOUT) {
	set_error(lt,error);
      }
    } else {
      fresh = 1;
    }
  }

  return fresh;
}

/*--------
void ltGetPosition(struct logitech3d *lt, int xyz[3])

get the position returned by the last ltReport() and
put it in xyz[3].
*/

void ltGetPosition(struct logitech3d *lt, int xyz[3])
{
  char *cp;
  int i, result;

  cp = lt->data_buffer + 1;

  for (i = 0; i < 3; i++) {
    result = (*cp++ & 0x7f);
    result = (result << 7) | (*cp++ & 0x7f);
    result = (result << 7) | (*cp++ & 0x7f);
    if (result & 0x00100000) {
      result |= 0xfff00000;
    }
    xyz[i] = result;
  }
}

/*--------
void ltGetAngles(struct logitech3d *lt, float zyx[3])

get the euler angles returned by the last ltReport() and
put in pyr[3].
*/

void ltGetRotation(struct logitech3d *lt, int pyr[3])
{
  char *cp;
  int i, result;

  cp = lt->data_buffer + 10;

  for (i = 0; i < 3; i++) {
    result = (*cp++ & 0x7f);
    result = (result << 7) | (*cp++ & 0x7f);
    pyr[i] = result;
  }
}

/*--------
void ltGetStatus(struct logitech3d *lt, int *status)

return the status byte from the last ltReport()
*/

void ltGetStatus(struct logitech3d *lt, int *status)
{
  *status = (lt->data_buffer[0] & 0x7f);
}

/*--------
double ltGetTime(struct logitech3d *lt)

get the timestamp in seconds since 1970, the second element is milliseconds, 
for the last ltReport()
*/

void ltGetTime(struct logitech3d *lt, int time[2])
{
  time[0] = lt->timestamp_secs;
  time[1] = lt->timestamp_msecs;
}

/*--------
int ltGetDiagnostics(struct logitech3d *lt)

diagnostic information
*/

int ltGetDiagnostics(struct logitech3d *lt)
{
  char data[2];

  if (!lt->file) {
    return 0;
  }

  ltSendCommand(lt, "*\x05");
  ltReceiveReply(lt, data, 2);

  return ((data[0] & 0xff) << 8) | data[1];
}

/*---------
void ltEnableAudio(struct logitech3d *lt)
*/

void ltEnableAudio(struct logitech3d *lt)
{
  ltSendCommand(lt,"*A");
}

/*---------
void ltDisableAudio(struct logitech3d *lt)
*/

void ltDisableAudio(struct logitech3d *lt)
{
  ltSendCommand(lt,"*O");
}

/*---------
void ltMPlusMode(struct logitech3d *lt)
*/

void ltMPlusMode(struct logitech3d *lt)
{
  ltSendCommand(lt,"*X");
}

/*---------
void ltMicrosoftMode(struct logitech3d *lt)
*/

void ltMicrosoftMode(struct logitech3d *lt)
{
  ltSendCommand(lt,"*V");
}

/*---------
char *ltGetInformation(struct logitech3d *lt)
*/

char *ltGetInformation(struct logitech3d *lt)
{
  if (lt->file) {
    ltSendCommand(lt,"*m");
    ltReceiveReply(lt, lt->information, 30);
  }
  return lt->information;
}

/*---------
char *ltGetCopyright(struct logitech3d *lt)
*/

char *ltGetCopyright(struct logitech3d *lt)
{
  int error;

  if (lt->file) {
    ltSendCommand(lt,"*c");
    memset(lt->copyright,'\0',128);
    error = ltReceiveRaw(lt, lt->copyright, 128);
    if (error && error != LT_ERROR_TIMEOUT) {
      set_error(lt,error);
    }
  }
  return lt->copyright;
}

/*---------
char *ltGetConfiguration(struct logitech3d *lt)
*/

char *ltGetConfiguration(struct logitech3d *lt)
{
  if (lt->file) {
    ltSendCommand(lt,"*?");
    ltReceiveReply(lt, lt->standard_config, 4);
  }
  return lt->standard_config;
}

/*---------
int ltGetDeviceType(struct logitech3d *lt)
*/

int ltGetDeviceType(struct logitech3d *lt)
{
  ltSendCommand(lt,"*!");
  ltReceiveReply(lt, lt->specific_config, 7);

  return lt->specific_config[0];
}

/*------
void ltSendCommand(struct logitech3d *lt, const char *command)

send a command to the mouse
*/

void ltSendCommand(struct logitech3d *lt, const char *command)
{
  int error;

  if (!lt->file) {
    return;
  }

  error = ltSendRaw(lt,command,2);

  if (error) {
    set_error(lt,error);
  }
}

/*------
void ltSendCommand(struct logitech3d *lt, const char *command)

send a command to the mouse
*/

void ltReceiveReply(struct logitech3d *lt, char *reply, int length)
{
  int error;

  if (!lt->file) {
    return;
  }

  error = ltReceiveRaw(lt,reply,length);

  if (error) {
    set_error(lt,error);
  }
}

/*----
void ltSetBaudRate(struct logitech3d *lt, int rate)

set the baud rate of the host computer, this does nothing
to the mouse
*/

void ltSetBaudRate(struct logitech3d *lt, int baud)
{
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  struct termios t;
#endif
#if defined(_WIN32) || defined(WIN32)
  DCB comm_settings;
#endif  

  if (!lt->file) {
    return;
  }

  baud = convert_baud_rate(baud);
  if (baud == -1) {
    set_error(lt,LT_ERROR_BAUD);
    return;
  }

#if defined(_WIN32) || defined(WIN32)   /* start of WIN32 portion of code -------- */
  if (GetCommState(lt->file,&comm_settings) == FALSE) {
    set_error(lt,LT_ERROR_COM);
    return;
  }

  if (baud == CBR_19200) { /* 3D mode */
    comm_settings.ByteSize = 8;
    comm_settings.Parity = NOPARITY;
    comm_settings.StopBits = ONESTOPBIT;
  } else {                 /* 2D mode */
    comm_settings.ByteSize = 7;
    comm_settings.Parity = NOPARITY;
    comm_settings.StopBits = ONESTOPBIT;
  }

  comm_settings.BaudRate = baud;  /* speed */

  if (SetCommState(lt->file,&comm_settings) == FALSE) {
    set_error(lt,LT_ERROR_COM);
    return;
  }
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
/* start of unix portion of code -------- */
  if (tcgetattr(lt->file,&t) == -1) { /* get I/O information */
    set_error(lt,LT_ERROR_COM);
    return;
  }

#if defined(sgi) && defined (__NEW_MAX_BAUD)
  if (baud == 19200) {
    t.c_cflag = CS8 | CREAD | CLOCAL;
  } else {
    t.c_cflag = CS7 | CREAD | CLOCAL;
  }    
  t.c_ospeed = baud;
#else
  if (baud == B19200) {
    t.c_cflag = baud | CS8 | CREAD | CLOCAL;
  } else {
    t.c_cflag = baud | CS7 | CREAD | CLOCAL;
  }    
#endif

  if (tcsetattr(lt->file,TCSANOW,&t) == -1) { /* set I/O information */
    set_error(lt,LT_ERROR_COM);
    return;
  }
#endif /* __unix__ */
}  

/*------
int ltSendRaw(struct logitech3d *lt, const char *text, int len)

send raw text over the serial port, return error code

the timeout is set by the TIMEOUT_PERIOD macro
*/

int ltSendRaw(struct logitech3d *lt, const char *text, int len)
{
  int error;

#if defined(_WIN32) || defined(WIN32)
  DWORD m,n,dumb;

#ifdef _MT
  int asynchronous;
  asynchronous = lt->asynchronous;

  if (asynchronous != LT_NOTHREAD) {
    WaitForSingleObject(lt->file_mutex,INFINITE);
  }
#endif /* _MT */
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  int i,n;
  struct timeval start_time,tv;

#ifdef _POSIX_THREADS
  int asynchronous;
  asynchronous = lt->asynchronous;

  if (asynchronous != LT_NOTHREAD)
    pthread_mutex_lock(&lt->file_mutex);
#endif /* _POSIX_THREADS */
#endif /* __unix__ */

  n = len;
  error = 0;
  
  /* fprintf(stderr,"send: %s\n",text);  debug line - print output */

#if defined(_WIN32) || defined(WIN32)
  while (WriteFile(lt->file,text,n,&m,NULL) == FALSE) {
    if (GetLastError() == ERROR_OPERATION_ABORTED) {/* system cancelled us */
      ClearCommError(lt->file,&dumb,NULL); /* so clear error */
    }
    else {
      error = LT_ERROR_WRITE;
      break;
    }
  }
  if (!error && m != n) {  /* incomplete write: must have timed out */
    error = LT_ERROR_WRITE_TIMEOUT;
  }

#ifdef _MT
  if (asynchronous != LT_NOTHREAD) {
    ReleaseMutex(lt->file_mutex);   /* release comm port */
  }
#endif
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  gettimeofday(&start_time,0);
  while ((i = write(lt->file,text,n)) != n) { 
    if (i == -1 && errno != EAGAIN) {
      error = LT_ERROR_WRITE;
      break;
    }
    n -= i;
    gettimeofday(&tv,0);
    tv.tv_sec -= start_time.tv_sec;
    tv.tv_usec -= start_time.tv_usec;
    if (tv.tv_sec*1000 + tv.tv_usec/1000 > TIMEOUT_PERIOD) {
      error = LT_ERROR_WRITE_TIMEOUT;
      break;
    }
  }
#ifdef _POSIX_THREADS
  if (asynchronous != LT_NOTHREAD)
    pthread_mutex_unlock(&lt->file_mutex);   /* release comm port */
#endif
#endif /* __unix__ */

  return error;
}

/*------
int ltReceiveRaw(struct logitech3d *lt, char *reply, int len)

this command will read 'len' bytes from the serial port and place them
in 'reply', and will return an error code if an error occurred

phase-error checking and correction is done:  the first byte of every
data record must have its high bit set, and no other bytes may have this
bit set, otherwise a phase error occurred
*/

int ltReceiveRaw(struct logitech3d *lt, char *reply, int len)
{
  int error;

  /* WIN32 code ------------------------*/
#if defined(_WIN32) || defined(WIN32)
  DWORD m,n,dumb;
  int i = 0;

#ifdef _MT
  int asynchronous;
  asynchronous = lt->asynchronous;

  if (asynchronous != LT_NOTHREAD) { /* request comm port */
    WaitForSingleObject(lt->file_mutex,INFINITE);
  }
#endif
#endif /* _WIN32 */

  /* unix code ------------------------*/
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  int m,n,i;

#ifdef _POSIX_THREADS
  int asynchronous;
  asynchronous = lt->asynchronous;

  if (asynchronous != LT_NOTHREAD) { /* request comm port */
    pthread_mutex_lock(&lt->file_mutex);
  }
#endif 
#endif /* __unix__ */

  /* shared code ------------------------*/
  error = 0;
  n = len;
  i = 0;

  /* fprintf(stderr,"receiving: %d\n",n);  debug line - print output */

  /* correct for previous phase error */
  i = lt->leftovers;
  n -= i;
  lt->leftovers = 0;

  /* WIN32 code ------------------------*/
#if defined(_WIN32) || defined(WIN32)
  while (ReadFile(lt->file,&reply[i],n,&m,NULL) == FALSE) {
    if (GetLastError() == ERROR_OPERATION_ABORTED) {/* cancelled */
      ClearCommError(lt->file,&dumb,NULL); /* so clear error */
      n -= m; /* number of chars read so far */
      i += m;
    }
    else {
      error = LT_ERROR_READ;
      break;
    }
  }
  if (!error && n != m) {
    lt->leftovers = i+m;
    error = LT_ERROR_TIMEOUT;
  }

#ifdef _MT
  if (asynchronous != LT_NOTHREAD) {
    ReleaseMutex(lt->file_mutex);   /* release comm port */
  }
#endif
#endif /* _WIN32 */

  /* unix code ------------------------*/
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  while (!error && (m = read(lt->file,&reply[i],n)) != n) {
    /* fprintf(stderr,"m = %d, n = %d, i = %d\n",m,n,i); */
    if (m == -1 && errno != EAGAIN) {    /* if problem is not 'temporary,' */ 
      error = LT_ERROR_READ;
    }
    else if (m == 0) {
      lt->leftovers = i;
      error = LT_ERROR_TIMEOUT;
    }
    else {
      i += m;
      n -= m;
    }
  }      
  /* fprintf(stderr,"m = %d, n = %d, i = %d\n",m,n,i); */
  
#ifdef _POSIX_THREADS
  if (asynchronous != LT_NOTHREAD) {
    pthread_mutex_unlock(&lt->file_mutex);   /* release comm port */
  }
#endif

#endif /* __unix__ */

  /* shared code ------------------------*/
  if (!error) {  /* check for phase errors */
    if (!(reply[0] & 0x80)) {
      error = LT_ERROR_PHASE;
    }
    for (i = 1; i < len; i++) {
      if (reply[i] & 0x80) {
	error = LT_ERROR_PHASE;
	memmove(reply,&reply[i],len-i);
	lt->leftovers = len-i;
	break;
      }
    }
  }

  return error;
}

/*-------------------------------------------------------------
The following code is for internal use only
*/
      
static int set_error(struct logitech3d *lt, int error_code)
{
  static char *error_text[14] = { "",
				  "mode not supported on this system",
				  "error opening serial port",
				  "no device found on serial port",
				  "out of system resources",
				  "incorrect baud rate",
				  "comm port setup error",
				  "I/0 error on read",
				  "I/0 error on write",
				  "timeout error on read",
				  "timeout error on write",
				  "phase error in data record",
				  "mouse failed diagnostics",
				  "bad error code"};

  if (error_code > 12 || error_code < 0) {
    error_code = 13;
  }

  lt->error = error_code;
  strncpy(lt->error_text,error_text[error_code],255);
  lt->error_text[255] = '\0';
  if (lt->error_handler) {
    lt->error_handler(lt->error_handler_data);
    lt->error = 0;
  }
  return error_code;
}

static void set_timestamp(struct logitech3d *lt)
{
#if defined(_WIN32) || defined(WIN32)
  struct timeb curr_time;
  ftime( &curr_time );
  lt->timestamp_secs = curr_time.time;
  lt->timestamp_msecs = curr_time.millitm;
#endif /* _WIN32 */
#if defined(__unix__) || defined(unix) || defined(__APPLE__)
  struct timeval curr_time;
  gettimeofday(&curr_time, 0);
  lt->timestamp_secs = curr_time.tv_sec;
  lt->timestamp_msecs = curr_time.tv_usec/1000;
#endif /* __unix__ */
}  

/*-------------------------------------------------------------
The following code is used only in asynchronous mode.  It is
non-portable and is only compiled if threading is available
*/

#ifdef LT3D_USE_THREADS

#if defined(_WIN32) || defined(WIN32)
#ifdef _MT

static void stream_thread(void *user_data)
{
  struct timeb curr_time,old_time;
  int count;
  int oldcount = 0;
  int error;
  char buffer[16];
  struct logitech3d *lt;
  lt = (struct logitech3d *)user_data;
  
  old_time.time = 0;

  /* the stream-recieve loop */
  for (count = 0;; count++) {

    /* the stream_mutex is used to wake up/put to sleep this thread */
    WaitForSingleObject(lt->stream_mutex,INFINITE);
    ReleaseMutex(lt->stream_mutex);

    if (lt->asynchronous == LT_NOTHREAD) {
      break; /* no longer in asynchronous mode: terminate thread */ 
    }

    error = ltReceiveRaw(lt,buffer,16);
    ftime(&curr_time);

    if (error == LT_ERROR_TIMEOUT) {
      continue;
    } 

    WaitForSingleObject(lt->data_mutex,INFINITE);    
    memcpy(lt->async_buffer,buffer,16);
    lt->async_error = error;
    lt->fresh_data = 1;
    lt->timestamp_secs = curr_time.time;
    lt->timestamp_msecs = curr_time.millitm;    
    ReleaseMutex(lt->data_mutex);
    SetEvent(lt->data_event);

    /* calculate the refresh rate every second */
    if (curr_time.time > old_time.time 
	&& curr_time.millitm > old_time.millitm) { 
      if (old_time.time != 0) { /* calc hertz */
	lt->async_data_rate = count - oldcount;
      }
      old_time.time = curr_time.time;
      old_time.millitm = curr_time.millitm;
      oldcount = count;
    }
  }
  // thread automatically terminates on return
}

static int start_stream_thread(struct logitech3d *lt)
{
  lt->file_mutex = CreateMutex(0,FALSE,0);
  lt->data_mutex = CreateMutex(0,FALSE,0);
  lt->data_event = CreateEvent(0,FALSE,FALSE,0);
  lt->stream_mutex = CreateMutex(0,FALSE,0);
  WaitForSingleObject(lt->stream_mutex,INFINITE);  
  lt->stream_paused = 1;
  lt->stream_thread = (HANDLE)_beginthread(&stream_thread,8*1024,lt);
  if (lt->stream_thread == 0) {
    CloseHandle(lt->file_mutex);
    CloseHandle(lt->data_mutex);
    CloseHandle(lt->data_event);
    CloseHandle(lt->stream_mutex);
    return 0;
  }
  SetThreadPriority(lt->stream_thread,THREAD_PRIORITY_TIME_CRITICAL);
  
  return 1;  /* success */ 
}

static void end_stream_thread(struct logitech3d *lt)
{
  int async = lt->asynchronous;
  lt->asynchronous = LT_NOTHREAD;  /* this signals thread to stop */
  if (lt->stream_paused) { 
    ReleaseMutex(lt->stream_mutex); /* this wakes the thread up */
  }
  WaitForSingleObject(lt->stream_thread,INFINITE);
  lt->asynchronous = async;
  CloseHandle(lt->file_mutex);
  CloseHandle(lt->data_mutex);
  CloseHandle(lt->data_event);
  CloseHandle(lt->stream_mutex);
}

static void pause_stream_thread(struct logitech3d *lt)
{
  if (lt->stream_paused) {
    return;
  }
  lt->stream_paused = 1;

  if (lt->asynchronous != LT_NOTHREAD) { 
    /* put the asynchronous stream thread to sleep */ 
    WaitForSingleObject(lt->stream_mutex,INFINITE);
  }

  PurgeComm(lt->file,PURGE_RXCLEAR);
}

static void wake_stream_thread(struct logitech3d *lt)
{
  if (!lt->stream_paused) {
    return;
  }
  lt->stream_paused = 0;

  if (lt->asynchronous != LT_NOTHREAD) {
    /* wake-up the asynchronous stream thread */
    ReleaseMutex(lt->stream_mutex);
  }
}

#endif /* _MT */
#endif /* _WIN32 */

#if defined(__unix__) || defined(unix) || defined(__APPLE__)
#ifdef _POSIX_THREADS

static void *stream_thread(void *user_data)
{
  struct timeval curr_time;
  struct timeval old_time;
  int count;
  int oldcount = 0;
  int error;
  char buffer[16];
  struct logitech3d *lt;
  lt = (struct logitech3d *)user_data;
  
  old_time.tv_sec = 0;

  /* the stream-recieve loop */
  for (count = 0;; count++) {
    /* the stream_mutex is used to wake up/put to sleep this thread */
    pthread_mutex_lock(&lt->stream_mutex);
    pthread_mutex_unlock(&lt->stream_mutex);

    if (lt->asynchronous == LT_NOTHREAD) {
      break; /* no longer in asynchronous mode: terminate thread */ 
    }

    error = ltReceiveRaw(lt,buffer,16);
    gettimeofday(&curr_time,0);
    
    if (error == LT_ERROR_TIMEOUT) {
      continue;
    }

    /* lock the async data buffer and copy data to it */
    pthread_mutex_lock(&lt->data_mutex);    
    memcpy(lt->async_buffer,buffer,16);
    lt->async_error = error;
    lt->async_timestamp_secs = curr_time.tv_sec;
    lt->async_timestamp_msecs = curr_time.tv_usec/1000;
    pthread_mutex_lock(&lt->fresh_data_mutex);
    lt->fresh_data = 1;
    pthread_mutex_unlock(&lt->data_mutex);
    pthread_cond_signal(&lt->fresh_data_cond);
    pthread_mutex_unlock(&lt->fresh_data_mutex);

    /* calculate the refresh rate every second */
    if (curr_time.tv_sec > old_time.tv_sec 
	&& curr_time.tv_usec > old_time.tv_usec) { 
      if (old_time.tv_sec != 0) {  /* calc hertz */
	lt->async_data_rate = (count-oldcount);
	/* fprintf(stderr,"hertz %d\n",lt->async_data_rate); */
      }
      old_time.tv_sec = curr_time.tv_sec;
      old_time.tv_usec = curr_time.tv_usec;
      oldcount = count;
    }
  }

  return 0;
}

static int start_stream_thread(struct logitech3d *lt)
{
  pthread_mutex_init(&lt->file_mutex,0);
  pthread_mutex_init(&lt->data_mutex,0);
  pthread_mutex_init(&lt->stream_mutex,0);
  pthread_mutex_init(&lt->fresh_data_mutex,0);
  pthread_cond_init(&lt->fresh_data_cond,0);
  pthread_mutex_lock(&lt->stream_mutex);
  lt->stream_paused = 1;
  if (pthread_create(&lt->stream_thread,0,&stream_thread,lt)) {
    pthread_mutex_destroy(&lt->file_mutex);
    pthread_mutex_destroy(&lt->data_mutex);
    pthread_mutex_destroy(&lt->stream_mutex);
    pthread_mutex_destroy(&lt->fresh_data_mutex);
    pthread_cond_destroy(&lt->fresh_data_cond);
    return 0;
  }
  return 1;
} 

static void end_stream_thread(struct logitech3d *lt)
{
  int async = lt->asynchronous;
  lt->asynchronous = LT_NOTHREAD;  /* this signals thread to stop */
  if (lt->stream_paused) {
    pthread_mutex_unlock(&lt->stream_mutex); /* this wakes the thread up */
  }
  pthread_join(lt->stream_thread,0);
  lt->asynchronous = async;
  pthread_mutex_destroy(&lt->file_mutex);
  pthread_mutex_destroy(&lt->data_mutex);
  pthread_mutex_destroy(&lt->stream_mutex);
  pthread_mutex_destroy(&lt->fresh_data_mutex);
  pthread_cond_destroy(&lt->fresh_data_cond);
}

static void pause_stream_thread(struct logitech3d *lt)
{
  if (lt->stream_paused) {
    return;
  }
  lt->stream_paused = 1;

  if (lt->asynchronous != LT_NOTHREAD) { 
    /* put the asynchronous stream thread to sleep */ 
    pthread_mutex_lock(&lt->stream_mutex);
  }

  tcflush(lt->file,TCIFLUSH);
}

static void wake_stream_thread(struct logitech3d *lt)
{
  if (!lt->stream_paused) {
    return;
  }
  lt->stream_paused = 0;

  if (lt->asynchronous != LT_NOTHREAD) {
    /* wake-up the asynchronous stream thread */
    pthread_mutex_unlock(&lt->stream_mutex);
  }
}

#endif /* _POSIX_THREADS */
#endif /* __unix__ */

#endif /* LT3D_USE_THREADS */





