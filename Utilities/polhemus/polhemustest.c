#include "polhemus.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "polhemus.c"

#define RAD_TO_DEG 57.2957795132

/* this is called when an error occurs */
void callback(void *vp)
{
  struct polhemus *ph = (struct polhemus *)vp;
  fprintf(stderr,"error %d: %s\n",phGetError(ph),phGetErrorMessage(ph));
}

/* this probes for the device on all serial ports and at all baud rates */
int probe(struct polhemus *ph, int *devicenum_p, int *baud_p, int *mode_p)
{
  static int baudrates[2] = { 9600,
                              38400 };
  int mode = PH_THREAD;
  int nbaud, baud, devicenum;
  int errnum;
  char *devicename;
  char reply[256];

  phSetThreadMode(ph,mode);
  errnum = phGetError(ph);
  if (errnum == PH_MODE_ERROR && mode != PH_NOTHREAD) {
    mode = PH_NOTHREAD;
    phSetThreadMode(ph,mode);
  }

  for (devicenum = 0; devicenum < 4; devicenum++) {
    devicename = phDeviceName(devicenum);
    for (nbaud = 0; nbaud < 2; nbaud++) {
      baud = baudrates[nbaud];
      phSetInitialComm(ph,baud,'N',8,0);
      fprintf(stderr, "trying %s baud %i\n", devicename, baud);
      phOpen(ph,devicename);
      errnum = phGetError(ph);
      /* if we can't open the serial port, go on to next serial port */
      if (errnum == PH_OPEN_ERROR) {
        break;
      }

      phSendCommand(ph, "\r\n");
      phReceiveReply(ph, reply, 256);
 
      errnum = phGetError(ph);
      /* if no error, then we're done switching baud rates */
      if (!errnum) {
        break;
      }
    }
    /* if no error, then we're done switching ports */
    if (!errnum) {
      break;
    }
  }

  if (!errnum) {
    phReset(ph);
    phClose(ph);
    *devicenum_p = devicenum;
    *baud_p = baud;
    *mode_p = mode;
  }
 
  return !errnum;
}

int main(int argc, char *argv[])
{
  float pos[3],ang[3];
  struct polhemus *ph;
  int i;
  char status[128];
  int devicenum = 2;
  int baud = PH_38400;
  int mode = PH_NOTHREAD;
  char *devicename;
  double starttime;
  int count = 10;

  if (argc > 1) {
    count = atoi(argv[1]);
  }

  /* return a polhemus object */
  ph = phNew();

  /* probe for port/baud rate */
  if (!probe(ph,&devicenum,&baud,&mode)) {
    fprintf(stderr,"error %s\n",phGetErrorMessage(ph));
    exit(0);
  }

  /* print out probed information */
  fprintf(stderr,"found ph on port %s at baud rate %d in mode %d\n",
          phDeviceName(devicenum),baud,mode);

  /* you can set an error callback if you want */
  phSetErrorCallback(ph,callback,ph); 

  /* get the serial port name, e.g. "COM1:" or "COM2:" */
  devicename = phDeviceName(devicenum);

  /* args are ph, serial port, baud, flags */
  fprintf(stderr, "opening device %s\n", devicename);
  phSetInitialComm(ph, baud, 'N', 8, 0); 
  phSetThreadMode(ph, mode);
  phOpen(ph,devicename);
  /* ph->file = ndiSerialOpen(devicename);
     ndiSerialComm(ph->file, 38400, "8N1", 0); */
  fprintf(stderr, "device opened\n");

  /*
  fprintf(stderr,"system status\n");
  fbExamineValueBytes(ph,FB_FBB_STATUS,status);
  fprintf(stderr,"1: %2.2x 2: %2.2x",status[0],status[1]);
  fprintf(stderr,"\n");
  */

  fprintf(stderr,"sending: P\r\n");
  /*
  write(ph->file, "P\r\n", 3);
  usleep(500000);
  read(ph->file, status, 128);
  read(ph->file, status, 128);
  */
  phSetReplyFormat(ph, 1, PH_POSITION | PH_ANGLES);
  phSendRaw(ph,"P\n",2);
  phReceiveRaw(ph, status, 45, 0);
  fprintf(stderr, "received: \'%.*s\'\n", 45, status);
  phSetReplyFormat(ph, 1, PH_POSITION | PH_ANGLES);

  /* grab a single record */
  
  starttime = phGetTime(ph);

  fprintf(stderr, "phStream\n");
  phStream(ph);
  for (i = 0;i<count;i++) {
    fprintf(stderr, "phUpdate %i\n", i);
    phUpdate(ph);
    phGetPosition(ph,pos);
    phGetAngles(ph,ang);
    fprintf(stderr,"ph %d time %10.3f pos %+6.2f %+6.2f %+6.2f ang %+6.2f %+6.2f %+6.2f\n",
	    phGetStation(ph),phGetTime(ph)-starttime,
	    pos[0],pos[1],pos[2],
	    ang[0],ang[1],ang[2]);
  }
  
  phClose(ph);

  return 0;
}
