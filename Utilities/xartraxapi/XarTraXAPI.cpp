/*=======================================================================
        XarTraXAPI.cpp

        Chris Wedlake 

==========================================================================
Copyright

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
=======================================================================*/


/*! \file XarTraXAPI.cpp
  This file contains a complete C++ interface to the XARTRAX API.
*/
#include "XarTraXAPI.h"		
#include "vtkMatrix4x4.h"
#include "ndicapi.c"		// has to be included so that CommandVA() will work.  otherwise it misses key definitions and functions only specified in the .c file and not the .h file.

// For some reason PI is not defined so i added it for calculations
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

/*=====================================================================*/
// Core
/*=====================================================================*/
/*! \defgroup XarTraXCore XarTrax Core Functions
  These are functions that are pass the functions to the XarTraX system.  These were essentially
  taken and modified from the Atamai ndicapi.h functions with one or two lines changed to work with
  the XarTraX system.
*/

/*! \ingroup XarTraXCore
		  
	\param pol pointer to a polaris structure.
	\param format an char array or pointer to the method being called.
	\param ap va_list.
	\return a char pointer to of a message defining the result.
*/
char *xarCommandVA(polaris *pol, const char *format, va_list ap)
{
  int i, m, nc;
  unsigned int CRC16 = 0;
  int use_crc = 0;
  int in_command = 1;
  char *cp, *rp, *crp;

  cp = pol->serial_command;      // text sent to ndicapi
  rp = pol->serial_reply;        // text received from ndicapi
  crp = pol->command_reply;      // received text, with CRC hacked off
  nc = 0;                        // length of 'command' part of command 

  pol->error_code = 0;           // clear error
  cp[0] = '\0';
  rp[0] = '\0';
  crp[0] = '\0';

  // verify that the serial device was opened
  if (pol->serial_device == NDI_INVALID_HANDLE) {
    ndi_set_error(pol, NDI_OPEN_ERROR);
    return crp;
  }

  vsprintf(cp, format, ap);                   // format parameters

  CRC16 = 0;                                  // calculate CRC 
  for (i = 0; cp[i] != '\0'; i++) {
    CalcCRC16(cp[i], &CRC16);
    if (in_command && cp[i] == ':') {         // only use CRC if a ':' 
      use_crc = 1;                            //  follows the command  
    }
    if (in_command &&
	!((cp[i] >= 'A' && cp[i] <= 'Z') || 
	  (cp[i] >= '0' && cp[i] <= '9'))) {
      in_command = 0;                         // 'command' part has ended 
      nc = i;                                 // command length 
    }
  }

  if (use_crc) {
    sprintf(&cp[i], "%04X", CRC16);           // tack on the CRC 
    i += 4;
  }

  cp[i++] = '\r';                             // tack on carriage return 
  cp[i] = '\0';                               // terminate for good luck 

  // if the command is GX and thread_mode is on, we copy the reply from the thread rather than getting it directly from the Measurement System 
  if (pol->thread_mode && pol->tracking && 
      cp[0] == 'G' && cp[1] == 'X' && nc == 2) {
    int errcode = 0;

    // check that the thread is sending the GX command that we want 
    if (strcmp(cp, pol->thread_command) != 0) {
      // tell thread to start using the new GX command 
      ndiMutexLock(pol->thread_mutex);
      strcpy(pol->thread_command, cp);
      ndiMutexUnlock(pol->thread_mutex);
      // wait for the next data record to arrive (we have to throw it away) 
      if (ndiEventWait(pol->thread_buffer_event, 5000)) {
        ndi_set_error(pol, NDI_TIMEOUT);
        return crp;
      }
    }
    // there is usually no wait, because usually new data is ready 
    if (ndiEventWait(pol->thread_buffer_event, 5000)) {
      ndi_set_error(pol, NDI_TIMEOUT);
      return crp;
    }
    // copy the thread's reply buffer into the main reply buffer 
    ndiMutexLock(pol->thread_buffer_mutex);
    for (m = 0; pol->thread_buffer[m] != '\0'; m++) {
      rp[m] = pol->thread_buffer[m];
    }
    rp[m] = '\0';   // terminate string 
    errcode = pol->thread_error;
    ndiMutexUnlock(pol->thread_buffer_mutex);

    if (errcode != 0) {
      ndi_set_error(pol, errcode);
      return crp;
    }
  }
  // if the command is not a GX or thread_mode is not on, then send the command directly to the Measurement System and get a reply 
  else {
    int errcode = 0;
    int thread_mode;

    // guard against pol->thread_mode changing while mutex is locked 
    thread_mode = pol->thread_mode;

    if (thread_mode && pol->tracking) {
      // block the tracking thread while we slip this command through 
      ndiMutexLock(pol->thread_mutex);
    }

    // change  pol->tracking  if either TSTOP or TSTART is sent   
    if ((nc == 5 && strncmp(cp, "TSTOP", nc) == 0) ||
        (nc == 4 && strncmp(cp, "INIT", nc) == 0)) {
      pol->tracking = 0;
    }
    else if (nc == 6 && strncmp(cp, "TSTART", nc) == 0) {
      pol->tracking = 1;
      if (thread_mode) {
        // this will force the thread to wait until the application sends the first GX command 
        pol->thread_command[0] = '\0';
      }
    }

    // flush the input buffer, because anything that we haven't read yet is garbage left over by a previously failed command 
    ndiSerialFlush(pol->serial_device, NDI_IFLUSH);

    // send the command to the Measurement System 
    if (errcode == 0) {
      m = ndiSerialWrite(pol->serial_device, cp, i);
      if (m < 0) {
        errcode = NDI_WRITE_ERROR;
      }
      else if (m < i) {
        errcode = NDI_TIMEOUT;
      }
    }

    // read the reply from the Measurement System 
    m = 0;
    if (errcode == 0) {
      m = ndiSerialRead(pol->serial_device, rp, 2047);
      if (m < 0) {
        errcode = NDI_WRITE_ERROR;
        m = 0;
      }
      else if (m == 0) {
        errcode = NDI_TIMEOUT;
      }
      rp[m] = '\0';   // terminate string 
    }

    if (thread_mode & pol->tracking) {
      // unblock the tracking thread 
      ndiMutexUnlock(pol->thread_mutex);
    }

    if (errcode != 0) {
      ndi_set_error(pol, errcode);
      return crp;
    }
  }

  // back up to before the CRC
  //MODIFIED HERE FROM ndiCommandVA()!!!!!!
  if (use_crc)
	m -= 5;
  else 
	m -= 1;
  //MODIFIED HERE FROM ndiCommandVA()!!!!!!
  if (m < 0) {
    ndi_set_error(pol, NDI_BAD_CRC);
    return crp;
  }

  // calculate the CRC and copy serial_reply to command_reply 
  CRC16 = 0;
  for (i = 0; i < m; i++) {
    CalcCRC16(rp[i], &CRC16);
    crp[i] = rp[i];
  }

  // terminate command_reply before the CRC 
  crp[i] = '\0';           

  // read and check the CRC value of the reply 
  if (CRC16 != ndiHexToUnsignedLong(&rp[m], 4)) {
    ndi_set_error(pol, NDI_BAD_CRC);
    return crp;
  }

  // check for error code 
  if (crp[0] == 'E' && strncmp(crp, "ERROR", 5) == 0)  {
    ndi_set_error(pol, ndiHexToUnsignedLong(&crp[5], 2));
    return crp;
  }

  //----------------------------------------
  // special behavior for specific commands 

  if (cp[0] == 'T' && cp[1] == 'X' && nc == 2) { // the TX command 
    ndi_TX_helper(pol, cp, crp);
  }
  else if (cp[0] == 'G' && cp[1] == 'X' && nc == 2) { // the GX command 
    ndi_GX_helper(pol, cp, crp);
  }
  else if (cp[0] == 'C' && nc == 4 && strncmp(cp, "COMM", nc) == 0) {
    ndi_COMM_helper(pol, cp, crp);
  }
  else if (cp[0] == 'I' && nc == 4 && strncmp(cp, "INIT", nc) == 0) {
    ndi_INIT_helper(pol, cp, crp);
  }
  else if (cp[0] == 'I' && nc == 5 && strncmp(cp, "IRCHK", nc) == 0) {
    ndi_IRCHK_helper(pol, cp, crp);
  }
  else if (cp[0] == 'P' && nc == 5 && strncmp(cp, "PHINF", nc) == 0) {
    ndi_PHINF_helper(pol, cp, crp);
  }
  else if (cp[0] == 'P' && nc == 4 && strncmp(cp, "PHSR", nc) == 0) {
    ndi_PHSR_helper(pol, cp, crp);
  }
  else if (cp[0] == 'P' && nc == 5 && strncmp(cp, "PSTAT", nc) == 0) {
    ndi_PSTAT_helper(pol, cp, crp);
  }
  else if (cp[0] == 'S' && nc == 5 && strncmp(cp, "SSTAT", nc) == 0) {
    ndi_SSTAT_helper(pol, cp, crp);
  }

  // return the Measurement System reply, but with the CRC hacked off 
  return crp;
}

/*! \ingroup XarTraXCore

	\param pol pointer to a polaris structure.
	\param format an char array or pointer to the method being called.
	\param ... 
	\return a char pointer to of a message defining the result.
*/
 char *xarCommand(polaris *pol, const char *format, ...) {
  char *reply; va_list ap; va_start(ap,format);
  reply = xarCommandVA(pol, format, ap); va_end(ap); return reply; }

/*=====================================================================*/
// INITIALIZATION COMMANDS
/*=====================================================================*/
/*! \defgroup XarTraXInit XarTraX Initialization Commands
	These commands are used to initialize and close the scanner system. These 
	routines should be the first (and last) routines to be called.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXInit
  Resets the controller to default values and directs the serial break signal to the 
  Polaris system.

  \param pol a pointer to a polaris structure
  \return char pointer containing result of operation called.
*/
char * xarRESET(polaris *pol) {
	return xarCommand((pol),NULL);
}

/*! \ingroup XarTraXInit
  Initializes the XarTraX system. It is important to call this function first before any 
  further commands can take place.

  \param pol a pointer to a polaris structure
  \return char pointer containing result of operation called.
*/
char * xarINIT(polaris *pol)	{
	return xarCommand((pol),"_init:");
}


/*! \ingroup XarTraXInit
  Clears any resources that have been allocated and reloads the system firmware. 
  Typically, it is not necessarily to call this function. This function causes a soft 
  reboot and reload of the controller firmware. It may be useful if an error condition 
  has been detected on the controller.

  \param pol a pointer to a polaris structure
  \return char pointer containing result of operation called.
*/
char * xarCLOSE(polaris *pol) {
	return xarCommand((pol),"_close:");
}




int xarProbe(const char *device) {
  char init_reply[16];
  NDIFileHandle serial_port;
  int n;

  serial_port = ndiSerialOpen(device);
  if (serial_port == NDI_INVALID_HANDLE) {
    return NDI_OPEN_ERROR;
  }

  /* set comm parameters to default, but decrease timeout to 0.1s */
  if (ndiSerialComm(serial_port, 9600, "8N1", 0) < 0 ||
      ndiSerialTimeout(serial_port, 100) < 0) {
    ndiSerialClose(serial_port);
    return NDI_BAD_COMM;
  }

  /* flush the buffers (which are unlikely to contain anything) */
  ndiSerialFlush(serial_port, NDI_IOFLUSH);

  /* try to initialize ndicapi */
  if (ndiSerialWrite(serial_port, "INIT:E3A5\r", 10) < 10 ||
      ndiSerialSleep(serial_port, 100) < 0 ||
      ndiSerialRead(serial_port, init_reply, 16) <= 0 ||
      strncmp(init_reply, "OKAYA896\r", 9) != 0) {

    /* increase timeout to 5 seconds for reset */
    ndiSerialTimeout(serial_port, 5000);

    /* init failed: flush, reset, and try again */
    ndiSerialFlush(serial_port, NDI_IOFLUSH);
    if (ndiSerialFlush(serial_port, NDI_IOFLUSH) < 0 ||
	ndiSerialBreak(serial_port)) {
      ndiSerialClose(serial_port);
      return NDI_BAD_COMM;
    }

    n = ndiSerialRead(serial_port, init_reply, 16);
    if (n < 0) {
      ndiSerialClose(serial_port);
      return NDI_READ_ERROR;
    }
    else if (n == 0) {
	  ndiSerialWrite(serial_port, "RESETBE6F\r", 10);
      ndiSerialClose(serial_port);
      return NDI_TIMEOUT;
    }

    /* check reply from reset */
    if (strncmp(init_reply, "RESETBE6F\r", 10) != 0)  {
      ndiSerialClose(serial_port);
      return NDI_PROBE_FAIL;
    }

    /* try to initialize a second time */
    ndiSerialSleep(serial_port, 100);
    n = ndiSerialWrite(serial_port, "INIT:E3A5\r", 10);
    if (n < 0) {
      ndiSerialClose(serial_port);
      return NDI_WRITE_ERROR;
    }
    else if (n < 10) {
      ndiSerialClose(serial_port);
      return NDI_TIMEOUT;
    }
      
    ndiSerialSleep(serial_port, 100);
    n = ndiSerialRead(serial_port, init_reply, 16);
    if (n < 0) {
      ndiSerialClose(serial_port);
      return NDI_READ_ERROR;
    }
    else if (n == 0) {
      ndiSerialClose(serial_port);
      return NDI_TIMEOUT;
    }

    if (strncmp(init_reply, "OKAYA896\r", 9) != 0) {
      ndiSerialClose(serial_port);
      return NDI_PROBE_FAIL;
    }
  }

  /* restore things back to the way they were */
  ndiSerialClose(serial_port);

  return NDI_OKAY;
}


/*=====================================================================*/
// GENERAL COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXGeneral XarTraX General Methods
	These commands are used for general tasks that apply to all waveform modes.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXGeneral
  Read table of active position points.  Provides access to the DAC value. After
  completely configuring a motion in any of the waveform modes, this routine can be
  called to examine the actual values that were generated to define the motion profile.
  CAUTION: It is the responsibility of the user to ensure the table is adequately sized to hold all the points.

  \param pol a pointer to a polaris structure
  \param tableSelect this parameter determines which position table to return :
	- XAR_CHANNEL_X			selects X table position points
	- XAR_CHANNEL_Y			selects Y table position points
	- XAR_CHANNEL_YFLY		selects Y Flyback points (Raster Mode Only!)
  \param dataFormat This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT		Positions scaled as voltages by the calibration scaling
	- XAR_UNIT_DAC_UBITS	Raw 6760 integer value (20 bits), with the synchronous user bits appended to the most significant bits of the DAC position value. Then converted to a double.

  \return char pointer containing result of operation called.
*/
 char * xarReadPosTable(polaris *pol, int tableSelect, int dataFormat) {
	return xarCommand((pol),"_greadpostable:%d,%d",(tableSelect),(dataFormat) );
}

/*! \ingroup XarTraXGeneral
  Returns the number of points in the presently active position table.  This 
  command should be used to create storage before calling xarReadPosTable(...). 
  The table size will vary considerably, depending on the user parameters. 
  Therefore, the size of the position table must be checked after every 
  "complete" command.

  \param pol a pointer to a polaris structure
  \param TableSize integer value that is used to store the size of the table if the call is sucessful

  \return char pointer containing result of operation called.
*/
 char * xarSizePosTable(polaris *pol, int *TableSize)
{
	char temp[100];
	char * message;
	message = xarCommand(pol, "_gsizepostable:");
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%d", &temp, TableSize);
	}
	return message;
}


/*! \ingroup XarTraXGeneral
	Converts mirror error code.

  \param pol a pointer to a polaris structure
  \param error integer value representing the error that occurred

  \return message char pointer containing a string description of the error
*/
//BTX
//char * xarGetError(polaris *pol, int error) {
//		return xarCommand((pol),"_gerrorstr:%d",(error) );
//	}
//ETX

/*! \ingroup XarTraXGeneral
	Converts mirror error code.

  \param pol a pointer to a polaris structure
  \param message char pointer containing the message returned from the system representing the error that occurred

  \return message char pointer containing a string description of the error
*/
 char * xarGetError(polaris *pol, char * message) {
	int error = atoi(message);
	return xarCommand((pol),"_gerrorstr:%d",(error) );
}


/*=====================================================================*/
//SCANNER CONTROL COMMANDS
/*=====================================================================*/
/*! \defgroup XarTraXScannerControl XarTraX Scanner Control Methods
	These commands are responsible for controlling the behavior of the scanner.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXScannerControl
  This command initiates the actual execution of the waveform on the scanner. Until
  this is called, the scanner position will remained unchanged. Before calling this
  command, remember to use the appropriate complete command. After execution, it is
  possible to modify the parameters of a waveform without calling this routine again. 

  \param pol a pointer to a polaris structure

  \return char pointer containing result of operation called.
*/
 char * xarExecute(polaris * pol) {
	return xarCommand((pol),"_oexecute:");
}


/*=====================================================================*/
// CONFIGURATION COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXConfiguration XarTraX Configuration Methods
	These commands are used to configure the scanner system to match the user
	system. Before defining a waveform, the configuration should be done to ensure
	proper computation of the waveform tables. It is NOT advisable to call ANY of
	these routines during waveform execution. If a configuration parameter must be
	changed, stop the scanner before setting the new value.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXConfiguration
  This command is not for general use and should not normally be called by the user
  as it is configured automatically when xarINIT() is called. The NONE option will
  disable hardware interrupts. If this option is selected, the only waveform mode
  that can be used is DAC Direct. The function generator, Raster and Arbitrary
  waveform modes require a hardware interrupt. This option can be changed without
  modifying jumpers on the scanner system.

  \param pol a pointer to a polaris structure
  \param interruptNumber This option specifies which hardware interrupt to apply for waveform generation:
	- XAR_INTERR_NONE
	- XAR_INTERR_NUM3
	- XAR_INTERR_NUM5
	- XAR_INTERR_NUM7
	- XAR_INTERR_NUM10
	- XAR_INTERR_NUM11
	- XAR_INTERR_NUM12
	- XAR_INTERR_NUM15

  \return char pointer containing result of operation called.
*/
 char * xarCInterrupt(polaris * pol, int interruptNumber) {
	return xarCommand((pol),"_cinterrupt:%d",(interruptNumber) );
}

/*! \ingroup XarTraXConfiguration
	The system supports simultaneous motion of two scanners, designated as X and Y. The
	physical connection defines which scanner becomes X and Y. By calling
	this function, it can reverse the normal scanner connections. If the swapped option
	is selected, the scanner connected to the Y channel will behave like the X scanner and
	vice versa.

  \param pol a pointer to a polaris structure
  \param xySwap This option specifies which hardware interrupt to apply for waveform generation:
	- XAR_XYSWAP_NORM		Normal mode, X will appear on the x scanner.
	- XAR_XYSWAP_SWAPPED	Swapped mode, X will appear on the y scanner.

  \return char pointer containing result of operation called.
*/
 char * xarXYSwap(polaris * pol, int xySwap) {
	return xarCommand((pol),"_cxyswap:%d",(xySwap) );
}


/*=====================================================================*/
//SCANNER FIELD CALIBRATION COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXCalibration XarTraX Scanner Field Calibration Methods
	This set of commands permits specification of the scanner field in units of
	degrees or volts instead of the native DAC units. Once the field has been
	calibrated, the scanner system will accept position values in these units and
	automatically apply a conversion to the internal trigger values used by the
	scanner’s DAC.

	These commands do not need to be run at ever time and are only needed to be run if
	the system appears to be reporting inaccurately.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXCalibration
	Positions for the parameters highCalDegree and midCalDegree can be determined by
	direct measurement (most accurate) or it is possible to estimate it from system parameters
	(less accurate). If the user does not wish to specify any positions in degrees, this
	command can be ignored.


  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param highCalDegree This double value paramaer should reflect the position of the scanner in degrees, when the scanner as the following DAC coordinates: 60,000 for 16-bit DACs or 240,000 for 18-bit DACs.
  \param midCalDegree  This double value parameter should reflect the position of the scanner in degrees, when commanded to the following DAC coordinates: 32,768 for 16-bit DACs or 131,072 for 18-bit DACs.

  \return char pointer containing result of operation called.
*/
 char * xarCalSetDeg(polaris *pol, int xyChannelSelect, double highCalDegree, double midCalDegree)	{
	return xarCommand((pol),"_setdeg:%d,%lf,%lf",(xyChannelSelect), (highCalDegree), (midCalDegree) );
}

/*! \ingroup XarTraXCalibration
	Positions for the parameters highCalDegree and midCalDegree can be determined by
	direct measurement (most accurate) or it is possible to estimate it from system parameters
	(less accurate). If the user does not wish to specify any positions in degrees, this
	command can be ignored.


  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param highCalVolt This double value paramaer should reflect the position of the scanner in volts, when the scanner as the following DAC coordinates: 60,000 for 16-bit DACs or 240,000 for 18-bit DACs.
  \param midCalVolt  This double value parameter should reflect the position of the scanner in volts, when commanded to the following DAC coordinates: 32,768 for 16-bit DACs or 131,072 for 18-bit DACs.

  \return char pointer containing result of operation called.
*/
 char * xarCalSetVol(polaris *pol, int xyChannelSelect, double highCalVolt, double midCalVolt) { 
	return xarCommand((pol),"_setvolt:%d,%lf,%lf",(xyChannelSelect), (highCalVolt), (midCalVolt) );
}

/*! \ingroup XarTraXCalibration
	Regarding to the input parameter selectUnits, for all units, position values will be
	expressed as double precision floating point numbers. This is a natural choice for
	positions expressed in degrees or volts. On the other hand, DAC values will be converted
	to floating point numbers. To convert the resulting DAC value to an integer, the user
	should ROUND the returned value to the nearest integer.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param selectUnits This parameter defines the unit system for the returned positions.
	- XAR_UNIT_VOLT		Returns limit of field in terms of volts.
	- XAR_UNIT_DEG		Returns limit of field in terms of degrees.
	- XAR_UNIT_DAC		Returns limit of field in terms of DAC values.
  \param minimum a integer pointer that will contain the minimum value for this unit if the command was successful.
  \param maximum a integer pointer that will contain the maximum value for this unit if the command was successful.

  \return char pointer containing result of operation called.
*/
 char * xarCalLimits(polaris *pol, int xyChannelSelect, int selectUnits, double *minimum, double *maximum) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_limits:%d,%d",(xyChannelSelect), (selectUnits) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf,%lf", &temp, *minimum, *maximum);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in degrees
	and returns the equivalent position in DAC count. If parameter degree has a position that
	is outside of the field, the position will be truncated to the largest possible value and
	a warning will be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param degree A position in the field that is to be converted.
  \param equivalentDAC Stores the DAC values corresponding to the passed value in degrees.

  \return char pointer containing result of operation called.
*/
 char * xarDegToDac(polaris *pol, int xyChannelSelect, double degree, double *equivalentDAC) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_degtodac:%d,%lf",(xyChannelSelect), (degree) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *equivalentDAC);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in degrees
	and returns the equivalent position in volts. If parameter degree has a position that
	is outside of the field, the position will be truncated to the largest possible value and
	a warning will be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param degree A position in the field that is to be converted.
  \param equivalentVolt Stores the Volt values corresponding to the passed value in degrees.

  \return char pointer containing result of operation called.
*/
 char * xarDegToVolt(polaris *pol, int xyChannelSelect, double degree, double *equivalentVolt) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_degtovolt:%d,%lf",(xyChannelSelect), (degree) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *equivalentVolt);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in degrees
	and returns the equivalent position in DAC count. If parameter volt has a position that
	is outside of the field, the position will be truncated to the largest possible value and
	a warning will be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param volt A position in the field that is to be converted.
  \param equivalentDeg Stores the degree values corresponding to the passed value in Volts.

  \return char pointer containing result of operation called.
*/
 char * xarVoltToDeg(polaris *pol, int xyChannelSelect, double volt, double *equivalentDeg) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_volttodeg:%d,%lf",(xyChannelSelect), (volt) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *equivalentDeg);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in Volts
	and returns the equivalent position in degrees. If parameter volt has a position that
	is outside of the field, the position will be truncated to the largest possible value and
	a warning will be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param volt A position in the field that is to be converted.
  \param equivalentDAC Stores the DAC values corresponding to the passed value in degrees.

  \return char pointer containing result of operation called.
*/
 char * xarVoltToDac(polaris *pol, int xyChannelSelect, double volt, double *equivalentDAC) {

	char temp[100];
	char * message;
	message = xarCommand((pol),"_volttodac:%d,%lf",(xyChannelSelect), (volt) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, &equivalentDAC);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in DAC counts
	and returns the equivalent values in degrees. If parameter volt has a position that is outside
	of the field, the position will be truncated to the largest possible value and a warning will
	be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param DACvalue A position in the field that is to be converted.
  \param equivalentVolt Stores the volts values corresponding to the passed value in DAC.

  \return char pointer containing result of operation called.
*/
 char * xarDacToVolt(polaris *pol, int xyChannelSelect, double DACvalue, double *equivalentVolt) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_dactovolt:%d,%lf",(xyChannelSelect), (DACvalue) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *equivalentVolt);
	}
	return message;
}

/*! \ingroup XarTraXCalibration
	Converts values between different units. This routine accepts position values in DAC counts
	and returns the equivalent values in volts. If parameter volt has a position that is outside
	of the field, the position will be truncated to the largest possible value and a warning will
	be issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param DACvalue A position in the field that is to be converted.
  \param equivalentDeg Stores the degree values corresponding to the passed value in DAC.

  \return char pointer containing result of operation called.
*/
 char * xarDacToDeg(polaris *pol, int xyChannelSelect, double DACvalue, double *equivalentDeg) {
	char temp[100];
	char * message;
	message = xarCommand((pol),"_dactodeg:%d,%lf",(xyChannelSelect), (DACvalue) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *equivalentDeg);
	}
	return message;
}


/*=====================================================================*/
// LASER CONTROL COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXLaser XarTraX Laser Contol Methods
	This set of commands is used to turn the visible and invisible (infrared) lasers on
	and off. The commands take effect immediately and are normally used in Direct
	mode only. Arbitrary waveforms use the information tagged with each point
	location to determine the state of the lasers.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXLaser
	Immediately turns on the visible laser.

  \param pol a pointer to a polaris structure

  \return message char pointer containing result of operation called.
*/
 char * xarVisibleOn(polaris *pol) { 
	return xarCommand((pol),"_VisibleLaserOn");
}

/*! \ingroup XarTraXLaser
	Immediately turns off the visible laser. Make sure if you are turning off
	the visibile laser that the infrared laser is off also as it is good practice
	so that no one accidently looks into it not knowing it is on.

  \param pol a pointer to a polaris structure

  \return message char pointer containing result of operation called.
*/
 char * xarVisibleOff(polaris *pol) {
	return xarCommand((pol),"_VisibleLaserOff");
}

/*! \ingroup XarTraXLaser
	Immediately turns on the invisible (infrared) laser.

  \param pol a pointer to a polaris structure

  \return message char pointer containing result of operation called.
*/
 char * xarInfraOn(polaris *pol) {
	return xarCommand((pol),"_LaserOn");
}

/*! \ingroup XarTraXLaser
	Immediately turns off the invisible (infrared) laser.

  \param pol a pointer to a polaris structure

  \return message char pointer containing result of operation called.
*/
 char * xarInfraOff(polaris *pol)	{
	return xarCommand((pol),"_LaserOff");
}


/*=====================================================================*/
//FUNCTION GENERATOR WAVEFORM COMMANDS
/*=====================================================================*/
/*! \defgroup XarTraXfunctionwaveform XarTraX Function Generator Waveform Methods
	This set of commands emulates a simple function generator. Using these calls, it
	is possible to produce motion profiles such sine waves, square waves and
	triangular waves. To make the profiles periodic, the scanner system repeats the
	same position table. Consequently, a single cycle of a waveform is placed in the
	data table and the executed repeatedly. The period of one cycle equals the
	number of points in the table times the position update rate.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXfunctionwaveform
	Emulates some common periodic waveforms. This command defines which waveform type to be emulated.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X		Causes this command to operate on the X scanner channel.
	- XAR_CHANNEL_Y		Causes this command to operate on the Y scanner channel.
  \param waveformType Defines the function generator waveform that will be created:
	- XAR_FG_TYPE_SQR	Square Wave or Pulse generator (depending on duty cycle)
	- XAR_FG_TYPE_SIN	Sine Wave
	- XAR_FG_TYPE_SIN	Saw tooth or Triangle wave (depending on duty cycle)
  \return char pointer containing result of operation called.
*/
 char * xarFType(polaris * pol, int xyChannelSelect, int waveformType) {
		return xarCommand((pol),"_ftype:%d,%d",(xyChannelSelect), (waveformType) );
}

/*! \ingroup XarTraXfunctionwaveform
	After selecting waveform type this can be used to adjust its duty cycle.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param dutyCycle Defines the waveform duty-cycle as a percentage. Thus, a 50% duty cycle translates into equal times spent on each section of the waveform. In the case of a pulse waveform, a 50% duty cycle produces a square wave. Similarly a saw tooth waveform of 50% duty cycle is a triangle wave. The range of this value is 0 to 100 %. This parameter is ignored when the waveform type is a sine wave. 
  \param actualDutyCycle Stores the actual duty cycle that can be achieved with the present configuration. The duty cycle has a limited range (0 to 100 %). Also the number of position points in the waveform affects the achievable duty cycle. Consequently, the actual duty cycle is affected by the parameter passed to xarSizePosTable(). For example, if only four points are used to create the waveform, the duty cycle is limited to 0,25,50,75 or 100%.

  \return char pointer containing result of operation called.
*/
 char * xarFDutyCycleA(polaris *pol, int xyChannelSelect, double dutyCycle, double *actualDutyCycle)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_fdutycycle:%d,%lf",(xyChannelSelect), (dutyCycle) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualDutyCycle);
	}
	return message;
}

/*! \ingroup XarTraXfunctionwaveform
	After selecting waveform type this can be used to adjust its duty cycle.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param dutyCycle Defines the waveform duty-cycle as a percentage. Thus, a 50% duty cycle translates into equal times spent on each section of the waveform. In the case of a pulse waveform, a 50% duty cycle produces a square wave. Similarly a saw tooth waveform of 50% duty cycle is a triangle wave. The range of this value is 0 to 100 %. This parameter is ignored when the waveform type is a sine wave. 

  \return char pointer containing result of operation called.
*/
 char * xarFDutyCycle(polaris *pol, int xyChannelSelect, double dutyCycle)
 { return xarCommand((pol),"_fdutycycle:%d,%lf",(xyChannelSelect), (dutyCycle) ); }


/*! \ingroup XarTraXfunctionwaveform
	Specifies the number of points in the waveform table.

  \param pol a pointer to a polaris structure
  \param tableSize defines The number of position samples in the waveform. This parameter and the time between DAC updates define the frequency of the resulting waveform.
  \param actualTableSize Stores the actual table size that can be implemented. The table size is limited to avoid using too much RAM memory.

  \return error integer value representing the result of the operation called.
*/
 char * xarFTableSizeA(polaris *pol, int tableSize, int *actualTableSize)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_ftablesize:%d",(tableSize) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%d", &temp, *actualTableSize);
	}
	return message;
}

/*! \ingroup XarTraXfunctionwaveform
	Specifies the number of points in the waveform table.

  \param pol a pointer to a polaris structure
  \param tableSize defines The number of position samples in the waveform. This parameter and the time between DAC updates define the frequency of the resulting waveform.

  \return error integer value representing the result of the operation called.
*/
 char * xarFTableSize(polaris *pol, int tableSize)
 { return xarCommand((pol),"_ftablesize:%d",(tableSize) ); }


/*! \ingroup XarTraXfunctionwaveform
  Specifies the phase difference in between the X and Y channel waveforms.

  \param pol a pointer to a polaris structure
  \param phaseDifference defines the phase shift (in degrees) between the X and Y waveforms in degrees. This parameter defines the relative phase shift between the X and Y channels. This input can range between: -180 < phase < +180.
  \param actualPhaseDifference Stores the actual phase difference that can be implemented. The resolution of the phase difference is affected by the number of position points in the waveform (i.e. table size) For Example, if only 4 values exist in the position table, the only achievable phase shifts are -180, -90, 0, +90 and +180 (same as -180).

  \return char pointer containing result of operation called.
*/
 char * xarFPhaseA(polaris *pol, double phaseDifference, double *actualPhaseDifference)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_fphase:%lf",(phaseDifference) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualPhaseDifference);
	}
	return message;
}

 /*! \ingroup XarTraXfunctionwaveform
  Specifies the phase difference in between the X and Y channel waveforms.

  \param pol a pointer to a polaris structure
  \param phaseDifference defines the phase shift (in degrees) between the X and Y waveforms in degrees. This parameter defines the relative phase shift between the X and Y channels. This input can range between: -180 < phase < +180.

  \return char pointer containing result of operation called.
*/
 char * xarFPhase(polaris *pol, double phaseDifference)
 { return xarCommand((pol),"_fphase:%lf",(phaseDifference) ); }


/*! \ingroup XarTraXfunctionwaveform
  Specifies the period between DAC updates.

  \param pol a pointer to a polaris structure
  \param time defines the period (in milliseconds) between DAC updates.
  \param actualTime Stores the nearest achievable DAC update rate (in millisecond).

  \return char pointer containing result of operation called.
*/
 char * xarFTimebptsA(polaris *pol, double time, double *actualTime)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_ftimebpts:%d", (time));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualTime);
	}
	return message;
}

/*! \ingroup XarTraXfunctionwaveform
  Specifies the period between DAC updates.

  \param pol a pointer to a polaris structure
  \param time defines the period (in milliseconds) between DAC updates.

  \return char pointer containing result of operation called.
*/
 char * xarFTimebpts(polaris *pol, double time)
 { return xarCommand((pol),"_ftimebpts:%d", (time)); }


/*! \ingroup XarTraXfunctionwaveform
  Sets the means and excursion size of the function generator waveform.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param selectUnits This parameter defines the unit system for the returned positions.
	- XAR_UNIT_VOLT		Returns limit of field in terms of volts.
	- XAR_UNIT_DEG		Returns limit of field in terms of degrees.
	- XAR_UNIT_DAC		Returns limit of field in terms of DAC values.
  \param amplitude this parameter defines the distance from the center of the waveform to the peak value.
  \param offset this value sets the middle point of the waveform. It is possible to translate the waveform to a new location in the field by changing this value, without affecting its size.

  \param actualAmplitude Stores an excessive amplitude value could exceed the field size. If clipping occurs on both sides of the field, the amplitude is automatically adjusted and the resulting amplitude is returned by this parameter. The returned value will have the same unit as the input.
  \param actualOffset Stores an excessive offset value could exceed the field size. If clipping occurs on both sides of the field, the offset is automatically adjusted and the resulting offset is returned by this parameter. The returned value will have the same unit as the input. 

  \return char pointer containing result of operation called.
*/
 char * xarFAmpOffA(polaris *pol, int xyChannelSelect, int selectUnits, double amplitude, double offset, double * actualAmplitude, double *actualOffset)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_fampoff:%d,%d,%lf,%lf", (xyChannelSelect),(selectUnits), (amplitude), (offset) );
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf,%lf", &temp, *actualAmplitude, *actualOffset);
	}
	return message;
}

/*! \ingroup XarTraXfunctionwaveform
  Sets the means and excursion size of the function generator waveform.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect This parameter is used to select the channel that is affected by the command:
	- XAR_CHANNEL_X			Causes this command to operate on the X scanner channel
	- XAR_CHANNEL_Y			Causes this command to operate on the Y scanner channel
  \param selectUnits This parameter defines the unit system for the returned positions.
	- XAR_UNIT_VOLT		Returns limit of field in terms of volts.
	- XAR_UNIT_DEG		Returns limit of field in terms of degrees.
	- XAR_UNIT_DAC		Returns limit of field in terms of DAC values.
  \param amplitude this parameter defines the distance from the center of the waveform to the peak value.
  \param offset this value sets the middle point of the waveform. It is possible to translate the waveform to a new location in the field by changing this value, without affecting its size.

  \return char pointer containing result of operation called.
*/
 char * xarFAmpOff(polaris *pol, int xyChannelSelect, int selectUnits, double amplitude, double offset)
 { return xarCommand((pol),"_fampoff:%d,%d,%lf,%lf", (xyChannelSelect),(selectUnits), (amplitude), (offset) ); }

/*! \ingroup XarTraXfunctionwaveform
  selects a Table size and Time between points that best matches the passed frequency value.

  \param pol a pointer to a polaris structure
  \param desiredFrequency the desired frequency in Hertz.

  \param bestFitFrequency Stores to indicate how close xarFFreq was to matching the desired frequency, this value reports the exact frequency that the optimized parameters would produce.
  \param optimizedTableSize Stores the optimized table size that, in conjunction with the time between DAC points, best fits the passed frequency value. 
  \param optimizedTimebptsmsec Stores the optimized time between DAC updates that, in conjunction with the returned table size, best fits the passed frequency value. (in milliseconds).

  \return char pointer containing result of operation called.
*/
 char * xarFFreqA(polaris *pol, int desiredFrequency, double *bestFitFrequency, double *optimizedTableSize, double *optimizedTimebptsmsec)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_ffreq:%lf", (desiredFrequency));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf,%lf,%lf", &temp, *bestFitFrequency, *optimizedTableSize, *optimizedTimebptsmsec);
	}
	return message;
}

 /*! \ingroup XarTraXfunctionwaveform
  selects a Table size and Time between points that best matches the passed frequency value.

  \param pol a pointer to a polaris structure
  \param desiredFrequency the desired frequency in Hertz.

  \return char pointer containing result of operation called.
*/
 char * xarFFreq(polaris *pol, int desiredFrequency)
 { return xarCommand((pol),"_ffreq:%lf", (desiredFrequency)); }

/*! \ingroup XarTraXfunctionwaveform
	computes the position (waveform) table. This step is necessary before waveform execution.

  \param pol a pointer to a polaris structure

  \return char pointer containing result of operation called.
*/
 char * xarFComplete(polaris *pol) {
	return xarCommand((pol),"_fcomplete:");
}


/*=====================================================================*/
//ARBITRARY WAVEFORM MODE COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXarbitrarywaveform XarTraX Arbitrary Generator Waveform Methods
	This mode allows the user to define every position value in a waveform. The
	user has complete control of the resulting waveform. The period between
	scanner position updates can be adjusted, but remains constant during a
	waveform execution. In addition to the mirror positions, the state of the infrared
	and visible lasers is included in the waveform table. When a position is
	encountered where the state of the lasers change, the affected laser is turned on
	or off just prior to moving to the new position. Time lags in large motions may
	affect the perception of when the beam actually appears to switch on or off.

	The sequence of commands to cause an arbitrary waveform to display typically
	takes the following form:
	-# xarARepeat: sets the number of times to repeat display of the waveform
	-# xarAPosition: transfers the data table to the controller over the serial link
	-# xarAtimebpts: sets the update frequency of the display
	-# xarAcomplete: signals the finish of the data load
	-# xarSizePosTable: confirm the size of the data table is reasonable
	-# xarReadPosTable: transfers the x data into the display buffers on the controller
	-# xarReadPosTable: transfers the y data into the display buffers on the controllor
	-# xarExecute: executes the waveform

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXarbitrarywaveform
	supplies a table of position values in units of volts, degrees or DAC counts. The
	table specifies the waveform at regular intervals over the motion profile. The spacing
	between positions is set using xarATimebpts: command. The wave form execution begins when
	the xarExecute command is issued.

  \param pol a pointer to a polaris structure
  \param Start Double value of the start point of the frame of data
  \param End Double value of the end point of the frame of data
  \param selectUnits This parameter defines the unit system for the returned positions.
	- XAR_UNIT_VOLT		Returns limit of field in terms of volts.
	- XAR_UNIT_DEG		Returns limit of field in terms of degrees.
	- XAR_UNIT_DAC		Returns limit of field in terms of DAC values.
  \param data Defines every position and laser status in the waveform.  See APosData Structure for how it should be laid out in a string format.  eg. -2.000000,1.977953,0,1,     1.977974,1.977953,0,1,      1.977974,-1.999977,0,1 .....
  
  \return char pointer containing result of operation called.
*/
 char * xarAPosition(polaris *pol, int Start, int End, int selectUnits, char * data)  {
	return xarCommand((pol), "_aposition:%d,%d,%d%s", (Start), (End), (selectUnits), (data));
}

/*! \ingroup XarTraXarbitrarywaveform
	The arbitrary waveform mode requires a table of position values to define the motion
	profile. This routine accepts integer DAC values, including user bits. The wave form
	execution begins when the xarExecute command is issued.

  \param pol a pointer to a polaris structure
  \param xPositionTable Integer value of the start point of the frame of data
  \param yPositionTable Integer value of the end point of the frame of data
  \param data Defines every position and laser status in the waveform.  See APosData2 Structure for how it should be laid out in a string format.  eg. -2,1,0,1,     1,1,0,1,      1,-1,0,1 .....
  
  \return char pointer containing result of operation called.
*/
 char * xarAIPosition(polaris *pol, int xPositionTable, int yPositionTable, char * data) {
	return xarCommand((pol), "_a_iposition:%d,%d,%s", (xPositionTable), (yPositionTable), (data));
}

/*! \ingroup XarTraXarbitrarywaveform
	The arbitrary waveform table can be repeated using this command to specify the number of times. The ErrorCode is returned only when the waveform has been executed the specified number of times.

  \param pol a pointer to a polaris structure
  \param num define the number of times to repeat the waveform table before halting. There is no limit on this value as long as it is within the range of an unsigned integer
  
  \return char pointer containing result of operation called.
*/
 char * xarARepeat(polaris *pol, unsigned num) {
	return xarCommand((pol),"_arepeat:%u", (num));
}

/*! \ingroup XarTraXarbitrarywaveform
  Specifies the period between DAC updates.

  \param pol a pointer to a polaris structure
  \param time defines the period (in milliseconds) between DAC updates.
  \param actualTime Stores the nearest achievable DAC update rate (in millisecond).

  \return char pointer containing result of operation called.
*/
 char * xarATimebptsA(polaris *pol, double time, double *actualTime)
{
	char temp[100];
	char * message;
	message = xarCommand((pol),"_atimebpts:%lf", (time));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, actualTime);
	}
	return message;
}

/*! \ingroup XarTraXarbitrarywaveform
  Specifies the period between DAC updates.

  \param pol a pointer to a polaris structure
  \param time defines the period (in milliseconds) between DAC updates.

  \return char pointer containing result of operation called.
*/
 char * xarATimebpts(polaris *pol, double time)
 { return xarCommand((pol),"_atimebpts:%lf", (time)); }

/*! \ingroup XarTraXarbitrarywaveform
  selects a Table size and Time between points that best matches the passed frequency value.

  \param pol a pointer to a polaris structure
  \param desiredPeriod the desired frequency in Hertz.

  \param bestFitPeriod this value reports the exact period that the optimized parameters will produce in order to indicate how close the routine was to matching the desired period.
  \param optimizedTableSize Stores the optimized table size that, in conjunction with the time between DAC points, best fits the passed frequency value. 
  \param optimizedTimebptsmsec Stores the optimized time between DAC updates that, in conjunction with the returned table size, best fits the passed frequency value. (in milliseconds).

  \return char pointer containing result of operation called.
*/
 char *  xarAPeriodA(polaris *pol, int desiredPeriod, double *bestFitPeriod, double *optimizedTableSize, double *optimizedTimebptsmsec)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_aperiod:%lf", (desiredPeriod));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf,%lf,%lf", &temp, *bestFitPeriod, *optimizedTableSize, *optimizedTimebptsmsec);
	}
	return message;
}

/*! \ingroup XarTraXarbitrarywaveform
  selects a Table size and Time between points that best matches the passed frequency value.

  \param pol a pointer to a polaris structure
  \param desiredPeriod the desired frequency in Hertz.

  \return char pointer containing result of operation called.
*/
 char *  xarAPeriod(polaris *pol, int desiredPeriod)
 { return xarCommand((pol), "_aperiod:%lf", (desiredPeriod)); }

/*! \ingroup XarTraXarbitrarywaveform
  is called to calculate and prepare the position (waveform) table after
  specifying all the desired parameters. This step is necessary before waveform execution.

  \param pol a pointer to a polaris structure

  \return char pointer containing result of operation called.
*/
 char * xarAComplete(polaris *pol) {
	return xarCommand((pol),"_acomplete:");
}

/*! \ingroup XarTraXarbitrarywaveform
   this is used to setup the environment and call the appropriate API commands to make a laser projection. Given the paramaters,
   it will load and run the projection.

  \param pol		a pointer to a polaris structure
  \param filename	char pointer to the name of the file to be loaded.
  \param time		xarATimebpts() time value
  \param repeats	number of times to repeat the pattern.
  \param maxAngle	maximum angle at which the image should be projected at. (size based on distance)


  \return char pointer containing result of operation called.
*/
int xarALaserProject(polaris *pol, char *filename, double time, unsigned repeats, double maxAngle)
{
	double alpha[2000];
	double beta[2000];
	unsigned i;
	FILE *fileIN;
	int tableSize;
	double actualDump;
	char * message;
	MaxMinValue * mmv = new MaxMinValue();

	unsigned TotalPoints;

	// open the data file
	if (xarLoadFile(filename, mmv, TotalPoints, false))
		return XAR_FILEERROR; //error_value?
	else
		fileIN = fopen(filename, "r");

  	message = xarARepeat(pol, repeats);  // set the repeating times
	if(atoi(message))
		return atoi(message);

	// parse in the coordinates data into variable alpha & beta
	for(i=0; i<TotalPoints; i++){
		getNextPoint(&alpha[i], &beta[i], mmv, maxAngle, fileIN);
	}
	fclose(fileIN);
	
	message = xarUploadDataTOSBC(pol, XAR_UNIT_DEG, alpha, beta, TotalPoints); // upload the data into the system
	if(atoi(message))
		return atoi(message);
	message = xarATimebptsA(pol, time, &actualDump);
	if(atoi(message))
		return atoi(message);
	message = xarAComplete(pol);												 // acknowledge the system
	if(atoi(message))
		return atoi(message);
	message = xarSizePosTable(pol, &tableSize);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_X, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_Y, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarExecute(pol);
	if(atoi(message))
		return atoi(message);
	// mirror command session ends here 
	return XAR_OKAY;
}

/*! \ingroup XarTraXarbitrarywaveform
   this is used to setup the environment and call the appropriate API commands to make a laser projection. Given the paramaters,
   it will load and run the projection.  This uses a file that has an X-Angle Y-Angle visibleOn infraOn to draw the appropriate
   representation

  \param pol		a pointer to a polaris structure
  \param filename	char pointer to the name of the file to be loaded.
  \param time		xarATimebpts() time value
  \param repeats	number of times to repeat the pattern.

  \return char pointer containing result of operation called.
*/
int xarALaserProjectDeg(polaris *pol, char *filename, double time, unsigned repeats)
{
	FILE *fileIN;
	int tableSize;
	double actualDump;
	char * message;

	// open the data file
	if (!(fileIN = fopen(filename, "r")))
		return XAR_FILEERROR; //error_value?
	
	message = xarARepeat(pol, repeats);  // set the repeating times
	if(atoi(message))
		return atoi(message);



	int data_count,	total_count=0,end = 0, start=0;
	char szX[50], szY[50], szInfra[50], szVisible[50];
	char readline[150];
	char data[1024], temp[75];
	while (fgets(readline, 80, fileIN) != NULL) {
			sscanf(readline, "%s\t%s\t%s\t%s", &szX, &szY, &szVisible,&szInfra);
			xarAPositionBuildCommand(data_count,temp,atof(szX),atof(szY),atoi(szVisible),atoi(szInfra));
			if (total_count==0)
				strcpy(data, temp);
			else
				strcat(data, temp);
			total_count+=data_count;
			end++;

			if (total_count >= 900) {
				message = xarAPosition(pol, start, end, XAR_UNIT_DEG, data);
				if(atoi(message))
					return atoi(message);
				start=end;
				total_count=0;
			}
	}
	if (total_count != 0) {
		message = xarAPosition(pol, start, end, XAR_UNIT_DEG, data);
		if(atoi(message))
			return atoi(message);

		start=end;
		total_count=0;
	}

	fclose(fileIN);

	message = xarATimebptsA(pol, time, &actualDump);
	if(atoi(message))
		return atoi(message);
	message = xarAComplete(pol);												 // acknowledge the system
	if(atoi(message))
		return atoi(message);
	message = xarSizePosTable(pol, &tableSize);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_X, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_Y, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarExecute(pol);
	if(atoi(message))
		return atoi(message);
	// mirror command session ends here 
	return XAR_OKAY;
}

/*! \ingroup XarTraXarbitrarywaveform
   this is used to setup the environment and call the appropriate API commands to make a laser projection. Given the paramaters,
   it will load and run the projection.  This uses a file that has an X-Position Y-Position Z-Position visibleOn infraOn to draw
   the appropriate representation

  \param pol		a pointer to a polaris structure
  \param filename	char pointer to the name of the file to be loaded.
  \param time		xarATimebpts() time value
  \param repeats	number of times to repeat the pattern.
  \param _OHAT		unit vector of the laser in the polaris space
  \param transform  4x4 array containing the file (original) to projection (target) transformation.

  \return char pointer containing result of operation called.
*/
int xarALaserProjectXYZ(polaris *pol, char *filename, double time, unsigned repeats, double _OHAT[3], double transform[4][4], vtkMatrix4x4 * worldMatrix)
{
	FILE *fileIN;
	int tableSize;
	double actualDump;
	char * message;

	// open the data file
	if (!(fileIN = fopen(filename, "r")))
		return XAR_FILEERROR; //error_value?
	
	message = xarARepeat(pol, repeats);  // set the repeating times
	if(atoi(message))
		return atoi(message);

	int data_count,	total_count=0,end = 0, start=0;
	char szX[50], szY[50], szZ[50], szInfra[50], szVisible[50];
	char readline[150];
	char data[1024], temp[75];
	double X,Y,Z;
	double angleX, angleY;
	bool OKtoRead = false;
	while (fgets(readline, 80, fileIN) != NULL) {
		sscanf(readline, "%s\t%s\t%s\t%s\t%s", &szX, &szY, &szZ, &szVisible,&szInfra);
		if (OKtoRead == false) {
			if (readline[0] == '#') {
				OKtoRead = true;
			}
			continue;
		}
		X = atof(szX); 
		Y = atof(szY); 
		Z = atof(szZ);
		xarPositionToAngle(	X*transform[0][0]+ Y*transform[0][1]+ Z*transform[0][2]+ transform[0][3],
							X*transform[1][0]+ Y*transform[1][1]+ Z*transform[1][2]+ transform[1][3],
							X*transform[2][0]+ Y*transform[2][1]+ Z*transform[2][2]+ transform[2][3],
							_OHAT, angleX, angleY,worldMatrix);
			xarAPositionBuildCommand(data_count,temp,angleX,angleY,atoi(szVisible),atoi(szInfra));
		if (total_count==0)
			strcpy(data, temp);
		else
			strcat(data, temp);
		total_count+=data_count;
		end++;
			if (total_count >= 900) {
			message = xarAPosition(pol, start, end, XAR_UNIT_DEG, data);
			if(atoi(message))
				return atoi(message);
			start=end;
			total_count=0;
		}
	}
	fclose(fileIN);
	if (total_count != 0) {
		message = xarAPosition(pol, start, end, XAR_UNIT_DEG, data);
		if(atoi(message))
			return atoi(message);

		start=end;
		total_count=0;
	}

	message = xarATimebptsA(pol, time, &actualDump);
	if(atoi(message))
		return atoi(message);
	message = xarAComplete(pol);												 // acknowledge the system
	if(atoi(message))
		return atoi(message);
	message = xarSizePosTable(pol, &tableSize);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_X, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarReadPosTable(pol, XAR_CHANNEL_Y, XAR_UNIT_DEG);
	if(atoi(message))
		return atoi(message);
	message = xarExecute(pol);
	if(atoi(message))
		return atoi(message);
	// mirror command session ends here 
	return XAR_OKAY;
}


/*=====================================================================*/
//DAC DIRECT MODE COMMANDS
/*=====================================================================*/
/*! \defgroup XarTraXDirect XarTraX Direct Mode Methods
	This mode permits direct (untimed) updates of the scanner position. Although
	much simpler to use than the other modes, the DAC direct mode uses the same
	command sequence as the other modes. Normally, direct mode commands
	follow the following sequence:
	-# xarDPosition to set the x mirror position
	-# xarDPosition to set the y mirror position
	-# xarDComplete to finish
	-# xarExecute	to cause the mirrors to move to the new position

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXDirect
	supplies a table of position values in units of volts, degrees or DAC counts. The
	table specifies the waveform at regular intervals over the motion profile. The spacing
	between positions is set using xarATimebpts: command. The wave form execution begins when
	the xarExecute command is issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect	this parameter determines which position table to return :
	- XAR_CHANNEL_X			selects X table position points
	- XAR_CHANNEL_Y			selects Y table position points
  \param selectUnits		This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param position			The position value.
  \param visible		
  	- XAR_LASER_ON			Sets the visible laser on
	- XAR_LASER_OFF			Sets the visible laser on
  \param invisible
  	- XAR_LASER_ON			Sets the infrared laser on
	- XAR_LASER_OFF			Sets the infrared laser off

  \return char pointer containing result of operation called.
*/
 char * xarDPosition(polaris * pol, int xyChannelSelect,int selectUnits, double position, int visible, int invisible) {
	return xarCommand((pol),"_dposition:%d,%d,%lf,%d,%d", (xyChannelSelect), (selectUnits), (position), (visible), (invisible));
}

 /*! \ingroup XarTraXDirect
	supplies a table of position values in units of volts, degrees or DAC counts. The
	table specifies the waveform at regular intervals over the motion profile. The spacing
	between positions is set using xarATimebpts: command. The wave form execution begins when
	the xarExecute command is issued.

  \param pol a pointer to a polaris structure
  \param xyChannelSelect	this parameter determines which position table to return :
	- XAR_CHANNEL_X			selects X table position points
	- XAR_CHANNEL_Y			selects Y table position points
  \param selectUnits		This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param position			The position value.
  \param actualPosition		Stores the actual position that the laser has moved too.
  \param visible		
  	- XAR_LASER_ON			Sets the visible laser on
	- XAR_LASER_OFF			Sets the visible laser on
  \param invisible
  	- XAR_LASER_ON			Sets the infrared laser on
	- XAR_LASER_OFF			Sets the infrared laser off

  \return char pointer containing result of operation called.
*/
 char * xarDPositionA(polaris *pol, int xyChannelSelect, int selectUnits, double position, double *actualPosition,  int visible, int invisible) {
	char temp[256];
	char * message;

	message = xarCommand(pol,"_dposition:%d,%d,%lf,%d,%d", (xyChannelSelect), (selectUnits), (position), (visible), (invisible));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, actualPosition);
	}

	return message;
}

/*! \ingroup XarTraXDirect
   Convert the points to degree format based on the Max-Min values of X and Y and the Maximum Angle

  \param pol a pointer to a polaris structure
  \param XValue				value to set mirror to in the X_CHANNEL
  \param YValue				value to set mirror to in the Y_CHANNEL
  \param actualXPosition	actual value to mirror was set to in the X_CHANNEL
  \param actualYPosition	actual value to mirror was set to in the Y_CHANNEL

  \return char pointer containing result of operation called.
*/
 char * xarSetDegPosA(polaris *pol, double XValue, double YValue, double *actualXPosition, double *actualYPosition) {
		char * message;
		message = xarDPositionA(pol, XAR_CHANNEL_X, XAR_UNIT_DEG, XValue, actualXPosition, XAR_LASER_ON, XAR_LASER_ON);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarDPositionA(pol, XAR_CHANNEL_Y, XAR_UNIT_DEG, YValue, actualYPosition, XAR_LASER_ON, XAR_LASER_ON);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarDComplete(pol);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarExecute(pol);

		return message;
 }
 



/*! \ingroup XarTraXDirect
   Convert the points to degree format based on the Max-Min values of X and Y and the Maximum Angle.

  \param pol a pointer to a polaris structure
  \param XValue				value to set mirror to in the X_CHANNEL
  \param YValue				value to set mirror to in the Y_CHANNEL

  \return char pointer containing result of operation called.
*/
 char * xarSetDegPos(polaris *pol, double XValue, double YValue) {
		char * message;
		message = xarDPosition(pol, XAR_CHANNEL_X, XAR_UNIT_DEG, XValue, XAR_LASER_ON, XAR_LASER_ON);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarDPosition(pol, XAR_CHANNEL_Y, XAR_UNIT_DEG, YValue, XAR_LASER_ON, XAR_LASER_ON);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarDComplete(pol);
		if (atoi(message) != XAR_OKAY)
			return message;
		message = xarExecute(pol);

		return message;
}

/*! \ingroup XarTraXDirect
	After specifying the scanner positions under the DAC direct mode this is called
	to prepare for execution on the scanner.
  \param pol a pointer to a polaris structure

  \return char pointer containing result of operation called.
*/
 char * xarDComplete(polaris *pol) {
	return xarCommand((pol),"_dcomplete:");
}


/*=====================================================================*/
//RASTER MODE COMMANDS 
/*=====================================================================*/
/*! \defgroup XarTraXRaster XarTraX Raster Mode Methods
	This waveform mode produces an efficient 2D sweep of a user defined
	rectangular area. The computation of the full raster is rather involved, so the
	individual parameter entry command performs only very simple range checking.

	To determine if an error occured, pass the return message from xarCommand into xarGetError(polaris, message)
*/

/*! \ingroup XarTraXRaster
	supplies a table of position values in units of volts, degrees or DAC counts. The
	table specifies the waveform at regular intervals over the motion profile. The spacing
	between positions is set using xarATimebpts: command. The wave form execution begins when
	the xarExecute command is issued.

  \param pol a pointer to a polaris structure
  
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param xstart				The first position of the active X-field.
 
  \return char pointer containing result of operation called.
*/
 char * xarRXStart(polaris *pol, int selectUnits, double xstart) {
	return xarCommand((pol), "_rxstart:%d,%lf", (selectUnits), (xstart));
}

/*! \ingroup XarTraXRaster
	supplies a table of position values in units of volts, degrees or DAC counts. The
	table specifies the waveform at regular intervals over the motion profile. The spacing
	between positions is set using xarATimebpts: command. The wave form execution begins when
	the xarExecute command is issued. 

  \param pol a pointer to a polaris structure
  
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param ystart				The first position of the active Y-field.
 
  \return char pointer containing result of operation called.
*/
 char * xarRYStart(polaris *pol, int selectUnits, double ystart) {
	return xarCommand((pol), "_rystart:%d,%lf", (selectUnits), (ystart));
}

/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections.

  \param pol a pointer to a polaris structure
  \param timebptsMsec defines the period (in milliseconds) between DAC updates.
  \param rampPts the number of ramp points defines the time period in a single sweep of the raster.
  \param recovPts defines the number of DAC positions that are allocated for the transition to the next line of raster. This period includes both the cycloidal flyback and the scanner settle time.
  \param actualTimebptsmsec Stores the optimized time between DAC updates that, in conjunction with the returned table size, best fits the passed frequency value. (in milliseconds).

  \return char pointer containing result of operation called.
*/
 char * xarRPeriodA(polaris *pol, double timebptsMsec, int rampPts, int recovPts, double *actualTimebptsmsec)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_rperiod:%lf,%d,%d", (timebptsMsec), (rampPts), (recovPts));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualTimebptsmsec);
	}
	return message;
}

/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections. This is without returning the actual possible values

  \param pol a pointer to a polaris structure
  \param timebptsMsec defines the period (in milliseconds) between DAC updates.
  \param rampPts the number of ramp points defines the time period in a single sweep of the raster.
  \param recovPts defines the number of DAC positions that are allocated for the transition to the next line of raster. This period includes both the cycloidal flyback and the scanner settle time.

  \return char pointer containing result of operation called.
*/
 char * xarRPeriod(polaris *pol, double timebptsMsec, int rampPts, int recovPts)
 { return xarCommand((pol), "_rperiod:%lf,%d,%d", (timebptsMsec), (rampPts), (recovPts)); }


/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections.

  \param pol a pointer to a polaris structure
  \param settleMsec defines the scanner settling time in milliseconds. This value can be 0.

  \param actualsettleMsec Stores the actual settle period that is possible (absolute range checking only)

  \return char pointer containing result of operation called.
*/
 char * xarRSettleA(polaris *pol, double settleMsec, double *actualsettleMsec)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_rsettle:%lf", (settleMsec));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualsettleMsec);
	}
	return message;
}

/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections. This is without returning the actual possible values

  \param pol a pointer to a polaris structure
  \param settleMsec defines the scanner settling time in milliseconds. This value can be 0.

  \return char pointer containing result of operation called.
*/
 char * xarRSettle(polaris *pol, double settleMsec)
 { return xarCommand((pol), "_rsettle:%lf", (settleMsec)); }

 
/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections.

  \param pol a pointer to a polaris structure
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param xStepSize defines the DAC increment that is added at every X position step in the linear ramp.

  \param actualXStepSize Stores the actual DAC increment (absolute range checking only). This value is returned with the same units as the input.

  \return char pointer containing result of operation called.
*/
 char * xarRNXStepA(polaris *pol, int selectUnits, double xStepSize, double *actualXStepSize)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_rnxstep:%d%lf", (selectUnits), (xStepSize));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualXStepSize);
	}
	return message;
}

/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections. This is without returning the actual possible values

  \param pol a pointer to a polaris structure
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param xStepSize defines the distance between the lines in the X field. This value is added to X between lines. XStepSize can be in DAC counts, volts and dregrees.

  \return char pointer containing result of operation called.
*/
 char * xarRNXStep(polaris *pol, int selectUnits, double xStepSize)
 { return xarCommand((pol), "_rnxstep:%d%lf", (selectUnits), (xStepSize)); }



/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections.

  \param pol a pointer to a polaris structure
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param yStepSize defines the distance between the lines in the Y field. This value is added to Y between lines. YStepSize can be in DAC counts, volts and dregrees.

  \param actualYStepSize Stores the actual DAC increment (absolute range checking only). This value is returned with the same units as the input.

  \return char pointer containing result of operation called.
*/
 char * xarRNYStepA(polaris *pol, int selectUnits, double yStepSize, double *actualYStepSize)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_rnystep:%d%lf", (selectUnits), (yStepSize));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualYStepSize);
	}
	return message;
}

/*! \ingroup XarTraXRaster
  specifies the time between DAC updates (in milliseconds) and number of points in the two main raster sections. This is without returning the actual possible values

  \param pol a pointer to a polaris structure
  \param selectUnits This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC			DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG			Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT			Positions scaled as voltages by the calibration scaling
  \param yStepSize defines the distance between the lines in the Y field. This value is added to Y between lines. YStepSize can be in DAC counts, volts and dregrees.

  \return char pointer containing result of operation called.
*/
 char * xarRNYStep(polaris *pol, int selectUnits, double yStepSize)
 { return xarCommand((pol), "_rnystep:%d%lf", (selectUnits), (yStepSize)); }



/*! \ingroup XarTraXRaster
  Define the number of Y lines in a full frame. Each raster frame is made up of multiple lines, this is used to define the number of lines per frame. 

  \param pol a pointer to a polaris structure
  \param numYLines defines the number of Y lines per frame be 0.
  \param actualNumYLines the actual number of lines that can be performed (absolute range checking only).

  \return char pointer containing result of operation called.
*/
 char * xarRNYLinesA(polaris *pol, int numYLines, double *actualNumYLines)
{
	char temp[100];
	char * message;
	message = xarCommand((pol), "_rnylines:%d", (numYLines));
	if (atoi(message) == XAR_OKAY){  // if successful
		sscanf(message, "%c,%lf", &temp, *actualNumYLines);
	}
	return message;
}
/*! \ingroup XarTraXRaster
  Define the number of Y lines in a full frame. Each raster frame is made up of multiple lines, this is used to define the number of lines per frame. This is without returning the actual possible values

  \param pol a pointer to a polaris structure
  \param numYLines defines the number of Y lines per frame be 0.

  \return char pointer containing result of operation called.
*/
 char * xarRNYLines(polaris *pol, int numYLines)
 {	return xarCommand((pol), "_rnylines:%d", (numYLines)); }


 /*! \ingroup XarTraXRaster
   Will do a raster scan of the giving area inside -XLength/2 to XLength/2 and -YLength/2 to YLength/2.
   the smaller the increment the higher the resolution will be but the slower it will be.  The points will
   be dumbed 

  \param pol					a pointer to a polaris structure
  \param OutFileName			file name of where the raster scan should be stored.
  \param XLength				Total number degrees at which it should scan in the X direction (center is always 0,0)
  \param YLength				Total number degrees at which it should scan in the Y direction (center is always 0,0)
  \param Increment				degrees in which it should increment at each step
  \param minRange				minimum value that should be stored.  anything smaller is assumed to be distortion and ignored.
  \param maxRange				maximum value that should be stored.  anything greater is assumed to be distortion and ignored.

  \return integer value representing error that occured or OK which finished
*/
int xarRasterScan(polaris *pol, char *OutFileName, double XLength, double YLength, double Increment, int maxRange, int minRange) {

	FILE *fileOut;
	double trans[4];
	int counter=0;
	int totalMisses=0;
	char * message;

	if (!(fileOut = fopen(OutFileName, "w"))){
		printf("Cannot create requested file: %s", OutFileName);
		return XAR_FILEERROR;
	}
/*	Using the GUI you should be able to figure out easily yourself the range.
	for (int quickScan = 0; quickScan < 50; quickScan++) {
		xarSetDegPos(pol,	(XLength/2)*-1,	(YLength/2)*-1 );
		xarSetDegPos(pol,	(XLength/2),	(YLength/2)*-1 );
		xarSetDegPos(pol,	(0),			(0)			   );
		xarSetDegPos(pol,	(XLength/2),	(YLength/2)    );
		xarSetDegPos(pol,	(XLength/2)*-1,	(YLength/2)    );
	}
*/
		fprintf(fileOut, "XLength: %lf\n", XLength);
		fprintf(fileOut, "YLength: %lf\n", YLength);
		fprintf(fileOut, "Increment: %lf\n", Increment);
		fprintf(fileOut, "MaxRange: %d\n", maxRange);
		fprintf(fileOut, "MinRange: %d\n\n", minRange);

	int j = 0;

	for (double angleX = (XLength/2)*-1; angleX <= (XLength/2); angleX+=Increment) {
		j=0;
		for (double angleY = (YLength/2)*-1; angleY <= (YLength/2); angleY+=Increment, j++) {
			message = xarSetDegPos(pol, angleX, angleY );
			if (atoi(message))
				return atoi(message);
			Sleep(25);
			message = xarReadFromPolaris(pol, trans);
			while (message == NULL) message = xarReadFromPolaris(pol, trans);
			if (j == 0)
				Sleep(20);
				message = xarReadFromPolaris(pol, trans);
				while (message == NULL) message = xarReadFromPolaris(pol, trans);

			if (trans[0] < 1) {
				counter = 0;
				while(trans[0] < 1 && counter < 4) {
					message = xarReadFromPolaris(pol, trans);
					while (message == NULL) message = xarReadFromPolaris(pol, trans);

					message = xarSetDegPos(pol, angleX, angleY);
					if (atoi(message))
						return atoi(message);
					counter++;
				}
				//if (counter == 4 && trans[0] == 0) {
					/*miss */
				//}
			}

			fprintf(fileOut, "%lf\t%lf\t%lf\n", trans[1], trans[2], trans[3]);
		}
		fprintf(fileOut, "#------------------------------------------#\n");
	}

	fclose(fileOut);

	return XAR_OKAY;
}

 /*! \ingroup XarTraXRaster
   Will do a raster scan of the giving area inside -XLength/2 to XLength/2 and -YLength/2 to YLength/2.
   the smaller the increment the higher the resolution will be but the slower it will be.  The points will
   be dumbed 

  \param pol					a pointer to a polaris structure
  \param OutFileName			file name of where the raster scan should be stored.
  \param topLeftX				The top left X angle at which the scanning shall start
  \param topLeftY				The top left Y angle at which the scanning shall start
  \param bottomRightX			The bottom right X angle is where the laser scanning will stop
  \param bottomRightY			The bottom right Y angle is where the laser scanning will stop
  \param Increment				degrees in which it should increment at each step
  \param minRange				minimum value that should be stored.  anything smaller is assumed to be distortion and ignored.
  \param maxRange				maximum value that should be stored.  anything greater is assumed to be distortion and ignored.

  \return integer value representing error that occured or OK which finished
*/
int xarRasterScan2(polaris *pol, char *OutFileName, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, double Increment, int maxRange, int minRange) {

	FILE *fileOut;
	double trans[4];
	int counter=0;
	int totalMisses=0;
	char * message;

	if (!(fileOut = fopen(OutFileName, "w"))){
		printf("Cannot create requested file: %s", OutFileName);
		return XAR_FILEERROR;
	}
/*	Using the GUI you should be able to figure out easily yourself the range.
	for (int quickScan = 0; quickScan < 50; quickScan++) {
		xarSetDegPos(pol,	(topLeftX),	(topLeftY)			);
		xarSetDegPos(pol,	(topLeftX),	(bottomRightY)		);
		xarSetDegPos(pol,	(topLeftX-bottomRightX), (topLeftY-bottomRightY) );		
		xarSetDegPos(pol,	(bottomRightX),	(bottomRightY)  );
		xarSetDegPos(pol,	(bottomRightX),	(topLeftY)		);
	}
*/	

		fprintf(fileOut, "XLength: %lf\n", abs(bottomRightX - topLeftX));
		fprintf(fileOut, "YLength: %lf\n", abs(bottomRightY - topLeftY));
		fprintf(fileOut, "Increment: %lf\n", Increment);
		fprintf(fileOut, "MaxRange: %d\n", maxRange);
		fprintf(fileOut, "MinRange: %d\n\n", minRange);

	int j = 0;

	for (double angleX = topLeftX; angleX <= bottomRightX; angleX+=Increment) {
		j=0;
		for (double angleY = topLeftY; angleY <= bottomRightY; angleY+=Increment, j++) {


			message = xarSetDegPos(pol, angleX, angleY );
			if (atoi(message))
				return atoi(message);
			Sleep(25);
			message = xarReadFromPolaris(pol, trans);
			while (message == NULL) message = xarReadFromPolaris(pol, trans);
			if (j == 0)
				Sleep(20);
				message = xarReadFromPolaris(pol, trans);
				while (message == NULL) message = xarReadFromPolaris(pol, trans);

			if (trans[0] < 1) {
				counter = 0;
				while(trans[0] < 1 && counter < 4) {
					message = xarReadFromPolaris(pol, trans);
					while (message == NULL) message = xarReadFromPolaris(pol, trans);

					message = xarSetDegPos(pol, angleX, angleY);
					if (atoi(message))
						return atoi(message);
					counter++;
				}
				//if (counter == 4 && trans[0] == 0) {
					/*miss */
				//}
			}

			fprintf(fileOut, "%lf\t%lf\t%lf\n", trans[1], trans[2], trans[3]);
		}
		fprintf(fileOut, "#------------------------------------------#\n");
	}

	fclose(fileOut);

	return XAR_OKAY;
}

/*=====================================================================*/
// EXTRA COMMANDS
/*=====================================================================*/
/*! \defgroup XarTraXExtra XarTraX Extra Methods
	These methods don't need to be called directly but are used by other methods.  There is
	no reason to have to call these methods directly.

	To determine if an error occured, check the return value
*/

/*! \ingroup XarTraXExtra
   Loads in a file, reads through the file and finds the maximum and minimum values for the X and Y and returns them in mmv.

  \param Filename		a char pointer or array to the name of the file to be loaded
  \param mmv			This is a datastructure containing the maximum and minumum values for X and Y.  check MaxMin Structure for details
  \param TotalPoints	Unsigned pointer storing the total number of points in the file that were loaded
  \param Loaded			Is the file already loaded into the system (boolean)

  \return integer value of error message
*/
int xarLoadFile(char *Filename, MaxMinValue *mmv, unsigned &TotalPoints, bool Loaded)
{

	FILE *fileIN;
	boolean init = Loaded;
	char szX[20], szY[20];
	float tempX, tempY;
	char readline[100];
	double MaxX, MinX;
	double MaxY, MinY;
	if (fileIN = fopen(Filename, "r")){
		TotalPoints = 0; // set the value of uTotalPoints

		while(fgets(readline, 80, fileIN) != NULL){
			if (strncmp(readline, "#", 1)==0);				//if it's comment, skip
			else if (strncmp(readline, "\n", 2)==0);
			else{
				TotalPoints++;
					sscanf(readline, "%s %s", &szX, &szY);
					if (!init){
						MaxX = MinX = atof(szX);
						MaxY = MinY = atof(szY);
						init = true;
					} // if
					else{
						tempX = (float)atof(szX);
						tempY = (float)atof(szY);
						if (MaxX<tempX)
							MaxX = tempX;
						if (MinX>tempX)
							MinX = tempX;
						if (MaxY<tempY)
							MaxY = tempY;
						if (MinY>tempY)
							MinY = tempY;
					} // else
			} // end else
		} // end while
	} // end if
	else{
		return XAR_FILEERROR;
	}

	fclose(fileIN);

    (mmv->MaxX) = MaxX;
	(mmv->MaxY) = MaxY;
	(mmv->MinX) = MinX;
	(mmv->MinY) = MinY;

	return XAR_OKAY;
}

/*! \ingroup XarTraXExtra
   Convert the points to degree format based on the Max-Min values of X and Y and the Maximum Angle

  \param x			pointer to a x value that is going to have an angle calculated for it.
  \param y			pointer to a y value that is going to have an angle calculated for it.
  \param mmv		pointer to a data structure holding maximum and minimum values of X and Y.
  \param maxAngle	maximum angle at which the image should be projected at. (size based on distance)
  \param file		a pointer to the file that is being read in

  \return boolean of if the file was able to be read and a point was gathered
*/
bool getNextPoint(double *x, double *y, MaxMinValue *mmv, double maxAngle, FILE *file)
{
	char readline[100];
	double tempX, tempY;
	char szX[20], szY[20];

	tempX = (mmv->MaxX - mmv->MinX)/maxAngle;
	tempY = (mmv->MaxY - mmv->MinY)/maxAngle;
	
	while (fgets(readline, 80, file) != NULL){	// fetch each line from the PLT data file		
		if (strncmp(readline, "#", 1)==0){}	//comment, so skip the entire line
		else if (strncmp(readline, "\n", 2)==0); // if blank, read next line
		else{	// fit the fetched x, y data values with the range of -maxAngle to +maxAngle according to the Max and Min found
			sscanf(readline, "%s %s", &szX, &szY);
			*x = atof(szX);  
			*y = atof(szY);

			if (*x == tempX)
				*x = 0;
			else
				*x = (tempX-*x) / (mmv->MaxX-tempX) * maxAngle;	// use this line to rotate about the image's x-axis
//				*x = (*x-tempX) / (mmv->MaxX-tempX) * maxAngle;

			if (*y == tempY)
				*y = 0;
			else
//				*y = (*y-tempY) / (mmv->MaxY-tempY) * maxAngle;  
				*y = (tempY-*y) / (mmv->MaxY-tempY) * maxAngle;  // use this line to rotate about the image's y-axis

			return true;
		} // end else
	} // end while
	return false;
}

/*! \ingroup XarTraXExtra
   builds a char array represetion of the data point that is to be added into the table for an Arbitrary wave format

  \param count		stores the total size of this point to be added into the table.
  \param data		data the char array represetation of the data to be added into the table
  \param x			pointer to a data structure holding maximum and minimum values of X and Y.
  \param y			maximum angle at which the image should be projected at. (size based on distance)
  \param vi
  	- XAR_LASER_ON			Sets the infrared laser on
	- XAR_LASER_OFF			Sets the infrared laser off
  \param invi
  	- XAR_LASER_ON			Sets the infrared laser on
	- XAR_LASER_OFF			Sets the infrared laser off

*/
void xarAPositionBuildCommand(int &count, char data[], double x, double y, int vi, int invi)
{
	int j=0;
	char temp[100];
	
	strcpy(data,",");
	ftoa(x,temp);
	j+=strlen(temp)+1;
	strcat(data,temp);
	strcat(data,",");
	ftoa(y,temp);
	j+=strlen(temp)+1;
	strcat(data,temp);
	strcat(data,",");
	_itoa(vi,temp,10);
	j+=strlen(temp)+1;
	strcat(data,temp);
	strcat(data,",");
	_itoa(invi,temp,10);
	j+=strlen(temp)+1;
	strcat(data,temp);
	count=j;
}

/*! \ingroup XarTraXExtra
   loadeds the position tables into the system for an arbitrary wave display

  \param pol					a pointer to a polaris structure
  \param selectUnits			This parameter should determine the unit of the position data:
	- XAR_UNIT_DAC				DAC value (16 or 18 bit integers, converted to doubles), with user bit removed
	- XAR_UNIT_DEG				Positions scaled to degrees by the calibration scaling
	- XAR_UNIT_VOLT				Positions scaled as voltages by the calibration scaling
  \param xPositionTable			position table that is created by the getNextPoint method for the X values
  \param yPositionTable			position table that is created by the getNextPoint method for the Y values
  \param sizeofPositionTable	total size of the position table that is going to be added into the system.

  \return char pointer containing result of operation called.
*/
 char * xarUploadDataTOSBC(	polaris *pol, 
									int selectUnits,
									double xPositionTable[], 
									double yPositionTable[],
									int sizeofPositionTable) {
	
	int i,j,start_point,end_point, Rtn;
	int data_count = 0;
	int laser_control = 0;
	char trans_data[1024];
	char temp[50];
	double x,y;
	char * message;

	int NumData = 0;
	start_point=0;
	end_point=0;
	i=0;
	

	while(sizeofPositionTable>0){
		j=0;
		while(j<900){    // control value to compare if the data cluster is smaller than 1024k size
			x=xPositionTable[i];
			y=yPositionTable[i];
			if(sizeofPositionTable>0){
				if((x==xPositionTable[i+1])&&(y==yPositionTable[i+1])){
					xarAPositionBuildCommand(data_count,temp,x,y,1,1);
					if(j==0)
						strcpy(trans_data,temp);
					else 
						strcat(trans_data,temp);
					j+=data_count;
				} // if
				else{
					xarAPositionBuildCommand(data_count,temp,x,y,1,1);
					if(j==0) 
						strcpy(trans_data,temp);
					else
						strcat(trans_data,temp);
					j+=data_count;
				} //else
			} // if

			i++;
			sizeofPositionTable--;
			if(sizeofPositionTable==0) 
				break;
		} // end while
		end_point=i;

		message = xarAPosition(pol, start_point, end_point, selectUnits, trans_data);

		start_point=end_point;
		Rtn=atoi(message);
		if (Rtn != XAR_OKAY)
			return message;
		
	} // end while

	
	return message;
}

/*! \ingroup XarTraXExtra
   Attempts to read where the XarTraX laser is located in the polaris space. (requires Infrared laser to be on
   or nothing will be found and error will be returned).  This is not the most efficent way to read the laser 
   position if other probes being tracked at the same time.  Also, this only gets the FIRST marker value.  If
   multiple marker positions are found, it will ignore those and just take the first one.  trans[0] will contain
   the number of markers found.

  \param pol					a pointer to a polaris structure
  \param trans					this is a array that will contain the position of the XarTraX laser. 
	- trans[0]					contains the number of markers found
	- trans[1]					contains the X position of the laser
	- trans[2]					contains the Y position of the laser
	- trans[3]					contains the Z position of the laser

  \return char pointer containing result of operation called.
*/
char * xarReadFromPolaris(polaris *pol, double trans[4]) {
	char temp[5000];
	char * message;
	trans[0]=trans[1]=trans[2]=trans[3]=0;
	
	message = plGX(pol, PL_PASSIVE + PL_PASSIVE_STRAY + PL_XFORMS_AND_STATUS);
	if (ndiGetError(pol) != 0x00) {
		//std::cout << "Error occurred on plGX\nError number:\t" <<ndiGetError(pol) << "\nMessage:\t" << message << '\n';
		return NULL;
	}

	sscanf(message, "%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n", temp, temp, temp, temp, temp, temp, temp, temp, temp);
	sscanf(temp, "%lf%lf%lf%lf\n", &trans[0], &trans[1], &trans[2], &trans[3]);
	trans[1] *= 0.01;
	trans[2] *= 0.01;
	trans[3] *= 0.01;
	return message;
}


/*=====================================================================*/
/*! \defgroup XarTraXMath XarTrax Misc Math Functions
  These are a set of functions that are needed by other methods inside the 
  XarTraX commands to function properly.  They will not need to be called by
  outside methods.
*/


/*! \ingroup XarTraXMath
	Converts a float value to an ASCII value. 
	  
	\param source double value that is to be converted to a string
	\param szOutString an char array or pointer to where the value should be stored as an ASCII.  Ensure that this is large enough to hold the value
	\return void
*/
void ftoa(double source, char szOutString[])
{
   int  decimal, sign,i,k;
   char *buffer;
   k=0;
   buffer = _fcvt( source, 7, &decimal, &sign );
   if(sign!=0)
   {
	   szOutString[k]='-';
	   k++;
   }
   if(decimal>0)
   {
		for(i=0;i<decimal;i++) 
		{
			szOutString[k]=*buffer;
			k++;
			buffer++;
		}
		szOutString[k]='.';k++;
		for(i=decimal;i<7;i++) 
		{
			szOutString[k]=*buffer;
			k++;
			buffer++;
		}
		szOutString[k]='\0';
   }
   else
   {
	   szOutString[k]='.';
	   k++;
	   for(i=0;i<abs(decimal);i++)
	   {
		   szOutString[k]='0';
		   k++;
	   }
		for(i=abs(decimal);i<7;i++) 
		{
			szOutString[k]=*buffer;
			k++;
			buffer++;
		}
		szOutString[k]='\0';
   }	
}

/*! \ingroup XarTraXExtra
   Find the vector at which the laser is being projected from the system when it is aiming at the (0,0) angle (CENTER).
   When calculating OHAT using this method, ensure that NO other passive markers are visible by the polaris or it can
   cause major problems and result in a completely incorrect answer.  Also, the laser MUST be able to be detected at the 
   Center position or it will wait and loop until it can.

  \param pol		a pointer to a polaris structure
  \param _OHAT		double array contain the returned X,Y,Z unit vectors for the vector being project
 
*/

void xarFindOHAT(polaris * pol, double _OHAT[3]) {
	double trans[4];
	double _CENTER[3];

    xarSetDegPos(pol, 0,0);
	xarReadFromPolaris(pol, trans);
	Sleep(200);
	xarReadFromPolaris(pol, trans);
	xarReadFromPolaris(pol, trans);
	while (trans[0] == 0) {
		Sleep(500);
		printf("UNABLE TO FIND LASER\n");
		xarReadFromPolaris(pol, trans);
	}

	_CENTER[0] = trans[1];
	_CENTER[1] = trans[2];
	_CENTER[2] = trans[3];

	double _ORIGIN[3];

	_ORIGIN[0] =	0;
	_ORIGIN[1] =	4.860;
	_ORIGIN[2] =	-65.166;


	double tempD =  sqrt( pow((_CENTER[0]-_ORIGIN[0]),2) + pow((_CENTER[1]-_ORIGIN[1]),2) + pow((_CENTER[2]-_ORIGIN[2]),2));


	_OHAT[0] = (_CENTER[0]-_ORIGIN[0])/(tempD);
	_OHAT[1] = (_CENTER[1]-_ORIGIN[1])/(tempD);
	_OHAT[2] = (_CENTER[2]-_ORIGIN[2])/(tempD);

}

/*! \ingroup XarTraXExtra
   Find the degree angles needed to be entered into the Xartrax system to hit a specific point in the Polaris space.  
   Several assumptions:
    - Polaris is attached to top of XartraX
	- XartraX is parallel to the Polaris.  I.E. going left does not actually go left and slightly up.
	- OHAT is correct.  When calculating OHAT using the given method in this API, ensure that NO other passive markers are 
	visible by the polaris or it can cause major problems.

  \param X			the X position at which the angles are to be calculated
  \param Y			the Y position at which the angles are to be calculated
  \param Z			the Z position at which the angles are to be calculated
  \param _OHAT		unit vector at which the laser is being projected (use xarFindOHAT to calculate)
  \param xAngle		contains the return value of the X-Angle to get the this position
  \param yAngle		contains the return value of the Y-Angle to get the this position
 
*/
void xarPositionToAngle(double X, double Y, double Z, double _OHAT[3], double &xAngle, double &yAngle, vtkMatrix4x4 * worldMatrix) {

	double _ORIGIN[3];

	_ORIGIN[0] =	0;
	_ORIGIN[1] =	4.860;
	_ORIGIN[2] =	-65.166;
	
	if (worldMatrix != NULL) {
		double tempX = X*worldMatrix->GetElement(0,0)+ Y*worldMatrix->GetElement(0,1)+ Z*worldMatrix->GetElement(0,2)+ worldMatrix->GetElement(0,3);
		double tempY = X*worldMatrix->GetElement(1,0)+ Y*worldMatrix->GetElement(1,1)+ Z*worldMatrix->GetElement(1,2)+ worldMatrix->GetElement(1,3);
		double tempZ = X*worldMatrix->GetElement(2,0)+ Y*worldMatrix->GetElement(2,1)+ Z*worldMatrix->GetElement(2,2)+ worldMatrix->GetElement(2,3);
		X = tempX; Y = tempY; Z = tempZ;
	}

	double distanceToOrigin = sqrt( pow(X-_ORIGIN[0],2) + pow((Y-_ORIGIN[1]),2) + pow(Z-(_ORIGIN[2]),2));

	double pointOnCenter[3];
	pointOnCenter[0] = _OHAT[0] * distanceToOrigin + _ORIGIN[0];
	pointOnCenter[1] = _OHAT[1] * distanceToOrigin + _ORIGIN[1];
	pointOnCenter[2] = (_OHAT[2] * distanceToOrigin + _ORIGIN[2]);

    double distanceToCenterPoint = sqrt( pow(pointOnCenter[0]-_ORIGIN[0],2) + pow((pointOnCenter[1] -_ORIGIN[1]),2) + pow(pointOnCenter[2]-(_ORIGIN[2]),2));

	double outsideDistance = sqrt( pow(X- pointOnCenter[0],2) + pow((Y- pointOnCenter[1]),2) + pow(Z-pointOnCenter[2],2));

	double l2o = sqrt( pow(pointOnCenter[0]-_ORIGIN[0],2) + 	pow((Y-_ORIGIN[1]),2) + pow(pointOnCenter[2]-(_ORIGIN[2]),2) );
	double l2c = sqrt( 0 + 	pow((Y-pointOnCenter[1]),2) + 0);
	xAngle = acosf(((pow(distanceToCenterPoint,2) + pow(l2o,2) - pow(l2c,2))/(2*l2o*distanceToCenterPoint)))/(PI/180); 
	double u2o = sqrt( pow(X-_ORIGIN[0],2) + 	pow((pointOnCenter[1]-_ORIGIN[1]),2) + pow(pointOnCenter[2]-(_ORIGIN[2]),2) );
	double u2c = sqrt( 0 + 	pow((X-pointOnCenter[0]),2) + 0);
	yAngle = acosf(((pow(distanceToCenterPoint,2) + pow(u2o,2) - pow(u2c,2))/(2*u2o*distanceToCenterPoint)))/(PI/180); 

	/* IF ANYONE can tell me why i need to multiple by this MAGIC number to make it
	 * hit the correct point it would be really nice.  From all my logic, i cannot figure
	 * out why it needs to be multipled.  The only thing that would makes sense is that 
	 * a) The origin of the laser is completely off
	 * b) the degrees entered into the laser movement system are not accurate
	 */
	 
	if (Y > pointOnCenter[1]) 
		xAngle /=1.5000-fabs((pow((xAngle/11),2))/200);
	else 
		xAngle /=-1.5000+fabs((pow((xAngle/11),2))/200);



	if (X > pointOnCenter[0]) 
		yAngle /=1.5000-fabs((pow((yAngle/11),2))/100);
	else 
		yAngle /=-1.5000+fabs((pow((yAngle/11),2))/100);

}

void xarPrintAngles(polaris * pol) {
	double trans[4];

	xarVisibleOn(pol);
	xarInfraOn(pol);

 		xarReadFromPolaris(pol, trans);
		xarReadFromPolaris(pol, trans);

	for (int i = -4; i <= 4; i++) {
		xarSetDegPos(pol, i,0);
		xarReadFromPolaris(pol, trans);
		xarReadFromPolaris(pol, trans);
		xarReadFromPolaris(pol, trans);
		Sleep(100);
		xarReadFromPolaris(pol, trans);
		if (trans[0] > 0) {
			printf("0,%d  x:%lf y:%lf z:%lf\n", i, trans[1], trans[2], trans[3]);
		}
	}

	for (i = -4; i <= 4; i++) {
		xarSetDegPos(pol, i,0);
		xarReadFromPolaris(pol, trans);
		xarReadFromPolaris(pol, trans);
		xarReadFromPolaris(pol, trans);
		Sleep(100);
		xarReadFromPolaris(pol, trans);
		if (trans[0] > 0) {
			printf("%d,0  x:%lf y:%lf z:%lf\n", i, trans[1], trans[2], trans[3]);
		}
	}

   xarVisibleOff(pol);
   xarInfraOff(pol);

}

/*
#ifdef __cplusplus
}
#endif
*/

