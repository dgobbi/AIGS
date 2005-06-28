/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkPolhemusTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2005/06/28 21:21:59 $
  Version:   $Revision: 1.2 $

==========================================================================

Copyright (c) 2005 Atamai, Inc.
All rights reserved.

THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE SOFTWARE "AS IS"
WITHOUT EXPRESSED OR IMPLIED WARRANTY INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE.  IN NO EVENT SHALL ANY COPYRIGHT HOLDER OR OTHER PARTY WHO MAY
MODIFY AND/OR REDISTRIBUTE THE SOFTWARE UNDER THE TERMS OF THIS LICENSE
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, LOSS OF DATA OR DATA BECOMING INACCURATE
OR LOSS OF PROFIT OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF
THE USE OR INABILITY TO USE THE SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.

=========================================================================*/

#include <limits.h>
#include <float.h>
#include <math.h>
#include "polhemus.h"
#include "vtkMath.h"
#include "vtkTransform.h"
#include "vtkPolhemusTracker.h"
#include "vtkTrackerTool.h"
#include "vtkTimerLog.h"
#include "vtkObjectFactory.h"

// maximum calibrated range for polhemus is 48 inches, this is in millimetres
// FIX ME
#define VTK_POLHEMUS_RANGE 1219.2

//----------------------------------------------------------------------------
vtkPolhemusTracker* vtkPolhemusTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkPolhemusTracker");
  if(ret)
    {
    return (vtkPolhemusTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkPolhemusTracker;
}

//----------------------------------------------------------------------------
vtkPolhemusTracker::vtkPolhemusTracker()
{
  this->SendMatrix = vtkMatrix4x4::New();
  this->SerialPort = 1;  // default serial port is COM1
  this->BaudRate = 9600;
  this->Mode = PH_NOTHREAD;
  this->Polhemus = phNew();
  this->SetNumberOfTools(1);
}

//----------------------------------------------------------------------------
vtkPolhemusTracker::~vtkPolhemusTracker() 
{
  if (this->Tracking)
    {
    this->StopTracking();
    }
  phDelete(this->Polhemus);
  this->SendMatrix->Delete();
}
  
//----------------------------------------------------------------------------
void vtkPolhemusTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkTracker::PrintSelf(os,indent);
  
  os << indent << "SerialPort: " << this->SerialPort << "\n";
  os << indent << "BaudRate: " << this->BaudRate << "\n";
  os << indent << "SendMatrix: " << this->SendMatrix << "\n";
  this->SendMatrix->PrintSelf(os,indent.GetNextIndent());
}  

//----------------------------------------------------------------------------
int vtkPolhemusTracker::Probe()
{
  static int baudrates[2] = { PH_38400, 
                              PH_9600,
  };
  int nbaud, baud, devicenum;
  int errnum;
  char *devicename;
  char reply[256];

  if (this->Tracking)
    {
    return 1;
    }

  switch (this->BaudRate)
    {
    case 1200: baud = PH_1200; break; 
    case 2400: baud = PH_2400; break; 
    case 4800: baud = PH_4800; break; 
    case 9600: baud = PH_9600; break; 
    case 19200: baud = PH_19200; break; 
    case 38400: baud = PH_38400; break; 
    case 57600: baud = PH_57600; break; 
    case 115200: baud = PH_115200; break;
    default:
      vtkErrorMacro(<< "Illegal baud rate");
      return 0;
    } 

  polhemus *ph = phNew();
  phSetThreadMode(ph, this->Mode);
  errnum = phGetError(ph);
  if (errnum == PH_MODE_ERROR && this->Mode != PH_NOTHREAD)
    {
    this->Mode = PH_NOTHREAD;
    phSetThreadMode(ph, this->Mode);
    }

  for (devicenum = 0; devicenum < 4; devicenum++)
    {
    devicename = phDeviceName((devicenum + this->SerialPort - 1) % 4);
    for (nbaud = 0; nbaud < 2; nbaud++)
      {
      baud = baudrates[nbaud];
      phSetInitialComm(ph, baud, 'N', 8, 0);
      fprintf(stderr, "trying %s baud %i\n", devicename, baud);
      phOpen(ph,devicename);
      errnum = phGetError(ph);
      /* if we can't open the serial port, go on to next serial port */
      if (errnum == PH_OPEN_ERROR)
	{
        break;
	}

      phSendCommand(ph, "\r\n");
      phReceiveReply(ph, reply, 256);
 
      errnum = phGetError(ph);
      /* if no error, then we're done switching baud rates */
      if (!errnum)
	{
        break;
	}
      phClose(ph);      
      }
    /* if no error, then we're done switching ports */
    if (!errnum)
      {
      break;
      }
    }

  if (!errnum)
    {
    phClose(ph);
    this->SerialPort = ((devicenum + this->SerialPort - 1) % 4) + 1;
    this->BaudRate = baud * 100;
    return 1;
    }
  
  return 0;
} 

//----------------------------------------------------------------------------
int vtkPolhemusTracker::InternalStartTracking()
{
  int errnum,i,baud;
  char status[128];

  switch (this->BaudRate)
    {
    case 1200: baud = PH_1200; break; 
    case 2400: baud = PH_2400; break; 
    case 4800: baud = PH_4800; break; 
    case 9600: baud = PH_9600; break; 
    case 19200: baud = PH_19200; break; 
    case 38400: baud = PH_38400; break; 
    case 57600: baud = PH_57600; break; 
    case 115200: baud = PH_115200; break;
    default:
      vtkErrorMacro(<< "Illegal baud rate");
      return 0;
    }

  if (!this->Tracking)
    {
    phSetInitialComm(this->Polhemus, this->BaudRate, 'N', 8, 0); 
    phSetThreadMode(this->Polhemus, this->Mode);
    errnum = phGetError(this->Polhemus);
    if (errnum == PH_MODE_ERROR)
      {
      this->Mode = PH_NOTHREAD;
      phSetThreadMode(this->Polhemus, this->Mode);
      }

    phOpen(this->Polhemus, phDeviceName(this->SerialPort-1));
    errnum = phGetError(this->Polhemus);
    if (errnum)
      {
      vtkErrorMacro(<< phGetErrorMessage(this->Polhemus));
      return 0;
      }
    
    for (i = 0; i < this->NumberOfTools; i++)
      {
      phSetReplyFormat(this->Polhemus, i+1, PH_POSITION | PH_ANGLES | PH_EXTENDED);
      phSetHemisphere(this->Polhemus, i+1, 0, 0, 0);
      char tool_type[8];
      sprintf(tool_type, "0000%03d", i+1);
      this->Tools[i]->SetToolType(tool_type);
      this->Tools[i]->SetToolManufacturer("POLHEMUS");
      this->Tools[i]->SetToolPartNumber("");
      this->Tools[i]->SetToolSerialNumber("");
      }
    }

  if (this->Mode != PH_NOTHREAD)
    {
    phStream(this->Polhemus);
    }
  if (phGetError(this->Polhemus))
    {
    vtkErrorMacro(<< phGetErrorMessage(this->Polhemus));
    return 0;
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkPolhemusTracker::InternalStopTracking()
{
  if (this->Mode != PH_NOTHREAD)
    {
    phEndStream(this->Polhemus);
    }
  if (phGetError(this->Polhemus))
    {
    vtkErrorMacro(<< phGetErrorMessage(this->Polhemus));
    }
  phClose(this->Polhemus);
  if (phGetError(this->Polhemus))
    {
    vtkErrorMacro(<< phGetErrorMessage(this->Polhemus));
    return 0;
    }

  return 1;
}  

//----------------------------------------------------------------------------
void vtkPolhemusTracker::InternalUpdate()
{
  int i,j,station,tool;
  int errnum = 0;
  float array[9];
  float refmat[4][4];
  float toolmat[4][4];

  float (*xyz)[3] = new float[this->NumberOfTools][3];
  float (*zyx)[3] = new float[this->NumberOfTools][3];
  long *flags = new long[this->NumberOfTools];
  double *timestamps = new double[this->NumberOfTools];

  double timestamp = vtkTimerLog::GetCurrentTime();

  // initialize the flags to 'out-of-view'
  for (tool = 0; tool < this->NumberOfTools; tool++)
    { 
    timestamps[tool] = timestamp;
    flags[tool] = TR_OUT_OF_VIEW;
    }

  // use point mode if not multithreading
  if (this->Mode == PH_NOTHREAD)
    {
    phPoint(this->Polhemus);
    }

  // update each station
  for (tool = 0; tool < 1; tool++) 
    {
    phUpdate(this->Polhemus);
    station = phGetStation(this->Polhemus)-1;
    // if phGetStation() returns 0, then a phase error has occurred
    if (station >= 0)
      {
      timestamps[station] = phGetTime(this->Polhemus);      
      phGetPosition(this->Polhemus,xyz[station]);
      phGetAngles(this->Polhemus,zyx[station]);
      flags[station] = TR_SWITCH1_IS_ON*phGetButton(this->Polhemus);
      // fprintf(stderr,"bird = %d, NumberOfBirds = %d, timestamp = %f\n",bird,this->NumberOfBirds,this->UpdateTimeStamp);
      }
    }

  if ((errnum = phGetError(this->Polhemus)) != 0)
    {
    if (errnum == PH_PHASE_ERROR)
      {
      vtkWarningMacro(<< phGetErrorMessage(this->Polhemus));
      }
    else 
      {
      vtkErrorMacro(<< phGetErrorMessage(this->Polhemus));
      }
    return;
    }

  // handle the reference tool first
  if (this->ReferenceTool >= 0 && this->ReferenceTool < this->NumberOfTools) 
    {
    tool = this->ReferenceTool;
    flags[tool] = 0;

    phMatrixFromAngles(array,zyx[tool]);
    
    for (i = 0; i < 3; i++)
      {
      for (j = 0; j < 3; j++)
	{
        refmat[i][j] = array[3*i + j];
	}
      refmat[3][i] = 0.0;
      refmat[i][3] = xyz[tool][i];
      }
    refmat[3][3] = 1.0;

    if (xyz[tool][0]*xyz[tool][0] +
	xyz[tool][1]*xyz[tool][1] +
	xyz[tool][2]*xyz[tool][2] > VTK_POLHEMUS_RANGE*VTK_POLHEMUS_RANGE)
      {
      flags[tool] = TR_OUT_OF_VOLUME;
      }
    }
  else if (this->ReferenceTool >= this->NumberOfTools)
    { // reference tool doesn't actually exist!
    flags[this->ReferenceTool] = TR_MISSING | TR_OUT_OF_VIEW;
    for (i = 0; i < 3; i++)
      {
      for (j = 0; j < 3; j++)
	{
        refmat[i][j] = 0;
	}
      refmat[i][i] = 1.0;
      refmat[3][i] = 0.0;
      refmat[i][3] = 0.0;
      }
    }

  for (tool = 0; tool < 1; tool++) 
    {
    flags[tool] = 0;

    phMatrixFromAngles(array,zyx[tool]);
    
    for (i = 0; i < 3; i++)
      {
      for (j = 0; j < 3; j++)
	{
        toolmat[i][j] = array[3*i + j];
	}
      toolmat[3][i] = 0.0;
      toolmat[i][3] = xyz[tool][i] * 25.4; // convert inches to mm
      }
    toolmat[3][3] = 1.0;

    // check if tool is within optimal tracking range
    if (xyz[tool][0]*xyz[tool][0] +
	xyz[tool][1]*xyz[tool][1] +
	xyz[tool][2]*xyz[tool][2] > VTK_POLHEMUS_RANGE*VTK_POLHEMUS_RANGE)
      {
      flags[tool] |= TR_OUT_OF_VOLUME;
      }

    if (this->ReferenceTool >= 0 && tool != this->ReferenceTool)
      {
      flags[tool] |= flags[this->ReferenceTool] \
	                & (TR_MISSING | TR_OUT_OF_VIEW | TR_OUT_OF_VOLUME);
      
      // multiply by the inverse of the reference tool matrix,
      // taking advantage of the orthogonality of the reference matrix
      for (i = 0; i < 3; i++)
	{
	for (j = 0; j < 3; j++)
	  {
	  this->SendMatrix->SetElement(i,j,
				       toolmat[i][0]*refmat[j][0] +
				       toolmat[i][1]*refmat[j][1] +
				       toolmat[i][2]*refmat[j][2] +
				       toolmat[i][3]*refmat[j][3]); 
	  }
	this->SendMatrix->SetElement(i,3,toolmat[i][3] - refmat[i][3]);
	this->SendMatrix->SetElement(3,i,0.0);
	}
      this->SendMatrix->SetElement(3,3,1.0);
      }
    else
      {
      for (i = 0; i < 4; i++)
	{
	for (j = 0; j < 4; j++)
	  {
	  this->SendMatrix->SetElement(i,j,toolmat[i][j]);
	  }
	}
      }
    this->ToolUpdate(tool,this->SendMatrix,flags[tool],timestamps[tool]);
    }
  // for tools beyond the max
  this->SendMatrix->Identity();
  for (tool = 1; tool < this->NumberOfTools; tool++) 
    {
    this->ToolUpdate(tool,this->SendMatrix,TR_MISSING | TR_OUT_OF_VIEW,
		     timestamp);
    }

  delete [] timestamps;
  delete [] flags;
  delete [] xyz;
  delete [] zyx;
}







