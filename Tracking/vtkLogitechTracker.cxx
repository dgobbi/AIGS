/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkLogitechTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2002/11/04 02:09:39 $
  Version:   $Revision: 1.1 $

==========================================================================

Copyright (c) 2000-2002 Atamai, Inc.
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
#include "logitech3d.h"
#include "vtkMath.h"
#include "vtkTransform.h"
#include "vtkLogitechTracker.h"
#include "vtkTrackerTool.h"
#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
vtkLogitechTracker* vtkLogitechTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkLogitechTracker");
  if(ret)
    {
    return (vtkLogitechTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkLogitechTracker;
}

//----------------------------------------------------------------------------
vtkLogitechTracker::vtkLogitechTracker()
{
  this->Transform = vtkTransform::New();
  this->SerialPort = 1;  // default serial port is COM1
  this->Mode = LT_THREAD;
  this->Logitech = ltNew();
  this->SetNumberOfTools(1);
}

//----------------------------------------------------------------------------
vtkLogitechTracker::~vtkLogitechTracker() 
{
  if (this->Tracking)
    {
    this->StopTracking();
    }
 
  ltDelete(this->Logitech);
  this->Transform->Delete();
}
  
//----------------------------------------------------------------------------
void vtkLogitechTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkTracker::PrintSelf(os,indent);
  
  os << indent << "SerialPort: " << this->SerialPort << "\n";
  os << indent << "Transform: " << this->Transform << "\n";
  this->Transform->PrintSelf(os,indent.GetNextIndent());
}  

//----------------------------------------------------------------------------
// check to see if there is a mouse on this->SerialPort
int vtkLogitechTracker::Probe()
{
  int errnum;

  if (this->Tracking)
    {
    return 1;
    }

  ltOpen(this->Logitech,this->SerialPort,LT_3D_MODE,this->Mode);
  errnum = ltGetError(this->Logitech);
  if (errnum == LT_ERROR_MODE)
    {
    this->Mode = LT_NOTHREAD;
    ltOpen(this->Logitech,this->SerialPort,LT_3D_MODE,this->Mode);
    errnum = ltGetError(this->Logitech);
    }
  if (errnum)
    {
    return 0;
    }
  else
    {
    ltClose(this->Logitech);
    }
  return 1;
} 

//----------------------------------------------------------------------------
int vtkLogitechTracker::InternalStartTracking()
{
  int errnum;

  ltOpen(this->Logitech,this->SerialPort,LT_3D_MODE,this->Mode);

  errnum = ltGetError(this->Logitech);
  if (errnum == LT_ERROR_MODE)
    {
    this->Mode = LT_NOTHREAD;
    ltOpen(this->Logitech,this->SerialPort,LT_3D_MODE,this->Mode);
    errnum = ltGetError(this->Logitech);
    }
  if (errnum)
    {
    vtkErrorMacro(<< ltGetErrorMessage(this->Logitech));
    return 0;
    }

  if (this->Mode != LT_NOTHREAD)
    {
    ltStreamMode(this->Logitech);
    }
  if (ltGetError(this->Logitech))
    {
    vtkErrorMacro(<< ltGetErrorMessage(this->Logitech));
    return 0;
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkLogitechTracker::InternalStopTracking()
{
  int stoperr = 0;

  if (this->Mode != LT_NOTHREAD)
    {
    ltDemandMode(this->Logitech);
    stoperr = 1;
    }
  if (ltGetError(this->Logitech))
    {
    vtkErrorMacro(<< ltGetErrorMessage(this->Logitech));
    stoperr = 1;
    }
  ltClose(this->Logitech);
  if (ltGetError(this->Logitech))
    {
    vtkErrorMacro(<< ltGetErrorMessage(this->Logitech));
    stoperr = 1;
    }

  return stoperr;
}

//----------------------------------------------------------------------------
void vtkLogitechTracker::InternalUpdate()
{
  int errnum = 0;

  if (!this->Tracking)
    {
    vtkWarningMacro( << "called Update() when the device was not tracking");
    return;
    }

  int xyz[3];
  int pyr[3];
  int status;
  int time[2];

  // use demand mode if not multithreading
  if (this->Mode == LT_NOTHREAD)
    {
    ltDemand(this->Logitech);
    }

  ltReport(this->Logitech,100);
  ltGetPosition(this->Logitech, xyz);
  ltGetRotation(this->Logitech, pyr);
  ltGetStatus(this->Logitech, &status);
  ltGetTime(this->Logitech, time);

  if ((errnum = ltGetError(this->Logitech)) != 0)
    {
    if (errnum == LT_ERROR_PHASE)
      {
      vtkWarningMacro(<< ltGetErrorMessage(this->Logitech));
      }
    else 
      {
      vtkErrorMacro(<< ltGetErrorMessage(this->Logitech));
      }
    return;
    }

  vtkTransform *transform = this->Transform;
  transform->Identity();
  transform->Translate(xyz[0]*0.0254,xyz[1]*0.0254,xyz[2]*0.0254);
  transform->RotateY(pyr[1]*0.025);
  transform->RotateX(pyr[0]*0.025);
  transform->RotateZ(pyr[2]*0.025);
  
  int flags = 0;
  if (status & LT3D_STATUS_FRINGE) {
    flags |= TR_OUT_OF_VOLUME;
  }
  if (status & LT3D_STATUS_OUT) {
    flags |= TR_OUT_OF_VIEW;
  }
  if (status & LT3D_STATUS_LEFT) {
    flags |= TR_SWITCH1_IS_ON;
  }
  if (status & LT3D_STATUS_MIDDLE) {
    flags |= TR_SWITCH2_IS_ON;
  }
  if (status & LT3D_STATUS_RIGHT) {
    flags |= TR_SWITCH3_IS_ON;
  }    

  this->ToolUpdate(0,transform->GetMatrix(),flags,
		   time[0] + 0.001*time[1]);
}







