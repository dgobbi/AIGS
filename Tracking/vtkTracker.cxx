/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2002/11/04 02:09:39 $
  Version:   $Revision: 1.1 $

==========================================================================

Copyright (c) 2000-2002 Atamai, Inc.

Use, modification and redistribution of the software, in source or
binary forms, are permitted provided that the following terms and
conditions are met:

1) Redistribution of the source code, in verbatim or modified
   form, must retain the above copyright notice, this license,
   the following disclaimer, and any notices that refer to this
   license and/or the following disclaimer.  

2) Redistribution in binary form must include the above copyright
   notice, a copy of this license and the following disclaimer
   in the documentation or with other materials provided with the
   distribution.

3) Modified copies of the source code must be clearly marked as such,
   and must not be misrepresented as verbatim copies of the source code.

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
#include "vtkTracker.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkTimerLog.h"
#include "vtkTrackerTool.h"
#include "vtkTrackerBuffer.h"
#include "vtkMultiThreader.h"
#include "vtkCriticalSection.h"
#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
vtkTracker* vtkTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkTracker");
  if(ret)
    {
    return (vtkTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkTracker;
}

//----------------------------------------------------------------------------
vtkTracker::vtkTracker()
{
  this->Tracking = 0;
  this->WorldCalibrationMatrix = vtkMatrix4x4::New();
  this->NumberOfTools = 0;
  this->ReferenceTool = -1;
  this->UpdateTimeStamp = 0;
  this->Tools = 0;
  this->LastUpdateTime = 0;
  this->InternalUpdateRate = 0;

  // for threaded capture of transformations
  this->Threader = vtkMultiThreader::New();
  this->ThreadId = -1;
  this->UpdateMutex = vtkCriticalSection::New();
}

//----------------------------------------------------------------------------
vtkTracker::~vtkTracker()
{
  // The thread should have been stopped before the
  // subclass destructor was called, but just in case
  // se stop it here.
  if (this->ThreadId != -1)
    {
    this->Threader->TerminateThread(this->ThreadId);
    this->ThreadId = -1;
    }

  for (int i = 0; i < this->NumberOfTools; i++)
    { 
    this->Tools[i]->SetTracker(NULL);
    this->Tools[i]->Delete();
    }
  if (this->Tools)
    {
    delete [] this->Tools;
    }

  this->WorldCalibrationMatrix->Delete();

  this->Threader->Delete();
  this->UpdateMutex->Delete();
}
  
//----------------------------------------------------------------------------
void vtkTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkObject::PrintSelf(os,indent);

  os << indent << "WorldCalibrationMatrix: " << this->WorldCalibrationMatrix << "\n";
  this->WorldCalibrationMatrix->PrintSelf(os,indent.GetNextIndent());
  os << indent << "Tracking: " << this->Tracking << "\n";
  os << indent << "ReferenceTool: " << this->ReferenceTool << "\n";
  os << indent << "NumberOfTools: " << this->NumberOfTools << "\n";
}
  
//----------------------------------------------------------------------------
// allocates a vtkTrackerTool object for each of the tools.
void vtkTracker::SetNumberOfTools(int numtools)
{
  int i;

  if (this->NumberOfTools > 0) 
    {
    vtkErrorMacro( << "SetNumberOfTools() can only be called once");
    }
  this->NumberOfTools = numtools;

  this->Tools = new vtkTrackerTool *[numtools];

  for (i = 0; i < numtools; i++) 
    {
    this->Tools[i] = vtkTrackerTool::New();
    this->Tools[i]->SetTracker(this);
    this->Tools[i]->SetToolPort(i);
    }
}  

//----------------------------------------------------------------------------
vtkTrackerTool *vtkTracker::GetTool(int tool)
{
  if (tool < 0 || tool > this->NumberOfTools) 
    {
    vtkErrorMacro( << "GetTool(" << tool << "): only " << \
                      this->NumberOfTools << " are available");
    }
  return this->Tools[tool];
}

//----------------------------------------------------------------------------
// this thread is run whenever the tracker is tracking
static void *vtkTrackerThread(vtkMultiThreader::ThreadInfo *data)
{
  vtkTracker *self = (vtkTracker *)(data->UserData);

  double currtime[10];

  for (int i = 0;; i++)
    {
    // get current tracking rate over last 10 updates
    double newtime = vtkTimerLog::GetCurrentTime();
    double difftime = newtime - currtime[i%10];
    currtime[i%10] = newtime;
    if (i > 10 && difftime != 0)
      {
      self->InternalUpdateRate = (10.0/difftime);
      }

    // query the hardware tracker
    self->UpdateMutex->Lock();
    self->InternalUpdate();
    self->UpdateTime.Modified();
    self->UpdateMutex->Unlock();
    
    // check to see if we are being told to quit 
    //data->ActiveFlagLock->Lock();
    int activeFlag = *(data->ActiveFlag);
    //data->ActiveFlagLock->Unlock();

    if (activeFlag == 0)
      {
      return NULL;
      }
    }
}

//----------------------------------------------------------------------------
int vtkTracker::Probe()
{
  this->UpdateMutex->Lock();

  if (this->InternalStartTracking() == 0)
    {
    this->UpdateMutex->Unlock();
    return 0;
    }

  this->Tracking = 1;

  if (this->InternalStopTracking() == 0)
    {
    this->Tracking = 0;
    this->UpdateMutex->Unlock();
    return 0;
    }

  this->Tracking = 0;
  this->UpdateMutex->Unlock();
  return 1;
}

//----------------------------------------------------------------------------
void vtkTracker::StartTracking()
{
  this->UpdateMutex->Lock();

  int tracking = this->Tracking;

  this->LastUpdateTime = 0;

  this->Tracking = this->InternalStartTracking();

  if (this->Tracking && !tracking && this->ThreadId == -1)
    {
    this->ThreadId = this->Threader->SpawnThread((vtkThreadFunctionType)\
						 &vtkTrackerThread,this);
    }

  this->UpdateMutex->Unlock();
}

//----------------------------------------------------------------------------
void vtkTracker::StopTracking()
{
  this->UpdateMutex->Lock();
  if (this->Tracking && this->ThreadId != -1)
    {
    this->UpdateMutex->Unlock();
    this->Threader->TerminateThread(this->ThreadId);
    this->ThreadId = -1;
    this->UpdateMutex->Lock();
    }

  this->InternalStopTracking();
  this->Tracking = 0;
  this->UpdateMutex->Unlock();
}

//----------------------------------------------------------------------------
void vtkTracker::Update()
{
  if (!this->Tracking)
    { 
    return; 
    }

  if (this->LastUpdateTime == 0 ||
      this->LastUpdateTime == this->UpdateTime.GetMTime())
    {
    // wait at most 0.1s for the next transform to arrive,
    // which is marked by a change in the UpdateTime
    double waittime = vtkTimerLog::GetCurrentTime() + 0.1;
    if (this->LastUpdateTime == 0)
      {  // if this is the first transform, wait up to 5 seconds
      waittime += 5.0;
      }
    while (this->LastUpdateTime == this->UpdateTime.GetMTime() &&
	   vtkTimerLog::GetCurrentTime() < waittime) 
      {
#ifdef _WIN32
      Sleep(10);
#else
#ifdef unix
#ifdef linux
      usleep(10*1000);
#endif
#ifdef sgi
      struct timespec sleep_time, remaining_time;
      sleep_time.tv_sec = 10 / 1000;
      sleep_time.tv_nsec = 1000000*(10 % 1000);
      nanosleep(&sleep_time,&remaining_time);
#endif
#endif
#endif
      }
    }

  for (int tool = 0; tool < this->NumberOfTools; tool++)
    {
    vtkTrackerTool *trackerTool = this->Tools[tool];

    trackerTool->Update();

    this->UpdateTimeStamp = trackerTool->GetTimeStamp();
    }

  this->LastUpdateTime = this->UpdateTime.GetMTime();
}

//----------------------------------------------------------------------------
void vtkTracker::SetWorldCalibrationMatrix(vtkMatrix4x4 *vmat)
{
  int i, j;

  for (i = 0; i < 4; i++) 
    {
    for (j = 0; j < 4; j++)
      {
      if (this->WorldCalibrationMatrix->GetElement(i,j) 
	  != vmat->GetElement(i,j))
	{
	break;
	}
      }
    if (j < 4)
      { 
      break;
      }
    }

  if (i < 4 || j < 4) // the matrix is different
    {
    this->WorldCalibrationMatrix->DeepCopy(vmat);
    this->Modified();
    }
}

//----------------------------------------------------------------------------
vtkMatrix4x4 *vtkTracker::GetWorldCalibrationMatrix()
{
  return this->WorldCalibrationMatrix;
}

//----------------------------------------------------------------------------
void vtkTracker::ToolUpdate(int tool, vtkMatrix4x4 *matrix, long flags,
			    double timestamp) 
{
  vtkTrackerBuffer *buffer = this->Tools[tool]->GetBuffer();

  buffer->Lock();
  buffer->AddItem(matrix, flags, timestamp);
  buffer->Unlock();
}
  
//----------------------------------------------------------------------------
void vtkTracker::Beep(int n)
{
  this->UpdateMutex->Lock();
  this->InternalBeep(n);
  this->UpdateMutex->Unlock();
}

//----------------------------------------------------------------------------
void vtkTracker::SetToolLED(int tool,  int led, int state)
{
  this->UpdateMutex->Lock();
  this->InternalSetToolLED(tool, led, state);
  this->UpdateMutex->Unlock();
}






