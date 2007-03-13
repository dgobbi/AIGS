/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2007/03/13 15:17:31 $
  Version:   $Revision: 1.12 $

==========================================================================

Copyright (c) 2000-2005 Atamai, Inc.

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
#include "vtkCharArray.h"
#include "vtkCriticalSection.h"
#include "vtkDoubleArray.h"
#include "vtkMatrix4x4.h"
#include "vtkMultiThreader.h"
#include "vtkMutexLock.h"
#include "vtkObjectFactory.h"
#include "vtkSocketCommunicator.h"
#include "vtkTracker.h"
#include "vtkTransform.h"
#include "vtkTimerLog.h"
#include "vtkTrackerTool.h"
#include "vtkTrackerBuffer.h"


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
  this->RequestUpdateMutex = vtkCriticalSection::New();

  this->SocketCommunicator = vtkSocketCommunicator::New();
  this->ServerMode = 0;
  this->RemoteAddress = NULL;
 
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
  this->RequestUpdateMutex->Delete();
  this->SocketCommunicator->Delete();
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

  // loop until cancelled
  for (int i = 0;; i++)
    {
    // get current tracking rate over last 10 updates
#if (VTK_MAJOR_VERSION <= 4)
    double newtime = vtkTimerLog::GetCurrentTime();
#else
    double newtime = vtkTimerLog::GetUniversalTime();
#endif
    double difftime = newtime - currtime[i%10];
    currtime[i%10] = newtime;
    if (i > 10 && difftime != 0)
      {
      self->InternalUpdateRate = (10.0/difftime);
      }
    // need to change this code for server/client/normal
    // query the hardware tracker
    self->UpdateMutex->Lock();
    if( !self->GetServerMode() && self->GetRemoteAddress() )
      {
      // client 
      for( int i = 0; i< self->GetNumberOfTools(); i++ )
	{
	vtkDoubleArray * msg = vtkDoubleArray::New();
	msg->SetNumberOfTuples(19);
	vtkMatrix4x4 *matrix = NULL;
	double vals[3];
	long flags=0; 
	double ts=0;
	int tool=0;
	if(self->GetSocketCommunicator()->Receive( msg, 1, 7 ) == 0)
	  {
	  vtkGenericWarningMacro(" Didin't receive the buffer");
	  }
	else
	  {

	  matrix = vtkMatrix4x4::New();
	  self->ConvertMessageToBuffer(msg, vals, matrix);
	  tool = static_cast<int>(vals[0]);
	  flags = static_cast<long>(vals[1]);
	  ts = vals[2];
	  int k = 3;
	  double elements[16];
	  for (k =3; k<3+16; k++)
	    {
	    elements[k-3] = msg->GetValue(k);
	    }
	  matrix->DeepCopy(elements);
	  self->ServerToolUpdate( tool, matrix, flags, ts );
	  }
	}
      }
    else // server & normal 
      {
      self->InternalUpdate();
      if(self->GetServerMode())
	{
        // server
	for( int i = 0; i< self->GetNumberOfTools(); i++ )
	  {
	  vtkTrackerBuffer* buffer  = self->GetTool(i)->GetBuffer();
	  if(buffer)
	    {
	    vtkMatrix4x4 *mat = vtkMatrix4x4::New(); 
	    long flags;
	    double ts;
	    int tool = i;
	    buffer->Lock();
	    buffer->GetUncalibratedMatrix(mat, 0);
	    flags = buffer->GetFlags(0);
	    ts = buffer->GetTimeStamp(0);
	    buffer->Unlock();
	    vtkDoubleArray *da = vtkDoubleArray::New();
	    self->ConvertBufferToMessage(tool, mat, flags, ts, da);
	    if(self->GetSocketCommunicator()->GetIsConnected())
	      {
	      if (self->GetSocketCommunicator()->Send(da, 1, 7) == 0)
		{
		vtkGenericWarningMacro("Could not set tool buffer message");
		}
	      }
	    }
	  }
	}
      }
    self->UpdateTime.Modified();
    self->UpdateMutex->Unlock();

    // check to see if main thread wants to lock the UpdateMutex
    self->RequestUpdateMutex->Lock();
    self->RequestUpdateMutex->Unlock();
    
    // check to see if we are being told to quit 
    data->ActiveFlagLock->Lock();
    int activeFlag = *(data->ActiveFlag);
    data->ActiveFlagLock->Unlock();

    if (activeFlag == 0)
      {
      return NULL;
      }
    }
}


//----------------------------------------------------------------------------
int vtkTracker::Probe()
{
// this->UpdateMutex->Lock();
//   if(!this->ServerMode && this->RemoteAddress) // client
//     {
//     vtkCharArray* ca = vtkCharArray::New();
//     ca->SetTupleValue(0, "Probe");
//     this->SocketCommunicator->Send(ca, 1, 9);
//     int i[1]= {0};
//     this->SocketCommunicator->Receive(i, 1, 1, 22);
//     this->UpdateMutex->Unlock();
//     return i[0];
//     }
//    if (this->InternalStartTracking() == 0)
//     {
//     this->UpdateMutex->Unlock();
//     if(this->ServerMode)
//       {
//       int i[1]= {0};
//       this->SocketCommunicator->Send(i, 1, 1, 22);
//       }
//     return 0;
//     }

//   this->Tracking = 1;

//   if (this->InternalStopTracking() == 0)
//     {
//     this->Tracking = 0;
//     this->UpdateMutex->Unlock();
//     if(this->ServerMode)
//       {
//       int i[1]= {0};
//       this->SocketCommunicator->Send(i, 1, 1, 22);
//       }
//     return 0;
//     }

//   this->Tracking = 0;
//   this->UpdateMutex->Unlock();
//   if(this->ServerMode)
//       {
//       int i[1]= {1};
//       this->SocketCommunicator->Send(i, 1, 1, 22);
//       }
  return 1;
}

//----------------------------------------------------------------------------
void vtkTracker::StartTracking()
{
  int tracking = this->Tracking;
  // client 
  if(!this->ServerMode && this->RemoteAddress) 
    {
    // ask the communication thread to send a message" StartTracking"
    vtkCharArray *ca = vtkCharArray::New();
  
    if(this->SocketCommunicator->GetIsConnected() )
      {
      ca->SetNumberOfComponents(16);
      ca->SetArray("StartTracking", 16, 1);
      ca->Modified();
      if(!this->SocketCommunicator->Send(ca, 1, 9))
	{
	vtkErrorMacro(" ClientStartTracking: Could not send the message\n");
	}
      else
	{
	// wait to receive the Tool Information acknoledgement
	char *msg = "InternalStartTracking";
	const int maxMessages = 50;
	int messageNum = 0;
	for (messageNum = 0; 
	     messageNum < maxMessages && 
	       strcmp( msg, "InternalStartTrackingSuccessful" ) ;
	     messageNum++)
	  {
	  vtkCharArray *toolInfoMessage = vtkCharArray::New();
	  if( !this->SocketCommunicator->Receive(toolInfoMessage, 1, 9) )
	    {
	    vtkErrorMacro("Could not RecieveToolInfo");
	    }
	  else
	    {
	    msg = NULL;
	    msg = new char [toolInfoMessage->GetNumberOfComponents()];
	    toolInfoMessage->GetTupleValue( 0, msg );
	    }
	  this->InterpretCommands( msg );
	  // Delete local variables
	  toolInfoMessage->Delete();
	  }
	if (messageNum == maxMessages)
	  {
	  vtkErrorMacro("Waited to receive \"EndEnabledToolPort\" "
			"but it never came");
	  }
	this->Tracking = 1;
	}
      }
    
    this->UpdateMutex->Lock();
    this->ThreadId = this->Threader->SpawnThread((vtkThreadFunctionType)\
                                                 &vtkTrackerThread,this);
    this->LastUpdateTime = this->UpdateTime.GetMTime();
    this->UpdateMutex->Unlock();
    // Delete 
    ca->Delete();
    } //end client

  //server / normal
  else if(this->ServerMode || !this->RemoteAddress) 
    {
    this->Tracking = this->InternalStartTracking();
    if(this->ServerMode)
      {
      char *msgText = "InternalStartTrackingSuccessful";
      vtkCharArray *endmsg = vtkCharArray::New();
     
      endmsg->SetNumberOfComponents(40);
      endmsg->SetArray( msgText, 40, 1);
      if(!this->SocketCommunicator->Send(endmsg, 1, 9))
	{
	vtkErrorMacro(
	  "Could not send message: InternalStartTrackingSuccessful");
	}
      // delete local variable
      endmsg->Delete();
      }
    // start the tracking thread
    if (!(this->Tracking && !tracking && this->ThreadId == -1))
      {
      return;
      }
    
    // this will block the tracking thread until we're ready
    this->UpdateMutex->Lock();
    
    // start the tracking thread
    this->ThreadId = this->Threader->SpawnThread((vtkThreadFunctionType)\
      						  &vtkTrackerThread,this);
    this->LastUpdateTime = this->UpdateTime.GetMTime();
    
    // allow the tracking thread to proceed
    this->UpdateMutex->Unlock();
    }
    // wait until the first update has occurred before returning
    int timechanged = 0;
    
    while (!timechanged)
      {
      this->RequestUpdateMutex->Lock();
      this->UpdateMutex->Lock();
      this->RequestUpdateMutex->Unlock();
      timechanged = (this->LastUpdateTime != this->UpdateTime.GetMTime());
      this->UpdateMutex->Unlock();
#ifdef _WIN32
      Sleep((int)(100));
#elif defined(__FreeBSD__) || defined(__linux__) || defined(sgi) || defined(__APPLE__)
      struct timespec sleep_time, dummy;
      sleep_time.tv_sec = 0;
      sleep_time.tv_nsec = 100000000;
      nanosleep(&sleep_time,&dummy);
#endif
      }
     

}

//----------------------------------------------------------------------------
void vtkTracker::StopTracking()
{
  if( !this->ServerMode && this->RemoteAddress ) // client
    {
    vtkCharArray *ca = vtkCharArray::New();
    ca->SetNumberOfComponents(13);
    ca->SetArray("StopTracking", 13, 1);
    
    if( this->SocketCommunicator->GetIsConnected())
      {
      if( !this->SocketCommunicator->Send(ca, 1, 9))
	{
	vtkErrorMacro("Could not Send the message StopTracking!\n");
	}
      }
    vtkCharArray *ca2= vtkCharArray::New();
    this->Threader->TerminateThread(this->ThreadId);
    this->ThreadId = -1;
    if(!this->SocketCommunicator->Receive(ca2, 1, 9))
      {
      vtkErrorMacro("Could not Receive message InternalStopTrackingSuccessful");
      }
    else
      {
      char *msgText = new char [ca2->GetNumberOfComponents()];
      ca2->GetTupleValue(0, msgText);
      this->InterpretCommands(msgText);
      msgText = NULL;
      ca2->Delete();
      }
    
   
    // Delete local variable
    ca->Delete();
    return;
    }
  // normal and server 
  if ( this->Tracking && this->ThreadId != -1 )
    {
    this->Threader->TerminateThread(this->ThreadId);
    this->ThreadId = -1;
    }
  
  this->InternalStopTracking();
  this->Tracking = 0;
  if(this->ServerMode)
    {
    vtkCharArray * ca = vtkCharArray::New();
    ca->SetNumberOfComponents(40);
    ca->SetArray("InternalStopTrackingSuccessful", 40, 1);
    if(!this->SocketCommunicator->Send(ca, 1, 9))
      {
      vtkErrorMacro("Could not send message InternalStopTrackingSuccessful");
      }
    ca->Delete();
    }
}

//----------------------------------------------------------------------------
void vtkTracker::Update()
{
  if (!this->Tracking)
    { 
    return; 
    }
 if(!this->ServerMode && this->RemoteAddress)// client
    {
    vtkCharArray *ca = vtkCharArray::New();
    ca->SetNumberOfComponents(7);
    ca->SetArray("Update", 7, 1);
  
    if( this->SocketCommunicator->GetIsConnected())
      {
      if( !this->SocketCommunicator->Send(ca, 1, 9) )
	{
	vtkErrorMacro("Could not send Update()!\n");
	}
      }
    ca->Delete();
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
  if(tool == 3)
    {
    double p[3] = {0, 0, 0};
    double p2[3] = {0, 0, 0};
    this->Tools[tool]->GetTransform()->TransformPoint(p, p2);
    }
}
  
//----------------------------------------------------------------------------
void vtkTracker::Beep(int n)
{
  this->RequestUpdateMutex->Lock();
  this->UpdateMutex->Lock();
  this->RequestUpdateMutex->Unlock();

  this->InternalBeep(n);

  this->UpdateMutex->Unlock();
}

//----------------------------------------------------------------------------
void vtkTracker::SetToolLED(int tool, int led, int state)
{
  this->RequestUpdateMutex->Lock();
  this->UpdateMutex->Lock();
  this->RequestUpdateMutex->Unlock();

  this->InternalSetToolLED(tool, led, state);

  this->UpdateMutex->Unlock();
}

//-----------------------------------------------------------------------------
void vtkTracker::Connect()
{
  if( this->RemoteAddress && !this->ServerMode )
    {
    if( !this->SocketCommunicator->ConnectTo(
	 this->RemoteAddress,this->NetworkPort))
      {
      vtkErrorMacro("Could not connect to server\n");
      }
    }
  else
    {
    vtkErrorMacro("Do not need to Call Connect() in Normal Mode");
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::Disconnect()
{
  if( this->RemoteAddress && !this->ServerMode )
    {
    vtkCharArray *ca = vtkCharArray::New();
    ca->SetNumberOfComponents(12);
    ca->SetArray("Disconnect", 12, 1);
    if(!this->SocketCommunicator->Send(ca, 1, 9))
      {
      vtkErrorMacro(" Could not send Disconnect\n");
      }
    ca->Delete();
    }
  else
    {
    vtkErrorMacro("Disconnect can be called from Client Tracker only !\n");
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::StartServer()
{
  int i = 0;
  if(this->SocketCommunicator->WaitForConnection(this->NetworkPort))
    {
    this->ClientConnected = 1;
    while( this->ClientConnected )
      {
      // receive length 
      vtkCharArray *ca = vtkCharArray::New();
      char *m;
      if (!this->SocketCommunicator->Receive( ca, 1, 9 ))
	{
	this->ClientConnected = 0;
	vtkErrorMacro("Could not receive message.");
	break;
	}
      
      char * messageText = new char [ca->GetNumberOfComponents()];
      ca->GetTupleValue( 0, messageText );
      int t = ca->GetNumberOfTuples();
      
      this->InterpretCommands( messageText );
      // delete local variables
      ca->Delete();
      }
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::InterpretCommands( char *messageText )
{
  vtkIdType t = 0;
  
  if(!messageText)
    {
    return;
    }
  if( !strcmp(messageText, "StartTracking" ))
    {
    this->StartTracking();
    return ;
    }
  if( !strcmp(messageText, "StopTracking" ))
    {
    this->StopTracking();
    return ;
    }
  if( !strcmp(messageText, "Probe" ))
    {
    this->Probe();
    return ;
    }
   if( !strcmp(messageText, "Update" ))
    {
    this->Update();
    return ;
    }
  if( !strcmp(messageText, "Disconnect" ))
    {
    this->ClientConnected = 0;
    this->SocketCommunicator->CloseConnection();
    return ;
    }
  
  this->InternalInterpretCommand( messageText );
}

//-----------------------------------------------------------------------------
void vtkTracker::ConvertMessageToBuffer( vtkDoubleArray *da, 
					 double *vals, vtkMatrix4x4 *matrix )
{
  double t, f;
  
  vals[0] = da->GetValue(0);
  vals[1] = da->GetValue(1);
  vals[2] = da->GetValue(2);
}

//-----------------------------------------------------------------------------
void vtkTracker::ConvertBufferToMessage( int tool, vtkMatrix4x4 *matrix, 
					 long flags, double ts,
					 vtkDoubleArray *msg )
{
  if(!msg)
    {
    msg = vtkDoubleArray::New();
    }
  msg->InsertNextValue(static_cast<double>(tool));
  msg->InsertNextValue(static_cast<double>(flags));
  msg->InsertNextValue(ts);
  
  for( int i = 0; i < 4; i++)
    {
    for( int j =0; j < 4; j++)
      {
      msg->InsertNextValue( matrix->GetElement(i,j));
      }
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::ServerToolUpdate( int tool, 
				   vtkMatrix4x4 *matrix, 
				   long flags, double ts )
{
  if(!this->ServerMode )
    {
    this->ToolUpdate( tool, matrix, flags, ts );
    }
}
