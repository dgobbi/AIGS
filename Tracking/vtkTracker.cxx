/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: pdas $
  Date:      $Date: 2007/05/30 18:36:48 $
  Version:   $Revision: 1.17 $

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
  this->NetworkPort = 11111;
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
  this->RemoteAddress = NULL;
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
	double *msg = new double[19];
	vtkMatrix4x4 *matrix = NULL;
	double vals[3];
	long flags=0; 
	double ts=0;
	int tool=0;
	if(self->GetSocketCommunicator()->Receive( msg, 19, 1, 33 ) == 0)
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
	    double *da = new double[19];
	    self->ConvertBufferToMessage(tool, mat, flags, ts, da);
	    if(self->GetSocketCommunicator()->GetIsConnected()>0)
	      {
	      if (self->GetSocketCommunicator()->Send(da, 19, 1, 33) == 0)
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
  this->UpdateMutex->Lock();
  // Client
  if(!this->ServerMode && this->RemoteAddress) 
    {
    int success[1] = {0};
    int len = 6;
    char *msg = "Probe";
    if(this->SocketCommunicator->GetIsConnected()>0)
      {
      if(this->SocketCommunicator->Send(&len, 1, 1, 11))
	{
	if(!this->SocketCommunicator->Send(msg, len, 1, 22))
	  {
	  vtkErrorMacro("Could not send message Probe\n");
	  exit(0);
	  this->UpdateMutex->Unlock();
	  return 0;
	  }
	}
      else
 	{
 	this->UpdateMutex->Unlock();
 	if(!this->SocketCommunicator->Receive(success, 1, 1, 11))
 	  {
 	  vtkErrorMacro("Could not receive the Probe results");
	  exit(0);
 	  }
 	}
      }
    return success[0];
    }
  // Server / Normal
  if (this->InternalStartTracking() == 0)
    {
    this->UpdateMutex->Unlock();
    int success[1] = {0};
    if(this->ServerMode)
      {
      if(this->SocketCommunicator->GetIsConnected()>0)
	{
	this->SocketCommunicator->Send(success,1, 1, 11);
	}
      }
      return 0;
      }
    
    this->Tracking = 1;
    
    if (this->InternalStopTracking() == 0)
      {
      this->Tracking = 0;
      this->UpdateMutex->Unlock();
      if(this->ServerMode)
	{
	int success[1]= {0};
	if(this->SocketCommunicator->GetIsConnected()>0)
	  {
	  if(!this->SocketCommunicator->Send(success, 1, 1, 11))
	    {
	    vtkErrorMacro("Could not send Success information!\n");
	    exit(0);
	    }
	  }
	else
	  {
	  vtkErrorMacro("Client Not Connected.\n");
	  }
	}
      return 0;
    }

  this->Tracking = 0;
  this->UpdateMutex->Unlock();
  if(this->ServerMode)
      {
      int success[1]= {1};
      if(this->SocketCommunicator->GetIsConnected()>0)
	  {
	  if(!this->SocketCommunicator->Send(success, 1, 1, 11))
	    {
	    vtkErrorMacro("Could not send Success information!\n");
	    exit(0);
	    }
	  }
      else
	{
	vtkErrorMacro("Client Not Connected.\n");
	}
      }
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
    int len = 16;
    char* msg1 = "StartTracking";
    
    if(this->SocketCommunicator->GetIsConnected()>0 )
      {
      if(!this->SocketCommunicator->Send(&len, 1, 1, 11) )
	{
	}
      else
	{
	if( !this->SocketCommunicator->Send(msg1, len, 1, 22))
	  {
	  vtkErrorMacro(" Could not send the message\n");
	  exit(0);
	  }
	else
	  {
	  // wait to receive the Tool Information acknowledgement
	  char *msg = "InternalStartTracking";
	  const int maxMessages = 50;
	  int messageNum = 0;
	  for (messageNum = 0; 
	       messageNum < maxMessages && 
		 strcmp( msg, "InternalStartTrackingSuccessful" ) ;
	       messageNum++)
	    {
	    int rlen[1] = {0};
	    if( this->SocketCommunicator->Receive(rlen, 1, 1, 11) )
	      {
	      char *rmsg = new char [rlen[0]];
	      if(!this->SocketCommunicator->Receive(rmsg, rlen[0], 1, 22) )
		{
		vtkErrorMacro("Could not RecieveToolInfo");
		exit(0);
		}
	      else
		{
		this->InterpretCommands( rmsg );
		msg = NULL;
		msg = new char [rlen[0]];
		memcpy(msg, rmsg, rlen[0]);
		}
	      }
	    }
	  if (messageNum == maxMessages)
	    {
	    vtkErrorMacro("Waited to receive \"EndEnabledToolPort\" "
			  "but it never came");
	    }
	  this->Tracking = 1;
	  }
	}
      }
    this->UpdateMutex->Lock();
    this->ThreadId = this->Threader->SpawnThread((vtkThreadFunctionType)\
                                                 &vtkTrackerThread,this);
    this->LastUpdateTime = this->UpdateTime.GetMTime();
    this->UpdateMutex->Unlock();
    } //end client
  
  //server / normal
  else if(this->ServerMode || !this->RemoteAddress) 
    {
    this->Tracking = this->InternalStartTracking();
    
    if(this->ServerMode)
      {
      if(this->SocketCommunicator->GetIsConnected()>0)
	{
	char *msgText = "InternalStartTrackingSuccessful";
	int len = strlen(msgText);
	if( this->SocketCommunicator->Send(&len, 1, 1, 11) )
	  {
	  if(!this->SocketCommunicator->Send(msgText, len, 1, 22))
	    {
	    vtkErrorMacro(
	      "Could not send message: InternalStartTrackingSuccessful");
	    exit(0);
	    }
	  }
	else
	  {
	  vtkErrorMacro(
	    "Could not send length of InternalStartTrackingSuccessful");
	  exit(0);
	  }
	}
      else
	{
	vtkErrorMacro("Client Not Connected.\n");
	}
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
    if(!this->Tracking)
      {
      return;
      }
    int slen = 13;
    char *smsg = "StopTracking";
    if( this->SocketCommunicator->GetIsConnected()>0)
      {
      if ( this->SocketCommunicator->Send(&slen, 1, 1, 11))
	{
	if(!this->SocketCommunicator->Send(smsg, slen, 1, 22))
	  {
	  vtkErrorMacro("Could not Send the message StopTracking!\n");
	  exit(0);
	  }
	}
      else
	{
	vtkErrorMacro("Could not send the length of  StopTracking!\n");
	exit(0);
	}
      this->Threader->TerminateThread(this->ThreadId);
      this->ThreadId = -1;
      int len[1] = {0};
      if ( this->SocketCommunicator->Receive(len, 1, 1, 11))
	{
	char *msg = new char [len[0]];
	if( !this->SocketCommunicator->Receive(msg, len[0], 1, 22))
	  {
	  vtkErrorMacro("Could not receive InternalStopTrackingSuccessful");
	  }
	
	else
	  {
	  this->InterpretCommands(msg);//ca2->GetPointer(0));
	  }
	}
      }
    else
      {
      vtkErrorMacro("Not connected to the Server. \n");
      }
    return;
    }
  // normal and server 
  if ( this->Tracking && this->ThreadId != -1 )
    {
    this->Threader->TerminateThread(this->ThreadId);
    this->ThreadId = -1;
    
    this->InternalStopTracking();
    this->Tracking = 0;
    if(this->ServerMode)
      {
      if(this->SocketCommunicator->GetIsConnected()<=0)
	{
	vtkErrorMacro("Client not Connected\n");
	return ;
	}
      int len = 31;
      if(this->SocketCommunicator->Send(&len, 1, 1, 11))
	{
	char *msg = "InternalStopTrackingSuccessful";
	if(!this->SocketCommunicator->Send(msg, 31, 1, 22))
	  {
	  vtkErrorMacro("Could not send InternalStopTrackingSuccessful\n");
	  exit(0);
	  }
	}
      else
	{
	vtkErrorMacro("Could not send length\n");
	exit(0);
	}
      }
    }
}

//----------------------------------------------------------------------------
void vtkTracker::Update()
{
  if (!this->Tracking)
    { 
    return; 
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
    if(this->SocketCommunicator->GetIsConnected()>0)
      {
      int len = 12;
      if (this->SocketCommunicator->Send( &len, 1, 1, 11 ))
	{
	char *msg = "Disconnect";
	if(!this->SocketCommunicator->Send( msg, len, 1, 22))
	  {
	  vtkErrorMacro("Could not receive message text\n");
	  exit(0);
	  }
	}
      else
	{
	vtkErrorMacro("Not Connected to server. \n");
	}
      }
    }
  else
    {
    vtkErrorMacro("Disconnect can be called from Client Tracker only !\n");
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::StartServer()
{
  while (1)
    { 
    if(this->SocketCommunicator->GetIsConnected())
      {
      exit(0);
      }
      if(this->SocketCommunicator->WaitForConnection(this->NetworkPort))
      {
      this->ClientConnected = 1;
      while( this->ClientConnected )
	{
	int len[1] = {0};
	if (this->SocketCommunicator->Receive( len, 1, 1, 11 ))
	  {
	  int msglen = static_cast<int>(len[0]);
	  char *msg = new char [msglen];
	  if(this->SocketCommunicator->Receive( msg, msglen, 1, 22))
	    {
	    this->InterpretCommands( msg );
	    }
	  else
	    {
	    vtkErrorMacro("Could not receive message text\n");
	    exit(0);
	    }
	  }
	else
	  {
	  vtkErrorMacro("Could not receive length.");
	  exit(0);
	  }
	}
      }
    }
}

//-----------------------------------------------------------------------------
void vtkTracker::InterpretCommands( char *messageText )
{
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
void vtkTracker::ConvertMessageToBuffer( double *da, 
					 double *vals, vtkMatrix4x4 *matrix )
{
  double elements[16];
  vals[0] = da[0];
  vals[1] = da[1];
  vals[2] = da[2];
  for( int i= 0; i<16; i++)
    {
    elements[i]=da[i+3];
    }
  matrix->DeepCopy(elements);
}

//-----------------------------------------------------------------------------
void vtkTracker::ConvertBufferToMessage( int tool, vtkMatrix4x4 *matrix, 
					 long flags, double ts,
					 double *msg )
{
  if(!msg)
    {
    msg = new double[19];
    }
  msg[0] = static_cast<double>(tool);
  msg[1] = static_cast<double>(flags);
  msg[2] = ts;
  int k = 3;
  for( int i = 0; i < 4; i++)
    {
    for( int j =0; j < 4; j++)
      {
      msg[k++] = matrix->GetElement(i,j);
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
