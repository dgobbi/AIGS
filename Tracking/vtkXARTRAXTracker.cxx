/*=========================================================================

  Program:   XartraxTracking for VTK
  Module:    $RCSfile: vtkXARTRAXTracker.cxx,v $
  Creator:   Chris Wedlake <cwedlake@imaging.robarts.ca>
  Language:  C++
  Author:    $Author: cwedlake $
  Date:      $Date: 2003/09/05 17:46:59 $
  Version:   $Revision: 1.1 $

==========================================================================

Copyright (c) 2000-2002
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
#include <ctype.h>
#include <iostream.h>
#include "polaris.h"
#include "polaris_math.h"
#include "vtkMath.h"
#include "vtkTimerLog.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkTrackerTool.h"
#include "vtkObjectFactory.h"
#include "XarTraXAPI.h"
#include "vtkTrackerBuffer.h"
#include "vtkPOLARISTracker.h"
#include "vtkXARTRAXTracker.h"
#include "vtkFrameToTimeConverter.h"

//----------------------------------------------------------------------------
vtkXARTRAXTracker* vtkXARTRAXTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkXARTRAXTracker");
  if(ret)
    {
    return (vtkXARTRAXTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkXARTRAXTracker;
}

vtkXARTRAXTracker::vtkXARTRAXTracker()
{
  XarTraXPort = -1;
  infraON = false;
  visibleON = false;
  strcpy(XarTraXFile, "ta001.rom");
  this->NumberOfStray = 0;

  for (int j =0; j < XAR_MAXSTRAY; j++) {
	this->Stray[j] = vtkTrackerTool::New();
    this->Stray[j]->SetTracker(this);
    this->Stray[j]->SetToolPort(0);
	this->Stray[j]->SetToolManufacturer("PASSIVE_STRAY");
	this->Stray[j]->SetToolType("1000001");
  }
}

//----------------------------------------------------------------------------
vtkXARTRAXTracker::~vtkXARTRAXTracker() 
{
  //xarCLOSE(this->Polaris);
  if (this->Tracking)
  {
    this->StopTracking();
  }
this->UpdateMutex->Lock();
  for (int i = 0; i < XAR_MAXSTRAY; i++)
  { 
    this->Stray[i]->SetTracker(NULL);
    this->Stray[i]->Delete();
  }
this->UpdateMutex->Unlock();

}

int vtkXARTRAXTracker::Probe()
{
  int errnum = PL_OPEN_ERROR;;

  if (this->IsPOLARISTracking) {
    return 1;
    }

  // if SerialPort is set to -1, then probe all serial ports
  if (this->SerialPort < 0) {
    for (int i = 0; i < 4; i++) {
       char *devicename = plDeviceName(i);
       if (devicename) {
          errnum = xarProbe(devicename);
		  if (errnum != 0)
		     errnum = xarProbe(devicename);
          if (errnum == PL_OKAY) {
             this->SerialPort = i+1;
             break;
		  }
        }
    }
  }
  else {
    char *devicename = plDeviceName(this->SerialPort-1);
    if (devicename) {
      errnum = xarProbe(devicename);
	  if (errnum != 0)
	     errnum = xarProbe(devicename);
    }
  }

  // if probe was okay, then send VER:0 to identify device
  if (errnum == PL_OKAY) {
    this->Polaris = plOpen(plDeviceName(this->SerialPort-1));
	if (strcmp(plVER(this->Polaris,0), "") == 0) {
		plClose(this->Polaris);
		return 0;
	}
    if (this->Polaris) {
      if( atoi(xarINIT(this->Polaris)) ) {
		plClose(this->Polaris);
		this->Polaris = 0;
		return 0;
	  }
	  else {
	    this->SetVersion(plVER(this->Polaris,0));
		plClose(this->Polaris);
		this->Polaris = 0;
      }
	}
    return 1;
  }
  return 0;
  //*/
} 

//#######################################################################################
// SURFACE SCANNING
//#######################################################################################

int vtkXARTRAXTracker::Scan(char *OutFileName,	double XLength,	double YLength,	double Increment, int maxRange, int minRange) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		xarVisibleOn(this->Polaris);
		xarInfraOn(this->Polaris);
		int value = xarRasterScan(this->Polaris, OutFileName, XLength, YLength, Increment, maxRange, minRange);
		if (!(visibleON))
			xarVisibleOff(this->Polaris);
		if (!(infraON))
			xarInfraOff(this->Polaris);
		this->UpdateMutex->Unlock();
		return value;
	}
	return 1;
}

int vtkXARTRAXTracker::ScanPos(char *OutFileName, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, double Increment, int maxRange, int minRange) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		xarVisibleOn(this->Polaris);
		xarInfraOn(this->Polaris);
		int value = xarRasterScan2(this->Polaris, OutFileName, topLeftX, topLeftY, bottomRightX, bottomRightY, Increment, maxRange, minRange);
		if (!(visibleON))
			xarVisibleOff(this->Polaris);
		if (!(infraON))
			xarInfraOff(this->Polaris);
		this->UpdateMutex->Unlock();
		return value;
	}
	return 1;
}

//#######################################################################################
// LASER STATES
//#######################################################################################

int vtkXARTRAXTracker::VisibleOn() {
	if (this->Polaris != 0) {
		visibleON = true;
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarVisibleOn(this->Polaris), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;

}

int vtkXARTRAXTracker::VisibleOff() {
	if (this->Polaris != 0) {
		visibleON = false;
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarVisibleOff(this->Polaris), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::InfraOn() {
	if (this->Polaris != 0) {
		infraON = true;
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarInfraOn(this->Polaris), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::InfraOff() {
	if (this->Polaris != 0) {
		infraON = false;
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarInfraOff(this->Polaris), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::IsVisibleOn() {
	return visibleON;
}

int vtkXARTRAXTracker::IsInfraOn() {
	return infraON;
}


//#######################################################################################
// POSITIONING
//#######################################################################################

/*
int vtkXARTRAXTracker::DegPosition(double X, double Y) {
	this->UpdateMutex->Lock();
	strncpy(this->CommandReply, xarSetDegPos(this->Polaris, X, Y), VTK_POLARIS_REPLY_LEN-1);
	this->UpdateMutex->Unlock();
	return atoi(this->CommandReply);
}
*/

int vtkXARTRAXTracker::SetPosition(double X, double Y) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarSetDegPosA(this->Polaris, X,Y, &(this->XMirror), &(this->YMirror)), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::SetPosition(char * X, char * Y) {
	if (this->Polaris != 0) {
		double x = atof(X);
		double y = atof(Y);
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarSetDegPosA(this->Polaris, x,y, &(this->XMirror), &(this->YMirror)), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::SetXPos(double X) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarDPositionA(this->Polaris, XAR_CHANNEL_X, XAR_UNIT_DEG, X, &(this->XMirror),(int)visibleON, (int)infraON), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::SetYPos(double Y) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarDPositionA(this->Polaris, XAR_CHANNEL_Y, XAR_UNIT_DEG, Y, &(this->YMirror), (int)visibleON, (int)infraON), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::SetXPos(char * X) {
	if (this->Polaris != 0) {
		double x = atof(X);
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarDPositionA(this->Polaris, XAR_CHANNEL_X, XAR_UNIT_DEG, x, &(this->XMirror), (int)visibleON, (int)infraON), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}

int vtkXARTRAXTracker::SetYPos(char * Y) {
	if (this->Polaris != 0) {
		double y = atof(Y);
		this->UpdateMutex->Lock();
		strncpy(this->CommandReply, xarDPositionA(this->Polaris, XAR_CHANNEL_Y, XAR_UNIT_DEG, y, &(this->YMirror), (int)visibleON, (int)infraON), VTK_POLARIS_REPLY_LEN-1);
		this->UpdateMutex->Unlock();
		return atoi(this->CommandReply);
	}
	return 1;
}


//#######################################################################################
// ANGLE CALCULATIONS
//#######################################################################################

int vtkXARTRAXTracker::GetAngleToTool(int tool, double *X, double *Y) {
	if (this->Polaris != 0) {
		double transform[8];
	    int port = ((tool < 3) ? ('1' + tool) : ('A' + tool - 3)); 
		int status;
		// get the transforms for all tools from the POLARIS
	
		this->UpdateMutex->Lock();
			plGX(this->Polaris,PL_XFORMS_AND_STATUS| PL_PASSIVE_EXTRA);
		this->UpdateMutex->Unlock();

		this->UpdateMutex->Lock();
			status = plGetGXTransform(this->Polaris, port, transform);
		this->UpdateMutex->Unlock();

		if (status == PL_OKAY) {
			vtkMatrix4x4 * invertedWorld = vtkMatrix4x4::New();
			invertedWorld->DeepCopy(this->GetWorldCalibrationMatrix());
			invertedWorld->Invert();
			xarPositionToAngle(transform[4],transform[5],transform[6],_OHAT,*X,*Y, invertedWorld);
			this->UpdateMutex->Lock();
				xarSetDegPosA(this->Polaris, *X, *Y, &(this->XMirror), &(this->YMirror));
			this->UpdateMutex->Unlock();
			return PL_OKAY;
		}
		return status;
	}
	return 1;
}

double *vtkXARTRAXTracker::FindAngleToPoint(double X, double Y, double Z) {
	static double ReturnValue[2];
    this->FindAngleToPoint(X,Y,Z, ReturnValue); return ReturnValue; }

double *vtkXARTRAXTracker::FindAngleToPoint(float X, float Y, float Z) {
	static double ReturnValue[2];
    this->FindAngleToPoint(X,Y,Z, ReturnValue); return ReturnValue; }

void vtkXARTRAXTracker::FindAngleToPoint(double X, double Y, double Z, double angle[2]) {
	vtkMatrix4x4 * invertedWorld = vtkMatrix4x4::New();
	invertedWorld->DeepCopy(this->GetWorldCalibrationMatrix());
	invertedWorld->Invert();
	xarPositionToAngle(X,Y,Z,_OHAT,angle[0],angle[1], invertedWorld);
}

void vtkXARTRAXTracker::FindAngleToPoint(float X, float Y, float Z, double angle[2]) {
	vtkMatrix4x4 * invertedWorld = vtkMatrix4x4::New();
	invertedWorld->DeepCopy(this->GetWorldCalibrationMatrix());
	invertedWorld->Invert();
	xarPositionToAngle(X,Y,Z,_OHAT,angle[0],angle[1], invertedWorld);
}

//#######################################################################################
// ARBITRARY WAVEFORM PROJECTION
//#######################################################################################

int vtkXARTRAXTracker::AProjection(char *filename, double time, int repeats, double maxAngle) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		int value = xarALaserProject(this->Polaris, filename, time, repeats, maxAngle);
		this->UpdateMutex->Unlock();
		this->SetPosition(0.0,0.0);
		return value;
	}
	return 1;
}


int vtkXARTRAXTracker::AProjectionDeg(char * filename, double time, int repeats) {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		int value = xarALaserProjectDeg(this->Polaris, filename, time, repeats);
		this->UpdateMutex->Unlock();
		this->SetPosition(0.0,0.0);
		return value;
	}
	return 1;
}


int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats) {

	double newTransform[4][4] = {1,0,0,0, 0,1,0,0 ,0,0,1,0 ,0,0,0,1} ;
	return AProjectionXYZ(filename, time, repeats, newTransform);
}



int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, float transform[4][4]) {

	double newTransform[4][4];
	for (int i = 0; i < 4; i++) 
		for (int j = 0; j < 4; j++) 
			newTransform[i][j] = (double)transform[i][j];

	return AProjectionXYZ(filename, time, repeats, newTransform);
}

int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, double transform[4][4]) {
	char szX[50], szY[50], szZ[50];
	char readline[150];
	double originalTransform[15][3];
	int counter = 0;
	FILE *fileIN;

	if (this->Polaris != 0) {
		if (!(fileIN = fopen(filename, "r")))
			return XAR_FILEERROR; //error_value?
	
		while (fgets(readline, 80, fileIN) != NULL) {
			sscanf(readline, "%s\t%s\t%s\t%s\t%s", &szX, &szY, &szZ);
			if (szX[0] == '#')
				break;
			originalTransform[counter][0] = atof(szX);
			originalTransform[counter][1] = atof(szY);
			originalTransform[counter][2] = atof(szZ);
			counter++;
		}
		fclose(fileIN);

		vtkMatrix4x4 * invertedWorld = vtkMatrix4x4::New();
		invertedWorld->DeepCopy(this->GetWorldCalibrationMatrix());
		invertedWorld->Invert();
		this->UpdateMutex->Lock();
		int value = xarALaserProjectXYZ(this->Polaris, filename, time, repeats,_OHAT, transform, invertedWorld);
		this->UpdateMutex->Unlock();
		this->SetPosition(0.0,0.0);
		return value;
	}
	return 1;
}

int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, 
				   double transform00, double transform01,double transform02,double transform03,
				   double transform10, double transform11,double transform12,double transform13,
				   double transform20, double transform21,double transform22,double transform23,
				   double transform30, double transform31,double transform32,double transform33) {

	double newTransform[4][4];
	newTransform[0][0] = transform00;
	newTransform[0][1] = transform01;
	newTransform[0][2] = transform02;
	newTransform[0][3] = transform03;
	newTransform[1][0] = transform10;
	newTransform[1][1] = transform11;
	newTransform[1][2] = transform12;
	newTransform[1][3] = transform13;
	newTransform[2][0] = transform20;
	newTransform[2][1] = transform21;
	newTransform[2][2] = transform22;
	newTransform[2][3] = transform23;
	newTransform[3][0] = transform30;
	newTransform[3][1] = transform31;
	newTransform[3][2] = transform32;
	newTransform[3][3] = transform33;

	return AProjectionXYZ(filename, time, repeats, newTransform);
}

int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, 
				   float transform00, float transform01, float transform02,float transform03,
				   float transform10, float transform11, float transform12,float transform13,
				   float transform20, float transform21, float transform22,float transform23,
				   float transform30, float transform31, float transform32,float transform33) {

	double newTransform[4][4];
	newTransform[0][0] = (double)transform00;
	newTransform[0][1] = (double)transform01;
	newTransform[0][2] = (double)transform02;
	newTransform[0][3] = (double)transform03;
	newTransform[1][0] = (double)transform10;
	newTransform[1][1] = (double)transform11;
	newTransform[1][2] = (double)transform12;
	newTransform[1][3] = (double)transform13;
	newTransform[2][0] = (double)transform20;
	newTransform[2][1] = (double)transform21;
	newTransform[2][2] = (double)transform22;
	newTransform[2][3] = (double)transform23;
	newTransform[3][0] = (double)transform30;
	newTransform[3][1] = (double)transform31;
	newTransform[3][2] = (double)transform32;
	newTransform[3][3] = (double)transform33;

	return AProjectionXYZ(filename, time, repeats, newTransform);
}

int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, double transform[16]) {
	double newTransform[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				newTransform[i][j] = transform[(4*i)+j];
	return AProjectionXYZ(filename, time, repeats, newTransform);
}

int vtkXARTRAXTracker::AProjectionXYZ(char * filename, double time, int repeats, float transform[16]) {
	double newTransform[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				newTransform[i][j] = (double)transform[(4*i)+j];
	return AProjectionXYZ(filename, time, repeats, newTransform);
}

//#######################################################################################
// COMPLETETION COMMANDS
//#######################################################################################

int vtkXARTRAXTracker::AFinish() {
	char * message;
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
		  message =xarAComplete(this->Polaris);
		this->UpdateMutex->Unlock();
		if(atoi(message))
			return atoi(message);

		this->UpdateMutex->Lock();
		  message =xarExecute(this->Polaris);
		this->UpdateMutex->Unlock();
		if(atoi(message))
			return atoi(message);

		return 0;
	}
	return 1;
}

int vtkXARTRAXTracker::DFinish() {
	if (this->Polaris != 0) {
		this->UpdateMutex->Lock();
			xarDComplete(this->Polaris);
			xarExecute(this->Polaris);
		this->UpdateMutex->Unlock();
		return 0;
	}
	return 1 ;
}

//#######################################################################################
// STREAM READING OF TOOLS
//#######################################################################################

int vtkXARTRAXTracker::StreamReadDeg(char * filename, int tool, int totalPoints, int delayBetweenPoints) {
	
	FILE *fileOut;
	// open the data file
	if (this->Polaris != 0) {
		if (!(fileOut = fopen(filename, "w")))
			return 1;

		vtkTransform * transform;
		double position[3];
		double angle[2];


		for(int i=0; i < totalPoints; i++) {
			this->UpdateMutex->Lock();
			this->Update();
			this->UpdateMutex->Unlock();
			transform = this->GetTool(tool)->GetTransform();
			if (this->GetTool(tool)->IsMissing() == 0) {
				transform->GetPosition(position);
				FindAngleToPoint(position[0], position[1], position[2],angle);
				SetPosition(angle[0], angle[1]);
				if (position[2] != 0)
					fprintf(fileOut,"%f\t%f\t%d\t%d\n", angle[0], angle[1], IsVisibleOn(), IsInfraOn());
			}
			Sleep(delayBetweenPoints);
		}
		fclose(fileOut);
		return 0;
	}
	return 1;
}

int vtkXARTRAXTracker::StreamReadXYZLandmarks(char * filename, int tool, int totalPoints, int delayBetweenPoints) {
	
	FILE *fileOut;
	// open the data file
	if (this->Polaris != 0) {
		if (!(fileOut = fopen(filename, "a+")))
			return 1;

		vtkTransform * transform;
		double position[3];
		double tempX, tempY, tempZ;
		double angle[2];

		for(int i=0; i < totalPoints; i++) {
			this->UpdateMutex->Lock();
			this->Update();
			this->UpdateMutex->Unlock();
			transform = this->GetTool(tool)->GetTransform();
			if (this->GetTool(tool)->IsMissing() == 0) {
				transform->GetPosition(position);
				FindAngleToPoint(position[0], position[1], position[2],angle);
                tempX = position[0];
				tempY = position[1];
				tempZ = position[2];
				vtkMatrix4x4 * invertedWorld = vtkMatrix4x4::New();
				invertedWorld->DeepCopy(this->GetWorldCalibrationMatrix());
				invertedWorld->Invert();
				position[0] = tempX*invertedWorld->GetElement(0,0)+ tempY*invertedWorld->GetElement(0,1)+ tempZ*invertedWorld->GetElement(0,2)+ invertedWorld->GetElement(0,3);
				position[1] = tempX*invertedWorld->GetElement(1,0)+ tempY*invertedWorld->GetElement(1,1)+ tempZ*invertedWorld->GetElement(1,2)+ invertedWorld->GetElement(1,3);
				position[2] = tempX*invertedWorld->GetElement(2,0)+ tempY*invertedWorld->GetElement(2,1)+ tempZ*invertedWorld->GetElement(2,2)+ invertedWorld->GetElement(2,3);
				SetPosition(angle[0], angle[1]);
				if (position[2] != 0)
				  fprintf(fileOut,"%f\t%f\t%f\t%d\t%d\n", position[0], position[1], position[2], IsVisibleOn(), IsInfraOn());
			}
			Sleep(delayBetweenPoints);
		}
		fclose(fileOut);
		return 0;
	}
	return 1;
}

int vtkXARTRAXTracker::StreamReadXYZ(char * filename, int tool, int totalPoints, int delayBetweenPoints) {
	
	FILE *fileOut;
	// open the data file
	if (this->Polaris != 0) {
		if (!(fileOut = fopen(filename, "a+")))
			return 1;

		vtkTransform * transform;
		double position[3];
		double angle[2];

		for(int i=0; i < totalPoints; i++) {
			this->UpdateMutex->Lock();
			this->Update();
			this->UpdateMutex->Unlock();
			transform = this->GetTool(tool)->GetTransform();
			if (this->GetTool(tool)->IsMissing() == 0) {
				transform->GetPosition(position);
				FindAngleToPoint(position[0], position[1], position[2],angle);
				SetPosition(angle[0], angle[1]);
				if (position[2] != 0)
				  fprintf(fileOut,"%f\t%f\t%f\t%d\t%d\n", position[0], position[1], position[2], IsVisibleOn(), IsInfraOn());
			}
			Sleep(delayBetweenPoints);
		}
		fclose(fileOut);
		return 0;
	}
	return 1;
}


//#######################################################################################
// XARTRAX TESTING
//#######################################################################################
/*
void vtkXARTRAXTracker::FlatTest(char * filename, int tool) {
   double angleX, angleY;
   for (int position=0; position < 6; position++) {
		switch(position) {
		case 0:
			GetAngleToTool(tool, angleX, angleY);

			goTo(cons*(0), (0));
			file << (0*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
			file << (0*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
			file << (0*_bhat[2])+C[2] + (0*_ahat[2])<< "\t";

std::cout << (0*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
std::cout << (0*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
std::cout << (0*_bhat[2])+C[2] + (0*_ahat[2])<< "\n";

			std::cout << endl;
			break;
		case 1:
			goTo(cons*(250), (0));
			file <<  (250*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
			file <<  (250*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
			file <<  (250*_bhat[2])+C[2] + (0*_ahat[2])<< "\t";


std::cout << (250*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
std::cout << (250*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
std::cout << (250*_bhat[2])+C[2] + (0*_ahat[2])<< "\t";
std::cout << endl;
			break;
		case 2:
			goTo(cons*(-250), (0));
			file <<  (-250*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
			file <<  (-250*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
			file <<  (-250*_bhat[2])+C[2] + (0*_ahat[2])<< "\t";


std::cout << (-250*_bhat[0])+C[0] + (0*_ahat[0]) << "\t";
std::cout << (-250*_bhat[1])+C[1] + (0*_ahat[1]) << "\t"; 
std::cout << (-250*_bhat[2])+C[2] + (0*_ahat[2])<< "\t";
std::cout << endl;
			break;
		case 3:
			goTo(cons*(0), (225));
			file <<  (0*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
			file <<  (0*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
			file <<  (0*_bhat[2])+C[2] + (225*_ahat[2])<< "\t";
std::cout << (0*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
std::cout << (0*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
std::cout << (0*_bhat[2])+C[2] + (225*_ahat[2]);
std::cout << endl;
			break;
		case 4:
			goTo(cons*(250), (225));
			file << (250*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
			file << (250*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
			file << (250*_bhat[2])+C[2] + (225*_ahat[2])<< "\t";
std::cout << (250*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
std::cout << (250*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
std::cout << (250*_bhat[2])+C[2] + (225*_ahat[2])<< "\t";
std::cout << endl;
			break;
		case 5:
			goTo(cons*(-250), (225));
			file << (-250*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
			file << (-250*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
			file << (-250*_bhat[2])+C[2] + (225*_ahat[2])<< "\t";
std::cout << (-250*_bhat[0])+C[0] + (225*_ahat[0]) << "\t";
std::cout << (-250*_bhat[1])+C[1] + (225*_ahat[1]) << "\t"; 
std::cout << (-250*_bhat[2])+C[2] + (225*_ahat[2])<< "\t";
std::cout << endl;
			break;

		}


		for (int tools = 'A'; tools < 'C'; tools++) {			// TEMPARARY!!! MUST SET STRUCTURE TO STORE NUMBER OF HANDLES PER DEVICE
//			std::cout << "PREPARE DEVICE:" << (char)tools <<". Press Any key to continue. "<<endl;
//			std::cin >> test;
missing:
			status = readFromDevice(trans, tools);
//			if (!(status == NDI_MISSING)) {
				if (!file) { printf("Unable to access requested file\n"); exit(-1); }
				
				if (tools == 'A') {
					/*
					ndiTransformToMatrixd(trans, matrix);
					MatrixMulti(convert,matrix,answer);
					file << answer[12] << "\t" << answer[13] << "\t" << answer[14] << "\t" << trans[7] <<"\t";

					std::cout << "DEVICE A "<< answer[12] << "\t" << answer[13] << "\t" << answer[14] << "\t" << trans[7] <<"\n";
					//*//*
				}
				else {
					file << trans[4] << "\t" << trans[5] << "\t" << trans[6] << "\t" << trans[0];
					std::cout << "DEVICE B " <<trans[4] << "\t" << trans[5] << "\t" << trans[6] << "\t" << trans[0] << "\n";
				}
//			}
//			else {
//				if (tools == 'A') {
//				std::cout << "POINT WAS MISSING FROM DEVICE: " << (char)tools <<". Press Any key to continue. "<<endl;
//				std::cin >> test;
//				tools--;
				
//				}
				if (tools == 'B' && trans[0] == 0) {
					//tools--;
					goto missing;
				}

				if (test[0] == 'q')		return;
				else					continue;
//			}
		}
		file << endl;
	}
	_maxX = 550;
	_maxY = 275;
	_minX = 50;
	_minY = 25;

}
*/




//#######################################################################################
// GENERAL
//#######################################################################################

void vtkXARTRAXTracker::GetXarError() {
	if (this->Polaris != 0) {
		vtkErrorMacro(<< xarGetError(this->Polaris, this->CommandReply));
	}
	return;
}

void vtkXARTRAXTracker::Delay(int n) {
	Sleep(n);
}


//----------------------------------------------------------------------------
// Send a raw command to the tracking unit.
// If communication has already been opened with the POLARIS,
// then lock the mutex to get exclusive access and then
// send the command.
// Otherwise, open communication with the unit, send the command,
// and close communication.
char * vtkXARTRAXTracker::Command(const char *command)
{
  this->CommandReply[0] = '\0';

  if (command[0] == '_') {
    if (this->Polaris) {
      this->UpdateMutex->Lock();
      strncpy(this->CommandReply, xarCommand(this->Polaris, command), VTK_POLARIS_REPLY_LEN-1);
      this->CommandReply[VTK_POLARIS_REPLY_LEN-1] = '\0';
      this->UpdateMutex->Unlock();
	}
    else {
      this->Polaris = plOpen(plDeviceName(this->SerialPort-1));
      if (this->Polaris == 0) {
        vtkErrorMacro(<< plErrorString(PL_OPEN_ERROR));
	  }
      else {
  	    xarINIT(this->Polaris);
        strncpy(this->CommandReply, xarCommand(this->Polaris, command), VTK_POLARIS_REPLY_LEN-1);
        this->CommandReply[VTK_POLARIS_REPLY_LEN-1] = '\0';
        plClose(this->Polaris);
	  }
      this->Polaris = 0;
	}
	  return this->CommandReply;
  }
  else
	  return vtkPOLARISTracker::Command(command);
}

//#######################################################################################
// VTK AND TRACKING REQUIRED METHODS
//#######################################################################################


//----------------------------------------------------------------------------
int vtkXARTRAXTracker::InternalStartTracking()
{
  int errnum, tool;
  int baud;

  if (this->IsPOLARISTracking)
    {
    return 1;
    }

  switch (this->BaudRate)
    {
    case 9600: baud = PL_9600; break; 
    case 14400: baud = PL_14400; break; 
    case 19200: baud = PL_19200; break; 
    case 38400: baud = PL_38400; break; 
    case 57600: baud = PL_57600; break; 
    case 115200: baud = PL_115200; break;
    default:
      vtkErrorMacro(<< "Illegal baud rate");
      return 0;
    }

  this->Polaris = plOpen(plDeviceName(this->SerialPort-1));
  if (this->Polaris == 0) 
    {
    vtkErrorMacro(<< plErrorString(PL_OPEN_ERROR));
    return 0;
    }
  // initialize Polaris
  plINIT(this->Polaris);
  
  if (plGetError(this->Polaris))
    {
    plRESET(this->Polaris);
    errnum = plGetError(this->Polaris);
    if (errnum) 
      {
      vtkErrorMacro(<< plErrorString(errnum));
      plClose(this->Polaris);
      this->Polaris = 0;
      return 0;
      }
    plINIT(this->Polaris);
    if (errnum) 
      {
      vtkErrorMacro(<< plErrorString(errnum));
      plClose(this->Polaris);
      this->Polaris = 0;
      return 0;
      }
    }

  strncpy(this->CommandReply, xarINIT(this->Polaris), VTK_POLARIS_REPLY_LEN-1);
  errnum = atoi(this->CommandReply);
	if (errnum)
		GetXarError();

  this->SetVersion(plVER(this->Polaris,0));
  plCOMM(this->Polaris,baud,PL_8N1,PL_HANDSHAKE);
  errnum = plGetError(this->Polaris);
  if (errnum) 
    {
    vtkErrorMacro(<< plErrorString(errnum));
    plClose(this->Polaris);
    this->Polaris = 0;
    return 0;
    }

  Sleep(150);

  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
    if (VirtualSROM[tool]) {
      this->InternalLoadVirtualSROM(tool,VirtualSROM[tool]);
      }
    }

  this->EnableToolPorts();

  plTSTART(this->Polaris);
  errnum = plGetError(this->Polaris);
  if (errnum) 
    {
    vtkErrorMacro(<< plErrorString(errnum));
    plClose(this->Polaris);
    this->Polaris = 0;
    return 0;
    }

  int passive = PL_PASSIVE|PL_PASSIVE_EXTRA;

  if (this->Version[0] == 'A') // if Aurora, no passive
    {
    passive = 0;
    }
  // prime the system by sending an initial GX command
  plGX(this->Polaris,PL_XFORMS_AND_STATUS|PL_FRAME_NUMBER|passive);

  // for accurate timing
  this->Timer->Initialize();

  InfraOn();
  VisibleOn();
  this->UpdateMutex->Lock();
  xarFindOHAT(this->Polaris, _OHAT);
  this->UpdateMutex->Unlock();
  InfraOff();
  VisibleOff();

  //Sleep(500);
  this->IsPOLARISTracking = 1;

  return 1;
}

//----------------------------------------------------------------------------
int vtkXARTRAXTracker::InternalStopTracking()
{
  InfraOff();
  VisibleOff();
  if (this->Polaris == 0)
    {
    return 0;
    }

  int errnum, tool;
  this->UpdateMutex->Lock();
  plTSTOP(this->Polaris);
  errnum = plGetError(this->Polaris);
  this->UpdateMutex->Unlock();
  if (errnum) 
    {
    vtkErrorMacro(<< plErrorString(errnum));
    }
  this->IsPOLARISTracking = 0;
  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++)
    {
    if (VirtualSROM[tool])
      {
      this->ClearVirtualSROM(tool);
      }
    }
  this->DisableToolPorts();

  // return to default comm settings
  this->UpdateMutex->Lock();
  plCOMM(this->Polaris,PL_9600,PL_8N1,PL_NOHANDSHAKE);
  errnum = plGetError(this->Polaris);
  this->UpdateMutex->Unlock();
  if (errnum) 
    {
    vtkErrorMacro(<< plErrorString(errnum));
    }
  this->UpdateMutex->Lock();
    Sleep(500);
  //xarCLOSE(this->Polaris);
  plClose(this->Polaris);
  this->UpdateMutex->Unlock();
  this->Polaris = 0;
  return 1;
}

//----------------------------------------------------------------------------
// Important notes on the data collection rate of the POLARIS:
//
// The camera frame rate is 60Hz, and therefore the maximum data
// collection rate is also 60Hz.  The maximum data transfer rate
// to the computer is also 60Hz.
//
// Depending on the number of enabled tools, the data collection
// rate might be reduced.  Each of the active tools requires one
// camera frame, and all the passive tools (if any are enabled)
// collectively require one camera frame.
//
// Therefore if there are two enabled active tools, the data rate
// is reduced to 30Hz.  Ditto for an active tool and a passive tool.
// If all tools are passive, the data rate is 60Hz.  With 3 active
// tools and one or more passive tools, the data rate is 15Hz.
// With 3 active tools, or 2 active and one or more passive tools,
// the data rate is 20Hz.
//
// The data transfer rate to the computer is independent of the data
// collection rate, and there might be duplicated records.  The
// data tranfer rate is limited by the speed of the serial port
// and by the number of characters sent per data record.  If tools
// are marked as 'missing' then the number of characters that
// are sent will be reduced.

void vtkXARTRAXTracker::InternalUpdate()
{
  int errnum, tool;
  int status[VTK_POLARIS_NTOOLS];
  int absent[VTK_POLARIS_NTOOLS];
  unsigned long frame[VTK_POLARIS_NTOOLS];
  double transform[VTK_POLARIS_NTOOLS][8];
  double *referenceTransform = 0;
  char *message;
  long flags;
  const unsigned long mflags = PL_TOOL_IN_PORT | PL_INITIALIZED | PL_ENABLED;

  if (!this->IsPOLARISTracking) {
    vtkWarningMacro( << "called Update() when POLARIS was not tracking");
    return;
  }

  if (!this->Polaris) {
    vtkWarningMacro( << "called Update() when no Polaris Existed.");
    return;
  }
  // initialize transformations to identity
  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
    transform[tool][0] = 1.0;
    transform[tool][1] = transform[tool][2] = transform[tool][3] = 0.0;
    transform[tool][4] = transform[tool][5] = transform[tool][6] = 0.0;
    transform[tool][7] = 0.0;
  }
  
  // check to see if passive ports are being used
  int passive = 0;
  if (this->Version[0] != 'A') {
	  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
        if (this->VirtualSROM[tool]) {
			if (tool >= 3) {
				passive |= PL_PASSIVE | PL_PASSIVE_STRAY;
			}
			if (tool >= 6) {
				passive |= PL_PASSIVE_EXTRA | PL_PASSIVE_STRAY;
			}
		}
      }
  }
   // get the transforms for all tools from the POLARIS
  message = plGX(this->Polaris,PL_XFORMS_AND_STATUS|PL_FRAME_NUMBER| passive);
  //printf("%s\n", message);
  errnum = plGetError(this->Polaris);

  if (errnum) {
	  if (errnum == PL_BAD_CRC) { // CRC errors are common 
		vtkWarningMacro(<< plErrorString(errnum));
      }
      else {
		vtkErrorMacro(<< plErrorString(errnum));
      }
    return;
  }

  // default to incrementing frame count by one (in case there are
  // no transforms for any tools)
  unsigned long nextcount = 0;

  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
    int port = ((tool < 3) ? ('1' + tool) : ('A' + tool - 3)); 
    absent[tool] = plGetGXTransform(this->Polaris, port, transform[tool]);
    status[tool] = plGetGXPortStatus(this->Polaris, port);
    frame[tool] = plGetGXFrame(this->Polaris, port);
    if (!absent[tool] && frame[tool] > nextcount) { 
		// 'nextcount' is max frame number returned
		nextcount = frame[tool];
    }
  }

  double OldNumberOfStray = this->NumberOfStray;
  this->NumberOfStray = plGetGXNumberOfPassiveStrays(this->Polaris);
  this->NumberOfStray = this->NumberOfStray > XAR_MAXSTRAY ? XAR_MAXSTRAY : this->NumberOfStray;
  bool laserOnly;
  double laserNumber = -1;

  if (XarTraXPort != -1)
    laserOnly = true;
  else 
	laserOnly = false;
//  Displays ONE stray marker as the registered tool on this port.

  absent[XarTraXPort] = (this->NumberOfStray) == 0 ? absent[XarTraXPort] : 0;
  if (absent[XarTraXPort] != 2) {
	double tempStray[3];
	for (int strayloop = 0; strayloop < this->NumberOfStray; strayloop++) {
		plGetGXPassiveStray(this->Polaris, strayloop, tempStray);
		int tempLaserCheck = isxarLaser(tempStray[0], tempStray[1], tempStray[2]);
		if (tempLaserCheck){
			status[XarTraXPort] = PL_TOOL_IN_PORT | PL_INITIALIZED | PL_ENABLED | PL_PARTIALLY_IN_VOLUME;
			transform[XarTraXPort][0] = 1;
			transform[XarTraXPort][1] = transform[XarTraXPort][2] = transform[XarTraXPort][3] =  0;
			transform[XarTraXPort][4] = tempStray[0];
			transform[XarTraXPort][5] = tempStray[1];
			transform[XarTraXPort][6] = tempStray[2];
			if (frame[XarTraXPort] > nextcount)
				nextcount = frame[XarTraXPort];
			else 
				frame[XarTraXPort] = nextcount;			
		}
		if (laserOnly && tempLaserCheck) {
			laserNumber = strayloop;
			this->NumberOfStray = 1;
			break;
		}
	}	
	if (laserOnly && laserNumber == -1){
		this->NumberOfStray = 0;
	}
  }




  // if no transforms were returned, advance frame count by 1
  // (assume the POLARIS will be returning the empty records at
  // its maximum reporting rate of 60Hz)
  if (nextcount == 0) {
    nextcount = this->Timer->GetLastFrame() + 1;
  }

  // the timestamp is always created using the frame number of
  // the most recent transformation
  this->Timer->SetLastFrame(nextcount);
  double timestamp = this->Timer->GetTimeStampForFrame(nextcount);

  // check to see if any tools have been plugged in
  int need_enable = 0;
  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
    need_enable |= ((status[tool] & PL_TOOL_IN_PORT) && 
		    !this->PortEnabled[tool]);
  }

  if (need_enable) { // re-configure, a new tool has been plugged in
    this->EnableToolPorts();
  }
  else {
    for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
      this->PortEnabled[tool] = ((status[tool] & mflags) == mflags);
    }
  }

  if (this->ReferenceTool >= 0) { // copy reference tool transform
    referenceTransform = transform[this->ReferenceTool];
  }

  for (tool = 0; tool < VTK_POLARIS_NTOOLS; tool++) {
    // convert status flags from POLARIS to vtkTracker format
    int port_status = status[tool];
    flags = 0;
    if ((port_status & mflags) != mflags) {
      flags |= TR_MISSING;
      }
    else {
      if (absent[tool])						{ flags |= TR_OUT_OF_VIEW;  }
      if (port_status & PL_OUT_OF_VOLUME)	{ flags |= TR_OUT_OF_VOLUME; }
      if (port_status & PL_SWITCH_1_ON)		{ flags |= TR_SWITCH1_IS_ON; }
      if (port_status & PL_SWITCH_2_ON)		{ flags |= TR_SWITCH2_IS_ON; }
      if (port_status & PL_SWITCH_3_ON)		{ flags |= TR_SWITCH3_IS_ON; }
    }

    // if tracking relative to another tool
    if (this->ReferenceTool >= 0 && tool != this->ReferenceTool) {
      if (!absent[tool]) {
		if (absent[this->ReferenceTool]) {
			flags |= TR_OUT_OF_VIEW;
		}
		if (status[this->ReferenceTool] & PL_OUT_OF_VOLUME) {
			flags |= TR_OUT_OF_VOLUME;
		}
	  }
      // pre-multiply transform by inverse of relative tool transform
      plRelativeTransform(transform[tool],referenceTransform,transform[tool]);
    }
    plTransformToMatrixd(transform[tool],*this->SendMatrix->Element);
    this->SendMatrix->Transpose();

    // by default (if there is no camera frame number associated with
    // the tool transformation) the most recent timestamp is used.
    double tooltimestamp = timestamp;
    if ((!absent[tool] && frame[tool]) || tool == XarTraXPort) {
      // this will create a timestamp from the frame number      
      tooltimestamp = this->Timer->GetTimeStampForFrame(frame[tool]);
    }
    // send the matrix and flags to the tool

    this->ToolUpdate(tool,this->SendMatrix,flags,tooltimestamp);
  }

	// PASSIVE TOOL UPDATE

  double tempStray[3];
  double strayTransform[8];
  int stray=0;
  bool laserFound = false;
  double tooltimestamp = this->Timer->GetTimeStampForFrame(nextcount);

  if(!laserOnly) {
	for (; stray < this->NumberOfStray; stray++) {
	  	plGetGXPassiveStray(this->Polaris, stray, tempStray);
		strayTransform[0] = 1;
		strayTransform[1] = strayTransform[2] = strayTransform[3] =  0;
		strayTransform[4] = tempStray[0];
		strayTransform[5] = tempStray[1];
		strayTransform[6] = tempStray[2];

		
		if (this->ReferenceTool >= 0)
			plRelativeTransform(strayTransform,referenceTransform,strayTransform);

		plTransformToMatrixd(strayTransform,*this->SendMatrix->Element);
		this->SendMatrix->Transpose();
		// what should flags be?
		this->StrayUpdate(stray, this->SendMatrix,0,tooltimestamp);

		vtkTrackerTool *trackerTool = this->Stray[stray];
		trackerTool->Update();
		this->UpdateTimeStamp = trackerTool->GetTimeStamp();
	}
  }
  else if (laserNumber != -1) {
  	plGetGXPassiveStray(this->Polaris, laserNumber, tempStray);
	strayTransform[0] = 1;
	strayTransform[1] = strayTransform[2] = strayTransform[3] =  0;
	strayTransform[4] = tempStray[0];
	strayTransform[5] = tempStray[1];
	strayTransform[6] = tempStray[2];

	if (this->ReferenceTool >= 0)
		plRelativeTransform(strayTransform,referenceTransform,strayTransform);

	plTransformToMatrixd(strayTransform,*this->SendMatrix->Element);
	this->SendMatrix->Transpose();
	// what should flags be?
	this->StrayUpdate(stray, this->SendMatrix,0,tooltimestamp);

	vtkTrackerTool *trackerTool = this->Stray[0];
	trackerTool->Update();
	this->UpdateTimeStamp = trackerTool->GetTimeStamp();
    stray = 1;
  }

  for (; stray <= OldNumberOfStray && stray < 20; stray++) {
		this->StrayUpdate(stray, this->SendMatrix,TR_MISSING,0);

		vtkTrackerTool *trackerTool = this->Stray[stray];
		trackerTool->Update();
		this->UpdateTimeStamp = trackerTool->GetTimeStamp();
  }

  this->LastUpdateTime = this->UpdateTime.GetMTime();
}

int vtkXARTRAXTracker::isxarLaser(double X, double Y, double Z) {
	double tempX, tempY;
    xarPositionToAngle(X, Y, Z, _OHAT, tempX, tempY);
	if ( (fabs(tempX - this->XMirror) < 0.1) &&  (fabs(tempY - this->YMirror) < 0.1) ) {
		return 1;
	}
	return 0;
}

void vtkXARTRAXTracker::StrayUpdate(int tool, vtkMatrix4x4 *matrix, long flags,
			    double timestamp) 
{
  vtkTrackerBuffer *buffer = this->Stray[tool]->GetBuffer();

  buffer->Lock();
  buffer->AddItem(matrix, flags, timestamp);
  buffer->Unlock();
}


//----------------------------------------------------------------------------
void vtkXARTRAXTracker::LoadXarTraXROM(int port, const char *filename)
{
	if (XarTraXPort != -1) {
		vtkErrorMacro(<< "XarTraX is already started on this Polaris System");
	}

	XarTraXPort = port;
	strcpy(XarTraXFile, filename);
	LoadVirtualSROM(port, filename);
	this->GetTool(port)->SetToolManufacturer("XarTraX");
}

//----------------------------------------------------------------------------
void vtkXARTRAXTracker::ClearXarTraXROM()
{
	if (XarTraXPort == -1) {
		vtkErrorMacro(<< "XarTraX is not started on this Polaris System");
		return;
	}
	ClearVirtualSROM(XarTraXPort);
	XarTraXPort = -1;
}

vtkTrackerTool *vtkXARTRAXTracker::GetStray(int stray)
{
  if (stray < 0 || stray > XAR_MAXSTRAY) 
    {
	  return NULL;
    }
  return this->Stray[stray];
}
