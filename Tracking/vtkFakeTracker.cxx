/* ============================================================================

  File: vtkFakeTracker.cxx
  Author: Kyle Charbonneau <kcharbon@imaging.robarts.ca>
  Language: C++
  Description: 
    This class represents a fake tracking system with tools that have
    predetermined behaviour. This allows someonew who doesn't have access to
    a tracking system to test code that relies on having one active.

============================================================================ */
#include "vtkFakeTracker.h"
#include "vtkObjectFactory.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkTrackerTool.h"

#include "vtkTimerLog.h"

vtkFakeTracker* vtkFakeTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkFakeTracker");
  if(ret)
    {
    return (vtkFakeTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkFakeTracker;
}

vtkFakeTracker::vtkFakeTracker() 
  {
#if (VTK_MAJOR_VERSION <= 4)
  this->TimeStamp = vtkTimerLog::GetCurrentTime();
#else
  this->TimeStamp = vtkTimerLog::GetUniversalTime();
#endif
  this->Frame = 0;
  this->InternalTransform = vtkTransform::New();
  this->SerialPort = 0;
  this->SetNumberOfTools(4);

  // Setup tool info for fake tools
  this->Tools[0]->SetToolType("Marker");
  this->Tools[0]->SetToolRevision("1.3");
  this->Tools[0]->SetToolManufacturer("ACME Inc.");
  this->Tools[0]->SetToolPartNumber("Stationary");
  this->Tools[0]->SetToolSerialNumber("A34643");

  this->Tools[1]->SetToolType("Pointer");
  this->Tools[1]->SetToolRevision("1.1");
  this->Tools[1]->SetToolManufacturer("ACME Inc.");
  this->Tools[1]->SetToolPartNumber("Rotate");
  this->Tools[1]->SetToolSerialNumber("B3464C");

  this->Tools[2]->SetToolType("Pointer");
  this->Tools[2]->SetToolRevision("1.1");
  this->Tools[2]->SetToolManufacturer("ACME Inc.");
  this->Tools[2]->SetToolPartNumber("Rotate");
  this->Tools[2]->SetToolSerialNumber("Q45P5");

  this->Tools[3]->SetToolType("Pointer");
  this->Tools[3]->SetToolRevision("2.0");
  this->Tools[3]->SetToolManufacturer("ACME Inc.");
  this->Tools[3]->SetToolPartNumber("Spin");
  this->Tools[3]->SetToolSerialNumber("Q34653");

  }

vtkFakeTracker::~vtkFakeTracker()
  {
  this->InternalTransform->Delete();
  }

int vtkFakeTracker::Probe()
  {
  return 1;
  }

int vtkFakeTracker::InternalStartTracking()
  {
  this->Frame = 0;
  return 1;
  }

int vtkFakeTracker::InternalStopTracking()
  {
  return 1;
  }

// Spins the tools around different axis to fake movement
void vtkFakeTracker::InternalUpdate()
  {
  if (this->Frame++ > 355559)
    {
    this->Frame = 0;
    }

  for (int tool = 0; tool < 4; tool++) 
    {
    int flags = 0;
      
    int rotation = this->Frame/1000;

    switch (tool)
      {
      case 0:
        // This tool is stationary
        this->InternalTransform->Identity();
        this->InternalTransform->Translate(0, 150, 200);
        break;
      case 1:
        // This tool rotates about a path on the Y axis
        this->InternalTransform->Identity();
        this->InternalTransform->RotateY(rotation);
        this->InternalTransform->Translate(0, 300, 0);
        break;
      case 2:
        // This tool rotates about a path on the X axis
        this->InternalTransform->Identity();
        this->InternalTransform->RotateX(rotation);
        this->InternalTransform->Translate(0, 300, 200);
        break;
      case 3:
        // This tool spins on the X axis
        this->InternalTransform->Identity();
        this->InternalTransform->Translate(100, 300, 0);
        this->InternalTransform->RotateX(rotation);
        break;
      }
    
    this->ToolUpdate(tool,this->InternalTransform->GetMatrix(),flags,this->TimeStamp);   
    //this->TimeStamp++;
    this->TimeStamp += 0.1;
    }
  }

