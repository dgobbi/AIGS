/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkNDICertusTracker.cxx,v $
  Creator:   David Gobbi <dgobbi@cs.queensu.ca>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2008/06/17 03:59:34 $
  Version:   $Revision: 1.1 $

==========================================================================

[ Copyright Notice Goes Here ]

=========================================================================*/

#include <limits.h>
#include <float.h>
#include <math.h>
#include <ctype.h>

// NDI header files require this on Windows
#if defined(_WIN32) && !defined(__WINDOWS_H)
#define __WINDOWS_H
#include "Windows.h"
#endif

// requires NDIoapi/ndlib
#include "ndtypes.h"
#include "ndpack.h"
#include "ndopto.h"

// requires Atamai ndicapi
#include "ndicapi_math.h"

#include "vtkTimerLog.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkNDICertusTracker.h"
#include "vtkTrackerTool.h"
#include "vtkFrameToTimeConverter.h"
#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
// map values 0, 1, 2 to the proper Certus VLED state constant 
static VLEDState vtkNDICertusMapVLEDState[] = {
  VLEDST_OFF, VLEDST_ON, VLEDST_BLINK };

//----------------------------------------------------------------------------
vtkNDICertusTracker* vtkNDICertusTracker::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkNDICertusTracker");
  if(ret)
    {
    return (vtkNDICertusTracker*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkNDICertusTracker;
}

//----------------------------------------------------------------------------
vtkNDICertusTracker::vtkNDICertusTracker()
{
  this->Version = NULL;
  this->SendMatrix = vtkMatrix4x4::New();
  this->IsDeviceTracking = 0;
  this->NumberOfMarkers = 0;
  this->NumberOfRigidBodies = 0;
  this->SetNumberOfTools(VTK_CERTUS_NTOOLS);

  for (int i = 0; i < VTK_CERTUS_NTOOLS; i++)
    {
    this->PortHandle[i] = 0;
    this->PortEnabled[i] = 0;
    }

  // for accurate timing
  this->Timer = vtkFrameToTimeConverter::New();
  this->Timer->SetNominalFrequency(60.0);
}

//----------------------------------------------------------------------------
vtkNDICertusTracker::~vtkNDICertusTracker() 
{
  if (this->Tracking)
    {
    this->StopTracking();
    }
  this->SendMatrix->Delete();
  if (this->Version)
    {
    delete [] this->Version;
    }
  if (this->Timer)
    {
    this->Timer->Delete();
    }
}
  
//----------------------------------------------------------------------------
void vtkNDICertusTracker::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkTracker::PrintSelf(os,indent);

  os << indent << "SendMatrix: " << this->SendMatrix << "\n";
  this->SendMatrix->PrintSelf(os,indent.GetNextIndent());
  os << indent << "NumberOfRigidBodies: " << this->NumberOfRigidBodies << "\n";
  os << indent << "NumberOfMarkers: " << this->NumberOfMarkers << "\n";
}
  
//----------------------------------------------------------------------------
int vtkNDICertusTracker::PrintCertusError()
{
  char szNDErrorString[MAX_ERROR_STRING_LENGTH + 1];
  if (OptotrakGetErrorString(szNDErrorString,
                             MAX_ERROR_STRING_LENGTH+1) == 0)
    {
    vtkErrorMacro("Optotrack: " << szNDErrorString);
    }

  return 0;
}  

//----------------------------------------------------------------------------
int vtkNDICertusTracker::InitializeCertusSystem()
{
  // Variable to indicate that probe failed
  int successFlag = 1;  

  // Do the initial load.
  if (successFlag &&
      TransputerLoadSystem("system") != OPTO_NO_ERROR_CODE)
    {
    vtkErrorMacro("Call to Certus TransputerLoadSystem() failed");
    this->PrintCertusError();
    successFlag = 0;
    }

  if (successFlag)
    {
    // Wait for 1 second, according to the Certus documentation
#ifdef _WIN32
    // Double colon ensures that Window's sleep is used instead of
    // a sleep defined in some other namespace somewhere
    ::Sleep(1000);
#else
    // On UNIX we use nanosleep because it is standardized by POSIX.
    // In contrast, usleep() is not standard, and sleep() might be
    // implemented by a SIGALRM, causing compatibility issues if the
    // code uses signals for other things.
    struct timespec sleep_time, dummy;
    sleep_time.tv_sec = 1;
    sleep_time.tv_nsec = 0;
    nanosleep(&sleep_time,&dummy);
#endif
    }
  
  // Do the initialization
  if (successFlag &&
      TransputerInitializeSystem(0) != OPTO_NO_ERROR_CODE)
    { // optionally, use "OPTO_LOG_ERRORS_FLAG" argument to above
    vtkErrorMacro("Call to Certus TransputerInitializeSystem() failed");
    this->PrintCertusError();
    successFlag = 0;
    }
  
  // Load the standard camera parameters
  if (successFlag &&
      OptotrakLoadCameraParameters("standard") != OPTO_NO_ERROR_CODE)
    {
    vtkErrorMacro("Call to OptotrakLoadCameraParameters()  failed");
    this->PrintCertusError();
    successFlag = 0;
    }
  
  // Get the Optotrak status
  int nNumSensors;
  int nNumOdaus;
  int nFlags;
  if (successFlag &&
      OptotrakGetStatus(&nNumSensors,
                        &nNumOdaus,
                        NULL,
                        NULL,
                        NULL,
                        NULL, 
                        NULL,
                        NULL,
                        NULL,
                        NULL,
                        NULL,
                        NULL,
                        NULL,
                        &nFlags ) != OPTO_NO_ERROR_CODE)
    {
    vtkErrorMacro("Call to OptotrakGetStatus() failed");
    this->PrintCertusError();
    successFlag = 0;
    }

  // Make sure that the attached system is a Certus
  if (successFlag &&
      (nFlags & (OPTOTRAK_3020_FLAG | OPTOTRAK_CERTUS_FLAG))
      != OPTOTRAK_CERTUS_FLAG)
    {
    vtkErrorMacro("Only Certus is supported. Attached device is not Certus.");
    successFlag = 0;
    }

  // Check to make sure configuration is what we expect
  if (successFlag &&
      (nNumSensors != 1 || nNumOdaus != 1))
    {
    vtkErrorMacro("Bad Certus configuration: " << nNumSensors << " sensors, "
                  << nNumOdaus << " odaus");
    successFlag = 0;
    }

  return successFlag;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::ShutdownCertusSystem()
{
  // Just a simple shutdown command
  if (TransputerShutdownSystem() != OPTO_NO_ERROR_CODE)
    {
    this->PrintCertusError();
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::ActivateCertusMarkers()
{
  // count the number of markers on all tools first
  if (OptotrakSetupCollection(
        this->NumberOfMarkers,  /* Number of markers in the collection. */
        (float)100.0,   /* Frequency to collect data frames at. */
        (float)2500.0,  /* Marker frequency for marker maximum on-time. */
        30,             /* Dynamic or Static Threshold value to use. */
        160,            /* Minimum gain code amplification to use. */
        0,              /* Stream mode for the data buffers. */
        (float)0.35,    /* Marker Duty Cycle to use. */
        (float)7.0,     /* Voltage to use when turning on markers. */
        (float)1.0,     /* Number of seconds of data to collect. */
        (float)0.0,     /* Number of seconds to pre-trigger data by. */
        OPTOTRAK_BUFFER_RAW_FLAG ) != OPTO_NO_ERROR_CODE)
    {
    return 0;
    }

  // Activate the markers
  if (OptotrakActivateMarkers() != OPTO_NO_ERROR_CODE)
    {
    return 0;
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::DeActivateCertusMarkers()
{
  if(OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
    {
    return 0;
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::Probe()
{
  // If device is already tracking, return success.
  if (this->IsDeviceTracking)
    {
    return 1;
    }

  // Perform initialization of the system
  int successFlag = this->InitializeCertusSystem();

  if (!successFlag)
    {
    this->PrintCertusError();
    }

  this->ShutdownCertusSystem();

  return successFlag;
} 


//----------------------------------------------------------------------------
int vtkNDICertusTracker::InternalStartTracking()
{
  if (this->IsDeviceTracking)
    {
    return 1;
    }

  // Attempt to initialize the Certus
  // and enable all the tools
  if (!this->InitializeCertusSystem()
      || !this->EnableToolPorts())
    {
    this->PrintCertusError();
    this->ShutdownCertusSystem();
    return 0;
    }

  // count the number of markers on all tools first
  if (!this->ActivateCertusMarkers())
    {
    this->PrintCertusError();
    this->ShutdownCertusSystem();
    return 0;
    }

  // For accurate timing
  this->Timer->Initialize();

  this->IsDeviceTracking = 1;

  return 1;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::InternalStopTracking()
{
  if (!this->DisableToolPorts())
    {
    this->PrintCertusError();
    }

  if(OptotrakDeActivateMarkers() != OPTO_NO_ERROR_CODE)
    {
    this->PrintCertusError();
    }

  return 1;
}

//----------------------------------------------------------------------------
void vtkNDICertusTracker::InternalUpdate()
{
  int tool;
  int missing[VTK_CERTUS_NTOOLS];
  long statusFlags[VTK_CERTUS_NTOOLS];
  double transform[VTK_CERTUS_NTOOLS][8];
  double *referenceTransform = 0;

  if (!this->IsDeviceTracking)
    {
    vtkWarningMacro( << "called Update() when Certus was not tracking");
    return;
    }

  // initialize transformations to identity
  for (tool = 0; tool < VTK_CERTUS_NTOOLS; tool++)
    {
    missing[tool] = 1; 

    transform[tool][0] = 1.0;
    transform[tool][1] = transform[tool][2] = transform[tool][3] = 0.0;
    transform[tool][4] = transform[tool][5] = transform[tool][6] = 0.0;
    transform[tool][7] = 0.0;
    }

  unsigned int uFrameNumber = 0;
  unsigned int uElements = 0;
  unsigned int uFlags = 0;
  OptotrakRigidStruct *rigidBodyData;
  rigidBodyData = new OptotrakRigidStruct[this->NumberOfRigidBodies];
  
  if (DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags,
                               rigidBodyData, 0) != OPTO_NO_ERROR_CODE)
    {
    this->PrintCertusError();
    delete [] rigidBodyData;
    return;
    }

  // these two calls are to generate an accurate timestamp
  this->Timer->SetLastFrame(uFrameNumber);
  double timestamp = this->Timer->GetTimeStampForFrame(uFrameNumber);

  for (int rigidCounter = 0; rigidCounter < this->NumberOfRigidBodies;
       rigidCounter++)
    {
    OptotrakRigidStruct& rigidBody = rigidBodyData[rigidCounter];
    long rigidId = rigidBody.RigidId;
    
    for (tool = 0; tool < VTK_CERTUS_NTOOLS; tool++)
      {
      if (this->PortHandle[tool] == rigidId)
        {
        break;
        }
      }

    if (tool >= VTK_CERTUS_NTOOLS)
      {
      vtkErrorMacro("InternalUpdate: bad rigid body ID " << rigidId);
      continue;
      }
    
    if ((rigidBody.flags & OPTOTRAK_UNDETERMINED_FLAG) == 0)
      {
      // this is done to keep the code similar to POLARIS
      double *trans = transform[tool];
      trans[0] = rigidBody.transformation.quaternion.rotation.q0;
      trans[1] = rigidBody.transformation.quaternion.rotation.qx;
      trans[2] = rigidBody.transformation.quaternion.rotation.qy;
      trans[3] = rigidBody.transformation.quaternion.rotation.qz;
      trans[4] = rigidBody.transformation.quaternion.translation.x;
      trans[5] = rigidBody.transformation.quaternion.translation.y;
      trans[6] = rigidBody.transformation.quaternion.translation.z;
      trans[7] = rigidBody.QuaternionError;
      }

    statusFlags[tool] = rigidBody.flags;
    }

  delete [] rigidBodyData;

  // get reference tool transform
  if (this->ReferenceTool >= 0)
    { 
    referenceTransform = transform[this->ReferenceTool];
    }

  for (tool = 0; tool < VTK_CERTUS_NTOOLS; tool++) 
    {
    // convert status flags from Optotrak format to vtkTracker format
    int flags = 0;
    if ((statusFlags[tool] & OPTOTRAK_UNDETERMINED_FLAG) != 0)
      {
      flags |= TR_MISSING;
      }
    else if ((statusFlags[tool] & OPTOTRAK_UNDETERMINED_FLAG) != 0)
      {
      flags |= TR_OUT_OF_VIEW;
      }

    // if tracking relative to another tool
    if (this->ReferenceTool >= 0 && tool != this->ReferenceTool)
      {
      if ((flags & TR_MISSING) == 0)
        {
        if ((statusFlags[this->ReferenceTool] & TR_OUT_OF_VIEW) != 0)
          {
          flags |= TR_OUT_OF_VIEW;
          }
        }
      // pre-multiply transform by inverse of relative tool transform
      ndiRelativeTransform(transform[tool],referenceTransform,transform[tool]);
      }
    ndiTransformToMatrixd(transform[tool],*this->SendMatrix->Element);
    this->SendMatrix->Transpose();

    // send the matrix and flags to the tool   
    this->ToolUpdate(tool, this->SendMatrix, flags, timestamp);   
    }
}

//----------------------------------------------------------------------------
// Enable all tool ports that have tools plugged into them.
// The reference port is enabled with NDI_STATIC.
int vtkNDICertusTracker::EnableToolPorts()
{
  int toolCounter = 0;

  // reset our information about the tool ports
  for (toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
    {
    if (this->PortEnabled[toolCounter])
      {
      if (RigidBodyDelete(this->PortHandle[toolCounter]) != OPTO_NO_ERROR_CODE)
        {
        this->PrintCertusError();
        }
      }
    this->PortEnabled[toolCounter] = 0;
    }

  // stop tracking
  if (this->IsDeviceTracking)
    {
    if(!this->DeActivateCertusMarkers())
      {
      this->PrintCertusError();
      }
    }

  // device handles (including status)
  int nDeviceHandles = 0;
  DeviceHandle *deviceHandles;

  int allDevicesEnabled = 0;
  for (int trialNumber = 0;
       trialNumber < 3 && !allDevicesEnabled;
       trialNumber++)
    {
    if (OptotrakGetNumberDeviceHandles(&nDeviceHandles) != OPTO_NO_ERROR_CODE)
      {
      this->PrintCertusError();
      return 0;
      }
 
    if (nDeviceHandles <= 0)
      {
      vtkErrorMacro("EnableToolPorts: no Optotrack strobers found");
      return 0;
      }

    // get all device handles and the status of each one
    deviceHandles = new DeviceHandle[nDeviceHandles];

    unsigned int flags = 0;
    if (OptotrakGetDeviceHandles(deviceHandles, nDeviceHandles, &flags)
        != OPTO_NO_ERROR_CODE)
      {
      this->PrintCertusError();
      delete [] deviceHandles;
      return 0;
      }

    // initialize this to 1 (set to 0 if unenabled handles found)
    allDevicesEnabled = 1;

    // free any unoccupied handles, enable any initialized handles
    for (int deviceCounter = 0;
         deviceCounter < nDeviceHandles;
         deviceCounter++)
      {
      int ph = deviceHandles[deviceCounter].nID;
      DeviceHandleStatus status = deviceHandles[deviceCounter].dtStatus;

      if (status == DH_STATUS_UNOCCUPIED)
        {
        if (OptotrakDeviceHandleFree(ph) != OPTO_NO_ERROR_CODE)
          {
          this->PrintCertusError();
          }
        allDevicesEnabled = 0;
        }
      else if (status == DH_STATUS_INITIALIZED)
        {
        if (OptotrakDeviceHandleEnable(ph) != OPTO_NO_ERROR_CODE)
          {
          this->PrintCertusError();
          }
        // enabling a strober will make other tools appear,
        // so let's be paranoid and always set this to zero
        allDevicesEnabled = 0;
        }
      }
    }

  // reset the number of markers and rigid bodies to zero
  this->NumberOfMarkers = 0;
  this->NumberOfRigidBodies = 0;

  // get information for all tools
  for (int deviceCounter = 0;
       deviceCounter < nDeviceHandles;
       deviceCounter++)
    {
    int ph = deviceHandles[deviceCounter].nID;
    DeviceHandleStatus status = deviceHandles[deviceCounter].dtStatus;

    if (status == DH_STATUS_UNOCCUPIED)
      {
      // this shouldn't happen, but just in case
      continue;
      }

    DeviceHandleProperty *properties = 0;
    int nProperties = 0;
    if (OptotrakDeviceHandleGetNumberProperties(ph, &nProperties)
        != OPTO_NO_ERROR_CODE
        || nProperties == 0)
      {
      this->PrintCertusError();
      }
    else
      {
      properties = new DeviceHandleProperty[nProperties];
      if (OptotrakDeviceHandleGetProperties(ph, properties, nProperties)
          != OPTO_NO_ERROR_CODE)
        {
        this->PrintCertusError();
        }
      else
        {
        // the properties of interest
        static const int deviceNameMaxlen = 128; 
        char deviceName[deviceNameMaxlen + 1];
        int hasROM = 0;
        int nToolPorts = 0;
        int nSwitches = 0;
        int nVLEDs = 0;
        int nSubPort = 0;
        int nMarkersToFire = 0;
        int status = 0;

        for (int propCounter = 0; propCounter < nProperties; propCounter++)
          {
          unsigned int propertyID = properties[propCounter].uPropertyID;
          if (propertyID == DH_PROPERTY_NAME)
            {
            strncpy(deviceName, properties[propCounter].dtData.szData,
                    deviceNameMaxlen);
            deviceName[deviceNameMaxlen] = '\0';
            }
          else if (propertyID == DH_PROPERTY_HAS_ROM)
            {
            hasROM = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_TOOLPORTS)
            {
            nToolPorts = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_SWITCHES)
            {
            nSwitches = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_VLEDS)
            {
            nVLEDs = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_SUBPORT)
            {
            nSubPort = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_MARKERSTOFIRE)
            {
            nMarkersToFire = properties[propCounter].dtData.nData;
            }
          else if (propertyID == DH_PROPERTY_STATUS)
            {
            status = properties[propCounter].dtData.nData;
            }
          }
          
        // verify that this is a tool, and not a strober
        if (hasROM && nToolPorts == 0)
          {
          // assume only one strober: index tools by SubPort
          int port = nSubPort - 1;

          if (port >= 0 && port < VTK_CERTUS_NTOOLS)
            {
            if (this->PortEnabled[port] &&
                this->PortHandle[port] != ph)
              {
              vtkErrorMacro("Port number " << port << " is already "
                            "taken by a different tool");
              }
            else
              {
              this->PortHandle[port] = ph;
              this->PortEnabled[port] = (status == DH_STATUS_ENABLED);
          
              // identifier info
              //this->Tools[port]->SetToolSerialNumber();
              //this->Tools[port]->SetToolRevision();
              this->Tools[port]->SetToolManufacturer(deviceName);
              //this->Tools[port]->SetToolType();
              this->Tools[port]->SetToolPartNumber(deviceName);
              }

            this->NumberOfMarkers += nMarkersToFire;
            }

          delete [] properties;
          }
        }
      }
    }

  if (deviceHandles)
    {
    delete [] deviceHandles;
    }
      
  // add rigid bodies
  for (toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
    {
    if (this->PortEnabled[toolCounter])
      {
      int ph = this->PortHandle[toolCounter];
      if (RigidBodyAddFromDeviceHandle(ph,
                                       ph, // rigID is port handle
                                       OPTOTRAK_QUATERN_RIGID_FLAG |
                                       OPTOTRAK_RETURN_QUATERN_FLAG)
          != OPTO_NO_ERROR_CODE)
        {
        this->PrintCertusError();
        }
      else
        {
        // increment the number of rigid bodies
        this->NumberOfRigidBodies++;

        // turn on the LEDs if set in TrackerTool
        int val;
        val = this->Tools[toolCounter]->GetLED1();
        if (val > 0 && val < 3)
          {
          OptotrakDeviceHandleSetVisibleLED(ph, 1,
                                            vtkNDICertusMapVLEDState[val]);
          }
        val = this->Tools[toolCounter]->GetLED2();
        if (val > 0 && val < 3)
          {
          OptotrakDeviceHandleSetVisibleLED(ph, 2,
                                            vtkNDICertusMapVLEDState[val]);
          }
        val = this->Tools[toolCounter]->GetLED3();
        if (val > 0 && val < 3)
          {
          OptotrakDeviceHandleSetVisibleLED(ph, 3,
                                            vtkNDICertusMapVLEDState[val]);
          }
        }
      }
    }
   
  // re-start the tracking
  if (this->IsDeviceTracking)
    {
    if (!this->ActivateCertusMarkers())
      {
      this->PrintCertusError();
      return 0;
      }
    }

  return 1;
}

//----------------------------------------------------------------------------
// Disable all enabled tool ports.
int vtkNDICertusTracker::DisableToolPorts()
{
  // stop tracking
  if (this->IsDeviceTracking)
    {
    if (!this->DeActivateCertusMarkers())
      {
      this->PrintCertusError();
      }
    }

  // disable the enabled ports
  for (int toolCounter = 0; toolCounter < VTK_CERTUS_NTOOLS; toolCounter++)
    {
    if (this->PortEnabled[toolCounter])
      {
      if (RigidBodyDelete(this->PortHandle[toolCounter]) != OPTO_NO_ERROR_CODE)
        {
        this->PrintCertusError();
        }
      }
    this->PortEnabled[toolCounter] = 0;
    }

  // re-start the tracking
  if (this->IsDeviceTracking)
    {
    if (!this->ActivateCertusMarkers())
      {
      this->PrintCertusError();
      }
    }

  return 1;
}

//----------------------------------------------------------------------------
int vtkNDICertusTracker::GetToolFromHandle(int handle)
{
  int tool;

  for (tool = 0; tool < VTK_CERTUS_NTOOLS; tool++)
    {
    if (this->PortHandle[tool] == handle)
      {
      return tool;
      }
    }

  return -1;
}

//----------------------------------------------------------------------------
// cause the system to beep
int vtkNDICertusTracker::InternalBeep(int n)
{
  // beep is not implemented yet
  return 1;
}

//----------------------------------------------------------------------------
// change the state of an LED on the tool
int vtkNDICertusTracker::InternalSetToolLED(int tool, int led, int state)
{
  if (this->Tracking &&
      tool >= 0 && tool < VTK_CERTUS_NTOOLS &&
      led >= 0 && led < 3)
    {
    VLEDState pstate = vtkNDICertusMapVLEDState[led];
    int ph = this->PortHandle[tool];
    if (ph == 0)
      {
      return 0;
      }

    OptotrakDeviceHandleSetVisibleLED(ph, led+1, pstate);
    }

  return 1;
}




