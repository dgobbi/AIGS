/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkNDICertusTracker.h,v $
  Creator:   David Gobbi <dgobbi@cs.queensu.ca>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2008/06/17 03:59:34 $
  Version:   $Revision: 1.1 $

==========================================================================

[ Copyright Notice Goes Here ]

=========================================================================*/
// .NAME vtkNDICertusTracker - VTK interface for the NDI Optotrak Certus
// .SECTION Description
// The vtkNDICertusTracker class provides an interface to the Optotrak
// Certus (Northern Digital Inc., Waterloo, Canada), utilizing the PCI
// interface card.
// .SECTION see also
// vtkNDITracker vtkTrackerTool


#ifndef __vtkNDICertusTracker_h
#define __vtkNDICertusTracker_h

#include "vtkTracker.h"
#include "ndicapi.h"

class vtkFrameToTimeConverter;

// the number of tools this class can handle
#define VTK_CERTUS_NTOOLS 12

class VTK_EXPORT vtkNDICertusTracker : public vtkTracker
{
public:

  static vtkNDICertusTracker *New();
  vtkTypeMacro(vtkNDICertusTracker,vtkTracker);
  void PrintSelf(ostream& os, vtkIndent indent);
 
  // Description:
  // Probe to check whether there is an attached Certus system that
  // is able to track.  After Probe is called, you can call
  // GetVersion() to get information about the attached Certus system.
  int Probe();

  // Description:
  // Get the a string (perhaps a long one) describing the type and version
  // of the device.
  vtkGetStringMacro(Version);

  // Description:
  // Get an update from the tracking system and push the new transforms
  // to the tools.  This should only be used within vtkTracker.cxx.
  void InternalUpdate();

protected:
  vtkNDICertusTracker();
  ~vtkNDICertusTracker();

  // Description:
  // Set the version information.
  vtkSetStringMacro(Version);

  // Description:
  // Start the tracking system.  The tracking system is brought from
  // its ground state into full tracking mode.  The device will
  // only be reset if communication cannot be established without
  // a reset.
  int InternalStartTracking();

  // Description:
  // Stop the tracking system and bring it back to its ground state:
  // Initialized, not tracking, at 9600 Baud.
  int InternalStopTracking();

  // Description:
  // Cause the device to beep the specified number of times.
  int InternalBeep(int n);

  // Description:
  // Set the specified tool LED to the specified state.
  int InternalSetToolLED(int tool, int led, int state);

  // Description:
  // Initialize communication with the Certus system.
  int InitializeCertusSystem();

  // Description:
  // Terminate communication with the Certus system.
  int ShutdownCertusSystem();

  // Description:
  // Activate the markers for tracking.
  int ActivateCertusMarkers();

  // Description:
  // Deactivate all markers.
  int DeActivateCertusMarkers();

  // Description:
  // Print the most recent error.
  int PrintCertusError();

  // Description:
  // Methods for detecting which ports have tools in them, and
  // auto-enabling those tools.
  int EnableToolPorts();
  int DisableToolPorts();

  // Description:
  // Find the tool for a specific port handle (-1 if not found).
  int GetToolFromHandle(int handle);

  // Description:
  // Class for updating the virtual clock that accurately times the
  // arrival of each transform, more accurately than is possible with
  // the system clock alone because the virtual clock averages out the
  // jitter.
  vtkFrameToTimeConverter *Timer;

  char *Version;

  int NumberOfMarkers;
  int NumberOfRigidBodies;

  vtkMatrix4x4 *SendMatrix;
  int IsDeviceTracking;

  int PortEnabled[VTK_CERTUS_NTOOLS];
  int PortHandle[VTK_CERTUS_NTOOLS];

private:
  vtkNDICertusTracker(const vtkNDICertusTracker&);
  void operator=(const vtkNDICertusTracker&);  
};

#endif





