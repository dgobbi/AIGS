/* ============================================================================

  File: vtkFakeTracker.h
  Author: Kyle Charbonneau <kcharbon@imaging.robarts.ca>
  Language: C++
  Description: 
    This class represents a fake tracking system with tools that have
    predetermined behaviour. This allows someonew who doesn't have access to
    a tracking system to test code that relies on having one active.

============================================================================ */

#ifndef __vtkFakeTracker_h
#define __vtkFakeTracker_h

#include "vtkTracker.h"

class vtkTransform;

class VTK_EXPORT vtkFakeTracker: public vtkTracker
{
public:
  static vtkFakeTracker *New();
  vtkTypeMacro(vtkFakeTracker,vtkObject);

  int Probe();
  int InternalStartTracking();
  int InternalStopTracking();
  void InternalUpdate();

  vtkSetMacro(SerialPort, int);
  vtkGetMacro(SerialPort, int);

protected:
  vtkFakeTracker();
  ~vtkFakeTracker();

  int Frame;
  double TimeStamp;
  vtkTransform *InternalTransform;
  int SerialPort;
};


#endif