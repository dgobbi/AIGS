/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkPolhemusTracker.h,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2005/06/27 13:52:47 $
  Version:   $Revision: 1.1 $

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
// .NAME vtkPolhemusTracker - VTK interface to the Polhemus
// .SECTION Description
// The vtkPolhemusTracker is a VTK interface to the the Polhemus magnetic
// position-sensor device.
// It requires the Polhemus C API which is available
// from David Gobbi at the Atamai Inc. (dgobbi@atamai.com).
// Note that the Polhemus FasTrak requires a null-modem cable instead
// of a standard straight-through serial cable.
// .SECTION see also
// vtkTrackerTool vtkPOLARISTracker


#ifndef __vtkPolhemusTracker_h
#define __vtkPolhemusTracker_h

#include "vtkTracker.h"
#include "vtkTransform.h"

struct polhemus;

class VTK_EXPORT vtkPolhemusTracker : public vtkTracker
{
public:

  static vtkPolhemusTracker *New();
  vtkTypeMacro(vtkPolhemusTracker,vtkTracker);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Probe to see if the tracking system is present on the 
  // serial port specified by SetSerialPort.
  int Probe();

  // Description:
  // Set which serial port to use, 1 through 4.  Default: 1.
  vtkSetMacro(SerialPort, int);
  vtkGetMacro(SerialPort, int);

  // Description:
  // Set the desired baud rate.  This baud rate must match the rate
  // specified by the DIP switches on the Polhemus unit. Default: 9600. 
  vtkSetMacro(BaudRate, int);
  vtkGetMacro(BaudRate, int);

  // Description:
  // Get an update from the tracking system and push the new transforms
  // to the tools.  This should only be called from the superclass,
  // i.e. within vtkTracker.cxx.
  void InternalUpdate();

protected:
  vtkPolhemusTracker();
  ~vtkPolhemusTracker();

  // Description:
  // Start the tracking system.  The tracking system is brought from
  // its ground state into full tracking mode.  Under Win32 and Linux
  // this will also clear all errors - under other operating systems,
  // you might have to use the 'FLY/STANDBY' switch to clear errors.
  int InternalStartTracking();

  // Description:
  // Stop the tracking system and bring it back to its ground state.
  int InternalStopTracking();

  polhemus *Polhemus;

  vtkMatrix4x4 *SendMatrix;
  int SerialPort;
  int BaudRate;
  int Mode;

private:
  vtkPolhemusTracker(const vtkPolhemusTracker&);
  void operator=(const vtkPolhemusTracker&);  
};

#endif





