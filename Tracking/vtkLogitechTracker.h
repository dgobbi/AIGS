/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkLogitechTracker.h,v $
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
// .NAME vtkLogitechTracker - VTK interface to the Logitech 3D mouse
// .SECTION Description
// The vtkLogitechTracker is a VTK interface to the logitech 3D mouse.
// It requires the logitech driver written by David Gobbi at the Robarts
// Research Institute (dgobbi@irus.rri.on.ca).  The logitech driver
// currently works under Win32, Linux, and IRIX.
// .SECTION see also
// vtkTrackerTool vtkPOLARISTracker vtkFlockTracker

#ifndef __vtkLogitechTracker_h
#define __vtkLogitechTracker_h

#include "vtkTracker.h"
#include "vtkTransform.h"

struct logitech3d;

class VTK_EXPORT vtkLogitechTracker : public vtkTracker
{
public:

  static vtkLogitechTracker *New();
  vtkTypeMacro(vtkLogitechTracker,vtkTracker);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Probe to see if a compatible Logitech 3D mouse is on the
  // specified serial port.
  int Probe();

  // Description:
  // Set which serial port to use, 1 through 4.  Default: 1.
  vtkSetMacro(SerialPort, int);
  vtkGetMacro(SerialPort, int);

  // Description:
  // Get an update from the tracking system and push the new transforms
  // to the tools.  This should only be used within vtkTracker.cxx.
  void InternalUpdate();

protected:
  vtkLogitechTracker();
  ~vtkLogitechTracker();

  // Description:
  // Start the tracking system.  The tracking system is brought from
  // its ground state into full tracking mode. 
  int InternalStartTracking();

  // Description:
  // Stop the tracking system and bring it back to its ground state.
  int InternalStopTracking();

  logitech3d *Logitech;

  vtkTransform *Transform;
  int SerialPort;
  int Mode;

private:
  vtkLogitechTracker(const vtkLogitechTracker&);
  void operator=(const vtkLogitechTracker&);  
};

#endif





