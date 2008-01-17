/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundFan.h,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.2 $
  Thanks:    Thanks to David G. Gobbi who developed this class.

==========================================================================

Copyright (c) 2000-2007 Atamai, Inc.

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
// .NAME vtkUltrasoundFan - Reslices a volume along the axes specified.
// .SECTION Description
// vtkUltrasoundFan will reslice a volume along the axes specified by
// the reslicing transformation matrix.  The extent, origin, and sampling
// density of the output data can also be set.
// .SECTION Caveats
// The OptimizationOn() in conjunction with InterpolateOff()
// may cause this filter to crash for compilers with poor floating 
// point consistency.  Doing crazy things like using nonlinear 
// (i.e. perspective) transformations is also risky, but will
// work under a broad set of circumstances. 
// .SECTION see also
// vtkTransform


#ifndef __vtkUltrasoundFan_h
#define __vtkUltrasoundFan_h


#include "vtkImageToImageFilter.h"
#include "vtkTransform.h"
#include "vtkDoubleArray.h"

class vtkMatrix4x4;

#define VTK_RESLICE_NEAREST 0
#define VTK_RESLICE_LINEAR 1
#define VTK_RESLICE_CUBIC 3

class VTK_EXPORT vtkUltrasoundFan : public vtkImageToImageFilter
{
public:
  vtkUltrasoundFan();
  ~vtkUltrasoundFan();
  static vtkUltrasoundFan *New();
  const char *GetClassName() {return "vtkUltrasoundFan";};

  virtual void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set the depth (in mm) of the axis about which the 
  // transducer is rotating -- can be negative or positive,
  // default = 0
  vtkSetMacro(RotationAxisDepth,double);
  vtkGetMacro(RotationAxisDepth,double);

  // Description:
  // In case the slices are not at evenly-spaced angles,
  // provide an angle for each slice (by default, the Z 
  // coordinate of the input is treated as an angle).
  vtkSetObjectMacro(RotationAngles, vtkDoubleArray);
  vtkGetObjectMacro(RotationAngles, vtkDoubleArray);

  // Description:
  // For clipping the image so that everything outside the
  // actual ultrasound data is cleared to black.

  // Description:
  // Set the depth at which the ultrasound fan originates.
  // Default: 0
  vtkSetMacro(FanApexDepth,double);
  vtkGetMacro(FanApexDepth,double);

  // Description:
  // Set the maximum depth of the fan.  
  // Default: from zero to max depth in image.
  vtkSetMacro(MaximumDepth,double);
  vtkGetMacro(MaximumDepth,double);

  // Description:
  // Set the angle range of the fan.  Default: -90.0, 90.0
  vtkSetVector2Macro(FanAngleExtent,double);
  vtkGetVector2Macro(FanAngleExtent,double);

  // Description:
  // Interpolate the date.  Default: no
  vtkSetMacro(Interpolate,int);
  vtkBooleanMacro(Interpolate,int);
  vtkGetMacro(Interpolate,int);

protected:
  double RotationAxisDepth;  // depth coordinate of rotation axis
  vtkDoubleArray *RotationAngles; // angle of each slice

  double FanApexDepth;       // depth coordinate of fan origin
  double MaximumDepth;       // penetration depth of beam
  double FanAngleExtent[2];  // start/stop angles of fan

  int Interpolate;  // interpolate, or don't

  int OutputExtent[6];
  float OutputSpacing[3];
  float OutputOrigin[3];

  void ExecuteInformation();
  void ComputeRequiredInputUpdateExtent(int inExt[6], int outExt[6]);
  
  // Description:
  // This method is passed a input and output region, and executes the filter
  // algorithm to fill the output from the input.
  // It just executes a switch statement to call the correct function for
  // the regions data types.
  void ThreadedExecute(vtkImageData *inData, vtkImageData *outData, 
		       int ext[6], int id);

private:
  vtkUltrasoundFan(const vtkUltrasoundFan &);
  void operator=(const vtkUltrasoundFan&);  
};

#endif





