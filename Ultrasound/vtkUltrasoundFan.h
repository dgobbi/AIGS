/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundFan.h,v $
  Language:  C++
  Date:      $Date: 2003/02/02 20:48:29 $
  Version:   $Revision: 1.1 $
  Thanks:    Thanks to David G. Gobbi who developed this class.

Copyright (c) 1993-1999 Ken Martin, Will Schroeder, Bill Lorensen.

This software is copyrighted by Ken Martin, Will Schroeder and Bill Lorensen.
The following terms apply to all files associated with the software unless
explicitly disclaimed in individual files. This copyright specifically does
not apply to the related textbook "The Visualization Toolkit" ISBN
013199837-4 published by Prentice Hall which is covered by its own copyright.

The authors hereby grant permission to use, copy, and distribute this
software and its documentation for any purpose, provided that existing
copyright notices are retained in all copies and that this notice is included
verbatim in any distributions. Additionally, the authors grant permission to
modify this software and its documentation for any purpose, provided that
such modifications are not distributed without the explicit consent of the
authors and that existing copyright notices are retained in all copies. Some
of the algorithms implemented by this software are patented, observe all
applicable patent law.

IN NO EVENT SHALL THE AUTHORS OR DISTRIBUTORS BE LIABLE TO ANY PARTY FOR
DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
OF THE USE OF THIS SOFTWARE, ITS DOCUMENTATION, OR ANY DERIVATIVES THEREOF,
EVEN IF THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE AUTHORS AND DISTRIBUTORS SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE, AND NON-INFRINGEMENT.  THIS SOFTWARE IS PROVIDED ON AN
"AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


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





