/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundImageStencilSource.h,v $
  Language:  C++
  Date:      $Date: 2003/02/02 20:48:30 $
  Version:   $Revision: 1.1 $
  Thanks:    Thanks to David G. Gobbi who developed this class.

Copyright (c) 1993-2001 Ken Martin, Will Schroeder, Bill Lorensen 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither name of Ken Martin, Will Schroeder, or Bill Lorensen nor the names
   of any contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

 * Modified source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=========================================================================*/
// .NAME vtkUltrasoundImageStencilSource - clip an ultrasound image
// .SECTION Description
// vtkUltrasoundImageStencilSource will use information about an ultrasound
// image (mainly the clipping rectangle and pizza-slice that define the
// extent of the image) to create a stencil that will extract only the
// ultrasound data from the image.  This class is usually not used directly,
// but is instead used from within vtkUltrasoundFrameAnalyze.
// .SECTION see also
// vtkImplicitFunctionToImageStencil vtkImageStencil vtkUltrasoundFrameAnalyze

#ifndef __vtkUltrasoundImageStencilSource_h
#define __vtkUltrasoundImageStencilSource_h


#include "vtkImageStencilSource.h"
#include "vtkImplicitFunction.h"

class VTK_EXPORT vtkUltrasoundImageStencilSource 
  : public vtkImageStencilSource
{
public:
  static vtkUltrasoundImageStencilSource *New();
  vtkTypeMacro(vtkUltrasoundImageStencilSource, vtkImageStencilSource);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set the clip rectangle to apply to the image (in slice image coords,
  // not pixel indices).
  vtkSetVector4Macro(ClipRectangle, float);
  vtkGetVector4Macro(ClipRectangle, float);

  // Description:
  // If the ultrasound probe collects a fan of data, specify the position and
  // dimensions of the fan.
  vtkSetVector2Macro(FanAngles, float);
  vtkGetVector2Macro(FanAngles, float);
  vtkSetVector2Macro(FanOrigin, float);
  vtkGetVector2Macro(FanOrigin, float);
  vtkSetMacro(FanDepth, float);
  vtkGetMacro(FanDepth, float);

protected:
  vtkUltrasoundImageStencilSource();
  ~vtkUltrasoundImageStencilSource();

  void ThreadedExecute(vtkImageStencilData *output,
		       int extent[6], int threadId);

  float ClipRectangle[4];
  float FanAngles[2];
  float FanOrigin[2];
  float FanDepth;

private:
  vtkUltrasoundImageStencilSource(const vtkUltrasoundImageStencilSource&);
  void operator=(const vtkUltrasoundImageStencilSource&);
};

#endif

