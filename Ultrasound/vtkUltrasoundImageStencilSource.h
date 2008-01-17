/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundImageStencilSource.h,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.6 $
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
  vtkSetVector4Macro(ClipRectangle, double);
  vtkGetVector4Macro(ClipRectangle, double);
  
  // Description:
  // If the ultrasound probe collects a fan of data, specify the position and
  // dimensions of the fan.
  vtkSetVector2Macro(FanAngles, double);
  vtkGetVector2Macro(FanAngles, double);
  vtkSetVector2Macro(FanOrigin, double);
  vtkGetVector2Macro(FanOrigin, double);
  vtkSetMacro(FanDepth, double);
  vtkGetMacro(FanDepth, double);
  
protected:
  vtkUltrasoundImageStencilSource();
  ~vtkUltrasoundImageStencilSource();

  // VTK 4
  // void ThreadedExecute(vtkImageStencilData *output,
  // int extent[6], int threadId);
  // VTK 5
  
  int RequestData( vtkInformation* request,
		   vtkInformationVector** inputVector,
		   vtkInformationVector* outputVector);
  int RequestInformation( vtkInformation *request,
			  vtkInformationVector **inputVector,
			  vtkInformationVector *outputVector);
  
  int FillInputPortInformation(int port, vtkInformation *info);
  double ClipRectangle[4];
  double FanAngles[2];
  double FanOrigin[2];
  double FanDepth;

private:
  vtkUltrasoundImageStencilSource(const vtkUltrasoundImageStencilSource&);
  void operator=(const vtkUltrasoundImageStencilSource&);
};

#endif
