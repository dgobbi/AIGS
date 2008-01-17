/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundFrameAnalyze.h,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.5 $

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
// .NAME vtkUltrasoundFrameAnalyze - get information about an ultrasound image
// .SECTION Description
// vtkUltrasoundFrameAnalyze extracts relevant information from a raw video
// ultrasound image, and modifies the Output and Origin of the image
// according to the x and y graticules present in the image.  It also
// provides a stencil that can be used to clip out only the pizza-slice
// of ultrasound data from the raw image.

#ifndef __vtkUltrasoundFrameAnalyze_h
#define __vtkUltrasoundFrameAnalyze_h

#include "vtkImageToImageFilter.h"
#include "vtkUltrasoundImageStencilSource.h"

class VTK_EXPORT vtkUltrasoundFrameAnalyze : public vtkImageToImageFilter
{
public:
  static vtkUltrasoundFrameAnalyze *New();
  vtkTypeMacro(vtkUltrasoundFrameAnalyze,vtkImageToImageFilter);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Update all the information to be extracted from the image.
  // This only performs "Execute" if an execute is required, i.e. 
  // if the image has changed since the last execute.
  void Analyze();

  // Description:
  // Get the origin and spacing for the input image accoding to
  // the graticules (the rulers) in the image.
  vtkGetVector3Macro(Spacing, vtkFloatingPointType);
  vtkGetVector3Macro(Origin, vtkFloatingPointType);

  // Description:
  // Get the clipping extent for the part of the image that lies
  // within the rectangle that is defined by the two rulers.
  vtkGetVector6Macro(ClipExtent, int);

  // Description:
  // Get the rectangle that contains the actual ultrasound data,
  // I.e. (xmin, xmax, ymin, ymax).
  // This isn't completely functional yet, instead it simply returns
  // the ClipExtent converted into millimetre units according to the
  // Spacing and Origin.
  vtkGetVector4Macro(ClipRectangle, double);

  // Description:
  // Get a boolean value to specify whether the ultrasound image
  // is flipped across one or more axes, according to the position
  // of the white dot in the image.
  vtkGetVector3Macro(Flip, int);

  // Description:
  // Get the color levels that correspond to black and white.
  vtkGetMacro(BlackLevel, double);
  vtkGetMacro(WhiteLevel, double);

  // Description:
  // Get the fan angles and the fan depth
  vtkGetVector2Macro(FanAngles, double);
  vtkGetVector2Macro(FanOrigin, double);
  vtkGetMacro(FanDepth, double);

  // Description:
  // Get a stencil for clipping out just the image portion of the
  // video frame.  Either vtkImageStencil or vtkImageBlend can be
  // used to apply the stencil.
  vtkImageStencilData *GetStencil() {
    return this->StencilSource->GetOutput(); };

protected:
  vtkUltrasoundFrameAnalyze();
  ~vtkUltrasoundFrameAnalyze();

  void ExecuteInformation(vtkImageData *inData, vtkImageData *outData);
  void ExecuteInformation() {
    this->vtkImageToImageFilter::ExecuteInformation(); };
  void ExecuteData(vtkDataObject *data);

  vtkFloatingPointType Spacing[3];
  vtkFloatingPointType Origin[3];
  int ClipExtent[6];
  double ClipRectangle[6];
  int Flip[3];
  float BlackLevel;
  float WhiteLevel;
  double FanAngles[2];
  double FanOrigin[2];
  double FanDepth;

  int ClipGuess[6];

  vtkUltrasoundImageStencilSource* StencilSource;

private:
  vtkUltrasoundFrameAnalyze(const vtkUltrasoundFrameAnalyze&);
  void operator=(const vtkUltrasoundFrameAnalyze&);
};

#endif
