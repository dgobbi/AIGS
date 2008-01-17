/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundCompare.h,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.2 $

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
// .NAME vtkUltrasoundCompare - Compares ultrasound images
// .SECTION Description
// vtkUltrasoundCompare takes two images and compares them and calculates
// statistics on them.  If either image has an alpha channel associated
// with it, then the images are compared only for pixels with an alpha
// value of 255 (i.e. 1.0).

#ifndef __vtkUltrasoundCompare_h
#define __vtkUltrasoundCompare_h

#include "vtkImageTwoInputFilter.h"

class VTK_EXPORT vtkUltrasoundCompare : public vtkImageTwoInputFilter
{
public:
  static vtkUltrasoundCompare *New();
  vtkTypeMacro(vtkUltrasoundCompare,vtkImageTwoInputFilter);
  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Get the error between the images, i.e. the sum of the
  // absolute value all the pixel differences.
  double GetError();
  
  // Description:
  // Get the number of pixels that were compared.  This will depend
  // on the alpha channel of the images.
  int GetPixelCount();

protected:
  vtkUltrasoundCompare();
  ~vtkUltrasoundCompare() {};

  double ErrorPerThread[VTK_MAX_THREADS];
  int PixelCountPerThread[VTK_MAX_THREADS];
  
  void ExecuteInformation(vtkImageData **inputs, vtkImageData *output); 
  void ComputeInputUpdateExtent(int inExt[6], int outExt[6],
                                int whichInput);
  void ExecuteInformation() {
    this->vtkImageTwoInputFilter::ExecuteInformation(); };
  void ThreadedExecute(vtkImageData **inDatas, vtkImageData *outData,
                       int extent[6], int id);  
  
private:
  vtkUltrasoundCompare(const vtkUltrasoundCompare&);  // Not implemented.
  void operator=(const vtkUltrasoundCompare&);  // Not implemented.
};

#endif


