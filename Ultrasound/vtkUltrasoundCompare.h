/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundCompare.h,v $
  Language:  C++
  Date:      $Date: 2003/02/02 20:48:29 $
  Version:   $Revision: 1.1 $

  Copyright (c) 1993-2002 Ken Martin, Will Schroeder, Bill Lorensen 
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notice for more information.

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


