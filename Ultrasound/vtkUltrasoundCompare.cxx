/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundCompare.cxx,v $
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
#include "vtkUltrasoundCompare.h"

#include "vtkImageData.h"
#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
vtkUltrasoundCompare* vtkUltrasoundCompare::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkUltrasoundCompare");
  if(ret)
    {
    return (vtkUltrasoundCompare*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkUltrasoundCompare;
}

// Construct object to extract all of the input data.
vtkUltrasoundCompare::vtkUltrasoundCompare()
{
  int i;
  for ( i = 0; i < this->NumberOfThreads; i++ )
    {
    this->ErrorPerThread[i] = 0.0;
    this->PixelCountPerThread[i] = 0;
    }
}


//----------------------------------------------------------------------------
// This method computes the input extent necessary to generate the output.
void vtkUltrasoundCompare::ComputeInputUpdateExtent(int inExt[6],
                                                    int outExt[6],
                                                    int whichInput)
{
  int *wholeExtent;

  wholeExtent = this->GetInput(whichInput)->GetWholeExtent();  
  memcpy(inExt,wholeExtent,6*sizeof(int));
}

//----------------------------------------------------------------------------
void vtkUltrasoundCompare::ThreadedExecute(vtkImageData **inData, 
                                           vtkImageData *outData,
                                           int outExt[6], int id)
{
  unsigned char *in1Ptr0, *in1Ptr1, *in1Ptr2;
  unsigned char *in2Ptr0, *in2Ptr1, *in2Ptr2;
  unsigned char *outPtr0, *outPtr1, *outPtr2;
  int min0, max0, min1, max1, min2, max2;
  int idx0, idx1, idx2;
  int in1Inc0, in1Inc1, in1Inc2;
  int in2Inc0, in2Inc1, in2Inc2;
  int outInc0, outInc1, outInc2;
  int alpha1 = 0;
  int alpha2 = 0;
  int alphaO = 0;
  unsigned long count = 0;
  unsigned long target;

  double errorval = 0;
  int pixelcount = 0;
  
  // these are the values returned if an error occurs
  this->ErrorPerThread[id] = 0;
  this->PixelCountPerThread[id] = 1;
  
  if (inData[0] == NULL || inData[1] == NULL || outData == NULL)
    {
    if (!id)
      {
      vtkErrorMacro(<< "Execute: Missing data");
      }
    return;
    }

  if (inData[0]->GetNumberOfScalarComponents() > 2 ||
      inData[1]->GetNumberOfScalarComponents() > 2 ||
      outData->GetNumberOfScalarComponents() > 2)
    {
    if (!id)
      {
      vtkErrorMacro(<< "Execute: RGB images are not supported");
      }
    return;
    }
    
  // this filter expects that input is the same type as output.
  if (inData[0]->GetScalarType() != VTK_UNSIGNED_CHAR || 
      inData[1]->GetScalarType() != VTK_UNSIGNED_CHAR || 
      outData->GetScalarType() != VTK_UNSIGNED_CHAR)
      {
      if (!id)
        {
        vtkErrorMacro(<< "Execute: All ScalarTypes must be unsigned char");
        }
      return;
      }
  
  alpha1 = (inData[0]->GetNumberOfScalarComponents() > 1);
  alpha2 = (inData[1]->GetNumberOfScalarComponents() > 1);
  alphaO = (outData->GetNumberOfScalarComponents() > 1);

  in1Ptr2 = (unsigned char *) inData[0]->GetScalarPointerForExtent(outExt);  
  in2Ptr2 = (unsigned char *) inData[1]->GetScalarPointerForExtent(outExt);  
  outPtr2 = (unsigned char *) outData->GetScalarPointerForExtent(outExt);  

  inData[0]->GetIncrements(in1Inc0, in1Inc1, in1Inc2);
  inData[1]->GetIncrements(in2Inc0, in2Inc1, in2Inc2);
  outData->GetIncrements(outInc0, outInc1, outInc2);
  
  min0 = outExt[0];  max0 = outExt[1];
  min1 = outExt[2];  max1 = outExt[3];
  min2 = outExt[4];  max2 = outExt[5];
  
  target = (unsigned long)((max2 - min2 +1)*(max1 - min1 + 1)/50.0);
  target++;

  for (idx2 = min2; idx2 <= max2; ++idx2)
    {
    in1Ptr1 = in1Ptr2;
    in2Ptr1 = in2Ptr2;
    outPtr1 = outPtr2;
    for (idx1 = min1; !this->AbortExecute && idx1 <= max1; ++idx1)
      {
      if (!id) 
        {
        if (!(count%target))
          {
          this->UpdateProgress(count/(50.0*target));
          }
        count++;
        }
      in1Ptr0 = in1Ptr1;
      in2Ptr0 = in2Ptr1;
      outPtr0 = outPtr1;
      for (idx0 = min0; idx0 <= max0; ++idx0)
        {
        int inmask = 1;
        if (alpha1 && in1Ptr0[1] != 255)
          {
          inmask = 0;
          }
        else if (alpha2 && in2Ptr0[1] != 255)
          {
          inmask = 0;
          }
        if (inmask) 
          {
          pixelcount++;
          int diff = in1Ptr0[0] - in2Ptr0[0];
          if (diff < 0)
            {
            diff = -diff;
            }
          errorval += diff;

          outPtr0[0] = diff;
          }
        else
          {
          outPtr0[0] = 0;
          }
        if (alphaO)
          {
          outPtr0[1] = inmask*255;
          }
            
        outPtr0 += outInc0;
        in1Ptr0 += in1Inc0;
        in2Ptr0 += in2Inc0;
        }
      outPtr1 += outInc1;
      in1Ptr1 += in1Inc1;
      in2Ptr1 += in2Inc1;
      }
    outPtr2 += outInc2;
    in1Ptr2 += in1Inc2;
    in2Ptr2 += in2Inc2;
    }

  this->ErrorPerThread[id] = errorval;
  this->PixelCountPerThread[id] = pixelcount;
}

//----------------------------------------------------------------------------
// Make sure both the inputs are the same size. Doesn't really change 
// the output. Just performs a sanity check
void vtkUltrasoundCompare::ExecuteInformation(vtkImageData **inputs,
                                              vtkImageData *output)
{
  int i;
  int *in1Ext, *in2Ext;
  int ext[6];
  
  // Make sure the Input has been set.
  // we require that input 1 be set.
  if ( ! inputs[0] || ! inputs[1])
    {
    vtkErrorMacro(<< "ExecuteInformation: Input is not set.");
    return;
    }
  
  in1Ext = inputs[0]->GetWholeExtent();
  in2Ext = inputs[1]->GetWholeExtent();

  if (in1Ext[0] != in2Ext[0] || in1Ext[1] != in2Ext[1] || 
      in1Ext[2] != in2Ext[2] || in1Ext[3] != in2Ext[3] || 
      in1Ext[4] != in2Ext[4] || in1Ext[5] != in2Ext[5])
    {
    for (i = 0; i < this->NumberOfThreads; i++)
      {
      this->ErrorPerThread[i] = 0;
      this->PixelCountPerThread[i] = 1;
      }
    vtkErrorMacro("ExecuteInformation: Input are not the same size.");
    }

  // We still need to set the whole extent to be the intersection.
  // Otherwise the execute may crash.
  for (i = 0; i < 3; ++i)
    {
    ext[i*2] = in1Ext[i*2];
    if (ext[i*2] < in2Ext[i*2])
      {
      ext[i*2] = in2Ext[i*2];
      }
    ext[i*2+1] = in1Ext[i*2+1];
    if (ext[i*2+1] > in2Ext[i*2+1])
      {
      ext[i*2+1] = in2Ext[i*2+1];
      }
    }
  output->SetWholeExtent(ext);
  
  //cerr << "ext " << ext[0] << ' ' << ext[1] << ' ' << ext[2] << ' ' << ext[3] << ' ' << ext[4] << ' ' << ext[5] << '\n';

  output->SetNumberOfScalarComponents(
        inputs[0]->GetNumberOfScalarComponents());
}

double vtkUltrasoundCompare::GetError()
{
  double error = 0.0;
  int i;

  for (i = 0; i < this->NumberOfThreads; i++ )
    {
    error += this->ErrorPerThread[i];
    }

  return error;
}

int vtkUltrasoundCompare::GetPixelCount()
{
  int count = 0;
  int i;

  for (i = 0; i < this->NumberOfThreads; i++ )
    {
    count += this->PixelCountPerThread[i];
    }

  return count;
}

void vtkUltrasoundCompare::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
  int i;

  for ( i= 0; i < this->NumberOfThreads; i++ )
    {
    os << indent << "Error for thread " << i << ": " << this->ErrorPerThread[i] << "\n";
    os << indent << "PixelCount for thread " << i << ": " << this->PixelCountPerThread[i] << "\n";
    }
  os << indent << "Error: " << this->GetError() << "\n";
  os << indent << "PixelCount: " << this->GetPixelCount() << "\n";
}


