/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundImageStencilSource.cxx,v $
  Language:  C++
  Date:      $Date: 2007/08/07 20:40:19 $
  Version:   $Revision: 1.7 $
  Thanks:    Thanks to David G Gobbi who developed this class.

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

#include <math.h>
#include "vtkUltrasoundImageStencilSource.h"
#include "vtkImageStencilData.h"
#include "vtkObjectFactory.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkStreamingDemandDrivenPipeline.h"

//----------------------------------------------------------------------------
vtkUltrasoundImageStencilSource* vtkUltrasoundImageStencilSource::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkUltrasoundImageStencilSource");
  if(ret)
    {
    return (vtkUltrasoundImageStencilSource*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkUltrasoundImageStencilSource;
}

//----------------------------------------------------------------------------
vtkUltrasoundImageStencilSource::vtkUltrasoundImageStencilSource()
{
  this->ClipRectangle[0] = -1e8;
  this->ClipRectangle[1] = -1e8;
  this->ClipRectangle[2] = +1e8;
  this->ClipRectangle[3] = +1e8;

  this->FanAngles[0] = 0.0;
  this->FanAngles[1] = 0.0;
  
  this->FanOrigin[0] = 0.0;
  this->FanOrigin[1] = 0.0;

  this->FanDepth = +1e8;
  // VTK 5
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkUltrasoundImageStencilSource::~vtkUltrasoundImageStencilSource()
{
}

//----------------------------------------------------------------------------
void vtkUltrasoundImageStencilSource::PrintSelf(ostream& os,
                                                  vtkIndent indent)
{
  vtkImageStencilSource::PrintSelf(os,indent);

  os << indent << "ClipRectangle: (" << this->ClipRectangle[0] << ", "
     << this->ClipRectangle[1] << ", " << this->ClipRectangle[2] << ", "
     << this->ClipRectangle[3] << ")\n";
  os << indent << "FanAngles: (" << this->FanAngles[0] << ", "
     << this->FanAngles[1] << ")\n";
  os << indent << "FanOrigin: (" << this->FanOrigin[0] << ", "
     << this->FanOrigin[1] << ")\n";
  os << indent << "FanDepth: " << this->FanDepth << "\n";
}

//----------------------------------------------------------------------------
// set up the clipping extents from an implicit function by brute force
// (i.e. by evaluating the function at each and every voxel)
// void vtkUltrasoundImageStencilSource::ThreadedExecute(vtkImageStencilData 
//                                                       *data,
//                                                       int
//extent[6], int id)

int vtkUltrasoundImageStencilSource::RequestData(
  vtkInformation *request,
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // VTK 5
  this->Superclass::RequestData(request, inputVector, outputVector);

  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  vtkImageStencilData *data = vtkImageStencilData::SafeDownCast(
    outInfo->Get(vtkDataObject::DATA_OBJECT()));

  int extent[6];
  data->GetExtent(extent);

  vtkFloatingPointType *spacing = data->GetSpacing();
  vtkFloatingPointType *origin = data->GetOrigin();

  double xf = (this->GetFanOrigin()[0] - origin[0])/spacing[0];
  double yf = (this->GetFanOrigin()[1] - origin[1])/spacing[1];

  double d2 = this->GetFanDepth()*this->GetFanDepth();

  vtkFloatingPointType xs = spacing[0];
  vtkFloatingPointType ys = spacing[1];

  double ml = tan(this->GetFanAngles()[0]*0.0174532925199)/xs*ys;
  double mr = tan(this->GetFanAngles()[1]*0.0174532925199)/xs*ys;

  if (ml > mr)
    {
    double tmp = ml; ml = mr; mr = tmp;
    }

  int x0 =  (int)ceil((this->GetClipRectangle()[0] - origin[0])/spacing[0]);
  int x1 = (int)floor((this->GetClipRectangle()[1] - origin[0])/spacing[0]);
  int y0 =  (int)ceil((this->GetClipRectangle()[2] - origin[1])/spacing[1]);
  int y1 = (int)floor((this->GetClipRectangle()[3] - origin[1])/spacing[1]);
  
  if (x0 > x1)
    {
    int tmp = x0; x0 = x1; x1 = tmp;
    }
  if (y0 > y1)
    {
    int tmp = y0; y0 = y1; y1 = tmp;
    }
  
  // for keeping track of progress
  unsigned long count = 0;
  unsigned long target = (unsigned long)
    ((extent[5] - extent[4] + 1)*(extent[3] - extent[2] + 1)/50.0);
  target++;

  // loop through all voxels
  for (int idZ = extent[4]; idZ <= extent[5]; idZ++)
    {
    double z = idZ*spacing[2] + origin[2];

    for (int idY = extent[2]; idY <= extent[3]; idY++)
      {
      // update progress if we're the main thread
      if (count%target == 0) 
	{
	this->UpdateProgress(count/(50.0*target));
	}
      count++;
      

      int r1 = extent[0];
      int r2 = extent[1];

      // next, handle the 'fan' shape of the input
      double y = (yf - idY);
      if (spacing[1] < 0)
        {
        y = -y;
        }
      if (!(ml == 0 && mr == 0))
        {
        // first, check the angle range of the fan
        if (r1 < ml*y + xf )
            {
          r1 = int(ceil(ml*y + xf));
          }
        if (r2 > mr*y + xf)
          {
          r2 = int(floor(mr*y + xf));
          }

        // next, check the radius of the fan
        double dx = (d2 - (y*y)*(ys*ys))/(xs*xs);
        if (dx < 0)
          {
          r1 = extent[0];
          r2 = extent[0]-1;
          }
        else
          {
          dx = sqrt(dx);
          if (r1 < xf - dx)
            {
            r1 = int(ceil(xf - dx));
            }
          if (r2 > xf + dx)
            {
            r2 = int(floor(xf + dx));
            }
          }
        }

      // bound to the ultrasound clip rectangle
      if (r1 < x0)
        {
        r1 = x0;
        }
      if (r2 > x1)
        {
        r2 = x1;
        }
      if (r1 > r2 || idY < y0 || idY > y1) 
        {
        r1 = extent[0];
        r2 = extent[0]-1;
        }

      // insert the extent into the stencil
      if (r2 >= r1)
        {
        data->InsertNextExtent(r1, r2, idY, idZ);
        }
      }
    }
}

int vtkUltrasoundImageStencilSource::FillInputPortInformation(
  int port, vtkInformation *info)
{
  
}

int vtkUltrasoundImageStencilSource::RequestInformation(
  vtkInformation *,
  vtkInformationVector **,
  vtkInformationVector *outputVector)
{
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  // this is an odd source that can produce any requested size.  so its whole
  // extent is essentially infinite. This would not be a great source to
  // connect to some sort of writer or viewer. For a sanity check we will
  // limit the size produced to something reasonable (depending on your
  // definition of reasonable)
  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
               0, VTK_LARGE_INTEGER >> 2,
               0, VTK_LARGE_INTEGER >> 2,
               0, VTK_LARGE_INTEGER >> 2);
  return 1;
}
