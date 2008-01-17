/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundFan.cxx,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.2 $
  Thanks:    Thanks to David G Gobbi who developed this class.

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

#include <limits.h>
#include <float.h>
#include <math.h>
#include "vtkUltrasoundFan.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"


//------------------------------------------------------------------------------
vtkUltrasoundFan* vtkUltrasoundFan::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkUltrasoundFan");
  if(ret)
    {
    return (vtkUltrasoundFan*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkUltrasoundFan;
}




#define DEGREES_TO_RADIANS (3.14159265359/180.0)

//----------------------------------------------------------------------------
vtkUltrasoundFan::vtkUltrasoundFan()
{
  this->RotationAxisDepth = 0;
  this->RotationAngles = NULL;
  this->Interpolate = 0;
  // these items must have reasonable defaults
  this->FanApexDepth = 0;
  this->MaximumDepth = 1000;
  this->FanAngleExtent[0] = -90.0;
  this->FanAngleExtent[1] = +90.0;
}

//----------------------------------------------------------------------------
vtkUltrasoundFan::~vtkUltrasoundFan()
{
  this->SetRotationAngles(NULL);
}

//----------------------------------------------------------------------------
void vtkUltrasoundFan::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkImageToImageFilter::PrintSelf(os,indent);

  os << indent << "RotationAngles: " << this->RotationAngles << "\n";
  if (this->RotationAngles)
    {
    this->RotationAngles->PrintSelf(os,indent.GetNextIndent());
    }
  os << indent << "RotationAxisDepth: " << this->RotationAxisDepth << "\n";
  os << indent << "FanAngleExtent: " << this->FanAngleExtent[0] << " " <<
    this->FanAngleExtent[1] << "\n";
  os << indent << "MaximumDepth: " << this->MaximumDepth << "\n";
  os << indent << "FanApexDepth: " << this->FanApexDepth << "\n";
  os << indent << "Interpolate: " << (this->Interpolate ? "On\n" : "Off\n");
}

void vtkUltrasoundFan::ComputeRequiredInputUpdateExtent(int inExt[6], 
                                                        int outExt[6])
{
  this->GetInput()->UpdateInformation();
  this->GetInput()->GetWholeExtent(inExt);

  inExt[0] = outExt[0];
  inExt[1] = outExt[1];
}

//---------------------------------------------------------------------------
void vtkUltrasoundFan::ExecuteInformation() 
{
  int i;
  float inOrigin[3];
  float inSpacing[3];
  int inExt[6];

  float outOrigin[3];
  float outSpacing[3];
  int outExt[6];

  this->GetInput()->GetSpacing(inSpacing);
  this->GetInput()->GetOrigin(inOrigin);
  this->GetInput()->GetWholeExtent(inExt);

  double maxDepth;

  if (inSpacing[1] < 0)
    {
    maxDepth = inOrigin[1];
    }
  else
    {
    maxDepth = inOrigin[1]+inExt[3]*inSpacing[1];
    }

  double initialZ,finalZ;

  if (this->RotationAngles)
    {
    initialZ = -sin(this->RotationAngles->GetValue(0)*DEGREES_TO_RADIANS)
      * (maxDepth - this->RotationAxisDepth);
    finalZ = -sin(this->RotationAngles->GetValue(this->RotationAngles->\
                   GetNumberOfTuples()-1)*DEGREES_TO_RADIANS)
      * (maxDepth - this->RotationAxisDepth);
    }
  else
    {
    initialZ = sin(inOrigin[2] + inExt[4]*inSpacing[2])
      * (maxDepth - this->RotationAxisDepth);
    finalZ = sin(inOrigin[2] + inExt[5]*inSpacing[2])
      * (maxDepth - this->RotationAxisDepth);      
    }

  outSpacing[0] = inSpacing[0];
  outSpacing[1] = inSpacing[1];
  outSpacing[2] = inSpacing[1];

  if (inSpacing[1] < 0)
    {
    outSpacing[1] = -inSpacing[1];
    outSpacing[2] = -inSpacing[1];
    }

  if (finalZ < initialZ)
    {
    double tmp = initialZ;
    initialZ = finalZ;
    finalZ = tmp;
    }

  outOrigin[0] = inOrigin[0];
  outOrigin[1] = this->RotationAxisDepth;
  outOrigin[2] = initialZ;

  outExt[0] = inExt[0];
  outExt[1] = inExt[1];
  outExt[2] = 0;
  outExt[3] = outExt[2]+int(ceil((maxDepth-outOrigin[1])/outSpacing[1]))-1;
  outExt[4] = 0;
  outExt[5] = outExt[4]+int(ceil((finalZ-outOrigin[2])/outSpacing[2]))-1;


  for (i = 0; i < 3; i++)
    {
    this->OutputExtent[2*i] = outExt[2*i]; 
    this->OutputExtent[2*i+1] = outExt[2*i+1]; 
    this->OutputSpacing[i] = outSpacing[i];
    this->OutputOrigin[i] = outOrigin[i];
    }

  this->GetOutput()->SetNumberOfScalarComponents( \
                this->GetInput()->GetNumberOfScalarComponents());
  this->GetOutput()->SetScalarType(this->GetInput()->GetScalarType());
  this->GetOutput()->SetWholeExtent(this->OutputExtent);
  this->GetOutput()->SetSpacing(this->OutputSpacing);
  this->GetOutput()->SetOrigin(this->OutputOrigin);
}

//  Search a table of theta values to find the two thetas, theta_n and
//  theta_(n+1),  between which the specified theta lies.
//  The index n (both the whole-number and the fractional component)
//  is returned.
static double vtkGetThetaIndexFromTable(vtkDoubleArray *table, double theta)
{
  int n = table->GetNumberOfTuples();
  double index;
  int i;

  double theta0 = table->GetValue(0);
  double theta1 = table->GetValue(n-1);

  // calculate a good first guess for i
  index = (theta-theta0)/(theta1-theta0)*(n-1);
  if (index < 0)
    {
    return -1;
    }
  if (index > n-1)
    {
    return n;
    }

  i = (int)index;

  for (;;)
    {
    theta0 = table->GetValue(i);
    theta1 = table->GetValue(i+1);
    index = (theta-theta0)/(theta1-theta0);
    if (index < 0.0)
      {
      i--;
      }
    else if (index >= 1.0)
      {
      i++;
      }
    else
      {
      return i+index;
      }
    }   
}

//----------------------------------------------------------------------------
// rounding functions, split and optimized for each type
// (because we don't want to round if the result is a float!)

// in the case of a tie between integers, the larger integer wins.

static inline void vtkUltrasoundRound(double val, unsigned char& rnd)
{
  rnd = (unsigned char)(val+0.5);
}

static inline void vtkUltrasoundRound(double val, short& rnd)
{
  rnd = (short)((int)(val+32768.5)-32768);
}

static inline void vtkUltrasoundRound(double val, unsigned short& rnd)
{
  rnd = (unsigned short)(val+0.5);
}

static inline void vtkUltrasoundRound(double val, int& rnd)
{
  rnd = (int)(floor(val+0.5));
}

static inline void vtkUltrasoundRound(double val, float& rnd)
{
  rnd = (float)(val);
}

//----------------------------------------------------------------------------
// This templated function executes the filter for any type of data.
// (this one function is pretty much the be-all and end-all of the
// filter)
template <class T>
static void vtkUltrasoundFanExecute(vtkUltrasoundFan *self,
                                    vtkImageData *inData, T *inPtr,
                                    vtkImageData *outData, T *outPtr,
                                    int outExt[6], int id)
{
  int numScalars, rowLength;
  int idC, idX, idY, idZ;
  int outIncX, outIncY, outIncZ;
  int inExt[6], inInc[3];
  float inOrigin[3],outOrigin[3];
  float inSpacing[3],outSpacing[3];
  unsigned long count = 0;
  unsigned long target;

  // find maximum input range
  inData->GetExtent(inExt);

  // set target so we know how far along we are
  target = (unsigned long)
    ((outExt[5]-outExt[4]+1)*(outExt[3]-outExt[2]+1)/50.0);
  target++;
  
  // Get Increments to march through data 
  inData->GetIncrements(inInc);
  outData->GetContinuousIncrements(outExt, outIncX, outIncY, outIncZ);
  numScalars = inData->GetNumberOfScalarComponents();

  // Get spacing and origin
  inData->GetSpacing(inSpacing);
  inData->GetOrigin(inOrigin);
  outData->GetSpacing(outSpacing);
  outData->GetOrigin(outOrigin);

  double y0 = self->GetRotationAxisDepth();
  double depth0 = self->GetFanApexDepth();
  double y,z,dy,theta,depth;

  // phi describes the sweep of the utrasound beam within a single image
  double *phiExtent = self->GetFanAngleExtent();
  double tanPhi0 = -1000.0;  // small number 
  double tanPhi1 = +1000.0;  // large number

  if (phiExtent[0] > -90.0)
    {
    tanPhi0 = tan(phiExtent[0]*DEGREES_TO_RADIANS);
    }
  if (phiExtent[1] < +90.0)
    {
    tanPhi1 = tan(phiExtent[1]*DEGREES_TO_RADIANS);
    }

  // prime the input pointer
  inPtr += outExt[0]*inInc[0];

  // Loop through ouput pixels
  for (idZ = outExt[4]; idZ <= outExt[5]; idZ++)
    {
    for (idY = outExt[2]; idY <= outExt[3]; idY++)
      {
      if (!id) 
        {
        if (!(count%target)) 
          {
          self->UpdateProgress(count/(50.0*target));
          }
        count++;
        }

      y = outOrigin[1]+idY*outSpacing[1];
      z = outOrigin[2]+idZ*outSpacing[2];
      dy = y-y0;

      // calculate the 'theta' and 'depth' input coordinates that
      // correspond to the current y and z output coordinates
      // (the input and output x is the same)
      theta = atan2(z,dy)/DEGREES_TO_RADIANS; // theta is in degrees
      depth = sqrt(dy*dy+z*z)+y0; // depth is in millimetres

      // convert depth and theta into indices so that we can
      // interpolate
      double idDepth = (depth-inOrigin[1])/inSpacing[1];
      double idTheta = vtkGetThetaIndexFromTable(self->GetRotationAngles(),
                                                 theta)+inExt[4];

      int idDepth1 = int(idDepth+1.0);
      int idTheta1 = int(idTheta+32768.0)-32767; // ensure proper rounding

      int idDepth0 = idDepth1 - 1;
      int idTheta0 = idTheta1 - 1;

      int idDepthNearest = int(idDepth+1.5)-1;
      int idThetaNearest = int(idTheta+32767.5)-32767; // ensure proper round
        
      // don't interpolate if not necessary
      if (idDepth0 == idDepth)
        {
        idDepth1 = idDepth0;
        }
      if (idTheta0 == idTheta)
        {
        idTheta1 = idTheta0;
        }

      if (idDepth0 < inExt[2] || idDepth1 > inExt[3] ||
          idTheta0 <= inExt[4] || idTheta1 >= inExt[5] ||
          depth < 0.0 || depth > self->GetMaximumDepth())
        { 
        // out of bounds! clear to black
        rowLength = (outExt[1]-outExt[0]+1)*numScalars;
        memset(outPtr,0,rowLength*sizeof(T));
        outPtr += rowLength;
        }
      else
        {
        int minX = outExt[0];
        int maxX = outExt[1];
        
        // check to see what the x extent should be
        // first, according to the specified beam-sweep angle
        int tmpMinX = int((depth*tanPhi0-outOrigin[0])/outSpacing[0]+1.5)-1;
        int tmpMaxX = int((depth*tanPhi1-outOrigin[0])/outSpacing[0]+1.5)-1;
        
        if (tmpMinX > minX)
          {
          minX = tmpMinX;
          }
        if (tmpMaxX < maxX)
          {
          maxX = tmpMaxX;
          }

        // next, according to the depth of penetration of the beam
        double absX = sqrt(self->GetMaximumDepth()*self->GetMaximumDepth()
                           - depth*depth);
        tmpMinX = int((-absX - outOrigin[0])/outSpacing[0] + 1.5) - 1;  
        tmpMaxX = int((absX - outOrigin[0])/outSpacing[0] + 1.5) - 1;  
        
        if (tmpMinX > minX)
          {
          minX = tmpMinX;
          }
        if (tmpMaxX < maxX)
          {
          maxX = tmpMaxX;
          }
      
        // make sure the limits themselves are within limits
        if (minX > maxX)
          {
          minX = outExt[1]+1;
          maxX = outExt[1];
          }
        
        // clear leading space
        rowLength = (minX-outExt[0])*numScalars;
        memset(outPtr,0,rowLength*sizeof(T));
        outPtr += rowLength;

        // reconstruct where appropriate
        T *inPtrTmp = inPtr+rowLength;

        if (self->GetInterpolate())
          { // do linear interpolation
        
          // extract whole and fractional portion of indices
          double fd = idDepth-idDepth0;
          double ft = idTheta-idTheta0;

          double rd = 1.0-fd;
          double rt = 1.0-ft;

          double f00 = rd*rt;
          double f01 = rd*ft;
          double f10 = fd*rt;
          double f11 = fd*ft;
        
          int factDepth0 = idDepth0*inInc[1];
          int factTheta0 = idTheta0*inInc[2];
          int factDepth1 = idDepth1*inInc[1];
          int factTheta1 = idTheta1*inInc[2];
        
          int inc00 = factDepth0 + factTheta0;
          int inc01 = factDepth0 + factTheta1;
          int inc10 = factDepth1 + factTheta0;
          int inc11 = factDepth1 + factTheta1;

          for (idX = minX; idX <= maxX; idX++)
            {
            for (idC = 0; idC < numScalars; idC++)
              {
              vtkUltrasoundRound(f00*inPtrTmp[inc00] + f01*inPtrTmp[inc01] + 
                                 f10*inPtrTmp[inc10] + f11*inPtrTmp[inc11],
                                 *outPtr++);
              inPtrTmp++;
              }
            }
          }
        else
          { // do nearest-neighbor as efficiently as possible
          rowLength = (maxX-minX+1)*numScalars;
          memcpy(outPtr,
                 inPtrTmp+idDepthNearest*inInc[1]+idThetaNearest*inInc[2],
                 rowLength*sizeof(T));
          outPtr += rowLength;
          }

        // clear trailing space
        rowLength = (outExt[1]-maxX)*numScalars;
        memset(outPtr,0,rowLength*sizeof(T));
        outPtr += rowLength;
        }
      outPtr += outIncY;
      }
    outPtr += outIncZ;
    }
}

//----------------------------------------------------------------------------
// This method is passed a input and output region, and executes the filter
// algorithm to fill the output from the input.
// It just executes a switch statement to call the correct function for
// the regions data types.
void vtkUltrasoundFan::ThreadedExecute(vtkImageData *inData, 
                                       vtkImageData *outData,
                                       int outExt[6], int id)
{
  void *inPtr = inData->GetScalarPointerForExtent(inData->GetExtent());
  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  
  vtkDebugMacro(<< "Execute: inData = " << inData 
  << ", outData = " << outData);
  
  // this filter expects that input is the same type as output.
  if (inData->GetScalarType() != outData->GetScalarType())
    {
    vtkErrorMacro(<< "Execute: input ScalarType, " << inData->GetScalarType()
            << ", must match out ScalarType " << outData->GetScalarType());
    return;
    }

  switch (inData->GetScalarType())
    {
    case VTK_FLOAT:
      vtkUltrasoundFanExecute(this, inData, (float *)(inPtr), 
                              outData, (float *)(outPtr),outExt, id);
      break;
    case VTK_INT:
      vtkUltrasoundFanExecute(this, inData, (int *)(inPtr), 
                              outData, (int *)(outPtr),outExt, id);
      break;
    case VTK_SHORT:
      vtkUltrasoundFanExecute(this, inData, (short *)(inPtr), 
                              outData, (short *)(outPtr),outExt, id);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkUltrasoundFanExecute(this, inData, (unsigned short *)(inPtr), 
                              outData, (unsigned short *)(outPtr),outExt,id);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkUltrasoundFanExecute(this, inData, (unsigned char *)(inPtr), 
                              outData, (unsigned char *)(outPtr),outExt, id);
      break;
    default:
      vtkErrorMacro(<< "Execute: Unknown input ScalarType");
      return;
    }
}


