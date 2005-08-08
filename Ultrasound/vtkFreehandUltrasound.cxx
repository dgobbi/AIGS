/*=========================================================================

Copyright (c) 2000,2002 David Gobbi.

=========================================================================*/

#include <limits.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

// includes for mkdir
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#endif

#include "fixed.h"
#include "vtkFreehandUltrasound.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkImageData.h"
#include "vtkMultiThreader.h"
#include "vtkCriticalSection.h"
#include "vtkTimerLog.h"
#include "vtkTrackerBuffer.h"
#include "vtkObjectFactory.h"
#include "vtkVideoSource.h"
#include "vtkTrackerTool.h"
#include "vtkPNGWriter.h"

vtkCxxRevisionMacro(vtkFreehandUltrasound, "$Revision: 1.2 $");
vtkStandardNewMacro(vtkFreehandUltrasound);

vtkCxxSetObjectMacro(vtkFreehandUltrasound,VideoSource,vtkVideoSource);
vtkCxxSetObjectMacro(vtkFreehandUltrasound,TrackerTool,vtkTrackerTool);
vtkCxxSetObjectMacro(vtkFreehandUltrasound,Slice,vtkImageData);
vtkCxxSetObjectMacro(vtkFreehandUltrasound,SliceAxes,vtkMatrix4x4);
vtkCxxSetObjectMacro(vtkFreehandUltrasound,SliceTransform,vtkLinearTransform);

// for keeping track of threading information
struct vtkFreehandThreadStruct
{
  vtkFreehandUltrasound *Filter;
  vtkImageData   *Input;
  vtkImageData   *Output;
};

//----------------------------------------------------------------------------
vtkFreehandUltrasound::vtkFreehandUltrasound()
{
  // set the video lag (i.e the lag between tracking information and
  // video information)
  this->VideoLag = 0.0;

  // set up the output (it will have been created in the superclass)
  // (the output is the reconstruction volume, the second component
  // is the alpha component that stores whether or not a voxel has
  // been touched by the reconstruction)
  this->GetOutput()->SetScalarType(VTK_UNSIGNED_CHAR);
  this->GetOutput()->SetNumberOfScalarComponents(2);

  // also see ExecuteInformation for how these are set
  this->OutputSpacing[0] = 1.0;
  this->OutputSpacing[1] = 1.0;
  this->OutputSpacing[2] = 1.0;

  this->OutputOrigin[0] = -127.5;
  this->OutputOrigin[1] = -127.5;
  this->OutputOrigin[2] = -127.5;

  this->OutputExtent[0] = 0;
  this->OutputExtent[1] = 255;
  this->OutputExtent[2] = 0;
  this->OutputExtent[3] = 255;
  this->OutputExtent[4] = 0;
  this->OutputExtent[5] = 255;

  // accumulation buffer is for compounding, there is a voxel in
  // the accumulation buffer for each voxel in the output
  this->AccumulationBuffer = vtkImageData::New();

  // this is set if some parameter is changed which causes the data
  // in the output to become invalid: the output will be erased
  // during the next ExecuteInformation
  this->NeedsClear = 1;

  // quality parameters
  this->InterpolationMode = VTK_FREEHAND_NEAREST; // no interpolation
  this->Compounding = 0; // don't average data, overwrite instead

  // optimization: 
  //   0 means no optimization (almost never used)
  //   1 means break transformation into x, y and z components, and
  //      don't do bounds checking for nearest-neighbor interpolation
  //   2 means used fixed-point (i.e. integer) math instead of float math
  this->Optimization = 2;

  // the slice is the vtkImageData 'slice' (kind of like an input)
  // that is inserted into the reconstructed 3D volume (the output)
  this->Slice = NULL;

  // the slice axes matrix and slice transform together give the
  // coordinate transformation from the local coordinate system
  // of the Slice to the coordinate system of the Output.
  this->SliceAxes = NULL;
  this->SliceTransform = NULL;

  // the IndexMatrix gives the coordinate transformation from (i,j,k)
  // voxel indices in the slice to (i,j,k) voxel indices in the
  // output.
  this->IndexMatrix = NULL;
  this->LastIndexMatrix = NULL;

  this->ClipRectangle[0] = -1e8;
  this->ClipRectangle[1] = -1e8;
  this->ClipRectangle[2] = +1e8;
  this->ClipRectangle[3] = +1e8;

  this->FanAngles[0] = 0.0;
  this->FanAngles[1] = 0.0;
  
  this->FanOrigin[0] = 0.0;
  this->FanOrigin[1] = 0.0;

  this->FanDepth = +1e8;

  // one thread for each CPU is used for the reconstruction
  this->Threader = vtkMultiThreader::New();
  this->NumberOfThreads = this->Threader->GetNumberOfThreads();  

  // for running the reconstruction in the background
  this->VideoSource = NULL;
  this->TrackerTool = NULL;
  this->TrackerBuffer = vtkTrackerBuffer::New();
  this->ReconstructionThreader = vtkMultiThreader::New();
  this->ReconstructionRate = 0;
  this->ReconstructionThreadId = -1;
  this->RealTimeReconstruction = 0; // # real-time or buffered
  this->ReconstructionFrameCount = 0; // # of frames to reconstruct
}

//----------------------------------------------------------------------------
vtkFreehandUltrasound::~vtkFreehandUltrasound()
{
  this->StopRealTimeReconstruction();

  this->SetSlice(NULL);
  this->SetSliceTransform(NULL);
  this->SetSliceAxes(NULL);

  if (this->IndexMatrix)
    {
    this->IndexMatrix->Delete();
    }
  if (this->LastIndexMatrix)
    {
    this->LastIndexMatrix->Delete();
    }
  if (this->AccumulationBuffer)
    {
    this->AccumulationBuffer->Delete();
    }
  if (this->Threader)
    {
    this->Threader->Delete();
    }
  this->SetVideoSource(NULL);
  this->SetTrackerTool(NULL);
  if (this->TrackerBuffer)
    {
    this->TrackerBuffer->Delete();
    }
  if (this->ReconstructionThreader)
    {
    this->ReconstructionThreader->Delete();
    }
}

//----------------------------------------------------------------------------
// convert the ClipRectangle (which is in millimetre coordinates) into a
// clip extent that can be applied to the input data.
void vtkFreehandUltrasound::GetClipExtent(int clipExt[6],
				     vtkFloatingPointType inOrigin[3],
				     vtkFloatingPointType inSpacing[3],
				     const int inExt[6])
{
  int x0 = (int)ceil((this->GetClipRectangle()[0]-inOrigin[0])/inSpacing[0]);
  int x1 = (int)floor((this->GetClipRectangle()[2]-inOrigin[0])/inSpacing[0]);
  int y0 = (int)ceil((this->GetClipRectangle()[1]-inOrigin[1])/inSpacing[1]);
  int y1 = (int)floor((this->GetClipRectangle()[3]-inOrigin[1])/inSpacing[1]);

  if (x0 > x1)
    {
    int tmp = x0; x0 = x1; x1 = tmp;
    }
  if (y0 > y1)
    {
    int tmp = y0; y0 = y1; y1 = tmp;
    }

  // make sure this lies within extent
  if (x0 < inExt[0])
    {
    x0 = inExt[0];
    }
  if (x1 > inExt[1])
    {
    x1 = inExt[1];
    }
  if (x0 > x1)
    {
    x0 = inExt[0];
    x1 = inExt[0]-1;
    }

  if (y0 < inExt[2])
    {
    y0 = inExt[2];
    }
  if (y1 > inExt[3])
    {
    y1 = inExt[3];
    }
  if (y0 > y1)
    {
    y0 = inExt[2];
    y1 = inExt[2]-1;
    }

  clipExt[0] = x0;
  clipExt[1] = x1;
  clipExt[2] = y0;
  clipExt[3] = y1;
  clipExt[4] = inExt[4];
  clipExt[5] = inExt[5];
}

//----------------------------------------------------------------------------
// Calculate the maximum distance between two slices, given the
// index transformation matrix for each.
// This method is currently not used, so chances are very good that
// it will NOT provide the correct result.
double vtkFreehandUltrasound::CalculateMaxSliceSeparation(vtkMatrix4x4 *m1,
                                                          vtkMatrix4x4 *m2)
{
  // The first thing to do is find the four corners of the plane.
  vtkImageData *inData = this->GetSlice();
  inData->UpdateInformation();

  vtkFloatingPointType inSpacing[3];
  vtkFloatingPointType inOrigin[3];
  int inExtent[6];

  inData->GetSpacing(inSpacing);
  inData->GetOrigin(inOrigin);
  inData->GetWholeExtent(inExtent);

  double x0 = ceil((this->GetClipRectangle()[0]-inOrigin[0])/inSpacing[0]);
  double x1 = floor((this->GetClipRectangle()[2]-inOrigin[0])/inSpacing[0]);
  double y0 = ceil((this->GetClipRectangle()[1]-inOrigin[1])/inSpacing[1]);
  double y1 = floor((this->GetClipRectangle()[3]-inOrigin[1])/inSpacing[1]);

  if (x0 < inExtent[0])
    {
    x0 = inExtent[0];
    }
  if (x1 > inExtent[1])
    {
    x1 = inExtent[1];
    }
  if (y0 < inExtent[2])
    {
    y0 = inExtent[2];
    }
  if (y1 > inExtent[3])
    {
    y1 = inExtent[3];
    }

  // next convert to a normal, origin, and radius i.e. approximate the
  // plane as a disk
  double point1[4],point2[4];
  point1[0] = 0.5*(x0 + x1);
  point1[1] = 0.5*(y0 + y1);
  point1[2] = 0.5*(inExtent[4] + inExtent[5]);
  point1[3] = 1.0;
 
  double normal1[4],normal2[4];
  normal1[0] = 0.0;
  normal1[1] = 0.0;
  normal1[2] = 1.0;
  normal1[3] = 0.0;

  double r = 0.5*sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1*y0));

  // divide the matrices to get the relative transformation
  double matrix[4][4];
  vtkMatrix4x4::DeepCopy(*matrix,*m1->Element);
  vtkMatrix4x4::Invert(*matrix,*matrix);
  vtkMatrix4x4::Multiply4x4(*matrix,*m2->Element,*matrix);
  vtkMatrix4x4::MultiplyPoint(*matrix,point1,point2);

  vtkMatrix4x4::Invert(*matrix,*matrix);
  vtkMatrix4x4::Transpose(*matrix,*matrix);
  vtkMatrix4x4::MultiplyPoint(*matrix,normal1,normal2);

  double f = 1.0/sqrt(normal2[0]*normal2[0] + 
                      normal2[1]*normal2[1] + 
                      normal2[2]*normal2[2]);
  normal2[0] *= f;
  normal2[1] *= f;
  normal2[2] *= f;

  // find distance produced by tilt of planes
  double dd = r*fabs(sqrt(normal2[0]*normal2[0] +
                          normal2[1]*normal2[1])/normal2[2]);

  // find the distance the centre point moved
  double d = sqrt((point1[0]-point2[0])*(point1[0]-point2[0]) + 
                  (point1[1]-point2[1])*(point1[1]-point2[1]) +
                  (point1[2]-point2[2])*(point1[2]-point2[2]));

  // the maximum separation is guaranteed to be less than the sum of these.
  // also, multiply by the mean pixel spacing (the pixels will be approximately
  // square)
  return (d + dd)*sqrt(inSpacing[0]*inSpacing[1]);
}


//----------------------------------------------------------------------------
void vtkFreehandUltrasound::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkImageSource::PrintSelf(os,indent);

  os << indent << "VideoLag: " << this->VideoLag << "\n";

  os << indent << "SliceAxes: " << this->SliceAxes << "\n";
  if (this->SliceAxes)
    {
    this->SliceAxes->PrintSelf(os,indent.GetNextIndent());
    }
  os << indent << "SliceTransform: " << this->SliceTransform << "\n";
  if (this->SliceTransform)
    {
    this->SliceTransform->PrintSelf(os,indent.GetNextIndent());
    }
  os << indent << "OutputSpacing: " << this->OutputSpacing[0] << " " <<
    this->OutputSpacing[1] << " " << this->OutputSpacing[2] << "\n";
  os << indent << "OutputOrigin: " << this->OutputOrigin[0] << " " <<
    this->OutputOrigin[1] << " " << this->OutputOrigin[2] << "\n";
  os << indent << "OutputExtent: " << this->OutputExtent[0] << " " <<
    this->OutputExtent[1] << " " << this->OutputExtent[2] << " " <<
    this->OutputExtent[3] << " " << this->OutputExtent[4] << " " <<
    this->OutputExtent[5] << "\n";
  os << indent << "InterpolationMode: " 
     << this->GetInterpolationModeAsString() << "\n";
  os << indent << "Optimization: " << (this->Optimization ? "On\n":"Off\n");
  os << indent << "Compounding: " << (this->Compounding ? "On\n":"Off\n");
  os << indent << "NumberOfThreads: " << this->NumberOfThreads << "\n";
}

//----------------------------------------------------------------------------
// Account for the MTime of the transform and its matrix when determining
// the MTime of the filter
// [note to self: this made sense in vtkImageReslice, but does it make
//  any sense here?]
unsigned long int vtkFreehandUltrasound::GetMTime()
{
  unsigned long mTime=this->vtkImageSource::GetMTime();
  unsigned long time;

  if ( this->SliceTransform != NULL )
    {
    time = this->SliceTransform->GetMTime();
    mTime = ( time > mTime ? time : mTime );
    time = this->SliceTransform->GetMatrix()->GetMTime();
    mTime = ( time > mTime ? time : mTime );    
    }

  return mTime;
}

//----------------------------------------------------------------------------
// In most VTK classes this method is responsible for calling Execute,
// but since the output data has already been generated it just fools
// the pipeline into thinking that Execute has been called.
void vtkFreehandUltrasound::UpdateData(vtkDataObject *outObject) 
{
  if (this->ReconstructionThreadId == -1 && this->NeedsClear == 1)
    {
    this->InternalClearOutput();
    }

  ((vtkImageData *)outObject)->DataHasBeenGenerated();
}

//----------------------------------------------------------------------------
// The ExecuteInformation gets the Output ready to receive data, so we
// need to call it before the reconstruction starts.
void vtkFreehandUltrasound::InternalExecuteInformation() 
{
  vtkImageData *output = this->GetOutput();
  int oldwholeextent[6];
  vtkFloatingPointType oldspacing[3];
  vtkFloatingPointType oldorigin[3];
  int oldtype = output->GetScalarType();
  int oldncomponents = output->GetNumberOfScalarComponents();
  output->GetWholeExtent(oldwholeextent);
  output->GetSpacing(oldspacing);
  output->GetOrigin(oldorigin);

  if (this->GetVideoSource())
    {
    if (this->Slice == 0)
      {
      this->Slice = this->GetVideoSource()->GetOutput();
      this->Slice->Register(this);
      }
    }
  // the 'Slice' is not an input, but it is like an input
  if (this->Slice)
    {
    this->Slice->UpdateInformation();
    output->SetScalarType(this->Slice->GetScalarType());
    output->SetNumberOfScalarComponents(this->Slice->\
                                        GetNumberOfScalarComponents()+1);
    // the PipelineMTime() is usually handled in the superclass,
    //  but because 'Slice' is registered as an input it has
    //  to be done here instead.
    unsigned long mtime = this->Slice->GetPipelineMTime();
    if (mtime > output->GetPipelineMTime())
      {
      output->SetPipelineMTime(mtime);
      }
    }

  // set up the output dimensions and info here
  output->SetWholeExtent(this->OutputExtent);
  output->SetSpacing(this->OutputSpacing);
  output->SetOrigin(this->OutputOrigin);

  // check to see if output has changed
  if (oldtype != output->GetScalarType() ||
      oldncomponents != output->GetNumberOfScalarComponents() ||
      oldwholeextent[0] != this->OutputExtent[0] ||
      oldwholeextent[1] != this->OutputExtent[1] ||
      oldwholeextent[2] != this->OutputExtent[2] ||
      oldwholeextent[3] != this->OutputExtent[3] ||
      oldwholeextent[4] != this->OutputExtent[4] ||
      oldwholeextent[5] != this->OutputExtent[5] ||
      oldspacing[0] != this->OutputSpacing[0] ||
      oldspacing[1] != this->OutputSpacing[1] ||
      oldspacing[2] != this->OutputSpacing[2] ||
      oldorigin[0] != this->OutputOrigin[0] ||
      oldorigin[1] != this->OutputOrigin[1] ||
      oldorigin[2] != this->OutputOrigin[2])
    {
    this->NeedsClear = 1;
    }

  // set up the accumulation buffer to be the same size as the
  // output
  if (this->Compounding)
    {
    this->AccumulationBuffer->SetScalarType(VTK_UNSIGNED_SHORT);
    this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
    this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
    this->AccumulationBuffer->SetOrigin(this->OutputOrigin);
    int *extent = this->AccumulationBuffer->GetExtent();
    if (extent[0] != this->OutputExtent[0] ||
        extent[1] != this->OutputExtent[1] ||
        extent[2] != this->OutputExtent[2] ||
        extent[3] != this->OutputExtent[3] ||
        extent[4] != this->OutputExtent[4] ||
        extent[5] != this->OutputExtent[5])
      {
      this->NeedsClear = 1;
      }
    }
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::ExecuteInformation() 
{
  // to avoid conflict between the main application thread and the
  // realtime reconstruction thread
  if (this->ReconstructionThreadId == -1)
    {
    this->InternalExecuteInformation();
    }
}

//----------------------------------------------------------------------------
//  Interpolation subroutines and associated code
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// rounding functions, split and optimized for each type
// (because we don't want to round if the result is a float!)

// in the case of a tie between integers, the larger integer wins.

// The 'floor' function on x86 and mips is many times slower than these
// and is used a lot in this code, optimize for different CPU architectures
static inline int vtkUltraFloor(double x)
{
#if defined mips || defined sparc
  return (int)((unsigned int)(x + 2147483648.0) - 2147483648U);
#elif defined i386
  double tempval = (x - 0.25) + 3377699720527872.0; // (2**51)*1.5
  return ((int*)&tempval)[0] >> 1;
#else
  return int(floor(x));
#endif
}

static inline int vtkUltraCeil(double x)
{
  return -vtkUltraFloor(-x - 1.0) - 1;
}

static inline int vtkUltraRound(double x)
{
  return vtkUltraFloor(x + 0.5);
}

static inline int vtkUltraFloor(float x)
{
  return vtkUltraFloor((double)x);
}

static inline int vtkUltraCeil(float x)
{
  return vtkUltraCeil((double)x);
}

static inline int vtkUltraRound(float x)
{
  return vtkUltraRound((double)x);
}

static inline int vtkUltraFloor(fixed x)
{
  return x.floor();
}

static inline int vtkUltraCeil(fixed x)
{
  return x.ceil();
}

static inline int vtkUltraRound(fixed x)
{
  return x.round();
}


// convert a float into an integer plus a fraction
template <class F>
static inline int vtkUltraFloor(F x, F &f)
{
  int ix = vtkUltraFloor(x);
  f = x - ix;

  return ix;
}

template <class F, class T>
static inline void vtkUltraRound(F val, T& rnd)
{
  rnd = vtkUltraRound(val);
}

//----------------------------------------------------------------------------
// Do nearest-neighbor interpolation of the input data 'inPtr' of extent 
// 'inExt' at the 'point'.  The result is placed at 'outPtr'.  
// If the lookup data is beyond the extent 'inExt', set 'outPtr' to
// the background color 'background'.  
// The number of scalar components in the data is 'numscalars'

template <class F, class T>
static int vtkNearestNeighborInterpolation(F *point, T *inPtr, T *outPtr,
                                           unsigned short *accPtr, 
                                           int numscalars, 
                                           int outExt[6], int outInc[3])
{
  int i;
  int outIdX = vtkUltraRound(point[0])-outExt[0];
  int outIdY = vtkUltraRound(point[1])-outExt[2];
  int outIdZ = vtkUltraRound(point[2])-outExt[4];
  
  // fancy way of checking bounds
  if ((outIdX | (outExt[1]-outExt[0] - outIdX) |
       outIdY | (outExt[3]-outExt[2] - outIdY) |
       outIdZ | (outExt[5]-outExt[4] - outIdZ)) >= 0)
    {
    int inc = outIdX*outInc[0]+outIdY*outInc[1]+outIdZ*outInc[2];
    outPtr += inc;
    if (accPtr)
      {
      // accumulation buffer: do compounding
      accPtr += inc/outInc[0];
      int newa = *accPtr + 255;
      for (i = 0; i < numscalars; i++)
        {
        *outPtr = ((*inPtr++)*255 + (*outPtr)*(*accPtr))/newa;
        outPtr++;
        }
      *outPtr = 255;
      *accPtr = 65535;
      if (newa < 65535)
        {
        *accPtr = newa;
        }
      }
    else
      {
      // no accumulation buffer, replace what was there before
      for (i = 0; i < numscalars; i++)
        {
        *outPtr++ = *inPtr++;
        }
      *outPtr = 255;
      }
    return 1;
    }
  return 0;
} 

// Do trilinear interpolation of the input data 'inPtr' of extent 'inExt'
// at the 'point'.  The result is placed at 'outPtr'.  
// If the lookup data is beyond the extent 'inExt', set 'outPtr' to
// the background color 'background'.  
// The number of scalar components in the data is 'numscalars'
template <class F, class T>
static int vtkTrilinearInterpolation(F *point, T *inPtr, T *outPtr,
                                     unsigned short *accPtr, int numscalars, 
                                     int outExt[6], int outInc[3])
{
  F fx, fy, fz;

  int outIdX0 = vtkUltraFloor(point[0], fx);
  int outIdY0 = vtkUltraFloor(point[1], fy);
  int outIdZ0 = vtkUltraFloor(point[2], fz);

  int outIdX1 = outIdX0 + (fx != 0);
  int outIdY1 = outIdY0 + (fy != 0);
  int outIdZ1 = outIdZ0 + (fz != 0);
  
  // bounds check
  if ((outIdX0 | (outExt[1]-outExt[0] - outIdX1) |
       outIdY0 | (outExt[3]-outExt[2] - outIdY1) |
       outIdZ0 | (outExt[5]-outExt[4] - outIdZ1)) >= 0)
    {// do reverse trilinear interpolation
    int factX0 = outIdX0*outInc[0];
    int factY0 = outIdY0*outInc[1];
    int factZ0 = outIdZ0*outInc[2];

    int factX1 = outIdX1*outInc[0];
    int factY1 = outIdY1*outInc[1];
    int factZ1 = outIdZ1*outInc[2];

    int factY0Z0 = factY0 + factZ0;
    int factY0Z1 = factY0 + factZ1;
    int factY1Z0 = factY1 + factZ0;
    int factY1Z1 = factY1 + factZ1;

    int idx[8];
    idx[0] = factX0 + factY0Z0;
    idx[1] = factX0 + factY0Z1;
    idx[2] = factX0 + factY1Z0;
    idx[3] = factX0 + factY1Z1;
    idx[4] = factX1 + factY0Z0;
    idx[5] = factX1 + factY0Z1;
    idx[6] = factX1 + factY1Z0;
    idx[7] = factX1 + factY1Z1;

    F rx = 1 - fx;
    F ry = 1 - fy;
    F rz = 1 - fz;
      
    F ryrz = ry*rz;
    F ryfz = ry*fz;
    F fyrz = fy*rz;
    F fyfz = fy*fz;

    F fdx[8];
    fdx[0] = rx*ryrz;
    fdx[1] = rx*ryfz;
    fdx[2] = rx*fyrz;
    fdx[3] = rx*fyfz;
    fdx[4] = fx*ryrz;
    fdx[5] = fx*ryfz;
    fdx[6] = fx*fyrz;
    fdx[7] = fx*fyfz;
    
    F f, r, a;
    T *inPtrTmp, *outPtrTmp;
    if (accPtr)
      {
      //------------------------------------
      // accumulation buffer: do compounding
      unsigned short *accPtrTmp;

      // loop over the eight voxels
      int j = 8;
      do 
        {
        j--;
        if (fdx[j] == 0)
          {
          continue;
          }
        inPtrTmp = inPtr;
        outPtrTmp = outPtr+idx[j];
        accPtrTmp = accPtr+idx[j]/outInc[0];
        f = fdx[j];
        r = F(*accPtrTmp)/255;
        a = f + r;
        int i = numscalars;
        do
          {
          i--;
          vtkUltraRound((f*(*inPtrTmp++) + r*(*outPtrTmp))/a,
                          *outPtrTmp);
          outPtrTmp++;
          }
        while (i);
        // set accumulation
        *accPtrTmp = 65535;
        *outPtrTmp = 255;
        a *= 255;
        if (a < F(65535)) // don't allow accumulation buffer overflow
          {
          vtkUltraRound(a, *accPtrTmp);
          }
        }
      while (j);
      }
    else 
      {
      //------------------------------------
      // no accumulation buffer

      // loop over the eight voxels
      int j = 8;
      do
        {
        j--;
        if (fdx[j] == 0)
          {
          continue;
          }
        inPtrTmp = inPtr;
        outPtrTmp = outPtr+idx[j];
        // if alpha is nonzero then the pixel was hit before, so
        //  average with previous value
        if (outPtrTmp[numscalars])
          {
          f = fdx[j];
          r = 1 - f;
          int i = numscalars;
          do
            {
            i--;
            vtkUltraRound(f*(*inPtrTmp++) + r*(*outPtrTmp),
                            *outPtrTmp);
            outPtrTmp++;
            }
          while (i);
          }
        // alpha is zero, so just insert the new value
        else
          {
          int i = numscalars;
          do
            {
            i--;
            *outPtrTmp++ = *inPtrTmp++;
            }
          while (i);
          }          
        *outPtrTmp = 255;
        }
      while (j);
      }
    return 1;
    }
  return 0;
}                          

//----------------------------------------------------------------------------
// Some helper functions
//----------------------------------------------------------------------------

// get appropriate interpolation function
template <class F, class T>
static void vtkGetUltraInterpFunc(vtkFreehandUltrasound *self, 
                                    int (**interpolate)(F *point, 
                                                        T *inPtr, T *outPtr,
                                                        unsigned short *accPtr,
                                                        int numscalars, 
                                                        int outExt[6], 
                                                        int outInc[3]))
{
  switch (self->GetInterpolationMode())
    {
    case VTK_FREEHAND_NEAREST:
      *interpolate = &vtkNearestNeighborInterpolation;
      break;
    case VTK_FREEHAND_LINEAR:
      *interpolate = &vtkTrilinearInterpolation;
      break;
    }
}
  

//----------------------------------------------------------------------------
// This templated function executes the filter for any type of data.
// (this one function is pretty much the be-all and end-all of the
// filter)
template <class T>
static void vtkFreehandUltrasoundInsertSlice(vtkFreehandUltrasound *self,
                                             vtkImageData *outData, T *outPtr,
                                             unsigned short *accPtr,
                                             vtkImageData *inData, T *inPtr,
                                             int inExt[6], 
                                             vtkMatrix4x4 *matrix)
{
  int numscalars;
  int idX, idY, idZ;
  int inIncX, inIncY, inIncZ;
  int outExt[6], outInc[3], clipExt[6];
  vtkFloatingPointType inSpacing[3], inOrigin[3];
  unsigned long target;
  double outPoint[4], inPoint[4];

  int (*interpolate)(double *point, T *inPtr, T *outPtr,
		     unsigned short *accPtr,
                     int numscalars, int outExt[6], int outInc[3]);
  
  inData->GetSpacing(inSpacing);
  inData->GetOrigin(inOrigin);

  double xf = (self->GetFanOrigin()[0]-inOrigin[0])/inSpacing[0];
  double yf = (self->GetFanOrigin()[1]-inOrigin[1])/inSpacing[1];

  double d2 = self->GetFanDepth()*self->GetFanDepth();

  double xs = fabs((double)(inSpacing[0]));
  double ys = fabs((double)(inSpacing[1]));

  double ml = tan(self->GetFanAngles()[0]*vtkMath::DoubleDegreesToRadians())/
    xs*ys;
  double mr = tan(self->GetFanAngles()[1]*vtkMath::DoubleDegreesToRadians())/
    xs*ys;

  if (ml > mr)
    {
    double tmp = ml; ml = mr; mr = tmp;
    }

  self->GetClipExtent(clipExt, inOrigin, inSpacing, inExt);

  // find maximum output range
  outData->GetExtent(outExt);
  
  target = (unsigned long)
    ((inExt[5]-inExt[4]+1)*(inExt[3]-inExt[2]+1)/50.0);
  target++;
  
  // Get Increments to march through data 
  outData->GetIncrements(outInc);
  inData->GetContinuousIncrements(inExt, inIncX, inIncY, inIncZ);
  numscalars = inData->GetNumberOfScalarComponents();
  
  // Set interpolation method
  vtkGetUltraInterpFunc(self,&interpolate);

  // Loop through input pixels
  for (idZ = inExt[4]; idZ <= inExt[5]; idZ++)
    {
    for (idY = inExt[2]; idY <= inExt[3]; idY++)
      {
      for (idX = inExt[0]; idX <= inExt[1]; idX++)
        {
        if (idX >= clipExt[0] && idX <= clipExt[1] && 
	    idY >= clipExt[2] && idY <= clipExt[3])
          {
          double x = (idX-xf);
          double y = (idY-yf);
          if ((ml == 0 && mr == 0) || y > 0 &&
              ((x*x)*(xs*xs)+(y*y)*(ys*ys) < d2 && x/y >= ml && x/y <= mr))
            {  
            inPoint[0] = idX;
            inPoint[1] = idY;
            inPoint[2] = idZ;
            inPoint[3] = 1;

            matrix->MultiplyPoint(inPoint,outPoint); // apply transform
            
            outPoint[0] /= outPoint[3]; // deal with w if the transform
            outPoint[1] /= outPoint[3]; //   was a perspective transform
            outPoint[2] /= outPoint[3];
            outPoint[3] = 1;
        
            interpolate(outPoint, inPtr, outPtr, accPtr, numscalars, 
                        outExt, outInc);
            }
          }
        inPtr += numscalars; 
        }
      inPtr += inIncY;
      }
    inPtr += inIncZ;
    }
}

//----------------------------------------------------------------------------
// This method is passed a input and output region, and executes the filter
// algorithm to fill the output from the input.
// It just executes a switch statement to call the correct function for
// the regions data types.
void vtkFreehandUltrasound::InsertSlice()
{
  if (this->GetOptimization())
    {
    this->OptimizedInsertSlice();
    return;
    }

  if (this->ReconstructionThreadId == -1)
    {
    this->InternalExecuteInformation();
    }
  if (this->NeedsClear)
    {
    this->InternalClearOutput();
    }

  vtkImageData *inData = this->GetSlice();
  vtkImageData *outData = this->GetOutput();
  vtkImageData *accData = this->AccumulationBuffer;
  int *inExt = inData->GetWholeExtent();
  int *outExt = this->OutputExtent;

  // fprintf(stderr,"inExt %d %d %d %d %d %d\n",inExt[0],inExt[1],inExt[2],inExt[3],inExt[4],inExt[5]);

  inData->SetUpdateExtent(inExt);
  inData->Update();

  void *inPtr = inData->GetScalarPointerForExtent(inData->GetExtent());
  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  void *accPtr = NULL;
  
  if (this->Compounding)
    {
    accPtr = accData->GetScalarPointerForExtent(outExt);
    }
  else
    {
    accPtr = NULL;
    }

  vtkDebugMacro(<< "InsertSlice: inData = " << inData 
                << ", outData = " << outData);
  
  // this filter expects that input is the same type as output.
  if (inData->GetScalarType() != outData->GetScalarType())
    {
    vtkErrorMacro(<< "InsertSlice: input ScalarType, " 
                  << inData->GetScalarType()
                  << ", must match out ScalarType " 
                  << outData->GetScalarType());
    return;
    }

  // change transform matrix so that instead of taking 
  // input coords -> output coords it takes output indices -> input indices
  vtkMatrix4x4 *matrix = this->GetIndexMatrix();

  if (this->LastIndexMatrix && 
      this->CalculateMaxSliceSeparation(matrix,this->LastIndexMatrix) < 1.0)
    {
    return;
    }

  if (this->LastIndexMatrix == 0)
    {
    this->LastIndexMatrix = vtkMatrix4x4::New();
    }
  this->LastIndexMatrix->DeepCopy(matrix);
  
  switch (inData->GetScalarType())
    {
    case VTK_SHORT:
      vtkFreehandUltrasoundInsertSlice(this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), 
                             inData, (short *)(inPtr), 
                             inExt, matrix);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasoundInsertSlice(this,outData,(unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), 
                             inData, (unsigned short *)(inPtr), 
                             inExt, matrix);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasoundInsertSlice(this, outData,(unsigned char *)(outPtr),
                             (unsigned short *)(accPtr), 
                             inData, (unsigned char *)(inPtr), 
                             inExt, matrix);
      break;
    default:
      vtkErrorMacro(<< "InsertSlice: Unknown input ScalarType");
      return;
    }

  this->Modified();
}

//----------------------------------------------------------------------------
template <class T>
static void vtkFreehandUltrasoundFillHolesInOutput(vtkFreehandUltrasound *self,
						   vtkImageData *outData,
						   T *outPtr,
						   unsigned short *accPtr,
						   int outExt[6])
{
  int idX, idY, idZ;
  int incX, incY, incZ;
  int accIncX, accIncY, accIncZ;
  int startX, endX, numscalars;
  int c;

  // clip the extent by 1 voxel width relative to whole extent
  int *outWholeExt = outData->GetWholeExtent();
  int extent[6];
  for (int a = 0; a < 3; a++)
    {
    extent[2*a] = outExt[2*a];
    if (extent[2*a] == outWholeExt[2*a])
      {
      extent[2*a]++;
      }
    extent[2*a+1] = outExt[2*a+1];
    if (extent[2*a+1] == outWholeExt[2*a+1])
      {
      extent[2*a+1]--;
      }
    }

  // get increments for output and for accumulation buffer
  outData->GetIncrements(incX, incY, incZ);
  accIncX = 1;
  accIncY = incY/incX;
  accIncZ = incZ/incX;
  // number of components not including the alpha channel
  numscalars = outData->GetNumberOfScalarComponents() - 1;
   
  T *alphaPtr = outPtr + numscalars;
  T *outPtrZ, *outPtrY, *outPtrX;
  unsigned short *accPtrZ, *accPtrY, *accPtrX;

  // go through all voxels except the edge voxels
  for (idZ = extent[4]; idZ <= extent[5]; idZ++)
    {
    outPtrZ = outPtr + (idZ - outExt[4])*incZ;
    accPtrZ = accPtr + (idZ - outExt[4])*accIncZ;
    for (idY = extent[2]; idY <= extent[3]; idY++)
      {
      outPtrY = outPtrZ + (idY - outExt[2])*incY;
      accPtrY = accPtrZ + (idY - outExt[2])*accIncY;
      // find entry point
      alphaPtr = outPtrY + numscalars;
      for (startX = outExt[0]; startX <= outExt[1]; startX++)
	{
	// check the point on the row as well as the 4-connected voxels 
	if (*alphaPtr |
	    *(alphaPtr-incY) | *(alphaPtr+incY) |
	    *(alphaPtr-incZ) | *(alphaPtr+incZ))
	  {// break when alpha component is nonzero
	  break;
	  }
	alphaPtr += incX;
	}
      if (startX > outExt[1])
	{ // the whole row is empty, do nothing
	continue;
	}
      // find exit point
      alphaPtr = outPtrY + (outExt[1]-outExt[0])*incX + numscalars;
      for (endX = outExt[1]; endX >= outExt[0]; endX--)
	{
	// check the point on the row as well as the 4-connected voxels 
	if (*alphaPtr |
	    *(alphaPtr-incY) | *(alphaPtr+incY) |
	    *(alphaPtr-incZ) | *(alphaPtr+incZ))
	  {
	  break;
	  }
	alphaPtr -= incX;
	}
      // go through the row, skip first and last voxel in row
      if (startX == outWholeExt[0])
	{
	startX++;
	}
      if (endX == outWholeExt[1])
	{
	endX--;
	}
      outPtrX = outPtrY + (startX - outExt[0])*incX;
      accPtrX = accPtrY + (startX - outExt[0])*accIncX;
      for (idX = startX; idX <= endX; idX++)
	{
	if (outPtrX[numscalars] == 0)
	  { // only do this for voxels that haven't been hit
	  double sum[32];
	  for (c = 0; c < numscalars; c++) 
	    {
	    sum[c] = 0;
	    }
	  double asum = 0; 
	  int n = 0;
	  int nmin = 14; // half of the connected voxels plus one
	  T *blockPtr;
	  unsigned short *accBlockPtr;
	  // sum the pixel values for the 3x3x3 block
          //  (this is turned off for now)
	  if (0) // (accPtr)
	    { // use accumulation buffer to do weighted average
	    for (int k = -accIncZ; k <= accIncZ; k += accIncZ)
	      {
	      for (int j = -accIncY; j <= accIncY; j += accIncY)
		{
		for (int i = -accIncX; i <= accIncX; i += accIncX)
		  {
		  int inc = j + k + i;
		  blockPtr = outPtrX + inc*incX;
		  accBlockPtr = accPtrX + inc;
		  if (blockPtr[numscalars] == 255)
		    {
		    n++;
		    for (c = 0; c < numscalars; c++)
		      { // use accumulation buffer as weight
		      sum[c] += blockPtr[c]*(*accBlockPtr);
		      }
		    asum += *accBlockPtr;
		    }
		  }
		}
	      }
	    // if less than half the neighbors have data, use larger block
	    if (n <= nmin && idX != startX && idX != endX &&
		idX - outWholeExt[0] > 2 && outWholeExt[1] - idX > 2 &&
		idY - outWholeExt[2] > 2 && outWholeExt[3] - idY > 2 &&
		idZ - outWholeExt[4] > 2 && outWholeExt[5] - idZ > 2)
	      {
	      // weigh inner block by a factor of four (multiply three,
	      // plus we will be counting it again as part of the 5x5x5
	      // block)
	      asum *= 3;
	      for (c = 0; c < numscalars; c++) 
		{
		sum[c]*= 3;
		}	      
	      nmin = 63;
	      n = 0;
	      for (int k = -accIncZ*2; k <= accIncZ*2; k += accIncZ)
		{
		for (int j = -accIncY*2; j <= accIncY*2; j += accIncY)
		  {
		  for (int i = -accIncX*2; i <= accIncX*2; i += accIncX)
		    {
		    int inc = j + k + i;
		    blockPtr = outPtrX + inc*incX;
		    accBlockPtr = accPtrX + inc;
		    if (blockPtr[numscalars] == 255)
		      { // use accumulation buffer as weight
		      n++;
		      for (c = 0; c < numscalars; c++)
			{
			sum[c] += blockPtr[c]*(*accBlockPtr);
			}
		      asum += *accBlockPtr; 
		      }
		    }
		  }
		}
	      }
	    }
	  else // no accumulation buffer
	    {
	    for (int k = -incZ; k <= incZ; k += incZ)
	      {
	      for (int j = -incY; j <= incY; j += incY)
		{
		for (int i = -incX; i <= incX; i += incX)
		  {
		  blockPtr = outPtrX + j + k + i;
		  if (blockPtr[numscalars] == 255)
		    {
		    n++;
		    for (int c = 0; c < numscalars; c++)
		      {
		      sum[c] += blockPtr[c];
		      }
		    }
		  }
		}
	      }
	    asum = n;
	    // if less than half the neighbors have data, use larger block,
	    // and count inner 3x3 block again to weight it by 2
	    if (n <= nmin && idX != startX && idX != endX &&
		idX - outWholeExt[0] > 2 && outWholeExt[1] - idX > 2 &&
		idY - outWholeExt[2] > 2 && outWholeExt[3] - idY > 2 &&
		idZ - outWholeExt[4] > 2 && outWholeExt[5] - idZ > 2)
	      { 
	      // weigh inner block by a factor of four (multiply three,
	      // plus we will be counting it again as part of the 5x5x5
	      // block)
	      asum *= 3;
	      for (c = 0; c < numscalars; c++) 
		{
		sum[c]*= 3;
		}
	      nmin = 63;
	      n = 0;
	      for (int k = -incZ*2; k <= incZ*2; k += incZ)
		{
		for (int j = -incY*2; j <= incY*2; j += incY)
		  {
		  for (int i = -incX*2; i <= incX*2; i += incX)
		    {
		    blockPtr = outPtrX + j + k + i;
		    if (blockPtr[numscalars] == 255)
		      {
		      n++;
		      for (int c = 0; c < numscalars; c++)
			{
			sum[c] += blockPtr[c];
			}
		      }
		    }
		  }
		}
	      asum += n;
	      }
	    }
	  // if more than half of neighboring voxels are occupied, then fill
	  if (n >= nmin)
	    {
	    for (int c = 0; c < numscalars; c++)
	      {
	      vtkUltraRound(sum[c]/asum, outPtrX[c]);
	      }
	    // set alpha to 1 now, change to 255 later
	    outPtrX[numscalars] = 1;
	    }
	  }
	  outPtrX += incX;
	}
      }
    }

  // change alpha value '1' to value '255'
  alphaPtr = outPtr + numscalars;
  // go through all voxels this time
  for (idZ = outExt[4]; idZ <= outExt[5]; idZ++)
    {
    for (idY = outExt[2]; idY <= outExt[3]; idY++)
      {
      for (idX = outExt[0]; idX <= outExt[1]; idX++)
	{
	// convert '1' to 255
	if (*alphaPtr == 1)
	  {
	  *alphaPtr = 255;
	  }
	alphaPtr += incX;
	}
      // add the continuous increment
      alphaPtr += (incY - (outExt[1]-outExt[0]+1)*incX);
      }
    // add the continuous increment
    alphaPtr += (incZ - (outExt[3]-outExt[2]+1)*incY);
    }
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::ThreadedFillExecute(vtkImageData *outData,	
                                            	int outExt[6], int threadId)
{
  vtkImageData *accData = this->AccumulationBuffer;

  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  void *accPtr = NULL;
  
  if (this->Compounding)
    {
    accPtr = accData->GetScalarPointerForExtent(outExt);
    }

  switch (outData->GetScalarType())
    {
    case VTK_SHORT:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData, (unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData,(unsigned char *)(outPtr),
                             (unsigned short *)(accPtr), outExt); 
      break;
    default:
      vtkErrorMacro(<< "FillHolesInOutput: Unknown input ScalarType");
      return;
    }
}

//----------------------------------------------------------------------------
// this mess is really a simple function. All it does is call
// the ThreadedExecute method after setting the correct
// extent for this thread.
VTK_THREAD_RETURN_TYPE vtkFreehandThreadedFillExecute( void *arg )
{
  vtkFreehandThreadStruct *str;
  int ext[6], splitExt[6], total;
  int threadId, threadCount;
  vtkImageData *output;

  threadId = ((ThreadInfoStruct *)(arg))->ThreadID;
  threadCount = ((ThreadInfoStruct *)(arg))->NumberOfThreads;

  str = (vtkFreehandThreadStruct *)(((ThreadInfoStruct *)(arg))->UserData);
  output = str->Output;
  output->GetExtent( ext );

  // execute the actual method with appropriate extent
  // first find out how many pieces extent can be split into.
  total = str->Filter->SplitExtent(splitExt, ext, threadId, threadCount);
  //total = 1;
  
  if (threadId < total)
    {
    str->Filter->ThreadedFillExecute(str->Output, splitExt, threadId);
    }
  // else
  //   {
  //   otherwise don't use this thread. Sometimes the threads dont
  //   break up very well and it is just as efficient to leave a 
  //   few threads idle.
  //   }
  
  return VTK_THREAD_RETURN_VALUE;
}

void vtkFreehandUltrasound::MultiThreadFill(vtkImageData *outData)
{
  vtkFreehandThreadStruct str;
  
  str.Filter = this;
  str.Input = 0;
  str.Output = outData;
  
  this->Threader->SetNumberOfThreads(this->NumberOfThreads);
  
  // setup threading and the invoke threadedExecute
  this->Threader->SetSingleMethod(vtkFreehandThreadedFillExecute, &str);
  this->Threader->SingleMethodExecute();
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::FillHolesInOutput()
{
  this->InternalExecuteInformation();
  if (this->NeedsClear)
    {
    this->InternalClearOutput();
    }

  vtkImageData *outData = this->GetOutput();
  this->MultiThreadFill(outData);

  this->Modified(); 
}

//----------------------------------------------------------------------------

/*
void vtkFreehandUltrasound::FillHolesInOutput()
{
  this->InternalExecuteInformation();
  if (this->NeedsClear)
    {
    this->InternalClearOutput();
    }

  vtkImageData *outData = this->GetOutput();
  vtkImageData *accData = this->AccumulationBuffer;

  int *outExt = this->OutputExtent;

  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  void *accPtr = NULL;
  
  if (this->Compounding)
    {
    accPtr = accData->GetScalarPointerForExtent(outExt);
    }

  switch (outData->GetScalarType())
    {
    case VTK_SHORT:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData, (unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasoundFillHolesInOutput(
                             this, outData,(unsigned char *)(outPtr),
                             (unsigned short *)(accPtr), outExt); 
      break;
    default:
      vtkErrorMacro(<< "FillHolesInOutput: Unknown input ScalarType");
      return;
    }

  this->Modified(); 
}
*/

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::ClearOutput()
{
  this->NeedsClear = 1;

  if (this->ReconstructionThreadId == -1)
    {
    this->InternalClearOutput();
    }

  this->Modified();
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::InternalClearOutput()
{
  this->UpdateInformation();

  int *outExtent = this->OutputExtent;

  vtkImageData *outData = this->GetOutput();
  int numScalars = outData->GetNumberOfScalarComponents();

  outData->SetExtent(outExtent);
  outData->AllocateScalars();

  void *outPtr = outData->GetScalarPointerForExtent(outExtent);
  memset(outPtr,0,(outExtent[1]-outExtent[0]+1)*
                  (outExtent[3]-outExtent[2]+1)*
                  (outExtent[5]-outExtent[4]+1)*
                  numScalars*outData->GetScalarSize());

  if (this->Compounding)
    {
    this->AccumulationBuffer->SetExtent(outExtent);
    this->AccumulationBuffer->AllocateScalars();
    void *accPtr = this->AccumulationBuffer->\
      GetScalarPointerForExtent(outExtent);
    memset(accPtr,0,(outExtent[1]-outExtent[0]+1)*
                    (outExtent[3]-outExtent[2]+1)*
                    (outExtent[5]-outExtent[4]+1)*
                    this->AccumulationBuffer->GetScalarSize());
    }

  if (this->LastIndexMatrix)
    {
    this->LastIndexMatrix->Delete();
    this->LastIndexMatrix = NULL;
    }

  this->NeedsClear = 0;
}

//----------------------------------------------------------------------------
// check a matrix to see whether it is the identity matrix

static int vtkIsIdentityMatrix(vtkMatrix4x4 *matrix)
{
  static double identity[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
  int i,j;

  for (i = 0; i < 4; i++)
    {
    for (j = 0; j < 4; j++)
      {
      if (matrix->GetElement(i,j) != identity[4*i+j])
        {
        return 0;
        }
      }
    }
  return 1;
}

//----------------------------------------------------------------------------
// The transform matrix supplied by the user converts output coordinates
// to input coordinates.  
// To speed up the pixel lookup, the following function provides a
// matrix which converts output pixel indices to input pixel indices.

vtkMatrix4x4 *vtkFreehandUltrasound::GetIndexMatrix()
{
  // first verify that we have to update the matrix
  if (this->IndexMatrix == NULL)
    {
    this->IndexMatrix = vtkMatrix4x4::New();
    }

  vtkFloatingPointType inOrigin[3];
  vtkFloatingPointType inSpacing[3];
  vtkFloatingPointType outOrigin[3];
  vtkFloatingPointType outSpacing[3];

  this->GetSlice()->GetSpacing(inSpacing);
  this->GetSlice()->GetOrigin(inOrigin);
  this->GetOutput()->GetSpacing(outSpacing);
  this->GetOutput()->GetOrigin(outOrigin);  
  
  vtkTransform *transform = vtkTransform::New();
  vtkMatrix4x4 *inMatrix = vtkMatrix4x4::New();
  vtkMatrix4x4 *outMatrix = vtkMatrix4x4::New();

  if (this->SliceAxes)
    {
    transform->SetMatrix(this->GetSliceAxes());
    }
  if (this->SliceTransform)
    {
    transform->PostMultiply();
    transform->Concatenate(this->SliceTransform->GetMatrix());
    }
  
  // check to see if we have an identity matrix
  int isIdentity = vtkIsIdentityMatrix(transform->GetMatrix());

  // the outMatrix takes OutputData indices to OutputData coordinates,
  // the inMatrix takes InputData coordinates to InputData indices
  for (int i = 0; i < 3; i++) 
    {
    if (inSpacing[i] != outSpacing[i] || inOrigin[i] != outOrigin[i])
      {
      isIdentity = 0;
      }
    inMatrix->Element[i][i] = inSpacing[i];
    inMatrix->Element[i][3] = inOrigin[i];
    outMatrix->Element[i][i] = 1.0f/outSpacing[i];
    outMatrix->Element[i][3] = -outOrigin[i]/outSpacing[i];
    }

  if (!isIdentity)
    {
    transform->PostMultiply();
    transform->Concatenate(outMatrix);
    transform->PreMultiply();
    transform->Concatenate(inMatrix);
    }

  transform->GetMatrix(this->IndexMatrix);
  
  transform->Delete();
  inMatrix->Delete();
  outMatrix->Delete();

  return this->IndexMatrix;
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// The remainder of this file is the 'optimized' version of the code.
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// helper functions for vtkOptimizedExecute()

// find approximate intersection of line with the plane x = x_min,
// y = y_min, or z = z_min (lower limit of data extent) 

template<class F>
static inline
int intersectionHelper(F *point, F *axis, int *limit, int ai, int *inExt)
{
  F rd = limit[ai]*point[3]-point[ai]  + 0.5; 
    
  if (rd < inExt[0])
    { 
    return inExt[0];
    }
  else if (rd > inExt[1])
    {
    return inExt[1];
    }
  else
    {
    return int(rd);
    }
}

template <class F>
static int intersectionLow(F *point, F *axis, int *sign,
                           int *limit, int ai, int *inExt)
{
  // approximate value of r
  int r = intersectionHelper(point,axis,limit,ai,inExt);

  // move back and forth to find the point just inside the extent
  for (;;)
    {
    F p = point[ai]+r*axis[ai];

    if ((sign[ai] < 0 && r > inExt[0] ||
         sign[ai] > 0 && r < inExt[1]) && 
        vtkUltraRound(p) < limit[ai])
      {
      r += sign[ai];
      }
    else
      {
      break;
      }
    }

  for (;;)
    {
    F p = point[ai]+(r-sign[ai])*axis[ai];

    if ((sign[ai] > 0 && r > inExt[0] ||
         sign[ai] < 0 && r < inExt[1]) && 
        vtkUltraRound(p) >= limit[ai])
      {
      r -= sign[ai];
      }
    else
      {
      break;
      }
    }

  return r;
}

// same as above, but for x = x_max
template <class F>
static int intersectionHigh(F *point, F *axis, int *sign, 
                            int *limit, int ai, int *inExt)
{
  // approximate value of r
  int r = intersectionHelper(point,axis,limit,ai,inExt);
    
  // move back and forth to find the point just inside the extent
  for (;;)
    {
    F p = point[ai]+r*axis[ai];

    if ((sign[ai] > 0 && r > inExt[0] ||
         sign[ai] < 0 && r < inExt[1]) &&
        vtkUltraRound(p) > limit[ai])
      {
      r -= sign[ai];
      }
    else
      {
      break;
      }
    }

  for (;;)
    {
    F p = point[ai]+(r+sign[ai])*axis[ai];

    if ((sign[ai] < 0 && r > inExt[0] ||
         sign[ai] > 0 && r < inExt[1]) && 
        vtkUltraRound(p) <= limit[ai])
      {
      r += sign[ai];
      }
    else
      {
      break;
      }
    }

  return r;
}

template <class F>
static int isBounded(F *point, F *xAxis, int *inMin, 
                     int *inMax, int ai, int r)
{
  int bi = ai+1; 
  int ci = ai+2;
  if (bi > 2) 
    { 
    bi -= 3; // coordinate index must be 0, 1 or 2 
    } 
  if (ci > 2)
    { 
    ci -= 3;
    }

  F fbp = point[bi]+r*xAxis[bi];
  F fcp = point[ci]+r*xAxis[ci];

  int bp = vtkUltraRound(fbp);
  int cp = vtkUltraRound(fcp);
  
  return (bp >= inMin[bi] && bp <= inMax[bi] &&
          cp >= inMin[ci] && cp <= inMax[ci]);
}

// this huge mess finds out where the current output raster
// line intersects the input volume

static void vtkUltraFindExtentHelper(int &r1, int &r2, int sign, int *inExt)
{
  if (sign < 0)
    {
    int i = r1;
    r1 = r2;
    r2 = i;
    }
  
  // bound r1,r2 within reasonable limits
  if (r1 < inExt[0]) 
    {
    r1 = inExt[0];
    }
  if (r2 > inExt[1]) 
    {
    r2 = inExt[1];
    }
  if (r1 > r2) 
    {
    r1 = inExt[0];
    r2 = inExt[0]-1;
    }
}  

template <class F>
static void vtkUltraFindExtent(int& r1, int& r2, F *point, F *xAxis, 
                                 int *inMin, int *inMax, int *inExt)
{
  int i, ix, iy, iz;
  int sign[3];
  int indx1[4],indx2[4];
  F p1,p2;

  // find signs of components of x axis 
  // (this is complicated due to the homogeneous coordinate)
  for (i = 0; i < 3; i++)
    {
    p1 = point[i];

    p2 = point[i]+xAxis[i];

    if (p1 <= p2)
      {
      sign[i] = 1;
      }
    else 
      {
      sign[i] = -1;
      }
    } 
  
  // order components of xAxis from largest to smallest
  
  ix = 0;
  for (i = 1; i < 3; i++)
    {
    if (((xAxis[i] < 0) ? (-xAxis[i]) : (xAxis[i])) >
        ((xAxis[ix] < 0) ? (-xAxis[ix]) : (xAxis[ix])))
      {
      ix = i;
      }
    }
  
  iy = ((ix > 1) ? ix-2 : ix+1);
  iz = ((ix > 0) ? ix-1 : ix+2);

  if (((xAxis[iy] < 0) ? (-xAxis[iy]) : (xAxis[iy])) >
      ((xAxis[iz] < 0) ? (-xAxis[iz]) : (xAxis[iz])))
    {
    i = iy;
    iy = iz;
    iz = i;
    }

  r1 = intersectionLow(point,xAxis,sign,inMin,ix,inExt);
  r2 = intersectionHigh(point,xAxis,sign,inMax,ix,inExt);
  
  // find points of intersections
  // first, find w-value for perspective (will usually be 1)
  for (i = 0; i < 3; i++)
    {
    p1 = point[i]+r1*xAxis[i];
    p2 = point[i]+r2*xAxis[i];

    indx1[i] = vtkUltraRound(p1);
    indx2[i] = vtkUltraRound(p2);
    }
  if (isBounded(point,xAxis,inMin,inMax,ix,r1))
    { // passed through x face, check opposing face
    if (isBounded(point,xAxis,inMin,inMax,ix,r2))
      {
      vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
      return;
      }
    
    if (indx2[iy] < inMin[iy])
      { // check y face
      r2 = intersectionLow(point,xAxis,sign,inMin,iy,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iy,r2))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    else if (indx2[iy] > inMax[iy])
      { // check other y face
      r2 = intersectionHigh(point,xAxis,sign,inMax,iy,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iy,r2))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    
    if (indx2[iz] < inMin[iz])
      { // check z face
      r2 = intersectionLow(point,xAxis,sign,inMin,iz,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iz,r2))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    else if (indx2[iz] > inMax[iz])
      { // check other z face
      r2 = intersectionHigh(point,xAxis,sign,inMax,iz,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iz,r2))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    }
  
  if (isBounded(point,xAxis,inMin,inMax,ix,r2))
    { // passed through the opposite x face
    if (indx1[iy] < inMin[iy])
      { // check y face
      r1 = intersectionLow(point,xAxis,sign,inMin,iy,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iy,r1))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    else if (indx1[iy] > inMax[iy])
      { // check other y face
      r1 = intersectionHigh(point,xAxis,sign,inMax,iy,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iy,r1))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    
    if (indx1[iz] < inMin[iz])
      { // check z face
      r1 = intersectionLow(point,xAxis,sign,inMin,iz,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iz,r1))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    else if (indx1[iz] > inMax[iz])
      { // check other z face
      r1 = intersectionHigh(point,xAxis,sign,inMax,iz,inExt);
      if (isBounded(point,xAxis,inMin,inMax,iz,r1))
        {
        vtkUltraFindExtentHelper(r1,r2,sign[ix],inExt);
        return;
        }
      }
    }
  
  if ((indx1[iy] >= inMin[iy] && indx2[iy] < inMin[iy]) ||
      (indx1[iy] < inMin[iy] && indx2[iy] >= inMin[iy]))
    { // line might pass through bottom face
    r1 = intersectionLow(point,xAxis,sign,inMin,iy,inExt);
    if (isBounded(point,xAxis,inMin,inMax,iy,r1))
      {
      if ((indx1[iy] <= inMax[iy] && indx2[iy] > inMax[iy]) ||
          (indx1[iy] > inMax[iy] && indx2[iy] <= inMax[iy]))
        { // line might pass through top face
        r2 = intersectionHigh(point,xAxis,sign,inMax,iy,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iy,r2))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iy],inExt);
          return;
          }
        }
      
      if (indx1[iz] < inMin[iz] && indx2[iy] < inMin[iy] ||
          indx2[iz] < inMin[iz] && indx1[iy] < inMin[iy])
        { // line might pass through in-to-screen face
        r2 = intersectionLow(point,xAxis,sign,inMin,iz,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iz,r2))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iy],inExt);
          return;
          }
        }
      else if (indx1[iz] > inMax[iz] && indx2[iy] < inMin[iy] ||
               indx2[iz] > inMax[iz] && indx1[iy] < inMin[iy])
        { // line might pass through out-of-screen face
        r2 = intersectionHigh(point,xAxis,sign,inMax,iz,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iz,r2))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iy],inExt);
          return;
          }
        } 
      }
    }
  
  if ((indx1[iy] <= inMax[iy] && indx2[iy] > inMax[iy]) ||
      (indx1[iy] > inMax[iy] && indx2[iy] <= inMax[iy]))
    { // line might pass through top face
    r2 = intersectionHigh(point,xAxis,sign,inMax,iy,inExt);
    if (isBounded(point,xAxis,inMin,inMax,iy,r2))
      {
      if (indx1[iz] < inMin[iz] && indx2[iy] > inMax[iy] ||
          indx2[iz] < inMin[iz] && indx1[iy] > inMax[iy])
        { // line might pass through in-to-screen face
        r1 = intersectionLow(point,xAxis,sign,inMin,iz,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iz,r1))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iy],inExt);
          return;
          }
        }
      else if (indx1[iz] > inMax[iz] && indx2[iy] > inMax[iy] || 
               indx2[iz] > inMax[iz] && indx1[iy] > inMax[iy])
        { // line might pass through out-of-screen face
        r1 = intersectionHigh(point,xAxis,sign,inMax,iz,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iz,r1))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iy],inExt);
          return;
          }
        }
      } 
    }
  
  if ((indx1[iz] >= inMin[iz] && indx2[iz] < inMin[iz]) ||
      (indx1[iz] < inMin[iz] && indx2[iz] >= inMin[iz]))
    { // line might pass through in-to-screen face
    r1 = intersectionLow(point,xAxis,sign,inMin,iz,inExt);
    if (isBounded(point,xAxis,inMin,inMax,iz,r1))
      {
      if (indx1[iz] > inMax[iz] || indx2[iz] > inMax[iz])
        { // line might pass through out-of-screen face
        r2 = intersectionHigh(point,xAxis,sign,inMax,iz,inExt);
        if (isBounded(point,xAxis,inMin,inMax,iz,r2))
          {
          vtkUltraFindExtentHelper(r1,r2,sign[iz],inExt);
          return;
          }
        }
      }
    }
  
  r1 = inExt[0];
  r2 = inExt[0] - 1;
}

// The vtkOptimizedExecute() function uses an optimization which
// is conceptually simple, but complicated to implement.

// In the un-optimized version, each output voxel
// is converted into a set of look-up indices for the input data;
// then, the indices are checked to ensure they lie within the
// input data extent.

// In the optimized version below, the check is done in reverse:
// it is first determined which output voxels map to look-up indices
// within the input data extent.  Then, further calculations are
// done only for those voxels.  This means that 1) minimal work
// is done for voxels which map to regions outside fo the input
// extent (they are just set to the background color) and 2)
// the inner loops of the look-up and interpolation are
// tightened relative to the un-uptimized version. 

template<class T>
static inline void vtkFreehandOptimizedNNHelper(int r1, int r2,
                                                double *outPoint,
                                                double *outPoint1,
						double *xAxis,
                                                T *&inPtr, T *outPtr,
                                                int *outExt, int *outInc,
                                                int numscalars, 
                                                unsigned short *accPtr)
{
  if (accPtr)  // Nearest-Neighbor, no extent checks
    {
    for (int idX = r1; idX <= r2; idX++)
      {
      outPoint[0] = outPoint1[0] + idX*xAxis[0]; 
      outPoint[1] = outPoint1[1] + idX*xAxis[1];
      outPoint[2] = outPoint1[2] + idX*xAxis[2];

      int outIdX = vtkUltraRound(outPoint[0]) - outExt[0];
      int outIdY = vtkUltraRound(outPoint[1]) - outExt[2];
      int outIdZ = vtkUltraRound(outPoint[2]) - outExt[4];

      /* bounds checking turned off to improve performance
      if (outIdX < 0 || outIdX > outExt[1] - outExt[0] ||
          outIdY < 0 || outIdY > outExt[3] - outExt[2] ||
          outIdZ < 0 || outIdZ > outExt[5] - outExt[4])
        {
        cerr << "out of bounds!!!\n";
        inPtr += numscalars;
        return;
        }
      */
      int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc;
      unsigned short *accPtr1 = accPtr + inc/outInc[0];
      unsigned short newa = *accPtr1 + 255;
      int i = numscalars;
      do 
        {
        i--;
        *outPtr1 = ((*inPtr++)*255 + (*outPtr1)*(*accPtr1))/newa;
        outPtr1++;
        }
      while (i);
      *outPtr1 = 255;
      *accPtr1 = 65535;
      if (newa < 65535)
        {
        *accPtr1 = newa;
        }
      }
    }
  else
    {  // Nearest-Neighbor, no extent checks, no accumulation
    for (int idX = r1; idX <= r2; idX++)
      {
      outPoint[0] = outPoint1[0] + idX*xAxis[0]; 
      outPoint[1] = outPoint1[1] + idX*xAxis[1];
      outPoint[2] = outPoint1[2] + idX*xAxis[2];

      int outIdX = vtkUltraRound(outPoint[0]) - outExt[0];
      int outIdY = vtkUltraRound(outPoint[1]) - outExt[2];
      int outIdZ = vtkUltraRound(outPoint[2]) - outExt[4];

      /* bounds checking turned off to improve performance
      if (outIdX < 0 || outIdX > outExt[1] - outExt[0] ||
          outIdY < 0 || outIdY > outExt[3] - outExt[2] ||
          outIdZ < 0 || outIdZ > outExt[5] - outExt[4])
        {
        cerr << "out of bounds!!!\n";
        inPtr += numscalars;
        return;
        }
      */

      int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc;
      int i = numscalars;
      do
        {
        i--;
        *outPtr1++ = *inPtr++;
        }
      while (i);
      *outPtr1 = 255;
      }
    } 
}

// specifically optimized for fixed-point (i.e. integer) mathematics
template <class T>
static inline void vtkFreehandOptimizedNNHelper(int r1, int r2,
                                                fixed *outPoint,
                                                fixed *outPoint1, fixed *xAxis,
                                                T *&inPtr, T *outPtr,
                                                int *outExt, int *outInc,
                                                int numscalars, 
                                                unsigned short *accPtr)
{
  outPoint[0] = outPoint1[0] + r1*xAxis[0] - outExt[0]; 
  outPoint[1] = outPoint1[1] + r1*xAxis[1] - outExt[2];
  outPoint[2] = outPoint1[2] + r1*xAxis[2] - outExt[4];

  if (accPtr)  // Nearest-Neighbor, no extent checks
    {
    for (int idX = r1; idX <= r2; idX++)
      {
      int outIdX = vtkUltraRound(outPoint[0]);
      int outIdY = vtkUltraRound(outPoint[1]);
      int outIdZ = vtkUltraRound(outPoint[2]);

      /* bounds checking turned off to improve performance
      if (outIdX < 0 || outIdX > outExt[1] - outExt[0] ||
          outIdY < 0 || outIdY > outExt[3] - outExt[2] ||
          outIdZ < 0 || outIdZ > outExt[5] - outExt[4])
        {
        cerr << "out of bounds!!!\n";
        inPtr += numscalars;
        return;
        }
      */
      int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc;
      unsigned short *accPtr1 = accPtr + inc/outInc[0];
      unsigned short newa = *accPtr1 + 255;
      int i = numscalars;
      do 
        {
        i--;
        *outPtr1 = ((*inPtr++)*255 + (*outPtr1)*(*accPtr1))/newa;
        outPtr1++;
        }
      while (i);
      *outPtr1 = 255;
      *accPtr1 = 65535;
      if (newa < 65535)
        {
        *accPtr1 = newa;
        }

      outPoint[0] += xAxis[0];
      outPoint[1] += xAxis[1];
      outPoint[2] += xAxis[2];
      }
    }
  else
    {  // Nearest-Neighbor, no extent checks, no accumulation
    for (int idX = r1; idX <= r2; idX++)
      {
      int outIdX = vtkUltraRound(outPoint[0]);
      int outIdY = vtkUltraRound(outPoint[1]);
      int outIdZ = vtkUltraRound(outPoint[2]);

      /* bounds checking turned off to improve performance
      if (outIdX < 0 || outIdX > outExt[1] - outExt[0] ||
          outIdY < 0 || outIdY > outExt[3] - outExt[2] ||
          outIdZ < 0 || outIdZ > outExt[5] - outExt[4])
        {
        cerr << "out of bounds!!!\n";
        inPtr += numscalars;
        return;
        }
      */
      int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc;
      int i = numscalars;
      do
        {
        i--;
        *outPtr1++ = *inPtr++;
        }
      while (i);
      *outPtr1 = 255;

      outPoint[0] += xAxis[0];
      outPoint[1] += xAxis[1];
      outPoint[2] += xAxis[2];
      }
    } 
}

template <class F, class T>
static void vtkOptimizedInsertSlice(vtkFreehandUltrasound *self,
                                    vtkImageData *outData, T *outPtr,
                                    unsigned short *accPtr,
                                    vtkImageData *inData, T *inPtr,
                                    int inExt[6],
                                    F matrix[4][4])
{
  int id = 0;
  int i, numscalars;
  int idX, idY, idZ;
  int inIncX, inIncY, inIncZ;
  int outExt[6];
  int outMax[3], outMin[3];
  int outInc[3];
  int clipExt[6];
  unsigned long count = 0;
  unsigned long target;
  int r1,r2;
  F outPoint0[3];
  F outPoint1[3];
  F outPoint[3];
  F xAxis[3], yAxis[3], zAxis[3], origin[3];
  vtkFloatingPointType inSpacing[3],inOrigin[3];

  inData->GetSpacing(inSpacing);
  inData->GetOrigin(inOrigin);

  double xf = (self->GetFanOrigin()[0]-inOrigin[0])/inSpacing[0];
  double yf = (self->GetFanOrigin()[1]-inOrigin[1])/inSpacing[1];

  double d2 = self->GetFanDepth()*self->GetFanDepth();

  double xs = inSpacing[0];
  double ys = inSpacing[1];

  double ml = tan(self->GetFanAngles()[0]*vtkMath::DoubleDegreesToRadians())/
    xs*ys;
  double mr = tan(self->GetFanAngles()[1]*vtkMath::DoubleDegreesToRadians())/
    xs*ys;

  if (ml > mr)
    {
    double tmp = ml; ml = mr; mr = tmp;
    }

  self->GetClipExtent(clipExt, inOrigin, inSpacing, inExt);

  // find maximum output range
  outData->GetExtent(outExt);
  for (i = 0; i < 3; i++)
    {
    outMin[i] = outExt[2*i];
    outMax[i] = outExt[2*i+1];
    }
  
  target = (unsigned long)
    ((inExt[5]-inExt[4]+1)*(inExt[3]-inExt[2]+1)/50.0);
  target++;
  
  // Get Increments to march through data 
  outData->GetIncrements(outInc);
  inData->GetContinuousIncrements(inExt, inIncX, inIncY, inIncZ);
  numscalars = inData->GetNumberOfScalarComponents();
  
  // break matrix into a set of axes plus an origin
  // (this allows us to calculate the transform Incrementally)
  for (i = 0; i < 3; i++)
    {
    xAxis[i]  = matrix[i][0];
    yAxis[i]  = matrix[i][1];
    zAxis[i]  = matrix[i][2];
    origin[i] = matrix[i][3];
    }

  // Loop through input pixels
  for (idZ = inExt[4]; idZ <= inExt[5]; idZ++)
    {
    outPoint0[0] = origin[0]+idZ*zAxis[0]; // incremental transform
    outPoint0[1] = origin[1]+idZ*zAxis[1];
    outPoint0[2] = origin[2]+idZ*zAxis[2];
    
    for (idY = inExt[2]; idY <= inExt[3]; idY++)
      {
      outPoint1[0] = outPoint0[0]+idY*yAxis[0]; // incremental transform
      outPoint1[1] = outPoint0[1]+idY*yAxis[1];
      outPoint1[2] = outPoint0[2]+idY*yAxis[2];
      
      if (!id) 
        {
        if (!(count%target)) 
          {
          self->UpdateProgress(count/(50.0*target));
          }
        count++;
        }
      
      // find intersections of x raster line with the output extent
      vtkUltraFindExtent(r1,r2,outPoint1,xAxis,outMin,outMax,inExt);

      // next, handle the 'fan' shape of the input
      double y = (yf - idY);
      if (ys < 0)
        {
        y = -y;
        }
      if (!(ml == 0 && mr == 0))
        {
        // first, check the angle range of the fan
	if (r1 < -vtkUltraFloor(-(ml*y + xf + 1)))
          {
          r1 = -vtkUltraFloor(-(ml*y + xf + 1));
          }
	  if (r2 > vtkUltraFloor(mr*y + xf - 1))
          {
	  r2 = vtkUltraFloor(mr*y + xf - 1);
          }
        
        // next, check the radius of the fan
        double dx = (d2 - (y*y)*(ys*ys))/(xs*xs);
        if (dx < 0)
          {
          r1 = inExt[0];
          r2 = inExt[0]-1;
          }
        else
          {
          dx = sqrt(dx);
          if (r1 < -vtkUltraFloor(-(xf - dx + 1)))
            {
	    r1 = -vtkUltraFloor(-(xf - dx + 1));
            }
          if (r2 > vtkUltraFloor(xf + dx - 1))
            {
	    r2 = vtkUltraFloor(xf + dx - 1);
            }
          }
        }

      // bound to the ultrasound clip rectangle
      if (r1 < clipExt[0])
        {
        r1 = clipExt[0];
        }
      if (r2 > clipExt[1])
        {
        r2 = clipExt[1];
        }
      if (r1 > r2 || idY < clipExt[2] || idY > clipExt[3]) 
        {
        r1 = inExt[0];
        r2 = inExt[0]-1;
        }

      // skip the portion of the slice that we don't want to include
      for (idX = inExt[0]; idX < r1; idX++)
        {
        inPtr += numscalars;
        }

      if (self->GetInterpolationMode() == VTK_FREEHAND_LINEAR)
        { 
        for (idX = r1; idX <= r2; idX++)
          {
          outPoint[0] = outPoint1[0] + idX*xAxis[0];
          outPoint[1] = outPoint1[1] + idX*xAxis[1];
          outPoint[2] = outPoint1[2] + idX*xAxis[2];
          
          vtkTrilinearInterpolation(outPoint, inPtr, outPtr, accPtr, 
                                    numscalars, outExt, outInc);

          inPtr += numscalars;
          }
        }
      else 
        {
        vtkFreehandOptimizedNNHelper(r1, r2, outPoint, outPoint1, xAxis, 
                                     inPtr, outPtr, outExt, outInc,
                                     numscalars, accPtr);
        }
  
      // skip the portion of the slice we don't want to reconstruct
      for (idX = r2+1; idX <= inExt[1]; idX++)
        {
        inPtr += numscalars;
        }

      inPtr += inIncY;
      }
    inPtr += inIncZ;
    }
}


// this mess is really a simple function. All it does is call
// the ThreadedExecute method after setting the correct
// extent for this thread. Its just a pain to calculate
// the correct extent.
VTK_THREAD_RETURN_TYPE vtkFreehandThreadedExecute( void *arg )
{
  vtkFreehandThreadStruct *str;
  int ext[6], splitExt[6], total;
  int threadId, threadCount;
  vtkImageData *input;

  threadId = ((ThreadInfoStruct *)(arg))->ThreadID;
  threadCount = ((ThreadInfoStruct *)(arg))->NumberOfThreads;

  str = (vtkFreehandThreadStruct *)(((ThreadInfoStruct *)(arg))->UserData);
  input = str->Input;
  input->GetUpdateExtent( ext );

  // execute the actual method with appropriate extent
  // first find out how many pieces extent can be split into.
  total = str->Filter->SplitExtent(splitExt, ext, threadId, threadCount);
  //total = 1;
  
  if (threadId < total)
    {
    str->Filter->ThreadedExecute(str->Input, str->Output, splitExt, threadId);
    }
  // else
  //   {
  //   otherwise don't use this thread. Sometimes the threads dont
  //   break up very well and it is just as efficient to leave a 
  //   few threads idle.
  //   }
  
  return VTK_THREAD_RETURN_VALUE;
}

//----------------------------------------------------------------------------
// For streaming and threads.  Splits output update extent into num pieces.
// This method needs to be called num times.  Results must not overlap for
// consistent starting extent.  Subclass can override this method.
// This method returns the number of peices resulting from a successful split.
// This can be from 1 to "total".  
// If 1 is returned, the extent cannot be split.
int vtkFreehandUltrasound::SplitExtent(int splitExt[6], int startExt[6], 
                                       int num, int total)
{
  int splitAxis;
  int min, max;

  vtkDebugMacro("SplitExtent: ( " << startExt[0] << ", " << startExt[1] << ", "
                << startExt[2] << ", " << startExt[3] << ", "
                << startExt[4] << ", " << startExt[5] << "), " 
                << num << " of " << total);

  // start with same extent
  memcpy(splitExt, startExt, 6 * sizeof(int));

  splitAxis = 2;
  min = startExt[4];
  max = startExt[5];
  while (min == max)
    {
    --splitAxis;
    if (splitAxis < 0)
      { // cannot split
      vtkDebugMacro("  Cannot Split");
      return 1;
      }
    min = startExt[splitAxis*2];
    max = startExt[splitAxis*2+1];
    }

  // determine the actual number of pieces that will be generated
  int range = max - min + 1;
  int valuesPerThread = (int)ceil(range/(double)total);
  int maxThreadIdUsed = (int)ceil(range/(double)valuesPerThread) - 1;
  if (num < maxThreadIdUsed)
    {
    splitExt[splitAxis*2] = splitExt[splitAxis*2] + num*valuesPerThread;
    splitExt[splitAxis*2+1] = splitExt[splitAxis*2] + valuesPerThread - 1;
    }
  if (num == maxThreadIdUsed)
    {
    splitExt[splitAxis*2] = splitExt[splitAxis*2] + num*valuesPerThread;
    }
  /*
  vtkDebugMacro("  Split Piece: ( " <<splitExt[0]<< ", " <<splitExt[1]<< ", "
                << splitExt[2] << ", " << splitExt[3] << ", "
                << splitExt[4] << ", " << splitExt[5] << ")");
  */

  return maxThreadIdUsed + 1;
}

void vtkFreehandUltrasound::MultiThread(vtkImageData *inData,
                                        vtkImageData *outData)
{
  vtkFreehandThreadStruct str;
  
  str.Filter = this;
  str.Input = inData;
  str.Output = outData;
  
  this->Threader->SetNumberOfThreads(this->NumberOfThreads);
  
  // setup threading and the invoke threadedExecute
  this->Threader->SetSingleMethod(vtkFreehandThreadedExecute, &str);
  this->Threader->SingleMethodExecute();
}


void vtkFreehandUltrasound::OptimizedInsertSlice()
{
  if (this->ReconstructionThreadId == -1)
    {
    this->InternalExecuteInformation();
    }
  if (this->NeedsClear)
    {
    this->InternalClearOutput();
    }

  vtkImageData *inData = this->GetSlice();
  vtkImageData *outData = this->GetOutput();

  // if not in ReconstructionThread, update the slice here
  // (otherwise, the slice is updated in vtkReconstructionThread
  // to ensure synchronization with the tracking)
  if (this->ReconstructionThreadId == -1) {
    int clipExt[6];
    this->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),
			inData->GetWholeExtent());
    inData->SetUpdateExtent(clipExt);
    inData->Update();
  }

  this->MultiThread(inData, outData);

  this->Modified();
}

//----------------------------------------------------------------------------
// This method is passed a input and output region, and executes the filter
// algorithm to fill the output from the input.
// It just executes a switch statement to call the correct function for
// the regions data types.
void vtkFreehandUltrasound::ThreadedExecute(vtkImageData *inData, 
                                            vtkImageData *outData,
                                            int inExt[6], int threadId)
{
  // fprintf(stderr,"inExt %d %d %d %d %d %d\n",inExt[0],inExt[1],inExt[2],inExt[3],inExt[4],inExt[5]);

  void *inPtr = inData->GetScalarPointerForExtent(inExt);
  int *outExt = this->OutputExtent;
  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  void *accPtr = NULL;

  vtkImageData *accData = this->AccumulationBuffer;
  if (this->Compounding)
    {
    accPtr = accData->GetScalarPointerForExtent(outExt);
    }
  else
    {
    accPtr = NULL;
    }

  vtkDebugMacro(<< "OptimizedInsertSlice: inData = " << inData 
  << ", outData = " << outData);
  
  // this filter expects that input is the same type as output.
  if (inData->GetScalarType() != outData->GetScalarType())
    {
    vtkErrorMacro(<< "OptimizedInsertSlice: input ScalarType, " 
            << inData->GetScalarType()
            << ", must match out ScalarType " << outData->GetScalarType());
    return;
    }

  // use fixed-point math for optimization level 2
  if (this->GetOptimization() == 2)
    {
    // change transform matrix so that instead of taking 
    // input coords -> output coords it takes output indices -> input indices
    vtkMatrix4x4 *matrix = this->GetIndexMatrix();
    fixed newmatrix[4][4];
    for (int i = 0; i < 4; i++)
      {
      newmatrix[i][0] = matrix->GetElement(i,0);
      newmatrix[i][1] = matrix->GetElement(i,1);
      newmatrix[i][2] = matrix->GetElement(i,2);
      newmatrix[i][3] = matrix->GetElement(i,3);
      }

    //if (this->LastIndexMatrix && 
    //this->CalculateMaxSliceSeparation(matrix,this->LastIndexMatrix) < 1.0)
    //{
    //return;
    //}

    //if (this->LastIndexMatrix == 0)
    //  {
    //  this->LastIndexMatrix = vtkMatrix4x4::New();
    //  }
    //this->LastIndexMatrix->DeepCopy(matrix);
  
    switch (inData->GetScalarType())
      {
      case VTK_SHORT:
        vtkOptimizedInsertSlice(this, outData, (short *)(outPtr), 
                                (unsigned short *)(accPtr), 
                                inData, (short *)(inPtr), 
                                inExt, newmatrix);
        break;
      case VTK_UNSIGNED_SHORT:
        vtkOptimizedInsertSlice(this,outData,(unsigned short *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned short *)(inPtr), 
                                inExt, newmatrix);
        break;
      case VTK_UNSIGNED_CHAR:
        vtkOptimizedInsertSlice(this, outData,(unsigned char *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned char *)(inPtr), 
                                inExt, newmatrix);
        break;
      default:
        vtkErrorMacro(<< "OptimizedInsertSlice: Unknown input ScalarType");
        return;
      }
    }
  else
    {
    // change transform matrix so that instead of taking 
    // input coords -> output coords it takes output indices -> input indices
    vtkMatrix4x4 *matrix = this->GetIndexMatrix();
    double newmatrix[4][4];
    for (int i = 0; i < 4; i++)
      {
      newmatrix[i][0] = matrix->GetElement(i,0);
      newmatrix[i][1] = matrix->GetElement(i,1);
      newmatrix[i][2] = matrix->GetElement(i,2);
      newmatrix[i][3] = matrix->GetElement(i,3);
      }

    //if (this->LastIndexMatrix && 
    //this->CalculateMaxSliceSeparation(matrix,this->LastIndexMatrix) < 1.0)
    //{
    //return;
    //}

    //if (this->LastIndexMatrix == 0)
    //  {
    //  this->LastIndexMatrix = vtkMatrix4x4::New();
    //  }
    //this->LastIndexMatrix->DeepCopy(matrix);
  
    switch (inData->GetScalarType())
      {
      case VTK_SHORT:
        vtkOptimizedInsertSlice(this, outData, (short *)(outPtr), 
                                (unsigned short *)(accPtr), 
                                inData, (short *)(inPtr), 
                                inExt, newmatrix);
        break;
      case VTK_UNSIGNED_SHORT:
        vtkOptimizedInsertSlice(this,outData,(unsigned short *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned short *)(inPtr), 
                                inExt, newmatrix);
        break;
      case VTK_UNSIGNED_CHAR:
        vtkOptimizedInsertSlice(this, outData,(unsigned char *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned char *)(inPtr), 
                                inExt, newmatrix);
        break;
      default:
        vtkErrorMacro(<< "OptimizedInsertSlice: Unknown input ScalarType");
        return;
      }
    }
}

//----------------------------------------------------------------------------
// platform-independent sleep function
static inline void vtkSleep(double duration)
{
  duration = duration; // avoid warnings
  // sleep according to OS preference
#ifdef _WIN32
  Sleep(vtkUltraFloor(1000*duration));
#elif defined(__FreeBSD__) || defined(__linux__) || defined(sgi)
  struct timespec sleep_time, remaining_time;
  int seconds = vtkUltraFloor(duration);
  int nanoseconds = vtkUltraFloor(1000000000*(duration - seconds));
  sleep_time.tv_sec = seconds;
  sleep_time.tv_nsec = nanoseconds;
  nanosleep(&sleep_time, &remaining_time);
#endif
}

//----------------------------------------------------------------------------
// Sleep until the specified absolute time has arrived.
// You must pass a handle to the current thread.  
// If '0' is returned, then the thread was aborted before or during the wait.
static int vtkThreadSleep(struct ThreadInfoStruct *data, double time)
{
  //fprintf(stderr,"sleep = %f\n",time-vtkTimerLog::GetCurrentTime());
  for (;;)
    {
    // slice 10 millisecs off the time, since this is how long it will
    // take for this thread to start executing once it has been
    // re-scheduled
    double remaining = time - vtkTimerLog::GetCurrentTime() - 0.01;

    // check to see if we have reached the specified time
    if (remaining <= 0)
      {
	//fprintf(stderr,"remaining = %f\n",remaining+0.01);
      return 1;
      }
    // check the ActiveFlag at least every 0.1 seconds
    if (remaining > 0.1)
      {
      remaining = 0.1;
      }

    // check to see if we are being told to quit 
    if (*(data->ActiveFlag) == 0)
      {
      return 0;
      }
    
    vtkSleep(remaining);
    }

  return 1;
}

//----------------------------------------------------------------------------
// This function is run in a background thread to perform the reconstruction.
// By running it in the background, it doesn't interfere with the display
// of the partially reconstructed volume.
static void *vtkReconstructionThread(struct ThreadInfoStruct *data)
{
  vtkFreehandUltrasound *self = (vtkFreehandUltrasound *)(data->UserData);

  double prevtimes[10];
  double currtime = 0;  // most recent timestamp
  double lastcurrtime = 0;  // previous timestamp
  double timestamp = 0;  // video timestamp, corrected for lag
  double videolag = self->GetVideoLag();
  int i;

  for (i = 0; i < 10; i++)
    {
    prevtimes[i] = 0.0;
    }

  // the tracker tool provides the position of each inserted slice
  if (!self->GetTrackerTool())
    {
    return 0;
    }

  vtkMatrix4x4 *matrix = self->GetSliceAxes();
  vtkTrackerBuffer *buffer = self->GetTrackerTool()->GetBuffer();
  if (!self->RealTimeReconstruction)
    { // if reconstructing previous data, use backup buffer
    buffer = self->TrackerBuffer;
    }

  vtkVideoSource *video = self->GetVideoSource();
  vtkImageData *inData = self->GetSlice();
  
  // wait for video to start (i.e. wait for timestamp to change)
  if (video && self->RealTimeReconstruction)
    {
    while (lastcurrtime == 0 || currtime == lastcurrtime) 
      {
      int clipExt[6];
      self->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),
			  inData->GetWholeExtent());
      inData->SetUpdateExtent(clipExt);
      inData->Update();
      lastcurrtime = currtime;
      currtime = video->GetFrameTimeStamp();
      double timenow = vtkTimerLog::GetCurrentTime();
      double sleepuntil = currtime + 0.010;
      if (sleepuntil > timenow)
	{
	vtkThreadSleep(data, sleepuntil);
	}
      }
    }

  if (!self->RealTimeReconstruction)
    {
    fprintf(stderr, "In Reconstruction Thread\n");
    }

  double starttime = 0;

  // loop continuously until reconstruction is halted
  for (i = 0;;)
    {
    // save the last timestamp
    lastcurrtime = currtime;

    // update the slice data
    int clipExt[6];
    self->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),
			inData->GetWholeExtent());
    inData->SetUpdateExtent(clipExt);
    inData->Update();
    // get the timestamp for the video frame data
    if (video) {
      currtime = video->GetFrameTimeStamp();
      timestamp = currtime - videolag;
    }

    if (starttime == 0)
      {
      starttime = timestamp;
      }

    buffer->Lock();
    int flags = 0;
    if (video && (videolag > 0.0 || !self->RealTimeReconstruction))
      { // only do this if videolag is nonzero
      flags = buffer->GetFlagsAndMatrixFromTime(matrix, timestamp);
      }
    else
      {
      buffer->GetMatrix(matrix, 0);
      flags = buffer->GetFlags(0);
      if (!video)
	{
	currtime = buffer->GetTimeStamp(0);
	}
      }
    buffer->Unlock();

    // tool must be properly tracking, and the position must have updated,
    // if not we sleep until the next video frame
    if (currtime == lastcurrtime && self->RealTimeReconstruction)
      {
      double timenow = vtkTimerLog::GetCurrentTime();
      double sleepuntil = currtime + 0.033;
      if (sleepuntil > timenow)
	{
	if (vtkThreadSleep(data, sleepuntil) == 0)
	  { // return if abort occurred during sleep
	  return NULL;
	  }
	}
      }
    else if (flags & (TR_MISSING | TR_OUT_OF_VIEW))
      {
      double timenow = vtkTimerLog::GetCurrentTime();
      double sleepuntil = currtime + 0.033;
      if (sleepuntil > timenow)
	{
	if (vtkThreadSleep(data, sleepuntil) == 0)
	  { // return if abort occurred during sleep
	  return NULL;
	  }
	}
      }
    else
      {
      // do the reconstruction
      self->InsertSlice();
    
      // get current reconstruction rate over last 10 updates
      double tmptime = currtime;
      if (!self->RealTimeReconstruction)
	{ // calculate frame rate using computer clock, not timestamps
	tmptime = vtkTimerLog::GetCurrentTime();
	}
      double difftime = tmptime - prevtimes[i%10];
      prevtimes[i%10] = tmptime;
      if (i > 10 && difftime != 0)
	{
        self->ReconstructionRate = (10.0/difftime);
	}
      i++;
      }

    // check to see if we are being told to quit 
    //data->ActiveFlagLock->Lock();
    int activeFlag = *(data->ActiveFlag);
    //data->ActiveFlagLock->Unlock();

    if (activeFlag == 0)
      {
      return NULL;
      }

    if (!self->RealTimeReconstruction)
      {
      // sleep for a millisecond, just to give the main application
      // thread some time
      vtkSleep(0.001);

      if (video)
	{
	  //fprintf(stderr, "go!  %i %i %g\n", self->ReconstructionFrameCount, video->GetFrameIndex(), timestamp - starttime);
	if (--self->ReconstructionFrameCount == 0)
	  {
	  return NULL;
	  }
	video->Seek(1);
	}
      }
    }
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::StartReconstruction(int frames)
{
  if (frames <= 0)
    {
    return;
    }

  if (this->ReconstructionThreadId == -1)
    {
    fprintf(stderr, "Reconstruction Start\n");
    this->RealTimeReconstruction = 0;
    this->ReconstructionFrameCount = frames;
    vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
    this->SetSliceAxes(matrix);
    matrix->Delete();
    this->InternalExecuteInformation();
    this->ReconstructionThreadId = \
      this->Threader->SpawnThread((vtkThreadFunctionType)\
				  &vtkReconstructionThread,
				  this);
    }
}

//----------------------------------------------------------------------------
int vtkFreehandUltrasound::StopReconstruction()
{
  if (this->ReconstructionThreadId != -1)
    {
    this->Threader->TerminateThread(this->ReconstructionThreadId);
    this->ReconstructionThreadId = -1;
    return this->ReconstructionFrameCount;
    }
  return 0;
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::StartRealTimeReconstruction()
{
  if (this->ReconstructionThreadId == -1)
    {
    this->RealTimeReconstruction = 1;
    vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
    this->SetSliceAxes(matrix);
    matrix->Delete();
    this->InternalExecuteInformation();
    this->ReconstructionThreadId = \
      this->Threader->SpawnThread((vtkThreadFunctionType)\
				  &vtkReconstructionThread,
				  this);
    }
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::StopRealTimeReconstruction()
{
  if (this->ReconstructionThreadId != -1)
    {
    this->Threader->TerminateThread(this->ReconstructionThreadId);
    this->ReconstructionThreadId = -1;
    if (this->TrackerTool)
      {
      this->TrackerTool->GetBuffer()->Lock();
      this->TrackerBuffer->DeepCopy(this->TrackerTool->GetBuffer());
      this->TrackerTool->GetBuffer()->Unlock();
      }
    }
}

//----------------------------------------------------------------------------
//int vtkFreehandUltrasound::InRealTimeReconstruction()
//{
//  return (this->ReconstructionThreadId != -1);
//}

//----------------------------------------------------------------------------
// simple utility function
char *vtkJoinPath(char *cp, int n, const char *directory, const char *file)
{
  int dlen = strlen(directory);
  int flen = strlen(file);

  if (n < (dlen + flen + 2))
    {
    return 0;
    }

  strncpy(cp,directory,n);
#ifdef _WIN32
  strncpy(cp+dlen,"\\",n-dlen);
#else
  strncpy(cp+dlen,"/",n-dlen);
#endif
  strncpy(cp+dlen+1,file,n-dlen);

  return cp;
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::SaveRawData(const char *directory, int frames)
{
  if (this->ReconstructionThreadId != -1)
    {
    if (this->RealTimeReconstruction)
      {
      this->StopRealTimeReconstruction();
      }
    else
      {
      this->StopReconstruction();
      }
    }

  int res;
#ifdef _WIN32
  res = _mkdir(directory);
#else
  int mode = 0777;
  res = mkdir(directory, mode);
#endif

  if (res < 0)
    {
    vtkErrorMacro(<< "couldn't create directory " << directory);
    return;
    }

  char path[512];

  vtkJoinPath(path,512,directory,"track.txt");
  this->TrackerBuffer->WriteToFile(path);

  vtkJoinPath(path,512,directory,"video.txt");
  FILE *file = fopen(path,"w");

  fprintf(file, "# vtkFreehandUltrasound output\n\n");

  vtkImageData *image = this->VideoSource->GetOutput();
  image->UpdateInformation();

  fprintf(file, "PixelSpacing = %7.5f %7.5f;\n",
	  image->GetSpacing()[0], image->GetSpacing()[1]);
  fprintf(file, "PixelOrigin = %7.3f %7.3f;\n",
	  image->GetOrigin()[0], image->GetOrigin()[1]);
  fprintf(file, "ClipRectangle = %7.3f %7.3f %7.3f %7.3f;\n",
	  this->ClipRectangle[0], this->ClipRectangle[1],
	  this->ClipRectangle[2], this->ClipRectangle[3]);
  fprintf(file, "FanAngles = %7.2f %7.2f;\n",
	  this->FanAngles[0], this->FanAngles[1]);
  fprintf(file, "FanOrigin = %7.3f %7.3f;\n",
	  this->FanOrigin[0], this->FanOrigin[1]);
  fprintf(file, "FanDepth = %7.3f;\n", this->FanDepth);
  fprintf(file, "VideoLag = %5.3f;\n\n", this->VideoLag);

  vtkPNGWriter *writer = vtkPNGWriter::New();
  writer->SetInput(image);

  int i;
  for (i = 0; i < frames; i++)
    {
    double currtime = this->VideoSource->GetFrameTimeStamp();
    char filename[64];
    sprintf(filename,"z%04.4d.png",i);
    fprintf(file, "%-10s %14.3f;\n", filename, currtime);
    vtkJoinPath(path,512,directory,filename);
    writer->SetFileName(path);
    writer->Write();
    if (i != frames-1)
      {
      this->VideoSource->Seek(1);
      }
    }

  writer->Delete();
  fclose(file);
}

//----------------------------------------------------------------------------
char *vtkFreehandUltrasoundEatWhitespace(char *text)
{
  int i = 0;

  for (i = 0; i < 128; i++)
    {
    switch (*text)
      {
      case ' ':
      case '\t':
      case '\r':
      case '\n':
        text++;
        break;
      default:
        return text;
        break;
      }
    }

  return 0;
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound::ReadRawData(const char *directory)
{
  if (this->ReconstructionThreadId != -1)
    {
    if (this->RealTimeReconstruction)
      {
      this->StopRealTimeReconstruction();
      }
    else
      {
      this->StopReconstruction();
      }
    }

  char path[512];
  char text[128];
  char filename[128];
  char *cp;
  int i = 0;
  int line;
  int state;
  double timestamp;
  double videolag = 0.0;
  

  vtkJoinPath(path,512,directory,"track.txt");
  this->TrackerBuffer->ReadFromFile(path);

  vtkJoinPath(path,512,directory,"video.txt");
  FILE *file = fopen(path,"r");

  if (file == 0)
    {
    vtkErrorMacro(<< "can't open file " << path);
    return;
    }

  for (line = 1;; line++)
    {
    if (fgets(text, 128, file) == 0)
      { // error or end of file
      if (i != 0)
        {
        vtkErrorMacro( << "bad data: " << path << " line " << line);
        }
      break;
      }
    // eat leading whitespace
    cp = vtkFreehandUltrasoundEatWhitespace(text);
    // skip over empty lines or comments
    if (cp == 0 || *cp == '\0' || *cp == '#')
      {
      continue;
      }

    if (strncmp(cp,"VideoLag =",
                strlen("VideoLag =")) == 0)
      {
      cp += strlen("VideoLag =");
      state = 1;
      if (i != 0)
        {
        vtkErrorMacro( << "bad data: " << path << " line " << line);
        break;
        }
      }

    for (;;i++)
      {
      cp = vtkFreehandUltrasoundEatWhitespace(cp);
      if (cp == 0 || *cp == '\0' || *cp == '#')
        {
        break;
        }

      if (state == 0)
        {
        if (i == 0)
          {
	  int j;
	  for (j = 0; *cp != ' '; j++)
	    {
	    filename[j] = *cp++;
	    }
	  filename[j] = '\0';
	  }
        else if (i == 1)
          {
          timestamp = strtod(cp, &cp);
          }
        else
          {
          if (i > 1 || *cp != ';')
            {
            vtkErrorMacro( << "bad data: " << path << " line " << line);
            fclose(file);
            return;
            }
	  // do the stuff
          i = 0;
          state = 0;
          break;
          }
        }
      else if (state == 1)
        {
        if (i == 0)
          {
	  videolag = strtod(cp, &cp);
          }
        else
          {
          if (i > 0 || *cp != ';')
            {
            vtkErrorMacro( << "bad data: " << path << " line " << line);
            fclose(file);
            return;
            }
          i = 0;
          state = 0;
          break;
          }
        }
      }
    }

  fclose(file);
}




