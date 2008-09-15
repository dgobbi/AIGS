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
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkExecutive.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkFreehandUltrasound2.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkTransform.h"
#include "vtkImageAlgorithm.h"
#include "vtkImageData.h"
#include "vtkMultiThreader.h"
#include "vtkCriticalSection.h"
#include "vtkTimerLog.h"
#include "vtkTrackerBuffer.h"
#include "vtkObjectFactory.h"
#include "vtkVideoSource2.h"
#include "vtkTrackerTool.h"
#include "vtkMutexLock.h"
#include "vtkCriticalSection.h"
#include "vtkImageThreshold.h" // added by Danielle
#include "vtkImageClip.h" // added by Danielle
#include "vtkImageFlip.h" // added by Danielle

vtkCxxRevisionMacro(vtkFreehandUltrasound2, "$Revision: 1.3 $");
vtkStandardNewMacro(vtkFreehandUltrasound2);

//----------------------------------------------------------------------------
// SetVideoSource
// Set the video source to input the slices from to the parameter
//----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkFreehandUltrasound2,VideoSource,vtkVideoSource2);

//----------------------------------------------------------------------------
// SetTrackerTool
// Set the tracker tool to input transforms from to the parameter
//----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkFreehandUltrasound2,TrackerTool,vtkTrackerTool);

//----------------------------------------------------------------------------
// SetSliceAxes
// Set the slice axes to the parameter
//----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkFreehandUltrasound2,SliceAxes,vtkMatrix4x4);

//----------------------------------------------------------------------------
// SetSliceTransform
// Set the slice transform to the parameter
//----------------------------------------------------------------------------
vtkCxxSetObjectMacro(vtkFreehandUltrasound2,SliceTransform,vtkLinearTransform);

// added by danielle
//vtkCxxSetObjectMacro(vtkFreehandUltrasound2, DepthClipData, vtkImageClip);
vtkCxxSetObjectMacro(vtkFreehandUltrasound2, RotationClipData, vtkImageClip);
//vtkCxxSetObjectMacro(vtkFreehandUltrasound2, DepthThreshold, vtkImageThreshold);
vtkCxxSetObjectMacro(vtkFreehandUltrasound2, RotationThreshold, vtkImageThreshold);
//vtkCxxSetObjectMacro(vtkFreehandUltrasound2, FlipThreshold, vtkImageThreshold);
vtkCxxSetObjectMacro(vtkFreehandUltrasound2, FlipTransform, vtkTransform);


// for keeping track of threading information
struct vtkFreehand2ThreadStruct
{
  vtkFreehandUltrasound2 *Filter;
  vtkImageData   *Input;
  vtkImageData   *Output;
};

//----------------------------------------------------------------------------
// Constructor
// Just initialize objects and set initial values for attributes
//----------------------------------------------------------------------------
vtkFreehandUltrasound2::vtkFreehandUltrasound2()
{

cout << "HI I'm in Danielle's new code!\n";

  this->SetNumberOfInputPorts(0); // inherited from vtkAlgorithm
  this->SetNumberOfOutputPorts(1); // inherited from vtkAlgorithm
 
  // set the video lag (i.e the lag between tracking information and
  // video information)
  this->VideoLag = 0.0;

  // TODO one PixelCount for each threadId, where 0 <= threadId < 4
  this->PixelCount[0] = 0;
  this->PixelCount[1] = 0;
  this->PixelCount[2] = 0;
  this->PixelCount[3] = 0;
  // set up the output (it will have been created in the superclass)
  // (the output is the reconstruction volume, the second component
  // is the alpha component that stores whether or not a voxel has
  // been touched by the reconstruction)
 
  if(this->GetExecutive())
    {
    if(this->GetExecutive()->GetOutputInformation())
      {
      //cout<<"output port info\n"; // removed by Danielle
      }
    }

  // also see ExecuteInformation for how these are set
  // TODO should these be hard coded like this?
  // TODO executeInformation is vtk 4
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

  this->OldScalarType = VTK_UNSIGNED_CHAR;
  this->OldNComponents = 1;
  
  this->OldOutputSpacing[0] = 1.0;
  this->OldOutputSpacing[1] = 1.0;
  this->OldOutputSpacing[2] = 1.0;

  this->OldOutputOrigin[0] = 0;
  this->OldOutputOrigin[1] = 0;
  this->OldOutputOrigin[2] = 0;

  this->OldOutputExtent[0] = 0;
  this->OldOutputExtent[1] = 0;
  this->OldOutputExtent[2] = 0;
  this->OldOutputExtent[3] = 0;
  this->OldOutputExtent[4] = 0;
  this->OldOutputExtent[5] = 0;
  
  // accumulation buffer is for compounding, there is a voxel in
  // the accumulation buffer for each voxel in the output
  this->AccumulationBuffer = vtkImageData::New();
   
  // this is set if some parameter is changed which causes the data
  // in the output to become invalid: the output will be erased
  // during the next ExecuteInformation
  // TODO executeInformation is vtk4
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
  //this->SliceTransform = NULL; // changed by danielle
  this->SliceTransform = vtkTransform::New(); // initialized to identity

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
  this->NumberOfThreads = 1;//this->Threader->GetNumberOfThreads();  
  
  // for running the reconstruction in the background
  this->VideoSource = NULL;
  this->TrackerTool = NULL;
  this->TrackerBuffer = vtkTrackerBuffer::New();
  this->ReconstructionThreader = vtkMultiThreader::New();
  this->ReconstructionRate = 0;
  this->ReconstructionThreadId = -1;
  this->RealTimeReconstruction = 0; // # real-time or buffered
  this->ReconstructionFrameCount = 0; // # of frames to reconstruct
  this->ActiveFlagLock = vtkCriticalSection::New();

  // added by Danielle
  this->FanRotation = 0;
  this->PreviousFanRotation = 0;
 // this->DepthClipData = NULL;
  this->RotationClipData = NULL;
 // this->DepthThreshold = NULL;
  this->RotationThreshold = NULL;
  //this->FlipThreshold = NULL;
  //this->ImageIsFlipped = -1; // we don't know
  this->FlipTransform = vtkTransform::New();

  this->FlipHorizontalOnOutput = 0;
  this->FlipVerticalOnOutput = 0;
}

//----------------------------------------------------------------------------
// Destructor
// Stop the reconstruction and delete stuff
//----------------------------------------------------------------------------
vtkFreehandUltrasound2::~vtkFreehandUltrasound2()
{
  this->StopRealTimeReconstruction();

  // TODO why setting these to NULL instead of deleting them?
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
  // TODO why setting these to null instead of deleting them?
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
	/*if (this->DepthClipData)
	{
		this->DepthClipData->Delete();
	}*/
	if (this->RotationClipData)
	{
		this->RotationClipData->Delete();
	}
	/*if (this->DepthThreshold)
	{
		this->DepthThreshold->Delete();
	}*/
	if (this->RotationThreshold)
	{
		this->RotationThreshold->Delete();
	}
	/*if (this->FlipThreshold)
	{
		this->FlipThreshold->Delete();
	}*/
	if (this->FlipTransform)
	{
		this->FlipTransform->Delete();
	}

  // TODO also delete OutputOrigin, OutputSpacing, OutputExtent,
  // OldOutputOrigin, OldOutputSpacing, OldOutputExtent, ClipRectangle,
  // FanAngles, FanOrigin, ActiveFlagLock
}


// added by Danielle to replace deleted generic SetCompounding method
void vtkFreehandUltrasound2::SetCompounding(int comp)
{
	// we are switching from no compounding to compounding
	/*if (this->GetCompounding() == 0 &&  comp == 1)
	{
		this->AccumulationBuffer->SetScalarType(VTK_UNSIGNED_SHORT);
		this->AccumulationBuffer->SetUpdateExtent(this->OutputExtent);// new
		this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
		this->AccumulationBuffer->SetExtent(this->OutputExtent); // new from copied below
		this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
		this->AccumulationBuffer->SetOrigin(this->OutputOrigin);


		std::cout << "number of scalars in acc buffer = " << this->AccumulationBuffer->GetNumberOfScalarComponents() << std::endl;


		//this->AccumulationBuffer->AllocateScalars(); // new!
		//this->AccumulationBuffer->Update(); // new!

	//			std::cout << "in SetCompounding() the accumulation buffer is now:" << std::endl;
	//	this->AccumulationBuffer->Print(std::cout);


	//}*/
	this->Compounding = comp;

	//std::cout << "compounding = " << this->GetCompounding() << std::endl;
}



//----------------------------------------------------------------------------
// SetSlice
// Set the image slice to insert into the reconstruction volume
// If there is a video source, then set the slice to be a slice from the video
// source.  Otherwise, set the slice to the parameter.
// TODO looks weird - shouldn't we be always setting the slice to the parameter?
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::SetSlice(vtkImageData *slice)
{  
  /*if( slice )
    {
    this->SetInputConnection(0, slice->GetProducerPort());
    }*/
	if(this->VideoSource)
	{
	this->Slice = this->VideoSource->GetOutput();
	}
	else if(slice)
	{
	this->Slice = slice;
	}
}



//----------------------------------------------------------------------------
// GetSlice
// If there is a video source, return the slice from the video source.
// Otherwise, return the slice attribute.
//----------------------------------------------------------------------------
vtkImageData* vtkFreehandUltrasound2::GetSlice()
{
	//this->Slice->Update();
	if(this->VideoSource)
	{
	return this->VideoSource->GetOutput(); 
	}
	else
	{
	return this->Slice;
	}
  /*if (this->GetNumberOfInputConnections(0) < 1)
    {
    return NULL;
    }
  
  if (this->GetExecutive())
    {
    return vtkImageData::SafeDownCast(this->GetExecutive()->GetInputData(0, 0));
    }
  else
    {
    cerr<< "GetSlice: Executive = NULL \n";
    exit(0);
    }*/
}

//----------------------------------------------------------------------------
// GetOutput
// Get the algorithm output as a vtkImageData
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
vtkImageData *vtkFreehandUltrasound2::GetOutput()
{
  // inherited from vtkAlgorithm - gets the algorithm output from the given port
  if(this->GetOutputDataObject(0))
    {
      // GetOutputDataObject returns a vtkDataObject
      // TODO I think vtkImageData::SafeDownCast casts it as a vtkImageData - true?
    return vtkImageData::SafeDownCast(this->GetOutputDataObject(0));
    }
  else
    {
    return NULL;
    }
}



//----------------------------------------------------------------------------
// SetPixelCount
// Set the number of pixels inserted by a particular threadId
//----------------------------------------------------------------------------

void  vtkFreehandUltrasound2::SetPixelCount(int threadId, int count)
{
  if( threadId < 4 && threadId >= 0)
    {
    this->PixelCount[threadId] = count;
    }
}

//----------------------------------------------------------------------------
// IncrementPixelCount
// Increment the number of pixels inserted by a particular threadId by a
// particular increment value
//----------------------------------------------------------------------------
void  vtkFreehandUltrasound2::IncrementPixelCount(int threadId, int increment)
{
  if( threadId < 4 && threadId >= 0)
    {
    this->PixelCount[threadId] += increment;
    }
}

//----------------------------------------------------------------------------
// GetPixelCount
// Get the total number of pixels inserted
//----------------------------------------------------------------------------
int  vtkFreehandUltrasound2::GetPixelCount()
{
  return ( this->PixelCount[0] + this->PixelCount[1] +
	   this->PixelCount[2] + this->PixelCount[3] );
}
//----------------------------------------------------------------------------
// GetClipExtent
// convert the ClipRectangle (which is in millimetre coordinates) into a
// clip extent that can be applied to the input data - number of pixels (+ or -)
// from the origin (the z component is copied from the inExt parameter)
// 
// clipExt = {x0, x1, y0, y1, z0, z1} <-- the "output" of this function is to
//                                        change this array
// inOrigin = {x, y, z} <-- the origin in mm
// inSpacing = {x, y, z} <-- the spacing in mm
// inExt = {x0, x1, y0, y1, z0, z1} <-- min/max possible extent, in pixels
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::GetClipExtent(int clipExt[6],
				     vtkFloatingPointType inOrigin[3],
				     vtkFloatingPointType inSpacing[3],
				     const int inExt[6])
{
  // TODO potential problem - clip rectangle is x0, y0, x1, y1 while extent is
  // x0, x1, y0, y1, z0, z1
  // Map the clip rectangle (millimetres) to pixels --> number of pixels (+ or -)
  // from the origin
  int x0 = (int)ceil((this->GetClipRectangle()[0]-inOrigin[0])/inSpacing[0]);
  int x1 = (int)floor((this->GetClipRectangle()[2]-inOrigin[0])/inSpacing[0]);
  int y0 = (int)ceil((this->GetClipRectangle()[1]-inOrigin[1])/inSpacing[1]);
  int y1 = (int)floor((this->GetClipRectangle()[3]-inOrigin[1])/inSpacing[1]);

  // Make sure that x0 <= x1 and y0 <= y1, otherwise swap 
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

  // Set the clip extent
  clipExt[0] = x0;
  clipExt[1] = x1;
  clipExt[2] = y0;
  clipExt[3] = y1;
  clipExt[4] = inExt[4];
  clipExt[5] = inExt[5];
}

//----------------------------------------------------------------------------
// SliceCalculateMaxSliceSeparation
// Calculate the maximum distance between two slices, given the
// index transformation matrix for each.
// This method is currently not used, so chances are very good that
// it will NOT provide the correct result.
//----------------------------------------------------------------------------
double vtkFreehandUltrasound2::SliceCalculateMaxSliceSeparation(vtkMatrix4x4 *m1,
                                                          vtkMatrix4x4 *m2)
{
  // The first thing to do is find the four corners of the plane.
  vtkImageData *inData = this->GetSlice();
  // TODO I don't think I need to change these for vtk 5 b/c backwards compatible, right?  If you have to change this, have to do this for multiple
  // vtkImageData::UpdateInformation calls
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
// PrintSelf
// Prints out attribute data
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

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
// GetMTime
// Account for the MTime of the transform and its matrix when determining
// the MTime of the filter - the MTime of the transform is the largest mTime of
// the superclass, the slice transform, and the slice transform's matrix
// TODO [note to self: this made sense in vtkImageReslice, but does it make
//  any sense here?]
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
unsigned long int vtkFreehandUltrasound2::GetMTime()
{
  unsigned long mTime=this->Superclass::GetMTime();
  unsigned long time;

  if ( this->SliceTransform != NULL )
    {
    time = this->SliceTransform->GetMTime();
    // if time > mTime then mTime = time; else mTime = mTime
    mTime = ( time > mTime ? time : mTime );
    time = this->SliceTransform->GetMatrix()->GetMTime();
    mTime = ( time > mTime ? time : mTime );    
    }

  return mTime;
}

//----------------------------------------------------------------------------
// RequestData
// Asks for the output data object to be populated with the actual output data
// Overrides the vtkImageAlgorithm algorithm
// TODO - this isn't really doing anything... should be coping the output
// somewhere?
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::RequestData(vtkInformation* request,
				       vtkInformationVector **vtkNotUsed(inInfo),
				       vtkInformationVector* outInfo)
{
	//std::cout << "In RequestData" << std::endl;

  // TODO remove this comment later
  //VTK 4: In most VTK classes this method is responsible for calling Execute,
  // but since the output data has already been generated it just fools
  // the pipeline into thinking that Execute has been called

  // Get the vtkDataObject associated with output port 0
  vtkDataObject *outObject = 
    outInfo->GetInformationObject(0)->Get(vtkDataObject::DATA_OBJECT());

  // if we are not currently running a reconstruction and we need to clear, then
  // clear
  if (this->ReconstructionThreadId == -1 && this->NeedsClear == 1)
    {
    this->InternalClearOutput();
    }
  
  // TODO this would have been done already in the call to ProcessRequest, so don't do it here
  outInfo->GetInformationObject(0)->Set(vtkDemandDrivenPipeline::DATA_NOT_GENERATED(), 1);
  //Set the flag for the data object associated with port 0 that data has been generated -
  // sets the data released flag to 0 and sets a new update time
  ((vtkImageData *)outObject)->DataHasBeenGenerated();

  return 1;
}

//----------------------------------------------------------------------------
// VTK 4
// void vtkFreehandUltrasound2::UpdateData(vtkDataObject *outObject) 
// {
//   if (this->ReconstructionThreadId == -1 && this->NeedsClear == 1)
//     {
//     this->InternalClearOutput();//     }

//   ((vtkImageData *)outObject)->DataHasBeenGenerated();
// }
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
// RequestInformation
// Asks the algorithm to provide as much information as it can about what the
// output data will look like once the algorithm has generated it.
// For both the first vtkInformation object and the data object associate with
// it, sets the whole extent, spacing and origin to match those of this->
// object, and the scalar type and number of scalar components to match those
// of the slice.  Also updates the "old" attributes and NeedsClear if an
// parameter has changed.
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::RequestInformation(
	vtkInformation* vtkNotUsed(request),
	vtkInformationVector** vtkNotUsed(inInfo),
	vtkInformationVector* outInfoVector)
{
	//std::cout << "In RequestInformation" << std::endl;

	//std::cout << "BEFORE REQUEST INFORMATION" << std::endl;
	//this->GetOutput()->Print(std::cout);



	// TODO remove this comment later
	// could be equivalent to UpdateInformation() which calls
	// DemandDrivenPipeline->UpdateInformation and it in turn calls 
	// vtkImageAlgorithm::ProcessRequest()
	// TODO what is vtkNotUsed???

	// to avoid conflict between the main application thread and the
	// realtime reconstruction thread
	// cout<<"RequestInformation"<<endl;
	if (this->ReconstructionThreadId == -1)
	{
		// TODO note that I think calls to internalexecuteinformation should be replaced
		// with calls to RequestInformation
		//this->InternalExecuteInformation();

		// get the information object for the first  port of the output informationVector
		vtkInformation *outInfo = outInfoVector->GetInformationObject(0);
		// would have been created by a call to REQEST_DATA_OBJECT, presumably handled
		// by vtkImageAlgorithm
		vtkImageData *output = 
			dynamic_cast<vtkImageData *>(outInfo->Get(vtkDataObject::DATA_OBJECT()));

		// the whole extent, spacing, origin, PIPELINE scalar type (ex double; until REQUEST_DATA
		// is called, the actual scalar type may be different) and number of scalar components of
		// the object created by the REQUEST_DATA_OBJECT call
		// TODO still valid?  Since vtkImageAlgorithm generates this?
		int oldwholeextent[6];
		vtkFloatingPointType oldspacing[3];
		vtkFloatingPointType oldorigin[3];
		int oldtype = output->GetScalarType();
		int oldncomponents = output->GetNumberOfScalarComponents();
		output->GetWholeExtent(oldwholeextent);
		output->GetSpacing(oldspacing);
		output->GetOrigin(oldorigin);

		// if we don't have a slice yet, then set the slice to be the output of the video source
		//cout<<"RequestInformation"<<endl;
		if (this->GetVideoSource())
		{
			if (this->GetSlice() == 0)
			{
				//cout<<"INITIALLY NO SLICE SET"<<endl;
				this->SetSlice( this->GetVideoSource()->GetOutput());
				//this->Slice->Register(this);
			} 
		} 

		// if we have a slice now...
		if (this->GetSlice())
		{
			// get the newest slice information - updating origin and spacing and extent from pipeline
			// TODO what is this really doing?  we don't expect the slice to change, do we? but maybe it hasn't
			// been updated before...
			this->GetSlice()->UpdateInformation();

			// for both the outInfo vtkInformation object and the data object associate with outInfo,
			// set the whole extent, spacing and origin to match those of this-> object, and the scalar
			// type and number of scalar components to match those of the slice
			vtkDataObject::SetPointDataActiveScalarInfo(outInfo,
				this->GetSlice()->GetScalarType(),
				this->GetSlice()->GetNumberOfScalarComponents()+1);
			//int *wExtent = this->GetOutput()->GetExtent();	
			//cout<<"RequestInfo: Whole Extent Of Output: "<< wExtent[0]<<", "<< wExtent[1]<<", "<< wExtent[2]<<", " 
			// << wExtent[3]<<", " << wExtent[4]<<", "<< wExtent[5]<<endl;  // taken out by danielle
			output->SetScalarType(this->GetSlice()->GetScalarType());
			output->SetNumberOfScalarComponents(this->GetSlice()->
				GetNumberOfScalarComponents()+1);
			outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
				this->OutputExtent, 6);
			outInfo->Set(vtkDataObject::SPACING(),
				this->OutputSpacing, 3);
			outInfo->Set(vtkDataObject::ORIGIN(),
				this->OutputOrigin, 3);
			output->SetExtent(this->OutputExtent);
			output->SetWholeExtent(this->OutputExtent);
			output->SetSpacing(this->OutputSpacing);
			output->SetOrigin(this->OutputOrigin);

			// if the output has changed, then we need to clear
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

			/* if (this->OldScalarType != output->GetScalarType() ||
			this->OldNComponents != output->GetNumberOfScalarComponents() ||
			this->OldOutputExtent[0] != this->OutputExtent[0] ||
			this->OldOutputExtent[1] != this->OutputExtent[1] ||
			this->OldOutputExtent[2] != this->OutputExtent[2] ||
			this->OldOutputExtent[3] != this->OutputExtent[3] ||
			this->OldOutputExtent[4] != this->OutputExtent[4] ||
			this->OldOutputExtent[5] != this->OutputExtent[5] ||
			this->OldOutputSpacing[0] != this->OutputSpacing[0] ||
			this->OldOutputSpacing[1] != this->OutputSpacing[1] ||
			this->OldOutputSpacing[2] != this->OutputSpacing[2] ||
			this->OldOutputOrigin[0] != this->OutputOrigin[0] ||
			this->OldOutputOrigin[1] != this->OutputOrigin[1] ||
			this->OldOutputOrigin[2] != this->OutputOrigin[2])
			{
			this->NeedsClear = 1;
			}*/

			// if we are compounding...
			if (this->Compounding)
			{
				// Remember accumulation buffer - a voxel here for each voxel in output
				// Set the scalar type, whole extent, spacing and origin of the accumulation
				// buffer to be the same as this->

				//std::cout << "we are trying thisssssssssssssssss" << std::endl;

/* old stuff */
				int *extent = this->AccumulationBuffer->GetExtent();
				//this->AccumulationBuffer->SetScalarType(VTK_UNSIGNED_SHORT); // Danielle = should match above and get from the slice?
				this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
				this->AccumulationBuffer->SetExtent(this->OutputExtent); // another trial by Danielle
				this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
				this->AccumulationBuffer->SetOrigin(this->OutputOrigin);
				this->AccumulationBuffer->SetScalarType(this->GetOutput()->GetScalarType()); // added by Danielle
				//this->AccumulationBuffer->SetNumberOfScalarComponents(this->GetOutput()->GetNumberOfScalarComponents()); // Added by Danielle
				this->AccumulationBuffer->SetUpdateExtent(this->OutputExtent); // added by Danielle - makes things work!
				//this->AccumulationBuffer->UpdateInformation(); // perhaps does the same thing as above!
				//this->AccumulationBuffer->Initialize(); // more trials

				/*if(vtkInformation* pInfo = this->AccumulationBuffer->GetPipelineInformation())
				 {
					 std::cout << "HEEEEEEEEEEERRRRRRRRRRRREEEEEEEEEEEE" << std::endl;
					pInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT_INITIALIZED(), 1);
				 }*/
				this->AccumulationBuffer->Update(); // another attempt!


/* copied from above stuff
				int *extent = this->AccumulationBuffer->GetExtent();
				vtkInformation *accInfo = this->AccumulationBuffer->GetInformation();
				vtkDataObject::SetPointDataActiveScalarInfo(accInfo,
				this->GetOutput()->GetScalarType(),
				this->GetOutput()->GetNumberOfScalarComponents()-1); // changed because acc has only 1 scalar component
				this->AccumulationBuffer->SetScalarType(this->GetOutput()->GetScalarType());
				this->AccumulationBuffer->SetNumberOfScalarComponents(this->GetOutput()->
					GetNumberOfScalarComponents()-1);
				this->AccumulationBuffer->SetExtent(this->OutputExtent);
				this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
				this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
				this->AccumulationBuffer->SetOrigin(this->OutputOrigin);
				this->AccumulationBuffer->Update();
				*/




				// if the accumulation buffer has changed, we need to clear
				if (extent[0] != this->OutputExtent[0] ||
					extent[1] != this->OutputExtent[1] ||
					extent[2] != this->OutputExtent[2] ||
					extent[3] != this->OutputExtent[3] ||
					extent[4] != this->OutputExtent[4] ||
					extent[5] != this->OutputExtent[5])
				{

					//std::cout << "we are clearing because the accumulation buffer has changed" << std::endl;
					this->NeedsClear = 1;
				}
			}

			//std::cout << "we are finished thisssssssssssssssss" << std::endl;



			// copy the extent, whole extent, spacing, origin, scalar type and number of
			// components into the "old" stuff
			/*output->GetExtent(this->OldOutputExtent);
			output->GetWholeExtent(this->OldOutputExtent);
			output->GetSpacing(this->OldOutputSpacing);
			output->GetOrigin(this->OldOutputOrigin);
			this->OldScalarType = output->GetScalarType();
			this->OldNComponents = output->GetNumberOfScalarComponents();*/
			// deleted by Danielle - useless because these attributes are never used = because stuff above is
			// commented out
		}
	}


	//std::cout << "AFTER REQUEST INFORMATION" << std::endl;
	//this->GetOutput()->Print(std::cout);



	return 1;
}


//----------------------------------------------------------------------------
// RequestUpdateExtent
// Asks for the input update extent necessary to produce a given output
// update extent.  Sets the update extent of the input information object
// to equal the whole extent of hte input information object - we need the
// entire whole extent of the input data object to generate the output
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
int  vtkFreehandUltrasound2::RequestUpdateExtent(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *vtkNotUsed(outputVector))
{

	//std::cout << "In RequestUpdateExtent" << std::endl;

  // TODO this is a big problem!  Could be everything...
  // changed by danielle
	return 1; // removed by Danielle!
  
  int inExt[6];
  // TODO what is the point of this?
  //this->GetSlice()->GetWholeExtent(); // deleted by Danielle



  // Set the update extent of the input information object to equal the
  // whole extent of the input information object
  cout << "1\n";
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0); // TODO this dies if you take out the return above
  cout << "2\n";
  inInfo->Get(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), inExt); // get the whole extent of inInfo
  cout << "3\n";
  inInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT(), inExt, 6); // se the update extent of inInfo
  cout << "4\n";



  return 1;
}

//----------------------------------------------------------------------------
// This is vtk 4
// void vtkFreehandUltrasound2::ExecuteInformation() 
// {
//   // to avoid conflict between the main application thread and the
//   // realtime reconstruction thread
//   if (this->ReconstructionThreadId == -1)
//     {
//     //    this->InternalExecuteInformation();
//     }
// }
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
// FillOutputPortInformation
// Define the output port information - all output ports produce vtkImageData
// Overrides the vtkImageAlgorithm function
//---------------------------------------------------------------------------
int vtkFreehandUltrasound2::FillOutputPortInformation(
  int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkImageData");
 
  return 1;
}


//----------------------------------------------------------------------------
// FillInputPortInformation
// Define the input port information - the input at port 0 needs vtkImageData
// as input
// Overrides the vtkImageAlgorithm function
//----------------------------------------------------------------------------
int  vtkFreehandUltrasound2::FillInputPortInformation(
  int port, vtkInformation* info)
{
  if (port == 0)
    {
    // The feature image input
    //info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL());
    info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkImageData");
  
    }
  return 0;
}

//----------------------------------------------------------------------------
// InternalExecuteInformation
// Gets the output ready to receive data, so we need to call it before the
// reconstruction starts.  Must update the information for the output and for
// the accumulation buffer.
//----------------------------------------------------------------------------

// TODO should remove this function - has the same stuff as RequestInformation
// Should replace all calls to it with calls to RequestInformation
void vtkFreehandUltrasound2::InternalExecuteInformation() 
{
	//std::cout << "In InternalExecuteInformation" << std::endl;
	
//	std::cout << "BEFORE INTERNAL EXECUTE INFORMATION" << std::endl;
//	this->GetOutput()->Print(std::cout);



  vtkImageData *output = this->GetOutput();
  vtkInformation *outInfo = output->GetPipelineInformation();
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
    if (this->GetSlice() == 0)
      {
      this->SetSlice( this->GetVideoSource()->GetOutput());
      //this->Slice->Register(this);
      }
    }
  // the 'Slice' is not an input, but it is like an input
  if (this->GetSlice())
    {
    this->GetSlice()->UpdateInformation();// update the slice info
    // VTK 5: Done inside the Request Information 

    // Set up the pipeline (need outInfo from the pipeline)
    // vtkDataObject::
//       SetPointDataActiveScalarInfo(outInfo,
// 				   this->GetSlice()->GetScalarType(),
// 				   this->GetSlice()->
// 				   GetNumberOfScalarComponents()+1);
    
    // Set up the data object to make sure everything is in sync
    // (don't trust pipeline mechanism to copy pipeline info to data)
    
  //   output->SetScalarType(this->GetSlice()->GetScalarType());
//     output->SetNumberOfScalarComponents(this->GetSlice()->
//                                         GetNumberOfScalarComponents()+1);
   
//*****
//  For VTK 5, this was moved into ComputePipelineMTIme()
//    unsigned long mtime = this->GetSlice()->GetPipelineMTime(); 
    //for compiling temporarily turned off
    // if (mtime > output->GetPipelineMTime())
//       {
//       output->SetPipelineMTime(mtime);
//       }
    }    
 
  // set up the output dimensions and info here
  outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(),
	       this->OutputExtent, 6);
  outInfo->Set(vtkDataObject::SPACING(),
	       this->OutputSpacing, 3);
  outInfo->Set(vtkDataObject::ORIGIN(),
	       this->OutputOrigin, 3);

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

		//std::cout << "we are trying thissssssssssssssssssssssss 2" << std::endl;


    int *extent = this->AccumulationBuffer->GetExtent();

    /*this->AccumulationBuffer->SetScalarType(VTK_UNSIGNED_SHORT);
    this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
    this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
    this->AccumulationBuffer->SetOrigin(this->OutputOrigin);*/  // removed by Danielle

	// added by Danielle to match stuff in RequestInformation()
	//this->AccumulationBuffer->SetScalarType(VTK_UNSIGNED_SHORT); // Danielle = should match above and get from the slice?
	this->AccumulationBuffer->SetWholeExtent(this->OutputExtent);
	this->AccumulationBuffer->SetExtent(this->OutputExtent); // another trial by Danielle
	this->AccumulationBuffer->SetSpacing(this->OutputSpacing);
	this->AccumulationBuffer->SetOrigin(this->OutputOrigin);
	this->AccumulationBuffer->SetScalarType(this->GetOutput()->GetScalarType()); // added by Danielle
	//this->AccumulationBuffer->SetNumberOfScalarComponents(this->GetOutput()->GetNumberOfScalarComponents()); // Added by Danielle
	this->AccumulationBuffer->SetUpdateExtent(this->OutputExtent); // added by Danielle - makes things work!
	//this->AccumulationBuffer->UpdateInformation(); // perhaps does the same thing as above!
	//this->AccumulationBuffer->Initialize(); // more trials
					/*if(vtkInformation* pInfo = this->AccumulationBuffer->GetPipelineInformation())
				 {
					 std::cout << "HEEEEEEEEEEERRRRRRRRRRRREEEEEEEEEEEE" << std::endl;
					pInfo->Set(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT_INITIALIZED(), 1);
				 }*/
	
	
	this->AccumulationBuffer->Update(); // another attempt!


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


	//std::cout << "we are finished thisssssssssssssssssssssssssssss 2 " << std::endl;

//		std::cout << "BEFORE INTERNAL EXECUTE INFORMATION" << std::endl;
//	this->GetOutput()->Print(std::cout);



}

//----------------------------------------------------------------------------
// this is vtk 4
// VTK 5: Change to a VTK 5 style RequestInformation method
// could be equivalent to UpdateInformation() which calls
// DemandDrivenPipeline->UpdateInformation and it in turn calls 
// vtkImageAlgorithm::ProcessRequest()

// void vtkFreehandUltrasound2::UpdateInformation()
// {
//   if (this->ReconstructionThreadId == -1)
//     {
//     this->Superclass::UpdateInformation();
//     }
// }
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
// ProcessRequest
// The main method of an algorithm - called whenever there is a request
// We make sure that the REQUEST_DATA_NOT_GENERATED is set so that the data
// object is not initialized everytime an update is called
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::ProcessRequest(vtkInformation* request,
                              vtkInformationVector** inputVector,
                              vtkInformationVector* outputVector)
{

	//std::cout << "In ProcessRequest" << std::endl;

  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA_NOT_GENERATED()))
    {
    // Mark all outputs as not generated so that the executive does
    // not try to handle initialization/finalization of the outputs.
    // We will do it here.
    int i;
    for(i=0; i < outputVector->GetNumberOfInformationObjects(); ++i)
      {
      vtkInformation* outInfo = outputVector->GetInformationObject(i);
      outInfo->Set(vtkDemandDrivenPipeline::DATA_NOT_GENERATED(), 1);
      }
    }

  // Calls to RequestInformation, RequestUpdateExtent and RequestData are
  //handled here, in vtkImageAlgorithm's ProcessRequest
  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

//----------------------------------------------------------------------------
// ComputePipelineMTime
// Compute the modified time for the pipeline - traverses the full length of
// the pipeline
// TODO not intuitive for me as to why the pipeline m time should be the mtime
// of the slice... should be calling ComputePipelineMTime of the associated
// executive?  Or should return it's own mtime (or call GetMTime) like
// like vtkAlgorithm does?
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::ComputePipelineMTime(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **vtkNotUsed(inInfoVec),
  vtkInformationVector *vtkNotUsed(outInfoVec),
  int requestFromOutputPort,
  unsigned long* mtime)
{
  if(this->GetSlice())
    {
    *mtime = this->GetSlice()->GetPipelineMTime(); 
    }
  return 1;
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
// static inline int vtkUltraFloor(double x)
// {
// #if defined mips || defined sparc
//   return (int)((unsigned int)(x + 2147483648.0) - 2147483648U);
// #elif defined i386
//   double tempval = (x - 0.25) + 3377699720527872.0; // (2**51)*1.5
//   return ((int*)&tempval)[0] >> 1;
// #else
//   return int(floor(x));
// #endif
// }

static inline int vtkUltraFloor(double x)
{
#if defined mips || defined sparc || defined __ppc__
  x += 2147483648.0;
  unsigned int i = (unsigned int)(x);
  return (int)(i - 2147483648U);
#elif defined i386 || defined _M_IX86
  union { double d; unsigned short s[4]; unsigned int i[2]; } dual;
  dual.d = x + 103079215104.0;  // (2**(52-16))*1.5
  return (int)((dual.i[1]<<16)|((dual.i[0])>>16));
#elif defined ia64 || defined __ia64__ || defined IA64
  x += 103079215104.0;
  long long i = (long long)(x);
  return (int)(i - 103079215104LL);
#else
  double y = floor(x);
  return (int)(y);
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
// vtkNearestNeighborInterpolation
// Do nearest-neighbor interpolation of the input data 'inPtr' of extent 
// 'inExt' at the 'point'.  The result is placed at 'outPtr'.  
// If the lookup data is beyond the extent 'inExt', set 'outPtr' to
// the background color 'background'.  
// The number of scalar components in the data is 'numscalars'
//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
// vtkTrilinearInterpolation
// Do trilinear interpolation of the input data 'inPtr' of extent 'inExt'
// at the 'point'.  The result is placed at 'outPtr'.  
// If the lookup data is beyond the extent 'inExt', set 'outPtr' to
// the background color 'background'.  
// The number of scalar components in the data is 'numscalars'
//----------------------------------------------------------------------------
template <class F, class T>
static int vtkTrilinearInterpolation(F *point, T *inPtr, T *outPtr,
                                     unsigned short *accPtr, int numscalars, 
                                     int outExt[6], int outInc[3])
{

	//std::cout << "in vtkTrilinearInterpolation" << std::endl;

  F fx, fy, fz;

  int outIdX0 = vtkUltraFloor(point[0], fx);  // covert point[0] into integer component and a fraction
  int outIdY0 = vtkUltraFloor(point[1], fy);	// point[0] is unchanged, outIdX0 is the integer (floor), fx is the float
  int outIdZ0 = vtkUltraFloor(point[2], fz);

  int outIdX1 = outIdX0 + (fx != 0); // ceiling
  int outIdY1 = outIdY0 + (fy != 0);
  int outIdZ1 = outIdZ0 + (fz != 0);

  // at this point in time we have the floor (outIdX0), the ceiling (outIdX1) and the fractional component (fx) for
  // x, y and z
  
  // bounds check (remember | is bitwise OR)
  if ((outIdX0 | (outExt[1]-outExt[0] - outIdX1) |
       outIdY0 | (outExt[3]-outExt[2] - outIdY1) |
       outIdZ0 | (outExt[5]-outExt[4] - outIdZ1)) >= 0)
    {// do reverse trilinear interpolation
		// trilinear interpolation would use the pixel values to interpolate something in the middle
		// we have the something in the middle and want to spread it to the discrete pixel values around it, in an
		// interpolated way
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

    int idx[8];// increment between the output pointer and the 8 pixels to work on
    idx[0] = factX0 + factY0Z0;
    idx[1] = factX0 + factY0Z1;
    idx[2] = factX0 + factY1Z0;
    idx[3] = factX0 + factY1Z1;
    idx[4] = factX1 + factY0Z0;
    idx[5] = factX1 + factY0Z1;
    idx[6] = factX1 + factY1Z0;
    idx[7] = factX1 + factY1Z1;

    F rx = 1 - fx; // remainders from the fractional components - difference between the fractional value and the ceiling
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
    
    F f, r, a; // changed by Danielle



    T *inPtrTmp, *outPtrTmp;
    if (accPtr)
      {
      //------------------------------------
      // accumulation buffer: do compounding

		  //std::cout << "in the compounding if loop" << std::endl;

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
        accPtrTmp = accPtr+ ((unsigned short)(idx[j]/outInc[0])); ///// compensate for different number of scalar components
											// this is the same as adding the increments for the acc buffer (should be)
        f = fdx[j];

		//std::cout << "problem here?" << std::endl;

		//int temp = int(*accPtr);

		/*std::cout << "output is ok... " << accPtr;
		std::cout << " " << (int)(*accPtr);
		std::cout << " " << idx[j] << " ";
		std::cout << outInc[0] << " ";
		std::cout << accPtrTmp << " ";
		*/

		/*
		- accPtr = 4D550020
		- *accPtr = 0
		- idx[j] = 200379422
		- outInc[0] = 2
		accPtrTmp = 1497795646 (in int, not hex)
		*/
		//std::cout << std::endl << "finished!" << std::endl;
		//std::cout << "printing outPtrTmp" << std::endl;
		//std::cout << (int) (*outPtrTmp) << std::endl;
		//std::cout << "did it" << std::endl;
				//std::cout << "*accPtrTmp = " << (int)(*accPtrTmp) << std::endl;

				//std::cout << "printed that thing!" << std::endl;

		r = F((*accPtrTmp)/255);

 /* COPIED FROM NNINTERPOLATION - dies whenever you access accPtrTmp / accPtr1    
 int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc;
	  std::cout << "making accPtr1" << std::endl;
      unsigned short *accPtr1 = accPtr + inc/outInc[0];
	  std::cout << "making newa" << std::endl;
      unsigned short newa = *accPtr1 + 255; // dies here!
	  std::cout << "made newa" << std::endl;*/
		
		//int temp2 = (int) F(*accPtrTmp);

		//std::cout << "temp2 is ok temp 2 = " <<temp2<< std::endl;

		//printf("CW-TEST: %d %s\n",  F(*accPtrTmp), F(*accPtrTmp));

		//std::cout << "trying  " << std::endl;


        //r = F((*accPtrTmp)/255);///////// changed by Danielle

		//std::cout << "here" << std::endl;

        //r = F(*accPtrTmp)/255;///////// replaced by Danielle with teh thing above

		//std::cout << "f = ";
		//std::cout << (float) f;
		//std::cout << " r = " << (float) r << std::endl;

        a = f + r;
		//std::cout << "just about to copy stuff" << std::endl;

        int i = numscalars;
        do
          {
          i--;
		  //std::cout << "going to copy stuff" << std::endl;
          vtkUltraRound((f*(*inPtrTmp++) + r*(*outPtrTmp))/a,
                          *outPtrTmp);
		  //std::cout << "copied stuff" << std::endl;
          outPtrTmp++;
          }
        while (i);
        // set accumulation
        *accPtrTmp = 65535;
        *outPtrTmp = 255;
        a *= 255;
        if (a < F(65535)) // don't allow accumulation buffer overflow
          {
//	  cout<<"Overflow"<<endl;
          vtkUltraRound(a, *accPtrTmp);
          }

		  //std::cout << "we are here" << std::endl;

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
          F r = 1 - f;
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
	// if bounds check fails
  return 0;
}                          

//----------------------------------------------------------------------------
// Some helper functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// vtkGetUltraInterpFunc
// Sets interpolate (pointer to a function) to match the current interpolation
// mode
//----------------------------------------------------------------------------
template <class F, class T>
static void vtkGetUltraInterpFunc(vtkFreehandUltrasound2 *self, 
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
// Actually inserts the slice - executes the filter for any type of data.
// Given an input and output region, execute the filter algorithm to fill the
// output from the input - no optimization.
// (this one function is pretty much the be-all and end-all of the
// filter) FOR NO OPTIMIZATION
// TODO figure out the index matrix stuff and how it works - because this fn
// basically just multiples the slice by that
//----------------------------------------------------------------------------
template <class T>
static void vtkFreehandUltrasound2InsertSlice(vtkFreehandUltrasound2 *self, /* the vtkFreehandUltrasound2 object */
                                             vtkImageData *outData,       /* the volume to insert into (the output data) */
					          T *outPtr,                   /* the scalar pointer to the output extent */
                                             unsigned short *accPtr,      /* the scalar pointer to the accumulation */
					                                       /* buffer, if we are compounding */
                                             vtkImageData *inData,        /* the slice (input data) */
					          T *inPtr,                    /* the scalar pointer to the input extent */
                                             int inExt[6],                /* the input extent */
                                             vtkMatrix4x4 *matrix)        /* the index matrix - transformation from (i,j,k) */
                                                                          /* voxel indices in the slice to (i,j,k) voxel */
                                                                          /*  indices in the output*/
  
{

  // local variables
  int numscalars; // number of scalar components in the slice
  int idX, idY, idZ; // these are just counters for for loops
  int inIncX, inIncY, inIncZ; // continuous increments for the slice input data
  int outExt[6], outInc[3], clipExt[6]; // extent, increments and clip extent for the output volume
  vtkFloatingPointType inSpacing[3], inOrigin[3]; // the spacing and origin of the input
  unsigned long target; // TODO never used
  double outPoint[4], inPoint[4]; // the resulting point in the output volume (outPoint) from a point in the input slice
                                  // (inpoint)

  // pointer to a function - declared the function's return type and argument list
  // note that you need parentheses around the pointer's name
  // can later assign it to the address of a matching function and use "(*interpolate)"
  // or "interpolate" as if it were a function name
  int (*interpolate)(double *point, T *inPtr, T *outPtr,
		     unsigned short *accPtr,
                     int numscalars, int outExt[6], int outInc[3]);
  
  // slice spacing and origin
  inData->GetSpacing(inSpacing);
  inData->GetOrigin(inOrigin);
  // number of pixels in the x and y directions b/w the fan origin and the slice origin
  double xf = (self->GetFanOrigin()[0]-inOrigin[0])/inSpacing[0];
  double yf = (self->GetFanOrigin()[1]-inOrigin[1])/inSpacing[1]; 
  // fan depth squared 
  double d2 = self->GetFanDepth()*self->GetFanDepth();
  // absolute value of slice spacing
  double xs = fabs((double)(inSpacing[0]));
  double ys = fabs((double)(inSpacing[1]));
  // tan of the left and right fan angles
  double ml = tan(self->GetFanAngles()[0]*vtkMath::DoubleDegreesToRadians())/xs*ys;
  double mr = tan(self->GetFanAngles()[1]*vtkMath::DoubleDegreesToRadians())/xs*ys;
  // the tan of the right fan angle is always greater than the left one
  if (ml > mr)
    {
    double tmp = ml; ml = mr; mr = tmp;
    }
  // get the clip rectangle as an extent
  self->GetClipExtent(clipExt, inOrigin, inSpacing, inExt);

  // find maximum output range = output extent
  outData->GetExtent(outExt);
  
  //TODO target is never used
  target = (unsigned long)
    ((inExt[5]-inExt[4]+1)*(inExt[3]-inExt[2]+1)/50.0);
  target++;
  
  // Get increments to march through data - ex move from the end of one x scanline of data to the
  // start of the next line
  outData->GetIncrements(outInc);
  inData->GetContinuousIncrements(inExt, inIncX, inIncY, inIncZ);
  numscalars = inData->GetNumberOfScalarComponents();
  
  // Set interpolation method - nearest neighbor or trilinear
  vtkGetUltraInterpFunc(self,&interpolate);

  // Loop through  slice pixels in the input extent and put them into the output volume
  for (idZ = inExt[4]; idZ <= inExt[5]; idZ++) // for z0 to z1
    {
      for (idY = inExt[2]; idY <= inExt[3]; idY++) // for y0 to y1
      {
	for (idX = inExt[0]; idX <= inExt[1]; idX++) // for x0 to x1
        {

	  // if we are within the current clip extent
        if (idX >= clipExt[0] && idX <= clipExt[1] && 
	    idY >= clipExt[2] && idY <= clipExt[3])
          {
	    // current x/y index minus num pixels in the x/y direction b/w the fan origin and the slice origin
	    double x = (idX-xf);
          double y = (idY-yf);

	  // if we are within the fan
          if (((ml == 0 && mr == 0) || y > 0 &&
              ((x*x)*(xs*xs)+(y*y)*(ys*ys) < d2 && x/y >= ml && x/y <= mr)))
            {  
            inPoint[0] = idX;
            inPoint[1] = idY;
            inPoint[2] = idZ;
            inPoint[3] = 1;

	    //recall matrix = the index matrix --> transform voxels in the slice to indices in the output
            matrix->MultiplyPoint(inPoint,outPoint); // apply transform
            
	    // deal with w (homogeneous transform) if the transform was a perspective transform
            outPoint[0] /= outPoint[3]; 
            outPoint[1] /= outPoint[3]; 
            outPoint[2] /= outPoint[3];
            outPoint[3] = 1;
        
	    // interpolation functions return 1 if the interpolation was successful, 0 otherwise
            int hit = interpolate(outPoint, inPtr, outPtr, accPtr, numscalars, 
                        outExt, outInc);
	    //    self->SetPixelCount( self->GetPixelCount() + hit);
	    // increment the number of pixels inserted
	    self->IncrementPixelCount(0, hit);
	    
            } // end if
          } // end if

        inPtr += numscalars; 
        } // end for x
      inPtr += inIncY;
      } // end for y
    inPtr += inIncZ;
    } // end for z
}


//----------------------------------------------------------------------------
// InsertSlice
// Given an input and output region, execute the filter algorithm to fill the
// output from the input - no optimization.
// It just executes a switch statement to call the
// vtkFreehandUltrasound2InsertSlice method
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::InsertSlice()
{

	//std::cout << "In InsertSlice()" << std::endl;

  // if we are optimizing by either splitting into x, y, z components or with
  // integer math, then run the optimized insert slice function instead
  if (this->GetOptimization())
    {
    this->OptimizedInsertSlice();
    return;
    }

  // if we are not reconstructing at the moment, then get the output ready to
  // receive data
  if (this->ReconstructionThreadId == -1)
    {
      // TODO note that I think calls to internalexecuteinformation should be replaced
      // with calls to requestinformation
    this->InternalExecuteInformation();
    }

  // if we need to clear, then clear
  if (this->NeedsClear)
    {
    this->InternalClearOutput();
    }

  // get the slice, output data, accumulation buffer, whole extent == input extent (from the
  // slice) and output extent (from this object)
  vtkImageData *inData = this->GetSlice();
  vtkImageData *outData = this->GetOutput();
  vtkImageData *accData = this->AccumulationBuffer;
  int *inExt = inData->GetWholeExtent();
  int *outExt = this->OutputExtent;

  // fprintf(stderr,"inExt %d %d %d %d %d %d\n",inExt[0],inExt[1],inExt[2],inExt[3],inExt[4],inExt[5]);

  // need the entire input slice to generate the requested output extent, so set the
  // update extent of the input to equal the whole extent of the input
  inData->SetUpdateExtentToWholeExtent();//(inExt);
  // get the updated slice information
  inData->Update();
  // get the native pointer for the scalar data
  //void *inPtr = inData->GetScalarPointerForExtent(inData->GetExtent()); changed by Danielle
  void *inPtr = inData->GetScalarPointerForExtent(inExt);
  void *outPtr = outData->GetScalarPointerForExtent(outExt);
  void *accPtr = NULL;
  
  // if we are compounding, then we have a native pointer for the extent of the
  // accumulation buffer
  if (this->Compounding)
    {
    accPtr = accData->GetScalarPointerForExtent(outExt);

	/*accData->SetUpdateExtentToWholeExtent(); // new by Danielle
	accData->Update();*/
    }
  else
    {
    accPtr = NULL;
    }

  // print out the input and output data
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

  // TODO what does this comment mean?  Did Pieli mean to do this?
  // change transform matrix so that instead of taking 
  // input coords -> output coords it takes output indices -> input indices
  vtkMatrix4x4 *matrix = this->GetIndexMatrix();

  // Never fully implemented, so comment it out 
  //if (this->LastIndexMatrix && 
  //    this->CalculateMaxSliceSeparation(matrix,this->LastIndexMatrix) < 1.0)
  //  {
  //  return;
  //  }

  // the LastIndexMatrix is a copy of the most recent index matrix used
  if (this->LastIndexMatrix == 0)
    {
    this->LastIndexMatrix = vtkMatrix4x4::New();
    }
  this->LastIndexMatrix->DeepCopy(matrix);
  
  // Call the vtkFreehandUltrasound2InsertSlice method to actually insert the slice
  switch (inData->GetScalarType())
    {
    case VTK_SHORT:
      vtkFreehandUltrasound2InsertSlice(this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), 
                             inData, (short *)(inPtr), 
                             inExt, matrix);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasound2InsertSlice(this,outData,(unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), 
                             inData, (unsigned short *)(inPtr), 
                             inExt, matrix);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasound2InsertSlice(this, outData,(unsigned char *)(outPtr),
                             (unsigned short *)(accPtr), 
                             inData, (unsigned char *)(inPtr), 
                             inExt, matrix);
      break;
    default:
      vtkErrorMacro(<< "InsertSlice: Unknown input ScalarType");
      return;
    }

  // this->Modified();
}

//----------------------------------------------------------------------------
template <class T>
static void vtkFreehandUltrasound2FillHolesInOutput(vtkFreehandUltrasound2 *self,
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
void vtkFreehandUltrasound2::ThreadedFillExecute(vtkImageData *outData,	
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
      vtkFreehandUltrasound2FillHolesInOutput(
                             this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasound2FillHolesInOutput(
                             this, outData, (unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasound2FillHolesInOutput(
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
VTK_THREAD_RETURN_TYPE vtkFreehand2ThreadedFillExecute( void *arg )
{
  vtkFreehand2ThreadStruct *str;
  int ext[6], splitExt[6], total;
  int threadId, threadCount;
  vtkImageData *output;

  threadId = ((ThreadInfoStruct *)(arg))->ThreadID;
  threadCount = ((ThreadInfoStruct *)(arg))->NumberOfThreads;

  str = (vtkFreehand2ThreadStruct *)(((ThreadInfoStruct *)(arg))->UserData);
  output = str->Output;
  output->GetExtent( ext );

  // execute the actual method with appropriate extent
  // first find out how many pieces extent can be split into.
  total = str->Filter->SplitSliceExtent(splitExt, ext, threadId, threadCount);
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

void vtkFreehandUltrasound2::MultiThreadFill(vtkImageData *outData)
{
  vtkFreehand2ThreadStruct str;
  
  str.Filter = this;
  str.Input = 0;
  str.Output = outData;
  
  this->Threader->SetNumberOfThreads(this->NumberOfThreads);
  
  // setup threading and the invoke threadedExecute
  this->Threader->SetSingleMethod(vtkFreehand2ThreadedFillExecute, &str);
  this->Threader->SingleMethodExecute();
}

//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::FillHolesInOutput()
{
  // TODO note that I think calls to internalexecuteinformation should be
  // replaced with calls to requestinformation
//  this->InternalExecuteInformation();
  // this->GetOutput()->Update();
  // TODO should replace this with requestinformation for vtk 5
  this->UpdateInformation();
  if (this->NeedsClear)
    {
    //this->UpdateInformation();
    this->InternalClearOutput();
    }

  vtkImageData *outData = this->GetOutput();
  this->MultiThreadFill(outData);

  this->Modified(); 
}

//----------------------------------------------------------------------------

/*
void vtkFreehandUltrasound2::FillHolesInOutput()
{
// TODO note that i think calls to internalexecuteinformation should be replaced
// with calls to RequestInformation
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
      vtkFreehandUltrasound2FillHolesInOutput(
                             this, outData, (short *)(outPtr), 
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_SHORT:
      vtkFreehandUltrasound2FillHolesInOutput(
                             this, outData, (unsigned short *)(outPtr),
                             (unsigned short *)(accPtr), outExt);
      break;
    case VTK_UNSIGNED_CHAR:
      vtkFreehandUltrasound2FillHolesInOutput(
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
// ClearOutput
// Setup to clear the data volume, then call InternalClearOutput
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::ClearOutput()
{
  this->NeedsClear = 1;

  // modified by Danielle
  //std::cout << "in clear output" << std::endl;

  // if we are not currently reconstructing
  if (this->ReconstructionThreadId == -1)
    {

		// modified by Danielle
		//std::cout << "going to clear output" << std::endl;


    // TODO for vtk 5, should replace updateinformation with requestInformation
    // Actually, do we?  Do we have to do anything with the information objects,
      //except perhaps change the update time?
    
		// Modified by Danielle based on David's suggestion
		this->GetOutput()->UpdateInformation();
		//this->UpdateInformation();


    this->InternalClearOutput();
    }

  // inherited from vtkImageAlgorithm - update the modification time
  this->Modified();
}

//----------------------------------------------------------------------------
// InternalClearOutput
// Actually clear the data volume
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::InternalClearOutput()
{
	//std::cout << "In InternalClearOutput" << std::endl;

  int *outExtent = this->OutputExtent;

  this->NeedsClear = 0;

  //vtkImageData *outData = this->GetOutput(); //TODO should be specifying a port?
  //int numScalars = outData->GetNumberOfScalarComponents();

  this->GetOutput()->SetExtent(outExtent);
	this->GetOutput()->AllocateScalars();

  void *outPtr = this->GetOutput()->GetScalarPointerForExtent(outExtent);


  // added by danielle
/*	memset(outPtr,255,((outExtent[1]-outExtent[0]+1)*
	   (outExtent[3]-outExtent[2]+1)*
	   (outExtent[5]-outExtent[4])+1)*
	   this->GetOutput()->GetScalarSize());*/

  //std::cout << "number scalar components" << this->GetOutput()->GetNumberOfScalarComponents() << std::endl;

		/*memset(outPtr,255,((outExtent[1]-outExtent[0]+1)*
	   (outExtent[3]-outExtent[2]+1)*
	   (outExtent[5]-outExtent[4])+1)*
	   this->GetOutput()->GetScalarSize()*2 + ((640*480+(640*2))*2));*/

	// fixed by Danielle to add number of scalar components - previously this was clearing only half of the volume
		memset(outPtr,0,((outExtent[1]-outExtent[0]+1)*
	   (outExtent[3]-outExtent[2]+1)*
	   (outExtent[5]-outExtent[4]+1)*
	   this->GetOutput()->GetScalarSize()*this->GetOutput()->GetNumberOfScalarComponents()));




    // accPtr is a void pointer - pointers that point to a value that has no type, so we
  // always have to cast the address in the void pointer to some other pointer type that
  // points to a concrete data type before dereferencing it (taking the pointer to the value
  // and returning the value)
		
		
	if (this->Compounding)
    {


		//std::cout << "we are trying thisssssssssssssssssssssss 3" << std::endl;

    this->AccumulationBuffer->SetExtent(outExtent);

	//std::cout << "we set the extent" << std::endl;

    this->AccumulationBuffer->AllocateScalars();

	//std::cout << "we allocated scalars" << std::endl;

    // GetScalarPointerForExtent returns the pointer for the scalar data
    void *accPtr = this->AccumulationBuffer->GetScalarPointerForExtent(outExtent);

	//std::cout << "we got scalar pointers for the extent" << std::endl;

    // memset is a fast way to copy characters into a buffer: memset(buffer, ch, count)
    // copies ch into the first count characters of buffer
    // so this copies 0 into the first *crazy number* characters of accPtr

   /* memset(accPtr,0,(outExtent[1]-outExtent[0]+1)*
	   (outExtent[3]-outExtent[2]+1)*
	   (outExtent[5]-outExtent[4]+1)*
	   this->AccumulationBuffer->GetScalarSize());*/

	//std::cout << "number of scalar components in the accumulation buffer = " << this->AccumulationBuffer->GetNumberOfScalarComponents() << std::endl;

	// changed by Danielle to incorporate the number of scalar components
	memset(accPtr,0,((outExtent[1]-outExtent[0]+1)*
	   (outExtent[3]-outExtent[2]+1)*
	   (outExtent[5]-outExtent[4]+1)*
	   this->AccumulationBuffer->GetScalarSize()*this->AccumulationBuffer->GetNumberOfScalarComponents()));



	//std::cout << " we memset the accumulation buffer" << std::endl;
	}

	//std::cout << "we are finished thisssssssssssssssssssss 3" << std::endl;

  if (this->LastIndexMatrix)
    {
    this->LastIndexMatrix->Delete();
    this->LastIndexMatrix = NULL;
    }

  this->SetPixelCount(0,0);
  this->SetPixelCount(1,0);
  this->SetPixelCount(2,0);
  this->SetPixelCount(3,0);
  this->NeedsClear = 0;
}

//----------------------------------------------------------------------------
// vtkIsIdentityMatrix
// check a matrix to see whether it is the identity matrix
//----------------------------------------------------------------------------
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

vtkMatrix4x4 *vtkFreehandUltrasound2::GetIndexMatrix()
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
		// cutting this out prevents the transform from being applied
    transform->SetMatrix(this->GetSliceAxes());
    }

  //std::cout << "***************" << std::endl;
  //std::cout << "SLICE AXES:" << std::endl;
  //transform->GetMatrix()->Print(std::cout);

  if (this->SliceTransform)
    {
    transform->PostMultiply();
    transform->Concatenate(this->SliceTransform->GetMatrix());
    //std::cout << "SLICE TRANSFORM: " << std::endl;
    //this->SliceTransform->GetMatrix()->Print(std::cout);
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
  //std::cout << "IN MATRIX" << std::endl;
  //inMatrix->Print(std::cout);
  //std::cout << "OUT MATRIX" << std::endl;
  //outMatrix->Print(std::cout);

	// outMatrix * (sliceTransform * sliceAxes) * inMatrix
  if (!isIdentity)
    {
    transform->PostMultiply();
    transform->Concatenate(outMatrix);
    transform->PreMultiply();
    transform->Concatenate(inMatrix);
    }

// this->IndexMatrix is the result
  transform->GetMatrix(this->IndexMatrix);
  //std::cout << "INDEX MATRIX" << std::endl;
  //this->IndexMatrix->Print(std::cout);
  
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
static inline void vtkFreehand2OptimizedNNHelper(int r1, int r2,
                                                double *outPoint,
                                                double *outPoint1,
												double *xAxis,
                                                T *&inPtr, T *outPtr,
                                                int *outExt, int *outInc,
                                                int numscalars, 
                                                unsigned short *accPtr)
{
	

  if (accPtr)  // Nearest-Neighbor, no extent checks, we are compounding
    {

		//std::cout << "here!!!!" << std::endl;
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
	  //std::cout << "going to make accPtr1" << std::endl;
      unsigned short *accPtr1 = accPtr + ((unsigned short)(inc/outInc[0])); /////  addresss of the corresponding point in the accumulation buffer
	  //std::cout << "going to make newa" << std::endl;     
	  unsigned short newa = *accPtr1 + ((unsigned short)(255)); //////////// intensity of the corresponding accumulation point + 255
	 // std::cout << "ok" << std::endl;
	  int i = numscalars;
      do 
        {
        i--;
		// intensity of the point in the output, if there was no compounding this would be *outPtr1++ = *inPtr++;
        *outPtr1 = ((*inPtr++)*255 + (*outPtr1)*(*accPtr1))/newa;//////////// intensity of the point in the output
		outPtr1++;/////////
        }
      while (i);

	  		//std::cout << "made outPtr1" << std::endl;
      *outPtr1 = 255;
      *accPtr1 = 65535;////////////// intensity of the corresponding point in the accumulation buffer
      if (newa < 65535)/////
        {
        *accPtr1 = newa;//////////// change the intensity of the corresponding point in the accumulation buffer
        }
		//std::cout << "made accptr1" << std::endl;
      }
    }
  else
    {  // Nearest-Neighbor, no extent checks, no accumulation
    for (int idX = r1; idX <= r2; idX++)
      {
      outPoint[0] = outPoint1[0] + idX*xAxis[0]; 
      outPoint[1] = outPoint1[1] + idX*xAxis[1];
      outPoint[2] = outPoint1[2] + idX*xAxis[2];

	  // difference between the nearest pixel and the start of the output extent
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
	  // the increment needed to get from the start of the pointer to the output extent to the nearest pixel
      int inc = outIdX*outInc[0] + outIdY*outInc[1] + outIdZ*outInc[2];
      T *outPtr1 = outPtr + inc; // the output pixel in terms of increments
      int i = numscalars;
      do
        {
        i--;
		// copy the input pointer value into the output pointer (this is where the intensities get copied)
		*outPtr1++ = *inPtr++;
		}
      while (i);
      *outPtr1 = 255;
      }
    } 
}

// specifically optimized for fixed-point (i.e. integer) mathematics
template <class T>
static inline void vtkFreehand2OptimizedNNHelper(int r1, int r2,
                                                fixed *outPoint,
                                                fixed *outPoint1, fixed *xAxis,
                                                T *&inPtr, T *outPtr,
                                                int *outExt, int *outInc,
                                                int numscalars, 
                                                unsigned short *accPtr)
{
  outPoint[0] = outPoint1[0] + r1*xAxis[0] - outExt[0]; // outPoint changes below, so this is not constant
  outPoint[1] = outPoint1[1] + r1*xAxis[1] - outExt[2];
  outPoint[2] = outPoint1[2] + r1*xAxis[2] - outExt[4];

  if (accPtr)  // Nearest-Neighbor, no extent checks
    {

	//	std::cout << "here!!!!!!!!!!!!" << std::endl; // danielle for debugging


    for (int idX = r1; idX <= r2; idX++)
      {
      int outIdX = vtkUltraRound(outPoint[0]); // outpoint changes below, so this is not constant
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
	  //std::cout << "making accPtr1" << std::endl;
      unsigned short *accPtr1 = accPtr + ((unsigned short)(inc/outInc[0])); // divide by outInc[0] to accomodate for the difference
														// in the number of scalar pointers between the output
														// and the accumulation buffer
	 // std::cout << "making newa" << std::endl;

	
	  //std::cout << (int)outPtr1 << " " << (int)(*outPtr1) << " " << (int)accPtr1 << " " << inc << " " << outInc[0] << " " << << std::endl;
	  //std::cout << "finished that!" << std::endl;

	/*
	- outPtr1 = 973417532
	- *outPtr1 = 0
	- accPtr1 = 1497181244
	- inc = 199765020
	- outInc[2] = 2
	*/


	  //std::cout << (int)(*accPtr1) << std::endl;
	  //std::cout << "finished second part" << std::endl;


      unsigned short newa = *accPtr1 + ((unsigned short)(255)); // dies here!
	 // std::cout << "made newa" << std::endl;
      int i = numscalars;
      do 
        {
        i--;
        *outPtr1 = ((*inPtr++)*255 + (*outPtr1)*(*accPtr1))/newa;
        outPtr1++;
		
		//std::cout << *accPtr1 << std::endl; // new by danielle for debugging

        }
      while (i);
	  //std::cout << "fnished the loop" << std::endl;
      *outPtr1 = 255;
      *accPtr1 = 65535;
      if (newa < 65535)
        {
        *accPtr1 = newa;
        }
	//	std::cout << "finished the if"<< std::endl;
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

//----------------------------------------------------------------------------
// vtkOptimizedInsertSlice
// Actually inserts the slice, with optimization.
//----------------------------------------------------------------------------
template <class F, class T>
static void vtkOptimizedInsertSlice(vtkFreehandUltrasound2 *self, // the freehand us
									vtkImageData *outData, // the output volume
									T *outPtr, // scalar pointer to the output volume over the output extent
									unsigned short *accPtr, // scalar pointer to the accumulation buffer over the output extent
									vtkImageData *inData, // input slice
									T *inPtr, // scalar pointer to the input volume over the input slice extent
									int inExt[6], // input slice extent (could have been split for threading)
									F matrix[4][4], // index matrix, output indices -> input indices
									int threadId) // current thread id
{
	

  /*for (int u = 0; u < 4; u++)
    {
    std::cout << (double)matrix[u][0] << " " << (double)matrix[u][1] << " " << (double)matrix[u][2] << " " << (double)matrix[u][3] << std::endl;
    }
  std::cout << "*****" << std::endl;*/

	//std::cout << "In vtkOptimizedInsertSlice" << std::endl;
 

	// added by Danielle
	int prevPixelCount = self->GetPixelCount();


	// local variables
	int id = 0;
	int i, numscalars; // numscalars = number of scalar components in the input image
	int idX, idY, idZ; // the x, y, and z pixel of the input image
	int inIncX, inIncY, inIncZ; // increments for the input extent
	int outExt[6]; // output extent
	int outMax[3], outMin[3]; // the max and min values of the output extents -
	// if outextent = (x0, x1, y0, y1, z0, z1), then
	// outMax = (x1, y1, z1) and outMin = (x0, y0, z0)
	int outInc[3]; // increments for the output extent
	int clipExt[6];
	unsigned long count = 0;
	unsigned long target;
	int r1,r2;
	// outPoint0, outPoint1, outPoint is a fancy way of incremetally multiplying the input point by
	// the index matrix to get the output point... =)  Outpoint is the result
	F outPoint0[3]; // temp, see above
	F outPoint1[3]; // temp, see above
	F outPoint[3]; // this is the final output point, created using Output0 and Output1
	F xAxis[3], yAxis[3], zAxis[3], origin[3]; // the index matrix (transform), broken up into axes and an origin
	vtkFloatingPointType inSpacing[3],inOrigin[3]; // input spacing and origin

	// input spacing and origin
	inData->GetSpacing(inSpacing);
	inData->GetOrigin(inOrigin);



	// number of pixels in the x and y directions b/w the fan origin and the slice origin
	double xf = (self->GetFanOrigin()[0]-inOrigin[0])/inSpacing[0];
	double yf = (self->GetFanOrigin()[1]-inOrigin[1])/inSpacing[1];

			if (self->GetFlipHorizontalOnOutput())
			{
      yf = (double)self->GetNumberOfPixelsFromTipOfFanToBottomOfScreen();
      }


	// fan depth squared
	double d2 = self->GetFanDepth()*self->GetFanDepth();
	// input spacing in the x and y directions
	// TODO in vtkFreehandUltrasound2insertslice they take the fabs here
	double xs = inSpacing[0];
	double ys = inSpacing[1];
	//tan of the left and right fan angles
	double ml = tan(self->GetFanAngles()[0]*vtkMath::DoubleDegreesToRadians())/xs*ys;
	double mr = tan(self->GetFanAngles()[1]*vtkMath::DoubleDegreesToRadians())/xs*ys;
	// the tan of the right fan angle is always greater than the left one
	if (ml > mr)
	{
		double tmp = ml; ml = mr; mr = tmp;
	}


	//std::cout << "going to get the clip extent" << std::endl;

	// get the clip rectangle as an extent
	self->GetClipExtent(clipExt, inOrigin, inSpacing, inExt);

	// find maximum output range
	outData->GetExtent(outExt);

	// added by Danielle
	 //std::cout << "output extent:  " << outExt[0] << " " << outExt[1] << " " << outExt[2] << " " << outExt[3] << " " << outExt[4] << " " << outExt[5] << std::endl;



	for (i = 0; i < 3; i++)
	{
		outMin[i] = outExt[2*i];
		outMax[i] = outExt[2*i+1];
	}

	target = (unsigned long)
		((inExt[5]-inExt[4]+1)*(inExt[3]-inExt[2]+1)/50.0);
	target++;

	//std::cout << "going to start getting the whole extent of the output " << std::endl;

	int wExtent[6]; // output whole extent
	outData->GetWholeExtent(wExtent);

	// Get Increments to march through data - move from one pixel to the other (remember that we are not
	// necessarily moving trough the whole image!  We want to be able to easily move through the subset of the extents
	
	//std::cout << "going to get the increments of the output" << std::endl;

	outData->GetIncrements(outInc);
	
	/*std::cout << "output increments: " << outInc[0] << " " << outInc[1] << " " << outInc[2] << std::endl;
	std::cout << "output number of scalar components " << outData->GetNumberOfScalarComponents() << std::endl;
	int tempInc[3];
	self->GetAccumulationBuffer()->GetIncrements(tempInc);
	std::cout << "accumulation buffer increments " << tempInc[0] << " " << tempInc[1] << " " << tempInc[2] << std::endl;
	std::cout << "accumulation buffer number of scalar components " << self->GetAccumulationBuffer()->GetNumberOfScalarComponents() << std::endl;
*/


	inData->GetContinuousIncrements(inExt, inIncX, inIncY, inIncZ); // x increment is zero
	// y increment tells you how to move from the end of one lineto the start of the next one
	// z increment tells you how to move between images
	numscalars = inData->GetNumberOfScalarComponents(); // number of scalar components within each "pixel"

	// break matrix into a set of axes plus an origin
	// (this allows us to calculate the transform Incrementally)
	for (i = 0; i < 3; i++)
	{
		xAxis[i]  = matrix[i][0];   // remember that the matrix is the indexMatrix, and transforms
		yAxis[i]  = matrix[i][1];	// output pixels to input pixels
		zAxis[i]  = matrix[i][2];
		origin[i] = matrix[i][3];
	}


    static int firstFrame = 1;
    ofstream outStream;
    outStream.open("lines.txt");

	//std::cout << "going to start looping through input pixels"<< std::endl;


				//TODO take me out
			/*static int firstTime = 0;
			ofstream outStream;
			if (self->GetFlipHorizontalOnOutput())
			{
			outStream.open("firstFrame_flip.txt");
			}
			else
			{
			outStream.open("firstFrame_noFlip.txt");
			}*/

	// Loop through INPUT pixels - remember this is a 3D cube represented by the input extent
	for (idZ = inExt[4]; idZ <= inExt[5]; idZ++) // for each image...
	{
		//std::cout << "Z = " << idZ<< std::endl;

		outPoint0[0] = origin[0]+idZ*zAxis[0]; // incremental transform
		outPoint0[1] = origin[1]+idZ*zAxis[1];
		outPoint0[2] = origin[2]+idZ*zAxis[2];

		for (idY = inExt[2]; idY <= inExt[3]; idY++) // for each horizontal line in the image...
		{

			//TODO implement this for other options
			
			if (self->GetFlipHorizontalOnOutput())
			{
      int dist = self->GetNumberOfPixelsFromTipOfFanToBottomOfScreen();
			outPoint1[0] = outPoint0[0]+(dist-idY)*yAxis[0]; // incremental transform
			outPoint1[1] = outPoint0[1]+(dist-idY)*yAxis[1];
			outPoint1[2] = outPoint0[2]+(dist-idY)*yAxis[2];
			}
			else
			{
			outPoint1[0] = outPoint0[0]+idY*yAxis[0]; // incremental transform
			outPoint1[1] = outPoint0[1]+idY*yAxis[1];
			outPoint1[2] = outPoint0[2]+idY*yAxis[2];
			}

			if (!id)
			{
				if (!(count%target)) 
				{
					self->UpdateProgress(count/(50.0*target));  // progress between 0 and 1
				}
				count++;
			}

			// find intersections of x raster line with the output extent
			

      // this only changes r1 and r2
			vtkUltraFindExtent(r1,r2,outPoint1,xAxis,outMin,outMax,inExt);

      //std::cout << "r1 = " << r1 << ", r2 = " << r2 << std::endl;

			/*if (self->GetFlipHorizontalOnOutput())
			{
      int dist = self->GetNumberOfPixelsFromTipOfFanToBottomOfScreen();
			outPoint1[0] = outPoint0[0]+(dist-idY)*yAxis[0]; // incremental transform
			outPoint1[1] = outPoint0[1]+(dist-idY)*yAxis[1];
			outPoint1[2] = outPoint0[2]+(dist-idY)*yAxis[2];
			}*/

			// next, handle the 'fan' shape of the input
			double y = (yf - idY);
      //std::cout << "yf = " << yf << ", y = " << y << std::endl;
			if (ys < 0)
			{
				y = -y;
			}

      // first, check the angle range of the fan - choose r1 and r2 based
      // on the triangle that the fan makes from the fan origin to the bottom
      // line of the video image
			if (!(ml == 0 && mr == 0))
			{
				// equivalent to: r1 < vtkUltraCeil(ml*y + xf + 1)
        // this is what the radius would be based on tan(fanAngle)
				if (r1 < -vtkUltraFloor(-(ml*y + xf + 1)))
				{
					r1 = -vtkUltraFloor(-(ml*y + xf + 1));
				}
				if (r2 > vtkUltraFloor(mr*y + xf - 1))
				{
					r2 = vtkUltraFloor(mr*y + xf - 1);
				}

        //if (firstFrame) {outStream << r1 << " " << r2 << " ";};

				// next, check the radius of the fan - crop the triangle to the fan
        // depth
				double dx = (d2 - (y*y)*(ys*ys))/(xs*xs);

        // if we are outside the fan's radius, ex at the bottom lines
				if (dx < 0)
				{
					r1 = inExt[0];
					r2 = inExt[0]-1;
				}
        // if we are within the fan's radius, we have to adjust if we are in
        // the "ellipsoidal" (bottom) part of the fan instead of the top
        // "triangular" part
				else
				{
					dx = sqrt(dx);
          // this is what r1 would be if we calculated it based on the
          // pythagorean theorem
					if (r1 < -vtkUltraFloor(-(xf - dx + 1)))
					{
						r1 = -vtkUltraFloor(-(xf - dx + 1));
					}
					if (r2 > vtkUltraFloor(xf + dx - 1))
					{
						r2 = vtkUltraFloor(xf + dx - 1);
					}
				}
				//	  cout<< "Found Extent: ( "<< r1 << ", " << r2 <<" )"
				//		  << " Radius Of the Fan: "<<dx<<" \n "
				//		  << " clipExt: (" << clipExt[0] << ", "<< clipExt[1] 
				//		  << ", " << clipExt[2]<< ", " << clipExt[3] << endl;
			}



      //std::cout << "AFTER HANDLING THE FAN SHAPE: r1 = " << r1 << ", r2 = " << r2 << std::endl;


			//cout<< "Found Extent: ( "<< r1 << ", " << r2 <<" )"
			//	  << " clipExt: (" << ", " << clipExt[2]<< ", " << clipExt[3] << endl;
			// bound to the ultrasound clip rectangle
			if (r1 < clipExt[0])
			{
				r1 = clipExt[0];
			}
			if (r2 > clipExt[1])
			{
				r2 = clipExt[1];
			}

    //std::cout << "AFTER CLIPPING: r1 = " << r1 << ", r2 = " << r2 << std::endl;

			if (r1 > r2  )//|| idY < clipExt[2] || idY > clipExt[3]) 
			{
				r1 = inExt[0];
				r2 = inExt[0]-1;
			}
			else
			{
				/* // removed by danielle because this output is annoying
				if(!( idY < clipExt[2] || idY > clipExt[3]))
				{
				cout<< "idY: "<<idY<<endl;
				}*/
			}

			// skip the portion of the slice to the left of the fan
			for (idX = inExt[0]; idX < r1; idX++)
			{
				inPtr += numscalars;
			}

			/*if (self->GetFlipHorizontalOnOutput())
			{

			vtkFloatingPointType outSpacing[3];
			vtkFloatingPointType videoOrigin[3];
			vtkFloatingPointType fanOrigin[2];
			self->GetOutputSpacing(outSpacing);
			self->GetVideoSource()->GetDataOrigin(videoOrigin);
			self->GetFanOrigin(fanOrigin);

			int dist = (videoOrigin[1]-fanOrigin[1])/outSpacing[1]*-1.0;
			//std::cout << dist << std::endl;
			outPoint1[0] = outPoint0[0]+((dist-idY)+dist)*yAxis[0]; // incremental transform
			outPoint1[1] = outPoint0[1]+((dist-idY)+dist)*yAxis[1];
			outPoint1[2] = outPoint0[2]+((dist-idY)+dist)*yAxis[2];
			}*/

			//TODO remove me
			/*static int counter = 0;
			static int yFrig;
			yFrig = outPoint1[1];
			if (!(r1 == 0 && r2 == -1))
			{
				std::cout << yFrig << " " << r1 << " " << r2 << std::endl;
				counter++;
			}*/

			/*(if (firstTime)
			{
				std::cout << idY << " " << r1 << " " << r2 << "          ";
			}*/

			// we want to insert the stuff within the fan
			// REMEMBER THAT MULTIPLYING THE INPUT POINT BY THE TRANSFORM WILL GIVE YOU FRACTIONAL PIXELS!!!! EWWWWW
			// interpolation stuff if we are interpolating linearly (code 1)
			if (self->GetInterpolationMode() == VTK_FREEHAND_LINEAR)
			{ 
      
      //std::cout << "BEFORE RESETTING: r1 = " << r1 << ", r2 = " << r2 << std::endl;

        //TODO take out
        //r1 = inExt[0];
        //r2 = inExt[1];

				for (idX = r1; idX <= r2; idX++) // for all of the x pixels within the fan
				{

					//std::cout << "X = " << idX << std::endl;

					//TODO implement this for other options
					if (self->GetFlipVerticalOnOutput())
					{
					outPoint[0] = outPoint1[0] + (r1+r2-idX)*xAxis[0];
					outPoint[1] = outPoint1[1] + (r1+r2-idX)*xAxis[1];
					outPoint[2] = outPoint1[2] + (r1+r2-idX)*xAxis[2];
					}
					else
					{
					outPoint[0] = outPoint1[0] + idX*xAxis[0];
					outPoint[1] = outPoint1[1] + idX*xAxis[1];
					outPoint[2] = outPoint1[2] + idX*xAxis[2];
					}

/*			if (firstTime == 0 && !(r1 == 0 && r2 == -1))
			{
				int out0 = outPoint[0];
				int out1 = outPoint[1];
				int out2 = outPoint[2];
				outStream << out0 << " " << out1 << " " << out2 << std::endl;
        //std::cout << out0 << " " << out1 << " " << out2 << std::endl;
			}*/



					//std::cout << "going to trilinear interpolation" << std::endl;

				//std::cout << std::endl << "printing accPtr!" << std::endl;

				//std::cout << (int)(*accPtr) << std::endl;

				//std::cout << "printing accPtr + 1!" << std::endl;
				//std::cout << (int) (*(accPtr+1)) << std::endl;



					int hit = vtkTrilinearInterpolation(outPoint, inPtr, outPtr, accPtr, 
						numscalars, outExt, outInc); // hit is either 1 or 0

					//std::cout << "finished trilinear interpolation" << std::endl;

					inPtr += numscalars; // go to the next x pixel
					//self->PixelCount += hit;
					self->IncrementPixelCount(threadId, hit);
				}

			}

			// interpolation stuff if we are interpolating with nearest neighbor (code 0)
			else 
			{

				//std::cout << "going to freehand optimized nn helper" << std::endl;
				vtkFreehand2OptimizedNNHelper(r1, r2, outPoint, outPoint1, xAxis, 
					inPtr, outPtr, outExt, outInc,
					numscalars, accPtr);
				// self->PixelCount += r2-r1+1;
				self->IncrementPixelCount(threadId, r2-r1+1); // we added all the pixels between r1 and r2,
																// so increment our count of the number of pixels added
				//cout<<" NN: Inserted pixels: "<<r2-r1+1<<endl;
			}

			// skip the portion of the slice to the right of the fan
			for (idX = r2+1; idX <= inExt[1]; idX++)
			{
				inPtr += numscalars;
			}

			inPtr += inIncY; // move to the next line
		}
		inPtr += inIncZ; // move to the next image
	}

	//TODO take me out
	/*if (firstTime == 0)
	{
		firstTime = 1;
		outStream.close();
		std::cout << "FIRST FRAME FINISHED" << std::endl;
	}*/

	//cout<<"Pixels inserted: "<<self->GetPixelCount()<<endl;
	//cout<< "-->Pixels added:  " << self->GetPixelCount() - prevPixelCount << endl; // added by Danielle

}

//----------------------------------------------------------------------------
// vtkFreehand2ThreadedExecute
// this mess is really a simple function. All it does is call
// the ThreadedSliceExecute method after setting the correct
// extent for this thread. Its just a pain to calculate
// the correct extent.
//----------------------------------------------------------------------------
VTK_THREAD_RETURN_TYPE vtkFreehand2ThreadedExecute( void *arg )
{


  vtkFreehand2ThreadStruct *str; // contains the filter, input and output
  int ext[6], splitExt[6], total; // the input slice extent, the input slice extent
                                  // for this thread, and the total number of pieces
                                  // the extent can be split into (i.e. the number of
                                  // threads we should use)
  int threadId, threadCount; // thread id and number of threads, from the argument
  vtkImageData *input; // the slice input

  // get the thread id and number of threads
  threadId = ((ThreadInfoStruct *)(arg))->ThreadID;
  threadCount = ((ThreadInfoStruct *)(arg))->NumberOfThreads;

  // get the filter, input and output in the form of a vtkFreehand2ThreadStruct
  // and get the input extent
  str = (vtkFreehand2ThreadStruct *)(((ThreadInfoStruct *)(arg))->UserData);
  input = str->Input;
  input->GetUpdateExtent( ext );


  
	//std::cout << "***********************************" << std::endl;
	//std::cout << "vtkFreehand2ThreadedExecute"<<std::endl;
	int trialextent[6];
	str->Output->GetUpdateExtent(trialextent);
	//std::cout << "output UPDATE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	





  // execute the actual method with appropriate extent
  // first find out how many pieces the extent can be split into and calculate
  // the extent for this thread (the splitExt)
  // TODO the documentation for SplitSliceExtent says that it needs to be called
  // threadCount times in order to update the split extent for each threadID -
  // when does that happen?... maybe because we are only using one thread at the moment...
  // but that depends on the calling function
  total = str->Filter->SplitSliceExtent(splitExt, ext, threadId, threadCount);
  //total = 1;
  
  // if we can use this thread, then call ThreadedSliceExecute
  if (threadId < total)
    {
    str->Filter->ThreadedSliceExecute(str->Input, str->Output,
				      splitExt, threadId);
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
// SplitSliceExtent
// For streaming and threads.  Splits the output update extent (startExt) into
// "total" pieces and calculates the split extent for the thread of thread id
// "num".  This method needs to be called "total" times
// with different values of num (thread id) to fill in the split extents for each
// therad.  This method returns the number of pieces resulting from a successful
// split - from 1 to "total".  If 1 is returned, the extent cannot be split.
// 
// TODO Old description - I think there are some errors here:
// For streaming and threads.  Splits output update extent into num pieces.
// This method needs to be called num times.  Results must not overlap for
// consistent starting extent.  Subclass can override this method.
// This method returns the number of peices resulting from a successful split.
// This can be from 1 to "total".  
// If 1 is returned, the extent cannot be split.
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::SplitSliceExtent(int splitExt[6], // the extent of this split
					    int startExt[6],  //the original extent to be split up
					    int num, // current thread id
					    int total) // the maximum number of threads (pieces)
{
  int splitAxis; // the axis we should split along
  int min, max; // the min and max indices of the axis of interest

  // prints where we are, the starting extent, num and total for debugging
   vtkDebugMacro("SplitSliceExtent: ( " << startExt[0] << ", " << startExt[1]
		<< ", "
                << startExt[2] << ", " << startExt[3] << ", "
                << startExt[4] << ", " << startExt[5] << "), " 
                << num << " of " << total);
  
  // start with same extent - copy the extent from startExt to splitExt
  memcpy(splitExt, startExt, 6 * sizeof(int));

  // determine which axis we should split along - preference is z, then y, then x
  // as long as we can possibly split along that axis (i.e. as long as z0 != z1)
  splitAxis = 2; // at the end, shows whether we split along the z(2), y(1) or x(0) axis
  min = startExt[4]; // z0 of startExt
  max = startExt[5]; // z1 of startExt
  while (min == max)
    {
    --splitAxis;
    // we cannot split if the input extent is something like [50, 50, 100, 100, 0, 0]
    if (splitAxis < 0)
      { 
      vtkDebugMacro("  Cannot Split");
      return 1;
      }
    min = startExt[splitAxis*2];
    max = startExt[splitAxis*2+1];
    }

  // determine the actual number of pieces that will be generated (return value)
  int range = max - min + 1;
  // split the range over the maximum number of threads
  int valuesPerThread = (int)ceil(range/(double)total);
  // figure out the largest thread id used
  // TODO we will always use the maximum number of threads, so this will always be 
  // valuesPerThread - 1... 
  int maxThreadIdUsed = (int)ceil(range/(double)valuesPerThread) - 1;
  // if we are in a thread that will work on part of the extent, then figure
  // out the range that this thread should work on
  if (num < maxThreadIdUsed)
    {
    splitExt[splitAxis*2] = splitExt[splitAxis*2] + num*valuesPerThread;
    splitExt[splitAxis*2+1] = splitExt[splitAxis*2] + valuesPerThread - 1;
    }
  if (num == maxThreadIdUsed)
    {
    splitExt[splitAxis*2] = splitExt[splitAxis*2] + num*valuesPerThread;
    }

  // print for debugging
  /*
  vtkDebugMacro("  Split Piece: ( " <<splitExt[0]<< ", " <<splitExt[1]<< ", "
                << splitExt[2] << ", " << splitExt[3] << ", "
                << splitExt[4] << ", " << splitExt[5] << ")");
  */
 /* cout<<"SplitSliceExtent: ( " << splitExt[0] << ", " << splitExt[1]
	  << ", " << splitExt[2] << ", " << splitExt[3] << ", "
      << splitExt[4] << ", " << splitExt[5] << "), " 
      << num << " of " << total<<endl;*/

  // return the number of threads used
  return maxThreadIdUsed + 1;
}

//----------------------------------------------------------------------------
// MultiThread
// Setup the threader, and set the single method to be vtkFreehand2ThreadedExecute.
// Then execute vtkFreehand2ThreadedExecute.
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::MultiThread(vtkImageData *inData,
                                        vtkImageData *outData)
{
  // set up a vtkFreehand2ThreadStruct (defined above)
  vtkFreehand2ThreadStruct str;
  str.Filter = this;
  str.Input = inData;
  str.Output = outData;

    
	//std::cout << "***********************************" << std::endl;
	//std::cout << "MultiThread"<<std::endl;
	int trialextent[6];
	str.Output->GetUpdateExtent(trialextent);
	//std::cout << "output UPDATE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	



  
  // TODO this->NumberOfThreads is never updated past one when threads are
  // spawned or terminated
  this->Threader->SetNumberOfThreads(this->NumberOfThreads);
  // set the single method
  this->Threader->SetSingleMethod(vtkFreehand2ThreadedExecute, &str);
  // execute the single method using this->NumberOfThreads threads
  this->Threader->SingleMethodExecute();
}

//----------------------------------------------------------------------------
// OptimizedInsertSlice
// Given an input and output region, execute the filter algorithm to fill the
// output from the input - optimized by splitting into x,y,z components or
// with integer math
// It just executes a switch statement to call the
// vtkOptimizedInsertSlice method
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::OptimizedInsertSlice()
{


	  
	//std::cout << "***********************************" << std::endl;
	//std::cout << "OptimizedInsertSlice"<<std::endl;
	int trialextent[6];
	this->GetOutput()->GetUpdateExtent(trialextent);
	//std::cout << "output UPDATE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	




   // cout << "optimized insertslice whole extent: " << this->GetOutput()->GetWholeExtent()[0] << " " << this->GetOutput()->GetWholeExtent()[1] << endl;
  if (this->ReconstructionThreadId == -1)
    {

		// modified by Danielle
		//cout << "\t Within OptimizedInsertSlice - reconstructionThreadId == -1\n";

    // this->GetOutput()->Update();
      // TODO note that I think calls to internalexecuteinformation should be replaced with calls to requestinformation
    // this->InternalExecuteInformation();
      // TODO should replace this with requestinformation for vtk 5
		this->UpdateInformation(); // this goes to vtkAlgorithm version (I checked)
	//return; // ADDED BY CWEDLAKE
    }
  if (this->NeedsClear)
    {
		// modified by Danielle
		//cout << "\t Within OptimizedInsertSlice - this->NeedsClear\n";
    this->InternalClearOutput();
    }


	// added by Danielle for debugging
	//std::cout << "**************************************" << std::endl;
	//std::cout << "In optimized insert slice" << std::endl;
	//int trialextent[6];
	//this->GetOutput()->GetUpdateExtent(trialextent);
	//std::cout << "output update extent " << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	
	//std::cout << "OUTPUT IMAGE:" << std::endl;
	//this->GetOutput()->Print(std::cout);
	//std::cout << "ACCUMULATION BUFFER" << std::endl;
	//this->AccumulationBuffer->Print(std::cout);
	//std::cout << "**************************************" << std::endl;
	



  vtkImageData *inData = this->GetSlice();
  vtkImageData *outData = this->GetOutput();
  //coutcout << "optimized insertslice whole extent: " << this->GetOutput()->GetWholeExtent()[0] << " " << this->GetOutput()->GetWholeExtent()[1] << endl;
 
  this->ActiveFlagLock->Lock();
  // if not in ReconstructionThread, update the slice here
  // (otherwise, the slice is updated in vtkReconstructionThread
  // to ensure synchronization with the tracking)
  if (this->ReconstructionThreadId == -1) {
	  // modified by Danielle
		//cout << "\t Within OptimizedInsertSlice - this->ReconstructionThreadId == -1 still\n";

    int clipExt[6];
    this->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),
			inData->GetWholeExtent());
    inData->SetUpdateExtentToWholeExtent(); //(clipExt);
    inData->Update();

	/*if (this->Compounding) // new by Danielle
	{
		this->AccumulationBuffer->SetUpdateExtentToWholeExtent();
		this->AccumulationBuffer->Update();
	}*/

  }
  this->ActiveFlagLock->Unlock();
  //if (this->ReconstructionThreadId == -1) {
	this->MultiThread(inData, outData);
  //}
  this->Modified();
}

//----------------------------------------------------------------------------
// ThreadedSliceExecute
// This method is passed a input and output region, and executes the filter
// algorithm to fill the output from the input.
// It just executes a switch statement to call the correct vtkOptimizedInsertSlice
// function for the regions data types.
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::ThreadedSliceExecute(vtkImageData *inData, // input data
						 vtkImageData *outData, // output data
						 int inExt[6], // input extent (could be split for this thread)
						 int threadId) // current thread id
{

	//std::cout << "in ThreadedSliceExecute" << std::endl;

// TODO vtkFreehandUltrasound2.cxx says that we need to change ThreadedSliceExecute
// to ThreadedRequestData...
// void vtkFreehandUltrasound2::ThreadedRequestData
// (vtkInformation *,
//  vtkInformationVector **,
//  vtkInformationVector*,
//   vtkImageData ***inputData, 
//  vtkImageData **outputData,
//  int inExt[6], int threadId)
// {
//   // fprintf(stderr,"inExt %d %d %d %d %d %d\n",inExt[0],inExt[1],inExt[2],inExt[3],inExt[4],inExt[5]);
//   vtkImageData *inData = inputData[0][0];
//   vtkImageData *outData = outputData[0];

  // get scalar pointers for extents and output extent
  void *inPtr = inData->GetScalarPointerForExtent(inExt);
  int *outExt = this->OutputExtent;
  void *outPtr = outData->GetScalarPointerForExtent(outExt);
 
  // get the accumulation buffer and the scalar pointer for its extent, if we are compounding
  void *accPtr = NULL; 
  vtkImageData *accData = this->AccumulationBuffer;

  /*std::cout << "the accumulation buffer in threadedsliceexecute:" << std::endl;
  accData->Print(std::cout);
  std::cout << "the output in threadedsliceexecute" << std::endl;
  outData->Print(std::cout);
*/


 //this->AccumulationBuffer->Print(std::cout);

   if(vtkInformation* pInfo = this->AccumulationBuffer->GetPipelineInformation())
    {
    if(pInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT_INITIALIZED()))
      {
		 // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!UpdateExtent: Initialized\n";
      }
    else
      {
		 // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!UpdateExtent: Not Initialized\n";
      }
	}


	
	//std::cout << "***********************************" << std::endl;
	//std::cout << "threaded slice execute"<<std::endl;
	//std::cout << "THIS->UPDATEExtent" << outExt[0] << " " << outExt[1] << " " << outExt[2] << " " << outExt[3] << " " << outExt[4] << " " << outExt[5] << std::endl;
	int trialextent[6];

	outData->GetWholeExtent(trialextent);
	//std::cout << "output WHOLE extent"<< trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	outData->GetExtent(trialextent);
	//std::cout << "output EXTENT"<< trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
	outData->GetUpdateExtent(trialextent);
	//std::cout << "output UPDATE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;


  if (this->Compounding)
    {
		//std::cout << "trying to get the scalar pointer for the extent" << std::endl;
		
			// HACK!!!  Danielle
			//outData->SetUpdateExtent(outData->GetWholeExtent());

			
			accData->GetWholeExtent(trialextent);

			//std::cout << "accumulation buffer WHOLE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
			accData->GetExtent(trialextent);
			//std::cout << "accumulation buffer EXTENT" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
			accData->GetUpdateExtent(trialextent);
			//std::cout << "accumulation buffer UPDATE extent" << trialextent[0] << " " << trialextent[1] << " " << trialextent[2] << " " << trialextent[3] << " " << trialextent[4] << " " << trialextent[5] << std::endl;
		
			//std::cout << "THE OUT EXTENT GIVEN TO THE ACCUMULATION BUFFER IS " << outExt[0] << " " << outExt[1] << " " << outExt[2] << " " << outExt[3] << " " << outExt[4] << " " << outExt[5] << std::endl;

			accPtr = accData->GetScalarPointerForExtent(outExt);
			//std::cout << "is accPtr null? " << (accPtr == NULL) << std::endl;

	//std::cout << "got the scalar pointer for the extent" << std::endl;

	//std::cout << "trying the same thing for the output" << std::endl;
	void * outPtr2 = this->GetOutput()->GetScalarPointerForExtent(outExt);
	//std::cout << "finished trying the same thing for the output" << std::endl;



	}
  else
    {
    accPtr = NULL;
    }

	//std::cout << "***********************************" << std::endl;

  // print out for debugging
  vtkDebugMacro(<< "OptimizedInsertSlice: inData = " << inData 
  << ", outData = " << outData);
  
  // this filter expects that input is the same type as output.
  if (inData->GetScalarType() != outData->GetScalarType())
    {
    vtkErrorMacro(<< "OptimizedInsertSlice: input ScalarType, " 
		  << inData->GetScalarType()
		  << ", must match out ScalarType "<<outData->GetScalarType());
    return;
    }

  // use fixed-point math for optimization level 2
  if (this->GetOptimization() == 2)
    {
      // TODO did Pieli mean to do this?  It seems like this part just copies
      // matrix into newmatrix
    // change transform matrix so that instead of taking 
    // input coords -> output coords it takes output indices -> input indices
    vtkMatrix4x4 *matrix = this->GetIndexMatrix();
    fixed newmatrix[4][4]; // NOTE that this is fixed for optimization = 2!!!
    for (int i = 0; i < 4; i++)
      {
      newmatrix[i][0] = matrix->GetElement(i,0);
      newmatrix[i][1] = matrix->GetElement(i,1);
      newmatrix[i][2] = matrix->GetElement(i,2);
      newmatrix[i][3] = matrix->GetElement(i,3);
      }

    //std::cout << "*****" << std::endl;
    //matrix->Print(std::cout);

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
	case VTK_SHORT:{
		vtkOptimizedInsertSlice(this, outData, (short *)(outPtr), 
                                (unsigned short *)(accPtr), 
                                inData, (short *)(inPtr), 
								inExt, newmatrix, threadId);}
        break;
	case VTK_UNSIGNED_SHORT:{
        vtkOptimizedInsertSlice(this,outData,(unsigned short *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned short *)(inPtr), 
								inExt, newmatrix, threadId);}
        break;
      case VTK_UNSIGNED_CHAR:{
        vtkOptimizedInsertSlice(this, outData,(unsigned char *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned char *)(inPtr), 
								inExt, newmatrix, threadId);}
        break;
      default:
        vtkErrorMacro(<< "OptimizedInsertSlice: Unknown input ScalarType");
        return;
      }
    }

  // if we are not using fixed point math for optimization = 2, we are either doing no
  // optimization (0) or we are breaking into x, y, z components with no bounds checking for
  // nearest neighbor (1)
  // the stuff in this part is the exact same as above in optimization == 2 except the index matrix
	// is of type double instead of fixed
  else
    {
        // TODO did Pieli mean to do this?  It seems like this part just copies
      // matrix into newmatrix
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
                                inExt, newmatrix, threadId);
        break;
      case VTK_UNSIGNED_SHORT:
        vtkOptimizedInsertSlice(this,outData,(unsigned short *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned short *)(inPtr), 
                                inExt, newmatrix, threadId);
        break;
      case VTK_UNSIGNED_CHAR:
        vtkOptimizedInsertSlice(this, outData,(unsigned char *)(outPtr),
                                (unsigned short *)(accPtr), 
                                inData, (unsigned char *)(inPtr), 
                                inExt, newmatrix, threadId);
        break;
      default:
        vtkErrorMacro(<< "OptimizedInsertSlice: Unknown input ScalarType");
        return;
      }
    }
}

//----------------------------------------------------------------------------
// vtkSleep
// platform-independent sleep function
//----------------------------------------------------------------------------
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
    //  vtkTimerLog::GetCurrentTime() is depracated and replace with
    //GetUniversalTime in VTK 5:
    double remaining = time - vtkTimerLog::GetUniversalTime() - 0.01;

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



  vtkFreehandUltrasound2 *self = (vtkFreehandUltrasound2 *)(data->UserData);


  
		// added by Danielle
	/*std::cout << "output at the very beginning of the reconstruction thread" << std::endl;
	self->GetOutput()->Print(std::cout);
	vtkSleep(5.0);*/



  //cout<<" vtkReconstructionThread \n\n"<<endl;
  double prevtimes[10];
  double currtime = 0;  // most recent timestamp
  double lastcurrtime = 0;  // previous timestamp
  double timestamp = 0;  // video timestamp, corrected for lag
  double videolag = self->GetVideoLag();
  int i;

  for (i = 0; i < 10; i++) {
    prevtimes[i] = 0.0;
  }

  // the tracker tool provides the position of each inserted slice
  if (!self->GetTrackerTool()) {
	  cout << "Couldn't find tracker tool =( =( =( =( =( =( =(" << endl;
    return NULL;
  }
  else {
	cout<<"Found Tracker Tool"<<endl;
  }

  vtkMatrix4x4 *matrix = self->GetSliceAxes();
  vtkTrackerBuffer *buffer = self->GetTrackerTool()->GetBuffer();
  if (!self->RealTimeReconstruction) { // if reconstructing previous data, use backup buffer
    buffer = self->TrackerBuffer;
  }

  //cout << "recon thread whole extent1: " << self->GetOutput()->GetWholeExtent()[0] << " " << self->GetOutput()->GetWholeExtent()[1] << endl;
  vtkVideoSource2 *video = self->GetVideoSource();
  vtkImageData *inData = self->GetSlice();
  
  // wait for video to start (i.e. wait for timestamp to change)
  if (video && self->RealTimeReconstruction) {
    while (lastcurrtime == 0 || currtime == lastcurrtime) {
      int clipExt[6];

	  self->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),
			  inData->GetWholeExtent());

	  // TODO VTK 5 : use Request-style method
      inData->SetUpdateExtentToWholeExtent();//(clipExt);
      inData->Update();

	  if (self->GetCompounding()) // new by Danielle
	  {
		  self->GetAccumulationBuffer()->SetUpdateExtentToWholeExtent();
		  self->GetAccumulationBuffer()->Update();
	  }


      lastcurrtime = currtime;
      currtime = video->GetFrameTimeStamp();
      double timenow = vtkTimerLog::GetUniversalTime();
      double sleepuntil = currtime + 0.010;
      if (sleepuntil > timenow) {
		vtkThreadSleep(data, sleepuntil);
	  }
    }
  }

  // cout << "recon thread whole extent2: " << self->GetOutput()->GetWholeExtent()[0] << " " << self->GetOutput()->GetWholeExtent()[1] << endl;
  double starttime = 0;

  // loop continuously until reconstruction is halted - HERE IS THE LOOP FOR THE WHOLE THING!!!!!
  for (i = 0;;) {
    // save the last timestamp
    lastcurrtime = currtime;

    // update the slice data
    int clipExt[6];
    self->GetClipExtent(clipExt, inData->GetOrigin(), inData->GetSpacing(),	inData->GetWholeExtent());
    // TODO VTK 5: use Request methods
    inData->SetUpdateExtentToWholeExtent();//(clipExt);
    inData->Update(); // TODO  VTK 5: RequestData ?? 


	/*if (self->GetCompounding()) // new by Danielle
	{
		self->GetAccumulationBuffer()->SetUpdateExtentToWholeExtent();
		self->GetAccumulationBuffer()->Update();
	}*/


    // get the timestamp for the video frame data
    
	if (video) {
	  //cout<<"Frame Index: "<<video->GetFrameIndex()<<endl;
      currtime = video->GetFrameTimeStamp();
      timestamp = currtime - videolag;
	  //cout<< "TimeStamp: "<< timestamp<<endl;
	}
	else {
		//cout<<"Video: "<<video<<endl;
	}
     
	if (starttime == 0) {
		starttime = timestamp;
    }

    buffer->Lock();
    int flags = 0;
    if (video && (videolag > 0.0 || !self->RealTimeReconstruction)) { // only do this if videolag is nonzero
      flags = buffer->GetFlagsAndMatrixFromTime(matrix, timestamp);
      //matrix->Print(std::cout);
    }
    else {
      buffer->GetMatrix(matrix, 0);
      flags = buffer->GetFlags(0);
	  if (!video) {
		currtime = buffer->GetTimeStamp(0);
	  }
    }
    buffer->Unlock();

	// modified by Danielle
	// SHOULD INCORPORATE FAN ROTATION HERE

	//// this code should invert the effect from the slice axes, so the slice isn't transformed =)
	//vtkMatrix4x4* danielle = vtkMatrix4x4::New();
	//danielle->DeepCopy(self->GetSliceAxes());
	//danielle->Invert();
	//vtkTransform * danielleTransform = vtkTransform::New();
	//danielleTransform->SetMatrix(danielle);
	//self->SetSliceTransform(danielleTransform);

	// this code should do nothing - it does! =)
	//vtkMatrix4x4* danielle = vtkMatrix4x4::New();
	//danielle->Identity();
	//vtkTransform * danielleTransform = vtkTransform::New();
	//danielleTransform->SetMatrix(danielle);
	//self->SetSliceTransform(danielleTransform);

	// WORKINGSPOT - porting from python code in US_TEprobe.py

	// very first image - use it to figure out the the flipping
	// the depth and the flipping should be constant, so there should be no thrashing -
	// therefore this should be reliable

		/*if (self->GetImageIsFlipped() == 1) // because we are flipping the image
		{
      //std::cout << "HERE!!!!!" << std::endl;
			// we are assuming we are flipped
			vtkFloatingPointType spacingTemp[3] = {0,0,0};
			int frameSizeTemp[3] = {0,0,0};
			self->GetOutputSpacing(spacingTemp);
			video->GetFrameSize(frameSizeTemp);
			//((vtkTransform *) (self->GetSliceTransform()))->Translate(0.0, -1.0*spacingTemp[1]*frameSizeTemp[1], 0.0);
			//((vtkTransform *) (self->GetSliceTransform()))->Scale(1.0, -1.0, 1.0);
			self->GetFlipTransform()->Translate(0.0, -1.0*spacingTemp[1]*frameSizeTemp[1], 0.0);
			self->GetFlipTransform()->Scale(1.0, -1.0, 1.0);
			//flip the image
			vtkImageFlip* flipper = vtkImageFlip::New();
			vtkImageData* newImage = vtkImageData::New();
			flipper->SetFilteredAxis(1);
			flipper->SetInput(inData);
			flipper->SetOutput(newImage);
			newImage->Update();
			inData = newImage;
			flipper->Delete();

			// TODO more stuff here with spacing and such...?
		}*/


/*	if (self->GetRotationClipData() == NULL)
	{

		// create the clip and threshold objects
		//self->SetDepthClipData(vtkImageClip::New());
		//self->GetDepthClipData()->ClipDataOn();
		self->SetRotationClipData(vtkImageClip::New());
		self->GetRotationClipData()->ClipDataOn();
		//self->SetDepthThreshold(vtkImageThreshold::New());
		//self->GetDepthThreshold()->ThresholdBetween(self->GetFanRotationImageThreshold1(), self->GetFanRotationImageThreshold2());
		//self->GetDepthThreshold()->SetOutValue(1);
		//self->GetDepthThreshold()->SetInValue(0);
		self->SetRotationThreshold(vtkImageThreshold::New());
		self->GetRotationThreshold()->ThresholdBetween(self->GetFanRotationImageThreshold1(), self->GetFanRotationImageThreshold2());
		self->GetRotationThreshold()->SetOutValue(1);
		self->GetRotationThreshold()->SetInValue(0);
//		self->SetFlipThreshold(vtkImageThreshold::New());
//		self->GetFlipThreshold()->ThresholdBetween(self->GetFanFlipThreshold1(), self->GetFanFlipThreshold2());
//		self->GetFlipThreshold()->SetOutValue(1);
//		self->GetFlipThreshold()->SetInValue(0);

		//// will find the depth assuming flipped and not flipped images
		//self->GetDepthClipData()->SetInput(inData);
		//self->GetDepthThreshold()->SetInput(self->GetDepthClipData()->GetOutput());
		//self->GetDepthThreshold()->Update();

		//self->SetImageIsFlipped(0); // image is not flipped
		//int depthNotFlipped = self->CalculateFanDepthCmValue(self->GetDepthThreshold());
		//self->SetImageIsFlipped(1); // image is flipped
		//int depthFlipped = self->CalculateFanDepthCmValue(self->GetDepthThreshold());

		//// we are not flipped
		//if (depthNotFlipped != -1 && depthFlipped == -1)
		//{
		//	self->SetImageIsFlipped(0);
		//	self->SetFanDepthCm(depthNotFlipped);
		//	
		//}
		//// we are flipped
		//else if (depthNotFlipped == -1 && depthFlipped != -1)
		//{
		//	self->SetImageIsFlipped(1);
		//	self->SetFanDepthCm(depthFlipped);
		//}
		//else
		//{
		//	std::cout << "problem with initializing depth and flip on first slice - aborting" << std::endl;
		//	return NULL;
		//}


		if (self->GetImageIsFlipped() == 0)
		{
			// we are assuming we are flipped
			vtkFloatingPointType spacingTemp[3] = {0,0,0};
			int frameSizeTemp[3] = {0,0,0};
			self->GetOutputSpacing(spacingTemp);
			video->GetFrameSize(frameSizeTemp);
			//((vtkTransform *) (self->GetSliceTransform()))->Translate(0.0, -1.0*spacingTemp[1]*frameSizeTemp[1], 0.0);
			//((vtkTransform *) (self->GetSliceTransform()))->Scale(1.0, -1.0, 1.0);
			self->GetFlipTransform()->Translate(0.0, -1.0*spacingTemp[1]*frameSizeTemp[1], 0.0);
			self->GetFlipTransform()->Scale(1.0, -1.0, 1.0);
			//flip the image
			vtkImageFlip* flipper = vtkImageFlip::New();
			vtkImageData* newImage = vtkImageData::New();
			flipper->SetFilteredAxis(1);
			flipper->SetInput(inData);
			flipper->SetOutput(newImage);
			newImage->Update();
			inData = newImage;
			flipper->Delete();
		}
		// more stuff here with spacing and such...?


	}
*/

	// get the rotation
	self->GetRotationClipData()->SetInput(inData);
	self->GetRotationThreshold()->SetInput(self->GetRotationClipData()->GetOutput());
	self->GetRotationThreshold()->Update();
	int rot = self->CalculateFanRotationValue(self->GetRotationThreshold());
	//std::cout << "**********" << std::endl;
	//std::cout << "*  rotation = " << rot << " ";
	//std::cout << "*  depth = " << self->GetFanDepthCm()< std::endl;
	//std::cout << "*  flipped = " << self->GetImageIsFlipped() << std::endl;
	//std::cout << "**********" << std::endl;*/
  
	if (rot > 0)
	{
		self->SetPreviousFanRotation(self->GetFanRotation());
		self->SetFanRotation(rot);
	}
  // ignore rotations of -1
  else
    {
    self->SetPreviousFanRotation(self->GetFanRotation());
    }

  //std::cout << "rot = " << rot << ", previous = " << self->GetPreviousFanRotation() << ", current = "<< self->GetFanRotation() << std::endl;

	
	// now use the rotation to change the SliceTransform (vtkTransform)
	// want to make sure that we're rotating in the right direction!!!
  vtkTransform* tempTransform = vtkTransform::New();
  vtkMatrix4x4* sliceAxesInverseMatrix = vtkMatrix4x4::New();
  vtkMatrix4x4::Invert(matrix, sliceAxesInverseMatrix);

	if (self->GetSliceTransform())
	{
		// the code assumes the image is flipped
		if (self->GetFanRotation() != self->GetPreviousFanRotation())
		{
      // original was these two lines
			//((vtkTransform *) (self->GetSliceTransform()))->RotateY(-1.0 * self->GetPreviousFanRotation());
			//((vtkTransform *) (self->GetSliceTransform()))->RotateY(self->GetFanRotation());
      

      tempTransform = (vtkTransform *) (self->GetSliceTransform());
      tempTransform->Identity();
      tempTransform->RotateY(self->GetFanRotation());
      //MATLAB CODE = (sliceAxes * sliceTransform(rotation) * inv(sliceAxes)
      tempTransform->PostMultiply();
      tempTransform->Concatenate(matrix); // remember, matrix = this->SliceAxes
      tempTransform->PreMultiply();
      tempTransform->Concatenate(sliceAxesInverseMatrix);
		}
	}

    // tool must be properly tracking, and the position must have updated,
    // if not we sleep until the next video frame
    if (currtime == lastcurrtime && self->RealTimeReconstruction) {
	  //cout<< "last current time: "<<lastcurrtime<<endl;	
      double timenow = vtkTimerLog::GetUniversalTime();
      double sleepuntil = currtime + 0.033;
      if (sleepuntil > timenow) {
		if (vtkThreadSleep(data, sleepuntil) == 0)  { // return if abort occurred during sleep
			return NULL;
		}
	  }
    }
    else if (flags & (TR_MISSING | TR_OUT_OF_VIEW)) {
      double timenow = vtkTimerLog::GetUniversalTime();
      double sleepuntil = currtime + 0.033;
	  cout<<"Out Of View"<<endl;
      if (sleepuntil > timenow) {
		if (vtkThreadSleep(data, sleepuntil) == 0) { // return if abort occurred during sleep
			return NULL;
		}
	  }
	}
    else {
		// do the reconstruction
		// TODO VTK 5: this method should stay the same
		//cout<<"Before InsertSlice"<<endl;
		self->InsertSlice();
		//cout<<"Inserted Slice"<<endl;
		// get current reconstruction rate over last 10 updates
		double tmptime = currtime;
		if (!self->RealTimeReconstruction) { // calculate frame rate using computer clock, not timestamps
			tmptime = vtkTimerLog::GetUniversalTime();
		}
		double difftime = tmptime - prevtimes[i%10];
		prevtimes[i%10] = tmptime;
		if (i > 10 && difftime != 0) {
			self->ReconstructionRate = (10.0/difftime);
		}
		i++;
	}

    // check to see if we are being told to quit 
    int activeFlag = *(data->ActiveFlag);

    if (activeFlag == 0) {
      return NULL;
    }

    if (!self->RealTimeReconstruction) {
      // sleep for a millisecond, just to give the main application
      // thread some time
      vtkSleep(0.001);

      if (video) {
		//fprintf(stderr, "go!  %i %i %g\n", self->ReconstructionFrameCount, video->GetFrameIndex(), timestamp - starttime);
		if (--self->ReconstructionFrameCount == 0) {
			return NULL;
		}
		video->Seek(1);
	  }
    }
  }
}


// TODO right now this is hard coded for the adult TEE
int vtkFreehandUltrasound2::CalculateFanRotationValue(vtkImageThreshold* threshold)
{
	int d1, d2, d3; // rotation values

	// not flipped
	if (this->GetImageIsFlipped() == 0)
	{
		// First Rotation Value
		int array3[12] = {72+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 72+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(),
											75+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 75+this->GetFanRotationXShift(), 479-299+this->GetFanRotationYShift(),
											71+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(), 77+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift()};
		d3 = this->GetFanRepresentation(threshold, array3);
		// Second Rotation Value
		int array2[12] = {62+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 62+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(),
												65+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 65+this->GetFanRotationXShift(), 479-299+this->GetFanRotationYShift(),
												61+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(), 67+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift()};
		d2 = this->GetFanRepresentation(threshold, array2);
		// Third Rotation Value
		int array1[12] = {52+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 52+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(),
												55+this->GetFanRotationXShift(), 479-294+this->GetFanRotationYShift(), 55+this->GetFanRotationXShift(), 479-299+this->GetFanRotationYShift(),
												51+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift(), 57+this->GetFanRotationXShift(), 479-298+this->GetFanRotationYShift()};
		d1 = this->GetFanRepresentation(threshold, array1);
	}
	// flipped
	else if (this->GetImageIsFlipped() == 1)
	{
		// First rotation value
		int array3[12] = {502+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 502+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(),
														505+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 505+this->GetFanRotationXShift(), 479-131+this->GetFanRotationYShift(),
														501+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(), 507+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift()};
		d3 = this->GetFanRepresentation(threshold, array3);

		//Second Rotation Value
		int array2[12] = {492+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 492+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(),
														495+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 495+this->GetFanRotationXShift(), 479-131+this->GetFanRotationYShift(),
														491+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(), 497+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift()};
		d2 = this->GetFanRepresentation(threshold, array2);

		// Third rotation value
		int array1[12] = {482+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 482+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(),
														485+this->GetFanRotationXShift(), 479-126+this->GetFanRotationYShift(), 485+this->GetFanRotationXShift(), 479-131+this->GetFanRotationYShift(),
														481+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift(), 487+this->GetFanRotationXShift(), 479-130+this->GetFanRotationYShift()};
		d1 = this->GetFanRepresentation(threshold, array1);
	}
	else
	{
		return -1;
	}

    if (d3 >= 0)
	{
        if (d2 >=0)
		{
            if (d1 >=0)
			{
                return d1*100+d2*10+d3;
			}
            else
			{
                return d2*10+d3;
			}
		}
        else
		{
            return d3;
		}
	}
	else
	{
		return -1;
	}

}

/*int vtkFreehandUltrasound2::CalculateFanDepthCmValue (vtkImageThreshold* threshold)
{

	int d1, d2;

	// not flipped
	if (this->GetImageIsFlipped() == 0)
	{
		// First Depth Value
		int array1[12] = {132+this->GetFanRotationXShift(), 479-58+this->GetFanRotationYShift(), 132+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift(),
												135+this->GetFanRotationXShift(), 479-58+this->GetFanRotationYShift(), 135+this->GetFanRotationXShift(), 479-63+this->GetFanRotationYShift(),
												131+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift(), 137+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift()};
		d1 = this->GetFanRepresentation (threshold, array1);
		// Second Depth Value
		int array2[12] = {142+this->GetFanRotationXShift(), 479-58+this->GetFanRotationYShift(), 142+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift(),
												145+this->GetFanRotationXShift(), 479-58+this->GetFanRotationYShift(), 145+this->GetFanRotationXShift(), 479-63+this->GetFanRotationYShift(),
												141+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift(), 147+this->GetFanRotationXShift(), 479-62+this->GetFanRotationYShift()};
		d2 = this->GetFanRepresentation(threshold, array2);
	}

	// flipped		
	else if (this->GetImageIsFlipped() == 1)
	{
		// First Depth Value
		int array1[12] = {22+this->GetFanRotationXShift(), 479-268+this->GetFanRotationYShift(), 22+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift(),
												25+this->GetFanRotationXShift(), 479-268+this->GetFanRotationYShift(), 25+this->GetFanRotationXShift(), 479-273+this->GetFanRotationYShift(),
												21+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift(), 27+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift()};
		d1 = this->GetFanRepresentation(threshold, array1);
		// Second Depth Value
		int array2[12] = {32+this->GetFanRotationXShift(), 479-268+this->GetFanRotationYShift(), 32+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift(),
												35+this->GetFanRotationXShift(), 479-268+this->GetFanRotationYShift(), 35+this->GetFanRotationXShift(), 479-273+this->GetFanRotationYShift(),
												31+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift(), 37+this->GetFanRotationXShift(), 479-272+this->GetFanRotationYShift()};
		d2 = this->GetFanRepresentation(threshold, array2);
	}
	else
	{
		return -1;
	}

    if (d2 >=0)
	{
        if (d1 >=0)
		{
            return d1*10+d2;
		}
        else
		{
            return d2;
		}
	}
    else
	{
        return -1;
	}
}*/

// TODO right now this is hard coded for the adult TEE
int vtkFreehandUltrasound2::GetFanRepresentation (vtkImageThreshold* threshold, int array[12])
{
	int list[6];
	int B = 0;
	int W = 1;
	int result = -1;

	for (int i = 0; i < 6; i++)
	{
		list[i] = threshold->GetOutput()->GetScalarComponentAsFloat(array[2*i], array[2*i+1],0,0);
	}

    if ((list[0] == B) && (list[1] == W) && (list[2] == W) && (list[3] == B) && (list[4] == W) && (list[5] == W))
	{
        result = 0;
	}
	else if ((list[0] == W) && (list[1] == B) && (list[2] == W) && (list[3] == W) && (list[4] == B) && (list[5] == B))
	{
        result = 1;  
	}
	else if ((list[0] == B) && (list[1] == W) && (list[2] == B) && (list[3] == B) && (list[4] == B) && (list[5] == B))
	{
        result = 2;
	}
	else if ((list[0] == B) && (list[1] == B) && (list[2] == W) && (list[3] == B) && (list[4] == B) && (list[5] == W))
	{
        result = 3;
	}
	else if ((list[0] == W) && (list[1] == W) && (list[2] == W) && (list[3] == W) && (list[4] == W) && (list[5] == W))
	{    
		result = 4;
	}
	else if ((list[0] == B) && (list[1] == B) && (list[2] == B) && (list[3] == B) && (list[4] == B) && (list[5] == W))
	{
        result = 5;
	}
    else if ((list[0] == W) && (list[1] == B) && (list[2] == B) && (list[3] == B) && (list[4] == W) && (list[5] == W))
	{
        result = 6;
	}
    else if ((list[0] == B) && (list[1] == W) && (list[2] == W) && (list[3] == B) && (list[4] == W) && (list[5] == B))
	{
        result = 7;
	}
    else if ((list[0] == B) && (list[1] == B) && (list[2] == B) && (list[3] == B) && (list[4] == W) && (list[5] == W))
	{
        result = 8;
	}
    else if ((list[0] == B) && (list[1] == B) && (list[2] == B) && (list[3] == W) && (list[4] == B) && (list[5] == W))
	{
        result = 9;
	}
	else
	{
		result = -1;
	}

	delete[] &list;
	return result;
}

/*int vtkFreehandUltrasound2::CheckIfUpsideDown(vtkImageThreshold* threshold)
{
	int result;

	// if depth value < 10
	int isDownDepthLessThan10 = threshold->GetOutput()->GetScalarComponentAsFloat(346+this->GetFanRotationXShift(), 479-57+this->GetFanRotationYShift() ,0,0);
    int isUpDepthLessThan10 = threshold->GetOutput()->GetScalarComponentAsFloat(337+this->GetFanRotationXShift(), 479-380+this->GetFanRotationYShift() ,0,0);

	// if depth value >= 10
    int isDownDepthGreaterThan10 = threshold->GetOutput()->GetScalarComponentAsFloat(338+this->GetFanRotationXShift(), 479-36+this->GetFanRotationYShift() ,0,0);
    int isUpDepthGreaterThan10 = threshold->GetOutput()->GetScalarComponentAsFloat(338+this->GetFanRotationXShift(), 479-423+this->GetFanRotationYShift() ,0,0);
	
	// image is flipped
	if ((isDownDepthLessThan10 == 1 && isUpDepthLessThan10 == 0) || (isDownDepthGreaterThan10 == 1 && isUpDepthGreaterThan10 == 0))
	{
		result = 1;
	}
	// image is not flipped
	else if ((isUpDepthLessThan10 == 1 && isDownDepthLessThan10 == 0) || (isUpDepthGreaterThan10 == 1 && isDownDepthGreaterThan10 == 0))
	{
		result = 0;
	}
	else
	{
		result = -1;
	}

	return result;
}*/


//----------------------------------------------------------------------------
// StartReconstruction
// Start doing a reconstruction from the video frames stored
// in the VideoSource buffer.  You should first use 'Seek'
// on the VideoSource to rewind first.  Then the reconstruction
// will advance through n frames one by one until the
// reconstruction is complete.  The reconstruction
// is performed in the background.
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::StartReconstruction(int frames)
{
  if (frames <= 0)
    {
    return;
    }

  // If the reconstruction isn't running (reconstructionThreadId == -1)
  if (this->ReconstructionThreadId == -1)
    {
    fprintf(stderr, "Reconstruction Start\n");
    this->RealTimeReconstruction = 0; // doing buffered reconstruction
    this->ReconstructionFrameCount = frames;
    // TODO why these next three lines? probably bad - look at if this deep copies
    vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
    this->SetSliceAxes(matrix);
    matrix->Delete();
	
	/*// added by danielle
	vtkTransform *transform = vtkTransform::New();
	this->SetSliceTransform(transform);
	transform->Delete();*/

    this->GetOutput()->Update();
    // TODO note that I think that calls to internalexecuteinformation should be
    // replaced with calls to requestinformation
    //this->InternalExecuteInformation(); // this was vtk 4
    // TODO does this call the vtkReconstructionThread function below?
    this->ReconstructionThreadId = \
      this->Threader->SpawnThread((vtkThreadFunctionType)\
				  &vtkReconstructionThread,
				  this);
	//cout<<" Reconstruction Thread: "<<this->ReconstructionThreadId<<endl;
    }
}

//----------------------------------------------------------------------------
// StopReconstruction
// Stop the reconstruction started wtih StartReconstruction() - returns the
// number of frames remaining to be reconstructed
//----------------------------------------------------------------------------
int vtkFreehandUltrasound2::StopReconstruction()
{
  // if a reconstruction is running (reconstructionThreadId != -1)
  if (this->ReconstructionThreadId != -1)
    {
    this->Threader->TerminateThread(this->ReconstructionThreadId);
    this->ReconstructionThreadId = -1;
    return this->ReconstructionFrameCount;
    }
  return 0;
}



//----------------------------------------------------------------------------
// StartRealTimeReconstruction
// Start doing real-time reconstruction from the video source.
// This will spawn a thread that does the reconstruction in the
// background.
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::StartRealTimeReconstruction()
{
	//std::cout << "depth ======= " << this->GetFanDepthCm() << std::endl;
	//std::cout << "flipped ===== " << this->GetImageIsFlipped() << std::endl;




  // if the reconstruction isn't running
  if (this->ReconstructionThreadId == -1)
    {

		// Danielle's note = we do go here right after hitting start

    this->RealTimeReconstruction = 1; // we are doing realtime reconstruction
    vtkMatrix4x4 *matrix = vtkMatrix4x4::New();
    this->SetSliceAxes(matrix);
    matrix->Delete();

	/*// added by danielle
	vtkTransform *transform = vtkTransform::New();
	this->SetSliceTransform(transform);
	transform->Delete();*/

    //TODO These two lines are different from StartReconstruction() above...
    // which is correct?
   // this->GetOutput()->Update();
    // TODO note that i think that calls to internalexecuteinformation should be
    // replaced with requestinformation
   this->InternalExecuteInformation();


		// Added by Danielle to deal with rotations
		this->SetRotationClipData(vtkImageClip::New());
		this->GetRotationClipData()->ClipDataOn();
		this->SetRotationThreshold(vtkImageThreshold::New());
		this->GetRotationThreshold()->ThresholdBetween(this->GetFanRotationImageThreshold1(), this->GetFanRotationImageThreshold2());
		this->GetRotationThreshold()->SetOutValue(1);
		this->GetRotationThreshold()->SetInValue(0);

		// End added by Danielle

   //cout << "start realtime whole extent: " << this->GetOutput()->GetWholeExtent()[0] << " " << this->GetOutput()->GetWholeExtent()[1] << endl;
    this->ReconstructionThreadId = \
      this->Threader->SpawnThread((vtkThreadFunctionType)\
				  &vtkReconstructionThread,
				  this);
	//cout<<"Reconstruction thread id: "<< this->ReconstructionThreadId <<endl;
	//cout<<"Video Information: "<<this->GetVideoSource()<<endl;
	//cout<<"Tracker Information: "<<this->GetTrackerTool()<<endl;
    }
}

//----------------------------------------------------------------------------
// StopRealTimeReconstruction
// Stop the reconstruction started with StartRealTimeReconstruction
//----------------------------------------------------------------------------
void vtkFreehandUltrasound2::StopRealTimeReconstruction()
{	
  // if a reconstruction is currently running
  if (this->ReconstructionThreadId != -1)
    {
      // recall that ActiveFlagLock is a vtkCriticalSection - locking of variables
      // accessed through different threads
      // TODO does this lock all vtkFreehandUltrasound2 attributes?  or what? There
      // doesn't seem to be anywhere in vtkCriticalSection that specifies which
      // variables are to be locked - this is just a flag
	this->ActiveFlagLock->Lock();
	cout<<"Thread : "<<this->ReconstructionThreadId <<" should terminate"<<endl;
	int killingThread = this->ReconstructionThreadId;
	// TODO why are we sleeping here?
	Sleep(2000);
	cout<<"Sleep Over"<<endl;
	this->Threader->TerminateThread(killingThread);
	cout<<"Thread : "<<this->ReconstructionThreadId <<" terminated"<<endl;
	this->ReconstructionThreadId = -1;
	this->ActiveFlagLock->Unlock();
	if (this->TrackerTool)
	  {
	    // the vtkTrackerBuffer (Atamai) should be locked before changing or
	    // accessing the data in the buffer if the buffer is being used from
	    // multiple threads
	    this->TrackerTool->GetBuffer()->Lock();
	    this->TrackerBuffer->DeepCopy(this->TrackerTool->GetBuffer());
	    this->TrackerTool->GetBuffer()->Unlock();
	  }
    }
}

//----------------------------------------------------------------------------
//int vtkFreehandUltrasound2::InRealTimeReconstruction()
//{
//  return (this->ReconstructionThreadId != -1);
//}

//----------------------------------------------------------------------------
// simple utility function
char *vtkJoinPath2(char *cp, int n, const char *directory, const char *file)
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
void vtkFreehandUltrasound2::SaveRawData(const char *directory, int frames)
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

	//TODO fix me so that it works when the directory is already created
  int res;
#ifdef _WIN32
  res = _mkdir(directory);
#else
  int mode = 0777;
  res = mkdir(directory, mode);
#endif

  //TODO put me back
  /*if (res < 0)
    {
    vtkErrorMacro(<< "couldn't create directory " << directory);
    return;
    }*/

  char path[512];

  vtkJoinPath2(path,512,directory,"track.txt");
  this->TrackerBuffer->WriteToFile(path);

  vtkJoinPath2(path,512,directory,"freehand.txt");
  FILE *file = fopen(path,"w");

  fprintf(file, "# vtkFreehandUltrasound2 output\n\n");

  vtkImageData *image = this->VideoSource->GetOutput();
  image->UpdateInformation();

  fprintf(file, "PixelSpacing = %7.5f %7.5f %7.5f;\n",
	  image->GetSpacing()[0], image->GetSpacing()[1], image->GetSpacing()[2]);
  fprintf(file, "PixelOrigin = %7.3f %7.3f %7.3f;\n",
	  image->GetOrigin()[0], image->GetOrigin()[1], image->GetOrigin()[2]);
  fprintf(file, "ClipRectangle = %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f;\n",
	  this->ClipRectangle[0], this->ClipRectangle[1],
	  this->ClipRectangle[2], this->ClipRectangle[3],
	  this->ClipRectangle[4], this->ClipRectangle[5]);
  fprintf(file, "FanAngles = %7.2f %7.2f;\n",
	  this->FanAngles[0], this->FanAngles[1]);
  fprintf(file, "FanOrigin = %7.3f %7.3f;\n",
	  this->FanOrigin[0], this->FanOrigin[1]);
  fprintf(file, "FanDepth = %7.3f;\n", this->FanDepth);
  fprintf(file, "VideoLag = %5.3f;\n\n", this->VideoLag);

  fclose(file);

  vtkJoinPath2(path,512,directory,"video.txt");
  char filePath[512];
  vtkJoinPath2(filePath,512,directory,"z");
  this->VideoSource->WriteFramesAsPNG(path, filePath, frames);
}

//----------------------------------------------------------------------------
char *vtkFreehandUltrasound2EatWhitespace(char *text)
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
void vtkFreehandUltrasound2::ReadRawData(const char *directory)
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
  
  // read in the tracking information
  vtkJoinPath2(path,512,directory,"track.txt");
  this->TrackerBuffer->ReadFromFile(path);

  // read in the freehand information
  vtkJoinPath2(path,512,directory,"freehand.txt");
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
    cp = vtkFreehandUltrasound2EatWhitespace(text);
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
      cp = vtkFreehandUltrasound2EatWhitespace(cp);
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
  

  // read in the video frames
  /*vtkJoinPath2(path,512,directory,"video.txt");
  this->VideoSource->ReadFramesAsPNG(path);

  //TODO take me out
  vtkJoinPath2(path,512,directory,"TESTvideo.txt");
  char filePath[512];
  this->VideoSource->WriteFramesAsPNG(path, filePath);*/

}
