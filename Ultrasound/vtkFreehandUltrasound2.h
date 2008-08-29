/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkFreehandUltrasound2.h,v $
  Language:  C++
  Date:      $Date: 2008/08/29 19:25:22 $
  Version:   $Revision: 1.1 $
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
// .NAME vtkFreehandUltrasound2 - real-time freehand ultrasound reconstruction
// .SECTION Description
// vtkFreehandUltrasound2 will incrementally compound ultrasound images into a
// reconstruction volume, given a transform which specifies the location of
// each ultrasound slice.  An alpha component is appended to the output to
// specify the coverage of each pixel in the output volume (i.e. whether or
// not a voxel has been touched by the reconstruction)
// .SECTION see also
// vtkVideoSource2, vtkTracker, vtkTrackerTool


#ifndef __vtkFreehandUltrasound2_h
#define __vtkFreehandUltrasound2_h

#include "vtkImageSource.h"
#include "vtkImageAlgorithm.h"

class vtkLinearTransform;
class vtkMatrix4x4;
class vtkMultiThreader;
class vtkVideoSource2;
class vtkTrackerTool;
class vtkTrackerBuffer;
class vtkCriticalSection;
class vtkImageData;
class vtkImageThreshold;
class vtkImageClip;
class vtkTransform;

#define VTK_FREEHAND_NEAREST 0
#define VTK_FREEHAND_LINEAR 1

class VTK_EXPORT vtkFreehandUltrasound2 : public vtkImageAlgorithm
{
public:
  static vtkFreehandUltrasound2 *New();
  vtkTypeRevisionMacro(vtkFreehandUltrasound2, vtkImageAlgorithm);
  virtual void PrintSelf(ostream& os, vtkIndent indent);

  // Description: 
  // Set the image slice to insert into the reconstruction volume.
  virtual void SetSlice(vtkImageData *);
  virtual vtkImageData* GetSlice();

	// added by danielle for debugging
  vtkGetObjectMacro(AccumulationBuffer, vtkImageData);


  virtual vtkImageData *GetOutput();
  // Description:
  // Set the video source to input the slices from.
  virtual void SetVideoSource(vtkVideoSource2 *);
  vtkGetObjectMacro(VideoSource,vtkVideoSource2);

  // Description:
  // Set the tracker tool to input transforms from.
  virtual void SetTrackerTool(vtkTrackerTool *);
  vtkGetObjectMacro(TrackerTool,vtkTrackerTool);

  // Description:
  // Start doing a reconstruction from the video frames stored
  // in the VideoSource buffer.  You should first use 'Seek'
  // on the VideoSource to rewind first.  Then the reconstruction
  // will advance through n frames one by one until the
  // reconstruction is complete.  The reconstruction
  // is performed in the background.
  void StartReconstruction(int n);
  
  // Description:
  // Stop the reconstruction.  The number of frames remaining to
  // be reconstructed is returned.
  int StopReconstruction();

  // Description:
  // Start doing real-time reconstruction from the video source.
  // This will spawn a thread that does the reconstruction in the
  // background.
  void StartRealTimeReconstruction();
  
  // Description:
  // Stop the real-time reconstruction.
  void StopRealTimeReconstruction();

  // Description:
  // Get the reconstruction rate.
  double GetReconstructionRate() { return this->ReconstructionRate; };

  double SliceCalculateMaxSliceSeparation(vtkMatrix4x4 *m1, vtkMatrix4x4 *m2);
  // Description:
  // Fill holes in the output by using the weighted average of the
  // surrounding voxels.  If Compounding is off, then all hit voxels
  // are weighted equally. 
  void FillHolesInOutput();

  // Description:
  // Save the raw data in the (relative!) directory specified.  The directory will
  // be created if it doesn't exist, and the following files will be
  // written inside it:
  // freehand.txt - a file with the freehand parameters within it
  // track.txt - a file with timestamped tracking information;
  // video.txt - a file with timestamps for each video image;
  // zXXXX.png - all of the video images, in sequential order.
  // You should first use 'Seek' on the VideoSource to rewind it.
  // Then the vtkVideoSource2 will be advanced one frame at a time
  // until n frames have been saved.
  void SaveRawData(const char *directory, int n);

  // Description:
  // Read the raw data from the specified directory and use it for the
  // following reconstructions.
  void ReadRawData(const char *directory);

  // Description:
  // Set the time by which the video lags behind the tracking information,
  // in seconds.  This value may be negative.  Default: 0.
  vtkSetMacro(VideoLag,double);
  vtkGetMacro(VideoLag,double);

  // Description:
  // Cause the slice to be inserted into the reconstruction volume.
  void InsertSlice();

  // Description:
  // Clear the data volume.
  void ClearOutput();

  // Description:
  // Set the clip rectangle (x0,y0,x1,y1) to apply to the image. 
  // Specify the rectange in millimeter coords, not pixel indices.
  vtkSetVector4Macro(ClipRectangle,double);
  vtkGetVector4Macro(ClipRectangle,double);

  // Description:
  // Get the clip rectangle as an extent, given a specific origin
  // spacing, and max possible extent.
  void GetClipExtent(int clipExtent[6],
		     vtkFloatingPointType origin[3],
		     vtkFloatingPointType spacing[3],
		     const int extent[6]);

  // Description:
  // If the ultrasound probe collects a fan of data, specify the position and
  // dimensions of the fan.
  vtkSetVector2Macro(FanAngles,double);
  vtkGetVector2Macro(FanAngles,double);
  vtkSetVector2Macro(FanOrigin,double);
  vtkGetVector2Macro(FanOrigin,double);
  vtkSetMacro(FanDepth,double);
  vtkGetMacro(FanDepth,double);

  // Description:
  // Set the axes of the slice to insert into the reconstruction volume,
  // relative the (x,y,z) axes of the reconstruction volume itself.
  // The axes are extracted from the 4x4 matrix:  The x-axis is the 
  // first column, the y-axis is the second column, the z-axis is the 
  // third column, and the origin is the final column.  The bottom
  // row of the matrix should always be (0,0,0,1).
  // If you don't set the axes, the axes will default to 
  // (1,0,0), (0,1,0), (0,0,1) and their origin will be (0,0,0)
  virtual void SetSliceAxes(vtkMatrix4x4 *);
  vtkGetObjectMacro(SliceAxes,vtkMatrix4x4);

  // Description:
  // Set a transform to be applied to the SliceAxes.
  // If you don't set this, it will be treated as the identity transform.
  virtual void SetSliceTransform(vtkLinearTransform *);
  vtkGetObjectMacro(SliceTransform,vtkLinearTransform);

  // Added by Danielle
  // Description:
  // Set the fan rotation (TEE probe)
  // If you don't set this, it will be treated as 0.
  vtkSetMacro(FanRotation,int);
  vtkGetMacro(FanRotation,int);
  vtkSetMacro(PreviousFanRotation,int);
  vtkGetMacro(PreviousFanRotation,int);
  vtkSetMacro(FanRotationImageThreshold1,int);
  vtkGetMacro(FanRotationImageThreshold1,int);
  vtkSetMacro(FanRotationImageThreshold2,int);
  vtkGetMacro(FanRotationImageThreshold2,int);
  vtkSetMacro(FanRotationXShift,int);
  vtkGetMacro(FanRotationXShift,int);
  vtkSetMacro(FanRotationYShift,int);
  vtkGetMacro(FanRotationYShift,int);
  vtkSetMacro(FanDepthCm,int);
  vtkGetMacro(FanDepthCm,int);
  //virtual void SetDepthClipData(vtkImageClip *);
  //vtkGetObjectMacro(DepthClipData, vtkImageClip);
  virtual void SetRotationClipData(vtkImageClip *);
  vtkGetObjectMacro(RotationClipData, vtkImageClip);
  //virtual void SetDepthThreshold(vtkImageThreshold *);
  //vtkGetObjectMacro(DepthThreshold, vtkImageThreshold);
  virtual void SetRotationThreshold(vtkImageThreshold *);
  vtkGetObjectMacro(RotationThreshold, vtkImageThreshold);
  //vtkSetMacro(FanFlipThreshold1,int);
  //vtkGetMacro(FanFlipThreshold1,int);
  //vtkSetMacro(FanFlipThreshold2,int);
  //vtkGetMacro(FanFlipThreshold2,int);
  //virtual void SetFlipThreshold(vtkImageThreshold *);
  //vtkGetObjectMacro(FlipThreshold, vtkImageThreshold);
  vtkSetMacro(ImageIsFlipped,int);
  vtkGetMacro(ImageIsFlipped,int);
  virtual void SetFlipTransform(vtkTransform *);
  vtkGetObjectMacro(FlipTransform, vtkTransform);

  // added by Danielle
  int CalculateFanRotationValue(vtkImageThreshold *);
 // int CalculateFanDepthCmValue(vtkImageThreshold *);
//BTX
  int GetFanRepresentation (vtkImageThreshold *, int[12]);
//ETX
  //int CheckIfUpsideDown(vtkImageThreshold *);



  // Description:
  // Turn on and off optimizations (default on, turn them off only if
  // they are not stable on your architecture).  Optimization level 2
  // uses integer math instead of floating-point math.
  vtkSetMacro(Optimization,int);
  vtkGetMacro(Optimization,int);
  vtkBooleanMacro(Optimization,int);

  // Description:
  // Set the interpolation mode, default is nearest neighbor. 
  vtkSetMacro(InterpolationMode,int);
  vtkGetMacro(InterpolationMode,int);
  void SetInterpolationModeToNearestNeighbor()
    { this->SetInterpolationMode(VTK_FREEHAND_NEAREST); };
  void SetInterpolationModeToLinear()
    { this->SetInterpolationMode(VTK_FREEHAND_LINEAR); };
  char *GetInterpolationModeAsString();

  // Description:
  // Turn on or off the compounding (default on, which means
  // that scans will be compounded where they overlap instead of the
  // new scan simply replacing the voxels from the old scan)
  //vtkSetMacro(Compounding,int); // removed by Danielle
  vtkGetMacro(Compounding,int);
  //vtkBooleanMacro(Compounding,int);
  void SetCompounding(int);

  // Description:
  // Spacing, origin, and extent of output data
  // You MUST set this information.
  vtkSetVector3Macro(OutputSpacing, vtkFloatingPointType);
  vtkGetVector3Macro(OutputSpacing, vtkFloatingPointType);
	// Call the command associated wtih the compoundvar checkbut
  vtkSetVector3Macro(OutputOrigin, vtkFloatingPointType);
  vtkGetVector3Macro(OutputOrigin, vtkFloatingPointType);
  vtkSetVector6Macro(OutputExtent, int);
  vtkGetVector6Macro(OutputExtent, int);

  // Description:
  // If true, flips image data along the X/Y axis when copying from the
  // input frame to the output image
  vtkSetMacro(FlipHorizontalOnOutput, int);
  vtkGetMacro(FlipHorizontalOnOutput, int);
  vtkBooleanMacro(FlipHorizontalOnOutput,int);
  vtkSetMacro(FlipVerticalOnOutput, int);
  vtkGetMacro(FlipVerticalOnOutput, int);
  vtkBooleanMacro(FlipVerticalOnOutput, int);

  // Description:
  // When determining the modified time of the source. 
  unsigned long int GetMTime();

  // Description:
  // Have to override because of the funny way that data is
  // generated.
  //void UpdateData(vtkDataObject *output);
  // need to change to ThreadedRequestData
  void ThreadedSliceExecute(vtkImageData *inData, vtkImageData *outData,
			    int extent[6], int threadId);
  int SplitSliceExtent(int splitExt[6], int startExt[6], int num, int total);

  // for filling holes
  void ThreadedFillExecute(vtkImageData *outData,	
			   int outExt[6], int threadId);
//  void IncrementPixelCount(int i){this->PixelCount += i;};

//BTX
  // Description:
  // Not protected because it has to be accessible from reconstruction thread.
  double ReconstructionRate;
  int RealTimeReconstruction;
  int ReconstructionFrameCount;
  vtkTrackerBuffer *TrackerBuffer;
//ETX
  int PixelCount[4];
  int GetPixelCount();
  void SetPixelCount(int threadId, int val);
  void IncrementPixelCount(int threadId, int increment);

  int GetReconstructionThreadId(){ return this->ReconstructionThreadId; };

protected:
  vtkFreehandUltrasound2();
  ~vtkFreehandUltrasound2();

  double VideoLag;

  vtkImageData *Slice;

  // added by danielle
  int FanRotation;
  int PreviousFanRotation;
  int FanRotationImageThreshold1;
  int FanRotationImageThreshold2;
  int FanRotationXShift;
  int FanRotationYShift;
  int FanDepthCm;
  //vtkImageClip *DepthClipData;
  vtkImageClip *RotationClipData;
  //vtkImageThreshold* DepthThreshold;
  vtkImageThreshold* RotationThreshold;
  //int FanFlipThreshold1;
  //int FanFlipThreshold2;
  //vtkImageThreshold* FlipThreshold;
  int ImageIsFlipped; // 0 means no (good pizza), 1 means yes (bad pizza)
  vtkTransform* FlipTransform;
  int FlipHorizontalOnOutput;
  int FlipVerticalOnOutput;

  vtkMatrix4x4 *SliceAxes;
  vtkLinearTransform *SliceTransform;
  int InterpolationMode;
  int Optimization;
  int Compounding;
  vtkFloatingPointType OutputOrigin[3];
  vtkFloatingPointType OutputSpacing[3];
  int OutputExtent[6];

  vtkFloatingPointType OldOutputOrigin[3];
  vtkFloatingPointType OldOutputSpacing[3];
  int OldOutputExtent[6];
  int OldScalarType;
  int OldNComponents;

  double ClipRectangle[4];
  double FanAngles[2];
  double FanOrigin[2];
  double FanDepth;

  vtkMatrix4x4 *IndexMatrix;
  vtkMatrix4x4 *LastIndexMatrix;

  vtkImageData *AccumulationBuffer;
  //TODO should remove ReconImage, as it is never used in the .cxx
  vtkImageData *ReconImage;
  int NeedsClear;

  vtkCriticalSection *ActiveFlagLock;

  vtkMultiThreader *Threader;
  int NumberOfThreads;

  vtkVideoSource2 *VideoSource;
  vtkTrackerTool *TrackerTool;
  vtkMultiThreader *ReconstructionThreader;
  int ReconstructionThreadId;

  void MultiThread(vtkImageData *inData, vtkImageData *outData);
  void MultiThreadFill(vtkImageData *outData);
  double CalculateMaxSliceSeparation(vtkMatrix4x4 *m1, vtkMatrix4x4 *m2);
  vtkMatrix4x4 *GetIndexMatrix();
  void OptimizedInsertSlice();
  void InternalClearOutput();
  void InternalExecuteInformation();

  // Remove these methods (they are VTK 4)
  //void ExecuteInformation();
  //void Execute(vtkImageData *data) {};
  //void Execute() { this->vtkImageAlgorithm::Execute(); };

  //virtual void UpdateData(vtkDataObject *outObject);
  //virtual void UpdateInformation();
  // These are the VTK 5 methods
  virtual int FillInputPortInformation(int port, vtkInformation* info);
  virtual int FillOutputPortInformation(int port, vtkInformation* info);
  virtual int ProcessRequest(vtkInformation*,
                             vtkInformationVector**,
                             vtkInformationVector*);
  virtual int RequestInformation(vtkInformation* request,
                                 vtkInformationVector** inputVector,
                                 vtkInformationVector* outputVector);
  virtual int RequestUpdateExtent(vtkInformation*,
                                 vtkInformationVector**,
                                 vtkInformationVector*);
 // main implementation of the algorithm
  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector** inputVector,
                          vtkInformationVector* outputVector);
  virtual int ComputePipelineMTime(vtkInformation *request,
				   vtkInformationVector **inInfoVec,
				   vtkInformationVector *outInfoVec,
				   int requestFromOutputPort,
				   unsigned long* mtime);


private:
  vtkFreehandUltrasound2(const vtkFreehandUltrasound2&);
  void operator=(const vtkFreehandUltrasound2&);


};

//----------------------------------------------------------------------------
inline char *vtkFreehandUltrasound2::GetInterpolationModeAsString()
{
  switch (this->InterpolationMode)
    {
    case VTK_FREEHAND_NEAREST:
      return "NearestNeighbor";
    case VTK_FREEHAND_LINEAR:
      return "Linear";
    default:
      return "";
    }
}  

#endif





