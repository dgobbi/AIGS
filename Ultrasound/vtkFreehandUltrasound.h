/*=========================================================================

Copyright (c) 2000,2002 David Gobbi.

=========================================================================*/
// .NAME vtkFreehandUltrasound - real-time freehand ultrasound reconstruction
// .SECTION Description
// vtkFreehandUltrasound will incrementally compound ultrasound images into a
// reconstruction volume, given a transform which specifies the location of
// each ultrasound slice.  An alpha component is appended to the output to
// specify the coverage of each pixel in the output volume.
// .SECTION see also
// vtkVideoSource, vtkTracker, vtkTrackerTool


#ifndef __vtkFreehandUltrasound_h
#define __vtkFreehandUltrasound_h

#include "vtkImageSource.h"

class vtkLinearTransform;
class vtkMatrix4x4;
class vtkVideoSource;
class vtkTrackerTool;
class vtkTrackerBuffer;
class vtkMultiThreader;
class vtkCriticalSection;
class vtkImageData;

#define VTK_FREEHAND_NEAREST 0
#define VTK_FREEHAND_LINEAR 1

class VTK_EXPORT vtkFreehandUltrasound : public vtkImageSource
{
public:
  static vtkFreehandUltrasound *New();
  vtkTypeRevisionMacro(vtkFreehandUltrasound, vtkImageSource);

  virtual void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Set the video source to input the slices from.
  virtual void SetVideoSource(vtkVideoSource *);
  vtkGetObjectMacro(VideoSource,vtkVideoSource);

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

  // Description:
  // Fill holes in the output by using the weighted average of the
  // surrounding voxels.  If Compounding is off, then all hit voxels
  // are weighted equally. 
  void FillHolesInOutput();

  // Description:
  // Save the raw data in the directory specified.  The directory will
  // be created if it doesn't exist, and the following files will be
  // written inside it:
  // track.txt - a file with timestamped tracking information;
  // video.txt - a file with timestamps for each video image;
  // zXXXX.png - all of the video images, in sequential order.
  // You should first use 'Seek' on the VideoSource to rewind it.
  // Then the vtkVideoSource will be advanced one frame at a time
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
  // Set the image slice to insert into the reconstruction volume.
  virtual void SetSlice(vtkImageData *);
  vtkGetObjectMacro(Slice,vtkImageData);

  // Description:
  // Cause the slice to be inserted into the reconstruction volume.
  void InsertSlice();

  // Description:
  // Clear the data volume.
  void ClearOutput();

  // Description:
  // Set the clip rectangle (x0,y0,x1,y1) to apply to the image. 
  // Specify the rectange in millimeter coords, not pixel indices.
  vtkSetVector4Macro(ClipRectangle,float);
  vtkGetVector4Macro(ClipRectangle,float);

  // Description:
  // Get the clip rectangle as an extent, given a specific origin,
  // spacing, and max possible extent.
  void GetClipExtent(int clipExtent[6], const float origin[3],
		     const float spacing[3], const int extent[6]);

  // Description:
  // If the ultrasound probe collects a fan of data, specify the position and
  // dimensions of the fan.
  vtkSetVector2Macro(FanAngles,float);
  vtkGetVector2Macro(FanAngles,float);
  vtkSetVector2Macro(FanOrigin,float);
  vtkGetVector2Macro(FanOrigin,float);
  vtkSetMacro(FanDepth,float);
  vtkGetMacro(FanDepth,float);

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
  vtkSetMacro(Compounding,int);
  vtkGetMacro(Compounding,int);
  vtkBooleanMacro(Compounding,int);

  // Description:
  // Spacing, origin, and extent of output data
  // You MUST set this information.
  vtkSetVector3Macro(OutputSpacing, float);
  vtkGetVector3Macro(OutputSpacing, float);
  vtkSetVector3Macro(OutputOrigin, float);
  vtkGetVector3Macro(OutputOrigin, float);
  vtkSetVector6Macro(OutputExtent, int);
  vtkGetVector6Macro(OutputExtent, int);

  // Description:
  // When determining the modified time of the source. 
  unsigned long int GetMTime();

  // Description:
  // Have to override because of the funny way that data is
  // generated.
  void UpdateData(vtkDataObject *output);
  void ThreadedExecute(vtkImageData *inData, vtkImageData *outData,
		       int extent[6], int threadId);
  int SplitExtent(int splitExt[6], int startExt[6], int num, int total);

  // for filling holes
  void ThreadedFillExecute(vtkImageData *outData,	
			   int outExt[6], int threadId);
  

//BTX
  // Description:
  // Not protected because it has to be accessible from reconstruction thread.
  double ReconstructionRate;
  int RealTimeReconstruction;
  int ReconstructionFrameCount;
  vtkTrackerBuffer *TrackerBuffer;
//ETX

protected:
  vtkFreehandUltrasound();
  ~vtkFreehandUltrasound();

  double VideoLag;

  vtkImageData *Slice;

  vtkMatrix4x4 *SliceAxes;
  vtkLinearTransform *SliceTransform;
  int InterpolationMode;
  int Optimization;
  int Compounding;
  float OutputOrigin[3];
  float OutputSpacing[3];
  int OutputExtent[6];

  float ClipRectangle[4];
  float FanAngles[2];
  float FanOrigin[2];
  float FanDepth;

  vtkMatrix4x4 *IndexMatrix;
  vtkMatrix4x4 *LastIndexMatrix;

  vtkImageData *AccumulationBuffer;

  int NeedsClear;

  vtkMultiThreader *Threader;
  int NumberOfThreads;

  vtkVideoSource *VideoSource;
  vtkTrackerTool *TrackerTool;
  vtkMultiThreader *ReconstructionThreader;
  int ReconstructionThreadId;

  void MultiThread(vtkImageData *inData, vtkImageData *outData);
  void MultiThreadFill(vtkImageData *outData);
  double CalculateMaxSliceSeparation(vtkMatrix4x4 *m1, vtkMatrix4x4 *m2);
  vtkMatrix4x4 *GetIndexMatrix();
  void OptimizedInsertSlice();
  void ExecuteInformation();
  void Execute(vtkImageData *data) {};
  void Execute() { this->vtkImageSource::Execute(); };
  void InternalClearOutput();
  void InternalExecuteInformation();

private:
  vtkFreehandUltrasound(const vtkFreehandUltrasound&);
  void operator=(const vtkFreehandUltrasound&);
};

//----------------------------------------------------------------------------
inline char *vtkFreehandUltrasound::GetInterpolationModeAsString()
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





