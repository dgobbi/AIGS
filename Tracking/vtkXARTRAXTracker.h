/*=========================================================================

  Program:   XartraxTracking for VTK
  Module:    $RCSfile: vtkXARTRAXTracker.h,v $
  Creator:   Chris Wedlake <cwedlake@imaging.robarts.ca>
  Language:  C++
  Author:    $Author: cwedlake $
  Date:      $Date: 2003/09/05 17:46:59 $
  Version:   $Revision: 1.1 $

==========================================================================

Copyright (c) 2000-2002
All rights reserved.

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
// .NAME vtkXARTRAXTracker - VTK interface for Northern Digital's POLARIS and Traxtal's XARTRAX
// .SECTION Description
// The vtkXARTRAXTracker class provides an  interface to the POLARIS
// (Northern Digital Inc., Waterloo, Canada) optical tracking system.
// It also controls the XARTRAX laser system by Traxtal using the XARTRAX
// API.   The only difference between the display of this method and
// vtkPOLARISTracker is that this class also forces stray markers to
// be displayed. (created as another array of tools)
// .SECTION Caveats
// This class refers to ports 1,2,3,A,B,C as ports 0,1,2,3,4,5
// .SECTION see also
// vtkTrackerTool vtkTracker vtkPOLARISTracker
//
// ADD TO HINT FILE:
//
// vtkXARTRAXTracker       FindAngleToPoint        307 2


#ifndef __vtkXARTRAXTracker_h
#define __vtkXARTRAXTracker_h

#include "vtkTracker.h"				// Shouldn't need this as it is included in the vtkPOLARISTracker.h
#include "XarTraXAPI.h"				// API for interacting with the XARTRAX system
#include "vtkPOLARISTracker.h"		// base class that operates the POLARIS camera

#define XAR_MAXSTRAY 20				/* maximum stray markers that will be recorded and displayed.  
									   If this changes, the InstrumentTrackerStray.py must also be modified
									   to accomidate this */

class vtkFrameToTimeConverter;

class VTK_EXPORT vtkXARTRAXTracker : public vtkPOLARISTracker
{
	 //BTX
public:
     //ETX

  static vtkXARTRAXTracker *New();
  vtkTypeMacro(vtkXARTRAXTracker,vtkPOLARISTracker);

//  void PrintSelf(ostream& os, vtkIndent indent);

  // Description:
  // Probe to see if the tracking system is present on the
  // specified serial port.  If the SerialPort is set to -1,
  // then all serial ports will be checked.
  int Probe();

  // Description:
  // Send a command to the POLARIS in the format INIT: or VER:0 (the
  // command should include a colon).  Commands can only be done after
  // either Probe() or StartTracking() has been called.
  // The text reply from the POLARIS is returned, without the CRC or
  // final carriage return.
  char *Command(const char *command);

  // Description:
  // Get the a string (perhaps a long one) describing the type and version
  // of the device.
  vtkGetStringMacro(Version);

  // Description:
  // Enable the xartrax tool by uploading a virtual SROM for that
  // it.  It will always be loaded into port 3. (although any port could have
  // been decided,  port 3 was just the first passive port)
  void LoadXarTraXROM(int port, const char *filename);

  // Description:
  // Unloads the xartrax tool from the system.
  void ClearXarTraXROM();

  // Description:
  // Original projection scheme created by Traxtal for use with the xartrax
  // contains an X and Y point and determines the location to places these by
  // a maximum angle at which the points will be displayed.
  // NOTE:
  // Locks out update to perform projection or system can stall out.
  int AProjection(char *filename, double time, int repeats, double maxAngle);

  // Description:
  // Projection scheme that uses an X angle and Y angle to determine where the 
  // points should be placed.  Since these are simple degree angles, transformation
  // cannot be calculated on this and are perminantely placed at these locations.
  // For projections that will be shifted and translated to different positions,
  // use AProjectionXYZ(...)
  // NOTE:
  // Locks out update to perform projection or system can stall out.
  int AProjectionDeg(char * filename, double time, int repeats);

  // Description:
  // Will scan a object at which the center is XLENGTH/2, YLENGTH/2.  Increment determines how fine of
  // resolution will be gathered.  Anything smaller that .1 will take a relatively long time depending
  // on how big XLENGTH and YLENGTH are.  It also requires a maximum and minumum range to filter out positions
  // detected on the camera that are not part of the object.  (I.E. walls, disturbance, ...)
  // NOTE:
  // Locks out update to perform scan or system can stall out.
  int Scan(char *OutFileName,	double XLength,	double YLength,	double Increment, int maxRange, int minRange);

  // Description:
  // Will scan a object from the TopLeft position to the BottomRight position.  This is the method to
  // use if the target is not directly in front of the camera.  Since the object will be on an angle, 
  // it is not always the best  result of scan in an image.  Increment determines how fine of resolution
  // will be gathered.  Anything smaller that .1 will take a relatively long time depending how far
  // apart TopLeft and BottomRight are.  It also requires a maximum and minumum range to filter out
  // positions detected on the camera that are not part of the object.  (I.E. walls, disturbance, ...)
  // NOTE:
  // Locks out update to perform scan or system can stall out.
  int ScanPos(char *OutFileName, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, double Increment, int maxRange, int minRange);

  // Description:
  // Turns the visible laser on
  // NOTE:
  // Locks out update to modify laser settings or system can stall out.
  int VisibleOn();

  // Description:
  // Turns the visible laser off
  // NOTE:
  // Locks out update to modify laser settings or system can stall out.
  int VisibleOff();

  // Description:
  // Returns an integer representing a boolean value describing if the visible laser is on.
  // Returns 1 if the visible laser is on.  Returns 0 if the laser is off.
  int IsVisibleOn();

  // Description:
  // Turns the infrared laser on
  // NOTE:
  // Locks out update to modify laser settings or system can stall out.
  int InfraOn();

  // Description:
  // Turns the visible laser off
  // NOTE:
  // Locks out update to modify laser settings or system can stall out.
  int InfraOff();

  // Description:
  // Returns an integer representing a boolean value describing if the infrared laser is on.
  // Returns 1 if the visible laser is on.  Returns 0 if the laser is off.
  int IsInfraOn();

  // Description:
  // Completes the Direct mode commands and executes them
  // NOTE:
  // Locks out update to call completion or system can stall out.
  int DFinish();

  // Description:
  // Completes the Arbitrary Waveform mode commands and executes them.  If the waveform has
  // already been uploaded to the system, to redisplay the image, all that is required is to
  // call this method.  There is no need to re-upload the data to the system.
  // NOTE:
  // Locks out update to call completion or system can stall out.
  int  AFinish();

  // Description:
  // Completes the Arbitrary Waveform mode commands and executes them.  If the waveform has
  // already been uploaded to the system, to redisplay the image, all that is required is to
  // call this method.  There is no need to re-upload the data to the system.
  // NOTE:
  // Locks out update to call completion or system can stall out.
  int GetAngleToTool(int tool, double *angleX, double *angleY);

/*
  // Description:
  // Given and X angle and a Y angle the system will move the laser to this position.
  // NOTE:
  // Locks out update to perform laser movement or system can stall out.
  int DegPosition(double X, double Y);
*/
  // Description:
  // Given and X angle and a Y angle the system will move the laser to this position.
  // The EXACT same method as DegPosition() so one should be removed. (DegPosition)
  // NOTE:
  // Locks out update to perform laser movement or system can stall out.
  int SetPosition(double X, double Y);

  // Description:
  // Given and X angle and a Y angle the system will move the laser to this position.
  // NOTE:
  // Locks out update to perform laser movement or system can stall out.
  int SetPosition(char * X, char * Y);

  // Description:
  // Sends a command to the xartrax to move the X mirror to a new degree angle.  This
  // is not actually executed until DFinish() is called.
  // NOTE:
  // Locks out update to send command to move X Mirror or system can stall out.
  int SetXPos(double X);
  // Description:
  // Sends a command to the xartrax to move the Y mirror to a new degree angle.  This
  // is not actually executed until DFinish() is called.
  // NOTE:
  // Locks out update to send command to move Y Mirror or system can stall out.
  int SetYPos(double Y);

  // Description:
  // Sends a command to the xartrax to move the X mirror to a new degree angle.  This
  // is not actually executed until DFinish() is called.
  // NOTE:
  // Locks out update to send command to move X Mirror or system can stall out.
  int SetXPos(char * X);

  // Description:
  // Sends a command to the xartrax to move the Y mirror to a new degree angle.  This
  // is not actually executed until DFinish() is called.
  // NOTE:
  // Locks out update to send command to move Y Mirror or system can stall out.
  int SetYPos(char * Y);

  // Description:
  // Finds if any error methods have been set and what they were.  Displays them to the screen
  // using vtkErrorMacro()
  void GetXarError();

  // Description:
  // Just calls the Sleep command as i was not able to find one to do the same thing in python.
  // Will be removed once a sleep command in python is found and replaced in xartrax.py
  void Delay(int n);

  // Description:
  // Will keep reading points and storing them in a file until it has aquired totalPoints.  If the
  // tool was NOT found, it still counts as a point. (to prevent long or stalled scans).  Stores the
  // file information as X-Angle Y-Angle isVisibleOn isInfraOn.  delayBetweenPoints is how long it should
  // sleep before getting the next point.  Total time of scan (in seconds) is (delayBetweenPoints*totalPoints)/1000.
  // For a spread out scan increase delay, for a packed in scan decrease delay.  usually 50-100 is good for delay.
  // NOTE:
  // Locks out main update periodically to prevent system stall out so that it can call update()
  // itself when it is needed.
  int StreamReadDeg(char * filename, int tool, int totalPoints, int delayBetweenPoints);

  // Description:
  // Will keep reading points and storing them in a file until it has aquired totalPoints.  This method
  // will also require you to store 4+ points that can be used for transformation markers later on.  
  // Marker points should be added to file first and then this method will append the points scanned.
  // E.G.
  // X Y Z (transform marker points)
  // ######## (break indicating scan points starting)
  // X Y Z isVisibleOn isInfraOn(scan points)
  //
  // If the tool was NOT found, it still counts as a point. (to prevent long or stalled scans).  Stores the
  // file information as X Y Z isVisibleOn isInfraOn.  delayBetweenPoints is how long it should
  // sleep before getting the next point.  Total time of scan (in seconds) is (delayBetweenPoints*totalPoints)/1000.
  // For a spread out scan increase delay, for a packed in scan decrease delay.  usually 50-100 is good for delay.
  // NOTE:
  // Locks out main update periodically to prevent system stall out so that it can call update()
  // itself when it is needed.
  int StreamReadXYZ(char * filename, int tool, int totalPoints, int delayBetweenPoints);

  // Description:
  // Will keep reading points and storing them in a file until it has aquired totalPoints.  This method
  // will also require you to store 4+ points that can be used for transformation markers later on.  
  // Marker points should be added to file first and then this method will append the points scanned.
  // E.G.
  // X Y Z (transform marker points)
  // ######## (break indicating scan points starting)
  // X Y Z isVisibleOn isInfraOn(scan points)
  //
  // If the tool was NOT found, it still counts as a point. (to prevent long or stalled scans).  Stores the
  // file information as X Y Z isVisibleOn isInfraOn.  delayBetweenPoints is how long it should
  // sleep before getting the next point.  Total time of scan (in seconds) is (delayBetweenPoints*totalPoints)/1000.
  // For a spread out scan increase delay, for a packed in scan decrease delay.  usually 50-100 is good for delay. Stores Landmarks and
  // converts the positions back to real positions and out of world coordinate values.
  // NOTE:
  // Locks out main update periodically to prevent system stall out so that it can call update()
  // itself when it is needed.
  int StreamReadXYZLandmarks(char * filename, int tool, int totalPoints, int delayBetweenPoints);

  // Description:
  // Finds the angle to a specific X,Y,Z point.  xarFindOrigin() needs to be called and completed for this to work first
  // which is called in the main start up method.  Ensuring that there are NO markers in the view at start up will
  // ensure that it calculates the correct vector for the laser.
  // NOTE:
  // - xarFindOrigin is required for this to work (called on startup already)
  // - Make sure that the hints file contains:
  //     vtkXARTRAXTracker       FindAngleToPoint        307 2
  double * FindAngleToPoint(float X, float Y, float Z);

  // Description:
  // Finds the angle to a specific X,Y,Z point.  xarFindOrigin() needs to be called and completed for this to work first
  // which is called in the main start up method.  Ensuring that there are NO markers in the view at start up will
  // ensure that it calculates the correct vector for the laser.
  // - xarFindOrigin is required for this to work (called on startup already)
  // - Make sure that the hints file contains:
  //     vtkXARTRAXTracker       FindAngleToPoint        307 2
  double * FindAngleToPoint(double X, double Y, double Z);

  // Description:
  // Finds the angle to a specific X,Y,Z point.  xarFindOrigin() needs to be called and completed for this to work first
  // which is called in the main start up method.  Ensuring that there are NO markers in the view at start up will
  // ensure that it calculates the correct vector for the laser.
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  void FindAngleToPoint(float X, float Y, float Z, double angle[2]);

  // Description:
  // Finds the angle to a specific X,Y,Z point.  xarFindOrigin() needs to be called and completed for this to work first
  // which is called in the main start up method.  Ensuring that there are NO markers in the view at start up will
  // ensure that it calculates the correct vector for the laser.
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  void FindAngleToPoint(double X, double Y, double Z, double angle[2]);

  // EXAMPLE XYZ FILE
  // 43.410770	-99.708839	-958.410583
  // 103.754303	-70.996101	-930.304077
  // 85.106567	-42.341690	-942.364746
  // 82.951508	-3.635500	-946.427429
  // #############################################
  // 42.474948	-100.606301	-958.218430	1	0
  // 41.871939	-101.120947	-958.481836	1	0
  // ...

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored. Uses Identity Matrix as transform
  // matrix
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats); 

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, 
				   double transform00, double transform01,double transform02,double transform03,
				   double transform10, double transform11,double transform12,double transform13,
				   double transform20, double transform21,double transform22,double transform23,
				   double transform30, double transform31,double transform32,double transform33);

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, double transform[4][4]);

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, float transform[4][4]);

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, 
				   float transform00, float transform01, float transform02,float transform03,
				   float transform10, float transform11, float transform12,float transform13,
				   float transform20, float transform21, float transform22,float transform23,
				   float transform30, float transform31, float transform32,float transform33);
  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, float transform[16]);

  // Description:
  // reads in a file that contains the X Y Z transition points then the X Y Z visible infra information.
  // See EXAMPLE XYZ FILE above for how the information should be stored.  Other than the transform matrix,
  // it is the same setup and display as the AProjectionDeg
  // NOTE:
  // xarFindOrigin is required for this to work (called on startup already)
  // Locks out update
  int AProjectionXYZ(char * filename, double time, int repeats, double transform[16]);

  vtkGetMacro(NumberOfStray, int);

  vtkGetMacro(XarTraXPort, int);

  // Description:
  // Get an update from the tracking system and push the new transforms
  // to the tools.  This should only be used within vtkTracker.cxx.
  void InternalUpdate();

  // Description:
  // Method for getting the stray marker for a given number.
  vtkTrackerTool * GetStray(int stray);

protected:
  vtkXARTRAXTracker();
  ~vtkXARTRAXTracker();

  // Description:
  // Set the version information.
  vtkSetStringMacro(Version);

  // Description:
  // Start the tracking system.  The tracking system is brought from
  // its ground state into full tracking mode.  The POLARIS will
  // only be reset if communication cannot be established without
  // a reset.
  int InternalStartTracking();

  // Description:
  // Stop the tracking system and bring it back to its ground state:
  // Initialized, not tracking, at 9600 Baud.
  int InternalStopTracking();

  // Description:
  // Class for updating the virtual clock that accurately times the
  // arrival of each transform, more accurately than is possible with
  // the system clock alone because the virtual clock averages out the
  // jitter.

  // Descriptions
  // Essentially the same as ToolUpdate but does it for all the stray markers that
  // it detected on the update. This way the stray markers (and xartrax) can be
  // displayed onto the screen as a green circle with a silver tip on the front. (towards camera)
  void StrayUpdate(int tool, vtkMatrix4x4 *matrix, long flags, double timestamp);

  int isxarLaser(double X, double Y, double Z);


private:
  vtkXARTRAXTracker(const vtkXARTRAXTracker&);
  void operator=(const vtkXARTRAXTracker&);
  int XarTraXPort;						// the port that the xartrax was loaded on (suggested 3)
  char XarTraXFile[150];				// the file that is to be loaded into the SROM for the xartrax
  int NumberOfStray;					// number of stray markers that were detected at last update
  bool infraON;							// stores information on if infrared laser is on
  bool visibleON;						// stores information on if visible laser is on
  double XMirror;
  double YMirror;
  double _OHAT[3];						// unit vector of the laser when it is aimed at (0,0)
  vtkTrackerTool *Stray[XAR_MAXSTRAY];	// all the stray markers. (max 20). manipulated the same as a tool
};

#endif





