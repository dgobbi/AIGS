/*=========================================================================

  Program:   AtamaiTracking for VTK
  Module:    $RCSfile: vtkFrameToTimeConverter.cxx,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2005/08/08 19:42:54 $
  Version:   $Revision: 1.3 $

==========================================================================

Copyright (c) 2000-2005 Atamai, Inc.

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
#include "vtkFrameToTimeConverter.h"
#include "vtkTimerLog.h"
#include "vtkObjectFactory.h"

//----------------------------------------------------------------------------
vtkFrameToTimeConverter* vtkFrameToTimeConverter::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkFrameToTimeConverter");
  if(ret)
    {
    return (vtkFrameToTimeConverter*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkFrameToTimeConverter;
}

//----------------------------------------------------------------------------
vtkFrameToTimeConverter::vtkFrameToTimeConverter()
{
  this->NominalFrequency = 100.0;
  this->LastTimeStamp = 0;
  this->LastFrameCount = 0;
  this->LastLastFrameCount = 0;
  this->EstimatedFramePeriod = 1.0/this->NominalFrequency;
  this->NextFramePeriod = this->EstimatedFramePeriod;  
}

//----------------------------------------------------------------------------
vtkFrameToTimeConverter::~vtkFrameToTimeConverter()
{
}

//----------------------------------------------------------------------------
void vtkFrameToTimeConverter::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkObject::PrintSelf(os,indent);
  
  os << indent << "NominalFrequency: " << this->NominalFrequency << "\n";
  os << indent << "LastFrame: " << this->LastFrameCount << "\n";
  os << indent << "InstantaneousFrequency:" <<
    this->GetInstantaneousFrequency() << "\n";
}

//----------------------------------------------------------------------------
void vtkFrameToTimeConverter::Initialize()
{
  this->LastTimeStamp = 0;
  this->LastFrameCount = 0;
  this->LastLastFrameCount = 0;
  this->EstimatedFramePeriod = 1.0/this->NominalFrequency;
  this->NextFramePeriod = this->EstimatedFramePeriod;  
}

//----------------------------------------------------------------------------
void vtkFrameToTimeConverter::SetLastFrame(unsigned long framecount)
{
  if (framecount <= this->LastFrameCount)
    {
    return;
    }

  // read the system clock (name changed in VTK 5.0)
#if (VTK_MAJOR_VERSION <= 4)
  double timestamp = vtkTimerLog::GetCurrentTime();
#else
  double timestamp = vtkTimerLog::GetUniversalTime();
#endif

  double frameperiod = ((timestamp - this->LastTimeStamp)/
                        (framecount - this->LastFrameCount));
  double deltaperiod = frameperiod - this->EstimatedFramePeriod;

  this->LastTimeStamp += ((framecount - this->LastFrameCount)*
                          this->NextFramePeriod);
  this->LastLastFrameCount = this->LastFrameCount;
  this->LastFrameCount = framecount;

  // check the difference between the system clock and the
  // 'predicted' time for this framecount
  double diffperiod = (timestamp - this->LastTimeStamp);

  if (diffperiod < -0.2 || diffperiod > 0.2 || 
      framecount > 10 + this->LastLastFrameCount)
    { // time is off by more than 0.2 seconds: reset the clock
    this->NextFramePeriod = this->EstimatedFramePeriod;
    this->LastTimeStamp = timestamp;
    return;
    }

  // update our estimate of the current frame period based on the
  // measured period (the measured period will have a large error
  // associated with it, hence we only use 1% of the difference)
  this->EstimatedFramePeriod += deltaperiod*0.01;

  // vary the period for the next time round, but only let it
  // fluctuate by 1ms relative to the estimated frame period
  diffperiod *= 0.1;
  double maxdiff = 0.001;
  if (diffperiod < -maxdiff)
    {
    diffperiod = -maxdiff;
    }
  else if (diffperiod > maxdiff)
    {
    diffperiod = maxdiff;
    }
 
  this->NextFramePeriod = this->EstimatedFramePeriod + diffperiod;
  /*
  fprintf(stderr, "T %4i %i %.4f %.4f %.3f %.3f %f %f\n",
          framecount, framecount - this->LastLastFrameCount,
          this->EstimatedFramePeriod, this->NextFramePeriod,
          this->LastTimeStamp, timestamp, deltaperiod, diffperiod);
  */
}

//----------------------------------------------------------------------------
double vtkFrameToTimeConverter::GetTimeStampForFrame(unsigned long frame)
{
  return this->LastTimeStamp - 
    this->EstimatedFramePeriod*(this->LastFrameCount - frame);
}

//----------------------------------------------------------------------------
double vtkFrameToTimeConverter::GetInstantaneousFrequency()
{
  return 1.0/this->EstimatedFramePeriod;
}
 





