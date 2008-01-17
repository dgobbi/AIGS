/*=========================================================================

  Program:   Visualization Toolkit
  Module:    $RCSfile: vtkUltrasoundFrameAnalyze.cxx,v $
  Language:  C++
  Date:      $Date: 2008/01/17 16:57:23 $
  Version:   $Revision: 1.5 $

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

#include "vtkUltrasoundFrameAnalyze.h"
#include "vtkObjectFactory.h"
#include "vtkImageData.h"
#include "vtkPointData.h"

//----------------------------------------------------------------------------
vtkUltrasoundFrameAnalyze* vtkUltrasoundFrameAnalyze::New()
{
  // First try to create the object from the vtkObjectFactory
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkUltrasoundFrameAnalyze");
  if(ret)
    {
    return (vtkUltrasoundFrameAnalyze*)ret;
    }
  // If the factory was unable to create the object, then create it here.
  return new vtkUltrasoundFrameAnalyze;
}

//----------------------------------------------------------------------------
vtkUltrasoundFrameAnalyze::vtkUltrasoundFrameAnalyze()
{
  this->Spacing[0] = this->Spacing[1] = this->Spacing[2] = 1.0;
  this->Origin[0] = this->Origin[1] = this->Origin[2] = 0.0;
  this->Flip[0] = this->Flip[1] = this->Flip[2] = 0;
  this->ClipExtent[0] = this->ClipExtent[1] = 0;
  this->ClipExtent[2] = this->ClipExtent[3] = 0;
  this->ClipExtent[4] = this->ClipExtent[5] = 0;
  this->ClipRectangle[0] = this->ClipRectangle[1] = 0;
  this->ClipRectangle[2] = this->ClipRectangle[3] = 0;
  this->ClipRectangle[4] = this->ClipRectangle[5] = 0;
  this->FanAngles[0] = this->FanAngles[1] = 0;
  this->FanOrigin[0] = this->FanOrigin[1] = 0;
  this->FanDepth = 0;

  // set up these for the ALOKA SSD-1700 NTSC by default
  this->ClipGuess[0] = 50;  // 50 for SSD-1700, 60 for SSD-5000 
  this->ClipGuess[1] = 509;
  this->ClipGuess[2] = 40;
  this->ClipGuess[3] = 439;
  this->ClipGuess[4] = 0;
  this->ClipGuess[5] = 0;

  this->StencilSource = vtkUltrasoundImageStencilSource::New();
}

//----------------------------------------------------------------------------
vtkUltrasoundFrameAnalyze::~vtkUltrasoundFrameAnalyze()
{
  this->StencilSource->Delete();
}

//--------------------------------------------------------------------------
// The 'floor' function on x86 and mips is many times slower than these
// and is used a lot in this code, optimize for different CPU architectures
static inline int vtkResliceFloor(double x)
{
#if defined mips || defined sparc
  return (int)((unsigned int)(x + 2147483648.0) - 2147483648U);
#elif defined i386 || defined _M_IX86
  unsigned int hilo[2];
  *((double *)hilo) = x + 103079215104.0;  // (2**(52-16))*1.5
  return (int)((hilo[1]<<16)|(hilo[0]>>16));
#else
  return int(floor(x));
#endif
}

static inline int vtkResliceCeil(double x)
{
  return -vtkResliceFloor(-x - 1.0) - 1;
}

static inline int vtkResliceRound(double x)
{
  return vtkResliceFloor(x + 0.5);
}

// convert a double into an integer plus a fraction  
static inline int vtkResliceFloor(double x, double &f)
{
  int ix = vtkResliceFloor(x);
  f = x - ix;
  return ix;
}

// convert a double into an integer plus a fraction  
static inline int vtkResliceFloor(float x, float &f)
{
  int ix = vtkResliceFloor(x);
  f = x - ix;
  return ix;
}

// Find the black and white levels for the image.
// The black level is the the most abundant pixel color, i.e. the
// histogram bin that contains the largest number of pixels.
// The white level is the color of the brightest pixel in the image.
static void vtkGetBlackAndWhite(vtkUltrasoundFrameAnalyze *self,
                                vtkImageData *input, unsigned char *inPtr,
                                float& black, float& white)
{
  int i,j;
  int inExt[6], inInc[3], numScalars;
  input->GetExtent(inExt);
  input->GetIncrements(inInc);
  numScalars = input->GetNumberOfScalarComponents();

  // initialize the histogram to zero
  int *histogram = new int[256];
  for (i = 0; i <= 255; i++)
    {
    histogram[i] = 0;
    }

  // loop over all pixels to build the histogram
  for (i = inExt[0]; i <= inExt[1]; i++)
    {
    for (j = inExt[2]; j <= inExt[3]; j++)
      {
      int val = inPtr[i*inInc[0] + j*inInc[1]];
      if (val >= 0 && val <= 255)
        {
        histogram[val] += 1;
        }
      }
    }

  // loop over the histogram bins to find white and black
  black = 0;
  white = 0;
  int maxcount = 0;
  for (i = 255; i >= 0; --i)
    {
    if (histogram[i] > maxcount)
      {
      maxcount = histogram[i];
      black = i;
      }
    if (histogram[i] > 0 && white == 0)
      {
      white = i;
      }
    }

  delete [] histogram;
}  

// Provide indices and coefficients for interpolation of values, where the
// interpolation kernel is a boxcar that is 2 pixels across.
// This is the most appropriate kernel to use, because the graticule lines
// themselves are boxcars that are 2 pixels across.
static inline void vtkGratInterpCoeffs(double x,
                                       int &x0, int &x1, int &x2,
                                       double &f0, double &f1, double &f2)
{
  x1 = vtkResliceRound(x);
  x0 = x1 - 1;
  x2 = x1 + 1;

  double f = 0.5*(x - x1);

  f0 = 0.25 - f;
  f1 = 0.5;
  f2 = 0.25 + f;
}

// This function analyzes a 1D image array of size 'size' which is known
// to contain a graticule pattern. 
// The spacing between the graticules, the position of the first graticule,
// and the number of graticules are returned.
static void vtkAnalyzeGratingImage(float *image, int size,
				   double mins, double maxs,
                                   vtkFloatingPointType &rspacing,
				   vtkFloatingPointType &rstart, int &rn)
{
  int i;
  double bestspacing = 0;
  double beststart = 0;
  double bestfom = 0;

  // loop over a whole bunch of spacings & start positions, find the
  // one for which the most 'white' lies on the graticules.
  for (double spacing = mins; spacing <= maxs; spacing += 0.03125)
    {
    for (double start = 1; start < spacing+1; start += 0.25)
      {
      double fom = 0;
      for (i = 0; ; i++)
        {
        double x = start + spacing*i;
        double f0, f1, f2;
        int x0, x1, x2;
        vtkGratInterpCoeffs(x, x0, x1, x2, f0, f1, f2);
        if (x2 >= size)
          {
          break;
          }
        fom += f0*image[x0] + f1*image[x1] + f2*image[x2];
        }
      fom /= i + 1;

      if (fom > bestfom)
        {
        //fprintf(stderr, "f, s, s %f %f %f\n",fom,start,spacing);
        double ratio = spacing/bestspacing;
        int iratio = vtkResliceRound(ratio);
        if (fom*0.7 > bestfom || iratio == 1 ||
	    ((ratio > iratio) ? (ratio - iratio) : (iratio - ratio)) > 0.1)
          {
          bestfom = fom;
          beststart = start;
          bestspacing = spacing;
          }
        }
      }
    }
  rspacing = bestspacing;

  // find out precisely where the graticule starts, i.e. find the
  // first graticule position that is 'white'
  for (i = 0; ; i++)
    {
    double x = beststart + bestspacing*i;
    double f0, f1, f2;
    int x0, x1, x2;
    vtkGratInterpCoeffs(x, x0, x1, x2, f0, f1, f2);
    if (x2 >= size)
      {
      break;
      }
    if (f0*image[x0] + f1*image[x1] + f2*image[x2] > bestfom*0.2)
      {
      break;
      }
    }
  rstart = beststart + bestspacing*i;

  // find out where the graticules stops
  for (i = 0; ; i++)
    {
    double x = rstart + bestspacing*i;
    double f0, f1, f2;
    int x0, x1, x2;
    vtkGratInterpCoeffs(x, x0, x1, x2, f0, f1, f2);
    if (x2 >= size)
      {
      break;
      }
    if (f0*image[x0] + f1*image[x1] + f2*image[x2] < bestfom*0.2)
      {
      break;
      }
    }
  rn = i;
}

// the orientation dot on an Aloka SSD-1700
static unsigned char SSD1700_DOT[]     = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};

// the orientation dot on an Aloka SSD-5000
static unsigned char SSD5000_DOT[]     = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,
                                          0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};

// half-resolution dot
static unsigned char SSDHALF_DOT[]     = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,
                                          0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};


// the focus depth triangle on an Aloka SSD-1700
static unsigned char SSD1700_FOCUS[]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,
                                          0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,
                                          0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};

// the focus depth triangle on an Aloka SSD-5000
static unsigned char SSD5000_FOCUS[]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};

// Search for the specified 16x16 binary glyph within an image, where
// [black,white] is the range of grey values in the image.
// You must pass a coord array for this function to fill in, as well as
// the maximum number 'ncoords' of matches to place in the array.
// The return value from this function is the number of matches actually
// found, which might be either greater or less than ncoords.
static int vtkGetGlyphPositions(unsigned char *imPtr, int imExt[6], 
                                int imInc[3], float black, float white,
                                unsigned char *glyph, 
                                int (*coords)[2], int ncoords)
{
  float scale = 1.0/(white-black);
  int whitethresh = vtkResliceFloor(white*0.5);

  int mini = imExt[0];
  int maxi = imExt[1] - 16;
  
  int minj = imExt[2];
  int maxj = imExt[3] - 16;
  
  int imIncX = imInc[0];
  int imIncY = imInc[1];

  unsigned char *tmpGlyph = new unsigned char[288];
  memset(tmpGlyph,0,288);
  unsigned char *tmpGlyph0 = tmpGlyph + 16;
  memcpy(tmpGlyph0,glyph,256);
  tmpGlyph0 -= 2;

  unsigned char *tmpMask = new unsigned char[288];
  memset(tmpMask,0,288);
  unsigned char *tmpMask0 = tmpMask + 16;

  // convolve the shape with an approximate of the video prefilter
  static float kernel[5] = {-0.05, 0.30, 0.50, 0.30, -0.05};
  unsigned char gPtr[256], gMaskPtr[256];
  //fprintf(stderr,"%3.3i %3.3i ",0,0);
  int numpixels = 0;
  for (int k = 0; k < 256; k++)
    {
    float val = (kernel[0]*tmpGlyph0[0] +
                 kernel[1]*tmpGlyph0[1] +
                 kernel[2]*tmpGlyph0[2] +
                 kernel[3]*tmpGlyph0[3] +
                 kernel[4]*tmpGlyph0[4]);
    int morph = (tmpGlyph0[0] | 
                 tmpGlyph0[1] | 
                 tmpGlyph0[2] | 
                 tmpGlyph0[3] | 
                 tmpGlyph0[4]);
    tmpMask0[k] = morph;
    val = val*(white-black) + black;
    if (val < 0)
      {
      val = 0;
      }
    if (val > 255)
      {
      val = 255;
      }
    gPtr[k] = vtkResliceRound(val);
    tmpGlyph0++;
    }

  tmpMask0 = tmpMask;
  for (int l = 0; l < 256; l++)
    {
    int morph = (tmpMask0[0] | 
                 tmpMask0[16] | 
                 tmpMask0[32]);
    gMaskPtr[l] = morph;
    numpixels += morph;

    tmpMask0++;
    }

  delete [] tmpGlyph;
  delete [] tmpMask;

  int *coordsum2 = new int[ncoords];
  for (int c = 0; c < ncoords; c++)
    {
    coordsum2[c] = 1073741824;
    }

  int numfound = 0;

  int llastsum2 = 1073741824;
  int lastsum2 = 1073741824;
  for (int j = minj; j <= maxj; j++)
    {
    for (int i = mini; i <= maxi; i++)
      {
      if (imPtr[(j+7)*imIncY + (i+7)*imIncX] < whitethresh)
        {
        continue;
        }

      int sum2 = 0;
      for (int jj = 0; jj < 16; jj++)
        {
        int jjj = j + jj;
        unsigned char *imPtr0 = imPtr + jjj*imIncY + i*imIncX;
        unsigned char *glyph0 = gPtr + jj*16;
        unsigned char *mask0 = gMaskPtr + jj*16;
        for (int ii = 0; ii < 16; ii++)
          {
          int diff = (*glyph0 - *imPtr0)*(*mask0);
          sum2 += (diff*diff)*255/(*glyph0 + 8);
          glyph0++;
          mask0++;
          imPtr0 += imIncX;
          }
        }

      if (lastsum2 < sum2 && lastsum2 < llastsum2 &&
          lastsum2 < 0.2*numpixels*(white-black)*(white-black))
        {
        fprintf(stderr,"found at %i,%i  %i %f\n",i-1,j,lastsum2,
                0.2*numpixels*(white-black)*(white-black));
        // insert it into the list
        numfound++;
        for (int c = 0; c < ncoords; c++)
          {
          if (lastsum2 < coordsum2[c])
            {
            for (int cc = ncoords-1; cc > c; cc--)
              {
              coords[cc][0] = coords[cc-1][0];
              coords[cc][1] = coords[cc-1][1];
              coordsum2[cc] = coordsum2[cc-1];
              }
            coords[c][0] = i-1; 
            coords[c][1] = j;
            coordsum2[c] = lastsum2;
            break;
            }
          }
        }
      llastsum2 = lastsum2;
      lastsum2 = sum2;
      }
    }

  delete [] coordsum2;

  return numfound;
}
                          
// This function finds the x and y graticules that are present in the
// image and uses them to determine the x and y pixel spacing and
// origin of the image.
static void vtkGetSpacingAndOrigin(vtkUltrasoundFrameAnalyze *self,
                                   vtkImageData *input, unsigned char *inPtr,
                                   vtkFloatingPointType Origin[3],
				   vtkFloatingPointType Spacing[3],
				   int ClipGuess[6],
                                   int ClipExtent[6],
				   double ClipRectangle[6])
{
  int i, j, jstart, jstop;
  int inExt[6], inInc[3], numScalars;
  input->GetExtent(inExt);
  input->GetIncrements(inInc);
  numScalars = input->GetNumberOfScalarComponents();

  // get the black & white levels
  float black = self->GetBlackLevel();
  float white = self->GetWhiteLevel();

  // sum together the columns of the image between 
  // one-thirtenth and 1/13 + 1/32 of the way across
  // the image to make a single-column 1D image
  float *ygrating = new float[ClipGuess[3] - ClipGuess[2] + 1];
 
  jstart = ClipGuess[0];
  jstop =  ClipGuess[0] + (ClipGuess[1] - ClipGuess[0] + 1)/32;
  for (i = ClipGuess[2]; i <= ClipGuess[3]; i++)
    {
    double val = 0;
    for (j = jstart; j < jstop; j++)
      {
      val += inPtr[i*inInc[1] + j*inInc[0]];
      }
    ygrating[i - ClipGuess[2]] = float((val/(jstop - jstart) - black)/
				       (white - black));
    }

  // minimum and maximum spacing between graticules
  double mins = (ClipGuess[3] - ClipGuess[2] + 1)/25;
  double maxs = (ClipGuess[3] - ClipGuess[2] + 1)/5;

  // analyze the 1D image to find the graticule parameters
  vtkFloatingPointType yspacing, ystart;
  int yn;
  vtkAnalyzeGratingImage(ygrating, ClipGuess[3] - ClipGuess[2] + 1,
			 mins, maxs,
                         yspacing, ystart, yn);
  ystart += ClipGuess[2];
  delete [] ygrating;

  cerr << "best " << yspacing << " " << ystart << " " << yn << "\n";

  // find the precise y position of the x graticule, in order to
  // determine the image bounding box:

  int tickratio = 1;
  int major_yn = 1;
  int last_major_yn = 1;
  double maxfom = 0;
  int xmin = jstart - 1;
  int xmin2 = jstop;
  for (j = jstart; j < jstop; j++)
    {
    double f = 1.0/(white - black)/yn;
    double fom = 0;
    int try_major_yn = 0;
    for (int yi = 0; yi < yn; yi++)
      { // user linear interpolation
      double f0, f1, f2;
      int y0, y1, y2;
      double y = ystart + yspacing*yi;
      vtkGratInterpCoeffs(y, y0, y1, y2, f0, f1, f2);
      double val0 = (inPtr[y0*inInc[1] + j*inInc[0]] - black);
      double val1 = (inPtr[y1*inInc[1] + j*inInc[0]] - black);
      double val2 = (inPtr[y2*inInc[1] + j*inInc[0]] - black);
      double partfom = f*(f0*val0 + f1*val1 + f2*val2);
      fom += partfom;
      if (partfom*yn > maxfom*0.5)
	{
	try_major_yn++;
	}
      }
    //cerr << "fom " << fom << "\n";
    if (fom > 0.25 && xmin < jstart)
      {
      xmin = j;
      }
    if (fom <= 0.07 && xmin >= jstart && xmin2 >= jstop)
      {
      xmin2 = j;
      tickratio = vtkResliceRound(1.0*yn/last_major_yn);
      }
    if (fom > maxfom)
      {
      maxfom = fom;
      }
    last_major_yn = major_yn;
    major_yn = try_major_yn;
    }
  cerr << "x position: " << xmin << " " << xmin2 << "\n";

  // get the ratio of tick marks that are major ticks
  cerr << "major tick every " << tickratio << " ticks\n";
  
  /*
  // for debugging purposes: re-draw the graticules
  for (i = 0; i < yn; i++)
    {
    double f0, f1, f2;
    int y0, y1, y2;
    double y = ystart + yspacing*i;
    vtkGratInterpCoeffs(y, y0, y1, y2, f0, f1, f2);
    inPtr[y0*inInc[1] + xmin*inInc[0]] = vtkResliceRound(f0*255);
    inPtr[y1*inInc[1] + xmin*inInc[0]] = vtkResliceRound(f1*255);
    inPtr[y2*inInc[1] + xmin*inInc[0]] = vtkResliceRound(f2*255);
    inPtr[y0*inInc[1] + xmin2*inInc[0]] = vtkResliceRound(f0*255);
    inPtr[y1*inInc[1] + xmin2*inInc[0]] = vtkResliceRound(f1*255);
    inPtr[y2*inInc[1] + xmin2*inInc[0]] = vtkResliceRound(f2*255);
    inPtr[y0*inInc[1] + jstart*inInc[0]] = vtkResliceRound(f0*255);
    inPtr[y1*inInc[1] + jstart*inInc[0]] = vtkResliceRound(f1*255);
    inPtr[y2*inInc[1] + jstart*inInc[0]] = vtkResliceRound(f2*255);
    inPtr[y0*inInc[1] + jstop*inInc[0]] = vtkResliceRound(f0*255);
    inPtr[y1*inInc[1] + jstop*inInc[0]] = vtkResliceRound(f1*255);
    inPtr[y2*inInc[1] + jstop*inInc[0]] = vtkResliceRound(f2*255);
    }
  */
  
  // sum together the rows of the image between 
  // one-twelfth and three-twentyfourths of the way up
  // the image to make a single-row 1D image
  float *xgrating = new float[ClipGuess[1] - ClipGuess[0] + 1];
 
  jstart = ClipGuess[2];
  jstop =  ClipGuess[2] + (ClipGuess[3] - ClipGuess[2] + 1)/20;
  for (i = ClipGuess[0]; i <= ClipGuess[1]; i++)
    {
    double val = 0;
    for (j = jstart; j < jstop; j++)
      {
      val += inPtr[i*inInc[0] + j*inInc[1]];
      }
    xgrating[i - ClipGuess[0]] = float((val/(jstop - jstart) - black)/
				       (white - black));
    }

  // analyze the 1D image to find the graticule parameters
  vtkFloatingPointType xspacing, xstart;
  int xn;
  vtkAnalyzeGratingImage(xgrating, ClipGuess[1] - ClipGuess[0] + 1,
			 yspacing*0.8, yspacing*1.2,
                         xspacing, xstart, xn);
  xstart += ClipGuess[0];
  delete [] xgrating;

  cerr << "best " << xspacing << " " << xstart << " " << xn << "\n";

  // find the precise y position of the x graticule, in order to
  // determine the image bounding box:

  int ymin = jstart - 1;
  int ymin2 = jstop;
  for (j = jstart; j < jstop; j++)
    {
    double f = 1.0/(white - black)/xn;
    double fom = 0;
    for (int xi = 0; xi < xn; xi++)
      { // use interpolation
      double f0, f1, f2;
      int x0, x1, x2;
      double x = xstart + xspacing*xi;
      vtkGratInterpCoeffs(x, x0, x1, x2, f0, f1, f2);
      double val0 = f*(inPtr[x0*inInc[0] + j*inInc[1]] - black);
      double val1 = f*(inPtr[x1*inInc[0] + j*inInc[1]] - black);
      double val2 = f*(inPtr[x2*inInc[0] + j*inInc[1]] - black);
      fom += f0*val0 + f1*val1 + f2*val2;
      }
    //cerr << "fom " << fom << "\n";
    if (fom > 0.25 && ymin < jstart)
      {
      ymin = j;
      }
    if (fom <= 0.03 && ymin >= jstart && ymin2 >= jstop)
      {
      ymin2 = j;
      }
    }
  cerr << "y position: " << ymin << " " << ymin2 << "\n";  

 /* 
  // for debugging purposes: re-draw the graticules
  for (i = 0; i < xn; i++)
    {
    double f0, f1, f2;
    int x0, x1, x2;
    double x = xstart + xspacing*i;
    vtkGratInterpCoeffs(x, x0, x1, x2, f0, f1, f2);
    inPtr[x0*inInc[0] + ymin*inInc[1]] = vtkResliceRound(f0*255);
    inPtr[x1*inInc[0] + ymin*inInc[1]] = vtkResliceRound(f1*255);
    inPtr[x2*inInc[0] + ymin*inInc[1]] = vtkResliceRound(f2*255);
    inPtr[x0*inInc[0] + ymin2*inInc[1]] = vtkResliceRound(f0*255);
    inPtr[x1*inInc[0] + ymin2*inInc[1]] = vtkResliceRound(f1*255);
    inPtr[x2*inInc[0] + ymin2*inInc[1]] = vtkResliceRound(f2*255);
    inPtr[x0*inInc[0] + jstart*inInc[1]] = vtkResliceRound(f0*255);
    inPtr[x1*inInc[0] + jstart*inInc[1]] = vtkResliceRound(f1*255);
    inPtr[x2*inInc[0] + jstart*inInc[1]] = vtkResliceRound(f2*255);
    inPtr[x0*inInc[0] + jstop*inInc[1]] = vtkResliceRound(f0*255);
    inPtr[x1*inInc[0] + jstop*inInc[1]] = vtkResliceRound(f1*255);
    inPtr[x2*inInc[0] + jstop*inInc[1]] = vtkResliceRound(f2*255);
    }
  */

  // convert graticule information into image spacing and origin
  Spacing[0] = 10.0/xspacing;
  Spacing[1] = 10.0/yspacing;
  Spacing[2] = 1.0;

  // in case the tick marks are every 5 mm
  if (tickratio == 2)
    {
    Spacing[0] /= 2;
    Spacing[1] /= 2;
    }

  Origin[0] = (inExt[0] - (xstart + xspacing*(xn-1)/2))*Spacing[0];
  Origin[1] = (inExt[2] - (ystart + yspacing*(yn-1)))*Spacing[1];
  Origin[2] = 0.0;

  ClipExtent[0] = xmin;
  ClipExtent[1] = vtkResliceRound(xmin + 
                                  ((xstart + xspacing*(xn-1)/2) - xmin)*2);
  ClipExtent[2] = ymin;
  ClipExtent[3] = vtkResliceRound(ystart + yspacing*(yn-1));
  ClipExtent[4] = 0;
  ClipExtent[5] = 0;

  ClipRectangle[0] = xmin2*Spacing[0] + Origin[0];
  ClipRectangle[1] = ClipExtent[1]*Spacing[0] + Origin[0];
  ClipRectangle[2] = ymin2*Spacing[1] + Origin[1];
  ClipRectangle[3] = ClipExtent[3]*Spacing[1] + Origin[1];
  ClipRectangle[4] = Origin[2];
  ClipRectangle[5] = Origin[2];
}

//----------------------------------------------------------------------------
// There is an 'orientation dot' that specifies whether the operator has
// flipped the ultrasound image either left/right or top/bottom.  This
// function finds the orientation dot and determines what quadrant of
// the image it lies within, then stores the results in 'flip'.  The
// Z-flip is always set to zero.
static void vtkGetFlip(vtkUltrasoundFrameAnalyze *self,
                       vtkImageData *input, unsigned char *inPtr,
                       int flip[2])
{
  int inExt[6], inInc[3], inWholeExt[6], numScalars;
  input->GetExtent(inExt);
  input->GetWholeExtent(inWholeExt);
  input->GetIncrements(inInc);
  numScalars = input->GetNumberOfScalarComponents();

  float black = self->GetBlackLevel();
  float white = self->GetWhiteLevel();

  fprintf(stderr, "black = %f, white = %f\n", black, white);

  int coords[1][2];
  int found = 0;

  if (inExt[1] - inExt[0] + 1 > 380) // full-resolution ultrasound image
    {
    // try finding the orientation dots for each machine
    fprintf(stderr, "SSD5000_DOT\n");
    found |= vtkGetGlyphPositions(inPtr,inExt,inInc,black,white,SSD5000_DOT,
                                  coords,1);
    fprintf(stderr, "SSD1700_DOT\n");
    found |= vtkGetGlyphPositions(inPtr,inExt,inInc,black,white,SSD1700_DOT,
                                  coords,1);
    }
  else // reduced-resolution (e.g. 320x240) ultrasound image
    {
    fprintf(stderr, "SSD HALF_DOT\n");
    found |= vtkGetGlyphPositions(inPtr,inExt,inInc,black,white,SSDHALF_DOT,
                                  coords,1);    
    }

  // find the 'centre' of the image, i.e. the points that separates
  // it into its four quadrants
  vtkFloatingPointType *spacing = self->GetSpacing();
  vtkFloatingPointType *origin = self->GetOrigin();
  int xcenter = vtkResliceFloor(-origin[0]/spacing[0]);
  int ycenter = (inWholeExt[0] + inWholeExt[1])/2;

  // compare the dot coords to the image center
  flip[0] = (coords[0][0] > xcenter);
  flip[1] = (coords[0][1] < ycenter);
  flip[2] = 0;

  fprintf(stderr,"Flip -> %i %i %i\n",flip[0],flip[1],flip[2]);
}

//----------------------------------------------------------------------------
// Get the two angles (left and right) that define the extent of the
// ultrasound fan.  This is done by finding the two edges that define
// the fan.
static void vtkGetFanAngles(vtkUltrasoundFrameAnalyze *self,
                            vtkImageData *input, unsigned char *inPtr,
                            double FanAngles[2], double FanOrigin[2],
                            double &FanDepth)
{
  int inExt[6], inInc[3], inWholeExt[6], numScalars;
  input->GetExtent(inExt);
  input->GetWholeExtent(inWholeExt);
  input->GetIncrements(inInc);
  numScalars = input->GetNumberOfScalarComponents();

  vtkFloatingPointType Spacing[3];
  vtkFloatingPointType Origin[3];
  self->GetSpacing(Spacing);
  self->GetOrigin(Origin);

  float black = self->GetBlackLevel();
  float white = self->GetWhiteLevel();

  int ClipExtent[6];
  self->GetClipExtent(ClipExtent);

  int xmin = ClipExtent[0];
  int xmax = ClipExtent[1];
  int ymin = ClipExtent[2];
  int ymax = ClipExtent[3];

  int xcenter = (xmin + xmax)/2;

  int thresh = vtkResliceRound(black + (white-black)*0.05);

  FanAngles[0] = 0;
  FanAngles[1] = 0;
  FanOrigin[0] = 0;
  FanOrigin[1] = 0;
  FanDepth = 0;

  double fom = 0;
  double bestfom = 0;
  int leftsum, rightsum;

  double bx0[2], by0[2], bx1[2], by1[2];
  for (int side = -1; side <= 1; side += 2)
    {
    int x, y, x0, x1, tx1, y0, y1;
    int bestx0 = 0;
    int besty0 = 0;
    int bestx1 = 0;
    int besty1 = 0;
    y0 = ymax;
    bestfom = 0;

    for (x0 = xcenter; (x0 - xcenter)*side <= (xmax - xcenter)*8/10; x0+=side)
      {
      for (tx1 = x0; (tx1 - x0)*side < (ymax - ymin)*2; tx1 += side)
        {
        x1 = tx1;
        y1 = ymin;
        if (x1 > xmax)
          {
          x1 = xmax;
          y1 = vtkResliceRound(y0 - ((x1 - x0)*1.0/(tx1 - x0))*(ymax - ymin));
          }
        else if (x1 < xmin)
          {
          x1 = xmin;
          y1 = vtkResliceRound(y0 - ((x1 - x0)*1.0/(tx1 - x0))*(ymax - ymin));
          }
        if ((y0 - y1)*3 < (ymax - ymin))
          {
          continue;
          }
        leftsum = rightsum = 0;
        for (y = y1; y <= y0; y += 4)
          {
          x = vtkResliceRound(x0 + ((x1 - x0)*1.0/(y0 - y1))*(y0 - y));
          int lv = (inPtr[x*inInc[0] + y*inInc[1]] +
                    inPtr[(x-side)*inInc[0] + y*inInc[1]]) >> 1;
          int rv = inPtr[(x+side)*inInc[0] + y*inInc[1]];
          if (lv > thresh)
            {
            lv = thresh;
            }
          if (rv > thresh)
            {
            rv = thresh;
            }
          leftsum += lv;
          rightsum += rv;
          }
        double leftval = ((leftsum*1.0/((y0 - y1 + 1)/4) - black)
                          /(white - black));
        double rightval = ((rightsum*1.0/((y0 - y1 + 1)/4) - black)
                           /(white - black));
        fom = (leftval - rightval)*(1 - rightval*rightval);
        
        if (fom > bestfom)
          {
          bestfom = fom;
          bestx0 = x0;
          besty0 = y0;
          bestx1 = x1;
          besty1 = y1;
          }
        /*
        if (fom > 0.005)
        { 
        cerr << fom;
        cerr << " (" << x0 << ", " << y0 << "), (" << x1 << ", " << y1 << ") ";
        cerr << " left " << leftval;
        cerr << " right " << rightval;
        cerr << "\n";
        }
                         */
      
        }
      }
    fprintf(stderr, "best line = (%i, %i), (%i, %i) %f\n",bestx0,besty0,bestx1,besty1,bestfom);

    int j = (1 + side)/2;
    double dx = bestx1 - bestx0;
    double dy = besty1 - besty0;
    FanAngles[j] = atan2(dx*Spacing[0],-dy*Spacing[1])*57.2957795131;

    bx0[j] = bestx0;
    by0[j] = besty0;
    bx1[j] = bestx1;
    by1[j] = besty1;
    }

  /* find the intersections of the two lines */
  double l[2];
  l[0] = (((bx1[1]-bx0[1])*(by0[0]-by0[1]) - (by1[1]-by0[1])*(bx0[0]-bx0[1]))/
          ((by1[1]-by0[1])*(bx1[0]-bx0[0]) - (bx1[1]-bx0[1])*(by1[0]-by0[0])));
  l[1] = (((bx1[0]-bx0[0])*(by0[0]-by0[1]) - (by1[0]-by0[0])*(bx0[0]-bx0[1]))/
          ((by1[1]-by0[1])*(bx1[0]-bx0[0]) - (bx1[1]-bx0[1])*(by1[0]-by0[0])));

  FanOrigin[0] = Origin[0] + (bx0[0] + l[0]*(bx1[0] - bx0[0]))*Spacing[0];
  FanOrigin[1] = Origin[1] + (by0[0] + l[0]*(by1[0] - by0[0]))*Spacing[1];

  fprintf(stderr,"angles = (%f, %f), origin = (%f, %f)\n",
          FanAngles[0],FanAngles[1],FanOrigin[0],FanOrigin[1]);

  bestfom = 0;
  int bestdepth = 0;
  int bestcount = 0;
  double x0 = (FanOrigin[0] - Origin[0])/Spacing[0];
  double y0 = (FanOrigin[1] - Origin[1])/Spacing[1];
  int mindepth = vtkResliceRound(FanOrigin[1]/Spacing[1] + (ymax-ymin)*1.0);
  int maxdepth = vtkResliceRound(FanOrigin[1]/Spacing[1] + (ymax-ymin)*1.5);

  fprintf(stderr, "x0 = %f, y0 = %f, off = %f\n", x0, y0, FanOrigin[1]/Spacing[1]);

  for (int depth = mindepth; depth < maxdepth; depth++)
    {
    int count = 0;
    leftsum = rightsum = 0;
    for (int x = xmin; x < xmax; x += 2)
      {
      double arg = depth*depth - (x - x0)*(x - x0);
      if (arg <= 0)
        {
        continue;
        }
      int y = vtkResliceRound(y0 - sqrt(arg));
      if (y < ymin || y >= ymax)
        {
        continue;
        }
      int lv = (inPtr[x*inInc[0] + y*inInc[1]] +
                inPtr[x*inInc[0] + (y + 1)*inInc[1]]) >> 1;
      int rv = inPtr[x*inInc[0] + (y - 1)*inInc[1]];
      if (lv > thresh)
        {
        lv = thresh;
        }
      if (rv > thresh)
        {
        rv = thresh;
        }
      leftsum += lv;
      rightsum += rv;
      count++;
      }
    if (count < 10)
      {
      continue;
      }

    double leftval = ((leftsum*1.0/count - black)
                      /(white - black));
    double rightval = ((rightsum*1.0/count - black)
                       /(white - black));
    fom = (leftval - rightval)*(1 - rightval*rightval);
    
    if (fom > bestfom)
      {
      bestfom = fom;
      bestdepth = depth;
      bestcount = count;
      }
    }

  if (bestdepth == 0)
    {
    bestdepth = maxdepth;
    }

  fprintf(stderr, "best depth = %i %i %f\n",bestdepth,bestcount,bestfom);

  FanDepth = bestdepth*Spacing[1];
}

//----------------------------------------------------------------------------
void vtkUltrasoundFrameAnalyze::Analyze()
{
  vtkImageData *input = this->GetInput();
  vtkFloatingPointType *inSpacing, *inOrigin;
  int inExt[6], inInc[3], numScalars;
  unsigned char *inPtr;

  // Check the inputs
  if (input == NULL)
    {
    vtkErrorMacro("Execute:  Input is NULL");
    return;
    }

  // update the image
  input->UpdateInformation();
  input->SetUpdateExtent(input->GetWholeExtent());
  input->Update();

  // get information about the image
  input->GetExtent(inExt);
  input->GetIncrements(inInc);
  inOrigin = input->GetOrigin();
  inSpacing = input->GetSpacing();
  numScalars = input->GetNumberOfScalarComponents();
  inPtr = (unsigned char *)input->GetScalarPointerForExtent(inExt);

  // adjust clipping guess according to whether image is 320x240 or
  // 640x480
  int clipGuess[6];
  for (int i = 0; i < 3; i++)
    {
    clipGuess[2*i] = this->ClipGuess[2*i];
    clipGuess[2*i+1] = this->ClipGuess[2*i+1];
    if (inExt[1] - inExt[0] + 1 <= 400)
      {
      clipGuess[2*i] = (clipGuess[2*i] + 1)/2;
      clipGuess[2*i+1] = (clipGuess[2*i+1] + 1)/2;
      }
    }

  vtkGetBlackAndWhite(this, input, inPtr, this->BlackLevel, this->WhiteLevel);
  vtkGetSpacingAndOrigin(this, input, inPtr, this->Origin, this->Spacing,
                         clipGuess,
			 this->ClipExtent, this->ClipRectangle);
  vtkGetFlip(this, input, inPtr, this->Flip);
  vtkGetFanAngles(this, input, inPtr, this->FanAngles, this->FanOrigin,
                  this->FanDepth);

  this->StencilSource->SetClipRectangle(this->ClipRectangle);
  this->StencilSource->SetFanAngles(this->FanAngles);
  this->StencilSource->SetFanOrigin(this->FanOrigin);
  this->StencilSource->SetFanDepth(this->FanDepth);

  this->Modified();
}

//----------------------------------------------------------------------------
// Change the information
void vtkUltrasoundFrameAnalyze::ExecuteInformation(vtkImageData *inData, 
                                                   vtkImageData *outData)
{
  outData->SetWholeExtent(inData->GetWholeExtent());
  outData->SetSpacing(this->Spacing);
  outData->SetOrigin(this->Origin);
}

//----------------------------------------------------------------------------
// This method simply copies by reference the input data to the output.
void vtkUltrasoundFrameAnalyze::ExecuteData(vtkDataObject *data)
{
  vtkImageData *inData = this->GetInput();
  vtkImageData *outData = (vtkImageData *)(data);

  outData->SetExtent(inData->GetExtent());
  outData->GetPointData()->PassData(inData->GetPointData());
}

//----------------------------------------------------------------------------
void vtkUltrasoundFrameAnalyze::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkImageToImageFilter::PrintSelf(os,indent);
  os << indent << "Input: " << this->GetInput() << "\n";

  os << indent << "BlackLevel: " << this->BlackLevel << "\n";
  os << indent << "WhiteLevel: " << this->WhiteLevel << "\n";
  os << indent << "Spacing: (" << this->Spacing[0] << ", "
     << this->Spacing[1] << ", " << this->Spacing[2] << ")\n";
  os << indent << "Origin: (" << this->Origin[0] << ", "
     << this->Origin[1] << ", " << this->Origin[2] << ")\n";
  os << indent << "Flip: (" << this->Flip[0] << ", "
     << this->Flip[1] << ", " << this->Flip[2] << ")\n";
  os << indent << "ClipExtent: (" << this->ClipExtent[0] << ", "
     << this->ClipExtent[1] << ", " << this->ClipExtent[2] << ", "
     << this->ClipExtent[3] << ", " << this->ClipExtent[4] << ", "
     << this->ClipExtent[5] << ")\n";
  os << indent << "ClipRectangle: (" << this->ClipRectangle[0] << ", "
     << this->ClipRectangle[1] << ", " << this->ClipRectangle[2] << ", "
     << this->ClipRectangle[3] << ", " << this->ClipRectangle[4] << ", "
     << this->ClipRectangle[5] << ")\n";
  os << indent << "FanAngles: (" << this->FanAngles[0] << ", "
     << this->FanAngles[1] << ")\n";
  os << indent << "FanOrigin: (" << this->FanOrigin[0] << ", "
     << this->FanOrigin[1] << ")\n";
  os << indent << "FanDepth: " << this->FanDepth << "\n";
  os << indent << "Stencil: " << this->StencilSource->GetOutput() << "\n";
}
