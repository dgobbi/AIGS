/*=======================================================================

  Program:   NDI Combined API C Interface Library
  Module:    $RCSfile: polaris_math.h,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2002/11/04 02:09:39 $
  Version:   $Revision: 1.1 $

==========================================================================
Copyright 2000,2001 Atamai, Inc.

Redistribution of this source code and/or any binary applications created
using this source code is prohibited without the expressed, written
permission of the copyright holders.  

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
=======================================================================*/

/*! \file polaris_math.h
  This file contains some math functions that are useful with the POLARIS.
*/

#ifndef POLARIS_MATH_H
#define POLARIS_MATH_H 1

#include "ndicapi_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

/*=====================================================================*/
/*! \defgroup PolarisMath Mathematical Functions 

   These are some useful math functions.  Note that the matrices are
   stored using the OpenGL convention:
  \f[
  \left(
  \begin{array}{cccc}
  m[0] & m[4] & m[8] & m[12] \\
  m[1] & m[5] & m[9] & m[13] \\
  m[2] & m[6] & m[10] & m[14] \\
  m[3] & m[7] & m[11] & m[15] \\
  \end{array}
  \right)
  \f]
*/

/*! \ingroup PolarisMath
  Find the position and orientation of a tool relative to a 
  reference tool.  This is done by quaternion division.

  \param a   the original tool transformation
  \param b   the reference tool transformation
  \param c   the resulting relative transformation

  The pointer \em c can be the same as pointer \em a if you want to do
  the division in-place.
*/
static void plRelativeTransform(const double a[8], const double b[8], double c[8]) {
  ndiRelativeTransform(a, b, c); }

/*! \ingroup PolarisMath
  Convert a quaternion transformation into a 4x4 float matrix.
*/
static void plTransformToMatrixf(const double trans[8], float matrix[16]) {
  ndiTransformToMatrixf(trans, matrix); }

/*! \ingroup PolarisMath
  Convert a quaternion transformation into a 4x4 double matrix.
*/
static void plTransformToMatrixd(const double trans[8], double matrix[16]) {
  ndiTransformToMatrixd(trans, matrix); }

/*! \ingroup PolarisMath
  Extract rotation angles from a 4x4 float matrix.  The order of the
  rotations is: 
  -# roll around \em x axis
  -# pitch around \em y axis
  -# yaw around \em z axis
*/
static void plAnglesFromMatrixf(float angles[3], const float matrix[16]) {
  ndiAnglesFromMatrixf(angles, matrix); }

/*! \ingroup PolarisMath
  Extract rotation angles from a 4x4 double matrix.  The order of the
  rotations is: 
  -# roll around \em x axis
  -# pitch around \em y axis
  -# yaw around \em z axis
*/
static void plAnglesFromMatrixd(double angles[3], const double matrix[16]) {
  ndiAnglesFromMatrixd(angles, matrix); }

/*! \ingroup PolarisMath
  Extract position coordinates from a 4x4 float matrix.  These have
  the same value as the position coordinates in the quaternion
  transformation.
*/
static void plCoordsFromMatrixf(float coords[3], const float matrix[16]) {
  ndiCoordsFromMatrixf(coords, matrix); }

/*! \ingroup PolarisMath
  Extract position coordinates from a 4x4 double matrix.  These have
  the same value as the position coordinates in the quaternion
  transformation.
*/
static void plCoordsFromMatrixd(double coords[3], const double matrix[16]) {
  ndiCoordsFromMatrixd(coords, matrix); }

#ifdef __cplusplus
}
#endif

#endif /* POLARIS_MATH_H */

