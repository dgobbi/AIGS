=======================================================================

  Program:   NDI Combined API C Interface Library
  Module:    $RCSfile: README.TXT,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C++
  Author:    $Author: dgobbi $
  Date:      $Date: 2002/11/07 04:00:57 $
  Version:   $Revision: 1.1 $

==========================================================================

This package provides a portable C library that provides a straightforward
interface to AURORA and POLARIS systems manufactured by Northern Digital Inc.
This library is provided by Atamai Inc., and is not supported by Northern
Digital Inc.

The contents of this package have been built successfully under
Linux-gcc, IRIX-cc, and Windows-MSVC.  A configure script is
supplied for use with Linux and IRIX.  It is used as follows:

  ./configure --prefix=/usr/local  (or another appropriate directory)
  make
  make install 

-----
The main contents of this package are as follows:

1) A C library (libndicapi.a) that provides a set of C functions for
   communicating with an NDI device via the NDI Combined API.  The
   documentation for this library is provided in the ndicapi_html
   directory.

2) Two C++ header files (polaris.h and polaris_math.h) that provide
   and interface, via libndicapi.a, to an NDI device via the  
   POLARIS Serial Communications API that predated the Combined API.
   Documentation is provided in the polaris_html directory.

3) The files CMakeLists.txt and vtkndicapi.def provide support for
   building the ndicapi library within the VTK directory tree.
   The vtkPOLARISTracker class supports the use of the POLARIS
   and AURORA systems within VTK.

A python interface to the ndicapi library is also available from
Atamai Inc., email dgobbi@atamai.com if you are interested. 

---
END OF FILE
