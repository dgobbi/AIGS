/*=======================================================================

  Program:   NDI Combined API C Interface Library
  Module:    $RCSfile: ndicapi_thread.h,v $
  Creator:   David Gobbi <dgobbi@atamai.com>
  Language:  C
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

/*! \file ndicapi_thread.h
  This file contains the platform-dependent portions of the
  NDICAPI C API for doing thread handling.
*/

#ifndef NDICAPI_THREAD_H
#define NDICAPI_THREAD_H 1

#ifdef __cplusplus
extern "C" {
#endif

/*=====================================================================*/
/*! \defgroup NDIThread NDI Thread Methods
  These are low-level methods that provide a platform-independent
  multithreading.
*/


#if defined(WIN32) || defined(_WIN32)

#include <windows.h>
#include <winbase.h>
#include <sys/timeb.h>

typedef HANDLE NDIThread;
typedef HANDLE NDIMutex;
typedef HANDLE NDIEvent;

#elif defined(unix) || defined(__unix__)

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>
#include <pthread.h>

typedef struct {
  int signalled;
  pthread_cond_t cond;
  pthread_mutex_t mutex;
} pl_cond_and_mutex_t;
typedef pthread_t NDIThread;
typedef pthread_mutex_t *NDIMutex;
typedef pl_cond_and_mutex_t *NDIEvent;

#endif

NDIMutex ndiMutexCreate();
void ndiMutexDestroy(NDIMutex mutex);
void ndiMutexLock(NDIMutex mutex);
void ndiMutexUnlock(NDIMutex mutex);

NDIEvent ndiEventCreate();
void ndiEventDestroy(NDIEvent event);
void ndiEventSignal(NDIEvent event);
int ndiEventWait(NDIEvent event, int milliseconds);

NDIThread ndiThreadSplit(void *thread_func(void *userdata), void *userdata);
void ndiThreadJoin(NDIThread thread);

#ifdef __cplusplus
}
#endif

#endif /* NDICAPI_THREAD_H */
