/*=======================================================================
        XarTraXAPI.h

        Chris Wedlake 

==========================================================================
Copyright

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
#ifndef XARTRAX_H
#define XARTRAX_H XARTRAX_H

#define XARTRAX_MAJOR_VERSION 1
#define XARTRAX_MINOR_VERSION 3

/*! \file XarTraXAPI.h
  This file contains the header to the C++ interface of the XARTRAX API.
*/

#include "polaris.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vtkMatrix4x4.h"

struct MaxMinXY {
	double MaxX;
	double MaxY;
	double MinX;
	double MinY;
};
typedef struct MaxMinXY MaxMinValue;

struct MaxMinXYZ {
	double MaxX;
	double MaxY;
	double MaxZ;
	double MinX;
	double MinY;
	double MinZ;
};
typedef struct MaxMinXYZ MaxMin3Value;


typedef struct{
	double x;
	double y;
	int vi;
	int invi;
} APosData;

typedef struct{
	int x;
	int y;
	int vi;
	int inv;
} APosData2;

//------------------------------------------------------
/* Used to define which channel is being addressed */
/*\{*/
#define XAR_CHANNEL_X			0
#define XAR_CHANNEL_Y			1
#define XAR_CHANNEL_YFLY		2			// Only used in raster mode	(causes an error in other modes)
/*\}*/

//------------------------------------------------------
/* modes to define if the laser is to be set On or Off */
/*\{*/
#define XAR_LASER_OFF			0
#define XAR_LASER_ON			1
/*\}*/


//-----------------------------------------------------
/* Used to define which value type is being used */
/*\{*/
#define XAR_UNIT_DAC			0			// User bits are NOT included
#define XAR_UNIT_DEG			1
#define XAR_UNIT_VOLT			2
#define XAR_UNIT_DAC_UBITS		3			// Only valid with PosiProL_gn_read_postable
/*\}*/										//  and PosiProL_aw_position

//------------------------------------------------------
/* Definitions used by PosiPro to define function generator */
/*\{*/
#define XAR_FG_TYPE_SQR			0			// Pulse and Square mode (Square = DC 50%)
#define XAR_FG_TYPE_SIN			1  
#define XAR_FG_TYPE_SAW			2			// Triangle and Sawtooth (Triangle = DC 50%)
/*\}*/


//------------------------------------------------------  
/* Interrrupt Select. Pass Actual interrrupt number */
/*\{*/
#define XAR_INTERR_NONE			0
#define XAR_INTERR_NUM3			3
#define XAR_INTERR_NUM5			5
#define XAR_INTERR_NUM7			7
#define XAR_INTERR_NUM10		10
#define XAR_INTERR_NUM11		11
#define XAR_INTERR_NUM12		12
#define XAR_INTERR_NUM15		15
/*\}*/

//------------------------------------------------------  
// Servo Types
#define XAR_CF_SERVO_602XX_B	0
#define XAR_CF_SERVO_602XX_C	2
#define XAR_CF_SERVO_653XX		1
#define XAR_CF_SERVO_650XX		0
#define XAR_CF_SERVO_6520		0
#define XAR_CF_SERVO_6521		0
#define XAR_CF_SERVO_6790		9
#define XAR_CF_SERVO_670XX		0

//---------------------------------------------------------
/* DAC communications timing*/
/*\{*/
#define XAR_DACTIME_NORM		0		//400 nsec
#define XAR_DACTIME_SLOW		1		//1 usec
/*\}*/

//------------------------------------------------------  
/* Enbable (or disable) external Trigger (NOT USED) */
/*\{*/
#define XAR_TRIG_INTERNAL		0			// The Default and safest mode
#define XAR_TRIG_EXTERNAL		1			// Requires external source!
/*\}*/

//------------------------------------------------------ 
/* DAC Rate range */
/*\{*/
#define XAR_MIN_DACRATE			10000
#define XAR_MAX_DACRATE			300000
/*\}*/


//------------------------------------------------------ 
/* DAC type  */
/*\{*/
#define XAR_16BITDAC			0
#define XAR_18BITDAC			1
/*\}*/


//---------------------------------------------------------
/* Swap X and Y  */
/*\{*/
#define XAR_XYSWAP_NORM			0
#define XAR_XYSWAP_SWAPPED		1
/*\}*/

//------------------------------------------------------ 
// CALL: PosProL_op_status()
// Parameter 1:  
/* Status Bits.
   These represent the most recent value read from the 6760 Status Register 1
// See the hardware reference manual for details  */
/*\{*/

#define XAR_SB_FIFO_FULL		0x0100		// 1 = Fifo is full 
#define XAR_SB_FIFO_EMPTY		0x0200		// 1 = Fifo is empty
#define XAR_SB_FIFO_HALF		0x0400		// 1 = Fifo is half empty
#define XAR_SB_FIFO_BLOWN		0x0800		// 1 = Fifo is blown

#define XAR_SB_SS_MUXD			0x0001		// Multiplexed Data Bus 
#define XAR_SB_SS_ICS			0x0002		// Inverted CS signal
#define XAR_SB_SS_DUAL			0x0004		// Enable Dual Scanner mode (X and Y)
#define XAR_SB_SS_SLOW			0x0008		// Slow Latch Timing

#define XAR_SB_INT_TRIG			0x0010		// Internal trigger is active
#define XAR_SB_BYPASS			0x0020		// Bypass mode is active (no trigger)
#define XAR_SB_EXT_TRIG			0x0040		// External Trigger is enabled
#define XAR_SB_EN_DRV			0x0080		// Drivers are active (necessary for all modes)

#define XAR_SB_UBITS			0xF000		// Read User Bits (All)
#define XAR_SB_UB0				0x1000
#define XAR_SB_UB1				0x2000
#define XAR_SB_UB2				0x4000
#define XAR_SB_UB3				0x8000
/*\}*/

// Parameter 2: 
/* Waveform Execution Status */
/*\{*/
#define XAR_WS_ERROR			-3			// Error condition will not permit execution, must restart
#define XAR_WS_DETACHED			-2			// Dettached, PosiProL not configured to execute
#define XAR_WS_WAIT_EXE			-1			// Configured, now waiting for execute command
#define XAR_WS_RUNNING			0			// Waveform is running (No errors)
#define XAR_WS_RUN_REDUCED		1			// Waveform is running, but at a reduced rate, due to excessive speed
#define XAR_WS_DONE_OK			2			// Completed Waveform without problems (new execute to start)
#define XAR_WS_DONE_REDUCED		3			// Completed Waveform, but had to reduced speed slower (new execute to re-start)
#define XAR_WS_HALTED			10			// Waveform is has been halted
/*\}*/

// Parameter 3:
/* Waveform Mode */
/*\{*/
#define XAR_WM_BYPASS			1			// DAC Direct mode
#define XAR_WM_XYTABLE			2			// used for Function Generator and Arbitrary Waveform Mode
#define XAR_WM_XTABLE			3			// Not used 
#define XAR_WM_RASTER			4			// Raster Mode
#define XAR_WM_DETACH			-1
/*\}*/

/* Some useful Defines */
/*\{*/
#define SYS_CLOCK				20000000	//*--system clock = 20 Mhz 
#define XAR_RS_MAXPTS			30000		// Maximum number of points in Raster
#define XAR_FG_MAXPTS			30000		// Maximum number of points in function generator table
#define XAR_AW_MAXPTS			50000
#define XAR_REPEAT_MAX			0xFFFFFFFF	// Maximum possible number of repeats
/*\}*/

//#####################################################
// ERROR CODES
//#####################################################
/*=====================================================================*/
/*! \defgroup ErrorCodes Error Codes 
  The error code is set only by xarCommand()

  Error codes that equal to or less than 0xff are error codes reported
  by the Measurement System itself.  Error codes that are between 1000 and 1037 are
  errors that are reported by the XarTraX system. Error codes greater than 0x0CD0 are reported
  by the host computer.
*/
/*\{*/
#define XAR_OKAY					0x00    /*!<\brief No error */ 
#define XAR_SWAPERROR				1000	/*!<\brief set up coordinates swap system error. */ 
#define XAR_NOINIT					1001	/*!<\brief Initialization has not been done */
#define XAR_TABLEEXCEEDED			1002	/*!<\brief Table Size exceeded */
#define XAR_TABLETOOSMALL			1003	/*!<\brief Table Size Too Small */
#define XAR_NOINTERRUPT				1004	/*!<\brief No Interrupt Specified for the mode */
#define XAR_REPEATNEGATIVE			1005	/*!<\brief Repeat number must be larger than one */
#define XAR_REPEATTOOLARGE			1006	/*!<\brief Set the repeat number too large */
#define XAR_RASTERERRORX1			1007 	/*!<\brief Raster exceeds X field */
#define XAR_RASTERERRORX2			1008 	/*!<\brief Raster exceeds X field */
#define XAR_RASTERERRORY1			1009 	/*!<\brief Raster exceeds Y field */
#define XAR_RASTERERRORY2			1010 	/*!<\brief Raster exceeds Y field */
#define XAR_RASTERERRORX3			1011 	/*!<\brief Raster exceeds X field */
#define XAR_RASTERERRORX4			1012 	/*!<\brief Raster exceeds X field */
#define XAR_RASTERERRORY3			1013 	/*!<\brief Raster exceeds Y field */
#define XAR_RASTERERRORY4			1014 	/*!<\brief Raster exceeds Y field */
#define XAR_TABLETOOBIG				1015 	/*!<\brief Size of table is bigger than permitted */
#define XAR_SETTLEERROR				1016 	/*!<\brief Specified Settle time leaves error */
#define XAR_STEPSIZETOOBIG			1017 	/*!<\brief Specified Stepsize is bigger than field */
#define XAR_STEPSIZENEGATIVE		1018 	/*!<\brief Specified Stepsize is bigger than entire field (negative) */
#define XAR_YLINESTOOLARGE			1019 	/*!<\brief Number of Y lines is excessive */
#define XAR_YLINESTOOSMALL			1020 	/*!<\brief Number of Y lines is < 1 */
#define XAR_DACTOOFAST				1021 	/*!<\brief Update DAC too fast */
#define XAR_DACTOOSLOW				1022 	/*!<\brief Update DAC too slow */
#define XAR_WAVEFORMERROR			1023 	/*!<\brief Unknown Waveform type */
#define XAR_WAVEFORMCLIPPED			1024 	/*!<\brief Amplitude and offset is such that the waveform is clipped */
#define XAR_AMPLITUDETOOLARGE		1025 	/*!<\brief Amplitude is too large */
#define XAR_AMPLITUDETOOSMALL		1026 	/*!<\brief Amplitude is small */
#define XAR_OFFSETERROR				1027 	/*!<\brief Offset is excessive */
#define XAR_AMPLITUDEERROR			1028 	/*!<\brief Amplitude is exceeded */
#define XAR_DUTYCYCLEOOR			1029 	/*!<\brief Duty Cycle is out-of-range */
#define XAR_DUTYCYCLEOOR2			1030 	/*!<\brief Duty Cycle is out-of-range */
#define XAR_PHASESHIFTOOR			1031 	/*!<\brief X or Y Phase Shift is out-of-range */
#define XAR_PHASESHIFTOOR2			1032 	/*!<\brief X or Y Phase Shift is out-of-range */
#define XAR_TABLETOOBIG2			1033 	/*!<\brief Table size is bigger than allowed */
#define XAR_TABLETOOSMALL2			1034 	/*!<\brief Table Size too small */
#define XAR_UPDATETOOSLOW			1035 	/*!<\brief Data Update rate too slow */
#define XAR_UPDATETOOFAST			1036 	/*!<\brief Data Update rate too fast */
#define XAR_CHECKTABLE				1037 	/*!<\brief Check the table size before read */

#define XAR_FILEERROR				0x0CD0  /*!<\brief File Error */ 
/*\}*/



/*=====================================================================*/
// Core
/*=====================================================================*/
 char *	xarCommandVA(polaris *pol, const char *format, va_list ap);
 char *	xarCommand(polaris *pol, const char *format, ...);
 void	ftoa(double source, char szOutString[]);

/*=====================================================================*/
// INITIALIZATION COMMANDS
/*=====================================================================*/
 char * xarRESET(polaris *pol);
 char * xarINIT(polaris *pol);
 char * xarCLOSE(polaris *pol);
 int xarProbe(const char *device);

/*=====================================================================*/
// GENERAL COMMANDS 
/*=====================================================================*/
 char * xarReadPosTable(polaris *pol,int tableSelect,	int dataFormat);
 char * xarSizePosTable(polaris *pol,int *TableSize);
 char * xarGetError(polaris *pol,	char * message);

/*=====================================================================*/
//SCANNER CONTROL COMMANDS
/*=====================================================================*/
 char * xarExecute(polaris * pol);

/*=====================================================================*/
// CONFIGURATION COMMANDS 
/*=====================================================================*/
 char * xarCInterrupt(polaris *pol,	int interruptNumber);
 char * xarXYSwap(polaris *pol,		int xySwap);

/*=====================================================================*/
//SCANNER FIELD CALIBRATION COMMANDS 
/*=====================================================================*/
 char * xarCalSetDeg(polaris *pol,	int xyChannelSelect, double highCalDegree,double midCalDegree);
 char * xarCalSetVol(polaris *pol,	int xyChannelSelect, double highCalVolt,double midCalVolt);
 char * xarCalLimits(polaris *pol,	int xyChannelSelect, int selectUnits,	double *minimum,		double *maximum);
 char * xarDegToDac(polaris *pol,	int xyChannelSelect, double degree,		double *equivalentDAC);
 char * xarDegToVolt(polaris *pol,	int xyChannelSelect, double degree,		double *equivalentVolt);
 char * xarVoltToDeg(polaris *pol,	int xyChannelSelect, double volt,		double *equivalentDeg);
 char * xarVoltToDac(polaris *pol,	int xyChannelSelect, double volt,		double *equivalentDAC);
 char * xarDacToVolt(polaris *pol,	int xyChannelSelect, double DACvalue,	double *equivalentVolt);
 char * xarDacToDeg(polaris *pol,	int xyChannelSelect, double DACvalue,	double *equivalentDeg);

/*=====================================================================*/
// LASER CONTROL COMMANDS 
/*=====================================================================*/
 char * xarVisibleOn(polaris *pol);
 char * xarVisibleOff(polaris *pol);
 char * xarInfraOn(polaris *pol);
 char * xarInfraOff(polaris *pol);

/*=====================================================================*/
//FUNCTION GENERATOR WAVEFORM COMMANDS
/*=====================================================================*/
 char * xarFType(polaris * pol,		int xyChannelSelect, int waveformType);
 char * xarFDutyCycle(polaris *pol, int xyChannelSelect, double dutyCycle);
 char * xarFTableSize(polaris *pol, int tableSize);
 char * xarFPhase(polaris *pol,		double phaseDifference);
 char * xarFTimebpts(polaris *pol,	double time);
 char * xarFAmpOff(polaris *pol,	int xyChannelSelect, int selectUnits,	double amplitude, double offset);
 char * xarFFreq(polaris *pol,		int desiredFrequency);
 char * xarFComplete(polaris *pol);

/*=====================================================================*/
//ARBITRARY WAVEFORM MODE COMMANDS 
/*=====================================================================*/
 char * xarAPosition(polaris *pol,	int Start, int End, int selectUnits, char * data);
 char * xarAIPosition(polaris *pol, int xPositionTable, int yPositionTable, char * data);
 char * xarARepeat(polaris *pol,	unsigned num);
 char * xarATimebpts(polaris *pol,	double time);
 char * xarAPeriod(polaris *pol,	int desiredPeriod);
 char * xarAComplete(polaris *pol);
 int	xarALaserProject(polaris *pol, char *filename, double time, unsigned repeats, double maxAngle);
 int	xarALaserProjectDeg(polaris *pol, char *filename, double time, unsigned repeats);
 int	xarALaserProjectXYZ(polaris *pol, char *filename, double time, unsigned repeats, double _OHAT[3], double transform[4][4], vtkMatrix4x4 * worldMatrix = NULL);


/*=====================================================================*/
//DAC DIRECT MODE COMMANDS
/*=====================================================================*/
 char * xarDPosition(polaris * pol, int xyChannelSelect,int selectUnits,	double position,		int visible,			int invisible);
 char * xarSetDegPos(polaris *pol,	double XValue,		double YValue);
 char * xarDComplete(polaris *pol);

/*=====================================================================*/
//RASTER MODE COMMANDS
/*=====================================================================*/
 char * xarRXStart(polaris *pol,	int selectUnits,	double xstart);
 char * xarRYStart(polaris *pol,	int selectUnits,	double ystart);
 char * xarRPeriod(polaris *pol,	double timebptsMsec,int rampPts,		int recovPts);
 char * xarRSettle(polaris *pol,	double settleMsec);
 char * xarRNXStep(polaris *pol,	int selectUnits,	double xStepSize);
 char * xarRNYStep(polaris *pol,	int selectUnits,	double yStepSize);
 char * xarRNYLines(polaris *pol,	int numYLines);
 int	xarRasterScan(polaris *pol,	char *OutFileName,	double XLength,		double YLength,	double Increment, int maxRange, int minRange);
 int	xarRasterScan2(polaris *pol, char *OutFileName, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, double Increment, int maxRange, int minRange);

/*=====================================================================*/
// MISC Functions
/*=====================================================================*/
 int	xarLoadFile(char *Filename,	MaxMinValue *mmv, unsigned &TotalPoints, bool Loaded = false);
 bool	getNextPoint(double *x,	double *y, MaxMinValue *mmv, double maxAngle, FILE *file);
 void	xarAPositionBuildCommand(int &count, char data[], double x, double y, int vi, int invi);
 char * xarUploadDataTOSBC(	polaris *pol, int selectUnits, double xPositionTable[], double yPositionTable[], int sizeofPositionTable);
 char * xarReadFromPolaris(polaris *pol, double trans[4]);
 void	xarFindOHAT(polaris * pol, double _OHAT[3]);
 void   xarPositionToAngle(double X, double Y, double Z, double _OHAT[3], double &xAngle, double &yAngle, vtkMatrix4x4 * worldMatrix = NULL);

//void xarPrintAngles(polaris * pol);


#ifdef __cplusplus
 extern "C" {
//BTX
 //char * xarGetError(polaris *pol,	int error);
//ETX
 char * xarFDutyCycleA(polaris *pol, int xyChannelSelect, double dutyCycle,	double *actualDutyCycle);
 char * xarFTableSizeA(polaris *pol, int tableSize,		 int *actualTableSize);
 char * xarFPhaseA(polaris *pol,		double phaseDifference, double *actualPhaseDifference);
 char * xarFTimebptsA(polaris *pol,	double time,		 double *actualTime);
 char * xarFAmpOffA(polaris *pol,	int xyChannelSelect, int selectUnits,	double amplitude, double offset,	double * actualAmplitude, double *actualOffset);
 char * xarFFreqA(polaris *pol,		int desiredFrequency,double *bestFitFrequency, double *optimizedTableSize,	double *optimizedTimebptsmsec);

 char * xarATimebptsA(polaris *pol,	double time, double *actualTime);
 char * xarAPeriodA(polaris *pol,	int desiredPeriod, double *bestFitPeriod, double *optimizedTableSize, double *optimizedTimebptsmsec);
 
 char * xarDPositionA(polaris *pol,	int xyChannelSelect,int selectUnits,	double position,		double *actualPosition, int visible = 0, int invisible = 0);
 char * xarSetDegPosA(polaris *pol,	double XValue,		double YValue,		double *actualXPosition,double *actualYPosition);

 char * xarRPeriodA(polaris *pol,	double timebptsMsec,int rampPts,		int recovPts,	double *actualTimebptsmsec);
 char * xarRSettleA(polaris *pol,	double settleMsec,	double *actualsettleMsec);
 char * xarRNXStepA(polaris *pol,	int selectUnits,	double xStepSize,	double *actualXStepSize);
 char * xarRNYLinesA(polaris *pol,	int numYLines,		double *actualNumYLines);
 char * xarRNYStepA(polaris *pol,	int selectUnits,	double yStepSize,	double *actualYStepSize);
 }
#endif


#endif /* XARTRAX_H */

