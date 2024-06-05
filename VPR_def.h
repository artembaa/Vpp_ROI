#ifndef  VPR_DEF_H
#define  VPR_DEF_H

#include "VPR.h"

#ifndef  MATH_H_BC
#define  MATH_H_BC
#include <math.h>
#endif

#ifndef  DOS_H_BC
#define  DOS_H_BC
#include <dos.h>
#endif

#ifndef  ALLOC_H_BC
#define  ALLOC_H_BC
#include <malloc.h>
#endif

#ifndef  MEM_H_BC
#define  MEM_H_BC
#include <memory.h>
#endif

#ifndef  STDLIB_H_BC
#define  STDLIB_H_BC
#include <stdlib.h>
#endif

#ifndef  TIME_H_BC
#define  TIME_H_BC
#include <time.h>
#endif

#if defined(_WIN64)
typedef unsigned __int64 UINT_PTR;
#else 
typedef unsigned int UINT_PTR;
#endif

//*********************************************************
//			        ОПРЕДЕЛЕНИЯ ТИПОВ
//*********************************************************
typedef unsigned char    UC;	 //8bit
typedef signed char      SC;	 //8bit
typedef unsigned short   UI;	 //16bit
typedef signed short     SI;	 //16bit
typedef unsigned long    UL;	 //32bit
typedef signed long      SL;	 //32bit
typedef float            FLOAT;	 //32bit
typedef double           DOUBLE; //64bit

#define TRUE   1
#define FALSE  0

//*********************************************************
//			 ОПРЕДЕЛЕНИЯ, СВЯЗАННЫЕ С ОБРАБОТКОЙ
//*********************************************************
#define DIV_N(divident,divisor) ((divident)/(divisor))

// Определения для входного изображения (RI - Rastr Input)
#define   RI_X			 1008 /* пикселей, должен быть ЧЕТНЫМ и кратным S_ITEM_SIZE (7) */
#define   RI_Y			 1008 /* пикселей, должен быть ЧЕТНЫМ и кратным S_ITEM_SIZE (7) */
#define   RI_XY			 (RI_X*RI_Y) 
#define   RI_MIN		 96 

// Определения для матрицы (S - Stream matrix)
#define   S_EXPANDER    3 /* ячейки матрицы */
#define   S_ITEM_SIZE   7 /* размер ячейки матрицы в пикселях */
#define   S_ITEM_HSIZE  (S_ITEM_SIZE/2)
#define   S_X			((RI_X/S_ITEM_SIZE)+(S_EXPANDER*2)) 
#define   S_Y			((RI_Y/S_ITEM_SIZE)+(S_EXPANDER*2))
#define   S_XY			(S_X*S_Y)
#define   S_OffsetToCoord(Offset,X,Y) { Y=DIV_N(Offset,S_X); X=(Offset)-((Y)*S_X); }
#define   S_CoordToOffset(X,Y) (((Y)*S_X)+(X))

// Определения для расширенного изображения (R - Rastr)
#define   R_EXPANDER    (S_EXPANDER*S_ITEM_SIZE)
#define   R_X			(RI_X+R_EXPANDER*2)
#define   R_Y			(RI_Y+R_EXPANDER*2)   
#define   R_XY			(R_X*R_Y)
#define   R_CoordToOffset(X,Y) (((Y)*R_X)+(X))							
void	  R_OffsetToCoord(UL index,SI *X,SI *Y);

#define   R_To_S_Coord(Coord) (DIV_N(Coord,S_ITEM_SIZE)) 
#define   S_To_R_Coord(Coord) ((Coord)*S_ITEM_SIZE)

// Предикаты (битовые признаки)
#define  IMAGE_PRED			0x0001
#define  REGION3_PRED		0x0002
#define  FNGREG_PRED		0x0004
#define  TMP1_PRED			0x1000
#define  TMP2_PRED			0x2000
#define  TMP3_PRED			0x4000
#define  CHANGE_PRED		0x8000

#define  INCL_NUMBER          16
#define  HALF_INCL_NUMBER     (INCL_NUMBER/2) /* 8 */

#define  R_CUTSET_HSIZE       21
#define  R_CUTSET_SIZE        (R_CUTSET_HSIZE+1+R_CUTSET_HSIZE) /* 43 */

#define  S_CUTSET_HSIZE		  7
#define  S_CUTSET_SIZE		  (S_CUTSET_HSIZE*2+1) /* 15 */

#define  STREAM_REGION_9_SIZE   61
#define  STREAM_REGION_7_SIZE   37
#define  STREAM_REGION_7_HSIZE  (STREAM_REGION_7_SIZE/2) /* 18 */
#define  STREAM_REGION_5x5_SIZE 25
#define  STREAM_REGION_5x5_HSIZE (STREAM_REGION_5x5_SIZE/2) /* 12 */
#define  STREAM_REGION_5_SIZE   21
#define  STREAM_REGION_5_HSIZE  (STREAM_REGION_5_SIZE/2) /* 10 */
#define  STREAM_REGION_3_SIZE   9
#define  STREAM_REGION_3_HSIZE  (STREAM_REGION_3_SIZE/2) /* 4 */
#define  STREAM_REGION_3R_SIZE  5
#define  STREAM_REGION_3R_HSIZE (STREAM_REGION_3R_SIZE/2) /* 2 */
#define  STREAM_REGION_1_SIZE   1

#define  REDEF_MARGIN_9  4
#define  REDEF_SIZE_9    (REDEF_MARGIN_9*2+1)
#define  REDEF_MARGIN_7  3
#define  REDEF_SIZE_7    (REDEF_MARGIN_7*2+1)
#define  REDEF_MARGIN_5  2
#define  REDEF_SIZE_5    (REDEF_MARGIN_5*2+1)
#define  REDEF_MARGIN_3  1
#define  REDEF_SIZE_3    (REDEF_MARGIN_3*2+1)
#define  REDEF_MARGIN_1  0
#define  REDEF_SIZE_1    (REDEF_MARGIN_1*2+1)

#define  LINE_STREAM_CORRELATOR_SIZE  5

#define  SetPred(AdrP,maskPred) ( *(AdrP) |=  (maskPred) )
#define  GetPred(AdrP,maskPred) ( *(AdrP) &   (maskPred) )
#define  DelPred(AdrP,maskPred) ( *(AdrP) &= ~(maskPred) )

#define  Incl_16_To_128(Incl) ((Incl)<<3) 
#define  Incl_128_To_16(Incl) ((Incl)>>3) 

#define  DEGREE_360			  256 /* 1 unit = 1.4 degrees */
#define  DEGREE_270			  (DEGREE_180+DEGREE_90)
#define  DEGREE_180			  (DEGREE_360/2)
#define  DEGREE_135			  (DEGREE_90+DEGREE_45)
#define  DEGREE_90			  (DEGREE_360/4)
#define  DEGREE_60			  (DEGREE_360/6)
#define  DEGREE_45			  (DEGREE_360/8)
#define  DEGREE_22			  (DEGREE_360/16)
#define  DEGREE_11			  (DEGREE_360/32)
#define  DEGREE_5			  (DEGREE_360/64)

#define  MAX_VALUE_UC			0xFF
#define  MAX_VALUE_UI			0xFFFF
#define  MAX_VALUE_UL			0xFFFFFFFF
#define  MAX_VALUE_SC			0x7F
#define  MAX_VALUE_SI			0x7FFF
#define  MAX_VALUE_SL			0x7FFFFFFF

typedef UC * R_PTR;

typedef struct
{
	UI  Pred;
}
	PRED_SP;

typedef struct
{
	UC I;
	UC NegDIncl;
	UC PosDIncl;
}
	INCL_SP;

typedef struct
{
	SC     I1;    
	SC     I2;    
	SC     I3;    
	SC     I4;    
}
	WORK_INCL_SP;

typedef struct
{
	SI  X;
	SI  Y;
}
POINT_COORDS;


//*********************************************************
//			       ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
//*********************************************************

extern    SI  S_Left,S_Right,S_Top,S_Bottom;
extern    SL  S_Offset;

extern    SI  R_Left,R_Right,R_Top,R_Bottom;
extern    SL  R_Offset;

extern   SI   R_Index[INCL_NUMBER*R_CUTSET_HSIZE];
extern   SI  S_Index[INCL_NUMBER*S_CUTSET_HSIZE];
extern   SI   S_RegionIndex[STREAM_REGION_9_SIZE];
extern   SI   S_LineIndex[HALF_INCL_NUMBER*LINE_STREAM_CORRELATOR_SIZE];

#define INCL_TABLE_64_SIZE 39
extern  UC    InclTable64[INCL_TABLE_64_SIZE*INCL_TABLE_64_SIZE];


//*********************************************************
//			       ВНУТРЕННИЕ ФУНКЦИИ
//*********************************************************/
UC  GetIncl(SI  X1,
			SI  Y1,
			SI  X2,
			SI  Y2);
UC DeltaIncl(UC  Incl1,
			UC  Incl2,
			UC  Range); 
UC MedIncl( UC Incl1,
			UC Incl2,
			UC Range); 
			       
UC ShiftIncl(SI Incl,
			SC dIncl,
			UC Range); 

UC  GetDir(SI  X1,
			SI  Y1,
			SI  X2,
			SI  Y2); 

SI  DeltaDir(SI Dir1,
			SI Dir2);

SI  DeltaDir2(SI Dir1,
				SI Dir2 );

SI MediumDir(SI Dir1,
			SI Dir2);

SI ShiftDir(SI Dir,
		    SI DDir);

UI GetDist(SI X1,SI Y1,
		    SI X2,SI Y2);

SL GetDist2(SI X1,SI Y1,
		    SI X2,SI Y2);

void FaultError(char *);

SI VPRInit(VPR_IMAGE_DESCRIPTOR *VPRDescriptor,
			PRED_SP *PredPtr);

SI VPRImagePosition(SI      ImageSizeX,
					SI      ImageSizeY,
					SI		 *ImageCoordX,
					SI		 *ImageCoordY);

SL   VPRImportImage(SI   ImageX,
					SI   ImageY,
					UC   *ImageBuffer,
					UC   *InternalExtendedImageBuffer);

SI  VPRProc(VPR_IMAGE_DESCRIPTOR *VPRDescriptor,
			R_PTR     RPtr1,
			UC	      *WRPtr);

UC	PointParallelIntegration_15_VPR(UC *rptr);

void DefPoint_VPR(PRED_SP *pptr, UC *Count, UC *CountAll, UC *CountChange, UI  Pred, UI Value);

UC   DefPointType_VPR(UC Count, UC CountChange);

#define  Incl_8_To_128(Incl) ((Incl)<<4) 
#define  Incl_128_To_8(Incl) ((Incl)>>4) 

typedef struct
{
	UI Start;
	UI End;
}
EDGE_LINE_SEGMENT_INDEX;
#define EDGE_LINE_SEGMENT_INDEX_NUMBER 20

#define MAX_END_POINT 4
typedef struct
{
	UI Name;
	UI Incl;
	POINT_COORDS EndPoint[MAX_END_POINT];
	UI IndexNumber;
	EDGE_LINE_SEGMENT_INDEX IndexList[EDGE_LINE_SEGMENT_INDEX_NUMBER];
	int SumM;
}
EDGE_LINE_SEGMENT;
#define EDGE_LINE_SEGMENT_LIST_SIZE  100

extern  SI  VPR_S_Left, VPR_S_Right, VPR_S_Top, VPR_S_Bottom;
extern  SL  VPR_S_Offset;
extern  SI  VPR_R_Left, VPR_R_Right, VPR_R_Top, VPR_R_Bottom;
extern  SL  VPR_R_Offset;

#endif
