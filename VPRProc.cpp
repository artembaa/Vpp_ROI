#include "VPR_def.h"

#ifndef VPR_RELEASE
#define ACTIVATE_HELPER
extern struct _timeb t1, t2;
#endif

typedef struct
{
	UC C[HALF_INCL_NUMBER]; /* контраст для различных ориентаций линейной выборки [0-255] */
}
C_SP;

typedef struct
{
	UI  M[HALF_INCL_NUMBER]; /* мера наилучшей ориентации для различных ориентаций линейной выборки [0-255] */
}
M_SP;

PRED_SP  *VPR_EdgeSegmNamePtr;
C_SP  *VPR_ContrPtr;
M_SP   *VPR_MPtr;
WORK_INCL_SP  *VPR_InclPtr;
int Histogram[256];

SI          vpr_poffset1, vpr_poffset2, vpr_poffset3, vpr_poffset4, vpr_poffset5, vpr_poffset6, vpr_poffset7, vpr_poffset8,
			vpr_noffset1, vpr_noffset3, vpr_noffset5;

#define MIN_M				 26
#define SEGM_FRONT_SIZE      ((S_X+S_Y)*2) 


SI  VPRProc(VPR_IMAGE_DESCRIPTOR *VPRDescriptor,
			R_PTR				RPtr1,
			UC					*WRPtr) 
{
	PRED_SP		*PredPtr;

	R_PTR		rptr;
	PRED_SP     *bpredptr,*predptr;
	C_SP		*bcontrptr;
	M_SP        *bmptr;
	WORK_INCL_SP     *binclptr,*inclptr;

	EDGE_LINE_SEGMENT *BELS, *ELS;
	POINT_COORDS *BEP1, *BEP2, *EP1, *EP2;

	UL			brindex,rindex,bsindex,sindex,index,bnindex,nindex;
	SI          rx, ry, sx, sy, i, n,bs, s, bep1, bep2, ep1, ep2, Iteration, s1, s2, offset, offset2, P;
	SI          Incl, LIncl, RIncl, Normal, BNormal, *BPIPtr, *BNIPtr, 
				Count, Count2, CountN, CountM, Contr;
	SL			MaxM,M;
	UC			RepeatFlag;

	UI			ip,ap,cindex,*SegmentIndex;

	SL          ThresholdM;
	SL          VPR_SumM;


	
	PredPtr = (PRED_SP *)(WRPtr);
	VPR_ContrPtr=(C_SP  *)((PredPtr + S_XY));
	VPR_MPtr=(M_SP   *)((VPR_ContrPtr+S_XY));
	SegmentIndex=(UI *)(((VPR_MPtr + S_XY)));
	VPR_InclPtr=(WORK_INCL_SP  *)((SegmentIndex+SEGM_FRONT_SIZE));
	EDGE_LINE_SEGMENT *EdgeLineSegmentList= (EDGE_LINE_SEGMENT *)((VPR_InclPtr + S_XY));
	VPR_EdgeSegmNamePtr = (PRED_SP  *)((EdgeLineSegmentList + EDGE_LINE_SEGMENT_LIST_SIZE));

	UC *WorkRegionEndPointer=(UC *)(VPR_EdgeSegmNamePtr + S_XY);
	if( ((UINT_PTR)WorkRegionEndPointer-(UINT_PTR)WRPtr) > 2*R_XY )
	{
		#ifdef ACTIVATE_HELPER
		FaultError("Недостаточно памяти");
		#endif
		return 0;
	}

	//********************************************
	//	     Инициализация слоев матрицы
	//********************************************

	if (!VPRInit(VPRDescriptor, PredPtr))
		return VPR_ERROR;
	memset(VPR_ContrPtr,0,S_XY*sizeof(C_SP));
	memset(VPR_MPtr,0,S_XY*sizeof(M_SP));
	memset(VPR_InclPtr,HALF_INCL_NUMBER,S_XY*sizeof(WORK_INCL_SP));

#ifdef ACTIVATE_HELPER
	_ftime(&t1);
#endif

	//***************************************************************************************
	//		    Формирование контраста для различно-ориентированных линейных выборок  	     
	//****************************************************************************************
	for(Incl=0;Incl < HALF_INCL_NUMBER;Incl++)
	{
		/* Параллельная интеграция */
		BPIPtr=R_Index+(UL)(Incl<<1)*R_CUTSET_HSIZE;
		vpr_poffset1=*(BPIPtr+0);
		vpr_poffset2=*(BPIPtr+1);
		vpr_poffset4=*(BPIPtr+3);
		vpr_poffset6=*(BPIPtr+5);
		vpr_poffset8=*(BPIPtr+7);

		/* Перпендикулярная выборка */
		Normal=Incl+HALF_INCL_NUMBER/2;
		if( Normal >= HALF_INCL_NUMBER )
			Normal-=HALF_INCL_NUMBER;
		BNIPtr=R_Index+(UL)(Normal<<1)*R_CUTSET_HSIZE;
		vpr_noffset1=*(BNIPtr+0);
		vpr_noffset3=*(BNIPtr+2);
		vpr_noffset5=*(BNIPtr+4);

		for(sy=VPR_S_Top,bsindex=VPR_S_Offset,ry=S_To_R_Coord(sy);
			sy <= VPR_S_Bottom;
			sy++,bsindex+=S_X,ry+=S_ITEM_SIZE)
		{
			for(sx=VPR_S_Left,sindex=bsindex,rx=S_To_R_Coord(sx),brindex=R_CoordToOffset(rx,ry);
			    sx <= VPR_S_Right;				
				sx++,sindex++,brindex+=S_ITEM_SIZE)
			{
				predptr=PredPtr+sindex;
				rindex = brindex +(S_ITEM_HSIZE)*R_X + (S_ITEM_HSIZE);
				rptr=RPtr1+rindex;
				
				int MinLight = 255;
				int MaxLight = 0;
				int SumLight = 0;

				P = PointParallelIntegration_15_VPR(rptr - vpr_noffset5);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				P = PointParallelIntegration_15_VPR(rptr - vpr_noffset3);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				P = PointParallelIntegration_15_VPR(rptr - vpr_noffset1);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				P = PointParallelIntegration_15_VPR(rptr + vpr_noffset1);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				P = PointParallelIntegration_15_VPR(rptr + vpr_noffset3);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				P = PointParallelIntegration_15_VPR(rptr + vpr_noffset5);
				if (MinLight > P) MinLight = P;
				if (MaxLight < P) MaxLight = P;
				SumLight += P;

				Contr = (UC)(MaxLight - MinLight);

				(VPR_ContrPtr+sindex)->C[Incl]=(UC)Contr;

			}
		}
	}


#ifdef ACTIVATE_HELPER
//_ftime( &t2 );
//if( !Helper("1", DiffTimeInMilliseconds(&t1,&t2),0,VPRPROC_DEMO_) ) return 0;
#endif

	//**********************************************************
	//		 Формирование матрицы наклонений <I1, I2>		 
	//**********************************************************
	VPR_SumM = 0;
	CountM = 0;
	for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
	{
		for(sx=VPR_S_Left,sindex=bsindex;sx <= VPR_S_Right;sx++,sindex++)
		{
			bpredptr=PredPtr+sindex;			
			bcontrptr=VPR_ContrPtr+sindex;
			bmptr=VPR_MPtr+sindex;
			inclptr=VPR_InclPtr+sindex;

			if (!GetPred(&bpredptr->Pred, REGION3_PRED))
				continue;

			for (Incl = 0; Incl < HALF_INCL_NUMBER; Incl++)
			{
				BPIPtr = S_LineIndex + Incl*LINE_STREAM_CORRELATOR_SIZE;
				Count = 1;
				Count2 = 1;
				Contr = bcontrptr->C[Incl];
				for (i = 0; i < LINE_STREAM_CORRELATOR_SIZE / 2; i++)
				{
					offset = *(BPIPtr + i);

					if (GetPred(&(bpredptr + offset)->Pred, REGION3_PRED))
					{
						Count++;
						Contr += (bcontrptr + offset)->C[Incl];
					}
					if (GetPred(&(bpredptr - offset)->Pred, REGION3_PRED))
					{
						Count++;
						Contr += (bcontrptr - offset)->C[Incl];
					}
				}
				bmptr->M[Incl] = Contr/Count;
			}

			MaxM=0;
			for(Incl=0;Incl < HALF_INCL_NUMBER;Incl++)
			{
				M = bmptr->M[Incl];
				if (MaxM < M)
				{
					MaxM = M;
					inclptr->I1 = (SC)Incl;
				}
			}

			if( inclptr->I1 == HALF_INCL_NUMBER )
				continue;

			M = (MaxM*3) >> 2; //M=(MaxM)>>1; 

			RIncl=inclptr->I1+1;
			if( RIncl >= HALF_INCL_NUMBER ) RIncl=0;
			LIncl=inclptr->I1-1;
			if( LIncl < 0 ) LIncl= HALF_INCL_NUMBER-1;

			if( bmptr->M[RIncl] >= M && bmptr->M[LIncl] <  M )
				inclptr->I2=(SC)RIncl;
			else
			if( bmptr->M[LIncl] >= M && bmptr->M[RIncl] <  M )
				inclptr->I2=(SC)LIncl;

			// включение в качестве альтернативы наилучшего левого/правого соседа
			if( bmptr->M[LIncl] >= bmptr->M[RIncl] ) RIncl=inclptr->I1;
			else									 LIncl=inclptr->I1;

			VPR_SumM += bmptr->M[inclptr->I1];
			CountM++;
		}
	}


#ifdef ACTIVATE_HELPER
//_ftime( &t2 );
//if( !Helper("2", DiffTimeInMilliseconds(&t1,&t2), 0, VPRPROC_DEMO_) )  return 0;
#endif

	if (CountM)
		VPR_SumM = VPR_SumM/CountM;
	else
		return 0;

	//*************************************************************************
	//			Первичное обнаружение краев в виде линейных сегментов									  
	//*************************************************************************
	ThresholdM = VPR_SumM/3; //ThresholdM=VPR_SumM>>1;
	if( ThresholdM < MIN_M ) ThresholdM=MIN_M;

	for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
		for(sx=VPR_S_Left,sindex=bsindex;sx <= VPR_S_Right;sx++,sindex++)
		{
			bpredptr=PredPtr+sindex;
			if (!GetPred(&bpredptr->Pred, IMAGE_PRED)) //if (!GetPred(&bpredptr->Pred, REGION3_PRED))
				continue;

			binclptr=VPR_InclPtr+sindex;
			if( binclptr->I1 >= HALF_INCL_NUMBER )
				continue;

			bmptr = VPR_MPtr + sindex;

#ifdef ACTIVATE_HELPER
			if (sx == 51 && sy == 19)
				int FFF_DEBUG = 0;
#endif

			BPIPtr = S_LineIndex + binclptr->I1*LINE_STREAM_CORRELATOR_SIZE;
			Count = 1;
			Count2 = 1;
			M = (bmptr)->M[binclptr->I1];
			for (i = 0; i < LINE_STREAM_CORRELATOR_SIZE / 2; i++)
			{
				offset = *(BPIPtr + i);

				#define MAX_DINCL 1
				if (GetPred(&(bpredptr + offset)->Pred, REGION3_PRED))
				{
					Count++;
					if (DeltaIncl(binclptr->I1, (binclptr + offset)->I1, HALF_INCL_NUMBER) <= MAX_DINCL
						||
						((binclptr + offset)->I2 < HALF_INCL_NUMBER) && DeltaIncl(binclptr->I1, (binclptr + offset)->I2, HALF_INCL_NUMBER) <= MAX_DINCL)
						Count2++;
				}
				if (GetPred(&(bpredptr - offset)->Pred, REGION3_PRED))
				{
					Count++;
					if (DeltaIncl(binclptr->I1, (binclptr - offset)->I1, HALF_INCL_NUMBER) <= MAX_DINCL
						||
						((binclptr - offset)->I2 < HALF_INCL_NUMBER) && DeltaIncl(binclptr->I1, (binclptr - offset)->I2, HALF_INCL_NUMBER) <= MAX_DINCL)
						Count2++;
				}
			}
			if ( Count >= LINE_STREAM_CORRELATOR_SIZE / 2 + 1 && Count2 >= Count - 1 &&	M >= ThresholdM )
				SetPred(&bpredptr->Pred, FNGREG_PRED);
		}

#ifdef ACTIVATE_HELPER
//if( !Helper("3",DiffTimeInMilliseconds(&t1,&t2),0,VPRPROC_DEMO_) )  return 0;
#endif

	//*************************************************************************
	//		  Определение линии края (скелетизация линейных сегментов)										  
	//*************************************************************************
	for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
		for(sx=VPR_S_Left,sindex=bsindex;sx <= VPR_S_Right;sx++,sindex++)
		{
			bpredptr=PredPtr+sindex;
			if (!GetPred(&bpredptr->Pred, FNGREG_PRED))
				continue;

			binclptr=VPR_InclPtr+sindex;
			if( binclptr->I1 >= HALF_INCL_NUMBER )
				continue;

			bmptr = VPR_MPtr + sindex;

#ifdef ACTIVATE_HELPER
			if (sx == 30 && sy == 20)
				int FFF_DEBUG = 0;
#endif
			Incl = binclptr->I1;

			Normal = Incl + HALF_INCL_NUMBER / 2;
			if (Normal >= HALF_INCL_NUMBER)
				Normal -= HALF_INCL_NUMBER;
			Normal *= 2; // *2 - S_Index имеет размерность INCL_NUMBER!
			BNIPtr = S_Index + (UL)(Normal)*S_CUTSET_HSIZE;


			int OtherPeakFound = 0;
			int LeftBound = 0, RightBound = 0;
			for (i = 0; i < S_CUTSET_HSIZE/2; i++)
			{
				offset = *(BNIPtr + i);

				if (!RightBound)
				{
					if (!GetPred(&(bpredptr + offset)->Pred, FNGREG_PRED))
						RightBound = 1;
					else
					if ((binclptr + offset)->I1 < HALF_INCL_NUMBER )
					{
						if (DeltaIncl((UC)Incl, (UC)(binclptr + offset)->I1, HALF_INCL_NUMBER) <= 1 )
						{
							if ((bmptr)->M[Incl] < (bmptr + offset)->M[(binclptr + offset)->I1])
							{
								OtherPeakFound = 1;
								break;
							}
						}
						else
							RightBound=1;
					}

				}

				if (!LeftBound)
				{
					if (!GetPred(&(bpredptr - offset)->Pred, FNGREG_PRED))
						LeftBound = 1;
					else
					if ((binclptr - offset)->I1 < HALF_INCL_NUMBER )
					{
						if (DeltaIncl((UC)Incl, (UC)(binclptr - offset)->I1, HALF_INCL_NUMBER) <= 1 )
						{
							if ((bmptr)->M[Incl] < (bmptr - offset)->M[(binclptr - offset)->I1])
							{
								OtherPeakFound = 1;
								break;
							}
						}
						else
							LeftBound =1;
					}
				}

			}
			if (OtherPeakFound)
				continue;
			SetPred(&bpredptr->Pred, CHANGE_PRED);
		}

#ifdef ACTIVATE_HELPER
//if( !Helper("4",DiffTimeInMilliseconds(&t1,&t2),0,VPRPROC_DEMO_) )  return 0;
#endif

for (P = 0, predptr = PredPtr; P < S_XY; P++, predptr++)
{
	if (GetPred(&predptr->Pred, CHANGE_PRED))
		DelPred(&predptr->Pred, CHANGE_PRED);
	else
		DelPred(&predptr->Pred, FNGREG_PRED);
}
#ifdef ACTIVATE_HELPER
//if (!Helper("5", DiffTimeInMilliseconds(&t1, &t2), 0, VPRPROC_DEMO_))  return 0;
#endif

	//*************************************************************************
	//		   Фильтрация точек линий краев (не утоньшенные линии)			 
	//*************************************************************************
	while(1)
	{
		for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
			for (sx = VPR_S_Left, sindex = bsindex; sx <= VPR_S_Right; sx++, sindex++)
			{
				bpredptr = PredPtr + sindex;
				if (!GetPred(&bpredptr->Pred, FNGREG_PRED))
					continue;

				binclptr = VPR_InclPtr + sindex;
				if (binclptr->I1 >= HALF_INCL_NUMBER)
					continue;

				if (GetPred(&(bpredptr - 1)->Pred, TMP2_PRED) || GetPred(&(bpredptr - S_X)->Pred, TMP2_PRED) || GetPred(&(bpredptr - S_X-1)->Pred, TMP2_PRED))
					continue;
				#ifdef ACTIVATE_HELPER
				if (sx == 47 && sy == 17)
					int FFF_DEBUG = 0;
				#endif
				UC c, ca, cc;
				DefPoint_VPR(bpredptr, &c, &ca, &cc, FNGREG_PRED, 0);
				if (c <= 2 && ca >= 3 && cc == 2)
				{
					SetPred(&(bpredptr)->Pred, TMP2_PRED);
					continue;
				}
			}

		RepeatFlag = 0;
		for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
			for (sx = VPR_S_Left, sindex = bsindex; sx <= VPR_S_Right; sx++, sindex++)
			{
				bpredptr = PredPtr + sindex;
				if (!GetPred(&bpredptr->Pred, TMP2_PRED))
					continue;

				DelPred(&(bpredptr)->Pred, TMP2_PRED | FNGREG_PRED);
				#ifdef ACTIVATE_HELPER
				SetPred(&(bpredptr)->Pred, CHANGE_PRED);
				#endif
				RepeatFlag = 1;
			}
		if (!RepeatFlag)
			break;
	}
#ifdef ACTIVATE_HELPER
	//if( !Helper("6", DiffTimeInMilliseconds(&t1,&t2), 0, VPRPROC_DEMO_) )  return 0;
	for (P = 0, predptr = PredPtr; P < S_XY; P++, predptr++)
			DelPred(&predptr->Pred, CHANGE_PRED);
#endif


	//*************************************************************************
	//				Фильтрация точек линий краев на основе наклонения								 
	//*************************************************************************
	for(sy=VPR_S_Top,bsindex=VPR_S_Offset;sy <= VPR_S_Bottom;sy++,bsindex+=S_X)
		for(sx=VPR_S_Left,sindex=bsindex;sx <= VPR_S_Right;sx++,sindex++)
		{
			bpredptr=PredPtr+sindex;
			if (!GetPred(&bpredptr->Pred, FNGREG_PRED))
				continue;

			binclptr=VPR_InclPtr+sindex;
			if( binclptr->I1 >= HALF_INCL_NUMBER )
				continue;

#ifdef ACTIVATE_HELPER
			if (sx == 47 && sy == 17)
				int FFF_DEBUG = 0;
#endif
			Count = 0;
			for (i = 1; i < STREAM_REGION_3_SIZE; i++)
			{
				offset = *(S_RegionIndex + i);
				if (!GetPred(&(bpredptr + offset)->Pred, FNGREG_PRED))
					continue;
				
				Count++;
				if (DeltaIncl(binclptr->I1, (binclptr + offset)->I1, HALF_INCL_NUMBER) <= 1)
				{
					int Xn, Yn;
					S_OffsetToCoord(sindex+offset, Xn, Yn);
					Incl = Incl_128_To_8(GetIncl(sx, sy, Xn, Yn));
					int dIncl = DeltaIncl((UC)binclptr->I1, (UC)Incl, HALF_INCL_NUMBER);
					if ((i <= STREAM_REGION_3R_SIZE && dIncl <= 1)
						|| 
						dIncl  <= 2)
						break;
				}

			}
			if (!Count || i < STREAM_REGION_3_SIZE)
				continue;

			SetPred(&bpredptr->Pred, CHANGE_PRED);
		}

#ifdef ACTIVATE_HELPER
//if( !Helper("7", DiffTimeInMilliseconds(&t1,&t2), 0, VPRPROC_DEMO_) )  return 0;
#endif
	for (P = 0, predptr = PredPtr; P < S_XY; P++, predptr++)
	{
		if (GetPred(&predptr->Pred, CHANGE_PRED))
			DelPred(&predptr->Pred, CHANGE_PRED| FNGREG_PRED);
	}



	//*************************************************************************
	//				Фильтрация точек ложных соединений линий краев
	//*************************************************************************
	for (sy = VPR_S_Top, bsindex = VPR_S_Offset; sy <= VPR_S_Bottom; sy++, bsindex += S_X)
		for (sx = VPR_S_Left, sindex = bsindex; sx <= VPR_S_Right; sx++, sindex++)
		{
			bpredptr = PredPtr + sindex;
			if (!GetPred(&bpredptr->Pred, FNGREG_PRED))
				continue;

			UC c, ca, cc;
			DefPoint_VPR(bpredptr, &c, &ca, &cc, FNGREG_PRED,0);
			UC Type = DefPointType_VPR(c, cc);
			if (Type == 3)  // точка соединения линий
			{
				binclptr = VPR_InclPtr + sindex;
				if (binclptr->I1 >= HALF_INCL_NUMBER)
					continue;

#ifdef ACTIVATE_HELPER
				if (sx == 51 && sy == 55)
					int FFF_DEBUG = 0;
#endif
				for (i = 1; i < STREAM_REGION_3_SIZE; i++)
				{
					offset = *(S_RegionIndex + i);
					if (!GetPred(&(bpredptr + offset)->Pred, FNGREG_PRED))
						continue;
					if (DeltaIncl(binclptr->I1, (binclptr + offset)->I1, HALF_INCL_NUMBER) > 2 )
						break;
				}
				if (i < STREAM_REGION_3_SIZE)
					continue;

				SetPred(&bpredptr->Pred, CHANGE_PRED);
			}
			else
			if (Type  == 1)  // точка окончания линии
				SetPred(&bpredptr->Pred, TMP2_PRED);
		}

#ifdef ACTIVATE_HELPER
	//if (!Helper("8", DiffTimeInMilliseconds(&t1, &t2), 0, VPRPROC_DEMO_))  return 0;
#endif
	// удаление точек окончаний соседних к точкам соединений
	for (P = 0, bpredptr = PredPtr; P < S_XY; P++, bpredptr++)
	{
		if (GetPred(&bpredptr->Pred, TMP2_PRED))// точка окончания линии
		{
			DelPred(&bpredptr->Pred, TMP2_PRED);// снять отметку точка окончания линии
			Count = 0;
			Count2 = 0;
			for (i = 1; i < STREAM_REGION_3R_SIZE; i++)
			{
				offset = *(S_RegionIndex + i);
				if (GetPred(&(bpredptr + offset)->Pred, CHANGE_PRED)) // точка соединения линий
				{
					offset2 = offset;
					Count++;
				}
				if (!GetPred(&(bpredptr + offset)->Pred, IMAGE_PRED)) //if (!GetPred(&(bpredptr + offset)->Pred, REGION3_PRED)) // border
					Count2++;
			}
			if( Count == 1 && Count2==0 )
			{
				DelPred(&bpredptr->Pred, FNGREG_PRED);// удалить точку окончания линии
				DelPred(&(bpredptr + offset2)->Pred, CHANGE_PRED);			 // снять отметку точка соединения линий
			}
		}
	}
#ifdef ACTIVATE_HELPER
	//if (!Helper("9", DiffTimeInMilliseconds(&t1, &t2), 0, VPRPROC_DEMO_))  return 0;
#endif
	// удаление оставшихся точек соединения, чтобы позже восстановить единую линию без ответвлений
	for (P = 0, bpredptr = PredPtr; P < S_XY; P++, bpredptr++)
	{
		if (GetPred(&bpredptr->Pred, CHANGE_PRED))
			DelPred(&bpredptr->Pred, CHANGE_PRED | FNGREG_PRED);
	}

	//************************************************************************
	//				   Сбор линий краев как отдельных сегментов								
	//*************************************************************************
	memset(EdgeLineSegmentList, 0, sizeof(EDGE_LINE_SEGMENT)*EDGE_LINE_SEGMENT_LIST_SIZE);
	memset(VPR_EdgeSegmNamePtr,0,S_XY*sizeof(PRED_SP));
	int EdgeLineSegmentNumber = 0, GlobalSegmentIndex=0;
	EDGE_LINE_SEGMENT EdgeLineSegment;

	ELS= EdgeLineSegmentList;
	UI SegmentPred = TMP1_PRED;
	for(sy=VPR_S_Top,bsindex=VPR_S_Offset; 
		sy <= VPR_S_Bottom;
		sy++,bsindex+=S_X)
	{
		for(sx=VPR_S_Left,sindex=bsindex; 
			sx <= VPR_S_Right;
			sx++,sindex++)
		{
			bpredptr=PredPtr+sindex;
			if( !GetPred(&bpredptr->Pred, FNGREG_PRED) )
				continue;
			if (GetPred(&bpredptr->Pred, SegmentPred))
				continue;

			binclptr = VPR_InclPtr + sindex;
			if (binclptr->I1 >= HALF_INCL_NUMBER)
				continue;

			bmptr = VPR_MPtr + sindex;

			ap = GlobalSegmentIndex;
			ip = GlobalSegmentIndex + 1;

			/* начало сегмента */
			ELS->Name = EdgeLineSegmentNumber+1;
			ELS->Incl = DEGREE_180;

			ELS->IndexNumber=1;
			ELS->IndexList[0].Start = ap;
			ELS->IndexList[0].End = ap;

			ELS->SumM = (VPR_MPtr + sindex)->M[binclptr->I1];

			*(SegmentIndex+ap) = (UI)sindex;
			SetPred(&bpredptr->Pred, SegmentPred);
			(VPR_EdgeSegmNamePtr + sindex)->Pred = ELS->Name;

#ifdef ACTIVATE_HELPER
			if (sx == 31 && sy == 17)
				int FFF_DEBUG = 0;
#endif
			
			Count=1;
			/* сбор сегмента */
			while(ap != ip)
			{
				cindex=*(SegmentIndex +ap);
				int Xc, Yc;
				S_OffsetToCoord(cindex, Xc, Yc);
#ifdef ACTIVATE_HELPER
				if (Xc == 50 && Yc == 49)
					int FFF_DEBUG = 0;
#endif
				int BIncl = (VPR_InclPtr+cindex)->I1;
				// поиск следующих ячеек
				int NeighbourFound = -1;
				for (i = 1; i < STREAM_REGION_3_SIZE; i++)
				{
					index = cindex + (*(S_RegionIndex + i));
					if (!GetPred(&(PredPtr + index)->Pred, FNGREG_PRED))
						continue;
					if (GetPred(&(PredPtr + index)->Pred, SegmentPred))
						continue;
	
					if (DeltaIncl(BIncl, (VPR_InclPtr + index)->I1, HALF_INCL_NUMBER) <= 1)
					{
						int Xn, Yn;
						
						S_OffsetToCoord(index, Xn, Yn);
#ifdef ACTIVATE_HELPER
						if (Xn == 17 && Yn == 53)
							int FFF_DEBUG = 0;
#endif
						 
						Incl = Incl_128_To_8(GetIncl(Xc, Yc, Xn, Yn));
						int dIncl = DeltaIncl((UC)BIncl, (UC)Incl, HALF_INCL_NUMBER);
						if ((i < STREAM_REGION_3R_SIZE && dIncl <= 1)
							||
							dIncl <= 2)
						{
							*(SegmentIndex + ip) = (UI)index;
							SetPred(&(PredPtr + index)->Pred, SegmentPred);
							(VPR_EdgeSegmNamePtr + index)->Pred = ELS->Name;
							ELS->IndexList[0].End = ip;
							ip++; 
							Count++;

							ELS->SumM += (VPR_MPtr + index)->M[BIncl];

							NeighbourFound = i;
							
						}
					}
				}
				ap++; 
			}
#ifdef ACTIVATE_HELPER
			if (EdgeLineSegmentNumber == 7)
				int GGGFFF = 9;
#endif
			GlobalSegmentIndex += Count;

			// назначение конечных точек
			Count = 0;
			if (ELS->IndexList[0].Start == ELS->IndexList[0].End)
			{
				index = *(SegmentIndex + ELS->IndexList[0].Start);
				S_OffsetToCoord((SI)index, ELS->EndPoint[0].X, ELS->EndPoint[0].Y);
				S_OffsetToCoord((SI)index, ELS->EndPoint[1].X, ELS->EndPoint[1].Y);
				Count=2;
			}
			else
			{
				for (i = ELS->IndexList[0].Start; i <= ELS->IndexList[0].End; i++)
				{
					index = *(SegmentIndex + i);

					UC c, ca, cc;
					DefPoint_VPR(VPR_EdgeSegmNamePtr + index, &c, &ca, &cc, 0, (VPR_EdgeSegmNamePtr + index)->Pred);
					UC Type = DefPointType_VPR(c, cc);
					if (Type == 1)  // точка окончания линии
					{
						if (Count < MAX_END_POINT)
						{
							S_OffsetToCoord((SI)index, ELS->EndPoint[Count].X, ELS->EndPoint[Count].Y);
							Count++;
						}
					}
				}

				if (Count < 2)
				{
					for (i = ELS->IndexList[0].Start; i <= ELS->IndexList[0].End; i++)
					{
						index = *(SegmentIndex + i);

						UC c, ca, cc;
						DefPoint_VPR(VPR_EdgeSegmNamePtr + index, &c, &ca, &cc, 0, (VPR_EdgeSegmNamePtr + index)->Pred);
						UC Type = DefPointType_VPR(c, cc);
						if (Type == 3)  // точка ветвления линии
						{
							if (Count < 2)
								S_OffsetToCoord((SI)index, ELS->EndPoint[Count].X, ELS->EndPoint[Count].Y);
							Count++;
							#ifdef ACTIVATE_HELPER
							FaultError("assign final EndPoints AS C-POINT ");
							#endif
						}
					}
				}
				if (Count > 2)
				{
					// нахождение 2 наиболее удаленных точек (остальные - ложные конечные точки)
					SL MaxDist = 0,Dist;
					for(ep1=0;ep1 < Count;ep1++)
						for (ep2 = ep1+1; ep2 < Count; ep2++)
						{
							Dist = GetDist(ELS->EndPoint[ep1].X, ELS->EndPoint[ep1].Y, ELS->EndPoint[ep2].X, ELS->EndPoint[ep2].Y);
							if (MaxDist < Dist)
							{
								bep1 = ep1;
								bep2 = ep2;
								MaxDist = Dist;
							}
						}
					ELS->EndPoint[0].X = ELS->EndPoint[bep1].X;
					ELS->EndPoint[0].Y = ELS->EndPoint[bep1].Y;
					ELS->EndPoint[1].X = ELS->EndPoint[bep2].X;
					ELS->EndPoint[1].Y = ELS->EndPoint[bep2].Y;
					Count=2;
				}
			}
			#ifdef ACTIVATE_HELPER
			if (Count != 2)
				FaultError("Ошибка определения финальных точек!!!");
			#endif

			EdgeLineSegmentNumber++;
			ELS++;
			if (EdgeLineSegmentNumber >= EDGE_LINE_SEGMENT_LIST_SIZE)
				break;
		}
	}
	// удаление оставшихся точек соединения, чтобы позже восстановить единую линию без ответвлений
	for (P = 0, bpredptr = PredPtr; P < S_XY; P++, bpredptr++)
		DelPred(&bpredptr->Pred, SegmentPred);

#ifdef ACTIVATE_HELPER
	for (s = 0; s < EdgeLineSegmentNumber; s++)
	{
		SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
		SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
	}
	if (!Helper("10", EdgeLineSegmentNumber, 0, VPRPROC_DEMO_))  return 0;
	for (s = 0; s < EdgeLineSegmentNumber; s++)
	{
		DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
		DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
	}
#endif

	Iteration = 0;
	while (1)
	{
		//*************************************************************************
		//				Сортировка линейных сегментов на основе их меры				 
		//*************************************************************************
		Count = 0;
		for (s = 0; s < EdgeLineSegmentNumber; s++)
		{
			MaxM = 0;
			for (i = s + 1; i < EdgeLineSegmentNumber; i++)
				if (MaxM < EdgeLineSegmentList[i].SumM)
				{
					MaxM = EdgeLineSegmentList[i].SumM;
					index = i;
				}
			if (EdgeLineSegmentList[s].SumM < MaxM)
			{
				EdgeLineSegment = EdgeLineSegmentList[s];
				EdgeLineSegmentList[s] = EdgeLineSegmentList[index];
				EdgeLineSegmentList[index] = EdgeLineSegment;
			}
		}
		// исключение соединенных сегментов
		if (Iteration && EdgeLineSegmentNumber)
			EdgeLineSegmentNumber--;


		//*************************************************************************
		//				Попытка соединения линейных сегментов между собой
		//*************************************************************************

		for (bs = 0; bs < EdgeLineSegmentNumber; bs++)
		{
			BELS = &EdgeLineSegmentList[bs];
			// конечная точка не определена
			if (BELS->EndPoint[0].X == 0 || BELS->EndPoint[0].Y == 0 || BELS->EndPoint[1].X == 0 || BELS->EndPoint[1].Y == 0)
				continue;
			int BIncl = GetIncl(BELS->EndPoint[0].X, BELS->EndPoint[0].Y, BELS->EndPoint[1].X, BELS->EndPoint[1].Y);

			for (bep1 = 0, bep2 = 1; bep1 < 2; bep1++, bep2--)
			{
				BEP1 = &BELS->EndPoint[bep1];

				bpredptr = PredPtr + BEP1->Y*S_X + BEP1->X;
	
				BEP2 = &BELS->EndPoint[bep2];
				int BDir = GetDir(BEP2->X, BEP2->Y, BEP1->X, BEP1->Y);

				SL MinM = MAX_VALUE_SL;
				for (s = bs + 1; s < EdgeLineSegmentNumber; s++)
				{
					ELS = &EdgeLineSegmentList[s];
					// конечная точка не определена
					if (ELS->EndPoint[0].X == 0 || ELS->EndPoint[0].Y == 0 || ELS->EndPoint[1].X == 0 || ELS->EndPoint[1].Y == 0)
						continue;

#ifdef ACTIVATE_HELPER
					if (BELS->Name == 19 && ELS->Name == 20)
						int FFF_DEBUG = 0;
					if (BELS->Name == 19 && ELS->Name ==18)
						int FFF_DEBUG = 0;
#endif
					for (ep1 = 0, ep2 = 1; ep1 < 2; ep1++, ep2--)
					{
						EP1 = &ELS->EndPoint[ep1];
						EP2 = &ELS->EndPoint[ep2];

						int Dist = GetDist(BEP1->X, BEP1->Y, EP1->X, EP1->Y);
						int DirP = GetDir(BEP1->X, BEP1->Y, EP1->X, EP1->Y);
						int DirP2 = GetDir(BEP1->X, BEP1->Y, EP2->X, EP2->Y);
						int Dir,DDir;
						if (EP1->X == EP2->X && EP1->Y == EP2->Y)// точечный сегмент - использует наклонение ячейки вместо dir ep1->ep2
						{
							Incl = Incl_8_To_128((VPR_InclPtr + (*(SegmentIndex + ELS->IndexList[0].Start)))->I1);
							DDir = DeltaIncl((UC)BIncl, (UC)Incl,DEGREE_180);
						}
						else
						{
							Dir = GetDir(EP1->X, EP1->Y, EP2->X, EP2->Y);
							DDir = DeltaDir(BDir, Dir);
						}
						int DDirP = DeltaDir(BDir, DirP);
						int DDirP2 = DeltaDir(BDir, DirP2);
						if ((Dist <= 2 && DDirP <= DEGREE_90+DEGREE_5 && DDirP2 <= DEGREE_22+DEGREE_11 && DDir <= DEGREE_45-DEGREE_5)
							||
							(Dist <= 3 && DDirP <= DEGREE_45+DEGREE_11 && DDirP2 <= DEGREE_22+DEGREE_11 && DDir <= DEGREE_45-DEGREE_5)
							||
							(Dist <= 4 && DDirP <= DEGREE_45-DEGREE_11 && DDirP2 <= DEGREE_22+DEGREE_11 && DDir <= DEGREE_45-DEGREE_5))
						{
							M = Dist * 2 + DDirP + DDirP2*2 + (DDir/DEGREE_5);
							if (MinM > M)
							{
								MinM = M;
								sindex = s;
								index = ep2;
							}
						}
					}
				}
				// найдено наилучшее соединение
				if (MinM != MAX_VALUE_SL)
				{
					ELS = &EdgeLineSegmentList[sindex];
					EP2 = &ELS->EndPoint[index];
					
					// переименование точек соединенных сегментов и добавление IndexList к базовому сегменту
					for (i = 0; i < ELS->IndexNumber; i++)
					{
						for (index = ELS->IndexList[i].Start; index <= ELS->IndexList[i].End; index++)
							(VPR_EdgeSegmNamePtr + (*(SegmentIndex + index) ))->Pred = BELS->Name;
						if (BELS->IndexNumber < EDGE_LINE_SEGMENT_INDEX_NUMBER)
						{
							BELS->IndexList[BELS->IndexNumber] = ELS->IndexList[i];
							BELS->IndexNumber++;
						}
					}

					*BEP1 = *EP2; // обновление конечной точки базового сегмента

					BELS->SumM += ELS->SumM; // обновление SumM базового сегмента
					ELS->SumM = 0;// отметка сегмента, который будет удален на следующей итерации

					break;// разрешение только одного соединения для одной итерации
				}
			}
			if (bep1 < 2)
				break;
		}
		// ни одно новое соединение не найдено, остановка поиска
		if (bs >= EdgeLineSegmentNumber)
			break;
		Iteration++;
	}
#ifdef ACTIVATE_HELPER
	for (s = 0; s < EdgeLineSegmentNumber; s++)
		if (EdgeLineSegmentList[s].SumM)
		{
			SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
			SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
		}
	if (!Helper("11", EdgeLineSegmentNumber, Iteration, VPRPROC_DEMO_))  return 0;
	for (s = 0; s < EdgeLineSegmentNumber; s++)
		if (EdgeLineSegmentList[s].SumM)
		{
			DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
			DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
		}
#endif

	//*************************************************************************
	//				    Выбор основных сегментов								
	//*************************************************************************
	int ActiveEdgeLineSegmentNumber = EdgeLineSegmentNumber;
	if (VPRDescriptor->ExpectedNumberOfFingers && ActiveEdgeLineSegmentNumber > VPRDescriptor->ExpectedNumberOfFingers * 2)
		ActiveEdgeLineSegmentNumber = VPRDescriptor->ExpectedNumberOfFingers * 2;
	ThresholdM = EdgeLineSegmentList[0].SumM;
	ThresholdM /= 4;
	for (s = 0; s < ActiveEdgeLineSegmentNumber; s++)
	{
		ELS = &EdgeLineSegmentList[s];
		if (ELS->SumM < ThresholdM)
			ELS->SumM = 0;
		else
		if (ELS->EndPoint[0].X && ELS->EndPoint[0].Y && ELS->EndPoint[1].X && ELS->EndPoint[1].Y)
		{
			ELS->Incl = GetIncl(ELS->EndPoint[0].X, ELS->EndPoint[0].Y, ELS->EndPoint[1].X, ELS->EndPoint[1].Y);
			if (VPRDescriptor->ExpectedOrienation)
			{
				if (VPRDescriptor->ExpectedOrienation == 1) //1 - палец ориентирован вертикально
					Incl = DEGREE_90;
				else
					Incl = 0;
				if (DeltaIncl((UC)ELS->Incl, (UC)Incl,DEGREE_180) > DEGREE_22)
					ELS->SumM = 0;
			}
		}
		else
			ELS->SumM = 0;
	}

	for (s = 0, i = 0; s < ActiveEdgeLineSegmentNumber; s++)
	{
		if (EdgeLineSegmentList[s].SumM)
		{
			if (i != s)
				EdgeLineSegmentList[i] = EdgeLineSegmentList[s];
			i++;
		}
	}
	ActiveEdgeLineSegmentNumber = i;

	//*************************************************************************
	// 	  Удаление всех сегментов, представляющих края линий, кроме основных				
	//*************************************************************************
	for (s = ActiveEdgeLineSegmentNumber; s < EdgeLineSegmentNumber; s++)
	{
		ELS = &EdgeLineSegmentList[s];
		for (i = 0; i < ELS->IndexNumber; i++)
		{
			for (index = ELS->IndexList[i].Start; index <= ELS->IndexList[i].End; index++)
			{
				(VPR_EdgeSegmNamePtr + (*(SegmentIndex + index)))->Pred = 0;
				DelPred(&(PredPtr + (*(SegmentIndex + index)))->Pred, FNGREG_PRED);
			}
		}
	}

#ifdef ACTIVATE_HELPER
	for (s = 0; s < ActiveEdgeLineSegmentNumber; s++)
		if (EdgeLineSegmentList[s].SumM)
		{
			SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
			SetPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
		}
	if (!Helper("12", ActiveEdgeLineSegmentNumber, 0, VPRPROC_DEMO_))  return 0;
	for (s = 0; s < ActiveEdgeLineSegmentNumber; s++)
		if (EdgeLineSegmentList[s].SumM)
		{
			DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[0].Y*S_X + EdgeLineSegmentList[s].EndPoint[0].X)->Pred, CHANGE_PRED);
			DelPred(&(PredPtr + EdgeLineSegmentList[s].EndPoint[1].Y*S_X + EdgeLineSegmentList[s].EndPoint[1].X)->Pred, TMP2_PRED);
		}
#endif

	//*************************************************************************
	//			Проверка наличия необходимого кол-ва линейных сегментов							
	//*************************************************************************
	if (VPRDescriptor->ExpectedNumberOfFingers)
	{
		if (ActiveEdgeLineSegmentNumber != VPRDescriptor->ExpectedNumberOfFingers * 2)
			return 0; //не выполнять очистку фона 
	}

	//************************************************************************************
	//		Определение внутренней области на основе активных сегментов линий края		
	//************************************************************************************
	if (ActiveEdgeLineSegmentNumber == 2)
	{
		s1 = 0;
		s2 = 1;

		

		Incl = Incl_128_To_16(MedIncl((UC)EdgeLineSegmentList[s1].Incl, (UC)EdgeLineSegmentList[s2].Incl, DEGREE_180));
		BPIPtr = S_Index + Incl*S_CUTSET_HSIZE;
		vpr_poffset1 = *(BPIPtr + 1);
		vpr_poffset2 = *(BPIPtr + 2);

		BNormal = Incl + INCL_NUMBER / 2;
		if (BNormal >= INCL_NUMBER)
			BNormal -= INCL_NUMBER;

		for (sy = VPR_S_Top, bsindex = VPR_S_Offset;
			sy <= VPR_S_Bottom;
			sy++, bsindex += S_X)
		{
			for (sx = VPR_S_Left, sindex = bsindex;
				sx <= VPR_S_Right;
				sx++, sindex++)
			{
				bpredptr = PredPtr + sindex;
				if (GetPred(&bpredptr->Pred, FNGREG_PRED))
					continue;

#ifdef ACTIVATE_HELPER
				if (sx == 31 && sy == 21)
					int FFF_DEBUG = 0;
#endif

				int ELSName1 = 0;
				int ELSName2 = 0;
				for (i = 0, Normal = BNormal; i < HALF_INCL_NUMBER-1; i++)
				{
					if (i)
					{
						if (i % 2)
							Normal = BNormal + ((i/2)+1);
						else
							Normal = BNormal - (i/2);
					}

					if (Normal < 0) Normal += INCL_NUMBER;
					if (Normal >= INCL_NUMBER) Normal -= INCL_NUMBER;

					BNIPtr = S_Index + Normal*S_CUTSET_HSIZE;

					ELSName1 = 0;
					ELSName2 = 0;

					if (!ELSName1)
					{
						// поиск сегмента линии края слева
						bnindex = sindex;
						Count = 0;
						while (1)
						{
							// проход на S_CUTSET_HSIZE шагов влево
							for (n = 0; n < S_CUTSET_HSIZE; n++)
							{
								offset = *(BNIPtr + n);
								nindex = bnindex - offset;

								if (!GetPred(&(PredPtr + nindex)->Pred, IMAGE_PRED))//if (!GetPred(&(PredPtr + nindex)->Pred, REGION3_PRED))
									break;
								if ((VPR_EdgeSegmNamePtr + nindex)->Pred)
								{
									ELSName1 = (VPR_EdgeSegmNamePtr + nindex)->Pred;
									break;
								}

								if (Count >= 1)
								{
									ELSName1 = (VPR_EdgeSegmNamePtr + nindex + vpr_poffset1)->Pred;
									if (ELSName1)	break;
									ELSName1 = (VPR_EdgeSegmNamePtr + nindex - vpr_poffset1)->Pred;
									if (ELSName1)	break;
									if (Count >= 2)
									{
										ELSName1 = (VPR_EdgeSegmNamePtr + nindex + vpr_poffset2)->Pred;
										if (ELSName1)	break;
										ELSName1 = (VPR_EdgeSegmNamePtr + nindex - vpr_poffset2)->Pred;
										if (ELSName1)	break;
									}
								}
								Count++;
							}
							if (n < S_CUTSET_HSIZE)
								break;
							bnindex = nindex; // перезапуск с последней точки
						}
					}

					if (!ELSName2)
					{
						// поиск сегмента линии края справа
						bnindex = sindex;
						Count = 0;
						while (1)
						{
							// проход на S_CUTSET_HSIZE шагов вправо
							for (n = 0; n < S_CUTSET_HSIZE; n++)
							{
								offset = *(BNIPtr + n);
								nindex = bnindex + offset;

								if (!GetPred(&(PredPtr + nindex)->Pred, IMAGE_PRED))
									break;
								if ((VPR_EdgeSegmNamePtr + nindex)->Pred)
								{
									ELSName2 = (VPR_EdgeSegmNamePtr + nindex)->Pred;
									break;
								}

								if (Count >= 1)
								{
									ELSName2 = (VPR_EdgeSegmNamePtr + nindex + vpr_poffset1)->Pred;
									if (ELSName2)	break;
									ELSName2 = (VPR_EdgeSegmNamePtr + nindex - vpr_poffset1)->Pred;
									if (ELSName2)	break;
									if (Count >= 2)
									{
										ELSName2 = (VPR_EdgeSegmNamePtr + nindex + vpr_poffset2)->Pred;
										if (ELSName2)	break;
										ELSName2 = (VPR_EdgeSegmNamePtr + nindex - vpr_poffset2)->Pred;
										if (ELSName2)	break;
									}
								}
								Count++;

							}
							if (n < S_CUTSET_HSIZE)
								break;
							bnindex = nindex; // перезапуск с последней точки
						}
					}
					if (ELSName1 && ELSName2)
						break;
				}
				if (!ELSName1 || !ELSName2)
					continue;

				if (ELSName1 == EdgeLineSegmentList[s1].Name && ELSName2 == EdgeLineSegmentList[s2].Name
					||
					ELSName1 == EdgeLineSegmentList[s2].Name && ELSName2 == EdgeLineSegmentList[s1].Name)
				{
					SetPred(&bpredptr->Pred, CHANGE_PRED);
				}
			}
		}

	}
#ifdef ACTIVATE_HELPER
	//if (!Helper("13", ActiveEdgeLineSegmentNumber, Incl, VPRPROC_DEMO_))  return 0;
#endif

	//****************************************************
	//		Завершение определения области пальца		
	//****************************************************
	for (sy = VPR_S_Top, bsindex = VPR_S_Offset; sy <= VPR_S_Bottom; sy++, bsindex += S_X)
		for (sx = VPR_S_Left, sindex = bsindex; sx <= VPR_S_Right; sx++, sindex++)
		{
			bpredptr = PredPtr + sindex;
			if (!GetPred(&bpredptr->Pred, IMAGE_PRED))
				continue;

#ifdef ACTIVATE_HELPER
			if (sx == 47 && sy == 17)
				int FFF_DEBUG = 0;
#endif
			Count = 0;
			for (i = 1; i < STREAM_REGION_3_SIZE; i++)
			{
				offset = *(S_RegionIndex + i);
				if (GetPred(&(bpredptr + offset)->Pred, CHANGE_PRED))
					Count++;
			}
			if (GetPred(&bpredptr->Pred, (REGION3_PRED|CHANGE_PRED)) == (REGION3_PRED|CHANGE_PRED) && Count <= 3)
				SetPred(&bpredptr->Pred, TMP3_PRED);// удаление выступов
			else
			if (!GetPred(&bpredptr->Pred,REGION3_PRED|CHANGE_PRED) && Count >= 3)
				SetPred(&bpredptr->Pred, TMP2_PRED);// добавление углов
			else
			if (GetPred(&bpredptr->Pred,REGION3_PRED) && !GetPred(&bpredptr->Pred,CHANGE_PRED) && Count >= 5)
				SetPred(&bpredptr->Pred, TMP2_PRED);// заполнение одиночных неопределенных ячеек
		}

#ifdef ACTIVATE_HELPER
	//if( !Helper("14",DiffTimeInMilliseconds(&t1,&t2),0,VPRPROC_DEMO_) )  return 0;
#endif
	for (P = 0, predptr = PredPtr; P < S_XY; P++, predptr++)
	{
		if (GetPred(&predptr->Pred, TMP2_PRED))
		{
			DelPred(&predptr->Pred, TMP2_PRED);
			SetPred(&predptr->Pred, CHANGE_PRED);
		}
		if (GetPred(&predptr->Pred, TMP3_PRED))
		{
			DelPred(&predptr->Pred, TMP3_PRED| CHANGE_PRED);
		}
	}

	//****************************************************
	//		Отметка линий края области пальца		
	//****************************************************
	for (sy = VPR_S_Top, bsindex = VPR_S_Offset, ry = S_To_R_Coord(sy);
		sy <= VPR_S_Bottom;
		sy++, bsindex += S_X, ry += S_ITEM_SIZE)
	{
		for (sx = VPR_S_Left, sindex = bsindex, rx = S_To_R_Coord(sx);
			sx <= VPR_S_Right;
			sx++, sindex++, rx += S_ITEM_SIZE)
		{
			bpredptr = PredPtr + sindex;

			Count = 0;
			for (i = 1; i < STREAM_REGION_3R_SIZE; i++)
			{
				offset = *(S_RegionIndex + i);
				if (GetPred(&(bpredptr + offset)->Pred, CHANGE_PRED))
					Count++;
			}

			if (GetPred(&(bpredptr)->Pred, CHANGE_PRED))
			{
				if (Count >= 4)
					continue;// внутренняя область

				SetPred(&bpredptr->Pred, TMP2_PRED);// внутренняя область края
			}
			else
			{
				if (Count >= 1)
				{
					SetPred(&bpredptr->Pred, TMP3_PRED);// внешняя область края
				}
			}
		}
	}
#ifdef ACTIVATE_HELPER
	if (!Helper("15", DiffTimeInMilliseconds(&t1, &t2), 0, VPRPROC_DEMO_))  return 0;
#endif

	//*********************************************************
	//		Очистка фона (зона за пределами области пальца)	
	//*********************************************************
	#define BACKGROUNG_VALUE 255

	Iteration = 0;
	int ChangeFlag = 0;

	for (index = 0; index < R_X*R_Y; index++)
		if (*(RPtr1 + index) >= 254)
			*(RPtr1 + index) = 253;

	while (1)
	{
		for (sy = VPR_S_Top, bsindex = VPR_S_Offset, ry = S_To_R_Coord(sy);
			sy <= VPR_S_Bottom;
			sy++, bsindex += S_X, ry += S_ITEM_SIZE)
		{
			for (sx = VPR_S_Left, sindex = bsindex, rx = S_To_R_Coord(sx);
				sx <= VPR_S_Right;
				sx++, sindex++, rx += S_ITEM_SIZE)
			{
				bpredptr = PredPtr + sindex;

				brindex = R_CoordToOffset(rx, ry);

#ifdef ACTIVATE_HELPER
				if (sx == 38 && sy == 30)
					int FFF_DEBUG = 0;
#endif

				int x, y;
				if (Iteration && GetPred(&(bpredptr)->Pred, TMP2_PRED | TMP3_PRED))
				{
					Count = 0;
					Count2 = 0;
					CountN = 0;
					memset(Histogram, 0, sizeof(int) * 256);
					for (i = 1; i < STREAM_REGION_5_SIZE; i++)
					{
						offset = *(S_RegionIndex + i);
						if (!GetPred(&(bpredptr + offset)->Pred, IMAGE_PRED))
							continue;
						if (!GetPred(&(bpredptr + offset)->Pred, CHANGE_PRED))
						{
							if (i < STREAM_REGION_3_SIZE)
								CountN++;
							continue;
						}
						if (GetPred(&(bpredptr + offset)->Pred, TMP3_PRED))// TMP2_PRED | 
							continue;

						S_OffsetToCoord(sindex + offset, x, y);
						x = S_To_R_Coord(x);
						y = S_To_R_Coord(y);
						bnindex = R_CoordToOffset(x, y);
						for (y = 0; y < S_ITEM_SIZE; y++, bnindex += R_X)
							for (x = 0, nindex = bnindex; x < S_ITEM_SIZE; x++, nindex++)
							{
								Histogram[*(RPtr1 + nindex)]++;
								Count2++;
							}

						Count++;
						if (Count >= 5)
							break;// достаточно данных
					}
					if (CountN == 0 || // ячейка НЕ находится близко к фоновой области
						!Count)
						continue;
					Count2 = (Count2 * 40) / 100; //Count2 = (Count2 * 25) / 100;// не принимаем во внимание некоторую часть темных пикселей
					Count = 0;
					int FilterValue;
					for (FilterValue = 0; FilterValue < 255; FilterValue++)
					{
						Count += Histogram[FilterValue];
						if (Count >= Count2)
							break;
					}

#ifdef ACTIVATE_HELPER
					if (sx == 38 && sy == 30)
						int FFF_DEBUG = 0;
#endif

					// очистка внешних областей
					for (y = 0, index = brindex;
						y < S_ITEM_SIZE;
						y++, index += R_X)
					{
						UC *fptr = RPtr1 + index;
						for (x = 0; x < S_ITEM_SIZE; x++,fptr++)
						{
							if (*(fptr) < FilterValue &&
								(*(fptr+1)==255 || *(fptr - 1) == 255 || *(fptr - R_X) == 255 || *(fptr + R_X) == 255 ))

							{
								*(fptr) = 254;
								ChangeFlag = 1;

#ifdef ACTIVATE_HELPER
								*(ptrMainWnd->ptrMemImgBIG_LEFT + (RI_Y - 1- (ry + y - R_EXPANDER))*RI_X + (rx + x - R_EXPANDER)) = 255; //
#endif
							}
						}
					}


				}
				else
				if (!Iteration && !GetPred(&(bpredptr)->Pred, CHANGE_PRED|TMP3_PRED))
				{
					// очистка внешних областей
					for (y = 0, index = brindex;
						y < S_ITEM_SIZE;
						y++, index += R_X)
					{
						UC *fptr = RPtr1 + index;
						for (x = 0; x < S_ITEM_SIZE; x++, fptr++)
						{
							*(fptr) = 254;
							ChangeFlag = 1;

#ifdef ACTIVATE_HELPER
							*(ptrMainWnd->ptrMemImgBIG_LEFT + (RI_Y - 1 - (ry + y - R_EXPANDER))*RI_X + (rx + x - R_EXPANDER)) = 255;
#endif
						}
					}
				}
			}
		}
		Iteration++;
		if (!ChangeFlag)
			break;

		ChangeFlag = 0;

#ifdef ACTIVATE_HELPER
		//if (!Helper("16", ActiveEdgeLineSegmentNumber, Iteration, VPRPROC_DEMO_))  return 0;
#endif

		for (index = 0; index < R_X*R_Y; index++)
			if (*(RPtr1 + index) == 254)
				*(RPtr1 + index) = 255;

	}

#ifdef ACTIVATE_HELPER
	//if (!Helper("17", ActiveEdgeLineSegmentNumber, Iteration, VPRPROC_DEMO_))  return 0;
#endif

	Iteration = 0;
	while (1)
	{
		for (sy = VPR_S_Top, bsindex = VPR_S_Offset, ry = S_To_R_Coord(sy);
			sy <= VPR_S_Bottom;
			sy++, bsindex += S_X, ry += S_ITEM_SIZE)
		{
			for (sx = VPR_S_Left, sindex = bsindex, rx = S_To_R_Coord(sx);
				sx <= VPR_S_Right;
				sx++, sindex++, rx += S_ITEM_SIZE)
			{
				bpredptr = PredPtr + sindex;

				brindex = R_CoordToOffset(rx, ry);

#ifdef ACTIVATE_HELPER
				if (sx == 38 && sy == 30)
					int FFF_DEBUG = 0;
#endif

				int x, y;
				if (GetPred(&(bpredptr)->Pred, TMP2_PRED | TMP3_PRED))
				{
#ifdef ACTIVATE_HELPER
					if (sx == 38 && sy == 30)
						int FFF_DEBUG = 0;
#endif

					// очистка внешних областей
					for (y = 0, index = brindex;
						y < S_ITEM_SIZE;
						y++, index += R_X)
					{
						UC *fptr = RPtr1 + index;
						for (x = 0; x < S_ITEM_SIZE; x++, fptr++)
						{
							if (*(fptr) < 254 &&
								(*(fptr + 1) == 255 || *(fptr - 1) == 255 || *(fptr - R_X) == 255 || *(fptr + R_X) == 255))

							{
								*(fptr) = 254;
#ifdef ACTIVATE_HELPER
								*(ptrMainWnd->ptrMemImgBIG_LEFT + (RI_Y - 1 - (ry + y - R_EXPANDER))*RI_X + (rx + x - R_EXPANDER)) = 255;
#endif
							}
						}
					}
				}
			}
		}

		for (index = 0; index < R_X*R_Y; index++)
			if (*(RPtr1 + index) == 254)
				*(RPtr1 + index) = 255;

		Iteration++;
		if(Iteration >= S_ITEM_SIZE/2+1)
			break;

#ifdef ACTIVATE_HELPER
		//if (!Helper("18", ActiveEdgeLineSegmentNumber, Iteration, VPRPROC_DEMO_))  return 0;
#endif

	}



#ifdef ACTIVATE_HELPER
	//WriteToBMP("D:\\TEMP\\vpr.bmp", RPtr1, R_X, R_Y);
	//if (!Helper("19", ActiveEdgeLineSegmentNumber, Iteration, VPRPROC_DEMO_))  return 0;
#endif

	//*************************************************************************
	//		 Копирования полученного изображения обратно в выходной буфер				
	//*************************************************************************
	memset(VPRDescriptor->InputImage, 255, VPRDescriptor->ImageSizeX*VPRDescriptor->ImageSizeY);
	index = ((VPRDescriptor->ImageSizeY - (VPR_R_Bottom - VPR_R_Top + 1)) / 2)* VPRDescriptor->ImageSizeX + 
		    ((VPRDescriptor->ImageSizeX - (VPR_R_Right - VPR_R_Left + 1)) / 2);
	for (ry = VPR_R_Top;
		 ry <= VPR_R_Bottom; 
		 ry++, index+= VPRDescriptor->ImageSizeX)
	{
		rindex = R_CoordToOffset(VPR_R_Left, ry);
		memcpy(VPRDescriptor->InputImage + index, RPtr1 + rindex, VPR_R_Right - VPR_R_Left + 1);
	}

#ifdef ACTIVATE_HELPER
	//WriteToBMP("D:\\TEMP\\vpr_int.bmp", RPtr1, R_X, R_Y);
	//WriteToBMP("D:\\TEMP\\vpr_out.bmp", VPRDescriptor->InputImage, VPRDescriptor->ImageSizeX, VPRDescriptor->ImageSizeY);
	if (!Helper("20", 0, 0, VPRPROC_DEMO_))  return 0;
#endif

	return 1;
}


/* ПЕРЕМЕННЫЕ ДОЛЖНЫ БЫТЬ НАЗНАЧЕНЫ ДО ВЫЗОВА ФУНКЦИИ: vpr_poffset1,vpr_poffset2,vpr_poffset4,vpr_poffset6,vpr_poffset8 */
UC	PointParallelIntegration_15_VPR(UC *rptr)
{
	UI P;
	P = (*(rptr + vpr_poffset1));
	P += (*(rptr - vpr_poffset1));
	P += (*(rptr + vpr_poffset2));
	P += (*(rptr - vpr_poffset2));
	P += (*(rptr + vpr_poffset4));
	P += (*(rptr - vpr_poffset4));
	P += (*(rptr + vpr_poffset6));
	P += (*(rptr - vpr_poffset6));
	P += (*(rptr + vpr_poffset8));
	P += (*(rptr - vpr_poffset8));
	return (P / 10);
}

void DefPoint_VPR(PRED_SP *pptr, UC *Count, UC *CountAll, UC *CountChange, UI  Pred,UI Value)
{
	UC  Pred0, Pred1, Pred2, Pred3, Pred4, Pred5, Pred6, Pred7;

	*Count = 0;
	*CountAll = 0;
	*CountChange = 0;

	if (Pred)
	{
		Pred7 = GetPred(&(pptr - S_X - 1)->Pred, Pred);
		Pred0 = GetPred(&(pptr - S_X)->Pred, Pred);
		Pred1 = GetPred(&(pptr - S_X + 1)->Pred, Pred);
		Pred6 = GetPred(&(pptr - 1)->Pred, Pred);
		Pred2 = GetPred(&(pptr + 1)->Pred, Pred);
		Pred5 = GetPred(&(pptr + S_X - 1)->Pred, Pred);
		Pred4 = GetPred(&(pptr + S_X)->Pred, Pred);
		Pred3 = GetPred(&(pptr + S_X + 1)->Pred, Pred);
	}
	else
	{
		Pred7 = ((pptr - S_X - 1)->Pred == Value);
		Pred0 = ((pptr - S_X)->Pred == Value);
		Pred1 = ((pptr - S_X + 1)->Pred == Value);
		Pred6 = ((pptr - 1)->Pred == Value);
		Pred2 = ((pptr + 1)->Pred == Value);
		Pred5 = ((pptr + S_X - 1)->Pred == Value);
		Pred4 = ((pptr + S_X)->Pred == Value);
		Pred3 = ((pptr + S_X + 1)->Pred == Value);
	}

	if (Pred0)
	{
		(*Count)++;
		(*CountAll)++;
	}

	if (Pred1)
	{
		if (!Pred0 && !Pred2)
			(*Count)++;
		(*CountAll)++;
	}

	if (Pred2)
	{
		(*Count)++;
		(*CountAll)++;
	}

	if (Pred3)
	{
		if (!Pred2 && !Pred4)
			(*Count)++;
		(*CountAll)++;
	}

	if (Pred4)
	{
		(*Count)++;
		(*CountAll)++;
	}

	if (Pred5)
	{
		if (!Pred4 && !Pred6)
			(*Count)++;
		(*CountAll)++;
	}

	if (Pred6)
	{
		(*Count)++;
		(*CountAll)++;
	}

	if (Pred7)
	{
		if (!Pred6 && !Pred0)
			(*Count)++;
		(*CountAll)++;
	}

	if (Pred0 != Pred1) (*CountChange)++;
	if (Pred1 != Pred2) (*CountChange)++;
	if (Pred2 != Pred3) (*CountChange)++;
	if (Pred3 != Pred4) (*CountChange)++;
	if (Pred4 != Pred5) (*CountChange)++;
	if (Pred5 != Pred6) (*CountChange)++;
	if (Pred6 != Pred7) (*CountChange)++;
	if (Pred7 != Pred0) (*CountChange)++;
}

UC   DefPointType_VPR(UC Count, UC CountChange)
{
	/* Внутренняя точка линии */
	if (Count == 2 && CountChange == 4)
		return 2;
	/* Точка окончания линии */
	if (Count == 1 && CountChange == 2)
		return 1;
	/* Точка соединения 3-х линий */
	if (Count == 3 && CountChange == 6)
		return 3;
	/* Точка соединения 4-х линий */
	if (Count == 4 && CountChange == 8)
		return 4;
	return 0;
}
