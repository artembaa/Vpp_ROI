#include "VPR_def.h"

SI  VPR_S_Left,VPR_S_Right,VPR_S_Top,VPR_S_Bottom;
SL  VPR_S_Offset;

SI  VPR_R_Left,VPR_R_Right,VPR_R_Top,VPR_R_Bottom;
SL  VPR_R_Offset;

SI VPRInit(VPR_IMAGE_DESCRIPTOR *VPRDescriptor,
		   PRED_SP *PredPtr)
{
	SI		sx,sy;
	PRED_SP *predptr;
	SI		Margin;


	if( !VPRImagePosition((SI)VPRDescriptor->ImageSizeX, (SI)VPRDescriptor->ImageSizeY,0,0) )
		return 0;

	// Инициализация матрицы ячеек 
	for(sy=0,predptr=PredPtr;sy < S_Y;sy++)
		for(sx=0;sx < S_X;sx++,predptr++)
		{
			predptr->Pred=0;
			if( sy < VPR_S_Top || sy > VPR_S_Bottom || sx < VPR_S_Left || sx > VPR_S_Right )
				continue;
			SetPred(&predptr->Pred,IMAGE_PRED);

			Margin=REDEF_MARGIN_7;
			if( Margin > sx-VPR_S_Left ) Margin=sx-VPR_S_Left;
			if( Margin > VPR_S_Right-sx ) Margin=VPR_S_Right-sx;
			if( Margin > sy-VPR_S_Top ) Margin=sy-VPR_S_Top;
			if( Margin > VPR_S_Bottom-sy ) Margin=VPR_S_Bottom-sy;

			if( Margin >= REDEF_MARGIN_3 )
				SetPred(&predptr->Pred,REGION3_PRED);
		}

	return 1;
}

SI VPRImagePosition(SI      ImageSizeX,
					SI      ImageSizeY,
					SI	   *ImageCoordX,
					SI	   *ImageCoordY)
{
	SI  R_L, R_R, R_T, R_B, OutL, OutR, OutB, OutT;


	if (ImageSizeX > RI_X || ImageSizeX < RI_MIN || ImageSizeY > RI_Y || ImageSizeY < RI_MIN)
		return 0;

	/* предварительное расположение изображения - в геометрическом центре расширенного буфера */
	VPR_R_Left = (R_X / 2) - (ImageSizeX / 2);
	VPR_R_Right = VPR_R_Left + ImageSizeX - 1;
	VPR_R_Top = (R_Y / 2) - (ImageSizeY / 2);
	VPR_R_Bottom = VPR_R_Top + ImageSizeY - 1;

	VPR_S_Left = R_To_S_Coord(VPR_R_Left);
	R_L = S_To_R_Coord(VPR_S_Left);
	if (R_L < VPR_R_Left)
	{
		VPR_S_Left++;
		OutL = S_ITEM_SIZE - (VPR_R_Left - R_L); /* внешняя область изображения - не покрыта матрицей */
	}
	else
		OutL = 0;

	VPR_S_Right = R_To_S_Coord(VPR_R_Right);
	R_R = S_To_R_Coord(VPR_S_Right) + (S_ITEM_SIZE - 1);
	if (R_R > VPR_R_Right)
	{
		VPR_S_Right--;
		OutR = S_ITEM_SIZE - (R_R - VPR_R_Right); /* внешняя область изображения - не покрыта матрицей */
	}
	else
		OutR = 0;

	VPR_S_Top = R_To_S_Coord(VPR_R_Top);
	R_T = S_To_R_Coord(VPR_S_Top);
	if (R_T < VPR_R_Top)
	{
		VPR_S_Top++;
		OutT = S_ITEM_SIZE - (VPR_R_Top - R_T); /* внешняя область изображения - не покрыта матрицей */
	}
	else
		OutT = 0;

	VPR_S_Bottom = R_To_S_Coord(VPR_R_Bottom);
	R_B = S_To_R_Coord(VPR_S_Bottom) + (S_ITEM_SIZE - 1);
	if (R_B > VPR_R_Bottom)
	{
		VPR_S_Bottom--;
		OutB = S_ITEM_SIZE - (R_B - VPR_R_Bottom); /* внешняя область изображения - не покрыта матрицей */
	}
	else
		OutB = 0;

	/* выравнивание позиции изображения по отношению к ячейкам матрицы */
	if (OutL + OutR >= S_ITEM_SIZE)
	{
		VPR_S_Left--; /* включение самой левой колонки */
		VPR_R_Left -= (S_ITEM_SIZE - OutL);
	}

	/* выравнивание позиции изображения по отношению к ячейкам матрицы */
	if (OutT + OutB >= S_ITEM_SIZE)
	{
		VPR_S_Top--;  /* включение самого верхнего ряда */
		VPR_R_Top -= (S_ITEM_SIZE - OutT);
	}

	if (ImageCoordX)
		*ImageCoordX = VPR_R_Left;
	if (ImageCoordY)
		*ImageCoordY = VPR_R_Top;

	VPR_S_Offset = S_CoordToOffset(VPR_S_Left, VPR_S_Top);

	VPR_R_Left = S_To_R_Coord(VPR_S_Left);
	VPR_R_Right = S_To_R_Coord(VPR_S_Right) + S_ITEM_SIZE - 1;
	VPR_R_Top = S_To_R_Coord(VPR_S_Top);
	VPR_R_Bottom = S_To_R_Coord(VPR_S_Bottom) + S_ITEM_SIZE - 1;
	VPR_R_Offset = R_CoordToOffset(VPR_R_Left, VPR_R_Top);

	return 1;
}

SL   VPRImportImage(SI   ImageX,
					SI   ImageY,
					UC   *ImageBuffer,
					UC   *InternalExtendedImageBuffer)
{
	UC  *ptr1, *ptr2;
	SI  CoordX, CoordY, y;

	if (!VPRImagePosition(ImageX, ImageY, &CoordX, &CoordY))
		return VPR_ERROR;

	memset(InternalExtendedImageBuffer, 255, R_X*R_Y);

	for (y = 0, ptr1 = InternalExtendedImageBuffer + CoordY*R_X + CoordX, ptr2 = ImageBuffer;
		y < ImageY;
		y++, ptr1 += R_X, ptr2 += ImageX)
		memcpy(ptr1, ptr2, ImageX);

	return VPR_SUCCESS;
}
