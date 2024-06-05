#include "VPR_def.h"

UL	VPR_MemoryBufferSize= R_XY*4;
UC *VPR_ImgBuff1=0;
UC *VPR_TempBuff=0;

VPR_API int	__stdcall VPR_Process(unsigned int   Operation,	VPR_IMAGE_DESCRIPTOR *VPRDescriptor)
{
	UC *MemoryBuffer = (UC *)malloc(VPR_MemoryBufferSize);
	if (MemoryBuffer == 0)
		return VPR_ERROR;
	memset(MemoryBuffer, 0, VPR_MemoryBufferSize);

	VPR_ImgBuff1 =(UC *)(MemoryBuffer);
	VPR_TempBuff=(UC *)(VPR_ImgBuff1 +R_XY);

	VPRImportImage((SI)VPRDescriptor->ImageSizeX, (SI)VPRDescriptor->ImageSizeY, VPRDescriptor->InputImage, VPR_ImgBuff1);
	int Result = VPRProc(VPRDescriptor, VPR_ImgBuff1, VPR_TempBuff);

	free(MemoryBuffer);

	if(Result==0)
		return VPR_ERROR;
	return VPR_SUCCESS;
}
