#ifndef  VPR_H
#define  VPR_H

#define VPR_API __declspec(dllexport)

typedef struct
{
	unsigned char *InputImage;     // указатель на буфер размера <ImageSizeX>*<ImageSizeY>, выделенный приложением, 
							       // содержащий входное изображение рисунка вен; (вены темнее, а участки ткани светлее)
	unsigned long ImageSizeX;      // ширина входного изображения в пикселях;
	unsigned long ImageSizeY;      // высота входного изображения в пикселях;
	unsigned long Resolution;      // разрешение изображения в dpi: 0-неизвестное, ожидается 250-500 точек на дюйм
	int	ExpectedOrienation;        // 0-неизвестно, 1-палец ориентирован вертикально, 2-горизонтально
	int	ExpectedNumberOfFingers;   // 0-неизвестно, 1-4 
}
VPR_IMAGE_DESCRIPTOR;  

//----------------------------------------------------------------------------------------------------------
// Функция выполняет одну или несколько запрошенных операций с заданным изображением рисунка вен.
//----------------------------------------------------------------------------------------------------------
// Коды возврата: 
#define VPR_ERROR	 0 // произошла ошибка, запрошенная операция не была выполнена
#define VPR_SUCCESS  1 // операция успешно завершена
//----------------------------------------------------------------------------------------------------------
// Параметры:
// <Operation> - возможные операции обработки:
#define VPR_BACKGROUND_CLEAN  0x00000001 // обнаружение внутренней области пальца, содержащей рисунок вен (ROI), 
										 // и очистка фона вокруг ROI в буфере <*InputImage>
// <*VPRDescriptor> - указатель на дескриптор изображения рисунка вен 
//----------------------------------------------------------------------------------------------------------
extern "C" VPR_API int	__stdcall 
VPR_Process(unsigned int   Operation,
		    VPR_IMAGE_DESCRIPTOR *VPRDescriptor);

#endif

