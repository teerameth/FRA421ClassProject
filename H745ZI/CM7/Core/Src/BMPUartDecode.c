/*
 * PNGUartDecode.c
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#include "BMPUartDecode.h"
typedef enum {
	BMP_idle,
	BMP_Header_2,
	BMP_Size_4,
	BMP_Reserved0_4,
	BMP_Imagestartpoint_4,
	BMP_SizeHeader_4,
	BMP_PicWidth_4,
	BMP_PicHeight_4,
	BMP_ColorPlanes_2,
	BMP_BitPerPixel_2,
	BMP_Notused1_n,
	BMP_Pixeldata_n,
	BMP_Notused2_n

} stateBMP;

typedef union U8ToU32Convert {
	uint8_t U8[4];
	uint32_t U32;
} Convert8_32;

#define IMG_W 128
#define IMG_H 128
static stateBMP State = 0;
void BMPDecoder(uint8_t dataIn, uint8_t *array) {

	static Convert8_32 size, StartPoint, HeaderSize, PW, PH, BPS;
	static uint32_t Substate, offset, imageSize;

	switch (State) {
	case BMP_idle:

		if (dataIn == 0x42) {
			State = BMP_Header_2;

		}
		Substate = 0;
		offset = 0;
		break;

	case BMP_Header_2:
		if (dataIn == 0x4D) {
			State = BMP_Size_4;
			Substate = 0;
		} else {
			State = BMP_idle;
		}
		break;
	case BMP_Size_4:
		size.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_Reserved0_4;
			Substate = 0;
		}
		break;
	case BMP_Reserved0_4:
		Substate++;
		if (Substate == 4) {
			State = BMP_Imagestartpoint_4;
			Substate = 0;
		}
		break;

	case BMP_Imagestartpoint_4:
		StartPoint.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_SizeHeader_4;
			Substate = 0;
		}
		break;

	case BMP_SizeHeader_4:
		HeaderSize.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			if (HeaderSize.U32 == 40) {
				State = BMP_PicWidth_4;
				Substate = 0;
			} else {
				State = BMP_idle;
			}
		}
		break;
	case BMP_PicWidth_4:
		PW.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_PicHeight_4;
			Substate = 0;
		}
		break;
	case BMP_PicHeight_4:
		PH.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_ColorPlanes_2;
			Substate = 0;
		}
		break;
	case BMP_ColorPlanes_2:

		Substate++;
		if (Substate == 2) {
			State = BMP_BitPerPixel_2;
			Substate = 0;
		}
		break;
	case BMP_BitPerPixel_2:
		BPS.U8[Substate] = dataIn;
		Substate++;
		if (Substate == 4) {
			State = BMP_Notused1_n;
			Substate = 0;
		}
		break;
	case BMP_Notused1_n:
		if (offset == StartPoint.U32) {
			State = BMP_Pixeldata_n;
			array[0] = dataIn;
			Substate = 1;
		}
		break;
	case BMP_Pixeldata_n:
		if ((Substate / (IMG_W * 3)) < PH.U32) {

			if ((Substate % (IMG_W * 3)) < (PW.U32 * 3)) {
				array[Substate++] = dataIn;
			}
			else if(((Substate) % 4))
			{
				array[Substate++] = dataIn;
			}
			else
			{
				while ((Substate % (IMG_W * 3)) != 0) {
					array[Substate++] = 0; 	//fill blankdata with black
				}
				array[Substate++] = dataIn;
			}
		} else {
			while (Substate / (IMG_W * 3) < IMG_H) {
				array[Substate++] = 0; 	//fill blankdata with black
			}

			State = BMP_Notused2_n;

		}
		if (offset >= size.U32-1) {
							State = BMP_idle;
							while (Substate / (IMG_W * 3) < IMG_H) {
										array[Substate++] = 0; 	//fill blankdata with black
									}
						}
		break;
	case BMP_Notused2_n:
		if (offset >= size.U32-1) {
			State = BMP_idle;

		}
		break;

	}
	offset++;

}
