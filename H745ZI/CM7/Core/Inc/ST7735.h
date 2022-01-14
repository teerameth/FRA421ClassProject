/*
 * ST7735.h
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#ifndef INC_ST7735_H_
#define INC_ST7735_H_
#include "stm32h7xx_hal.h"
typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *RstPort,*CSPort,*DCPort;
	uint32_t  RstPin;
	uint32_t  CSPin;
	uint32_t  DCPin;

}LCDHandle;
#define LCD_BUFFER_SIZE 3*128*128
extern uint8_t Framememory[LCD_BUFFER_SIZE];

void LCD_init(LCDHandle *lcd);
void LCD_flush(LCDHandle *lcd);
uint8_t* LCDBufferAddr();

#endif /* INC_ST7735_H_ */
