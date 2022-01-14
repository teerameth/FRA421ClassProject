/*
 * ST7735.C
 *
 *  Created on: Aug 21, 2021
 *      Author: AlphaP
 */

#include "ST7735.h"

uint8_t Framememory[LCD_BUFFER_SIZE]={0};
uint8_t LCDSTARTUPSeq[]=
{
		0x01,						//SW Reset
		0x11,						//Sleep Out
		0x29,
		0x36, 0b11000000,			//seting Scan Order
		0x2a,0x00,0x00,0x00,127,	//Set C Area
		0x2b,0x00,0x00,0x00,127,	//Set R Area
		0x2c						//Write Memory
};

void LCD_init(LCDHandle *lcd){
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->RstPort, lcd->RstPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lcd->CSPort, lcd->CSPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, LCDSTARTUPSeq, 4, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[4], 1, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[5], 1, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[6], 4, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[10], 1, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[11], 4, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lcd->hspi, &LCDSTARTUPSeq[15], 1, 100);
	HAL_GPIO_WritePin(lcd->DCPort, lcd->DCPin, GPIO_PIN_SET);

}

void LCD_flush(LCDHandle *lcd)
{
	HAL_SPI_Transmit_DMA(lcd->hspi, Framememory, LCD_BUFFER_SIZE);
}

uint8_t* LCDBufferAddr()
{
	return Framememory;
}
