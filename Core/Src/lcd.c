#include <ascii.h>
#include "lcd.h"
#include "main.h"
#include <stdlib.h>
#include <stdarg.h>

static void ILI9341_SPI_Init(void);
static void ILI9341_Send_Data(unsigned char data);
static void ILI9341_Send_Command(unsigned char command);
static void ILI9341_Send_Burst(unsigned short color, unsigned long len);
static void ILI9341_Draw_Char(int x, int y, unsigned int color, unsigned int phone, unsigned char charcode, unsigned char size);


static inline void ILI9341_Send_Command(unsigned char command) {
	CS_ON;
	DC_COMMAND;
	ILI9341_SPI_Send(command);
}

static inline void ILI9341_Send_Data(unsigned char data) {
	CS_ON;
	DC_DATA;
	ILI9341_SPI_Send(data);
}

static unsigned int X_SIZE = 240;
static unsigned int Y_SIZE = 320;

unsigned char hh;
static SPI_HandleTypeDef lcd_spi;

void ILI9341_Init() {
	ILI9341_SPI_Init();

	/* Reset The Screen */
	ILI9341_Reset();
	ILI9341_Send_Command(0x01);

	/* Power Control A */
	ILI9341_Send_Command(0xCB);
	ILI9341_Send_Data(0x39);
	ILI9341_Send_Data(0x2C);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x34);
	ILI9341_Send_Data(0x02);

	/* Power Control B */
	ILI9341_Send_Command(0xCF);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0xC1);
	ILI9341_Send_Data(0x30);

	/* Driver timing control A */
	ILI9341_Send_Command(0xE8);
	ILI9341_Send_Data(0x85);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x78);

	/* Driver timing control B */
	ILI9341_Send_Command(0xEA);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x00);

	/* Power on Sequence control */
	ILI9341_Send_Command(0xED);
	ILI9341_Send_Data(0x64);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x12);
	ILI9341_Send_Data(0x81);

	/* Pump ratio control */
	ILI9341_Send_Command(0xF7);
	ILI9341_Send_Data(0x20);

	/* Power Control 1 */
	ILI9341_Send_Command(0xC0);
	ILI9341_Send_Data(0x10);

	/* Power Control 2 */
	ILI9341_Send_Command(0xC1);
	ILI9341_Send_Data(0x10);

	/* VCOM Control 1 */
	ILI9341_Send_Command(0xC5);
	ILI9341_Send_Data(0x3E);
	ILI9341_Send_Data(0x28);

	/* VCOM Control 2 */
	ILI9341_Send_Command(0xC7);
	ILI9341_Send_Data(0x86);

	/* VCOM Control 2 */
	ILI9341_Send_Command(0x36);
	ILI9341_Send_Data(0x48);

	/* Pixel Format Set */
	ILI9341_Send_Command(0x3A);
	ILI9341_Send_Data(0x55);    //16bit

	ILI9341_Send_Command(0xB1);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x18);

	/* Display Function Control */
	ILI9341_Send_Command(0xB6);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x82);
	ILI9341_Send_Data(0x27);

	/* 3GAMMA FUNCTION DISABLE */
	ILI9341_Send_Command(0xF2);
	ILI9341_Send_Data(0x00);

	/* GAMMA CURVE SELECTED */
	ILI9341_Send_Command(0x26); //Gamma set
	ILI9341_Send_Data(0x01); 	//Gamma Curve (G2.2)

	//Positive Gamma  Correction
	ILI9341_Send_Command(0xE0);
	ILI9341_Send_Data(0x0F);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0x2B);
	ILI9341_Send_Data(0x0C);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x4E);
	ILI9341_Send_Data(0xF1);
	ILI9341_Send_Data(0x37);
	ILI9341_Send_Data(0x07);
	ILI9341_Send_Data(0x10);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x09);
	ILI9341_Send_Data(0x00);

	//Negative Gamma  Correction
	ILI9341_Send_Command(0xE1);
	ILI9341_Send_Data(0x00);
	ILI9341_Send_Data(0x0E);
	ILI9341_Send_Data(0x14);
	ILI9341_Send_Data(0x03);
	ILI9341_Send_Data(0x11);
	ILI9341_Send_Data(0x07);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0xC1);
	ILI9341_Send_Data(0x48);
	ILI9341_Send_Data(0x08);
	ILI9341_Send_Data(0x0F);
	ILI9341_Send_Data(0x0C);
	ILI9341_Send_Data(0x31);
	ILI9341_Send_Data(0x36);
	ILI9341_Send_Data(0x0F);

	//EXIT SLEEP
	ILI9341_Send_Command(0x11);

	//TURN ON DISPLAY
	ILI9341_Send_Command(0x29);
	ILI9341_Send_Data(0x2C);
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void ILI9341_SPI_Init()
{
	lcd_spi.Instance 				= SPI2;
	lcd_spi.Init.Mode 				= SPI_MODE_MASTER;
	lcd_spi.Init.Direction 			= SPI_DIRECTION_2LINES;
	lcd_spi.Init.DataSize 			= SPI_DATASIZE_8BIT;
	lcd_spi.Init.CLKPolarity 		= SPI_POLARITY_LOW;
	lcd_spi.Init.CLKPhase 			= SPI_PHASE_1EDGE;
	lcd_spi.Init.NSS 				= SPI_NSS_SOFT;
	lcd_spi.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_2;
	lcd_spi.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
	lcd_spi.Init.TIMode 			= SPI_TIMODE_DISABLE;
	lcd_spi.Init.CRCCalculation 	= SPI_CRCCALCULATION_DISABLE;
	lcd_spi.Init.CRCPolynomial 		= 10;
	if (HAL_SPI_Init(&lcd_spi) != HAL_OK) {
		Error_Handler();
	}
}

void ILI9341_SPI_Send(unsigned char data)
{
	HAL_SPI_Transmit(&lcd_spi, &data, 1, 1);
}

void ILI9341_SPI_SendU16(unsigned int data)
{
	HAL_SPI_Transmit(&lcd_spi, &data, 2, 1);
}

void ILI9341_SPI_Send_Multiple(uint8_t* data, int size)
{
	HAL_SPI_Transmit(&lcd_spi, data, size, 10);
}

void ILI9341_Reset() {
	RESET_ON;
	HAL_Delay(50);
	RESET_OFF;
	CS_ON;
	HAL_Delay(50);
	ILI9341_Send_Command(0x01);
	CS_OFF;
}

void ILI9341_Set_Rotation(unsigned char rotation) {
	ILI9341_Send_Command(0x36);
	switch (rotation) {
	case 0:
		ILI9341_Send_Data(0x48);
		X_SIZE = 240;
		Y_SIZE = 320;
		break;
	case 1:
		ILI9341_Send_Data(0x28);
		X_SIZE = 320;
		Y_SIZE = 240;
		break;
	case 2:
		ILI9341_Send_Data(0x88);
		X_SIZE = 240;
		Y_SIZE = 320;
		break;
	case 3:
		ILI9341_Send_Data(0xE8);
		X_SIZE = 320;
		Y_SIZE = 240;
		break;
	}
}

void ILI9341_SPI_Send_32(unsigned char command, unsigned long data) {
	CS_ON;
	DC_COMMAND;
	ILI9341_SPI_Send(command);

	DC_DATA;
	ILI9341_SPI_Send(data >> 24);
	ILI9341_SPI_Send(data >> 16);
	ILI9341_SPI_Send(data >> 8);
	ILI9341_SPI_Send(data);

}

void ILI9341_Set_Address(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
	unsigned long t;
	t = x1;
	t <<= 16;
	t |= x2;
	ILI9341_SPI_Send_32(0x2A, t); //Column Addres Set
	t = y1;
	t <<= 16;
	t |= y2;
	ILI9341_SPI_Send_32(0x2B, t); //Page Addres Set
}

// --------

#define LCD_Write_Cmd ILI9341_Send_Command

static inline void LCD_Write_Data(uint16_t data) {
	CS_ON;
	DC_DATA;
//	ILI9341_SPI_SendU16(data);
	ILI9341_SPI_Send(data);
	ILI9341_SPI_Send(data);
}

void LCD_OpenWindow ( uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usWidth, uint16_t usHeight )
{
	ILI9341_Set_Address(usCOLUMN, usPAGE, usCOLUMN + usWidth - 1, usPAGE + usHeight - 1);
}

void LCD_FillWindow ( uint32_t usPoint, uint16_t usColor )
{
	uint32_t i = 0;

	/* memory write */
	LCD_Write_Cmd ( CMD_SetPixel );

	CS_ON;
	DC_DATA;

	int chunk_size = usPoint / 10;
	if (chunk_size < 64) chunk_size = 64;
	if (chunk_size > 240) chunk_size = 240;
	unsigned int revertedColor = (usColor >> 8) | ((usColor & 0x00ff) << 8);
	uint16_t* array = (uint16_t*) malloc(chunk_size * sizeof(uint16_t));
	for (int i = 0; i < chunk_size; i++) {
		array[i] = revertedColor;
	}

	while (usPoint > chunk_size) {
		ILI9341_SPI_Send_Multiple(array, chunk_size * 2);
		usPoint -= chunk_size;
	}
	ILI9341_SPI_Send_Multiple(array, usPoint * 2);

	free(array);

//	for ( i = 0; i < usPoint; i ++ ) {
//		ILI9341_SPI_SendU16(revertedColor);
//	}

}

void LCD_DrawFilledRectangle ( uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usWidth, uint16_t usHeight, uint16_t usColor )
{
	LCD_OpenWindow ( usCOLUMN, usPAGE, usWidth, usHeight );
	LCD_FillWindow ( usWidth * usHeight, usColor );
//	ILI9341_Send_Burst(usColor,  usWidth * usHeight);
}


void LCD_FillScreen (uint16_t usColor )
{
	LCD_DrawFilledRectangle(0, 0, X_SIZE, Y_SIZE, usColor);
}


void LCD_DrawLine ( uint16_t usC1, uint16_t usP1, uint16_t usC2, uint16_t usP2, uint16_t usColor )
{
	uint16_t us;
	uint16_t usC_Current, usP_Current;

	int32_t lError_C = 0, lError_P = 0, lDelta_C, lDelta_P, lDistance;
	int32_t lIncrease_C, lIncrease_P;


	lDelta_C = usC2 - usC1;
	lDelta_P = usP2 - usP1;

	usC_Current = usC1;
	usP_Current = usP1;


	if ( lDelta_C > 0 )
		lIncrease_C = 1;

	else if ( lDelta_C == 0 )
		lIncrease_C = 0;

	else
  	{
		lIncrease_C = -1;
		lDelta_C = - lDelta_C;
  	}


	if ( lDelta_P > 0 )
		lIncrease_P = 1;

	else if ( lDelta_P == 0 )
		lIncrease_P = 0;
	else
	{
  		lIncrease_P = -1;
    		lDelta_P = - lDelta_P;
 	}

	if (  lDelta_C > lDelta_P )
		lDistance = lDelta_C;

	else
		lDistance = lDelta_P;


	for ( us = 0; us <= lDistance + 1; us ++ )
	{
		LCD_DrawDot ( usC_Current, usP_Current, usColor );

		lError_C += lDelta_C ;
		lError_P += lDelta_P ;

		if ( lError_C > lDistance )
		{
			lError_C -= lDistance;
			usC_Current += lIncrease_C;
		}

		if ( lError_P > lDistance )
		{
			lError_P -= lDistance;
			usP_Current += lIncrease_P;
		}

	}
}

void LCD_DrawChar ( uint16_t usC, uint16_t usP, const char cChar )
{
	uint8_t ucTemp, ucRelativePositon, ucPage, ucColumn;

	ucRelativePositon = cChar - ' ';

	LCD_OpenWindow ( usC, usP, WIDTH_EN_CHAR, HEIGHT_EN_CHAR );

	LCD_Write_Cmd ( CMD_SetPixel );

	uint16_t* array = (uint16_t*) malloc(HEIGHT_EN_CHAR * WIDTH_EN_CHAR * sizeof(uint16_t));
	uint16_t* pt = array;
	for ( ucPage = 0; ucPage < HEIGHT_EN_CHAR; ucPage ++ )
	{
		ucTemp = ucAscii_1608 [ ucRelativePositon ] [ ucPage ];

		for ( ucColumn = 0; ucColumn < WIDTH_EN_CHAR; ucColumn ++ )
		{

			if ( ucTemp & 0x01 ) {
				*pt = 0x0000;
			} else {
				*pt = 0xFFFF;
			}
			ucTemp >>= 1;
			pt++;
		}
	}
	DC_DATA;
	ILI9341_SPI_Send_Multiple(array, HEIGHT_EN_CHAR * WIDTH_EN_CHAR * 2);
	free(array);
}

//Task 2
void LCD_DrawDot(uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usColor)
{
	/*
	 *  Task 2 : Implement the LCD_DrawDot to turn on a particular dot on the LCD.
	 */

	LCD_OpenWindow(usCOLUMN, usPAGE, 1, 1);
	LCD_Write_Cmd(CMD_SetPixel);
	LCD_Write_Data(usColor);

}

//Task 3
void LCD_DrawEllipse ( uint16_t usC, uint16_t usP, uint16_t SR, uint16_t LR, uint16_t usColor)
{
	/*
	 *  Task 3 : Implement LCD_DrawEllipse by using LCD_DrawDot
	 *
	 */

	/*
	 * Implementation 1:
	 */
	float y = usP;
	int y_int = usP;
	for (int x = usC - LR; x <= usC + LR; x++) {
		y = usP + SR * sqrt(1 - ((float) (x-usC)/LR) * ((float) (x-usC)/LR));
//		y = 20 * ((float) x/100) * ((float) x/100);
		LCD_DrawDot(x, y_int, usColor);
		LCD_DrawDot(x, 2 * usP - y_int, usColor);
		if (y_int > y)
			for (;y_int > y + 0.5; y_int--) {
				LCD_DrawDot(x, y_int, usColor);
				LCD_DrawDot(x, 2 * usP - y_int, usColor);
			}
		else
			for (;y_int < y - 0.5; y_int++) {
				LCD_DrawDot(x, y_int, usColor);
				LCD_DrawDot(x, 2 * usP - y_int, usColor);
			}
	}
}



void LCD_DrawString ( uint16_t usC, uint16_t usP, const char * pStr )
{
	while ( * pStr != '\0' )
	{
		if ( ( usC - LCD_DispWindow_Start_COLUMN + WIDTH_EN_CHAR ) > LCD_DispWindow_COLUMN )
		{
			usC = LCD_DispWindow_Start_COLUMN;
			usP += HEIGHT_EN_CHAR;
		}

		if ( ( usP - LCD_DispWindow_Start_PAGE + HEIGHT_EN_CHAR ) > LCD_DispWindow_PAGE )
		{
			usC = LCD_DispWindow_Start_COLUMN;
			usP = LCD_DispWindow_Start_PAGE;
		}

		LCD_DrawChar ( usC, usP, * pStr );

		pStr ++;

		usC += WIDTH_EN_CHAR;

	}

}


void LCD_DrawFormattedString(uint16_t x, uint16_t y, const char* fmt, ...) {

	char formatted_str[40];

	va_list arglist;
	va_start(arglist, fmt);

	int num_chars_to_print = vsnprintf(formatted_str, sizeof(formatted_str), fmt, arglist);
	va_end(arglist);

	if (num_chars_to_print < 0) return;
	LCD_DrawString(x, y, formatted_str);
}

void LCD_Print(uint16_t xc, uint16_t yc, const char* fmt, ...) {

	char formatted_str[40];

	va_list arglist;
	va_start(arglist, fmt);

	int num_chars_to_print = vsnprintf(formatted_str, sizeof(formatted_str), fmt, arglist);
	va_end(arglist);

	if (num_chars_to_print < 0) return;
	LCD_DrawString(xc * WIDTH_EN_CHAR, yc * HEIGHT_EN_CHAR, formatted_str);
}



//
////
//void ILI9341_Fill_Screen(unsigned int color)
//{
////
////	void LCD_DrawFilledRectangle ( uint16_t usCOLUMN, uint16_t usPAGE, uint16_t usWidth, uint16_t usHeight, uint16_t usColor )
////	{
////		LCD_OpenWindow ( usCOLUMN, usPAGE, usWidth, usHeight );
////	//	LCD_FillWindow ( usWidth * usHeight, usColor );
////		ILI9341_Send_Burst(usColor,  usWidth * usHeight);
////	}
////
////
////	void LCD_FillScreen (uint16_t usColor )
////	{
////		LCD_DrawFilledRectangle(0, 0, X_SIZE, Y_SIZE, usColor);
////	}
//
//	LCD_OpenWindow ( 0, 0, X_SIZE-1, Y_SIZE-1 );
//	ILI9341_Set_Address(0, 0, X_SIZE-1, Y_SIZE-1);
//	ILI9341_Send_Burst(color, (long)X_SIZE * (long)Y_SIZE);
//}
//
//void ILI9341_Draw_Pixel(int x, int y, unsigned int color)
//{
//	if((x < 0) || (y < 0) || (x >= X_SIZE) || (y >= Y_SIZE))
//	{
//		return;
//	}
//	ILI9341_Set_Address(x, y, X_SIZE-1, Y_SIZE-1);
//	DC_COMMAND;
//	ILI9341_SPI_Send(0x2C);
//	DC_DATA;
//	ILI9341_SPI_Send(color>>8);
//	ILI9341_SPI_Send(color);
//}
////
//void ILI9341_Draw_Filled_Rectangle(unsigned int color,unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
//{
//	ILI9341_Set_Address(x1, y1, x2, y2);
//	ILI9341_Send_Burst(color, (long)(x2-x1+1) * (long)(y2-y1+1));
//}
////
//void ILI9341_Draw_Empty_Rectangle(unsigned int color,unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
//{
//	ILI9341_Draw_Line(color, x1, y1, x2, y1);
//	ILI9341_Draw_Line(color, x2, y1, x2, y2);
//	ILI9341_Draw_Line(color, x1, y1, x1, y2);
//	ILI9341_Draw_Line(color, x1, y2, x2, y2);
//}
//
//void ILI9341_Draw_Line(unsigned int color, unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
//{
//	int steep = abs(y2-y1) > abs(x2-x1);
//
//	if (steep)
//	{
//		swap(x1,y1);
//		swap(x2,y2);
//	}
//
//	if(x1>x2)
//	{
//		swap(x1,x2);
//		swap(y1,y2);
//	}
//
//	int dx,dy;
//	dx = (x2 - x1);
//	dy = abs(y2 - y1);
//	int err = dx / 2;
//	int ystep;
//	if(y1 < y2)
//	{
//		ystep = 1;
//	}
//	else
//	{
//		ystep = -1;
//	}
//	for (; x1 <= x2; x1++)
//	{
//		if (steep)
//		{
//			ILI9341_Draw_Pixel(y1, x1, color);
//		}
//		else
//		{
//			ILI9341_Draw_Pixel(x1, y1, color);
//		}
//		err -= dy;
//		if (err < 0)
//		{
//			y1 += ystep;
//			err = dx;
//		}
//	}
//}
////
////void ILI9341_Draw_Circle(unsigned int x0, unsigned int y0, int r, unsigned int color, unsigned char flood) {
////	int f = 1 - r;
////	int ddF_x = 1;
////	int ddF_y = -2 * r;
////	int x = 0;
////	int y = r;
////	if (flood == 0) {
////		ILI9341_Draw_Pixel(x0, y0 + r, color);
////		ILI9341_Draw_Pixel(x0, y0 - r, color);
////		ILI9341_Draw_Pixel(x0 + r, y0, color);
////		ILI9341_Draw_Pixel(x0 - r, y0, color);
////		while (x < y) {
////			if (f >= 0) {
////				y--;
////				ddF_y += 2;
////				f += ddF_y;
////			}
////			x++;
////			ddF_x += 2;
////			f += ddF_x;
////			ILI9341_Draw_Pixel(x0 + x, y0 + y, color);
////			ILI9341_Draw_Pixel(x0 - x, y0 + y, color);
////			ILI9341_Draw_Pixel(x0 + x, y0 - y, color);
////			ILI9341_Draw_Pixel(x0 - x, y0 - y, color);
////			ILI9341_Draw_Pixel(x0 + y, y0 + x, color);
////			ILI9341_Draw_Pixel(x0 - y, y0 + x, color);
////			ILI9341_Draw_Pixel(x0 + y, y0 - x, color);
////			ILI9341_Draw_Pixel(x0 - y, y0 - x, color);
////		}
////	} else {
////		ILI9341_Draw_Pixel(x0, y0 + r, color);
////		ILI9341_Draw_Pixel(x0, y0 - r, color);
////		ILI9341_Set_Address(x0 - r, y0, x0 + r, y0);
////		DC_COMMAND;
////		ILI9341_SPI_Send(0x2C);
////		DC_DATA;
////		for (uint32_t fff = 0; fff < r * 2 + 1; fff++) {
////			ILI9341_SPI_Send(color >> 8);
////			ILI9341_SPI_Send(color);
////		}
////		while (x < y) {
////			if (f >= 0) {
////				y--;
////				ddF_y += 2;
////				f += ddF_y;
////			}
////			x++;
////			ddF_x += 2;
////			f += ddF_x;
////			ILI9341_Set_Address(x0 - x, y0 + y, x0 + x, y0 + y);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for (uint32_t fff = 0; fff < x * 2 + 1; fff++) {
////				ILI9341_SPI_Send(color >> 8);
////				ILI9341_SPI_Send(color);
////			}
////			ILI9341_Set_Address(x0 - x, y0 - y, x0 + x, y0 - y);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for (uint32_t fff = 0; fff < x * 2 + 1; fff++) {
////				ILI9341_SPI_Send(color >> 8);
////				ILI9341_SPI_Send(color);
////			}
////			ILI9341_Set_Address(x0 - y, y0 + x, x0 + y, y0 + x);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for (uint32_t fff = 0; fff < y * 2 + 1; fff++) {
////				ILI9341_SPI_Send(color >> 8);
////				ILI9341_SPI_Send(color);
////			}
////			ILI9341_Set_Address(x0 - y, y0 - x, x0 + y, y0 - x);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for (uint32_t fff = 0; fff < y * 2 + 1; fff++) {
////				ILI9341_SPI_Send(color >> 8);
////				ILI9341_SPI_Send(color);
////			}
////		}
////	}
////}
////
////void ILI9341_Draw_String(unsigned int x, unsigned int y, unsigned int color, unsigned int phone, char *str, unsigned char size)
////{
////	switch (size)
////	{
////	case 1:
////		while (*str)
////		{
////			if ((x+(size*8))>X_SIZE)
////			{
////				x = 1;
////				y = y + (size*8);
////			}
////			ILI9341_Draw_Char(x, y, color, phone, *str, size);
////			x += size*8-2;
////			*str++;
////		}
////	break;
////	case 2:
////		hh=1;
////		while (*str)
////		{
////			if ((x+(size*8))>X_SIZE)
////			{
////				x = 1;
////				y = y + (size*8);
////			}
////			ILI9341_Draw_Char(x,y,color,phone,*str,size);
////			x += hh*8;
////			*str++;
////		}
////	break;
////	}
////}
////
////
////
//static void ILI9341_Send_Burst(unsigned short color, unsigned long len) {
//	unsigned short blocks;
//	unsigned char i, high_bit = color >> 8, low_bit = color;
//	CS_ON;
//	DC_COMMAND;
//	ILI9341_SPI_Send(0x2C);
//	DC_DATA;
//	ILI9341_SPI_Send(high_bit);
//	ILI9341_SPI_Send(low_bit);
//	len--;
//	blocks = (unsigned short) (len / 64); //64 pixels/block
//	if (high_bit == low_bit)
//	{
//		while (blocks--) {
//			i = 16;
//			do {
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(low_bit);
//			} while (--i);
//		}
//		//Fill any remaining pixels(1 to 64)
//		for (i = (unsigned char) len & 63; i--;) {
//			ILI9341_SPI_Send(low_bit);
//			ILI9341_SPI_Send(low_bit);
//		}
//	}
//	else
//	{
//		while (blocks--) {
//			i = 16;
//			do {
//				ILI9341_SPI_Send(high_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(high_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(high_bit);
//				ILI9341_SPI_Send(low_bit);
//				ILI9341_SPI_Send(high_bit);
//				ILI9341_SPI_Send(low_bit);
//			} while (--i);
//		}
//		//Fill any remaining pixels(1 to 64)
//		for (i = (unsigned char) len & 63; i--;) {
//			ILI9341_SPI_Send(high_bit);
//			ILI9341_SPI_Send(low_bit);
//		}
//	}
//	CS_OFF;
//}
////
////void ILI9341_Draw_Double_Pixel(int x, int y, unsigned int color1, unsigned int color2)
////{
////	if((x < 0) || (y < 0) || (x >= X_SIZE) || (y >= Y_SIZE))
////	{
////		return;
////	}
////	ILI9341_Set_Address(x, y, X_SIZE-1, Y_SIZE-1);
////	DC_COMMAND;
////	ILI9341_SPI_Send(0x2C);
////	DC_DATA;
////	ILI9341_SPI_Send(color1);
////	ILI9341_SPI_Send(color2);
////}
////
////static void ILI9341_Draw_Char(int x, int y, unsigned int color, unsigned int phone, unsigned char charcode, unsigned char size)
////{
////	int i,h;
////	switch(size)
////	{
////		case 1:
////			ILI9341_Set_Address(x, y, x+5, y+8);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for(h=0;h<8;h++)
////			{
////				for(i=2;i<8;i++)
////				{
////					if ((chars8[charcode-0x20][h]>>(7-i))&0x01)
////					{
////						ILI9341_SPI_Send(color>>8);
////						ILI9341_SPI_Send(color);
////					}
////					else
////					{
////						ILI9341_SPI_Send(phone>>8);
////						ILI9341_SPI_Send(phone);
////					}
////				}
////			}
////			break;
////		case 2:
////			ILI9341_Set_Address(x, y, x+7, y+16);
////			DC_COMMAND;
////			ILI9341_SPI_Send(0x2C);
////			DC_DATA;
////			for(h=0;h<16;h++)
////			{
////				for(i=0;i<8;i++)
////				{
////					if ((chars16[charcode-0x20][h]>>(7-i))&0x01)
////					{
////						ILI9341_SPI_Send(color>>8);
////						ILI9341_SPI_Send(color);
////					}
////					else
////					{
////						ILI9341_SPI_Send(phone>>8);
////						ILI9341_SPI_Send(phone);
////					}
////				}
////			}
////		break;
////	}
////}

