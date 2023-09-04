#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include "main.h"

#define  RESET_OFF 		HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET)
#define  RESET_ON 		HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET)
#define  CS_OFF 		HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)
#define  CS_ON 			HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)
#define  DC_DATA 		HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)
#define  DC_COMMAND 	HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)

#define BLACK       	0x0000
#define NAVY        	0x000F
#define DARKGREEN   	0x03E0
#define DARKCYAN    	0x03EF
#define MAROON      	0x7800
#define PURPLE      	0x780F
#define OLIVE       	0x7BE0
#define LIGHTGREY   	0xC618
#define DARKGREY    	0x7BEF
#define BLUE        	0x001F
#define GREEN       	0x07E0
#define CYAN        	0x07FF
#define RED         	0xF800
#define MAGENTA     	0xF81F
#define YELLOW      	0xFFE0
#define WHITE       	0xFFFF
#define ORANGE      	0xFD20
#define GREENYELLOW 	0xAFE5
#define PINK        	0xF81F

#define swap(a,b) 		{int16_t t=a;a=b;b=t;}

#define      LCD_Default_Max_COLUMN	240
#define      LCD_Default_Max_PAGE	320

#define      LCD_DispWindow_Start_COLUMN	0
#define      LCD_DispWindow_Start_PAGE		0

#define      LCD_DispWindow_COLUMN	320
#define      LCD_DispWindow_PAGE	320

#define      WIDTH_EN_CHAR		8
#define      HEIGHT_EN_CHAR		16

#define      CMD_Set_COLUMN		   0x2A
#define      CMD_Set_PAGE		   0x2B
#define      CMD_SetPixel		   0x2C


void ILI9341_Init(void);
void ILI9341_Reset(void);
void ILI9341_Set_Rotation(unsigned char rotation);

void LCD_FillWindow (uint32_t usPoint, uint16_t usColor);
void LCD_OpenWindow (uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight);
void LCD_DrawFilledRectangle (uint16_t usC, uint16_t usP, uint16_t usWidth, uint16_t usHeight, uint16_t usColor);
void LCD_FillScreen (uint16_t usColor);
void LCD_DrawLine (uint16_t usC1, uint16_t usP1, uint16_t usC2, uint16_t usP2, uint16_t usColor);
void LCD_DrawChar (uint16_t usC, uint16_t usP, const char cChar);
void LCD_DrawString (uint16_t usC, uint16_t usP, const char * pStr);
void LCD_DrawDot (uint16_t usC, uint16_t usP, uint16_t usColor);
void LCD_DrawEllipse (uint16_t usC, uint16_t usP, uint16_t SR, uint16_t LR, uint16_t usColor);
void LCD_DrawFormattedString(uint16_t x, uint16_t y, const char* fmt, ...);
void LCD_Print(uint16_t xc, uint16_t yc, const char* fmt, ...);
void LCD_DrawCircle(uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint16_t fillColor);
