/*
 * LCD_i2c.c
 *
 *  Created on: 9 sty 2021
 *      Author: swita
 */

/* Includes ------------------------------------------------------------------*/
#include "LCD_i2c.h"

/* Private variables ---------------------------------------------------------*/
const uint8_t LCD_ROW_16[] = {0x00, 0x40, 0x10, 0x50};

/* Private function ----------------------------------------------------------*/
void lcd_init (LCD_HandleTypeDef* hLCD)
{




	// 4 bit initialisation

	lcd_send_cmd (hLCD,0x33);  // 0011 0011
	lcd_send_cmd (hLCD, 0x32); // 0011 0010
	lcd_send_cmd (hLCD, LCD_FUNCTION_SET | LCD_OPT_N); // 4-bit mode


  // dislay initialisation

	 lcd_send_cmd(hLCD, LCD_CLEAR_DISPLAY);                    // Clear screen
	  lcd_send_cmd(hLCD, LCD_DISPLAY_ON_OFF_CONTROL | LCD_OPT_D);   // LCD on, Cursor off, No blink
	  lcd_send_cmd(hLCD, LCD_ENTRY_MODE_SET | LCD_OPT_INC);         // Cursor increment on

}

void lcd_clear (LCD_HandleTypeDef* hLCD)
{
	lcd_send_cmd(hLCD,LCD_CLEAR_DISPLAY);
	HAL_Delay(2);
}

void lcd_send_cmd (LCD_HandleTypeDef* hLCD, char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (hLCD->I2C, LCD_ADDRESS,(uint8_t *) data_t, 4, 100);
}


void lcd_send_data (LCD_HandleTypeDef* hLCD, char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (hLCD->I2C, LCD_ADDRESS,(uint8_t *) data_t, 4, 100);
}

void lcd_SetCursor(LCD_HandleTypeDef* hLCD, uint8_t row, uint8_t col)
{
	lcd_send_cmd(hLCD, LCD_SET_DDRAM_ADDR + LCD_ROW_16[row] + col);
}

void lcd_send_string (LCD_HandleTypeDef* hLCD, char *str, uint8_t row, uint8_t col)
{
	lcd_SetCursor(hLCD,row,col);
	HAL_Delay(1);
	while (*str) lcd_send_data (hLCD,*str++);
	//for(uint8_t i = 0; i < strlen(str); i++)
	  //  lcd_send_data(hLCD, str[i]);
}
