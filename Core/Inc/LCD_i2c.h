

#ifndef INC_LCD_I2C_H_
#define INC_LCD_I2C_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"


/* Typedef -------------------------------------------------------------------*/
#define LCD_I2CType I2C_HandleTypeDef*

typedef struct {
  LCD_I2CType I2C;
  uint8_t Address;
  uint32_t Timeout;
} LCD_HandleTypeDef;


/* Define --------------------------------------------------------------------*/
#define LCD_ADDRESS (0x27 << 1)

/* Public function prototypes ------------------------------------------------*/
void lcd_init (LCD_HandleTypeDef* hLCD);

void lcd_send_cmd (LCD_HandleTypeDef* hLCD, char cmd);

void lcd_send_data (LCD_HandleTypeDef* hLCD, char data);

void lcd_send_string (LCD_HandleTypeDef* hLCD, char *str);

#endif /* INC_LCD_I2C_H_ */
