
/**
  ******************************************************************************
  * @file    LCD_i2c_config.c
  * @author  DŚ
  * @version V1.0
  * @date    9 sty 2020
  * @brief   config file to LCD 2x16 with i2c expander.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "LCD_i2c_config.h"
#include "LCD_i2c.h"
#include "main.h"
#include "i2c.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
LCD_HandleTypeDef hLCD_1 = {
  .I2C = &hi2c1, .Address = LCD_ADDRESS, .Timeout = 0xffff
};


/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
