# SM_Lab_Projekt
 Projekt sterowania oświetleniem z wykorzystaniem Stm32 (Nucleo F746ZG)

 
 Encoder - TIM4 - PD12,PD13 - 4000 (D28, D29)
 BH1750 - I2C1 - SCL: PB8, SDA: PD9 (D15, D14)
 Diody - TIM3 - PWM_CH1: PA6, PWM_CH2: PC7   1kHZ  (D12, D21)
 LCD I2C - I2C1 - SCL: PB8, SDA: PD9 (D15, D14)
 LCD - TIM6 - Interupts to update LCD
 USART - USART3 115200 kbps 7bit + parity even
 CRC-1 - Parity bit Even
 
 
 Parametry Regulatora PID: 
PID_TS         0.1        /* Sampling Time [s]*/
PID_KP1        0.15       /* Proporcional */
PID_KI1        0.35	      /* Integral */
PID_KD1        0.01       /* Derivative */
  
 TO DO: 
 
 
 Doxygen
 
 
 
 
 
 Parametry PID całkiem OK
#define PID_TS        0.1         /* Sampling Time [s]*/
#define PID_KP1        0.5        /* Proporcional */
#define PID_KI1        0.1        /* Integral */
#define PID_KD1        0.00005	  /* Derivative */

Parametry też ok
#define PID_TS        0.1         /* Sampling Time [s]*/
#define PID_KP1        0.15       /* Proporcional */
#define PID_KI1        0.35       /* Integral */
#define PID_KD1        0.01       /* Derivative */