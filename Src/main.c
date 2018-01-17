/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "STM_MY_LCD16X2.h"
#include "stdio.h"
#include "math.h"
#include "float.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define RS 1
#define E 2

#define D4 4
#define D5 5 
#define D6 6 
#define D7 7

unsigned char buffer[6]; // I2C buffer
volatile long millis = 0; // Variable used in interrupt has to be volatile
unsigned long micros = 0,loop_timer=0 ;
int16_t gyro_x, gyro_y, gyro_z;
uint8_t lcd_loop_counter;
int16_t  angle_pitch_buffer,angle_roll_buffer;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
int32_t gyro_x_cal=0,gyro_y_cal=0,gyro_z_cal=0; // gyro calibration value
float angle_pitch, angle_roll, angle_pitch_acc,angle_roll_acc;
float angle_pitch_output,angle_roll_output;
bool set_gyro_angles = false;
char res[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void write_LCD();
void read_MPU_6050_data();
unsigned long micro();
void delay_us(unsigned long nTime);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	 unsigned short int i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  LCD_begin4BIT(GPIOA,RS,E,GPIOA,D4,D5,D6,D7);
	// Start setup MPU-6050 register
	// Activate MPU-6050
  buffer[0] = 0x6B; // Send request to the register you want to access
	buffer[1] = 0x00; // Set the requested register
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,2,100);
	// Configure gyro(500dps full scale)
	buffer[0] = 0x1B;  // Send request to the register you want to access
	buffer[1] = 0x08;  // Set the requested register
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,2,100);
	// Configure accelerometer(+/- 8g)
	buffer[0] = 0x1C;  // Send request to the register you want to access
	buffer[1] = 0x10;  // Set the requested register
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,2,100);
	// Finish setup MPU-6050 register
	
	// Print on LCD
  LCD_setCursor(1,1);
	LCD_print("   MPU6050   ");
	HAL_Delay(1500);
	LCD_clear();
	LCD_setCursor(1,1);
	LCD_print("Cal MPU-6050");
	LCD_setCursor(2,1);
  
	// Calculate gyro standstill value
	for(i=0; i<2000 ; ++i)
	{
		if(i%125 == 0)
		{
			LCD_print(".");
		}
		read_MPU_6050_data();
		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;
		HAL_Delay(3);
	}
	gyro_x_cal /= 2000;
	gyro_y_cal /= 2000;
	gyro_z_cal /= 2000;
	
	LCD_clear();
	LCD_setCursor(1,1);
	LCD_print("Pitch:");
	LCD_setCursor(2,1);
	LCD_print("Roll:");	
	millis = 0;
	loop_timer = micro();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		read_MPU_6050_data(); // Read gyro and accelerometer data

		gyro_x -= gyro_x_cal; 
		gyro_y -= gyro_y_cal;
		gyro_z -= gyro_z_cal;

		//Gyro angle calculations
		//0.0000611 = 1 / (250Hz / 65.5)
		angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
		angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
		
		//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) radians
		angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
		angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
		
		//Accelerometer angle calculations
		acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
		//57.296 = 1 / (3.142 / 180) radians
		angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
		angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
		
		//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
		angle_pitch_acc += 1.5;                                              //Accelerometer calibration value for pitch
		angle_roll_acc += 7.0;                                               //Accelerometer calibration value for roll

		if(set_gyro_angles){                                                 //If the IMU is already started
			angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
			angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
		}
		else{                                                                //At first start
			angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
			angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
			set_gyro_angles = true;                                            //Set the IMU started flag
		}
		
		angle_pitch_output = angle_pitch;
		angle_roll_output = angle_roll;
		
		write_LCD();                                                         //Write the roll and pitch values to the LCD display         

		while((micro()- loop_timer) < 4000); // Make sure the loop is in 250Hz or 4ms.
		millis = 0; // Reset millis
		loop_timer = micro();	
	  	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|E_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin 
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin 
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void print(int value)
{
	 LCD_ftoa(value,res,0);
	 LCD_print(res);
}

void write_LCD(){                                                      //Subroutine for writing the LCD
  //To get a 250Hz program loop (4ms) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch_output * 10;                      //Buffer the pitch angle because it will change
    LCD_setCursor(1,7);                                                //Set the LCD cursor to position to position 0,0
  }
  if(lcd_loop_counter == 2){
    if(angle_pitch_buffer < 0)LCD_print("-");                          //Print - if value is negative
    else LCD_print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 3)print(abs(angle_pitch_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 4)print((abs(angle_pitch_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 5)print((abs(angle_pitch_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 6)LCD_print(".");                             //Print decimal point
  if(lcd_loop_counter == 7)print(abs(angle_pitch_buffer)%10);      //Print decimal number

  if(lcd_loop_counter == 8){
    angle_roll_buffer = angle_roll_output * 10 ;
    LCD_setCursor(2,7);
  }
  if(lcd_loop_counter == 9){
    if(angle_roll_buffer < 0)LCD_print("-");                           //Print - if value is negative
    else LCD_print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 10)print(abs(angle_roll_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 11)print((abs(angle_roll_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 12)print((abs(angle_roll_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 13)LCD_print(".");                            //Print decimal point
  if(lcd_loop_counter == 14)print(abs(angle_roll_buffer)%10);      //Print decimal number
}
// To create micro second delay
unsigned long micro() 
{
	micros =  millis*1000 + 1000 - SysTick->VAL/72;
	return micros;
}
void read_MPU_6050_data()
{
	buffer[0] = 0x3B;//0x3B
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,1,100);
	HAL_I2C_Master_Receive(&hi2c1,0x68<<1,buffer,6,100);
	
	acc_x = buffer[0]<<8 | buffer[1];
	acc_y = buffer[2]<<8 | buffer[3];
	acc_z = buffer[4]<<8 | buffer[5];
	
	buffer[0] = 0x43;//0x43
	HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,1,100);
	HAL_I2C_Master_Receive(&hi2c1,0x68<<1,buffer,6,100);
	
	gyro_x = buffer[0]<<8 | buffer[1];
	gyro_y = buffer[2]<<8 | buffer[3];
	gyro_z = buffer[4]<<8 | buffer[5];
}
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	millis++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
