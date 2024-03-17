/**
  *
  * Brandon Mouser
  * U0962682
  *
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
#include "stm32f0xx_hal.h"
#include "stdlib.h"
void _Error_Handler(char * file, int line);
void setGyroscopeLEDS(int16_t x, int16_t y);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void setGyroscopeLEDS(int16_t x, int16_t y) {
  int tolerance = 1500;

  if((x > tolerance) | (x < -tolerance) | (y > tolerance) | (y < -tolerance)) {
    if (abs(x) > abs(y)) {
      if (x < 0) { // set orange
        GPIOC->ODR |= (1 << 8);
        GPIOC->ODR &= ~(1 << 6);
        GPIOC->ODR &= ~(1 << 7);
        GPIOC->ODR &= ~(1 << 9);
      } else if (x > 0) { // set green
        GPIOC->ODR |= (1 << 9);
        GPIOC->ODR &= ~(1 << 6);
        GPIOC->ODR &= ~(1 << 7);
        GPIOC->ODR &= ~(1 << 8);
      } else {
        // Reset all LEDs
        GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
      }
    } else {
      if (y < 0) { // set blue
        GPIOC->ODR |= (1 << 7);
        GPIOC->ODR &= ~(1 << 6);
        GPIOC->ODR &= ~(1 << 8);
        GPIOC->ODR &= ~(1 << 9);
      } else if (y > 0) { // set red
        GPIOC->ODR |= (1 << 6);
        GPIOC->ODR &= ~(1 << 7);
        GPIOC->ODR &= ~(1 << 8);
        GPIOC->ODR &= ~(1 << 9);
      } else {
        // Reset all LEDs
        GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
      }
    }
  } else {
    // Reset all LEDs
    GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
  }
}


int main(void)
{
  SystemClock_Config();

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Set up LEDs
  GPIOC -> MODER |= (1 << 12); // PIN 6 (RED)
	GPIOC -> MODER |= (1 << 14); // PIN 7 (BLLUE)
	GPIOC -> MODER |= (1 << 16); // PIN 8 (ORANGE)
	GPIOC -> MODER |= (1 << 18); // PIN 9 (GREEN)

  GPIOC -> OTYPER &= ~(1 << 6);
	GPIOC -> OTYPER &= ~(1 << 7);
	GPIOC -> OTYPER &= ~(1 << 8);
	GPIOC -> OTYPER &= ~(1 << 9);

	GPIOC -> OSPEEDR &= ~(1 << 12);
	GPIOC -> OSPEEDR &= ~(1 << 14);
	GPIOC -> OSPEEDR &= ~(1 << 16);
	GPIOC -> OSPEEDR &= ~(1 << 18);

	GPIOC -> PUPDR &= ~((1 << 12)|(1 << 13));
	GPIOC -> PUPDR &= ~((1 << 14)|(1 << 15));
	GPIOC -> PUPDR &= ~((1 << 16)|(1 << 17));
	GPIOC -> PUPDR &= ~((1 << 18)|(1 << 19));

  GPIOB -> MODER |= (1 << 23); // PB11 Alternate function
  GPIOB -> MODER &= ~(1 << 22); // PB11 Alternate function

  GPIOB -> OTYPER |= (1 << 11); // PB11 Output open-drain

  GPIOB -> AFR[1] |= (1<<12); // Set alternate function mode
	GPIOB -> AFR[1] &= ~(1<<13);
	GPIOB -> AFR[1] &= ~(1<<14);
	GPIOB -> AFR[1] &= ~(1<<15);

  GPIOB -> MODER |= (1 << 27); // PB13 Alternate function
  GPIOB -> MODER &= ~(1 << 26); // PB13 Alternate function

  GPIOB -> OTYPER |= (1 << 13); // PB13 Output open-drain

  GPIOB -> AFR[1] |= (1<<20); // Set alternate function mode
	GPIOB -> AFR[1] &= ~(1<<21);
	GPIOB -> AFR[1] |= (1<<22);
	GPIOB -> AFR[1] &= ~(1<<23);

  GPIOB -> MODER |= (1 << 28); // PB14 output mode
  GPIOB -> MODER &= ~(1 << 29); // PB14 ouput mode

  GPIOB -> OTYPER &= ~(1 << 14); // PB14 push pull

  GPIOB -> ODR |= (1 << 14); // Set pin 14 to high

  GPIOC -> MODER |= (1 << 0); // PC0 output mode
  GPIOC -> MODER &= ~(1 << 1); // PC0 ouput mode
 
  GPIOC -> OTYPER &= ~(1 << 0); // PC0 push pull

  GPIOC -> ODR |= (1 << 0); // Set pin 0 to high

  // Setting I2C2 to 1kHz
  I2C2 -> TIMINGR |= 0x13;
  I2C2 -> TIMINGR |= (0xF << 8);
  I2C2 -> TIMINGR |= (0x2 << 16);
  I2C2 -> TIMINGR |= (0x4 << 20);
  I2C2 -> TIMINGR |= (1 << 28);

  I2C2 -> CR1 |= (1 << 0);

  
  // Setting slave address to write 2 bytes
  I2C2 -> CR2 |= (0x69 << 1);
  I2C2 -> CR2 |= (1 << 17);
	I2C2 -> CR2 &= ~(1 << 16);
  I2C2 -> CR2 &= ~(1 << 10);
  I2C2 -> CR2 |= (1 << 13);

  // Wait for TXIS and NAXKF
  while (1){
    if(I2C2->ISR & (1<<1)){ // TXIS
      break;
    }
    if(I2C2->ISR & (1<<4)){ // NACKF
    }
  }

  // I2C2->TXDR |= 0x0f; // Who am I register
  I2C2->TXDR = 0x20; // Control Register 1

  // Wait for TXIS and NAXKF
  while (1){
    if(I2C2->ISR & (1<<1)){ // TXIS
      break;
    }
    if(I2C2->ISR & (1<<4)){ // NACKF
    }
  }

  I2C2->TXDR = 0x0B; // Set PD to 1, enable x, y

  while(1){
    if(I2C2->ISR & (1<<6)){ // Transfer Complete
      break;
    }
  }

  // Reading X and Y values

  while(1) { // Loop to read gyro data
  
    // Initial write to tell where to read the values from 
    I2C2 -> CR2 |= (0x69 << 1); // slave address
    I2C2 -> CR2 |= (1 << 16); // write 1 value
	  I2C2 -> CR2 &= ~(1 << 17);
    I2C2 -> CR2 &= ~(1 << 10); // write
    I2C2 -> CR2 |= (1 << 13); //start

    while (1){
      if(I2C2->ISR & (1<<1)){ // TXIS
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }

    I2C2->TXDR = 0xA8; // Data for x-axis

    while(1){ 
      if(I2C2->ISR & (1<<6)){ // Transfer Complete
        break;
      }
    }

    // Setting slave address to read 2 bytes for x
    I2C2 -> CR2 |= (0x69 << 1); // slave address
    I2C2 -> CR2 |= (1 << 17); // read 2 bytes
	  I2C2 -> CR2 &= ~(1 << 16);
    I2C2 -> CR2 |= I2C_CR2_RD_WRN; // read
    I2C2 -> CR2 |= I2C_CR2_START; // start

    char xhi;
    char xlo;
    int16_t x;
 
    // Wait for RXNE
    while (1){
      if(I2C2->ISR & (1<<2)){ // RXNE
        xlo = I2C2 -> RXDR;
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }
    while (1){
      if(I2C2->ISR & (1<<2)){ // RXNE
        xhi = I2C2 -> RXDR;
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }

    x = ((xhi << 8) | xlo);

    // Read 16 bits from y
    I2C2 -> CR2 |= (0x69 << 1); // Read 16 bits from y
    I2C2 -> CR2 |= (1 << 16);
	  I2C2 -> CR2 &= ~(1 << 17);
    I2C2 -> CR2 &= ~(1 << 10);
    I2C2 -> CR2 |= (1 << 13);

    while (1){
      if(I2C2->ISR & (1<<1)){ // TXIS
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }

    I2C2->TXDR = 0xAA; // Data for y-axis

    while(1){
      if(I2C2->ISR & (1<<6)){  // Transfer Complete
        break;
      }
    }

    // Initial write to tell where to read the values from 
    I2C2 -> CR2 |= (0x69 << 1);
    I2C2 -> CR2 |= (1 << 17);
	  I2C2 -> CR2 &= ~(1 << 16);
    I2C2 -> CR2 |= I2C_CR2_RD_WRN;
    I2C2 -> CR2 |= I2C_CR2_START;

    char yhi;
    char ylo;
    int16_t y;

    // Wait for RXNE
    while (1){
      if(I2C2->ISR & (1<<2)){ // RXNE
        ylo = I2C2 -> RXDR;
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }

    while (1){
      if(I2C2->ISR & (1<<2)){ // RXNE
        yhi = I2C2 -> RXDR;
        break;
      }
      if(I2C2->ISR & (1<<4)){ // NACKF
      }
    }

    y = ((yhi << 8) | ylo);
      
    // Wait for transfer complete
    // while(1){
    //   if(I2C2->ISR & (1<<6)){  // Transfer Complete
    //     break;
    //   }
    // }

    I2C2 -> CR2 |= (1 << 14); // Stop

    // set gyroscope leds
    setGyroscopeLEDS(x, y);

    HAL_Delay(100);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
