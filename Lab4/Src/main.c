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
#include "stdio.h"

void setLED1(void);
void setLED2(char color);
void colorChoice(void);
void transmitChar(char c);
void transmitStr(const char* str);

volatile int newDataFlag = 0;
volatile char savedChar;

void _Error_Handler(char * file, int line);

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

void setLED1(void) {
  switch(USART3->RDR) {
    case 'r':
      GPIOC -> ODR ^= GPIO_ODR_6;
      break;
    
    case 'b':
      GPIOC -> ODR ^= GPIO_ODR_7;
      break;

    case 'o':
      GPIOC -> ODR ^= GPIO_ODR_8;
      break;

    case 'g':
      GPIOC -> ODR ^= GPIO_ODR_9; 
      break; 
    
    default:
      transmitStr("Invalid selection.\n\r");
    break;
  }
}

void setLED2(char color) {
  switch(color) {
    case 'r':
      switch(savedChar) {
        case '0':
          GPIOC -> ODR &= ~ (1<<6);
          transmitStr("Red light turned off. \n\r");
        break;
        case '1':
          GPIOC -> ODR |= (1<<6);
          transmitStr("Red light turned on. \n\r");
        break;
        case '2':
          GPIOC -> ODR ^= (1<<6);
          transmitStr("Red light toggled. \n\r");
        break;
        default:
          transmitStr("Invalid selection. Please enter 0, 1, or 2. \n\r");
        break;
      }
    break;
    
    case 'b':
    switch(savedChar) {
        case '0':
          GPIOC -> ODR &= ~ (1<<7);
          transmitStr("Blue light turned off. \n\r");
        break;
        case '1':
          GPIOC -> ODR |= (1<<7);
          transmitStr("Blue light turned on. \n\r");
        break;
        case '2':
          GPIOC -> ODR ^= (1<<7);
          transmitStr("Blue light toggled. \n\r");
        break;
        default:
          transmitStr("Invalid selection. Please enter 0, 1, or 2. \n\r");
        break;
      }
    break;

    case 'o':
    switch(savedChar) {
        case '0':
          GPIOC -> ODR &= ~ (1<<8);
          transmitStr("Orange light turned off. \n\r");
        break;
        case '1':
          GPIOC -> ODR |= (1<<8);
          transmitStr("Orange light turned on. \n\r");
        break;
        case '2':
          GPIOC -> ODR ^= (1<<8);
          transmitStr("Orange light toggled. \n\r");
        break;
        default:
          transmitStr("Invalid selection. Please enter 0, 1, or 2. \n\r");
        break;
      }
    break;

    case 'g':
    switch(savedChar) {
        case '0':
          GPIOC -> ODR &= ~ (1<<9);
          transmitStr("Green light turned off. \n\r");
        break;
        case '1':
          GPIOC -> ODR |= (1<<9);
          transmitStr("Green light turned on. \n\r");
        break;
        case '2':
          GPIOC -> ODR ^= (1<<9);
          transmitStr("Green light toggled. \n\r");
        break;
        default:
          transmitStr("Invalid selection. Please enter 0, 1, or 2. \n\r");
        break;
      }
    break;
    
    default:
      transmitStr("Invalid selection. Please try again.\n\r");
    break;
  }
}

void colorChoice(void) {
  char color;

  switch(savedChar) {
    case 'r':
    case 'b':
    case 'o':
    case 'g':
      color = savedChar;
      transmitStr("CMD? (0, 1, 2)\n\r");
      while(1) {
        if(newDataFlag == 1) {
          newDataFlag = 0;
          break;
        }
      }

      setLED2(color);
    break;

    default:
      transmitStr("Invalid selection. Please enter r, b, o, or g. \n\r");
    break;
  }
}

void transmitChar(char c) {
  while(1) {
    if((USART3->ISR & (1<<7)) == (1<<7)) {
      break;
    }
  }
  USART3->TDR = c;
}

void transmitStr(const char* str) {
  int i = 0; 
	  while (str[i] != '\0') { 
      transmitChar(str[i]); 
      i++; 
    }
}

void USART3_4_IRQHandler(void) {
  if(newDataFlag == 0) {
		newDataFlag = 1;
		savedChar = USART3->RDR;
  }

}


int main(void)
{
  SystemClock_Config();
  HAL_Init();

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  GPIOB-> MODER &= ~(1 << 20); // PB10
  GPIOB -> MODER |= (1 << 21); // PB10
  GPIOB -> MODER &= ~(1 << 22); // PB11
  GPIOB -> MODER |= (1 << 23); // PB11

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

  // Setting the Baud Rate
  USART3->BRR = HAL_RCC_GetHCLKFreq()/9600;

  // Setting AFR to 4
  GPIOB -> AFR[1] |= (1<<10);
  GPIOB -> AFR[1] |= (1<<14);

  // // For part 2
  // // Enable the receive register not empty interrupt
  // USART3->CR1 |= (1<<5); 

  // // Enable and set the USART interrupt priority
  // NVIC_EnableIRQ(USART3_4_IRQn);
  // NVIC_SetPriority(USART3_4_IRQn, 3);

  // Setting TX and RX to 1
  USART3->CR1 |= (1<<0);
  USART3->CR1 |= (1<<2);
  USART3->CR1 |= (1<<3);

  // Turn on all LEDS
  GPIOC -> ODR |= GPIO_ODR_6;
  GPIOC -> ODR |= GPIO_ODR_7;
  GPIOC -> ODR |= GPIO_ODR_8;
  GPIOC -> ODR |= GPIO_ODR_9;

  while(1) {
    // HAL_Delay(1000); // Delay 200ms
    // GPIOC -> ODR ^= GPIO_ODR_6;

    // // Testing transmit char
    // transmitChar('k');

    // // Testing transmit string
    // const char* str = "hello ";
    // transmitStr(str);

    // Part 1
    while(1) {
      if((USART3->ISR & (1<<5)) == (1<<5)) {
        break;
     }
    }

    setLED1();

    // Part 2

    // transmitStr("Color? (r, b, o, g)\n\r");
    // while(1) {
    //   if(newDataFlag == 1){
    //     newDataFlag = 0;
    //     break;
    //   }
    // }

    // colorChoice();
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
