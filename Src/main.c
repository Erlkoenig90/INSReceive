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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t rxBuffer [176] __attribute__ ((aligned (16)));
static DMA_Stream_TypeDef* const dmaStream = DMA1_Stream1;
static unsigned int state = 0;
static unsigned int printCounter = 0;
extern uint32_t g_pfnVectors;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void USART3_IRQHandler (void) {
	if (USART3->SR & USART_SR_IDLE) {
		// Clear Interrupt via dummy read
		(void) USART3->DR;
		switch (state) {
			case 0:
				// First IDLE detected. Do nothing special.
				break;
			case 1:
				// IDLE has been detected without a DMA interrupt. This should not happen.
				printf ("Reception failed: NDTR = %lu, LISR = 0x%lx\n", dmaStream->NDTR, DMA1->LISR);
				printCounter = 0;
				break;
			case 2:
				// DMA Completion and IDLE has happened. A packet has been properly received.
				if (rxBuffer [0] == 0xFA && rxBuffer [160] == 0x27 && rxBuffer [161] == 0x10) {
					if (printCounter == 99) {
						puts ("Received 100 packets OK");
						printCounter = 0;
					} else {
						++printCounter;
					}
				} else
					puts ("Packet received, but is invalid");
				break;
		}
		state = 1;
		
		// Disable DMA stream properly
		dmaStream->CR = 0;
		while ((dmaStream->CR & DMA_SxCR_EN) != 0);
		
		// Clear Interrupt flags
		DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
	
		// Make sure buffer is correctly aligned
		uint32_t mptr = (uint32_t) rxBuffer;
		assert_param (mptr % 16 == 0);
		
		// (Re-)Initialize DMA
		dmaStream->PAR = (uint32_t) (&USART3->DR);
		dmaStream->M0AR = mptr;
		dmaStream->NDTR = 162;
		dmaStream->FCR = DMA_SxFCR_DMDIS;
		dmaStream->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | DMA_SxCR_MSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_EN | DMA_SxCR_TCIE;

		USART3->CR3 = USART_CR3_DMAR;
	}
}

void DMA1_Stream1_IRQHandler (void) {
	if (DMA1->LISR & DMA_LISR_TCIF1)
		state = 2;
}

// Dummy Timer ISR to simulate high workload
void TIM8_UP_TIM13_IRQHandler () {
	if (TIM13->SR & TIM_SR_UIF) {
		TIM13->SR = ~TIM_SR_UIF;

		__NOP ();
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	// Set VTOR register to point to ISR Vector, in case code is executed from RAM
	SCB->VTOR = (uint32_t) &g_pfnVectors;
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

  /* USER CODE BEGIN 2 */
  puts ("Application startup");


	// Configure interrupts
	HAL_NVIC_SetPriority (TIM8_UP_TIM13_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ (TIM8_UP_TIM13_IRQn);

	HAL_NVIC_SetPriority (USART3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ (USART3_IRQn);

	HAL_NVIC_SetPriority (DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ (DMA1_Stream1_IRQn);

	// Enable peripheral clocks
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// Initialize TIM13 to call the interrupt at 50kHz, which simulates some dummy load
	TIM13->PSC = 83;
	TIM13->DIER = TIM_DIER_UIE;
	TIM13->CR1 = 0;
	TIM13->SR = ~TIM_SR_UIF;

	TIM13->ARR = 19;
	TIM13->CR1 = TIM_CR1_URS;
	TIM13->EGR = TIM_EGR_UG;
	TIM13->CR1 = TIM_CR1_CEN;

	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM13_STOP;


	// Initialize UsART3 for reception
	USART3->BRR = 46;	// 921600 Baud.
	USART3->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE; // Only enable IDLE interrupt

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	
	// Sleep to save power.
	__WFI ();
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD9   ------> USART3_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  printf ("Error at %s:%d\n", file, line);
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
	
  printf ("Assertion failed at %s:%lu\n", file, line);
  while (1) {
	  __BKPT (0);
  }
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
