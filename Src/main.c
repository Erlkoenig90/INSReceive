#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stm32f407xx.h>

/*
 * Pin Configuration:
 * PD9 - USART RX
 * PD15 - Blue LED
 * PD14 - Red LED
 * PD13 - Orange LED
 * PD12	- Green LED
 */

static uint8_t rxBuffer [176] __attribute__ ((aligned (16)));
static DMA_Stream_TypeDef* const dmaStream = DMA1_Stream1;
static unsigned int state = 0;
extern uint32_t g_pfnVectors;


void USART3_IRQHandler (void) {
	if (USART3->SR & USART_SR_IDLE) {
		// Clear Interrupt via dummy read
		(void) USART3->DR;
		// Turn off orange LED
		GPIOD->BSRR = GPIO_BSRR_BR13;

		switch (state) {
			case 0:
				// First IDLE detected. Do nothing special.
				break;
			case 1:
				// IDLE has been detected without a DMA interrupt. This should not happen.
//				__BKPT ();	// Breakpoint for debugger.

				// Turn on red LED
				GPIOD->BSRR = GPIO_BSRR_BS14;

				break;
			case 2:
				// DMA Completion and IDLE has happened. A packet has been properly received.
				if (rxBuffer [0] == 0xFA && rxBuffer [160] == 0x27 && rxBuffer [161] == 0x10) {
					// Turn on blue LED, Turn off red LED
					GPIOD->BSRR = GPIO_BSRR_BS15 | GPIO_BSRR_BR14;
				} else {
					// Turn off blue LED
					GPIOD->BSRR = GPIO_BSRR_BR15;
				}
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
	if (DMA1->LISR & DMA_LISR_TCIF1) {
		state = 2;

		// Disable DMA stream properly
		dmaStream->CR = 0;
		while ((dmaStream->CR & DMA_SxCR_EN) != 0);

		// Clear Interrupt flags
		DMA1->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

		// Turn on orange LED
		GPIOD->BSRR = GPIO_BSRR_BS13;
	}
}

// Dummy Timer ISR to simulate high workload
void TIM8_UP_TIM13_IRQHandler () {
	if (TIM13->SR & TIM_SR_UIF) {
		TIM13->SR = ~TIM_SR_UIF;

		__NOP ();
	}
}


void initSysClock (void) {
	RCC->APB1ENR = RCC_APB1ENR_PWREN;
	RCC->APB2ENR = 0;
	// Enable HSE
	RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSEON | RCC_CR_HSION;
	// Wait for HSE ready
	while ((RCC->CR & RCC_CR_HSERDY) == 0);
	// Configure PLL to 168 MHz
	static const uint32_t PLL_M = 5;
	static const uint32_t PLL_N = 210;
	static const uint32_t PLL_P = 2;
	static const uint32_t PLL_Q = 7;
	// Select regulator voltage output Scale 1 mode
	PWR->CR = PWR_CR_VOS;
	// HCLK = SYSCLK / 1
	const uint32_t baseCFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR = baseCFGR;
	// Configure the main PLL
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16)| (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
	// Enable the main PLL
	RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_PLLON;
	// Wait till the main PLL is ready
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);
	// Configure Flash prefetch, Instruction cache, Data cache and wait state
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
	// Select the main PLL as system clock source
	RCC->CFGR = RCC_CFGR_SW_PLL | baseCFGR;
	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	// Disable HSI
	RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSEON | RCC_CR_PLLON;
}

void initGPIO (void) {
	// Initialize PD15-PD12 as output, and PD9 as AF for USART
	GPIOD->MODER = GPIO_MODER_MODE15_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODE9_1;
	GPIOD->OTYPER = 0;
	GPIOD->OSPEEDR = GPIO_OSPEEDER_OSPEEDR15_1 | GPIO_OSPEEDER_OSPEEDR15_0 | GPIO_OSPEEDER_OSPEEDR14_1 | GPIO_OSPEEDER_OSPEEDR14_0
					| GPIO_OSPEEDER_OSPEEDR13_1 | GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR12_1 | GPIO_OSPEEDER_OSPEEDR12_0;
	GPIOD->PUPDR = 0;
	GPIOD->ODR = 0;

	GPIOD->AFR[1] = 7 << 4;
}

void initIRQ () {
	// Configure priority: 2 Bits priority, 2 bits subpriority
	SCB->AIRCR = 0x05FA0500;

	NVIC->IP[USART3_IRQn] = 0;				// Priority 0, Subpriority 0
	NVIC->IP[DMA1_Stream1_IRQn] = 0x10;		// Priority 0, Subpriority 1
	NVIC->IP[TIM8_UP_TIM13_IRQn] = 0x40;	// Priority 1, Subpriority 0

	NVIC->ISER[USART3_IRQn >> 5UL] = 1UL << (USART3_IRQn & 0x1F);
	NVIC->ISER[DMA1_Stream1_IRQn >> 5UL] = 1UL << (DMA1_Stream1_IRQn & 0x1F);
	NVIC->ISER[TIM8_UP_TIM13_IRQn >> 5UL] = 1UL << (TIM8_UP_TIM13_IRQn & 0x1F);
}

int main (void) {
	SCB->VTOR = (uint32_t) &g_pfnVectors;

	initSysClock ();

	// Enable peripheral clocks
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN | RCC_APB1ENR_USART3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIODEN;

	initGPIO ();
	initIRQ ();

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

	while (1) {
		// Sleep to save power.
//		__WFI();

		// Toggle green LED to show processor idle state
		GPIOD->BSRR = GPIO_BSRR_BS12;
		__NOP ();
		GPIOD->BSRR = GPIO_BSRR_BR12;
		__NOP ();
	}
}

