/*
 * libPWM.c
 *
 *  Created on: Jul 27, 2020
 *      Author: Дмитрий
 */


/*************************************************** Library ******************************************************************/

#include "libPWM.h"

/************************************************ Use functions ***************************************************************/

// задаем значение делителей для частоты и сдвиг фазы
void PWM_Init(uint32_t psc, uint32_t arr, uint32_t duty) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	//PA6 push-pull channel 1
	GPIOA->CRL &= ~GPIO_CRL_CNF6;
	GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE6;
	GPIOA->CRL |= GPIO_CRL_MODE6;

	//PA7 push-pull channel 2
	GPIOA->CRL &= ~GPIO_CRL_CNF7;
	GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->CRL &= ~GPIO_CRL_MODE7;
	GPIOA->CRL |= GPIO_CRL_MODE7;

	// F for APB1 36 MHz
	TIM3->PSC = psc - 1;   	//clk_tim = input clk / psc / ARR  F tim = 10kHz (input clk = F for APB1)
	TIM3->ARR = arr;
	TIM3->CCR1 = 0;					// у канала 2 сдвиг отсутствует
	TIM3->CCR2 = duty;			// здесь мы задаем значение на которое хотим получить сдвиг относительно 1 сигнала (2 канала)

//	TIM2->CCR2 = duty + 450;
//	TIM2->CCR4 = duty;

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0; 	//toggle mode
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER &= ~TIM_CCER_CC1P;

	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;		//toggle mode
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->CCER &= ~TIM_CCER_CC2P;

	TIM3->CR1 &= ~TIM_CR1_DIR;
	//TIM2->CR1 |= TIM_CR1_CEN;

}

void tim_soft(void) {

	// 16000000/PSC/ARR = частота срабатывания таймера
	// 1 ms - 1 kHz
	// 16000000/PSC/ARR = 1000
	// 16000000/16/1000 = 1000 < 65535
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 				// вкл. тактирование
	TIM3->PSC = 8 - 1;
	//TIM3->PSC = 16000 - 1;
	TIM3->ARR = 1000;
	TIM3->DIER |= TIM_DIER_UIE; 								// разр прер.
	NVIC_EnableIRQ(TIM3_IRQn); 									// разр. прер. в NVIC
	// глобальное разрешение прерываний
	TIM3->CR1 |= TIM_CR1_CEN; 									// включить таймер

}

// Измерение длительности входного сигнала
void measure_init(void){

	// измеритель
	// TIM3__CH1 PA0
	// соединить PA1 (выход генератора) с PA0 (вход измерителя)

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	//PA0 push-pull
	GPIOA->CRL &= ~GPIO_CRL_CNF0;
	GPIOA->CRL |= GPIO_CRL_CNF0_1;

	GPIOA->CRL	&= ~GPIO_CRL_MODE0;
	GPIOA->CRL	|= GPIO_CRL_MODE0_1;

/**************************************************/
//	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
//	TIM2->CCMR1 |= (TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1);
//	TIM2->CCER &= ~TIM_CCER_CC1P;
//	TIM2->CCMR1 &= ~TIM_CCMR1_IC1PSC;
//	TIM2->CCER |= TIM_CCER_CC1E;
//	TIM2->DIER |= TIM_DIER_CC1IE;
//	TIM2->CR1 |= TIM_CR1_CEN;
/**************************************************/

	//////////////////////////////////////
	// 36000000/2 = 1000000 (1 мкс)
	TIM2->PSC = 36 - 1; // счет в мкс
	TIM2->ARR = 0xFFFF; // максимальное значение
	//
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; 	// 01: CC1 channel is configured as input, IC1 is mapped on TI1
	TIM2->CCER |= TIM_CCER_CC1E; 			// подключить сигнал к ножке
	TIM2->DIER |= TIM_DIER_CC1IE; 		// разр. прер. СС1
	NVIC_EnableIRQ(TIM2_IRQn); 				// разр. прер. в NVIC
	TIM2->CR1 |= TIM_CR1_CEN; 				// включить таймер

}
