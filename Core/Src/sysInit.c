/*
 * sysInit.c
 *
 *  Created on: Jul 27, 2020
 *      Author: Дмитрий
 */

/*************************************************** Library ******************************************************************/

#include "sysInit.h"

/************************************************ Use functions ***************************************************************/

void Mux_GPIO(void) {

	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_CNF5 | GPIO_CRL_CNF1 | GPIO_CRL_CNF3);
	GPIOA->CRL |=  (GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE3_0);

	GPIOB->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_CNF0);
	GPIOB->CRL |=  (GPIO_CRL_MODE1_0 | GPIO_CRL_MODE0_0);

	GPIOC->CRL &= ~(GPIO_CRL_CNF5);
	GPIOC->CRL |=  (GPIO_CRL_MODE5_0);

}
