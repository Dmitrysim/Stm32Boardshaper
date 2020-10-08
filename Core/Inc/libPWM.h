/*
 * libPWM.h
 *
 *  Created on: Jul 27, 2020
 *      Author: Дмитрий
 */

#ifndef INC_LIBPWM_H_
#define INC_LIBPWM_H_

#include "stm32f103x6.h"
/************************************************ Use functions ***************************************************************/

void PWM_Init(uint32_t psc, uint32_t arr, uint32_t duty);  // задаем частоту шим и сдвиг фазы
void tim_soft (void);									   // программный таймер
void measure_init(void);								   // настройка измерения длительности

#endif /* INC_LIBPWM_H_ */
