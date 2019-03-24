/*
 * Flexi.h
 *
 *  Created on: 17 lug 2018
 *      Author: Mally
 */

#ifndef FLEXI_H_
#define FLEXI_H_
#include "stm32f4xx_hal.h"


void ReadSensor(float* sumRawP,uint32_t* adcval);
void convertAngleAcc(float*, float*, float*,uint8_t*);
void convertWeight(float*);
uint8_t checkMovement(float*);
void CheckCorrectDistribution(float*, float*,uint8_t*);
void CheckWalk(float*,uint8_t*, float*);
void updateTemp(void);
void checkTemp(void);
//uint8_t checkAngle(float*);
//void ColorLed(uint8_t, uint8_t, uint8_t);

#endif /* FLEXI_H_ */
