/*
 * Flexi.c
 *
 *  Created on: 17 lug 2018
 *      Author: Mally
 */

#include "Flexi.h"
#include "string.h"
#include "math.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
TIM_HandleTypeDef* htim3_;
uint8_t temp[4][2] = {{0,0},{0,0},{0,0},{0,0}};
uint8_t fine_walk[4][2] = {{1,1},{0,1},{0,0},{1,0}};
uint8_t val[2] = {0,0};
char msg2[100];
char msg1[100];
char* new_msg="Tutto su";
uint8_t countStep = 0;
//uint8_t i = 0;
uint8_t* color_;

void ReadSensor(float* sumRaw, uint32_t* adcval) {
	for (uint8_t j = 0; j < 7; j++) {
		sumRaw[j] = ((float)adcval[j]*3300) / 4095;
	}
}

void convertAngleAcc(float* sumRaw, float* acc, float* angles,uint8_t* color){
	float valx = ((sumRaw[4])-1650.0)/330;
	float valy = ((sumRaw[5])-1650.0)/330;
	float valz = ((sumRaw[6])-1650.0)/330;

	float xangle = (atan(valx/sqrt(valy*valy+valz*valz))*180/3.1412);
	float yangle = (atan(valy/sqrt(valx*valx+valz*valz))*180/3.1412);
	float zangle = (atan(sqrt(valx*valx+valy*valy)/valz)*180/3.1412);
	if (fabs(acc[0]-valx) > 1.3) {
//	sprintf(msg2, "Delta %6.2f\t%6.2f\t%6.2f LED\n",fabs(acc[0]-valx),fabs(acc[1]-valy), fabs(acc[2]-valz));
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)msg2,strlen(msg2));
	color[4] = 0;
	}
	else {
//	sprintf(msg2, "Delta %6.2f\t%6.2f\t%6.2f\n",fabs(acc[0]-valx),fabs(acc[1]-valy), fabs(acc[2]-valz));
//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)msg2,strlen(msg2));
	color[4] = 1;
	}
	acc[0]=valx;
	acc[1]=valy;
	acc[2]=valz;
	angles[0]=xangle;
	angles[1]=yangle;
	angles[2]=zangle;
}

void convertWeight(float* sumRaw){
	for(uint8_t i=0; i<4; i++){
		sumRaw[i] = (sumRaw[i]-586.94)/152.93;
		if(sumRaw[i] < 0){
			sumRaw[i] = 0;
		}

	}

	sprintf(msg2, "%.2f-%.2f-%.2f-%.2f\n", sumRaw[0],sumRaw[1],sumRaw[2],sumRaw[3]);
	HAL_UART_Transmit_IT(&huart2,(uint8_t*)msg2,strlen(msg2));

	//bluetooth transmit
	sprintf(msg1,"{\"weights\":[\"%f\",\"%f\",\"%f\",\"%f\"]}",sumRaw[0],sumRaw[1],sumRaw[2],sumRaw[3]);
	HAL_UART_Transmit(&huart6, (uint8_t*) msg1, strlen(msg1),HAL_MAX_DELAY);
}

void CheckCorrectDistribution(float* sumRaw, float* weight, uint8_t* color){
	if(color_ != color){
			color_ = color;
		}

	if(*weight != 0){
		if((sumRaw[1]+sumRaw[2]) > 0.1 && sumRaw[0] > 0.1){

			float min = sumRaw[0]-sumRaw[0]*0.20;
			float max = sumRaw[0]+sumRaw[0]*0.20;

			sprintf(msg2,"Tallone %f\t Davanti:%f\r\n",sumRaw[0],sumRaw[1]+sumRaw[2]);
			HAL_UART_Transmit_IT(&huart2,(uint8_t*)msg2,strlen(msg2));

			if((sumRaw[1]+sumRaw[2]) > min && (sumRaw[1]+sumRaw[2]) < max){
				HAL_UART_Transmit_IT(&huart2,(uint8_t*)"BUONA\n",strlen("BUONA\n"));
				color_[0]=0;
				color_[1]=0;
				color_[2]=1;

			}else{
				HAL_UART_Transmit_IT(&huart2,(uint8_t*)"Sbaglio\n",strlen("Sbaglio\n"));
				color_[0]=0;
				color_[1]=0;
				color_[2]=0;
			}
		}
	}
}

void CheckWalk(float* sumRaw, uint8_t* color, float* weight){

	if(color_ != color){
		color_ = color;
	}

	if(*weight != 0){
		if(sumRaw[0] > 3 && sumRaw[1] <1){
				  //HAL_UART_Transmit(&huart2,(uint8_t*)"Tallone giu",strlen("Tallone giu"),HAL_MAX_DELAY);
				  if(strcmp("Tallone giu",new_msg) != 0){
					  new_msg = "Tallone giu";
					  val[0]= 1;
					  val[1]= 0;
					  updateTemp();
				  }
			  }
			  if(sumRaw[0] < 1 && sumRaw[1] > 2){
				 // HAL_UART_Transmit(&huart2,(uint8_t*)"Punta giu",strlen("Punta giu"),HAL_MAX_DELAY);
				  if(strcmp("Punta giu",new_msg) != 0){
					  new_msg = "Punta giu";
					  val[0]= 0;
					  val[1]= 1;
					  updateTemp();
				  }
			  }
			  if(sumRaw[0] > 3 && sumRaw[1] > 1){
				 // HAL_UART_Transmit(&huart2,(uint8_t*)"Tutto giu",strlen("Tutto giu"),HAL_MAX_DELAY);
				  if(strcmp("Tutto giu",new_msg) != 0){
					  new_msg = "Tutto giu";
					  val[0]= 1;
					  val[1]= 1;
					  updateTemp();
				  }
			  }
			  if(sumRaw[0] <1 && sumRaw[1] <1){
				  //HAL_UART_Transmit(&huart2,(uint8_t*)"Tutto su",strlen("Tutto su"),HAL_MAX_DELAY);
				  if(strcmp("Tutto su",new_msg) != 0){
					  new_msg = "Tutto su";
					  val[0]= 0;
					  val[1]= 0;
					  updateTemp();
				  }
			  }
		      sprintf(msg2,"\nCount %d \r\n",countStep);
		      HAL_UART_Transmit(&huart2,(uint8_t*)msg2,strlen(msg2),HAL_MAX_DELAY);
	}
}


void updateTemp(){

		temp[countStep][0]= val[0];
//		sprintf(msg2,"\nTEMP_0 %d \r\n",temp[countStep][0]);
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg2,strlen(msg2),HAL_MAX_DELAY);

		temp[countStep][1]= val[1];
//		sprintf(msg2,"\nTEMP_1 %d \r\n",temp[countStep][1]);
//		HAL_UART_Transmit(&huart2,(uint8_t*)msg2,strlen(msg2),HAL_MAX_DELAY);

		countStep++;

		if(countStep == 4){
			checkTemp();
		}
}

void checkTemp(){
	uint8_t k=0;
	uint8_t h=0;
	uint8_t breaked=0;


	for(k=0; k<4; k++) {
		for(h=0; h<2; h++) {
			if(temp[k][h] != fine_walk[k][h]){
				//HAL_UART_Transmit(&huart2,(uint8_t*)"WARNING",strlen("WARNING"),HAL_MAX_DELAY);
				color_[0]=1;
				color_[1]=0;
				k=4;
				breaked = 1;
				break;
			}
		}
	}
	if(!breaked){
		//HAL_UART_Transmit(&huart2,(uint8_t*)"FUNZIONA",strlen("FUNZIONA"),HAL_MAX_DELAY);
		color_[0]=1;
		color_[1]=1;
	}

	for(k=0; k<4; k++) {
		for(h=0; h<2; h++) {
			temp[k][h] = 0;
		}
	}

	countStep = 0;
}

uint8_t checkMovement(float* acc){

	if(fabs(acc[0]) > 0.5 || fabs(acc[1]) > 0.5)
		return 1;

	return 0;
}
