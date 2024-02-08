#include "main.h"

#define TRIG_PIN GPIO_PIN_1
#define TRIG_PORT GPIOA

uint8_t HCSR04_Distance(uint32_t Diff, uint8_t Distance){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_Delay(1);  // wait for 10 ms
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	Distance = Diff * .34/2;
	return Distance;
};

void HCSR04_Start(TIM_HandleTypeDef* const timer_handle){
	__HAL_TIM_ENABLE_IT(timer_handle, TIM_IT_CC1);
};

void HCSR04_Stop(TIM_HandleTypeDef* const timer_handle){
	__HAL_TIM_DISABLE_IT(timer_handle, TIM_IT_CC1);
};
