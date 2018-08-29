
#include "stm32f4xx.h"
#include "freertosconfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void GPIO_Init (void);

void vTaskLed1 (void *argument);
void vTaskLed2 (void *argument);

int main(void){		
	
	GPIO_Init();
	
	xTaskCreate(vTaskLed1, "LED1", 32, NULL, 4, NULL);
	BaseType_t p = xTaskCreate(vTaskLed2, "LED2", 32, NULL, 4, NULL);
	
	vTaskStartScheduler();
	//portDBG_TRACE("error");

	while(1)
	{
				GPIOD->BSRRL |= GPIO_BSRR_BS_13;
	}
	
}

/*******************************************************************************************************/

void GPIO_Init (void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOD->MODER |= GPIO_MODER_MODER15_0;
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13;
	GPIOD->MODER |= GPIO_MODER_MODER13_0;
	
	GPIOD->BSRRL |= GPIO_BSRR_BS_15;
}

void vTaskLed1 (void *argument){
	
	while(1)
	{		
		GPIOD->BSRRL |= GPIO_BSRR_BS_15;
		vTaskDelay(1000);
		//GPIOD->BSRRL |= GPIO_BSRR_BR_15;
		//vTaskDelay(1000);	
	}
	
}

void vTaskLed2 (void *argument){
	
	while(1)
	{
		
		GPIOD->BSRRL |= GPIO_BSRR_BS_13;
		vTaskDelay(100);
		//GPIOD->BSRRL |= GPIO_BSRR_BR_13;
		//vTaskDelay(10);
		
	}
	
}







