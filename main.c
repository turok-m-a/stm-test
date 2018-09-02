#include "stm32f4xx.h"
#include "freertosconfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "math.h"
#define SENSITIVITY 2 //2g
void GPIO_Init (void);
 void reverse(char s[]){
     int i, j;
     char c;
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }
const struct maxAccelerations {	// "callibration" (max/min readings at +/- free fall acceeration)
	 short int posMaxVals[3];
	 short int negMinVals[3];
 } 
//maxAccelerations_ = {{19558,16666,18102},{-17928,-17388,-17369}}; //measured while rotating
maxAccelerations_ = {{16600,16600,16650},{-17400,-16900,-16400}}; //measured at +/- 90 degree
//maxAccelerations_ = {{16384,16384,16384},{-16384,-16384,-16384}}; //ideal
 short int readingsToDegrees(short int reading,char axis,char negative){
	 long int deg = reading;
	 deg*=90;
	 if (negative){
		 deg/=maxAccelerations_.negMinVals[axis]*-1;
	 } else {
		 deg/=maxAccelerations_.posMaxVals[axis];
	 }
	 return (short int)deg;
 }
 void itoa(int n, char s[]){
     int i, sign;
     if ((sign = n) < 0)
         n = -n; 
     i = 0;
     do {       
     s[i++] = n % 10 + '0';   
     } while ((n /= 10) > 0);   
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
 }
void vTaskLed1 (void *argument);
void vTaskLed2 (void *argument);
xQueueHandle queue;
void uartInit(){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->AFR[0] |= 0x7000000;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	//USART1->BRR = 0xb64;// 9600
	USART1->BRR = 0x2d3; //38400
	//USART1->BRR = 0x222E; // 9600 for f(APB2) == 84 MHz
	USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_UE;
}
void spiInit(){
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOA->MODER |= GPIO_MODER_MODER5_1;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	GPIOE->MODER |= GPIO_MODER_MODER3_0;
	GPIOA->AFR[0] |= 0x55500000; //AF5 for 5,6,7
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5; //SCK
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; //MISO
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7; //MOSI
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3; //CS
	GPIOE->ODR |= 8; //0b1000 CS HIGH
	SPI1->CR1 |= SPI_CR1_SSM;
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= 0x38; // lowest clk
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_CPOL; 
	SPI1->CR1 |= SPI_CR1_CPHA;
	SPI1->CR1 |= SPI_CR1_SPE;
	for (int i=0; i<100000;i++); //pause before first transaction after enabling SPI is necessary
	GPIOE->ODR &= ~8; //CS LOW
	SPI1->DR = 0x008F;
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
	char dummy = SPI1->DR;
	SPI1->DR = dummy;
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
	unsigned char id = SPI1->DR;
	if (id == 0x3F ){
		GPIOD->BSRRL |= GPIO_BSRR_BS_15;//successful communication with accelerometer
	}
	GPIOE->ODR |= 8; //CS HIGH
	for (int i=0; i<10;i++); //small pause before next transaction
	GPIOE->ODR &= ~8; //CS LOW
	SPI1->DR = 0x0020; //CR4, enable accelerometer
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
	dummy = SPI1->DR;
	SPI1->DR = 0x27;
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
	dummy = SPI1->DR;
	GPIOE->ODR |= 8; //CS HIGH
	
	//for (int i=0; i<10;i++);
	//use for custom sensitivity
//		GPIOE->ODR &= ~8; //CS LOW 
//	SPI1->DR = 0x0024;
//	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
//	dummy = SPI1->DR;
//	SPI1->DR = 0x4;
//	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
//	dummy = SPI1->DR;
//	GPIOE->ODR |= 8; //CS HIGH
	
}
void accelGetValues(short int * val){
	
	
	for (int i = 0;i<6;i++){
	GPIOE->ODR &= ~8; //CS LOW
	for (int i=0; i<50000;i++);
	SPI1->DR = 0x80 | (0x28+i);
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
	char dummy = SPI1->DR;
	SPI1->DR = dummy;
	while(!(SPI1->SR & SPI_SR_RXNE) || (SPI1->SR & SPI_SR_BSY));//wait for byte receive
  *(((unsigned char *)val) + i) = SPI1->DR;
  GPIOE->ODR |= 8;	
	}
	
}
void sendStringToUart(char * s,char axis){
	USART1->DR = axis;
	while(!(USART1->CR1 | USART_SR_TXE) || !(USART1->CR1 | USART_SR_TC));
	for (int i=0; i<5000;i++); // cr1 flags check doesnt help
	USART1->DR = ':';
	while(!(USART1->CR1 | USART_SR_TXE) || !(USART1->CR1 | USART_SR_TC));
	for (int i=0; i<5000;i++);
	int i=0;
	while(s[i]!=0){
		USART1->DR = s[i];
		while(!(USART1->CR1 | USART_SR_TXE));
		for (int i=0; i<5000;i++);
		i++;
	}
	USART1->DR = ' ';
	while(!(USART1->CR1 | USART_SR_TC));
	for (int i=0; i<5000;i++);
}
void sendValsToUart(short int * val){
	for (int i=0; i<50000;i++);
	for (int i = 0;i<3;i++){
	char negative = 0;
	if (val[i] < 0){
		negative = 1;
	}
	short int v = readingsToDegrees(val[i],i,negative);
		
	// "calibration"
//		short int v= val[i];
//	if (v>0 && v>maxAccelerations_.posMaxVals[i]){
//		maxAccelerations_.posMaxVals[i] = v;
//	} 
//	if(v<0 && v<maxAccelerations_.negMinVals[i]) {
//		maxAccelerations_.negMinVals[i] = v;
//	}
	char s[10];
	itoa(v,s);
	sendStringToUart(s,'X'+i);
	}	
	USART1->DR = '\n';
	while(!(USART1->CR1 | USART_SR_TC));
	for (int i=0; i<5000;i++);
	USART1->DR = '\r';
	while(!(USART1->CR1 | USART_SR_TC));
	for (int i=0; i<5000;i++);
}
int main(void){		
	
	GPIO_Init();
	uartInit();
	spiInit();
	xTaskCreate(vTaskLed1, "LED1", 32, NULL, 3, NULL);
	BaseType_t p = xTaskCreate(vTaskLed2, "LED2", 60, NULL, 3, NULL);
	/*SysTick->CTRL = 0;
	SysTick->LOAD = 168000;
	SysTick->VAL = 1;
	SysTick->CTRL |= 7;*/
	
	vTaskStartScheduler();
	//portDBG_TRACE("error");
	
	while(1)
	{
				GPIOD->BSRRL |= GPIO_BSRR_BS_13; //scheduler error
	}
	
}

/*******************************************************************************************************/

void GPIO_Init (void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOD->MODER |= GPIO_MODER_MODER15_0;
		GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13;
	GPIOD->MODER |= GPIO_MODER_MODER13_0;
	
	//GPIOD->BSRRL |= GPIO_BSRR_BS_15;
}

void vTaskLed1 (void *argument){
	queue = xQueueCreate(5,sizeof(short int)* 3);
	while(1)
	{		
		short int vals[3];
		accelGetValues(vals);
		xQueueSendToBack(queue,vals,0);

	}
	
}

void vTaskLed2 (void *argument){
	
	while(1)
	{
		short int vals[3];
		if (xQueueReceive(queue,vals,0)== pdPASS){
		sendValsToUart(vals);
		} 
	
		
	}
	
}







