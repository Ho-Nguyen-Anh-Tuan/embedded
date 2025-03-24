#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM


#define SPI_CS_Pin		GPIO_Pin_4
#define SPI_SCK_Pin		GPIO_Pin_5
#define SPI_MISO_Pin	GPIO_Pin_6
#define SPI_MOSI_Pin	GPIO_Pin_7
#define SPI_GPIO 			GPIOA
#define SPI_RCC 			RCC_APB2Periph_GPIOA


void RCC_Config(){
	RCC_APB2PeriphClockCmd(SPI_RCC, ENABLE);
}

void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin | SPI_CS_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
}

void TIM_Config(){
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_Prescaler = 7200 - 1;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period = 0xFFFF;
	
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
}

//uint8_t SPI_Slave_Receive(){
//	uint8_t dataReceive = 0x00;
//	uint8_t temp = 0x00;
//	
//	// cho cs = 0
//	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
//	
//	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));

//	for(int i = 0; i < 8; i++){

//		// khi sck = 1 tien hanh truyen
//		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
//			
//			// doc lien tuc gia tri sck do pausing ~80%
//			while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
//				temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
//			}
//			
//			dataReceive <<= 1;
//			dataReceive |= temp;
//		}
//		
//		// doi sck = 1 roi tien hanh truyen bit tiep theo
//		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){}
//	}
//	
//	// truyen xong 8 bit thi doi cs = 1
//	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin)){}
//	
// 	return dataReceive;
//}

uint8_t SPI_Slave_Reveive(){
	uint8_t dataReceive = 0x00;
	uint8_t temp = 0x00;
	
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	
	for (int i = 0; i < 8; i++){
		
		// wait sck = 1
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
		
		while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin)){
			temp = GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin);
		}
		
		dataReceive = (dataReceive << 1) | temp;
		
		// doi sck = 1 roi truyen tiep
		// while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	}
	
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	
	return dataReceive;
}

uint8_t DataReceive;
int main(){
	RCC_Config();
	GPIO_Config();
	
	while(1){
		DataReceive = SPI_Slave_Reveive();
	}
}
