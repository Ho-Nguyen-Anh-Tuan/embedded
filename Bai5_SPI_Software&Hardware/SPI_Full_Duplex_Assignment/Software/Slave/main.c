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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
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
	TIM_InitStructure.TIM_Prescaler = 7200 - 1;					// 0.1ms count++
	TIM_InitStructure.TIM_Period = 0xFFFF;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	TIM_Cmd(TIM2, ENABLE);
}

void delay_ms(uint32_t time){
	TIM_SetCounter(TIM2, 0);
	while(TIM_GetCounter(TIM2) < time * 10){}
}

//// ham chuyen nhan 8 bit slave
//uint8_t SPI_Slave_TransRe(uint8_t u8Data){	// 0000 0111
//	uint8_t dataReceive = 0x00;	// 0000 0000
//	uint8_t u8Mask = 0x80;			// 1000 0000
//	
//	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
//	
//	for(int i =0; i < 8; i++){
//		
//		if(u8Mask & u8Data){
//			GPIO_SetBits(SPI_GPIO, SPI_MISO_Pin);
//		} else {
//			GPIO_ResetBits(SPI_GPIO, SPI_MISO_Pin);
//		}
//		
//		// cho sck 
//		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
//		
//		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin)){
//			dataReceive |= u8Mask;
//		}
//		
//		u8Mask >>= 1;
//		
//		while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
//	}
//	
//	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
//	
//	return dataReceive;
//}

uint8_t SPI_Slave_TransRe(uint8_t u8Data){	// 0000 0111
	uint8_t dataReceive = 0x00;	// 0000 0000
	uint8_t u8Mask = 0x80;			// 1000 0000
	
	while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	
	for(int i =0; i < 8; i++){
		
		// chuan bi du lieu o MISO truoc xung SCK
		if(u8Data & u8Mask){
			GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_SET);
		} else {
			GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
		}
		
		// cho sck
		while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
		
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_MOSI_Pin)){
			dataReceive |= u8Mask;
		}
		
			// co the chuan bi du lieu o MISO truoc hoac sau xung clock
//		if(u8Data & u8Mask){
//			GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_SET);
//		} else {
//			GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
//		}
		
		u8Mask >>= 1;
		
		while(GPIO_ReadInputDataBit(SPI_GPIO, SPI_SCK_Pin));
	}
	
	while(!GPIO_ReadInputDataBit(SPI_GPIO, SPI_CS_Pin));
	
	return dataReceive;
}


uint8_t DataTrans[] = {11, 12, 13, 14, 15, 16, 17};
uint8_t RxBuffer[7];

int main(){
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	
	while(1){
		for(int i = 0; i < 7; i++){
			RxBuffer[i] = SPI_Slave_TransRe(DataTrans[i]);
		}
		
		delay_ms(500);
		for(int i = 0; i < 7; i++){
			RxBuffer[i] = 0;
		}
		
	}
}
