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
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
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

//void clock(){
//	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);
//	delay_ms(4);
//	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
//	delay_ms(4);
//}

#define clock_1	\
					GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_SET);	\
					delay_ms(4);	\

				
#define clock_0	\
					GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);	\
					delay_ms(4);	

void SPI_Init(){
	GPIO_WriteBit(SPI_GPIO, SPI_SCK_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_CS_Pin, Bit_SET);
	GPIO_WriteBit(SPI_GPIO, SPI_MOSI_Pin, Bit_RESET);
	GPIO_WriteBit(SPI_GPIO, SPI_MISO_Pin, Bit_RESET);
}

// ham chuyen nhan 8 bit master
uint8_t SPI_Master_TransRe(uint8_t u8Data){	// 0101 1110
	uint8_t u8Mask = 0x80;			// 1000 0000
	uint8_t DataReceive = 0x00;
	
	GPIO_ResetBits(SPI_GPIO, SPI_CS_Pin);
	delay_ms(1);
	
	for(int i = 0; i < 8; i++){
		
		// chuan bi du lieu o MOSI
		if(u8Mask & u8Data){
			GPIO_SetBits(SPI_GPIO, SPI_MOSI_Pin);
		} else {
			GPIO_ResetBits(SPI_GPIO, SPI_MOSI_Pin);
		}
		delay_ms(1);
		
		clock_1;
		
		if(GPIO_ReadInputDataBit(SPI_GPIO, SPI_MISO_Pin)){
			DataReceive |= u8Mask;
		}
		
		u8Mask >>= 1;
		
		clock_0;
			
	}
	
	GPIO_SetBits(SPI_GPIO, SPI_CS_Pin);
	delay_ms(1);
	
	return DataReceive;
}


uint8_t DataTrans[] = {1, 2, 3, 4, 5, 6, 7};
uint8_t RxBuffer[7];

int main(){
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	SPI_Init();
	
	while(1){
		
		for(int i = 0; i < 7; i++){
			RxBuffer[i] = SPI_Master_TransRe(DataTrans[i]);
			delay_ms(1000);
		}
		
		for(int i = 0; i < 7; i++){
			RxBuffer[i] = 0;
		}
		
	}
}
