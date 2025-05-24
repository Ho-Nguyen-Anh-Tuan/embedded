#include "stm32f10x.h"      // Device header
#include "stm32f10x_rcc.h"  // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h" // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"  // Device:StdPeriph Drivers:TIM
#include "stm32f10x_exti.h" // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_usart.h"// Device:StdPeriph Drivers:USART

#define  TX  GPIO_Pin_9
#define  RX  GPIO_Pin_10

#define  UART_GPIO  GPIOA

volatile uint32_t TIM2_Counter = 0;
volatile uint8_t button_count = 0;
volatile uint8_t UART_data;

uint8_t count1, count2, count3;

typedef enum {NOT_EXPIRED = 0, EXPIRED = !NOT_EXPIRED} TimerState;

typedef struct
{
	uint32_t start_tick;	// thời điểm bắt đầu
	uint32_t delay_ms;		// thời gian delay (ms)
	TimerState state;		// trạng thái timer: còn chạy hoặc hết thời gian
} SoftTimer;

void RCC_Config();
void GPIO_Config();
void TIM_Config();
void UART_Config();
void EXTI_Config();
void NVIC_Config();
void delay_ms(uint32_t time);
void EXTI0_IRQHandler();
void TIM2_IRQHandler();
void USART1_IRQHandler();
void start_timer(SoftTimer *timer, uint32_t delayTime);
TimerState Is_Timer_Expired(SoftTimer timer);

int main()
{
	RCC_Config();
	GPIO_Config();
	TIM_Config();
	UART_Config();
	EXTI_Config();
	NVIC_Config();

	// khai báo soft timer
	SoftTimer timer1, timer2, timer3;

	// khởi tạo timer
	start_timer(&timer1, 500);
	start_timer(&timer2, 1000);
	start_timer(&timer3, 2000);

	while (1)
	{
		if (Is_Timer_Expired(timer1))
		{
			count1++;
			GPIO_WriteBit(GPIOA, GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1));
			start_timer(&timer1, 500); // khởi động lại
		}

		if (Is_Timer_Expired(timer2))
		{
			count2++;
			GPIO_WriteBit(GPIOA, GPIO_Pin_2, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2));
			start_timer(&timer2, 1000); // khởi động lại
		}

		if (Is_Timer_Expired(timer3))
		{
			count3++;
			GPIO_WriteBit(GPIOA, GPIO_Pin_3, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3));
			start_timer(&timer3, 2000); // khởi động lại
		}
	}
}

/*************************** System config *****************************/
void RCC_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC |
							RCC_APB2Periph_USART1, ENABLE);
}

void GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	// EXTI
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	// Toggle GPIOA Pin
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// LED 13
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// TX
	GPIO_InitStruct.GPIO_Pin = TX;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(UART_GPIO, &GPIO_InitStruct);

	// RX
	GPIO_InitStruct.GPIO_Pin = RX;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(UART_GPIO, &GPIO_InitStruct);
}

void TIM_Config()
{
	TIM_TimeBaseInitTypeDef TIM_InitStruct;

	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStruct.TIM_Prescaler = 7200 - 1;	// 0.1s tick++
	TIM_InitStruct.TIM_Period = 10 - 1;			// 1ms interrupt

	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

}

void UART_Config()
{
	USART_InitTypeDef UART_InitStruct;

	UART_InitStruct.USART_BaudRate = 9600;
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;
	UART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	UART_InitStruct.USART_Parity = USART_Parity_No;
	UART_InitStruct.USART_StopBits = USART_StopBits_1;
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART1, &UART_InitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void EXTI_Config()
{
	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStruct);
}

void NVIC_Config()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStruct;

	// EXTI0
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);

	// TIM2
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);

	// UART1
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);
}
/************************** Delay *******************************/

void delay_ms(uint32_t time)
{
	uint32_t start_time = TIM2_Counter;
	while ((TIM2_Counter - start_time) < time)
	{
		// wait
	}
}

/**
  * @brief khởi tạo soft timer delay (ms)
  * @param  timer: địa chỉ timer cần khởi tạo
  * @param  delayTime: thời gian muốn delay (ms)
  * @retval None
  */
void start_timer(SoftTimer *timer, uint32_t delayTime)
{
	timer->start_tick = TIM2_Counter;	// lưu thời điểm bắt đầu đếm
	timer->delay_ms = delayTime; 		// gán thời gian delay
	timer->state = NOT_EXPIRED;			// đặt trạng thái ban đầu
}

/**
  * @brief kiểm tra timer hết thời gian chưa
  * @param  timer: timer cần kiểm tra
  * @retval timer state (EXPIRED or NOT_EXPIRED) 
  */
TimerState Is_Timer_Expired(SoftTimer timer)
{	
	/*thời điểm hiện tại - thời điểm bắt đầu timer >= thời gian delay */
	if ((TIM2_Counter - timer.start_tick) >= timer.delay_ms)
	{
		return EXPIRED;	// timer đếm xong
	}	
	
	return NOT_EXPIRED;
}
/**************************IRQHandler ********************************/

void EXTI0_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		button_count++;
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM2_Counter++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void USART1_IRQHandler()
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		UART_data = USART_ReceiveData(USART1);
		
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, !GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13));

		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, UART_data);

		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

 
