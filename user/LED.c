#include "stm32f10x.h"                  // Device header


void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);
	
}


void LED_Set( uint8_t LED_Choose, uint8_t LED_On)//选择哪一个LED，以及它的状态
{
	switch (LED_Choose)
	{
		case 1:
			switch (LED_On)
			{
				case 1:{GPIO_ResetBits( GPIOA, GPIO_Pin_0);break;}
				case 0:{GPIO_SetBits( GPIOA, GPIO_Pin_0);break;}
				break;
			}
		case 2:
			switch (LED_On)
			{
				case 1:{GPIO_ResetBits( GPIOA, GPIO_Pin_1);break;}
				case 0:{GPIO_SetBits( GPIOA, GPIO_Pin_1);break;}
				break;
			}
		case 3:
			switch (LED_On)
			{
				case 1:{GPIO_ResetBits( GPIOA, GPIO_Pin_2);break;}
				case 0:{GPIO_SetBits( GPIOA, GPIO_Pin_2);break;}
				break;
			}
	}
}
