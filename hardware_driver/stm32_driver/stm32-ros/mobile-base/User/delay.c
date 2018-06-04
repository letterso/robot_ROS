
#include "stm32f10x.h"
#include "delay.h"
#include "debug_x.h"
/**************************************
@StevenShi
ʹ��sysTick���жϲ�����ʱ����
�жϼ��1us
**************************************/
//__IO �ȼ��� volatile �ο�core_cm3.c
static __IO uint32_t usTicks;
static __IO uint16_t		msTicks=0;
// SysTick_Handler function will be called every 1 us
void SysTick_Handler()
{

	if (usTicks != 0)
	{
			usTicks--;
			msTicks++;
	}
	//��Ϊÿ��ʹ��printf����Ҫ����һ�θú��������Խ��ú����ŵ�sysTick�ж���ȥ
	//ÿ1ms�����printf�Ķ����Ƿ�Ϊ�գ���Ϊ�վͽ���һ��DMA����
	//��˱�����ÿ�ε���printf������һ�θú���
	if(msTicks >= 1000){//1ms
		msTicks = 0;
		debug_x_usart_dma_ctl(); 
	}
	
}
//����sysTick�жϼ��Ϊ1us
void delay_init(void)
{
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
	SysTick_Config(SystemCoreClock / 1000000);
	// Configure the SysTick timer to overflow every 1 ms
	//SysTick_Config(SystemCoreClock / 1000);
}

void delay_us(u32 us)
{
  // Reload us value
    usTicks = us;
    // Wait until usTick reach zero
    while (usTicks);
}


void delay_ms(u16 ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
	// Delay 1ms
	delay_us(1000);
	}
}
/*
void delay_ms(u16 ms)
{
	// Reload us value
	usTicks = ms;
	// Wait until usTick reach zero
	while (usTicks);
}
*/
