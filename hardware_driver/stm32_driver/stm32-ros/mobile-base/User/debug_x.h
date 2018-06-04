#ifndef _debug_X_H
#define	_debug_X_H
#include "stm32f10x.h"
/*************************************************************
@StevenShi 
**************************************************************/

#define USARTm										USART1
#define USARTm_GPIO								GPIOA
#define USARTm_CLK								RCC_APB2Periph_USART1
#define USARTm_GPIO_CLK						RCC_APB2Periph_GPIOA
#define USARTm_RxPin							GPIO_Pin_10
#define USARTm_TxPin							GPIO_Pin_9
#define USARTm_IRQn								USART1_IRQn
#define USARTm_DR_Base						(uint32_t)(&USART1->DR)
//#define USARTm_DR_Base						USART1_BASE+0x04
#define USARTm_Tx_DMA_Channe			DMA1_Channel4
#define USARTm_Tx_DMA_FLAG				DMA1_FLAG_GL4 //DMA1_FLAG_TC4|DMA1_FLAG_TE4
#define UASRTm_TX_DMA_IRQ					DMA1_Channel4_IRQn

#define USARTm_Rx_DMA_Channe			DMA1_Channel5
#define USARTm_Rx_DMA_FLAG				DMA1_FLAG_GL5 //DMA1_FLAG_TC5 |DMA1_FLAG_TE5
#define UASRTm_RX_DMA_IRQ					DMA1_Channel5_IRQn 



#define USARTm_Tx_BUFFER_SIZE	256 //���ͻ��泤��
#define USARTm_Rx_BUFFER_SIZE	32  //���ջ��泤��

//����-����printf����Ҫ���͵�����
typedef struct { 	
	uint8_t	USARTm_Tx_Buffer[USARTm_Tx_BUFFER_SIZE]; //���пռ�
	__IO	uint32_t	USARTm_Tx_PTR_HEAD; //����ͷָ��
	__IO	uint32_t	USARTm_Tx_PTR_TAIL; //����βָ��
	__IO	uint32_t	USARTm_Tx_COUNTER; 	//��ǰ���г���
	__IO	uint8_t		USARTm_Tx_BUFFER_FULL;  //��������־
}USARTm_Tx_Buffer_TypeDef;  



//*****************�ڲ�����************************
void debug_x_usart_config(void);
void debug_x_usart_gpio_config(void);
void debug_x_uasrt_nvic_config(void);
void debug_x_usart_dma_rx_config(void);//DMA��������
void debug_x_usart_dma_tx_config(uint32_t memoryBaseAddr, uint8_t sendBufferSize);//DMA�������� 
void debug_x_usart_dma_read(void);//DMA ���������� �ڴ��ڵĿ����ж��е��ã�������д�ú������ŵ������ط�����
void debug_x_usart_rcc_config(void);

//****************�ⲿ���ú���************************
void debug_x_usart_Init(void);//��ʼ��
void debug_x_usart_dma_start_tx(uint32_t memoryBaseAddr,uint32_t SendBufferSize);//ʹ��DMA����ʱ�������ͻ����׵�ַ�ͳ��ȼ���
int debug_x_usart_dma_ctl(void);//�ú�������printf��ʵ�֣��ڴ��������ӡ������Ϣ
uint32_t debug_x_usart_In_Queue(uint8_t ch);//�����ݷ������
uint32_t debug_x_usart_Out_Queue(uint8_t *ch);//�����ݶ�������
#endif
//@StevenShi 
/**************************************************************************************************/
