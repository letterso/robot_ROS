
#include "debug_x.h"
#include "stm32f10x.h"
#include "string.h"
#include "stdio.h"
/*****************************************************
@StevenShi
STM32 USART1 DMA������������debug������
�ض���printf������DMA
����Ҫ��ӡ��Ϣ�ĵط�ֱ�ӵ���printf��������
û���õ����չ��ܣ����Թرոù���
*****************************************************/
USARTm_Tx_Buffer_TypeDef USARTm_Tx_Buf_Queue;//��������

//��������
uint32_t debug_x_usart_In_Queue(uint8_t data) 
{ 	
	if(USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER <USARTm_Tx_BUFFER_SIZE)//�ж϶����Ƿ���
		{
			if(USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD >= USARTm_Tx_BUFFER_SIZE) // �ж϶���ͷָ���Ƿ񳬳����п��
				USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD = 0; //������ͷָ��ָ��0 		
			USARTm_Tx_Buf_Queue.USARTm_Tx_Buffer[USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD] = data; //�����ݷ������		
			USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD++; 	//ͷָ���ƶ�	
			USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER++;  	//��ǰ���г��ȼ�һ
			return 0; 	
		} 		
	return 1;	 	 
}
//���ݳ���
uint32_t debug_x_usart_Out_Queue(uint8_t *data) 
{ 	
	uint32_t num;
	if(USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER > 0) 	//���в�Ϊ�ղ��ܽ��ж�������
		{ 		
			if(USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL >= USARTm_Tx_BUFFER_SIZE)//�ж϶���βָ���Ƿ񳬳����п�� 			
				USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL = 0;//��������0
			num = USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER;//��¼��ǰ�������ݳ���		
			*data = USARTm_Tx_Buf_Queue.USARTm_Tx_Buffer[USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL]; //�Ӷ����ж���һ����		
			USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL++; //βָ��ǰ��		
			USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER--; //���п�ȼ�һ 		
			return num; //���ض������ݳ���
		} 	
	else 	
		{ 		
			*data = 0xFF; //����Ϊ�գ����һ��ֵ		
			return 0;//���ض��г���0 	
		} 
}


uint8_t USARTmTxBuffer[USARTm_Tx_BUFFER_SIZE];//���ͻ���
uint8_t USARTmRxBuffer[USARTm_Rx_BUFFER_SIZE];//���ջ���
uint8_t USARTmRxBufferD[USARTm_Rx_BUFFER_SIZE];//���ջ���




//���ڲ�������-9600 8bit 1stop even parity
void debug_x_usart_config(void)
{
	// USARTm and USARTm configuration
	/* Configure USARTm */
	USART_InitTypeDef USARTm_InitStructure;
	USARTm_InitStructure.USART_BaudRate = 9600;
	USARTm_InitStructure.USART_WordLength = USART_WordLength_9b;//ϵͳbug USART_WordLength_8b�޷���������
	USARTm_InitStructure.USART_StopBits = USART_StopBits_1;
	USARTm_InitStructure.USART_Parity = USART_Parity_Even;
	USARTm_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTm_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTm, &USARTm_InitStructure);

	/* Enable USARTm Receive  interrupts */
	USART_ITConfig(USARTm, USART_IT_IDLE, ENABLE); //�������ڿ���IDLE�ж�
	/* Enable USARTm DMA TX request */
	USART_DMACmd(USARTm, USART_DMAReq_Tx, ENABLE);
	/* Enable USARTm DMA RX request */
	USART_DMACmd(USARTm, USART_DMAReq_Rx, ENABLE);
	/* Enable USARTm */
	USART_Cmd(USARTm, ENABLE);
	/* Enable USARTm DMA TX Channel */
	USART_DMACmd(USARTm, USART_DMAReq_Tx,ENABLE);// DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);

	/* Enable USARTm DMA RX Channel */
	USART_DMACmd(USARTm, USART_DMAReq_Rx,ENABLE);//DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);
	
}
//GPIO����
void debug_x_usart_gpio_config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USARTm Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//
	GPIO_InitStructure.GPIO_Pin = USARTm_RxPin;
	GPIO_Init(USARTm_GPIO, &GPIO_InitStructure);  

	/* Configure USARTm Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//������ʽ���������
	GPIO_InitStructure.GPIO_Pin = USARTm_TxPin;
	GPIO_Init(USARTm_GPIO, &GPIO_InitStructure);


}



//NVIC����
void debug_x_uasrt_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTm_TX_DMA_IRQ;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable the USARTm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//DMA-����ͨ������
void debug_x_usart_dma_tx_config(uint32_t memoryBaseAddr, uint8_t sendBufferSize)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* USARTm_Tx_DMA_Channel  */
	DMA_Cmd(USARTm_Tx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTm_Tx_DMA_Channe);//�ָ�ȱʡ����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTm_DR_Base;//���ڷ������ݼĴ���
	//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTmTxBuffer;//���ͻ����׵�ַ
	//DMA_InitStructure.DMA_PeripheralBaseAddr = peripheralBaseAddr;//���ڷ������ݼĴ���
	DMA_InitStructure.DMA_MemoryBaseAddr = memoryBaseAddr;//���ͻ����׵�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//��������ΪĿ��
	DMA_InitStructure.DMA_BufferSize = sendBufferSize;//��Ҫ���͵��ֽ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ 
	DMA_Init(USARTm_Tx_DMA_Channe, &DMA_InitStructure);//д������	
	DMA_ClearFlag(USARTm_Tx_DMA_FLAG);    //���DMA���б�־                          
	//DMA_Cmd(USARTm_Tx_DMA_Channe, ENABLE); 
	DMA_ITConfig(USARTm_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //����DMA����ͨ���ж�  
}
//DMA-����ͨ������
void debug_x_usart_dma_rx_config(void)	
{
	DMA_InitTypeDef DMA_InitStructure;

	/*USARTm_Rx_DMA_Channe*/
	DMA_Cmd(USARTm_Rx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTm_Rx_DMA_Channe);//�ָ�ȱʡ����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTm_DR_Base;//���ô��ڽ������ݼĴ���
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTmRxBuffer;//���ջ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//��������Ϊ����Դ
	DMA_InitStructure.DMA_BufferSize = USARTm_Tx_BUFFER_SIZE;//��Ҫ���յ��ֽ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ 
	DMA_Init(USARTm_Rx_DMA_Channe, &DMA_InitStructure);//д������	
	DMA_ClearFlag(USARTm_Rx_DMA_FLAG);    //���DMA���б�־                          
	DMA_Cmd(USARTm_Rx_DMA_Channe, ENABLE); //����DMA����ͨ��
	   
}
//DMA-��������ж�
void DMA1_Channel4_IRQHandler(void)//�ж�����������startup_stm32f10x.s�ļ���
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4))
	{
		DMA_ClearFlag(USARTm_Tx_DMA_FLAG);    //���DMA���б�־    
		DMA_Cmd(USARTm_Tx_DMA_Channe, DISABLE);  //�ر�DMA����ͨ��
	}
}


//��ʼDMA���� memoryBaseAddr-Ҫ���͵������׵�ַ��SendBufferSize-���ݳ���
void debug_x_usart_dma_start_tx(uint32_t memoryBaseAddr,uint32_t SendBufferSize)
{
	debug_x_usart_dma_tx_config(memoryBaseAddr,SendBufferSize);
	USARTm_Tx_DMA_Channe->CNDTR = (uint16_t)SendBufferSize; //���¸�ֵ ָ�����ͻ��泤��
	DMA_Cmd(USARTm_Tx_DMA_Channe, ENABLE);  //����DMA����      
}
//**************************************************************************

//���ڿ����ж�-һ֡��ɣ������ݶ���
void USART1_IRQHandler(void)//�ж�����������startup_stm32f10x.s��
{
    
	if(USART_GetITStatus(USARTm, USART_IT_IDLE) != RESET)  
		{
				debug_x_usart_dma_read();
				USART_ReceiveData( USARTm );
		}
}
//DMA����һ֡���ݣ���ʼ��һ֡�ĵȴ�����
void debug_x_usart_dma_read(void)
{
	uint8_t rxcounter;
	uint8_t i;
	DMA_Cmd(USARTm_Rx_DMA_Channe, DISABLE);    //�ر�DMA��ֹ����   
	DMA_ClearFlag( USARTm_Rx_DMA_FLAG );   //�����־λ       
	rxcounter= USARTm_Tx_BUFFER_SIZE - DMA_GetCurrDataCounter(USARTm_Rx_DMA_Channe);//��ȡ���յ����ֽ��� 
	USARTm_Rx_DMA_Channe->CNDTR = USARTm_Tx_BUFFER_SIZE; //���¸�ֵ����ֵ   
	memset(USARTmRxBufferD,0,sizeof(USARTmRxBufferD));
	for(i=0;i<rxcounter;i++){
		USARTmRxBufferD[i] = USARTmRxBuffer[i];//��ȡ���յ������ݣ���������RxBufferD��
	}

	//debug_x_get_data(USARTmRxBufferD);//�����յ������ݰ�
	for(i=0;i<rxcounter;i++)
		USARTmRxBuffer[i] = 0;//clear Rx buffer
	DMA_Cmd(USARTm_Rx_DMA_Channe, ENABLE);  //DMA���� �ȴ���һ֡����     
    
   
   
}
//RCC����
void debug_x_usart_rcc_config(void)
{
		/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd( USARTm_GPIO_CLK  |RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(USARTm_CLK,ENABLE);

}
//�ⲿ���ú��� ��ʼ��
void debug_x_usart_Init(void)
{
	
	debug_x_usart_rcc_config();
	debug_x_uasrt_nvic_config();
	debug_x_usart_gpio_config();

	debug_x_usart_dma_rx_config();//��������
	debug_x_usart_config();
}
//���DMA��printfʵ�� ÿ�ε���printf ����Ҫ����һ�θú���
//Ҳ���Խ��ú����ŵ���ʱ�ж���ȥִ�� ����sysTick
int debug_x_usart_dma_ctl(void) 
{ 	
	uint32_t num=0; 	uint8_t data; 	
	if(DMA_GetCurrDataCounter(USARTm_Tx_DMA_Channe)==0) 
	{
		DMA_Cmd(USARTm_Tx_DMA_Channe,DISABLE); 		
		while((debug_x_usart_Out_Queue(&data))!=0) 		
		{	 			
			USARTmTxBuffer[num]=data; 			
			num++; 			
			if(num==USARTm_Tx_BUFFER_SIZE) 			
				break; 		
		} 		
		if(num>0) 		
		{ 			
			debug_x_usart_dma_start_tx((uint32_t)USARTmTxBuffer,num);
		} 		
		return 0; 	
	} 	
	else 	
		return 1; 
}
/*********************************����printf���ض���*****************************************************************/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	//USART_SendData(USART1, (uint8_t) ch);
	/* Loop until the end of transmission */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	debug_x_usart_In_Queue((uint8_t)ch);//��Ҫ���͵����ݷ������
	return ch;
}
