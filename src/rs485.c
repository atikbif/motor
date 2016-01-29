#include "rs485.h"

#include "stm32f0xx_conf.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "crc.h"

#define USART1_TDR_Address    0x40013828

#define BUF_SIZE 128

unsigned char canal_tx_buf[BUF_SIZE];
volatile unsigned char canal_rx_buf[BUF_SIZE];
volatile unsigned int canal_rx_cnt=0;

uint8_t TxBuffer1[] = "Communication between USART1-USART2 using DMA2";

static void init_uart(void);
inline unsigned long get_pc_tmr(void) {return TIM2->CNT;}
void write_canal(unsigned short count);
void USART1_IRQHandler(void);
void DMA1_Channel2_3_IRQHandler(void);

void RS485Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastExecutionTime, ( portTickType ) 2000 / portTICK_RATE_MS);
    init_uart();
    while(1)
    { // backgroung loop
        if((canal_rx_cnt)&&(get_pc_tmr()>100))
		{
		    if((canal_rx_buf[0]==0x01)&&(GetCRC16((unsigned char*)canal_rx_buf,canal_rx_cnt)==0))
			{
				// разбор команд
				switch(canal_rx_buf[1])
				{
				    case 0x00:break;
				}
			}
		}
        vTaskDelayUntil( &xLastExecutionTime, RS485_DELAY);
        //GPIOA->ODR ^= GPIO_Pin_11;
    }
}


void init_uart(void)
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE );
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    TIM_DeInit( TIM2 );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
	TIM_TimeBaseStructure.TIM_Prescaler = 0xFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ( unsigned short ) 0xFFFF;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
	TIM_ARRPreloadConfig( TIM2, ENABLE );
	TIM_Cmd( TIM2, ENABLE );


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0); // USART1_TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0); // USART1_RX

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_7b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx| USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE , ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1, ENABLE);

	canal_tx_buf[0] = 'h';
    canal_tx_buf[1] = 'e';
    canal_tx_buf[2] = 'l';
    canal_tx_buf[3] = 'l';
    canal_tx_buf[4] = 'o';
    canal_tx_buf[5] = 0x0D;
    canal_tx_buf[6] = 0x0A;
    write_canal(7);
}

void write_canal(unsigned short count)
{
	DMA_InitTypeDef DMA_InitStructure;
	if(count>BUF_SIZE) return;
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_TDR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)canal_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);
}

void USART1_IRQHandler(void)
{

    if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
   {
     USART_ClearITPendingBit(USART1, USART_IT_TC);
     if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=RESET)
     {
        GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
     }
   }
   if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
   {
	 canal_rx_buf[canal_rx_cnt]=USART_ReceiveData(USART1);
	 canal_rx_cnt++;
     if(canal_rx_cnt>=BUF_SIZE) canal_rx_cnt=0;
     USART_ClearITPendingBit(USART1, USART_IT_RXNE);
     TIM2->CNT=0;
    }
}

void DMA1_Channel2_3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC2) != RESET)
	{
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_ClearITPendingBit(DMA1_IT_GL2);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);
	}
}
