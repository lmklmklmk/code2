/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-�Ե� STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "./485/bsp_485.h"
#include "bsp_GPRS.h"
#include "bsp_systick.h"

//1.��ʼ��485�õĴ��ں� �շ���������
//2.��д�жϷ�����

uint8_t sendData = 'A' ;


//����RS485���洮������

extern volatile uint16_t RS485_p ;
extern uint8_t RS485_buff[];

extern volatile uint32_t RS485_ms_timer ;// �δ��������
extern unsigned char RS485_Flag ;

//����GPRS���洮������

extern volatile uint16_t GPRS_p ;
extern uint8_t GPRS_buff[];

extern volatile uint32_t GPRS_ms_timer ;// �δ��������
extern unsigned char GPRS_Flag ;





/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
  USART_Config();
	
	RS485_USART_Config();
	
	GPRS_USART_Config();
	
	SysTick_Init();
 
	
	/* ����һ���ַ��� */
	Usart_SendString( DEBUG_USARTx,"����һ�������жϽ��ջ���ʵ��\n");
	printf("��ӭʹ�ñ���STM32������\n\n\n\n");
	
	while(1)
	{
		       if(RS485_Flag == 1) {

      		
		unsigned int k=0;
	   for(; k<RS485_p; k++) {
	    /* ����һ���ֽ����ݵ�USART */
	   RS485_Usart_SendByte(GPRS_USARTx,RS485_buff[k]);	
     
     }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(RS485_USARTx,USART_FLAG_TC)==RESET);
	 RS485_p=0;
     RS485_Flag = 0; //�����־λ
     RS485_ms_timer = 0;
       }
   
        
               if(GPRS_Flag == 1) {

      		
		unsigned int k=0;
	   for(; k<GPRS_p; k++) {
	    /* ����һ���ֽ����ݵ�USART */
	   GPRS_Usart_SendByte(RS485_USARTx,GPRS_buff[k]);	
     
     }
	/* �ȴ�������� */
	while(USART_GetFlagStatus(GPRS_USARTx,USART_FLAG_TC)==RESET);
	 GPRS_p=0;
     GPRS_Flag = 0; //�����־λ
     GPRS_ms_timer = 0;
       }
		
		
    }
}
///*********************************************END OF FILE**********************/
