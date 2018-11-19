/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "./485/bsp_485.h"
#include "bsp_GPRS.h"
#include "bsp_systick.h"

//1.初始化485用的串口和 收发控制引脚
//2.编写中断服务函数

uint8_t sendData = 'A' ;


//串口RS485缓存串口数据

extern volatile uint16_t RS485_p ;
extern uint8_t RS485_buff[];

extern volatile uint32_t RS485_ms_timer ;// 滴答计数变量
extern unsigned char RS485_Flag ;

//串口GPRS缓存串口数据

extern volatile uint16_t GPRS_p ;
extern uint8_t GPRS_buff[];

extern volatile uint32_t GPRS_ms_timer ;// 滴答计数变量
extern unsigned char GPRS_Flag ;





/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
  /*初始化USART 配置模式为 115200 8-N-1，中断接收*/
  USART_Config();
	
	RS485_USART_Config();
	
	GPRS_USART_Config();
	
	SysTick_Init();
 
	
	/* 发送一个字符串 */
	Usart_SendString( DEBUG_USARTx,"这是一个串口中断接收回显实验\n");
	printf("欢迎使用秉火STM32开发板\n\n\n\n");
	
	while(1)
	{
		       if(RS485_Flag == 1) {

      		
		unsigned int k=0;
	   for(; k<RS485_p; k++) {
	    /* 发送一个字节数据到USART */
	   RS485_Usart_SendByte(GPRS_USARTx,RS485_buff[k]);	
     
     }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(RS485_USARTx,USART_FLAG_TC)==RESET);
	 RS485_p=0;
     RS485_Flag = 0; //清除标志位
     RS485_ms_timer = 0;
       }
   
        
               if(GPRS_Flag == 1) {

      		
		unsigned int k=0;
	   for(; k<GPRS_p; k++) {
	    /* 发送一个字节数据到USART */
	   GPRS_Usart_SendByte(RS485_USARTx,GPRS_buff[k]);	
     
     }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(GPRS_USARTx,USART_FLAG_TC)==RESET);
	 GPRS_p=0;
     GPRS_Flag = 0; //清除标志位
     GPRS_ms_timer = 0;
       }
		
		
    }
}
///*********************************************END OF FILE**********************/
