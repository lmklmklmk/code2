/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "./485/bsp_485.h"
 
#include "bsp_GPRS.h"
#include "bsp_systick.h"



//串口RS485缓存串口数据
 #define RS485_BUFF_SIZE 256//数组大小
 volatile uint16_t RS485_p = 0;//字节数量
 uint8_t RS485_buff[RS485_BUFF_SIZE];//数组
 
 volatile uint32_t RS485_baudrate =9600;// 485波特率
     
 volatile  uint8_t RS485_Start = 0; //开始接收标识
 volatile uint32_t RS485_ms_timer = 0;// 滴答计数变量
  uint8_t RS485_Flag = 0;//标志位

//串口GPRS缓存串口数据
 #define GPRS_BUFF_SIZE 256//数组大小
 volatile uint16_t GPRS_p = 0;//字节数量
 uint8_t GPRS_buff[GPRS_BUFF_SIZE];//数组
 
 volatile uint32_t GPRS_baudrate =115200;// GPRS波特率

 volatile  uint8_t GPRS_Start = 0; //开始接收标识
 volatile uint32_t GPRS_ms_timer = 0;// 滴答计数变量
  uint8_t GPRS_Flag = 0;//标志位








/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */


// 滴答中断服务函数
void SysTick_Handler(void)
{
    
    // RS485滴答时间计数函数
	 if(RS485_Start == 1)
	 {
			if(++RS485_ms_timer >= 16000/RS485_baudrate) //2*8/波特率*1000(2是倍数,8是b换B,1000是换算成毫秒)
		{
			
			RS485_ms_timer = 0;
			RS485_Flag = 1; //标志位
        }
                       
    }
     
    
    // GPRS滴答时间计数函数
	 if(GPRS_Start == 1)
	 {
			if(++GPRS_ms_timer >=16000/GPRS_baudrate) //2*8/波特率*1000(2是倍数,8是b换B,1000是换算成毫秒)
		{
			
			GPRS_ms_timer = 0;
			GPRS_Flag = 1; //标志位
        }
            	
    }

		}
	


// 串口DEBUG中断服务函数
void DEBUG_USART_IRQHandler(void)
{
  uint8_t ucTemp;
	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
	{		
		
		ucTemp = USART_ReceiveData(DEBUG_USARTx);
    USART_SendData(DEBUG_USARTx,ucTemp);   

		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);		
		
	}	 
}


// 串口RS485中断服务函数
void RS485_USART_IRQHandler(void)
{

	uint8_t ucTemp1;

 
  if (RS485_p<RS485_BUFF_SIZE) 
{
		
	if(USART_GetITStatus(RS485_USARTx,USART_IT_RXNE)!=RESET)
	{		
		RS485_Start = 1;
		ucTemp1 = USART_ReceiveData(RS485_USARTx);
		RS485_buff[RS485_p] = ucTemp1;
        RS485_p++;
		RS485_ms_timer=0;
                       
		}

	}
	
			
}


// 串口GPRS中断服务函数
void GPRS_USART_IRQHandler(void)
{

	uint8_t ucTemp2;

 
  if (GPRS_p<GPRS_BUFF_SIZE) 
{
		
	if(USART_GetITStatus(GPRS_USARTx,USART_IT_RXNE)!=RESET)
	{		
		GPRS_Start = 1;
		ucTemp2 = USART_ReceiveData(GPRS_USARTx);
		GPRS_buff[GPRS_p] = ucTemp2;
        GPRS_p++;
		GPRS_ms_timer=0;
                       
		}

	}
	
			
}





/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
