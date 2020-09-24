/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart5;//chuankouping
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
/* USART1 init function */

void MX_USART5_UART_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;//460800
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);//2
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);	
    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}
/* USER CODE END 1 */

void transmit_update(void)//发送
{
	uint8_t Buff[5];
	Buff[0]=0x50;
	Buff[1]=0x4F;
	Buff[2]=0x53;
	Buff[3]=0x49;
	Buff[4]=0x11;
//	Buff[5]=( *( (char *)(&distance) + 3) );
//	Buff[6]=( *( (char *)(&distance) + 2) );
//	Buff[7]=( *( (char *)(&distance) + 1) );
//	Buff[8]=( *( (char *)(&distance) ) );
//  Buff[9]=( *( (char *)(&pwmcount) + 3) );
//	Buff[10]=( *( (char *)(&pwmcount) + 2) );
//	Buff[11]=( *( (char *)(&pwmcount) + 1) );
//	Buff[12]=( *( (char *)(&pwmcount) ) );
//	Buff[13]=0x01;
//	Buff[14]=0x00;
//	Buff[15]=0x00;
//	Buff[16]=0x00;
//	Buff[17]=0x01;
//	Buff[18]=0x00;
//	Buff[19]=0x00;
//	Buff[20]=0x00;
	HAL_UART_Transmit(&huart2,Buff,5,20);	
}
void send_atstart(void)//在起点
{
	uint8_t Buff1[5];
	Buff1[0]=0x50;
	Buff1[1]=0x4F;
	Buff1[2]=0x53;
	Buff1[3]=0x49;
	Buff1[4]=0x13;
	HAL_UART_Transmit(&huart2,Buff1,5,20);
}
void send_fix(void)//dingdian
{
	uint8_t Buff1[5];
	Buff1[0]=0x50;
	Buff1[1]=0x4F;
	Buff1[2]=0x53;
	Buff1[3]=0x49;
	Buff1[4]=0x14;
	HAL_UART_Transmit(&huart2,Buff1,5,20);
}
void send_atend(void)//在终点
{
	uint8_t Buff2[5];
	Buff2[0]=0x50;
	Buff2[1]=0x4F;
	Buff2[2]=0x53;
	Buff2[3]=0x49;
	Buff2[4]=0x12;
	HAL_UART_Transmit(&huart2,Buff2,5,20);
}

void HMIsendb()
{
	uint8_t k[3];
	k[0]=0xFF;
	k[1]=0xFF;
	k[2]=0xFF;
	HAL_UART_Transmit(&huart5,k,3,20);
	
}

void HMIbcd()
{
	uint8_t k1[7];
	k1[0]=0x63;
	k1[1]=0x6c;
	k1[2]=0x73;
	k1[3]=0x20;
	k1[4]=0x52;
	k1[5]=0x45;
	k1[6]=0x44;
	HAL_UART_Transmit(&huart5,k1,7,20);
	
}
void HMIstring()
{
	uint8_t k1[7];
	k1[0]=0x44;
	k1[1]=0x52;
	k1[2]=0x73;
	k1[3]=0x20;
	k1[4]=0x52;
	k1[5]=0x45;
	k1[6]=0x44;
	HAL_UART_Transmit(&huart5,k1,7,20);
	
}
void HMIsends(uint8_t *buff1)
{
	//uint8_t i=0;
//	while(1)
//	{
		if(buff1[0]!=0)
		{
			HAL_UART_Transmit(&huart5,(uint8_t *)buff1,strlen(buff1),20);	
//			while(HAL_UART_GetState(&huart5)==RESET){};
//			i++;
//			if(buff1[i+1]==0)
//			{
//				HAL_UART_Transmit(&huart5,buff1,10,20);
//			}
		}
		else
			return;
	
//	}
}


void reset_gm8125()
{
	int i;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(GM1_RST_GPIO_Port, GM1_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GM0_RST_GPIO_Port, GM0_RST_Pin, GPIO_PIN_RESET);
	
  HAL_GPIO_WritePin(GPIOE, GM1_STADD1_Pin|GM1_STADD2_Pin|GM1_SRADD1_Pin
  												|GM1_SRADD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GM1_STADD0_Pin|GM1_SRADD0_Pin, GPIO_PIN_SET);  // 默认选中第一个串口

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GM0_STADD1_Pin|GM0_STADD2_Pin|GM0_SRADD1_Pin
  												|GM0_SRADD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GM0_STADD0_Pin|GM0_SRADD0_Pin, GPIO_PIN_SET);  // 默认选中第一个串口
	
	HAL_GPIO_WritePin(GPIOB, GM1_MS_Pin|GM0_MS_Pin, GPIO_PIN_SET);  // 单通道模式

	GPIO_InitStruct.Pin = GM1_MS_Pin|GM0_MS_Pin|GM0_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GM1_STADD0_Pin|GM1_STADD1_Pin|GM1_STADD2_Pin|GM1_SRADD0_Pin 
                          |GM1_SRADD1_Pin|GM1_SRADD2_Pin|GM1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GM0_STADD0_Pin|GM0_STADD1_Pin|GM0_STADD2_Pin|GM0_SRADD0_Pin 
                          |GM0_SRADD1_Pin|GM0_SRADD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	i = 3000000;
	while (i--);
//	vTaskDelay(100/portTICK_PERIOD_MS);
	HAL_GPIO_WritePin(GM1_RST_GPIO_Port, GM1_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GM0_RST_GPIO_Port, GM0_RST_Pin, GPIO_PIN_RESET);

	i = 6000000;
	while (i--);
//	vTaskDelay(200/portTICK_PERIOD_MS);
	HAL_GPIO_WritePin(GM1_RST_GPIO_Port, GM1_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GM0_RST_GPIO_Port, GM0_RST_Pin, GPIO_PIN_SET);

	i = 30000000;
	while (i--);
//	vTaskDelay(2000/portTICK_PERIOD_MS);
}
/*************flash*******************/
//void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
//{
//	uint32_t i;
//	for(i=0;i<NumToRead;i++)
//	{
//		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
//		ReadAddr+=4;//偏移4个字节.	
//	}
//}
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}  
/**
  * @}
  */
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
