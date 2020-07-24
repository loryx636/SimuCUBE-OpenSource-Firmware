/**
 ******************************************************************************
 * File Name          : stm32f4xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 ******************************************************************************
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

//extern void Error_Handler(void);
extern void _Error_Handler(char *, int);

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void)
{
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 3, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 3, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 3, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 3, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 3, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 3, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

	/* USER CODE BEGIN MspInit 1 */
	asm("nop");
	/* USER CODE END MspInit 1 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = EXT_POT_IN_3_Pin|EXT_POT_IN_2_Pin|EXT_POT_IN_1_Pin|X11_LOWER_2_Pin
                          |X11_UPPER_5_Pin|X11_UPPER_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = X11_LOWER_3_Pin|X11_LOWER_5_Pin|X11_LOWER_6_Pin|X11_LOWER_7_Pin
                          |X11_UPPER_6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = X11_UPPER_2_Pin|X11_UPPER_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  }


}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();


    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOC, EXT_POT_IN_3_Pin|EXT_POT_IN_2_Pin|EXT_POT_IN_1_Pin|X11_LOWER_2_Pin
                          |X11_UPPER_5_Pin|X11_UPPER_3_Pin);

    HAL_GPIO_DeInit(GPIOA, X11_LOWER_3_Pin|X11_LOWER_5_Pin|X11_LOWER_6_Pin|X11_LOWER_7_Pin
                          |X11_UPPER_6_Pin);

    HAL_GPIO_DeInit(GPIOB, X11_UPPER_2_Pin|X11_UPPER_1_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(huart->Instance==USART1)
	{
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		/**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /* Peripheral DMA init*/

	    hdma_usart1_rx.Instance = DMA2_Stream2;
	    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	}
	else if(huart->Instance==USART3)
	  {
	  /* USER CODE BEGIN USART3_MspInit 0 */

	  /* USER CODE END USART3_MspInit 0 */
	    /* Peripheral clock enable */
	    __HAL_RCC_USART3_CLK_ENABLE();


	    /**USART3 GPIO Configuration
	    PB10     ------> USART3_TX
	    PB11     ------> USART3_RX
	    */

	    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	    /* Peripheral DMA init*/

	    hdma_usart3_rx.Instance = DMA1_Stream1;
	    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
	    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
	    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
	    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    __HAL_LINKDMA(huart,hdmarx,hdma_usart3_rx);

	    /* Peripheral interrupt init */
	    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(USART3_IRQn);
	  /* USER CODE BEGIN USART3_MspInit 1 */

	  /* USER CODE END USART3_MspInit 1 */
	  }
	  else  if(huart->Instance==UART4)
	  {
	  /* USER CODE BEGIN UART4_MspInit 0 */

	  /* USER CODE END UART4_MspInit 0 */
	    /* Peripheral clock enable */
	    __HAL_RCC_UART4_CLK_ENABLE();

	    /**UART4 GPIO Configuration
	    PA0-WKUP     ------> UART4_TX
	    PA1     ------> UART4_RX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* USER CODE BEGIN UART4_MspInit 1 */

	  /* USER CODE END UART4_MspInit 1 */
	  }
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

	if(huart->Instance==USART1)
	{
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	}
	else if(huart->Instance==USART3)
	{
		/* USER CODE BEGIN USART3_MspDeInit 0 */

		/* USER CODE END USART3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();
		/**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

		/* USER CODE BEGIN USART3_MspDeInit 1 */

		/* USER CODE END USART3_MspDeInit 1 */
	}
	else if(huart->Instance==UART4)
	  {
	  /* USER CODE BEGIN UART4_MspDeInit 0 */

	  /* USER CODE END UART4_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_UART4_CLK_DISABLE();

	    /**UART4 GPIO Configuration
	    PA0-WKUP     ------> UART4_TX
	    PA1     ------> UART4_RX
	    */
	    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

	  /* USER CODE BEGIN UART4_MspDeInit 1 */

	  /* USER CODE END UART4_MspDeInit 1 */
	  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* TIM4 interrupt Init */
    //HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    //HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }


}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
