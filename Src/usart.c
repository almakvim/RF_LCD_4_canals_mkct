/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "setup.h"
#include "system.h"
#include "control.h"
#include "mk_conf_tree.h"

USART_TX_BUFFER USART3Tx;
USART_RX_BUFFER USART3Rx;

u16 flag_usb_pkt = 0;
u16 flag_sbus_pkt = 0;
AT_SETUP at_set;

/* USER CODE END 0 */

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 1 */
//=========================================================================
 void type_byte(uint8_t data)
 {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, data);
 }
//=========================================================================
void USART_Write(USART_TypeDef *USARTx, uint8_t data)
 {
    while (!LL_USART_IsActiveFlag_TXE(USARTx));
    LL_USART_TransmitData8(USARTx, data);
 }
//=========================================================================
void USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t baud)
{
    uint32_t PeriphClk;
    LL_RCC_ClocksTypeDef rcc_clocks;
    
    LL_RCC_GetSystemClocksFreq(&rcc_clocks);

    LL_USART_Disable(USARTx);
    
    if(USARTx == USART1) PeriphClk = rcc_clocks.PCLK2_Frequency;
    else if(USARTx == USART2) PeriphClk = rcc_clocks.PCLK1_Frequency;
    
    LL_USART_SetBaudRate(USARTx, PeriphClk, baud);
    LL_USART_Enable(USARTx);
}

//=========================================================================
void  USART3_RX_Callback(void)
{
 	u8 Data;
	volatile USART_RX_BUFFER *USARTRx;

//    if(dev_var.dir) return;
    
    USARTRx = &USART3Rx;
 	Data = LL_USART_ReceiveData8(USART3);
 	if(USART_RxCount((*USARTRx)) < (USART_RX_BUFFER_SIZE - 1))
 	{
 		USARTRx->Buffer[USARTRx->WrPos++] = Data;
        //if(dev_var.sbus == 0) 
//        CDC_Transmit_FS(&Data, 1);
 		USARTRx->WrPos %= USART_RX_BUFFER_SIZE;
  	}
}
 
//=========================================================================
void Clear_buffers(void)
{
    USART3Rx.WrPos = USART3Rx.RdPos = 0;
    
    for(u16 i=0; i<256; i++)
    {
        USART3Rx.Buffer[i] = 0;
    }
}
//=========================================================================
void sbus_config_send(TX_CMD * dev)
{
    int len = 0;
    uint8_t TxBuf[64];

    TxBuf[len++] = 0xfa;
    TxBuf[len++] = 0xce;
    TxBuf[len++] = dev->id_dev;
    TxBuf[len++] = dev->id_dev>>8;
    TxBuf[len++] = 0;               // TxBuf[4]
    TxBuf[len++] = dev->cmd;

    switch(dev->cmd)
    {
    case CMD_10:
        TxBuf[4] = 7;
        TxBuf[len++] = dev->id_unit;
        TxBuf[len++] = dev->id_unit>>8;
        TxBuf[len++] = dev->n_par;
        TxBuf[len++] = dev->n_par>>8;
        TxBuf[len++] = dev->l_par;
        TxBuf[len++] = dev->l_par>>8;
        break;
    case CMD_11:
        TxBuf[4] = dev->l_par*4+7;
        TxBuf[len++] = dev->id_unit;
        TxBuf[len++] = dev->id_unit>>8;
        TxBuf[len++] = dev->n_par;
        TxBuf[len++] = dev->n_par>>8;
        TxBuf[len++] = dev->l_par;
        TxBuf[len++] = dev->l_par>>8;

		uint32_t param = (uint32_t)dev->data.dt_32;
		TxBuf[len++] = param;
		TxBuf[len++] = param>>8;
		TxBuf[len++] = param>>16;
		TxBuf[len++] = param>>24;
        break;

    }

    uint16_t crc = CRC16(TxBuf, len);
    TxBuf[len++] = crc;
    TxBuf[len++] = crc>>8;

    uint16_t k = 0;
    while(len--)
    {
        while(LL_USART_IsActiveFlag_TC(USART3) == 0);
        LL_USART_TransmitData8(USART3, TxBuf[k++]);
        //osDelay(2);
    }
}

//=========================================================================
void mkbus_config_send(TX_CMD * dev)
{
    int len = 0;
    uint8_t TxBuf[64];

    TxBuf[len++] = 0xca;
    TxBuf[len++] = 0xfe;
    TxBuf[len++] = dev->id_dev;
    TxBuf[len++] = dev->id_dev>>8;
    TxBuf[len++] = 0;               // TxBuf[4]
    TxBuf[len++] = dev->cmd;

    switch(dev->cmd)
    {
    case CMD_4:
        TxBuf[4] = 4;
        TxBuf[len++] = dev->id_unit;
        TxBuf[len++] = dev->n_par;
        TxBuf[len++] = dev->l_par;
        break;
    case CMD_5:
        TxBuf[4] = dev->l_par+4;
        TxBuf[len++] = dev->id_unit;
        TxBuf[len++] = dev->n_par;
        TxBuf[len++] = dev->l_par;
		uint8_t param = (uint8_t)dev->data.dt_8[0];
		TxBuf[len++] = param;
        break;

    }

    uint16_t crc = CRC16(TxBuf, len);
    TxBuf[len++] = crc;
    TxBuf[len++] = crc>>8;

    uint16_t k = 0;
    while(len--)
    {
        while(LL_USART_IsActiveFlag_TC(USART3) == 0);
        LL_USART_TransmitData8(USART3, TxBuf[k++]);
        //osDelay(2);
    }
}

//=========================================================================
void USART3_Proc(void)
{
    if(USART3Rx.WrPos != USART3Rx.RdPos)
    {
        dev_var.count_pkt_in++;
    	MKBUS_rx(&mkChan1, USART3Rx.Buffer[USART3Rx.RdPos] );
        USART3Rx.RdPos = (USART3Rx.RdPos + 1) & USART_RX_MASK;
    }
//--------------------------------------------------------------------    
	if( (mkChan1.time+10 < HAL_GetTick()) && (mkChan1.txLen))
    {
        MKBUS_send(mkChan1.pkt, mkChan1.txLen);
		mkChan1.txLen = 0;
	}
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
