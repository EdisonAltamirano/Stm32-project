/*
 * can_bus_task.c
 *
 *  Created on: Jan 9, 2022
 *      Author: abiel
 */

#include <string.h>
#include "CAN/can_bus.h"
#include "stm32g4xx_hal.h"
#include "CANMessage.h"
#include "CAN/can_bus_task.h"
#include "CAN/can.h"
#include "CAN/can_bus_parser.h"
#include "main.h"
//extern CAN_HandleTypeDef hcan2;
extern FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
//uint8_t TxData0[] = {0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22};
uint8_t TxData0[] = {0x43,0x04,0x60,0x00};
osMessageQueueId_t debugMessageQueue;
osMessageQueueId_t txMessageQueue;
uint32_t ab=0;
uint8_t RxData[8];
FDCAN_RxHeaderTypeDef RxHeader;
static uint32_t BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
uint32_t Tickstart;
#define TX_FAST_TIMEOUT 10

void can_init(){
	/* Configure standard ID reception filter to Rx FIFO 0 */
	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = 0x5A0;
	filter.FilterID2 = 0xFFF;

	HAL_StatusTypeDef ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
	if(ret != HAL_OK) Error_Handler();
	/* Configure extended ID reception filter to Rx FIFO 1 */
	filter.IdType = FDCAN_EXTENDED_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	filter.FilterID1 = 0x5A0;
	filter.FilterID2 = 0xFFF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
	{
	Error_Handler();
	}
  /* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	/*##-2 Start FDCAN controller (continuous listening CAN bus) ##############*/
	  /* Start the FDCAN module */
	  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    Error_Handler();
	  }
	//Initialize queues
	//First char -> size
	//	debugMessageQueue = osMessageQueueNew(64, 8, NULL);
	//	if(debugMessageQueue == NULL)
	//		Error_Handler();
	//
	//	txMessageQueue = osMessageQueueNew(64, sizeof(CAN_TX_QUEUE_OBJ), NULL);
	//	if(txMessageQueue == NULL)
	//		Error_Handler();
}

void can_tx_update(){
//	CAN_TxHeaderTypeDef txHeader;
//	uint32_t txMailbox;
//	txHeader.IDE = CAN_ID_STD;
//	txHeader.StdId = 0x111;
//	txHeader.RTR = CAN_RTR_DATA;

	FDCAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	txHeader.Identifier = 0x111;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	CAN_TX_QUEUE_OBJ txOut;

	if(osMessageQueueGet(txMessageQueue, &txOut, NULL, 0) == osOK){
		txHeader.DataLength = txOut.msg_size;
		HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txOut.buf);
		//txHeader.DLC = txOut.msg_size;
		//HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan2, &txHeader, txOut.buf, &txMailbox);
		if(ret != HAL_OK){
			//txHeader.DLC++;
			txHeader.DataLength++;
		}
	}
}


/**
  * @brief Compares two buffers.
  * @par Input
  *  - pBuffer1, pBuffer2: buffers to be compared.
  *  - BufferLength: buffer's length
  * @par Output
  * None.
  * @retval
  *   0: pBuffer1 identical to pBuffer2
  *   1: pBuffer1 differs from pBuffer2
  */

static uint32_t BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}


void can_tx_task(void){
	for(;;){



		  /* Add message to Tx FIFO */
//		  TxHeader.Identifier = 0x444;
//		  TxHeader.IdType = FDCAN_STANDARD_ID;
//		  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//		  TxHeader.DataLength =  FDCAN_DLC_BYTES_8;
//		  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//		  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
//		  TxHeader.FDFormat = FDCAN_FD_CAN;
//		  TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
//		  TxHeader.MessageMarker = 0x52;
		  TxHeader.Identifier = 0x620;//StdId
		  TxHeader.IdType = FDCAN_STANDARD_ID;//IDE
		  TxHeader.TxFrameType = FDCAN_DATA_FRAME;//RTR
		  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
		  TxHeader.DataLength = FDCAN_DLC_BYTES_4;//DLC
		  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
		  TxHeader.FDFormat = FDCAN_FD_CAN;
		  TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
		  TxHeader.MessageMarker = 0x52;
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData0) != HAL_OK)
		  {
			Error_Handler();
			//TxHeader.DataLength++;
		  }
		  /* Get tick */
		  Tickstart = HAL_GetTick();

		  /* Check transmission occurred before timeout */
		  while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0)
		  {
		    if((HAL_GetTick() - Tickstart) > TX_FAST_TIMEOUT)
		    {
		      Error_Handler();
		      break;
		    }
		  }

		   ab++;
		//sdv_autonomous_loop1();
		osDelay(can_tx_task_delay);

	}
}
void sdv_autonomous_loop1(){
	ab++;
	//queue_can_msg_short(HEARTBEAT_ID, HAL_GetTick());
}
void can_rx_update(){
	//TODO check both FIFO?
	//CAN_RxHeaderTypeDef rxHeader;
	//	FDCAN_RxHeaderTypeDef rxHeader;
	uint8_t buf[8];
		//while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) != 0){
		while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 1){
			//HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, buf);
			HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, buf);
			if(ret != HAL_OK) continue;
			//Parse can message
			can_parse_msg(&RxHeader, buf);
		}
		//while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) != 0){

//		while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) != 1){
//			//HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan2, FDCAN_RX_FIFO1, &rxHeader, buf);
//			HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, buf);
//			if(ret != HAL_OK) continue;
//				//Parse can message
//			can_parse_msg(&RxHeader, buf);
//		}

	 /* Wait transmissions complete */
	//ab++;
	 /* Get tick */
	  /* Get tick */
//		  Tickstart = HAL_GetTick();
//		  /* Check transmission occurred before timeout */
//		  while(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0)
//		  {
//		    if((HAL_GetTick() - Tickstart) > TX_FAST_TIMEOUT)
//		    {
//		      Error_Handler();
//		      break;
//		    }
//		  }

		//				  Tickstart = HAL_GetTick();
		//			//	  /* Check one message is received in Rx FIFO 0 */
		//				  while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) < 1)
		//				  {
		//				    if((HAL_GetTick() - Tickstart) > RCC_CRS_TIMEOUT)
		//				    {
		//				      Error_Handler();
		//				      break;
		//				    }
		//				  }

      //while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) < 1){}
	  /*##-4 Receive messages ###################################################*/
	  /* Check one message is received in Rx FIFO 0 */
	//	  if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) != 1)
	//	  {
	//	    Error_Handler();
	//	  }

	//	  /* Retrieve message from Rx FIFO 0 */
	//	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	//	  {
	//	    Error_Handler();
	//	  }
	//
	//	  /* Compare payload to expected data */
	//	  if (BufferCmp8b(TxData0, RxData, 12) != 0)
	//	  {
	//	    Error_Handler();
	//	  }
	//
	//	/* Retrieve message from Rx FIFO 0 */
	//	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	//	{
	//	  Error_Handler();
	//	}
	//
	//
	//	  /* Compare payload to expected data */
	//	  if (BufferCmp8b(TxData0, RxData, 12) != 0)
	//	  {
	//	    Error_Handler();
	//	  }
	//
	//	  /* Check two messages are received in Rx FIFO 1 */
	//	  if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) != 2)
	//	  {
	//	    Error_Handler();
	//	  }
	//
	//	  /* Retrieve message from Rx FIFO 1 */
	//	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
	//	  {
	//	    Error_Handler();
	//	  }
	//
	//	  /* Retrieve next message from Rx FIFO 1 */
	//	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
	//	  {
	//	    Error_Handler();
	//	  }


}

void can_rx_task(void *params){
	for(;;){
		can_rx_update();
		osDelay(can_rx_task_delay);
	}
}
