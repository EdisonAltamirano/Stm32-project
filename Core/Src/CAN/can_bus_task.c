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

//extern CAN_HandleTypeDef hcan2;
extern FDCAN_HandleTypeDef hcan2;

osMessageQueueId_t debugMessageQueue;
osMessageQueueId_t txMessageQueue;

void can_init(){
	//CAN_FilterTypeDef filter;
	//filter.FilterBank = 20;
	//filter.FilterMode = CAN_FILTERMODE_IDMASK;
	//filter.FilterScale = CAN_FILTERSCALE_32BIT;
	//filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	//filter.FilterActivation = CAN_FILTER_ENABLE;
	//filter.FilterIdHigh = 0;
	//filter.FilterIdLow = 0;
	//filter.FilterMaskIdHigh = 0;
	//filter.FilterMaskIdLow = 0;
	//filter.SlaveStartFilterBank = 14;
	 /* Configure standard ID reception filter to Rx buffer 0 */
	FDCAN_FilterTypeDef filter;
	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = 0x321;
	filter.FilterID2 = 0x7FF;

	HAL_StatusTypeDef ret = HAL_FDCAN_ConfigFilter(&hcan2, &filter);
	if(ret != HAL_OK) Error_Handler();
	//HAL_CAN_Start(&hcan2);
	//HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	//HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_FDCAN_Start(&hcan2);
	HAL_FDCAN_ActivateNotification(&hcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);

	//Initialize queues
	//First char -> size
	debugMessageQueue = osMessageQueueNew(64, 8, NULL);
	if(debugMessageQueue == NULL)
		Error_Handler();

	txMessageQueue = osMessageQueueNew(64, sizeof(CAN_TX_QUEUE_OBJ), NULL);
	if(txMessageQueue == NULL)
		Error_Handler();
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
		HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hcan2, &txHeader, txOut.buf);
		//txHeader.DLC = txOut.msg_size;
		//HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan2, &txHeader, txOut.buf, &txMailbox);
		if(ret != HAL_OK){
			//txHeader.DLC++;
			txHeader.DataLength++;
		}
	}
}

void can_tx_task(void * params){
	for(;;){

		while(HAL_FDCAN_GetTxFifoFreeLevel(&hcan2) != 0)
			can_tx_update();

		osDelay(can_tx_task_delay);
	}
}

void can_rx_update(){
	//TODO check both FIFO?
	//CAN_RxHeaderTypeDef rxHeader;
	FDCAN_RxHeaderTypeDef rxHeader;
	uint8_t buf[8];
	//while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) != 0){
	while(HAL_FDCAN_GetRxFifoFillLevel(&hcan2, FDCAN_RX_FIFO0) != 0){
		//HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, buf);
		HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(&hcan2, FDCAN_RX_FIFO0, &rxHeader, buf);
		if(ret != HAL_OK) continue;
		//Parse can message
		can_parse_msg(&rxHeader, buf);
	}
	//while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1) != 0){
	while(HAL_FDCAN_GetRxFifoFillLevel(&hcan2, FDCAN_RX_FIFO1) != 0){
			//HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan2, FDCAN_RX_FIFO1, &rxHeader, buf);
		HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(&hcan2, FDCAN_RX_FIFO1, &rxHeader, buf);
		if(ret != HAL_OK) continue;
			//Parse can message
			can_parse_msg(&rxHeader, buf);
		}
}

void can_rx_task(void *params){
	for(;;){
		can_rx_update();
		osDelay(can_rx_task_delay);
	}
}
