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
#include <stdio.h>
#include <stdlib.h>
#include "CAN/can_bus_tx_tasks.h"
#include <stdbool.h>
//extern CAN_HandleTypeDef hcan2;
extern FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t CarTxData[] = {0x00};
osMessageQueueId_t debugMessageQueue;
osMessageQueueId_t txMessageQueue;
uint32_t ab=0;
uint8_t buf_pot[12];
int cmp = 0;
extern volatile uint8_t g_panelModule;
extern volatile uint8_t g_sendInfopanel;
extern volatile uint8_t g_panelError;
extern volatile float data_panel;
void can_init(){

	FDCAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.IdType = FDCAN_STANDARD_ID;
	  sFilterConfig.FilterIndex = 0;
	  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig.FilterID1 = 0x111;
	  sFilterConfig.FilterID2 = 0xFFF;
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
	  if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
	  {
		  Error_Handler();
	  }
	  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	  {
	    /* Notification Error */
	    Error_Handler();
	  }
}
void panel_init(){

	led1.key = "Wiper";
	led1.type = GPIOA;
	led1.value = GPIO_PIN_0;
	led1.status = GPIO_PIN_RESET;
	led2.key = "Horn";
	led2.type = GPIOA;
	led2.value = GPIO_PIN_1;
	led2.status = GPIO_PIN_RESET;
	led3.key = "RULight";
	led3.type = GPIOA;
	led3.value = GPIO_PIN_2;
	led3.status = GPIO_PIN_RESET;
	led4.key = "LULight";
	led4.type = GPIOA;
	led4.value = GPIO_PIN_6;
	led4.status = GPIO_PIN_RESET;
	led5.key = "RDLight";
	led5.type = GPIOA;
	led5.value = GPIO_PIN_4;
	led5.status = GPIO_PIN_RESET;
	led6.key = "RDLight";
	led6.type = GPIOA;
	led6.value = GPIO_PIN_5;
	led6.status = GPIO_PIN_RESET;
	panel_leds[0] = led1;
	panel_leds[1]  =led2;
	panel_leds[2]  =led3;
	panel_leds[3]  =led4;
	panel_leds[4]  =led5;
	panel_leds[5]  =led6;
}

void can_tx_task(void){
	if(g_sendInfopanel){
		  TxHeader.Identifier = 0x222;
		  TxHeader.IdType = FDCAN_STANDARD_ID;
		  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		  TxHeader.DataLength = FDCAN_DLC_BYTES_1;
		  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
		  TxHeader.FDFormat = FDCAN_FD_CAN;
		  TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
		  TxHeader.MessageMarker = 0;
		  if(g_panelError){
			  CarTxData[0] = FAILURE_CAN_MSG;
			  g_panelError = 1;
		  }
		  else{
			  CarTxData[0] = SUCCESS_CAN_MSG;
		  }

		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CarTxData)!= HAL_OK)
		  {
		   Error_Handler();
		  }
		osDelay(can_tx_task_delay);
		g_sendInfopanel = 0;
	}
	osDelay(500);

}

void can_rx_task(void *params){
	for(;;){
		//TODO check both FIFO?
		//CAN_RxHeaderTypeDef rxHeader;
		FDCAN_RxHeaderTypeDef rxHeader;
		//while(HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) != 0){
		while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0){
			//HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rxHeader, buf);
			HAL_StatusTypeDef ret = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, buf);
			if(ret != HAL_OK) continue;
			//Parse can message
			can_parse_msg(&rxHeader, buf);
		}
		ab++;
		osDelay(can_rx_task_delay);
	}
}
void sdv_autonomous_loop1(){
	ab++;
	//queue_can_msg_short(HEARTBEAT_ID, HAL_GetTick());
}
void can_task_panel(void)
{
  /* USER CODE BEGIN panel_task */
  /* Infinite loop */
  for(;;)
  {
		if(g_panelModule){
			if(data_panel>=1 && data_panel<=16){
				if(data_panel>=10){
					data_panel=data_panel-10;
					HAL_GPIO_WritePin(panel_leds[(int) data_panel].type, panel_leds[(int) data_panel].value, GPIO_PIN_RESET);
				}
				else{
					HAL_GPIO_WritePin(panel_leds[(int) data_panel].type, panel_leds[(int) data_panel].value, GPIO_PIN_SET);
					osDelay(1000);
				}
			}
			else{
				g_panelError = 1;
			}
			//		for (int i = 0; i < sizeof(panel_leds)/sizeof(panel_leds[0]); ++i) {
			//			//HAL_GPIO_TogglePin(panel_leds[i].type, panel_leds[i].value);
			//			HAL_GPIO_WritePin(panel_leds[i].type, panel_leds[i].value, GPIO_PIN_SET);
			//			osDelay(200);
			//			HAL_GPIO_WritePin(panel_leds[i].type, panel_leds[i].value, GPIO_PIN_RESET);
			//			osDelay(200);
			//		}
			g_panelModule = 0;
			g_sendInfopanel = 1;
		}
	  osDelay(500);
  }
  /* USER CODE END panel_task */
}
void can_task_pot(){
	for(;;){
		osDelay(500);
	}
}

void can_task_lcd(){
	for(;;){
		osDelay(500);
	}
}
