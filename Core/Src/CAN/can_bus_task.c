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
extern I2C_HandleTypeDef hi2c3;
//extern I2C_HandleTypeDef hi2c2;
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
#define SLAVE_ADDRESS_LCD 0x27<<1 // change this according to ur setup
static const uint8_t DS3502_ADDR = 0x27<<1 ;
static const uint8_t DS3502_MODE_WR = 0x80;
static const uint8_t DS3502_MODE_WR_IVR = 0x00;
static const uint8_t DS3502_REG_WR_IVR = 0x00;
static const uint8_t DS3502_REG_CR = 0x02;
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
void lcd_send_string(char *str){
	while (*str) lcd_send_data (*str++);
}
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c3, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}
void lcd_send_cmd(char cmd){
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c3, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init(){
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
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

bool pot_init(){
    //@return True if initialization was successful, otherwise false.
    // Tell DS3502 that we want to read from the pot register
	 buf_pot[0] = DS3502_REG_CR;
	 HAL_StatusTypeDef  ret = HAL_I2C_Master_Transmit(&hi2c3, DS3502_ADDR, buf_pot, 1, HAL_MAX_DELAY);
	 if (ret != HAL_OK){
		 return false;
	 }
	 else{
		 //Write the MODE bit which determines how I2C data is written to the WR and IVR data register
		 buf_pot[0] = DS3502_MODE_WR;
		 ret =  HAL_I2C_Master_Transmit(&hi2c3, DS3502_ADDR, buf_pot, 1, HAL_MAX_DELAY);
		 if (ret != HAL_OK){
		 		 return false;
		 	 }
		 else{
			 return true;
		 }
	 }
}

bool getWiper(uint8_t reg,uint8_t *data){
	 buf_pot[0] = reg;
	 HAL_StatusTypeDef  ret = HAL_I2C_Master_Transmit(&hi2c3, DS3502_ADDR, buf_pot, 1, HAL_MAX_DELAY);
	 if (ret != HAL_OK){
		 return false;
	 }
	 else{
		 HAL_I2C_Master_Receive(&hi2c3, DS3502_ADDR, data, 1, HAL_MAX_DELAY);
		 if (ret != HAL_OK){
		 		 return false;
		 	 }
		 else{
			 return true;
		 }
	 }
}
bool setWiper(uint8_t reg,uint8_t data){
	 buf_pot[0] = reg;
	 HAL_StatusTypeDef  ret = HAL_I2C_Master_Transmit(&hi2c3, DS3502_ADDR, buf_pot, 1, HAL_MAX_DELAY);
	 if (ret != HAL_OK){
		 return false;
	 }
	 else{
		 buf_pot[0] = data;
		 ret =  HAL_I2C_Master_Transmit(&hi2c3, DS3502_ADDR, buf_pot, 1, HAL_MAX_DELAY);
		 if (ret != HAL_OK){
		 		 return false;
		 	 }
		 else{
			 return true;
		 }
	 }
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
		uint8_t new_wiper_value = 0x3C;
		bool read = true;
		if((new_wiper_value > 0) || (new_wiper_value < 0x7F) ) {
			setWiper(DS3502_REG_WR_IVR,new_wiper_value );
		}
		if(read){
			getWiper(DS3502_REG_WR_IVR,pos);
		}
		osDelay(500);
	}
}

void can_task_lcd(){
	for(;;){
		lcd_send_string("HELLO WORLD");
		osDelay(500);
	}
}
