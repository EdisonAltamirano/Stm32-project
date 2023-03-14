/*
 * can_bus_parser.c
 *
 *  Created on: Jan 29, 2022
 *      Author: abiel
 */

#include "CAN/can_bus.h"
#include "CAN/can_bus_parser.h"
#include "CANMessage.h"

extern volatile uint8_t g_sendPing;
extern volatile uint8_t g_panelModule;
extern volatile float data_panel;
extern volatile uint8_t pot_position;
extern volatile uint8_t max_velocity;
extern volatile uint8_t g_stateThrottle;
extern volatile uint8_t g_throttleModule;
//void can_parse_msg(CAN_RxHeaderTypeDef *header, uint8_t *data){
void can_parse_msg(FDCAN_RxHeaderTypeDef *header, uint8_t *data){
	if(data == NULL) return;

	//uint8_t id = can_parse_id(data, header->DLC);
	uint32_t id = header->Identifier;
	if(id >= MOTOR_ID_START && id <= MOTOR_ID_END){
		//Motor msg
		//float set = can_parse_float(data, header->DLC);
		float set = can_parse_float(data, header->DataLength);
		can_rx_data.motorSetpoints[id - MOTOR_ID_START] = set;
	} else if(id == JETSON_HEARTBEAT_ID) {
		//Jetson heartbeat
		can_rx_data.jetsonHBTick = HAL_GetTick();
	} else if(id == PING_ID){
		g_sendPing = 1;
	}
	else if(id == XAVIER_RECEIVE){
		//Activate flag for leds
		g_panelModule = 1;
		data_panel = data[3];
	}
	else if (id == THROTTLE_POS){
		g_throttleModule = 1;
		g_stateThrottle = 1;
		pot_position = data[0];
	}
	else if (id== THROTTLE_MAX){
		g_throttleModule = 1;
		g_stateThrottle = 2;
		max_velocity = data[0] ;

	}
}
