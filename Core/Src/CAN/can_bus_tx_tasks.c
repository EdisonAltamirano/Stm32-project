/*
 * can_bus_tx_tasks.c
 *
 *  Created on: Jan 15, 2022
 *      Author: abiel
 */

#include "CAN/can_bus_tx_tasks.h"
#include "cmsis_os.h"
#include "CAN/can.h"
#include <stdio.h>


volatile uint8_t g_sendPing;
volatile uint8_t g_panelModule;
volatile float data_panel;
volatile uint8_t g_sendInfopanel;
volatile uint8_t g_panelError;



static const osThreadAttr_t hbTask_attributes = {
  .name = "hbTxCan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t debugMsgTask_attributes = {
  .name = "debugMsgTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t pingTask_attributes = {
  .name = "pingMsgTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryMonitorTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static const osThreadAttr_t debugVarsTask_attributes = {
	.name = "debugVars",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};


static void can_hb_tx_task(void *param){
	for(;;){
		queue_can_msg_short(HEARTBEAT_ID, HAL_GetTick());
		osDelay(HEARTBEAT_DELAY);
	}
}




static void can_debug_msg_tx_task(void *param){
	for(;;){
		while(handle_debug_msg_queue() == osOK);
		osDelay(DEBUG_MSG_DELAY);
	}
}

static void can_send_ping_task(void * param){
	for(;;){
		if(g_sendPing == 1){
			queue_can_msg_byte(PING_ID, 0);
			g_sendPing = 0;
		}

		osDelay(1);
	}
}

#include "CAN/ver_data.h"
char debug_buffer[128];
static void debug_vars_task(void *param){
	for(;;){
		int len = snprintf(debug_buffer, sizeof(debug_buffer), "[DEBUG] Version: %s", GIT_VERSION);
		send_can_debug_msg(debug_buffer, len);
		len = snprintf(debug_buffer, sizeof(debug_buffer), "[DEBUG] Date: %s", COMPILE_DATE);
		send_can_debug_msg(debug_buffer, len);
		len = snprintf(debug_buffer, sizeof(debug_buffer), "[DEBUG] User: %s", COMPILE_USER);
		send_can_debug_msg(debug_buffer, len);
		osDelay(60*1000); //send every minute
	}
}



void start_can_tx_tasks(){
	osThreadNew(can_hb_tx_task, NULL, &hbTask_attributes);
	osThreadNew(can_debug_msg_tx_task, NULL, &debugMsgTask_attributes);
	osThreadNew(can_send_ping_task, NULL, &pingTask_attributes);
	osThreadNew(debug_vars_task, NULL, &debugVarsTask_attributes);
}
