/*
 * sdv_main.c
 *
 *  Created on: Oct 26, 2022
 *      Author= Z0180064
 */

#include "SDV/sdv.h"
#include "CAN/can_bus.h"
#include "CAN/can.h"
#include "cmsis_os.h"
#include "CAN/can_bus_tx_tasks.h"
#include <stdio.h>
uint32_t a=0;


//void createTasks_sdv() {
//	osThreadNew(mainTask_sdv,NULL,&sdvTask_attributes);
//}

void main_task_sdv(void){
	for(;;){
		sdv_autonomous_loop();
		osDelay(HEARTBEAT_DELAY);
	}
}
void sdv_autonomous_loop(){
	a++;
	//queue_can_msg_short(HEARTBEAT_ID, HAL_GetTick());
}
