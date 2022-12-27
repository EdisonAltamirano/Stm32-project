/*
 * can_bus_task.h
 *
 *  Created on: Jan 9, 2022
 *      Author: abiel
 */

#ifndef INC_CAN_CAN_BUS_TASK_H_
#define INC_CAN_CAN_BUS_TASK_H_

#include "can_bus.h"
#include <stdint.h>
#include "cmsis_os.h"
#include <stdio.h>
#include <stdbool.h>

static const uint32_t can_tx_task_delay = 5;
FDCAN_FilterTypeDef filter;

static const uint32_t can_rx_task_delay = 5;
struct panel_t_struct{
	char *key;
	GPIO_TypeDef *type;
	uint16_t value;
	int status;
	uint16_t can_value;
} ;
struct panel_t_struct led1, led2, led3, led4, led5, led6;
struct panel_t_struct panel_leds[6] ;
uint8_t buf[8];
uint8_t pos[8];
uint8_t buf1[8];

void panel_init();
void lcd_init();
void lcd_send_cmd(char cmd);
void lcd_send_string(char *str);
void lcd_send_data (char data);
bool getWiper(uint8_t reg,uint8_t *data);
bool setWiper(uint8_t reg,uint8_t data);
bool pot_init();

/**
 * Updates the tx_data struct which is sent over CAN
 */
void can_init();

void can_tx_update();

void can_tx_task();

void can_rx_update();
void can_task_panel();
void can_task_pot();
void can_task_lcd();
void sdv_autonomous_loop1();

void can_rx_task(void *);

#endif /* INC_CAN_CAN_BUS_TASK_H_ */
