/*
 * stepper.h
 *
 *  Created on: Dec 8, 2022
 *      Author: artem
 */

#include "stm32f4xx_hal.h"
#include "main.h"


#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_


typedef struct {
	enum State
	{
		Wait = 0,
		MoveForward = 1,
		MoveBack = 2,
		MoveTo = 3,
	} state;

	int curPosition;
	uint16_t timerPeriod; //  5 999 ---> 49 999
	uint16_t countTimer;
	int speed;
	GPIO_TypeDef* enable_port;
		uint16_t enable_pin;
	GPIO_TypeDef* direction_port;
		uint16_t direction_pin;
	GPIO_TypeDef* step_port;
		uint16_t step_pin;

	GPIO_TypeDef* btn_fw_port;
		uint16_t btn_fw_pin;
		uint8_t btn_fw_oldstate;
	GPIO_TypeDef* btn_bw_port;
		uint16_t btn_bw_pin;
		uint8_t btn_bw_oldstate;

	GPIO_TypeDef* end_fw_port;
		uint16_t end_fw_pin;
		uint8_t end_fw_oldstate;
	GPIO_TypeDef* end_bw_port;
		uint16_t end_bw_pin;
		uint8_t end_bw_oldstate;

	GPIO_TypeDef* end_led_bw_port;
		uint16_t end_led_bw_pin;

	GPIO_TypeDef* end_led_fw_port;
		uint16_t end_led_fw_pin;

	uint8_t block_fw;
	uint8_t block_bw;


//	unsigned long stepTimer = 0;
} Stepper;


void stepper_tick();

void stepper_setDelay(Stepper *stepper, int delay);
void stepper_moveForward(Stepper *stepper);
void stepper_moveBack(Stepper *stepper);
void stepper_moveTo(Stepper *stepper, int target);
void stepper_stop(Stepper *stepper);
void stepper_init(Stepper *stepper,
		GPIO_TypeDef* enable_port, uint16_t enable_pin,
		GPIO_TypeDef* direction_port, uint16_t direction_pin,
		GPIO_TypeDef* step_port, uint16_t step_pin );
void buttons_init(Stepper *stepper,
		GPIO_TypeDef* btn_fw_port, uint16_t btn_fw_pin,
		GPIO_TypeDef* btn_bw_port, uint16_t btn_bw_pin   );

void enders_init(Stepper *stepper,
		GPIO_TypeDef* end_fw_port, uint16_t end_fw_pin,
		GPIO_TypeDef* end_bw_port, uint16_t end_bw_pin  );

void check_enders(Stepper *stepper);
void check_buttons(Stepper *stepper);

void end_leds_init(Stepper *stepper,
		GPIO_TypeDef* end_led_fw_port, uint16_t end_led_fw_pin,
		GPIO_TypeDef* end_led_bw_port, uint16_t end_led_bw_pin);


#endif /* SRC_STEPPER_H_ */
