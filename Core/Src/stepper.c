/*
 * stepper.c
 *
 *  Created on: Dec 8, 2022
 *      Author: artem
 */


#include "stepper.h"

void stepper_setDelay(Stepper *stepper, int delay)
{
	//stepper->stepDelay = delay;
}


void stepper_moveForward(Stepper *stepper)
{
	stepper->state = MoveForward;
}

void stepper_moveBack(Stepper *stepper)
{
	stepper->state = MoveBack;
}

void stepper_moveTo(Stepper *stepper, int target)
{
	stepper->state = MoveTo;
}

void stepper_stop(Stepper *stepper)
{
	stepper->state = Wait;
}

void stepper_init(Stepper *stepper,
		GPIO_TypeDef* enable_port, uint16_t enable_pin,
		GPIO_TypeDef* direction_port, uint16_t direction_pin,
		GPIO_TypeDef* step_port, uint16_t step_pin )
{
	stepper->enable_port = enable_port;
	stepper->enable_pin = enable_pin;
	stepper->direction_port = direction_port;
	stepper->direction_pin = direction_pin;
	stepper->step_port = step_port;
	stepper->step_pin = step_pin;


	stepper->state = Wait;
	stepper->curPosition = 0;
	//stepper->stepDelay = 40;

	stepper->block_fw = 0;
	stepper->block_bw = 0;

}

void buttons_init(Stepper *stepper,
		GPIO_TypeDef* btn_fw_port, uint16_t btn_fw_pin,
		GPIO_TypeDef* btn_bw_port, uint16_t btn_bw_pin   )
{
	stepper->btn_fw_port = btn_fw_port;
	stepper->btn_fw_pin = btn_fw_pin;
	stepper->btn_bw_port = btn_bw_port;
	stepper->btn_bw_pin = btn_bw_pin;
	stepper->timerPeriod = 29999;
}

void enders_init(Stepper *stepper,
		GPIO_TypeDef* end_fw_port, uint16_t end_fw_pin,
		GPIO_TypeDef* end_bw_port, uint16_t end_bw_pin  )
{
	stepper->end_fw_port = end_fw_port;
	stepper->end_fw_pin = end_fw_pin;
	stepper->end_bw_port = end_bw_port;
	stepper->end_bw_pin = end_bw_pin;


	//проверка на блокировки движения при включении
	if (HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin))
		stepper->block_fw = 1;
	else stepper->block_fw = 0;
	if (HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin))
		stepper->block_bw = 1;
	else stepper->block_bw = 0;
}

void check_buttons(Stepper *stepper)
{
	if (HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin) != stepper->btn_fw_oldstate)
	{
		if((HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin)) == GPIO_PIN_SET)
		{
		 //движение вперед степ А
			if (stepper->block_fw == 0)
			{
				stepper_moveForward(stepper);
			}
		}
		else stepper_stop(stepper);

		stepper->btn_fw_oldstate = HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin);
	}



	if (HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin) != stepper->btn_bw_oldstate)
	{
		if((HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin)) == GPIO_PIN_SET)
		{
		 //движение вперед степ А
			if (stepper->block_bw == 0)
			{
				stepper_moveBack(stepper);
			}
		}
		else stepper_stop(stepper);

		stepper->btn_bw_oldstate = HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin);
	}
}

void check_enders(Stepper *stepper)
{
	if (HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin) != stepper->end_fw_oldstate)
	{
		if (!HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin))
		{
			stepper_stop(stepper);
			stepper->block_fw = 1;
			HAL_GPIO_WritePin(stepper->end_led_fw_port, stepper->end_led_fw_pin, GPIO_PIN_SET);
		}
		else
		{
			stepper->block_fw = 0;
			HAL_GPIO_WritePin(stepper->end_led_fw_port, stepper->end_led_fw_pin, GPIO_PIN_RESET);
		}

		stepper->end_fw_oldstate = HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin);
	}
	if (HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin) != stepper->end_bw_oldstate)
	{
		if (!HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin))
		{
			stepper_stop(stepper);
			stepper->block_bw = 1;
			HAL_GPIO_WritePin(stepper->end_led_bw_port, stepper->end_led_bw_pin, GPIO_PIN_SET);
		}
		else
		{
			stepper->block_bw = 0;
			HAL_GPIO_WritePin(stepper->end_led_bw_port, stepper->end_led_bw_pin, GPIO_PIN_RESET);
		}
		stepper->end_bw_oldstate = HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin);
	}
}

void end_leds_init(Stepper *stepper,
		GPIO_TypeDef* end_led_fw_port, uint16_t end_led_fw_pin,
		GPIO_TypeDef* end_led_bw_port, uint16_t end_led_bw_pin)
{
	stepper->end_led_fw_port = end_led_fw_port;
	stepper->end_led_fw_pin = end_led_fw_pin;
	stepper->end_led_bw_port = end_led_bw_port;
	stepper->end_led_bw_pin = end_led_bw_pin;
}


void stepper_tick(Stepper *stepper)
{
	switch(stepper->state)
	{
		case(Wait):
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_SET);
			break;
		case(MoveForward):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			break;
		case(MoveBack):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			break;
		case(MoveTo):
			break;
	}
}
