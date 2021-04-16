#include "movement.hpp"


Movement::Movement(TIM_HandleTypeDef* in_timer) : timer(in_timer) {

}

/////////////
// Public: //
/////////////

void Movement::move_forward(ESpeed speed) {

	move_right_tread(int(EDirection::Forward), int(speed));
	move_left_tread(int(EDirection::Forward), int(speed));
}

void Movement::move_backward(ESpeed speed) {

	move_right_tread(int(EDirection::Reverse), int(speed));
	move_left_tread(int(EDirection::Reverse), int(speed));
}

void Movement::turn_right(ESpeed speed) {
	move_right_tread(int(EDirection::Reverse), int(speed));
	move_left_tread(int(EDirection::Forward), int(speed));
}

void Movement::turn_left(ESpeed speed) {
	move_right_tread(int(EDirection::Forward), int(speed));
	move_left_tread(int(EDirection::Reverse), int(speed));
}

void Movement::move_forward_differential(ESpeed left, ESpeed right) {
	move_right_tread(int(EDirection::Forward), int(right));
	move_left_tread(int(EDirection::Forward), int(left));
}

void Movement::move_stop() {
	stop_right_tread();
	stop_left_tread();
}

/////////////
// Private //
/////////////

void Movement::move_right_tread(int direction, int speed) {
	if (direction == 0) {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_4, int(speed));
}

void Movement::move_left_tread(int direction, int speed) {
	if (direction == 0) {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, int(speed));
}

void Movement::stop_right_tread() {
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_4, 0);
}

void Movement::stop_left_tread() {
	__HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, 0);
}
