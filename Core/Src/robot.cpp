#include <cmath>
#include <iostream>

#include "robot.hpp"

/*
 * Compass/magnetometer code is derived from:
 * https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Compass_v2/blob/master/LSM303D.cpp
 */

Robot::Robot(Movement& movement_in, Scanner& scanner_in, I2C_HandleTypeDef* i2c_in)
	: comp_movement(&movement_in), comp_scanner(&scanner_in), i2c(i2c_in) {
	current_state = State::ScanLight;
	rot_offset = 0;

	write_byte(CTRL_REG1, 0x57);
	write_byte(CTRL_REG2, (3 << 6) | (0 << 3));
	write_byte(CTRL_REG3, 0x00);
	write_byte(CTRL_REG4, 0x00);
	write_byte(CTRL_REG5, (4 << 2));
	write_byte(CTRL_REG6, MAG_SCALE_2);
	write_byte(CTRL_REG7, 0x00);
}

void Robot::run() {

	double distance, current_heading, last_dist, current_dist, diff;

while (true) {
	switch (current_state) {

	case State::Idle:
		stop();
		HAL_Delay(5000);

		current_state = State::ScanLight;

		break;

	case State::ScanLight:

		scan_lights();

		if (comp_scanner->brightest_light < 60) {
			if (first) {
				target_heading(heading() + 90, angle_threshold);
				break;
			}

			target_heading(initial_heading, angle_threshold);

			comp_movement->move_backward(ESpeed::Slowest);
			HAL_Delay(1000);
			stop();
			break;
		}

		HAL_Delay(100);

		current_state = State::Orient;
		break;

	case State::Orient:

		if (rotate(rot_offset) == -1) {
			current_state = State::ScanLight;
			break;
		}

		stop();

		if (first) {
			initial_heading = avg_heading(3);
			first = false;
		}

		distance = comp_scanner->scan_object_front();

		if (distance < obj_threshold && comp_scanner->scan_light_front() > lux_stop_threshold) {
			current_state = State::Idle;
			break;
		}

		current_state = State::Forward;
		break;

	case State::Forward:

		if (comp_scanner->scan_object_front() < obj_threshold)  {
			comp_movement->move_backward(ESpeed::Slowest);
			HAL_Delay(1000);
			current_state = State::ScanLight;
			stop();
			break;
		}

		if (move_forward(ESpeed::Slowest, 20000, true) == 1) {
			current_state = State::ScanLight;
		} else {
			if (comp_scanner->scan_light_front() > lux_stop_threshold) {
				current_state = State::Idle;
			}
			else {
				scan_lights();
				if (comp_scanner->brightest_light > 500 && abs(rot_offset) > 7) {
					current_state = State::Orient;
					break;
				}

				if (comp_scanner->scan_object_left() > comp_scanner->scan_object_right()) {
					current_state = State::AvoidLeft;
				} else {
					current_state = State::AvoidRight;
				}
			}
		}

		break;

	case State::AvoidLeft:

		comp_movement->move_backward(ESpeed::Slowest);
		HAL_Delay(200);

		current_heading = avg_heading(3);
		if (target_heading(current_heading - 90, angle_threshold) == -1) {
			stop();
			current_state = State::ScanLight;
			break;
		}

		last_dist = comp_scanner->scan_object_right();
		comp_movement->move_forward(ESpeed::Slowest);

		while (true) {

			if (comp_scanner->scan_object_front() < 25) {
				stop();

				current_state = State::AvoidRight;
				if (target_heading(current_heading, angle_threshold) == -1) {
					current_state = State::ScanLight;
				}
				break;
			}

			current_dist = comp_scanner->scan_object_right();

			if (comp_scanner->scan_light_front() > lux_stop_threshold) {
				stop();
				current_state = State::ScanLight;
				break;
			}

			diff = abs(current_dist - last_dist);
			if (diff > 0.5) {
				if (current_dist < last_dist) {
					target_heading(heading() - 15, angle_threshold);
					last_dist = current_dist;
					comp_movement->move_forward(ESpeed::Slowest);
				}
				else if (current_dist > last_dist && current_dist < 60) {
					target_heading(heading() + 15, angle_threshold);
					last_dist = current_dist;
					comp_movement->move_forward(ESpeed::Slowest);
				}
			}

			if (current_dist > 50) {
				move_forward(ESpeed::Slowest, 500, false);

				target_heading(heading() + 90, angle_threshold);

				move_forward(ESpeed::Slowest, 700, false);
				stop();

				current_state = State::ScanLight;
				break;
			}
		}
		break;

	case State::AvoidRight:

		comp_movement->move_backward(ESpeed::Slowest);
		HAL_Delay(200);

		current_heading = avg_heading(3);
		if (target_heading(current_heading + 90, angle_threshold) == -1) {
			stop();
			current_state = State::ScanLight;
			break;
		}

		double last_dist = comp_scanner->scan_object_left();

		comp_movement->move_forward(ESpeed::Slowest);

		while (true) {

			if (comp_scanner->scan_object_front() < 25) {
				stop();

				current_state = State::AvoidLeft;
				if (target_heading(current_heading, angle_threshold) == -1) {
					current_state = State::ScanLight;
				}
				break;
			}

			current_dist = comp_scanner->scan_object_left();

			if (comp_scanner->scan_light_front() > lux_stop_threshold) {
				stop();
				current_state = State::ScanLight;
				break;
			}

			diff = abs(current_dist - last_dist);
			if (diff > 0.5) {
				if (current_dist < last_dist) {
					target_heading(heading() + 15, angle_threshold);
					last_dist = current_dist;
					comp_movement->move_forward(ESpeed::Slowest);
				}
				else if (current_dist > last_dist && current_dist < 60) {
					target_heading(heading() - 15, angle_threshold);
					last_dist = current_dist;
					comp_movement->move_forward(ESpeed::Slowest);
				}
			}

			if (current_dist > 50) {
				move_forward(ESpeed::Slowest, 500, false);

				target_heading(heading() - 90, angle_threshold);

				move_forward(ESpeed::Slowest, 700, false);
				stop();

				current_state = State::ScanLight;
				break;
			}
		}
		break;
	}
}

}

void Robot::scan_lights() {

	int pwm_position = comp_scanner->scan_lights();

	rot_offset = comp_scanner->pwm_to_rot_offset(pwm_position);

	comp_scanner->sensor_set_position(comp_scanner->sensor_pos_center);

	std::cout << rot_offset << "\r\n";

}

int Robot::move_forward(ESpeed speed, int duration, bool check_light) {
	comp_movement->move_forward(speed);

	comp_scanner->sensor_set_position(comp_scanner->sensor_pos_center);

	int state = 0; // 0 = object, 1 = off course

	double last_left_dist = comp_scanner->scan_object_left();
	double last_right_dist = comp_scanner->scan_object_right();
	double current_alt_dist;

	bool do_left = true;
	timeout.timeout_start(duration);
	while (comp_scanner->scan_object_front() > obj_threshold && !timeout.check_time()) {

		if (check_light && comp_scanner->scan_light_front() < (comp_scanner->brightest_light - 70)) {
			state = 1;
			break;
		}

		if (do_left) {
			current_alt_dist = comp_scanner->scan_object_left();
			if (current_alt_dist < obj_threshold && current_alt_dist < last_left_dist) {
				if (current_alt_dist < 10) {
					break;
				}
				target_heading(heading() + 15, angle_threshold);
				comp_movement->move_forward(speed);
				last_left_dist = current_alt_dist;
			}
		} else {
			current_alt_dist = comp_scanner->scan_object_right();
			if (current_alt_dist < obj_threshold && current_alt_dist < last_right_dist) {
				if (current_alt_dist < 10) {
					break;
				}
				target_heading(heading() - 15, angle_threshold);
				comp_movement->move_forward(speed);
				last_right_dist = current_alt_dist;
			}
		}
		do_left = !do_left;
	}

	stop();

	HAL_Delay(200);

	return state;
}

int Robot::rotate(int degrees) {

	if (degrees < 0) {
		comp_movement->turn_left(ESpeed::Slow);
	}
	else {
		comp_movement->turn_right(ESpeed::Slow);
	}

	double diff = comp_scanner->brightest_light - comp_scanner->scan_light_front();

	timeout.timeout_start(3000);

	while (diff > 15) {
		if (timeout.check_time()) {
			stop();
			return -1;
		}
		diff = comp_scanner->brightest_light - comp_scanner->scan_light_front();
	}

	return 0;
}

void Robot::stop() {
	comp_movement->move_stop();
}

double angle_distance(double a, double b) {
	double angle = abs(a - b) % 360;

	if (angle > 180) {
		angle = 360 - angle;
	}

	int sign = (a - b >= 0 && a - b <= 180) || (a - b <= -180 && a - b >= -360) ? 1 : -1;

	return angle * sign;
}

int Robot::target_heading(int target, float angle_tolerance) {

	target %= 360;

	timeout.timeout_start(5000);

	double current_heading = heading();
	double angle = angle_distance(target, current_heading);
	while (abs(angle) > angle_tolerance) {
		ESpeed speed = ESpeed::Normal;
		if (abs(angle) - angle_tolerance < 2 * angle_tolerance) {
			speed = ESpeed::Slowest;
		}

		if (angle >= 0) {
			comp_movement->turn_right(speed);
		} else {
			comp_movement->turn_left(speed);
		}

		if (timeout.check_time()) {
			stop();
			return -1;
		}

		current_heading = heading();
		angle = angle_distance(target, current_heading);
	}
	stop();
	return 0;
}

void Robot::calibrate_mag() {
	Vector3<int16_t> running_min = {32767, 32767, 32767};
	Vector3<int16_t> running_max = {-32767, -32767, -32767};

	comp_movement->turn_left(ESpeed::Normal);
	for (int i = 0; i < 200; ++i) {
		while(!mag_ready());
		auto mag = get_mag();

		running_min.x = std::min(running_min.x, mag.x);
		running_min.y = std::min(running_min.y, mag.y);
		running_min.z = std::min(running_min.z, mag.z);

		running_max.x = std::max(running_max.x, mag.x);
		running_max.y = std::max(running_max.y, mag.y);
		running_max.z = std::max(running_max.z, mag.z);

		std::cout << i << "/" << 100 << "\r\n";
		std::cout << "Min: " << running_min.x << ", " << running_min.y << ", " << running_min.z << "\r\n";
		std::cout << "Max: " << running_max.x << ", " << running_max.y << ", " << running_max.z << "\r\n";
	}

	stop();

	std::cout << "Min: " << running_min.x << ", " << running_min.y << ", " << running_min.z << "\r\n";
	std::cout << "Max: " << running_max.x << ", " << running_max.y << ", " << running_max.z << "\r\n";

	mag_min = running_min;
	mag_max = running_max;
}

float Robot::avg_heading(int sample_size) {

	double avg_x = 0;
	double avg_y = 0;

	for (int i = 0; i < sample_size; ++i) {
		float current_heading = heading();
		avg_x += cos(current_heading * M_PI/180);
		avg_y += sin(current_heading * M_PI/180);
	}

	avg_x /= sample_size;
	avg_y /= sample_size;

	double avg = (atan2(avg_y, avg_x) * 180) / M_PI;

	if (avg < 0) {
		avg += 360;
	}

	return avg;
}

float Robot::heading() {
	auto accel = get_accel();
	auto temp = get_mag();

	if (temp.x < mag_min.x) {
		temp.x = mag_min.x;
	}
	if (temp.y < mag_min.y) {
		temp.y = mag_min.y;
	}
	if (temp.z < mag_min.z) {
		temp.z = mag_min.z;
	}

	Vector3<float> mag;
	mag.x = 2.0 * ((double)temp.x - mag_min.x) / ((double)mag_max.x - mag_min.x) - 1;
	mag.y = 2.0 * ((double)temp.y - mag_min.y) / ((double)mag_max.y - mag_min.y) - 1;
	mag.z = 2.0 * ((double)temp.z - mag_min.z) / ((double)mag_max.z - mag_min.z) - 1;

	accel.x /= pow(2, 15);
	accel.y /= pow(2, 15);
	accel.z /= pow(2, 15);

	float pitch = asin(-accel.x);
	float roll = asin(accel.y / cos(pitch));

	float xh = mag.x * cos(pitch) + mag.z * sin(pitch);
	float yh = mag.x * sin(roll) * sin(pitch) + mag.y * cos(roll) - mag.z * sin(roll) * cos(pitch);
	float heading = 180 * atan2(yh, xh) / M_PI;

	if (yh >= 0) {
		return heading;
	} else {
		return (360 + heading);
	}
}

bool Robot::mag_ready() {
	char temp = receive_byte(STATUS_REG_M) & 0x03;

	if (temp != 0x03) {
		return false;
	}

	return true;
}

Vector3<int16_t> Robot::get_mag() {

	while(!mag_ready());
	Vector3<int16_t> mag_values;

	mag_values.x = ((int16_t)receive_byte(OUT_X_H_M) << 8) | receive_byte(OUT_X_L_M);
	mag_values.y = ((int16_t)receive_byte(OUT_Y_H_M) << 8) | receive_byte(OUT_Y_L_M);
	mag_values.z = ((int16_t)receive_byte(OUT_Z_H_M) << 8) | receive_byte(OUT_Z_L_M);

	return mag_values;
}

Vector3<int16_t> Robot::get_accel() {

	Vector3<int16_t> accel_values;

	accel_values.x = ((int16_t)receive_byte(OUT_X_H_A) << 8) | receive_byte(OUT_X_L_A);
	accel_values.y = ((int16_t)receive_byte(OUT_Y_H_A) << 8) | receive_byte(OUT_Y_L_A);
	accel_values.z = ((int16_t)receive_byte(OUT_Z_H_A) << 8) | receive_byte(OUT_Z_L_A);

	return accel_values;
}


void Robot::write_byte(uint8_t reg_address, uint8_t data) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(i2c, zumo_addr, reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		std::cout << "Error writing byte to" << reg_address << "\r\n";
	}
}

uint8_t Robot::receive_byte(uint8_t reg_address) {
	uint8_t byte;
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(i2c, zumo_addr, reg_address, I2C_MEMADD_SIZE_8BIT, &byte, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		std::cout << "Error reading byteto " << reg_address << "\r\n";
		return -1;
	}
	return byte;
}
