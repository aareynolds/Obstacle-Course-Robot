#include <math.h>
#include <cassert>
#include <iostream>

#include "scanner.hpp"

Scanner::Scanner(TIM_HandleTypeDef* htim_in, ADC_HandleTypeDef* hadc_in, double* distance_in, uint32_t* lux_in)
		: htim(htim_in), hadc(hadc_in), US_Distance(distance_in), Lux(lux_in) {
	brightest_light = 0;
}

double Scanner::convert_lux(uint32_t adc) {
	int Eref = 1000; // lux
	int Rf = 100000; // Ohms
	double Isc = 27e-6; // Amps
	double Vref = 3.3; // Volts
	int bits = 4096; // Bits (2^12)

	return ((adc * Eref * Vref) / (bits * Isc)) / Rf;
}

double Scanner::read_lux() {
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 0xFFFFFFFF);
	return convert_lux(HAL_ADC_GetValue(hadc));
}

void Scanner::sensor_set_position(int position) {
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, position);
}

int Scanner::pwm_to_rot_offset(int pwm) {
	int range = sensor_pos_max - sensor_pos_min;

	double degrees = (((double)(pwm - sensor_pos_min) / (double)range) - 0.5) * -180;

	return (int)degrees;
}

/*
 * Scanner::scan_lights
 *
 * Light sensor is rotated in two passes of 180 degrees
 * ADC polls for the current lux value at every half-degree of rotation
 * Returns the PWM position value of the brightest light
 *
 */
int Scanner::scan_lights() {
	sensor_set_position(sensor_pos_min);
	HAL_Delay(750);

	int num_readings = 360; // For one pass
	int mod_factor = (sensor_pos_max - sensor_pos_min) / num_readings;

	double max_lux = 0;
	int position = 0;

	std::vector<double> data(sensor_pos_max - sensor_pos_min);

	// Perform first pass
	for (int i = sensor_pos_min; i < sensor_pos_max; i++) {
	  sensor_set_position(i);

	  // Populate vector with lux values from scan
	  if (i % mod_factor == 0) {
		  data[(i - sensor_pos_min) / mod_factor] = read_lux();
	  }
	}

	// Perform second pass
	for (int i = sensor_pos_max - 1; i >= sensor_pos_min; i--) {
	  sensor_set_position(i);

	  // Populate vector with lux values from scan
	  if (i % mod_factor == 0) {

		  auto& val = data[(i - sensor_pos_min) / mod_factor];

		  // Find the average of the two passes

		  double lux = read_lux();

		  if (abs(lux - val) > 50) {
			  continue;
		  }

		  val = (val + lux) / 2;

		  if (val > max_lux) {
			  max_lux = val;
			  position = i;
		  }
	  }
	}

	brightest_light = max_lux;
	return position;
}

/*
 * Scanner::scan_objects
 *
 * Ultrasonic sensor is rotated 180 degrees
 * Takes a reading from the sensor every 10 degrees
 * Returns the PWM position value of the closest object
 *
 */
int Scanner::scan_objects() {
	double distance = 9999999;
	int position = 0;

	int num_readings = 18;
	int mod_factor = (sensor_pos_max - sensor_pos_min) / num_readings;

	sensor_set_position(sensor_pos_min);
	HAL_Delay(750);

	NVIC_EnableIRQ(TIM2_IRQn);
	std::vector<double> data(sensor_pos_max - sensor_pos_min);
	for (int i = sensor_pos_min; i < sensor_pos_max; i++) {
	  *US_Distance = -1;
	  sensor_set_position(i);

	  if (i % mod_factor == 0) {
		  // Wait until the interrupt updates the value
		  while (*US_Distance == -1);
		  data[(i - sensor_pos_min) / mod_factor] = *US_Distance;
	  }
	}
	for (int i = sensor_pos_max - 1; i >= sensor_pos_min; i--) {
	  *US_Distance = -1;
	  sensor_set_position(i);

	  if (i % mod_factor == 0) {
		  // Wait until the interrupt updates the value
		  while (*US_Distance == -1);

		  auto& val = data[(i - sensor_pos_min) / mod_factor];
		  val = (val + *US_Distance) / 2;

		  if (val < distance) {
			  distance = val;
			  position = i;
		  }
	  }
	}
	NVIC_DisableIRQ(TIM2_IRQn);

	return position;
}

double Scanner::scan_object_arb(int pwm) {
	sensor_set_position(pwm);
	HAL_Delay(500);

	double distance = 0;

	NVIC_EnableIRQ(TIM2_IRQn);

	int num_samples = 2;

	for (int i = 0; i < num_samples; ++i) {
		*US_Distance = -1;

		while (*US_Distance == -1);

		distance += *US_Distance;
	}

	NVIC_DisableIRQ(TIM2_IRQn);

	return distance / num_samples;
}

double Scanner::scan_object_front() {
	return scan_object_arb(sensor_pos_center);
}

double Scanner::scan_object_left() {
	return scan_object_arb(sensor_pos_max);
}

double Scanner::scan_object_right() {
	return scan_object_arb(sensor_pos_min);
}

double Scanner::scan_light_front() {

	double average = 0;
	int num_samples = 5;

	for (int i = 0; i < num_samples; ++i) {
		average += read_lux();
	}

	return average / num_samples;
}
