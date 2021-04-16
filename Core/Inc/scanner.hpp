#include <vector>

#include "main.h"

class Scanner {

public:
	Scanner(TIM_HandleTypeDef* htim_in, ADC_HandleTypeDef* hadc_in, double* distance_in, uint32_t* lux_in);

	double convert_lux(uint32_t adc);

	int scan_lights();

	int scan_objects();

	double scan_object_front();
	double scan_light_front();
	double scan_light_left();
	double scan_light_right();

	double scan_object_left();
	double scan_object_right();

	void sensor_set_position(int position);

	int pwm_to_rot_offset(int pwm);

	const int sensor_pos_center = 1300;

	double brightest_light;

private:

	double scan_object_arb(int pwm);
	double read_lux();

	TIM_HandleTypeDef* htim;
	ADC_HandleTypeDef* hadc;

	double* US_Distance;
	uint32_t* Lux;

	const int sensor_pos_min = 500;
	const int sensor_pos_max = 2300;
};
