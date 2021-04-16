#include <unordered_map>
#include <cmath>
#include <vector>

#include "main.h"

#include "movement.hpp"
#include "scanner.hpp"
#include "timeout.hpp"

#define _USE_MAT_DEFINES

#define STATUS_REG_M 0x07

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25
#define CTRL_REG7 0x26

#define OUT_X_L_A 		0x28
#define OUT_X_H_A 		0x29
#define OUT_Y_L_A 		0x2A
#define OUT_Y_H_A 		0x2B
#define OUT_Z_L_A 		0x2C
#define OUT_Z_H_A 		0x2D

#define MAG_SCALE_2 0x00

#define OUT_X_L_M 		0x08
#define OUT_X_H_M 		0x09
#define OUT_Y_L_M 		0x0A
#define OUT_Y_H_M 		0x0B
#define OUT_Z_L_M 		0x0C
#define OUT_Z_H_M 		0x0D

template <typename T>
struct Vector3 {
	T x;
	T y;
	T z;
};

enum class State {
	Idle,
	ScanLight,
	Orient,
	Forward,
	AvoidLeft,
	AvoidRight
};

class Robot {

public:

	Robot(Movement& movement_in, Scanner& scanner_in, I2C_HandleTypeDef* i2c_in);

	void run();

	void scan_lights();

	int rotate(int degrees);

	int move_forward(ESpeed speed, int duration, bool check_light);

	void stop();

	void calibrate_mag();

	float heading();
	float avg_heading(int sample_size);

	int target_heading(int target, float angle_tolerance);

private:

	void write_byte(uint8_t reg_address, uint8_t data);
	uint8_t receive_byte(uint8_t reg_address);

	bool mag_ready();
	Vector3<int16_t> get_mag();
	Vector3<int16_t> get_accel();

	Vector3<int16_t> mag_min = {2339, -2273, -32768};
	Vector3<int16_t> mag_max = {10238, 7492, 32767};

	State current_state;

	Movement* comp_movement;
	Scanner* comp_scanner;

	I2C_HandleTypeDef* i2c;

	Timeout timeout;

	int rot_offset;

	double initial_heading;

	const double lux_stop_threshold = 600;
	const int obj_threshold = 30;
	const int angle_threshold = 7;

	bool first = true;

	const uint8_t zumo_addr = 0b0011101 << 1;

};
