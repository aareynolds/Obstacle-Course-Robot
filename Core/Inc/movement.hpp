#include "main.h"

enum class ESpeed {
	Slowest = 4,
	Slow = 5,
	Normal = 8,
	Fast = 12,
	Fastest = 15
};

enum class EDirection {
	Reverse = 0,
	Forward = 1
};


class Movement {
public:
	Movement(TIM_HandleTypeDef* in_timer);

	void move_forward(ESpeed speed);
	void move_backward(ESpeed speed);

	void turn_right(ESpeed speed);
	void turn_left(ESpeed speed);

	void move_forward_differential(ESpeed left, ESpeed right);

	void move_stop();

private:

	void move_right_tread(int direction, int speed);
	void stop_right_tread();

	void move_left_tread(int direction, int speed);
	void stop_left_tread();

	TIM_HandleTypeDef* timer;
};
