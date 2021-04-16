#include "main.h"

class Timeout {

public:
	void timeout_start(int duration) {
		time_start = HAL_GetTick();
		while (time_start < 0) {
			time_start = HAL_GetTick();
		}
		timeout_period = duration;
	}

	bool check_time() {
		int current_time = HAL_GetTick();

		if (current_time > time_start && current_time - time_start > timeout_period) {
			return true;
		}
		return false;
	}

private:

	int time_start;
	int timeout_period;

};
