#include "main.h"
#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>

class ControllerButtonHandler {
	private:
		Controller *controller;
		ControllerDigital button;
		bool wasPressed;

	public:
		ControllerButtonHandler(Controller *controller, ControllerDigital button) {
			this->controller = controller;
			this->button = button;

			this->wasPressed = false;
		}

		bool update() {
			Controller controller = *this->controller;

			bool pressed = controller.getDigital(button);
			bool status = pressed && !wasPressed;
			wasPressed = pressed;

			return status;
		}
};

template<typename T>
double getAverage(std::vector<T> const& v) {
    if (v.empty()) {
        return 0;
    }

    return std::reduce(v.begin(), v.end(), 0.0) / v.size();
}

class MotorHoldOnStop {
	private:
		Motor *motor;
		std::vector<int> positionHistory;
		int lastTargetPosition;
		int lastPosition;

	public:
		MotorHoldOnStop(Motor *motor) {
			this->motor = motor;
		}

		void update() {
			Motor motor = *this->motor;

			if(motor.getTargetPosition() != this->lastTargetPosition) {
				this->positionHistory.clear();
				this->lastTargetPosition = motor.getTargetPosition();
				return;
			}

			this->positionHistory.push_back(std::abs(lastPosition - motor.getPosition()));

			if(this->positionHistory.size() < 100) {
				return;
			} else if(this->positionHistory.size() > 500) {
				this->positionHistory.erase(this->positionHistory.begin(), this->positionHistory.begin() + (this->positionHistory.size() - 500));
			}

			int average_change = getAverage(this->positionHistory);


		}
};
