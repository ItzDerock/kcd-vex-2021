#include "main.h"

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
