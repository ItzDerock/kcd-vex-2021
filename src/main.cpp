#include "main.h"

#define PNEUMATICS 'A'

#define BAR_LIFT_LEFT 20
#define BAR_LIFT_RIGHT 19
#define BAR_LIFT_GRABBER 17

#define DRIVE_TRAIN_LEFT 1
#define DRIVE_TRAIN_RIGHT -2

#define RIGHT_GRABBER -14
#define BACK_GRABBER -11

class ControllerButtonHandler {
	// create a class that takes in controller, a ControllerDigital button
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
			// Get the controller
			Controller controller = *this->controller;

			bool pressed = controller.getDigital(button);
			bool status = pressed && !wasPressed;
			wasPressed = pressed;

			return status;
		}
};

std::shared_ptr<ChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(DRIVE_TRAIN_LEFT, DRIVE_TRAIN_RIGHT)
		.withDimensions(AbstractMotor::gearset::red, {{4_in, 11_in}, imev5RedTPR})
		.build();

Controller controller;

MotorGroup barLift({
	Motor(BAR_LIFT_LEFT, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees),
	Motor(BAR_LIFT_RIGHT, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees)
});

std::string prd(const double x, const int decDigits) {
    std::stringstream ss;
    ss << std::fixed;
    ss.precision(decDigits); // set # places after decimal
    ss << x;
    return ss.str();
}

bool piston_extended = false;
bool back_piston_extended = false;
bool barliftGrabber_extended = false;
bool isDriveHoldMode = false;

Motor grabber_mtr(RIGHT_GRABBER);
Motor barliftGrabber(BAR_LIFT_GRABBER);
Motor backGrabber(BACK_GRABBER);
// TODO: Move this to auto start!
// grabber_mtr.moveRelative(-0.5, 1000);
// grabber_mtr.tarePosition();

void toggleBackGrabber() {
	back_piston_extended = !back_piston_extended;
	backGrabber.moveAbsolute(back_piston_extended ? -0.25 : 1.5, 1000);
}

void toggleFrontGrabber() {
	barliftGrabber_extended = !barliftGrabber_extended;
	barliftGrabber.moveAbsolute(barliftGrabber_extended ? 0.25 : 0, 1000);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "[!] Ready. Waiting for instructions...");

	barLift.setBrakeMode(AbstractMotor::brakeMode::hold);
	// chassis->getModel()->setMaxVoltage(12000);

	pros::ADIDigitalOut piston (PNEUMATICS);
	piston.set_value(false);
	piston_extended = false;

	// barliftGrabber.moveAbsolute(-0.25, 1000);
	// barliftGrabber.tarePosition();

	barliftGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	barliftGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
	backGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	backGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
	grabber_mtr.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	grabber_mtr.setBrakeMode(AbstractMotor::brakeMode::hold);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::lcd::set_text(0, "[!] Autonomous");

	toggleBackGrabber();
	toggleBackGrabber();

	chassis->setMaxVelocity(50);
	chassis->moveDistance(-93_cm);
	toggleFrontGrabber();
	chassis->getModel()->right(35);
	chassis->getModel()->left(-35);
	pros::delay(700);
	chassis->getModel()->stop();
	chassis->getModel()->right(-35);
	chassis->getModel()->left(35);
	pros::delay(700);
	toggleBackGrabber();
	chassis->moveDistance(40_cm);
	chassis->turnAngle(-45_deg);
	chassis->moveDistance(20_cm);
	toggleFrontGrabber();
	barLift.moveVelocity(20);
	pros::delay(400);
	barLift.moveVelocity(0);
	chassis->turnAngle(45_deg);
	chassis->moveDistance(33_cm);
	chassis->turnAngle(90_deg);
	barLift.moveVelocity(-20);
	pros::delay(400);
	barLift.moveVelocity(0);
	chassis->moveDistance(100_cm);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::lcd::set_text(0, "[!] Driver Control");

	auto XPressed = ControllerButtonHandler(&controller, ControllerDigital::X);
	auto APressed = ControllerButtonHandler(&controller, ControllerDigital::A);
	auto BPressed = ControllerButtonHandler(&controller, ControllerDigital::B);
	auto DownPressed = ControllerButtonHandler(&controller, ControllerDigital::down);

	while (true) {
		auto leftMoveStick = controller.getAnalog(ControllerAnalog::leftY);
		auto rightMoveStick = controller.getAnalog(ControllerAnalog::rightY);

		//if(leftMoveStick == 0 && rightMoveStick == 0) {
		//	chassis->getModel()->stop();
		//} else {
		chassis->getModel()->tank(
			leftMoveStick,
			rightMoveStick
		);
		//}

		pros::lcd::set_text(2, "Left: " + prd(leftMoveStick, 2) + " | Right: " + prd(rightMoveStick, 2));

		bool l1Pressed = controller.getDigital(ControllerDigital::L1);
		bool l2Pressed = controller.getDigital(ControllerDigital::R1);

		if((!l1Pressed && !l2Pressed) || (l1Pressed && l2Pressed)) {
			barLift.moveVelocity(0);
			pros::lcd::set_text(1, "Lift Stopped");
		} else {
			pros::lcd::set_text(1, l1Pressed ? "Moving Lift Up" : "Moving Lift Down");
			barLift.moveVelocity(l1Pressed ? +20 : -20);
		}

		if(DownPressed.update()) {
			isDriveHoldMode = !isDriveHoldMode;
			if(isDriveHoldMode) {
				chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::hold);
			} else {
				chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
			}
			pros::lcd::set_text(4, isDriveHoldMode ? "Drive Train in BREAK mode" : "Drive Train in HOLD mode");
		}

		if(APressed.update()) {
			piston_extended = !piston_extended;
			grabber_mtr.moveAbsolute(piston_extended ? 0.5 : 1.5, 1000);
		}

		if(BPressed.update()) {
			toggleBackGrabber();
		}

		if(XPressed.update()) {
			toggleFrontGrabber();
		}

		pros::delay(10);
	}
}
