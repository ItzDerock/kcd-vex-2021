#include "main.h"
#include "utils.cpp"

#define PNEUMATICS 'A'

#define BAR_LIFT_LEFT 20
#define BAR_LIFT_RIGHT 19
#define BAR_LIFT_GRABBER 9

#define DRIVE_TRAIN_LEFT -1
#define DRIVE_TRAIN_RIGHT 2
#define DRIVE_TRAIN_LEFT_BACK -4
#define DRIVE_TRAIN_RIGHT_BACK 5

#define RIGHT_GRABBER -14
// START FULLY FORWARDS HITTING RUBBER BANDS
#define BACK_GRABBER -11

#define MIDDLE_GRABBER 16

std::string prd(const double x, const int decDigits) {
    std::stringstream ss;
    ss << std::fixed;
    ss.precision(decDigits); // set # places after decimal
    ss << x;
    return ss.str();
}


std::shared_ptr<ChassisController> chassis =
	ChassisControllerBuilder()
		.withMotors(
      DRIVE_TRAIN_LEFT,
      DRIVE_TRAIN_RIGHT,
      DRIVE_TRAIN_RIGHT_BACK,
      DRIVE_TRAIN_LEFT_BACK
    )
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 11_in}, imev5GreenTPR})
		.build();

Controller controller;

Motor _barLiftLeft(BAR_LIFT_LEFT, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor _barLiftRight(BAR_LIFT_RIGHT, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

MotorGroup barLift({
	_barLiftLeft,
	_barLiftRight
});

bool piston_extended = false;
bool back_piston_extended = false;
bool barliftGrabber_extended = false;
bool middleGrabber_extended = false;
bool isDriveHoldMode = false;

Motor rightGrabber(RIGHT_GRABBER);
Motor barliftGrabber(BAR_LIFT_GRABBER);
Motor backGrabber(BACK_GRABBER);
Motor middleGrabber(MIDDLE_GRABBER);

pros::ADIDigitalOut piston (PNEUMATICS);
// TODO: Move this to auto start!
// rightGrabber.moveRelative(-0.5, 1000);
// rightGrabber.tarePosition();

void checkMotorTemp(Motor motor, int line) {
	if(motor.getTemperature() > 50) {
		controller.rumble("---");
		pros::lcd::set_text(4, std::to_string(motor.getTemperature()) + " on " + std::to_string(motor.getPort()));
	}
}

void toggleBackGrabber(int velocity = 1000) {
	back_piston_extended = !back_piston_extended;
	backGrabber.moveAbsolute(back_piston_extended ? -0.5 : 1.5, velocity);
}

void toggleFrontGrabber() {
	// piston.set_value(!piston.get_value());
	barliftGrabber_extended = !barliftGrabber_extended;
	piston.set_value(barliftGrabber_extended);
	// barliftGrabber.moveAbsolute(barliftGrabber_extended ? 0.25 : 0, 1000);
}

void toggleMiddleGrabber() {
	middleGrabber_extended = !middleGrabber_extended;
	middleGrabber.moveAbsolute(middleGrabber_extended ? -1 : -0.4, 1000);
}

void waitForMotorToStop(Motor motor, int timeout = -1) {
  int end = timeout > 0 ? (pros::millis() + timeout) : -1;

	while(abs(motor.getPosition() - motor.getTargetPosition()) > 0.2) {
		pros::delay(20);

    if(timeout > 0) {
      if(pros::millis() > end) {
        break;
      }
    }
	}
}

bool doAuto = true;

void autoToggle() {
  doAuto = !doAuto;
  pros::lcd::set_text(0, doAuto ? "[!] Auto enabled" : "[!] Auto disabled");
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

  pros::lcd::register_btn0_cb(autonomous);
  pros::lcd::register_btn1_cb(autoToggle);

	barLift.setBrakeMode(AbstractMotor::brakeMode::hold);
	chassis->setMaxVelocity(200);
	// chassis->getModel()->setMaxVoltage(12000);

	piston.set_value(false);

	// barliftGrabber.moveAbsolute(-0.25, 1000);
	// barliftGrabber.tarePosition();

	barliftGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	barliftGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
	backGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	backGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
	rightGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	rightGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
	middleGrabber.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
	middleGrabber.setBrakeMode(AbstractMotor::brakeMode::hold);
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
	pros::lcd::set_text(0, "[!] Autonomous Running.");

	// Intialize
	backGrabber.moveAbsolute(1, 1000);
	waitForMotorToStop(backGrabber);
	backGrabber.tarePosition();

  toggleBackGrabber();
	toggleBackGrabber();

  if(!doAuto) {
    pros::lcd::set_text(0, "[!] Autonomous disabled!");
    return;
  }

 	// TODO: Uncomment when its back
	// rightGrabber.moveAbsolute(0.5, 1000);
	// waitForMotorToStop(backGrabber);
	// rightGrabber.tarePosition();

  chassis->setMaxVelocity(600);
  // chassis->moveDistance(-155_cm);
  // chassis->moveDistance(-175_cm);
  chassis->moveDistance(-20_cm);
  barLift.moveRelative(200, 1000);
  chassis->moveDistance(-106_cm);
  chassis->turnAngle(-90_deg);
  chassis->moveDistance(-80_cm);

  chassis->setMaxVelocity(50);
  chassis->moveDistance(-50_cm);

  chassis->turnAngle(-60_deg);

  chassis->setMaxVelocity(600);

  toggleBackGrabber(100);
  waitForMotorToStop(backGrabber, 1000);

  chassis->turnAngle(30_deg);

  // chassis->moveDistance(34_cm);
  chassis->moveDistance(130_cm);
  chassis->turnAngle(-260_deg);
  chassis->moveDistance(-50_cm);

  pros::lcd::set_text(0, "[!] Autonomous Done");

	// // Extend back grabber to pickup back ring

	// // TODO: REMOVE AFTER testing
	// // Set the velocity to 50% so I can easily see what's happening.
	// chassis->setMaxVelocity(maxVelocity);
  //
	// // Move backwards to the first ring.
	// chassis->moveDistance(-94_cm);
  //
	// // Fully extend back grabber.
	// toggleFrontGrabber();
  //
	// // Turn a little to wiggle the ring in.
	// chassis->setMaxVelocity(20);
	// chassis->moveDistanceAsync(-10_cm);
	// chassis->setMaxVelocity(maxVelocity);
	// chassis->getModel()->right(10);
	// chassis->getModel()->left(-10);
	// pros::delay(700);
	// chassis->getModel()->stop();
	// pros::delay(300);
  //
	// // Grab it.
	// toggleBackGrabber();
  //
	// // Move to the other ring.
	// chassis->moveDistance(40_cm);
	// chassis->turnAngle(-45_deg);
	// chassis->moveDistance(20_cm);
	// toggleFrontGrabber();
	// barLift.moveVelocity(20);
	// pros::delay(400);
	// barLift.moveVelocity(0);
	// chassis->turnAngle(45_deg);
	// chassis->moveDistance(33_cm);
	// chassis->turnAngle(90_deg);
	// barLift.moveVelocity(-20);
	// pros::delay(400);
	// barLift.moveVelocity(0);
	// chassis->moveDistance(100_cm);
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
	auto UpPressed = ControllerButtonHandler(&controller, ControllerDigital::up);

  // auto backGrabberStopper = MotorHoldOnStop(&backGrabber);

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
		bool l2Pressed = controller.getDigital(ControllerDigital::L2);

		if((!l1Pressed && !l2Pressed) || (l1Pressed && l2Pressed)) {
			barLift.moveVelocity(0);
			pros::lcd::set_text(1, "Lift Stopped");
		} else {
			pros::lcd::set_text(1, l1Pressed ? "Moving Lift Up" : "Moving Lift Down");
			barLift.moveVelocity(l1Pressed ? +60 : -60);
		}

		bool r1Pressed = controller.getDigital(ControllerDigital::R1);
		bool r2Pressed = controller.getDigital(ControllerDigital::R2);

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
			rightGrabber.moveAbsolute(piston_extended ? 0.5 : 1.5, 1000);
		}

		if(BPressed.update()) {
			toggleBackGrabber();
		}

		if(XPressed.update()) {
			toggleFrontGrabber();
		}

		if(UpPressed.update()) {
			toggleMiddleGrabber();
		}

    // backGrabberStopper.update();

		// int motorTempLine = -1;
		// checkMotorTemp(rightGrabber, ++motorTempLine);
		// checkMotorTemp(backGrabber, ++motorTempLine);
		// checkMotorTemp(rightGrabber, ++motorTempLine);
		// checkMotorTemp(_barLiftLeft, ++motorTempLine);
		// checkMotorTemp(_barLiftRight, ++motorTempLine);
		// checkMotorTemp(barliftGrabber, ++motorTempLine);

		pros::delay(10);
	}
}
