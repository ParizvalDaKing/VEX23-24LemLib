#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

pros::Motor left_front_motor(2, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_motor(5, pros::E_MOTOR_GEARSET_06, true); // port 2, green gearbox, not reversed
pros::Motor right_front_motor(7, pros::E_MOTOR_GEARSET_06, false); // port 3, red gearbox, reversed
pros::Motor right_middle_motor(6,pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_motor(4, pros::E_MOTOR_GEARSET_06, false); // port 4, red gearbox, reversed

pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_back_motor});

pros::Imu imu(21);

lemlib::Drivetrain_t drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    10, // track width
    3.25, // wheel diameter
    450 // wheel rpm
};

lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	chassis.calibrate();
	chassis.setPose(-40, 53, 180);
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
	//intake
	chassis.turnTo(-25, 6, 1000);
	chassis.moveTo(-25, 6, 1000);
	chassis.turnTo(-43, 6, 1000);
	//outtake
	//wait
	//intake
	chassis.turnTo(-11, 2, 1000);
	chassis.moveTo(-11, 2, 1000);
	//stop intake
	//deploy wings
	chassis.turnTo(-42, 5, 1000);
	chassis.moveTo(-42, 5, 1000);
	chassis.moveTo(-38, 5, 1000);
	//retract wings
	//intake
	chassis.turnTo(-10, 21, 1000);
	chassis.moveTo(-10, 21, 1000);
	chassis.turnTo(-45, 59, 1000);
	chassis.moveTo(-45, 59, 1000);
	//deploy wings
	chassis.turnTo(-64, 43, 1000);
	chassis.moveTo(-64, 43, 1000);
	chassis.turnTo(-60, 28, 1000);
	chassis.moveTo(-60, 28, 1000);
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

float defaultDriveCurve(float input, float scale) {
    if (scale != 0) {
        return (powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) *
               input;
    }
    return input;
}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor leftShooterIntake(8, pros::E_MOTOR_GEARSET_06, true);
	pros::Motor rightShooterIntake(9, pros::E_MOTOR_GEARSET_06);
	pros::Motor_Group ShooterIntake({leftShooterIntake, rightShooterIntake});
	pros::ADIDigitalOut wings ('A');

	float leftY = 0;
	float rightX = 0;

	// float left_raw = 0;
	// float right_raw = 0;

	// float right_stick_smoothed = 0;
	// float left_stick_smoothed = 0;
	// float left_stick_prev = 0;
	// float right_stick_prev = 0;

	// int deadzone = 5;

	// left_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	// right_side_motors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	while (true) {
		leftY = master.get_analog(ANALOG_LEFT_Y);
		rightX = master.get_analog(ANALOG_RIGHT_X);

		// left_raw = -leftY - rightX;
		// right_raw = -leftY + rightX;

		//drift reduction

		//smoothing

		// right_stick_smoothed = (right_raw * 0.03) + (right_stick_prev * 0.97);
		// left_stick_smoothed =  (left_raw * 0.03) + (left_stick_prev * 0.97);

		// right_stick_prev = right_stick_smoothed;
		// left_stick_prev = left_stick_smoothed;


		//end of smoothing

		// if(abs(leftY) < deadzone) {
		// 	leftY = 0;
		// 	left_side_motors.brake();
		// }
		// if(abs(rightX) < deadzone) {
		// 	rightX = 0;
		// 	right_side_motors.brake();
		// }

		//end of drift reduction

		left_side_motors = defaultDriveCurve(leftY + rightX, 4);
		right_side_motors = defaultDriveCurve(leftY - rightX, 4);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			ShooterIntake = 90;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			ShooterIntake = -90;
		}
		else {
			ShooterIntake = 0;
		}

		pros::delay(20);
	}
}
