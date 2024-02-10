#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <string>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

pros::Motor left_front_motor(2, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor left_middle_motor(19, pros::E_MOTOR_GEARSET_06, true);
pros::Motor left_back_motor(5, pros::E_MOTOR_GEARSET_06, true); // port 2, green gearbox, not reversed
pros::Motor right_front_motor(7, pros::E_MOTOR_GEARSET_06, false); // port 3, red gearbox, reversed
pros::Motor right_middle_motor(6,pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back_motor(4, pros::E_MOTOR_GEARSET_06, false); // port 4, red gearbox, reversed

pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_back_motor});
pros::Motor leftShooterIntake(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightShooterIntake(9, pros::E_MOTOR_GEARSET_06);
pros::Motor_Group ShooterIntake({leftShooterIntake, rightShooterIntake});
pros::ADIDigitalOut wings ('H');
pros::ADIDigitalOut lift ('G');
pros::ADIDigitalOut pto ('F');
pros::ADIAnalogIn light('C');
pros::ADIDigitalIn limit('B');


//pros::Rotation rot(19, false);
pros::Imu imu(11);

//lemlib::TrackingWheel horizontal_tracking_wheel(&rot, 2.75, 4.3, 1);

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
	6.83, // kP
    29.2, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500 // largeErrorTimeout
};
 
// turning PID
lemlib::ChassisController_t angularController {
    1.9, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500 // largeErrorTimeout
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

int autonIndex = 2;
std::string autoNames[4] = {"one-ball", "homeside", "visitorside", "vistor-one-ball"};

void on_left_button() {
	if (autonIndex > 0) {
		autonIndex--;
		pros::lcd::set_text(0, autoNames[autonIndex] + " " + std::to_string(autonIndex));
		switch(autonIndex) {
			case 0:
				chassis.setPose(-40, 53, 180);
				break;
			case 1:
				chassis.setPose(-40, 53, 180);
				break;
			case 2:
				chassis.setPose(-42, -51, 180);
				break;
			case 3:
				chassis.setPose(-43, 50, 0);
			default:
				chassis.setPose(-40, 53, 180);
				break;
		}
	}
	else {
		autonIndex = 3;
		pros::lcd::set_text(0, autoNames[autonIndex] + " " + std::to_string(autonIndex));
		switch(autonIndex) {
			case 0:
				chassis.setPose(-40, 53, 180);
				break;
			case 1:
				chassis.setPose(-40, 53, 180);
				break;
			case 2:
				chassis.setPose(-42, -51, 180);
				break;
			case 3:
				chassis.setPose(-43, 50, 0);
			default:
				chassis.setPose(-40, 53, 180);
				break;
		}
	}

}
void on_right_button() {
	if (autonIndex < 3) {
		autonIndex++;
		pros::lcd::set_text(0, autoNames[autonIndex] + " " + std::to_string(autonIndex));
		switch(autonIndex) {
			case 0:
				chassis.setPose(-40, 53, 180);
				break;
			case 1:
				chassis.setPose(-40, 53, 180);
				break;
			case 2:
				chassis.setPose(-42, -51, 180);
				break;
			case 3:
				chassis.setPose(-43, 50, 0);
			default:
				chassis.setPose(-40, 53, 180);
				break;
		}
	}
	else {
		autonIndex = 0;
		pros::lcd::set_text(0, autoNames[autonIndex] + " " + std::to_string(autonIndex));
		switch(autonIndex) {
			case 0:
				chassis.setPose(-40, 53, 180);
				break;
			case 1:
				chassis.setPose(-40, 53, 180);
				break;
			case 2:
				chassis.setPose(-42, -51, 180);
				break;
			case 3:
				chassis.setPose(-43, 50, 0);
			default:
				chassis.setPose(-40, 53, 180);
				break;
		}
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(1, "x: %f", pose.x); // print the x position
        pros::lcd::print(2, "y: %f", pose.y); // print the y position
        pros::lcd::print(3, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}
 
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, autoNames[autonIndex] + " " + std::to_string(autonIndex));

	pros::Task screenTask(screen);

	chassis.calibrate();
	//homeside pose
	//chassis.setPose(-40, 53, 180);
	//visitorside pose
	//chassis.setPose(-42, -51, 0);

	// switch(autonIndex) {
	// 		case 0:
	// 			chassis.setPose(-40, 53, 180);
	// 			break;
	// 		case 1:
	// 			chassis.setPose(-40, 53, 180);
	// 			break;
	// 		case 2:
				chassis.setPose(-42, -51, 180);
		// 		break;
		// 	case 3:
		// 		chassis.setPose(-43, 50, 0);
		// 	default:
		// 		chassis.setPose(-40, 53, 180);
		// 		break;
		// } 
	//pros::lcd::register_btn0_cb(on_left_button);
	//pros::lcd::register_btn2_cb(on_right_button);

	//homeside pose
	//chassis.setPose(-40, 53, 180);
	//visitorside pose
	//chassis.setPose(-42, -51, 0);
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
void competition_initialize() {
	//pros::lcd::register_btn0_cb(on_left_button);
	//pros::lcd::register_btn2_cb(on_right_button);
}

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
	//uncomment pose

	//one-ball
	
	// chassis.turnTo(-25, 6, 1000, false, 50);
	// chassis.moveTo(-25, 6, 1000, 100);
	// chassis.turnTo(-44, 5, 1000, false, 50);
	// chassis.moveTo(-50, 5, 1000, 100);
	// ShooterIntake = -90;
	// pros::delay(1500);
	// ShooterIntake = 0;
	//end 

	//homeside

	// //intake
	// ShooterIntake = 90;
	// chassis.turnTo(-25, 6, 1000, false, 50);
	// chassis.moveTo(-25, 6, 1000, 100);
	// chassis.turnTo(-43, 6, 1000, false, 50);
	// pros::delay(200);
	// //outtake
	// ShooterIntake = -90;
	// //wait
	// pros::delay(200);
	// //intake
	// ShooterIntake = 90;
	// chassis.turnTo(-11, 2, 1000, false, 50);
	// chassis.moveTo(-11, 2, 1000, 100);
	// //stop intake
	// ShooterIntake = 0;
	// //deploy wings
	// chassis.turnTo(-42, 5, 1000, false, 50);
	// wings.set_value(true);
	// chassis.moveTo(-48, 5, 1000, 100);
	// //chassis.moveTo(-38, 5, 1000, 100);
	// //retract wings
	// wings.set_value(false);
	// //intake
	// ShooterIntake = 90;
	// chassis.turnTo(-10, 21, 1000, false, 50);
	// chassis.moveTo(-10, 21, 1000, 100);
	// chassis.turnTo(-45, 59, 1000, false, 50);
	// chassis.moveTo(-45, 59, 1000, 100);
	// ShooterIntake = 0;
	// //deploy wings
	// wings.set_value(true);
	// chassis.turnTo(-64, 43, 1000, false, 50);
	// chassis.moveTo(-64, 43, 1000, 100);
	// chassis.turnTo(-60, 22, 1000, false, 50);
	// chassis.moveTo(-60, 22, 1000, 100);

	//end

	//vistor side

	// //intake
	// ShooterIntake = 90;
	// chassis.turnTo(-24, -5, 1000, false, 50);
	// chassis.moveTo(-24, -5, 1000, 100);
	// //stop intake
	// pros::delay(200);
	// ShooterIntake = 0;
	// //deploy wings
	// chassis.turnTo(-50, -59, 1000, false, 50);
	// chassis.moveTo(-50, -59.6, 1000, 100);
	// wings.set_value(true);
	// chassis.turnTo(-6, -52, 1000, false, 50);
	// wings.set_value(false);
	// chassis.moveTo(-6, -52, 1000, 100);
	// //outtake
	// ShooterIntake = -90;

	//end
	// switch(autonIndex) {
	// 	case 0:
	// 	chassis.turnTo(-25, 6, 1000, false, 50);
	// 	chassis.moveTo(-25, 6, 1000, 100);
	// 	chassis.turnTo(-44, 5, 1000, false, 50);
	// 	chassis.moveTo(-50, 5, 1000, 100);
	// 	ShooterIntake = -90;
	// 	pros::delay(1500);
	// 	ShooterIntake = 0;
	// 		break;
	// 	case 1:
	// 	//intake
	// 	chassis.setPose(-40, 53, 180);
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-25, 6, 1000, false, 50);
	// 	chassis.moveTo(-25, 6, 1000, 100);
	// 	chassis.turnTo(-43, 6, 1000, false, 50);
	// 	pros::delay(200);
	// 	//outtake
	// 	ShooterIntake = -90;
	// 	//wait
	// 	pros::delay(200);
	// 	//intake
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-11, 2, 1000, false, 50);
	// 	chassis.moveTo(-11, 2, 1000, 100);
	// 	//stop intake
	// 	ShooterIntake = 0;
	// 	//deploy wings
	// 	chassis.turnTo(-42, 5, 1000, false, 50);
	// 	//wings.set_value(true);
	// 	chassis.moveTo(-48, 5, 1000, 100);
	// 	//chassis.moveTo(-38, 5, 1000, 100);
	// 	//retract wings
	// 	//wings.set_value(false);
	// 	//intake
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-10, 21, 1000, false, 50);
	// 	chassis.moveTo(-10, 21, 1000, 100);
	// 	chassis.turnTo(-45, 59, 1000, false, 50);
	// 	chassis.moveTo(-45, 59, 1000, 100);
	// 	ShooterIntake = 0;
	// 	//deploy wings
	// 	//wings.set_value(true);
	// 	chassis.turnTo(-64, 43, 1000, false, 50);
	// 	chassis.moveTo(-64, 43, 1000, 100);
	// 	chassis.turnTo(-60, 22, 1000, false, 50);
	// 	chassis.moveTo(-60, 22, 1000, 100);
	// 		break;
	// 	case 2:
		wings.set_value(true);
		chassis.turnTo(-33, -66, 1000, false, 50);
		pros::delay(200);
		wings.set_value(false);
		pros::delay(700);
		chassis.turnTo(-7, -60, 1000, true, 50);
		chassis.moveTo(-7, -60 , 1000, 100);
		//outtake
		ShooterIntake = -90;
		pros::delay(2000);
		ShooterIntake = 0;
	// 		break;
	// 	case 3: 
	// 	wings.set_value(true);
	// 	chassis.turnTo(8,58, 1000, false, 50);
	// 	wings.set_value(false);
	// 	chassis.moveTo(8, 58, 1000, 100);
	// 		break;
	// 	default:
	// 	chassis.setPose(-40, 53, 180);
	// 	//intake
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-25, 6, 1000, false, 50);
	// 	chassis.moveTo(-25, 6, 1000, 100);
	// 	chassis.turnTo(-43, 6, 1000, false, 50);
	// 	pros::delay(200);
	// 	//outtake
	// 	ShooterIntake = -90;
	// 	//wait
	// 	pros::delay(200);
	// 	//intake
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-11, 2, 1000, false, 50);
	// 	chassis.moveTo(-11, 2, 1000, 100);
	// 	//stop intake
	// 	ShooterIntake = 0;
	// 	//deploy wings
	// 	chassis.turnTo(-42, 5, 1000, false, 50);
	// 	wings.set_value(true);
	// 	chassis.moveTo(-48, 5, 1000, 100);
	// 	//chassis.moveTo(-38, 5, 1000, 100);
	// 	//retract wings
	// 	wings.set_value(false);
	// 	//intake
	// 	ShooterIntake = 90;
	// 	chassis.turnTo(-10, 21, 1000, false, 50);
	// 	chassis.moveTo(-10, 21, 1000, 100);
	// 	chassis.turnTo(-45, 59, 1000, false, 50);
	// 	chassis.moveTo(-45, 59, 1000, 100);
	// 	ShooterIntake = 0;
	// 	//deploy wings
	// 	wings.set_value(true);
	// 	chassis.turnTo(-64, 43, 1000, false, 50);
	// 	chassis.moveTo(-64, 43, 1000, 100);
	// 	chassis.turnTo(-60, 22, 1000, false, 50);
	// 	chassis.moveTo(-60, 22, 1000, 100);
	// 		break;

	// }
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

/*void detectBall() {
	while(pros::c::adi_digital_read('B') == 0) {
				ShooterIntake = -90;
	}

	while(true) {
		if(pros::c::adi_analog_read('C') < 500) {
			pros::delay(750);
			while(pros::c::adi_digital_read('B') == 1) {
				ShooterIntake = -90;
			}
			pros::delay(500);
			while(pros::c::adi_digital_read('B') == 0) {
				ShooterIntake = -90;
			}
			ShooterIntake = 0;
		}
	}
}*/

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	bool liftPressed = false;

	float leftY = 0;
	float rightX = 0;

	int right_stick_smoothed = 0;
	int left_stick_smoothed = 0;
	float left_stick_prev = 0;
	float right_stick_prev = 0;

	int deadzone = 10;
	int t = 20; //turningCurve --> change to adjust sensitivity of turning
	int d = 2; //drivingCurve --> change to adjust sensitivity of forward / backward movement

	//pros::Task detectandLuanch(detectBall);

	while (true) {
		leftY = master.get_analog(ANALOG_LEFT_Y);
		rightX = master.get_analog(ANALOG_RIGHT_X);

		if(std::abs(leftY) < deadzone) {
			leftY = 0;
		}

		if(std::abs(rightX) < deadzone) {
			rightX = 0;
		}
		
		//drift reduction 
		//Andrew - update 2/9/24
		//https://www.desmos.com/calculator/jhvddogfxv

		//smoothing
		right_stick_smoothed = ((std::exp(-t / 12.5102293) + std::exp((std::abs(rightX) - 132.55) / 69) * (1 - std::exp(-t / 10))) * rightX * 0.4) + (right_stick_prev * 0.6);
		left_stick_smoothed =  ((std::exp(-d / 10) + std::exp((std::abs(leftY) - 100) / 10) * (1 - std::exp(-d / 10))) * leftY * 0.4) + (left_stick_prev * 0.6);
		right_stick_prev = right_stick_smoothed;
		left_stick_prev = left_stick_smoothed;
		//end smoothing

		//Apply new values to motors, make them move
		left_side_motors = defaultDriveCurve(left_stick_smoothed + right_stick_smoothed, 4);
		right_side_motors = defaultDriveCurve(left_stick_smoothed - right_stick_smoothed, 4);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			ShooterIntake = 90;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			ShooterIntake = -90;
		}
		else {
			ShooterIntake = 0;
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			wings.set_value(true);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			wings.set_value(false);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) && !liftPressed) {
			lift.set_value(true);
			liftPressed = true;
		}
		else {
			lift.set_value(false);
			liftPressed = false;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			pto.set_value(true);
		}


		pros::delay(20);
	}
} 
