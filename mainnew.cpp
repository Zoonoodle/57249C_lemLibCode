#include "main.h"
#include "lemlib/api.hpp"

// // Left motors
// pros::Motor left_motor_1(3, pros::E_MOTOR_GEARSET_06, true);   // Port 3, reversed
// pros::Motor left_motor_2(7, pros::E_MOTOR_GEARSET_06, true);   // Port 7, reversed
// pros::Motor left_motor_3(13, pros::E_MOTOR_GEARSET_06, false); // Port 13, not reversed

// // Right motors
// pros::Motor right_motor_1(18, pros::E_MOTOR_GEARSET_06, false); // Port 18, not reversed
// pros::Motor right_motor_2(14, pros::E_MOTOR_GEARSET_06, false); // Port 14, not reversed
// pros::Motor right_motor_3(1, pros::E_MOTOR_GEARSET_06, true);   // Port 2, reversed

// Create motor groups
pros::MotorGroup left_motors({-3,-7,13});
pros::MotorGroup right_motors({18,14,-2});



// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 12.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              466, // change to 470 or 480 if this causes issues somehow
                              2 // horizontal drift is 2 (for now)
);
pros::Imu imu(2); // inertial sensor instantiation

pros::Rotation vertical_encoder(-5); // creates a v5 rotation sensor for odom on port 1
pros::Rotation	horizontal_encoder(20); // creates another v5 rotation sensor for odom on port 1

//Vertical tracking wheels are parallel with the wheels on the drivetrain. Horizontal tracking wheels are perpendicular to the wheels on the drivetrain.

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.625);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);




// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

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
    pros::lcd::initialize(); // Initialize brain screen
    
    chassis.calibrate(); // Calibrate sensors (including IMU)

    // Wait for IMU to finish calibrating
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    // Print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // Print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // Delay to save resources
            pros::delay(20);
        }
    });
}



/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    imu.reset();
     chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

/**
 * Runs in driver control
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}

