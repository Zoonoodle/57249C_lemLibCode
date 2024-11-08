#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "autons.h"
#include "global.h"
#include "pros/screen.hpp"
#include <cmath>
#include <vector>
#include <random>


//#include <concepts>
//#include <span>
//#include <sys/_intsup.h>
//#include <sys/signal.h>

// PROS Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


    pros::ADIDigitalOut mogo('A'); 
    pros::ADIDigitalOut doinker('E'); 
    pros::ADIDigitalOut intakeLift('H'); 



    // pros::Optical sorter(11);
   

    bool mogoActivated = false;
    bool intakeActivated = false;

    pros::Motor intake(18, pros::MotorGearset::blue);


pros::Rotation armSensor(19);
pros::MotorGroup left_motors({-3, -6, -8}, 
                     
                        pros::MotorGearset::blue); 
pros::MotorGroup right_motors({12, 15, 11}, 
                        pros::MotorGearset::blue); 
pros::MotorGroup armMotors({10, -20}, 
                                        pros::MotorGearset::green);


pros::Imu imu(16);

pros::Rotation vertical_encoder(9); 
pros::Rotation horizontal_encoder(21);



lemlib::TrackingWheel vertical(&vertical_encoder, lemlib::Omniwheel::NEW_2, -.5);
lemlib::TrackingWheel horizontal(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.5);


lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors, 
                              12, 
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 
);
// Controller Settings
lemlib::ControllerSettings lateral_controller(
    20.0,    // kP
    0,  // kI
    2.0,   // kD
    3.0,    // Integral limit
    1.0,    // Slew rate
    100.0,  // Max output
    3.0,    // Deadband
    500.0,  // On target time
    5.0     // Tolerance
);

lemlib::ControllerSettings angular_controller(
    2.0,    // kP
    0.0,    // kI
    10.0,   // kD
    3.0,    // Integral limit
    1.0,    // Slew rate
    100.0,  // Max output
    3.0,    // Deadband
    500.0,  // On target time
    0.0     // Tolerance
);

// Odometry Sensors
lemlib::OdomSensors sensors(
    &vertical,          // Left tracking wheel
    nullptr,            // Right tracking wheel (not used)
    &horizontal,        // Back tracking wheel
    nullptr,            // Front tracking wheel (not used)
    &imu                // Inertial sensor
);


lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.008);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.008);


lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors,
    &throttle_curve,
    &steer_curve
);

double armZeroOffset = 0;

double getZeroedAngle() {
    return (armSensor.get_position() - armZeroOffset);
}



bool red = true;
bool armIsMoving = false;
int i = 0;


void armLoadRings(void* param) {
    armIsMoving = true;
   
    double targetAngle = 2000;
    double armRotation = getZeroedAngle();

    
    // Check if the arm needs to move up or down to reach the target angle
    if (getZeroedAngle() <= targetAngle) {
        while (armIsMoving) {
            armRotation = getZeroedAngle(); // Update rotation value

            if (armRotation < targetAngle) {
                armMotors.move(-60); 
            } else {
                armMotors.move(0);
                armIsMoving = false;
            }
            pros::delay(20);
        }
    } else {
        while (armIsMoving) {
            armRotation = getZeroedAngle(); // Update rotation value

            if (armRotation > targetAngle) {
                armMotors.move(60); 
            } else {
                armMotors.move(0);
                armIsMoving = false;
            }
            pros::delay(20);
        }
    }
    armMotors.set_brake_mode(pros::MotorBrake::hold);

}

/*
class RamseteController {
public:
    RamseteController(double wheelBase, double wheelRadius)
        : wheelBase(wheelBase), wheelRadius(wheelRadius), currentLinearVelocity(0.0), currentAngularVelocity(0.0) {}
   
        double kBeta = 2.0;
        double kZeta = 0.7;
        double SCREEN_HEIGHT = 240;
        double SCREEN_WIDTH = 320;
        double SCALE_FACTOR = 1;
    void drawPath(double x1, double y1, double x2, double y2) {
    pros::lcd::clear();


    // Draw the starting point
    int startX = x1 * SCALE_FACTOR + SCREEN_WIDTH / 2;
    int startY = SCREEN_HEIGHT / 2 - y1 * SCALE_FACTOR;
    pros::screen::draw_circle(startX, startY, 5);

    // Draw the target point
    int targetX = x2 * SCALE_FACTOR + SCREEN_WIDTH / 2;
    int targetY = SCREEN_HEIGHT / 2 - y2 * SCALE_FACTOR;
    pros::screen::draw_circle(targetX, targetY, 5); // Draw target point


    // Draw the path
    pros::screen::draw_line(startX, startY, targetX, targetY);
    }
    bool atTarget(double targetX, double targetY) {
        // Check if close enough to target (you can adjust the threshold)
        lemlib::Pose currentPose = chassis.getPose();
        double currentX = currentPose.x;
        double currentY = currentPose.y;
        return (fabs(targetX - currentX) < 0.05 && fabs(targetY - currentY) < 0.05);
    }
    void moveTo(double targetX, double targetY, double targetTheta, double v_d, double omega_d, pros::Motor& leftMotor, pros::Motor& rightMotor) {
        while (atTarget(targetX, targetY) == false)
        {
        // Get current pose
        lemlib::Pose currentPose = chassis.getPose();
        double currentX = currentPose.x;    
        double currentY = currentPose.y;    
        double currentTheta = currentPose.theta;  
        drawPath(currentX, currentY, targetX, targetY);

        //set wheel base and radius
        wheelBase = 12.5;
        wheelRadius = 2.75/2;
       
        // Compute errors in the robot's local frame
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double e_x = dx * cos(currentTheta) + dy * sin(currentTheta);
        double e_theta = targetTheta - currentTheta;
       
        // RAMSETE control law
        double k = 2 * kZeta * sqrt(v_d * v_d + omega_d * omega_d);
        double linearVelocity = v_d * cos(e_theta) + k * e_x;
        double angularVelocity = omega_d + k * sin(e_theta);




        // Increment current velocities towards target velocities
        currentLinearVelocity = currentLinearVelocity + ((linearVelocity - currentLinearVelocity) * 0.1);  
        currentAngularVelocity = currentAngularVelocity + ((angularVelocity - currentAngularVelocity) * 0.1);




        // Convert to left and right motor commands
        double leftVelocity = (currentLinearVelocity - currentAngularVelocity * (wheelBase / 2)) / wheelRadius;
        double rightVelocity = (currentLinearVelocity + currentAngularVelocity * (wheelBase / 2)) / wheelRadius;;




        //Set motor velocities
        leftMotor.move_velocity(leftVelocity);
        rightMotor.move_velocity(rightVelocity);
        }
    }
private:
    double wheelBase;
    double wheelRadius;
    double currentLinearVelocity;
    double currentAngularVelocity;
};

*/


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); 
    
    chassis.calibrate(); 
     


        armZeroOffset = armSensor.get_position();
        

        pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources

            
            pros::delay(50);
        }
    });
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
 
     int autonSelector = 1;  

    switch (autonSelector) {
        case 1:
            redAWP();
            break;
        case 2:
            redSideMogoRush();
            break;
        case 3:
            skillsAuton();
            break;
        default:
            
            skillsAuton();
            break;
    }
    


    
        // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}


void opcontrol() {
    
    
    while (true) {
       
        
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      
        chassis.arcade(leftY, rightX);

        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    intake.move(127); 
} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    intake.move(-127); 
} else {
    intake.move(0); 
}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    armMotors.move(-127); 
   

} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    pros::Task armTask(armLoadRings, nullptr, "Arm Load Rings Task");
    pros::delay(1000);
     
} else {
    armMotors.move(0); 
}


     if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
         armMotors.move(127);
        }
       
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            mogoActivated = !mogoActivated; 
            mogo.set_value(mogoActivated); 
        }

   

       


        

      
        pros::delay(10);
    }
}



