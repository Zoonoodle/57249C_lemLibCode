#include "autons.h"
#include "lemlib/chassis/chassis.hpp"


#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

#include "global.h"






bool ejecting = false;

void eject_ring() {
    intake.move_velocity(0);
    pros::delay(150);
    intake.move_velocity(127);
}


void sort_red() {
    
     sorter.set_led_pwm(35);  

    if (sorter.get_proximity() > 50) {  
        pros::delay(80);
        intake.move(0);
        pros::delay(200);

        intake.move(127);
        }
    }
 
   


void sort_blue() {
    sorter.set_led_pwm(35);
    if (sorter.get_proximity() > 150) {
        pros::delay(20);
        if (sorter.get_hue() > 180 && sorter.get_hue() < 300 && !ejecting) {
            ejecting = true;
            intake.set_brake_mode(pros::MotorBrake::hold);
            pros::delay(10);
           
            eject_ring();
            intake.set_brake_mode(pros::MotorBrake::coast);
            ejecting = false;
        }
    }
}

void redAWP() {

chassis.setPose(-44,-65,0);

chassis.turnToHeading(90, 600);
chassis.turnToHeading(0, 600);


}
        
void redSideMogoRush() {
    
    
         
chassis.setPose(-62, -37, 270);
   
    
    
    

chassis.moveToPoint(-21, -38, 1500, {.forwards = false, .minSpeed = 100});


chassis.moveToPoint(-21, -55, 1000, {.forwards = false});

chassis.turnToHeading(270,800, {.maxSpeed = 70});

chassis.moveToPoint(-5.3, -49, 2000, {.forwards = false,  .maxSpeed = 50});
pros::delay(250);
chassis.waitUntilDone();
mogo.set_value(true);


pros::c::delay(250);


intake.move(127);
pros::delay(1000);
chassis.moveToPoint(-40, -65, 1000);

chassis.moveToPoint(-35, -40, 1000);

pros::delay(1200);
chassis.moveToPoint(-58, -80, 1200, {.forwards = false, .minSpeed = 110});
chassis.waitUntilDone();
mogo.set_value(false);
pros::delay(250);

intake.move(127);

chassis.moveToPoint(-10, -10, 1500, {.earlyExitRange = 10});


//////



}


void blueSideMogoRush() {
 
         
    chassis.setPose(9.5, 0, 180);
   
    
    
    

chassis.moveToPoint(-7, 44.3, 2000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 10});
chassis.moveToPoint(-7.5, 44.7, 3000, {.forwards = false, .maxSpeed = 80});
pros::delay(500);
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
chassis.waitUntilDone();


mogo.set_value(true);
pros::delay(350);
intake.move(127);


chassis.turnToHeading(201, 500);
chassis.moveToPoint(-13, 34, 1000);
chassis.moveToPoint(-16, 33, 1000);

pros::delay(300);
chassis.moveToPoint(-12, 42, 1000, {.forwards = false});



chassis.moveToPose(3,-7 , 180, 2000, {.minSpeed = 90});
chassis.waitUntilDone();

intake.move(0);





chassis.moveToPose(-24.8, -9.2, 242.6, 2500, {.maxSpeed = 110, .minSpeed = 70});














chassis.turnToHeading(45, 500);
chassis.waitUntilDone();



intake.move_velocity(0);
chassis.waitUntilDone();
intake.move(0);
mogo.set_value(false);

chassis.moveToPoint(22, 3, 2000);
intake.move(600);

chassis.moveToPoint(50, 4, 6000, {.maxSpeed = 70});
chassis.waitUntilDone();

intake.move(0);

chassis.turnToHeading(57, 500);
chassis.moveToPoint(15, 22, 4000, {.forwards = false, .minSpeed = 100});
chassis.moveToPoint(10, 24, 3000, {.forwards = false, .maxSpeed = 60});
chassis.waitUntilDone();

pros::delay(200);
mogo.set_value(true);
pros::delay(400);
intake.move(127);

chassis.moveToPoint (29, 23, 4000, {.maxSpeed = 100});



}


void skillsAuton() {
chassis.setPose(-65,0,90);
mogo.set_value(true);
intake.move(127);
pros::delay(400);
chassis.moveToPoint(-47, 0, 1000);
intake.move(0);
chassis.turnToHeading(0, 500);
chassis.moveToPoint(-47, -15, 1000, {.forwards = false});
chassis.waitUntilDone();
mogo.set_value(false);
chassis.swingToPoint(-30, -21, lemlib::DriveSide::RIGHT, 500, { .minSpeed = 100, .earlyExitRange = 20});
chassis.moveToPoint(17, -44, 1000);
intake.move(127);
chassis.turnToPoint(21, -31, 500);
chassis.moveToPoint(21, -31, 1000);
chassis.swingToHeading(25, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 100, .earlyExitRange = 20});
chassis.moveToPoint(0, -52, 1000);
chassis.turnToHeading(180, 500);
chassis.moveToPoint(0, -61, 1000);
//backpack load
chassis.waitUntilDone();
intake.move(0);
//backpack score
chassis.moveToPoint(0, -47, 1000, {.forwards = false});
chassis.turnToHeading(270, 500);
intake.move(127);
chassis.moveToPoint(-52, -47, 1000,{.maxSpeed = 90});
chassis.swingToPoint(-47, -52, lemlib::DriveSide::RIGHT, 500, {.forwards = false});
chassis.moveToPoint(-47, -52, 1000);
chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
chassis.moveToPoint(-59, -59, 1000);
chassis.waitUntilDone();
mogo.set_value(true);

chassis.moveToPoint(0, 0, 2500);
intake.move(127);
chassis.turnToHeading(315, 500);
intake.move(0);
chassis.moveToPoint(-23.5, 23.5, 1000);
intake.move(127);
pros::delay(100);
intake.move(0);
chassis.turnToHeading(90, 500);
chassis.moveToPoint(-38, 23.5, 1000, {.forwards = false});
chassis.waitUntilDone();
mogo.set_value(false);
intake.move(127);
chassis.swingToPoint(-23.5, 47, lemlib::DriveSide::LEFT, 500);
chassis.moveToPoint(-23.5, 47, 1000);
chassis.turnToHeading(270, 500);
chassis.moveToPoint(-53, 47, 1000);
chassis.swingToPoint(-49, 54, lemlib::DriveSide::LEFT, 500);
chassis.moveToPoint(-49, 54, 1000);
chassis.turnToHeading(135, 500);
chassis.moveToPoint(-59, 59, 1000, {.forwards = false});
chassis.waitUntilDone();
mogo.set_value(true);
chassis.turnToPoint(0, 57, 500);
//backpack load
chassis.moveToPoint(0, 61, 1000);
intake.move(0);
//backpack score
chassis.moveToPoint(0, 59, 1000, {.forwards = false});
chassis.turnToHeading(90, 500);
chassis.moveToPoint(40, 59, 1500);
intake.move(127);
pros::delay(100);
intake.move(0);
chassis.moveToPoint(44, 59, 1000);
chassis.waitUntilDone();
intake.move(127);
pros::delay(100);
intake.move(0);
chassis.moveToPoint(0, 59, 2000);
chassis.turnToHeading(0, 500);
chassis.moveToPoint(0, 61, 1000);
//backpack load
intake.move(127);
pros::delay(100);
intake.move(0);
//backpack score
//backpack load
intake.move(127);
pros::delay(200);
intake.move(0);
//backpack score
chassis.moveToPoint(0, 47, 1000, {.forwards = false, .earlyExitRange = 5});
chassis.swingToPoint(23.5, 47, lemlib::DriveSide::LEFT, 500,{.earlyExitRange = 20});
chassis.moveToPoint(40, 47, 1000);
chassis.waitUntil(23.5);
intake.move(127);
chassis.waitUntil(26);
intake.move(0);
chassis.waitUntilDone();
intake.move(127);
pros::delay(100);
intake.move(0);
chassis.swingToHeading(135, lemlib::DriveSide::RIGHT, 500);
chassis.moveToPoint(0, chassis.getPose().y - chassis.getPose().x, 1000, {.forwards = false});
chassis.moveToPoint(47, 7, 1000, {.forwards = false});
chassis.waitUntilDone();
mogo.set_value(false);
chassis.turnToHeading(180, 750);
intake.move(-127);
pros::delay(200);
intake.move(127);
chassis.moveToPoint(47, -40, 1000);
chassis.waitUntilDone();
//backpack load
chassis.moveToPoint(47, -53, 1000);
//move backpack up a tiny bit so it doesnt get in the way
chassis.waitUntilDone();
chassis.swingToPoint(59, -47, lemlib::DriveSide::RIGHT, 500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
chassis.moveToPoint(59, -47, 1000);
//move backpack back down to block the other ring
chassis.turnToHeading(0, 500);
intake.move(0);
chassis.moveToPoint(59, -59, 1000, {.forwards = false});
chassis.swingToHeading(330, lemlib::DriveSide::LEFT, 500, {.minSpeed = 100});
chassis.moveToPose(0, 35, 0, 2000);
chassis.turnToHeading(180, 500);
chassis.moveToPose(51, 19, 240, 1000, {.forwards = false});
chassis.swingToPoint(59, 47, lemlib::DriveSide::RIGHT, 500);
chassis.moveToPoint(59, 47, 1000);
intake.move(127);
pros::delay(100);
intake.move(0);
chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 500);
chassis.moveToPoint(59, 59, 1000, {.forwards = false});
chassis.waitUntilDone();
mogo.set_value(true);
chassis.moveToPose(50, 0, 270, 2000);
chassis.moveToPoint(61, 0, 1000, {.forwards = false});
chassis.waitUntilDone();
intake.move(127);
chassis.moveToPoint(30, 31, 1000);
chassis.moveToPoint(0, 1, 1000);
//hang activate

}
