#include "autons.h"
#include "lemlib/chassis/chassis.hpp"


#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

#include "global.h"






bool ejecting = false;

    // else  if (armMotors.get_position() >= 16000) {
    //      while (armIsMoving) {
    //     armMotors.move(60);
    //      while (armSensor.get_position() > 16000) {
    //     pros::delay(20);
    //     }

    //     if (armMotors.get_position() < 16000) {
    //         armIsMoving = false;
    //     }

    // }
    //   armMotors.set_brake_mode(pros::MotorBrake::hold);
    //     armMotors.move(0);
    // }
   

   


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
bool zeroOff = armSensor.get_angle();

void resetArm(void* param) {
    armMotors.move(127);
    pros::c::delay(1200);
    armMotors.move(0);
    
}


void redAWP() {


    bool zeroOff = armSensor.get_position();
    chassis.setPose(-0, 0, 0);
   
    armMotors.move(-127);
    pros::delay(600);
         
    armMotors.move(127);
   
    pros::Task armTask(resetArm, nullptr, "Arm Load Rings Task");

    chassis.moveToPoint(3,-22, 1200, {.forwards = false});
    
    armMotors.move(0);
    
    chassis.turnToHeading(90, 1000);

    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, -23, 1000, {.forwards = false, .earlyExitRange = 10});
    chassis.waitUntilDone();


    mogo.set_value(true);
    
}
        
void redSideMogoRush() {
    
    
         
    chassis.setPose(-58, -37, 270);
   
    
    
    

chassis.moveToPoint(-21, -38, 1500, {.forwards = false, .minSpeed = 100});


chassis.moveToPoint(-21, -55, 1000, {.forwards = false});

chassis.turnToHeading(270,800, {.maxSpeed = 70});

chassis.moveToPoint(-5.3, -49, 2000, {.forwards = false,  .maxSpeed = 50});

chassis.waitUntilDone();
mogo.set_value(true);


pros::c::delay(250);


intake.move(127);

chassis.moveToPoint(-35, -55, 1000);
pros::delay(500);

chassis.moveToPoint(-58, -80, 1200, {.forwards = false, .minSpeed = 110});
chassis.waitUntilDone();
mogo.set_value(false);
pros::delay(250);

intake.move(0);

chassis.moveToPoint(-45, -10, 1500);

pros::delay(100000);

chassis.turnToHeading(315,600);

chassis.moveToPoint(-20, -38, 1000, {.forwards = false, .maxSpeed = 80});


pros::delay(100000);
chassis.turnToHeading(-21, 500);
chassis.moveToPoint(-13, -38, 1000);
chassis.moveToPoint(-16, -37, 1000);

pros::delay(300);
chassis.moveToPoint(-12, -42, 1000, {.forwards = false});



chassis.moveToPose(3,7 , 0, 2000, {.minSpeed = 90});
chassis.waitUntilDone();

intake.move(0);



pros::delay(100000);




chassis.moveToPoint(-22, 7.8, 800, {.forwards = false, .minSpeed = 100});



chassis.turnToHeading(135, 500);
chassis.waitUntilDone();



intake.move_velocity(0);
chassis.waitUntilDone();
intake.move(0);
mogo.set_value(false);

chassis.moveToPoint(22, -1, 2000);
intake
.move(600);

chassis.moveToPoint(52, 1, 3000, {.maxSpeed = 70});
chassis.waitUntilDone();
pros:pros::c::delay(100);
intake.move(0);

chassis.turnToHeading(57, 500);
chassis.moveToPoint(15, -22, 4000, {.forwards = false});
chassis.moveToPoint(11, -23, 3000, {.forwards = false, .maxSpeed = 45});
chassis.waitUntilDone();

pros::delay(200);
mogo.set_value(true);
pros::delay(100);
intake.move(127);

chassis.moveToPoint (35, -23, 4000, {.maxSpeed = 100});



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

    
    chassis.setPose(-63,0,90);
   
    intake.move(127);
    pros::delay(500);
    
    chassis.moveToPoint(-45, 0, 2000);
    intake.move(0);
    
    chassis.turnToHeading(0,500);
    chassis.moveToPoint(-47, -19, 2000,{.forwards = false});


    chassis.waitUntilDone();
    mogo.set_value(true);
    
    
    chassis.turnToPoint(-27, -20, 500);
    chassis.waitUntilDone();
    
  


    chassis.moveToPoint(-27,-20,1000);
      intake.move(127);

    chassis.moveToPoint(1, -55,2000);
    pros::delay(350);
    
    

    chassis.moveToPoint(-10, -51, 2000);
  
  
    chassis.moveToPoint(-37, -50, 1000);
    
   
   
    
    chassis.moveToPoint(-50, -50, 1000);
    chassis.moveToPoint(-60, -50, 1200);
    intake.move(127);
    pros::delay(1000);

    
    
   

    chassis.moveToPoint(-45, -62, 2000, {.minSpeed = 50});
   pros::delay(1000);
    intake.move(127);
    

    chassis.turnToHeading(53, 600);
    intake.move(127);
    
  
     
    chassis.moveToPoint(-62, -66, 2000, {.forwards = false, .minSpeed = 90});
  chassis.waitUntilDone();
    mogo.set_value(false);
intake.move(0);
    
   
    chassis.moveToPoint(-45, -47, 2000);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(-47,10,3000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 10});
    chassis.moveToPoint(-47, 18.5, 2000 ,{.forwards = false, .maxSpeed = 70});
    pros::delay(250);
    chassis.waitUntilDone();
    mogo.set_value(true);

chassis.turnToPoint(-27, 20, 500);
    chassis.waitUntilDone();
    
  


    chassis.moveToPoint(-27,20,1000);
      intake.move(127);

    chassis.moveToPoint(1, 57,2000);
    
    
    

    chassis.moveToPoint(-10, 51, 2000);
  
  
    chassis.moveToPoint(-37, 50, 1000);
    
   
   
    
    chassis.moveToPoint(-50, 50, 1000);
    chassis.moveToPoint(-60, 50, 1200);
    intake.move(127);
    pros::delay(1200);

    
   

    chassis.moveToPoint(-45, 62, 2000, {.minSpeed = 50});
   pros::delay(1000);
    intake.move(127);
    


    intake.move(127);
    
  
     
    chassis.moveToPoint(-62, 65, 2000, {.forwards = false, .minSpeed = 90});
  chassis.waitUntilDone();
    mogo.set_value(false);




    


    intake.move(0);
    
   
    chassis.moveToPoint(24,55,2000);
  
    intake.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(125);
    intake.move_velocity(0);

   
    chassis.moveToPoint(50, 33, 2000, {.forwards = false, .maxSpeed  = 100});
    chassis.moveToPoint(63,22, 3000, {.forwards = false, . maxSpeed = 80});
    
    chassis.waitUntilDone();
    
    mogo.set_value(true);
    


    chassis.moveToPoint(59,40 , 1000);
    chassis.waitUntilDone();
    doinker.set_value(true);

    chassis.turnToHeading(-90, 1400, {.maxSpeed = 100});

    chassis.waitUntilDone();
    doinker.set_value(false);

    

    chassis.moveToPoint(62,65, 2000, {.forwards = false, .minSpeed = 90});
     chassis.waitUntilDone();
     chassis.turnToHeading(235, 1000);
    mogo.set_value(false);
    


    
    chassis.moveToPoint(60, 60, 1000);
    chassis.moveToPoint(48, 10, 2000, {.forwards = false});
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(47, 3, 3000, {.forwards = false, .maxSpeed = 900});
    chassis.waitUntilDone();
    mogo.set_value(true);
    
    pros::delay(300);
    intake.move_velocity(600);
    chassis.moveToPoint(24,31,2000);
   
    chassis.moveToPoint(0, 0, 1500);
    pros::delay(500);
    chassis.turnToHeading(135, 800);
    chassis.moveToPoint(24, -24, 1500);

    
    chassis.turnToHeading(180, 800);
    
    chassis.moveToPoint(24, -59, 1000);

    chassis.waitUntilDone();
    
    

    chassis.moveToPoint(50, -60, 1500);
  
  


    chassis.turnToHeading(270, 1000);
    


    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPoint(60, -64, 2000, {.forwards = false});
    chassis.turnToHeading(305, 800);
    chassis.waitUntilDone();
    mogo.set_value(false);

    pros::delay(350);
    intake.move(127);
    chassis.moveToPoint(47, -50, 1000);
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_velocity(0);

    chassis.moveToPoint(47, -3, 3000, {.forwards = false, .maxSpeed = 110});
    mogo.set_value(true);
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(75,-3,2500, {.forwards = false});
    
    
    chassis.waitUntilDone();
    pros:pros::delay(350);
    intake.move(127);
    pros::delay(600);
   chassis.moveToPoint(30, -31, 1500);
    chassis.waitUntilDone();
    intake.move_velocity(0);
    hang.set_value(true);

   chassis.turnToPoint(0, 0, 800);
    pros::delay(500);


     chassis.moveToPoint(0, 0, 2000, {.maxSpeed = 50, .minSpeed = 50});
   chassis.moveToPoint(30, -31, 600, {.forwards = false, .maxSpeed = 50, .minSpeed = 50 });
    chassis.moveToPoint(0, 0, 600, {.maxSpeed = 50, .minSpeed = 50});
       chassis.moveToPoint(30, -31, 400, {.forwards = false, .maxSpeed = 50, .minSpeed = 50 });
        chassis.moveToPoint(0, 0, 400, {.maxSpeed = 50, .minSpeed = 50});
       chassis.moveToPoint(30, -31, 400, {.forwards = false, .maxSpeed = 50, .minSpeed = 50 });
        chassis.moveToPoint(0, 0, 500, {.maxSpeed = 50, .minSpeed = 50});
       chassis.moveToPoint(30, -31, 500, {.forwards = false, .maxSpeed = 50, .minSpeed = 50 });
}
