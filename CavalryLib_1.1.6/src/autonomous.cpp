#include "autonomous.hpp"
#include "cavalry/pose.hpp"
#include "cavalry/robot/robot.hpp"
#include "globals.hpp"
#include "pros/adi.h"
#include <cmath>

void autonomous() {
    driveState = AUTONOMOUS;
    select_auto();
}

void skills() {
    particleFilter.set_enabled(false);
    intakeState = STORE;
    wing.extend();
    robot.moveToPoint(0, 7.5, 1500, {}, false);
    robot.turnToHeading(-52, 1500, {}, false);
    robot.moveToPoint(-14, 23, 3000, {}, false);
    robot.turnToHeading(-135, 1000, {}, false);
    robot.moveToPoint(-1.5, 35, 2000, {.forwards = false, .maxSpeed=80}, false);
    intakeState = MEDIUM_GOAL;
    particleFilter.set_enabled(true);
    distanceSensorFront.enable_sensor();
    pros::delay(800);
    robot.moveToPoint(-45, -47, 3000, {.maxSpeed=90});
    pros::delay(200);
    doinker.extend();
    intakeState = HIGH_GOAL;
    pros::delay(500);
    intakeState = STORE;
    robot.turnToHeading(-177, 1000);
    robot.moveToPoint(robot.getPose().x, -80, 2000, {.maxSpeed=40}, false);
    shimmy_shake();
    robot.moveToPoint(-54, -45, 1000, {.forwards=false, .minSpeed=60, .earlyExitRange=3});
    robot.moveToPoint(-55.5, 30, 5000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 3});
    pros::delay(500);
    doinker.retract();
    intakeState = OFF;
    robot.moveToPoint(-39, 40, 5000, {.forwards = false, .maxSpeed = 100});
    robot.turnToHeading(0, 1000, {.maxSpeed = 110}, false);
    particleFilter.set_starting_pose(cavalry::Pose(-48, 40, 0), true);
    pros::delay(300);
    robot.moveToPoint(-48.25, 25, 1000, {.forwards = false, .maxSpeed = 100}, false);
    doinker.extend();
    distanceSensorBack.disable_sensor();
    particleFilter.set_starting_pose(cavalry::Pose(-48, 30, 0), true);
    intakeState = LOW_GOAL;
    pros::delay(150);
    intakeState = HIGH_GOAL;
    pros::delay(2200);
    intakeState = STORE;
    robot.moveToPoint(-47.5, 65, 2700, {.maxSpeed=40}, false);
    shimmy_shake();
    robot.moveToPoint(-48.5, 28, 2000, {.forwards = false, .maxSpeed=100}, false);
    intakeState = LOW_GOAL;
    pros::delay(150);
    particleFilter.set_starting_pose(cavalry::Pose(-48, 30, 0), true);
    doinker.retract();
    intakeState = HIGH_GOAL;
    pros::delay(2200);
    intakeState = STORE;
    robot.moveToPoint(-18, 61, 2000, {}, false);
    robot.swingToHeading(90, DriveSide::RIGHT, 800, {AngularDirection::CW_CLOCKWISE}, false);
    robot.moveToPoint(15, 63.5, 1200, {.maxSpeed = 80}, false);
    pros::delay(400);
    robot.moveToPoint(35, 63.5, 2500, {.maxSpeed = 100}, false);
    pros::delay(500);
    robot.swingToHeading(-175, DriveSide::LEFT, 1200, {.direction = AngularDirection::CW_CLOCKWISE, .earlyExitRange=3});
    robot.moveToPoint(robot.getPose().x, 80, 500, {.forwards=false, .maxSpeed = 70}, false);
    doinker.retract();
    distanceSensorBack.enable_sensor();
    imu1.set_rotation(-180);
    imu2.set_rotation(-180);
    particleFilter.set_starting_pose(cavalry::Pose(24, 63, 0), true);
    pros::delay(500);
    intakeState = OFF;
    robot.moveToPoint(21, 21, 2000, {.maxSpeed = 80});
    robot.turnToHeading(-316, 1000, {}, false);
    robot.moveToPoint(13.5, 10.5, 1000, {.forwards=false, .maxSpeed = 60}, false);
    doinker.extend();
    intakeState = MEDIUM_GOAL;
    pros::delay(2500);
    robot.moveToPoint(46, 47, 3000, {.maxSpeed=90});
    pros::delay(500);
    intakeState = HIGH_GOAL;
    pros::delay(700);
    intakeState = STORE;
    robot.turnToHeading(5, 1000);
    robot.moveToPoint(robot.getPose().x, 80,1600, {.maxSpeed=40});
    shimmy_shake();
    robot.moveToPoint(55, 45, 1000, {.forwards=false, .minSpeed=60, .earlyExitRange=3});
    robot.moveToPoint(56, -38, 5000, {.forwards = false});
    pros::delay(500);
    doinker.retract();
    intakeState = OFF;
    robot.moveToPoint(43, -40, 5000, {.forwards = false, .maxSpeed = 100});
    robot.turnToHeading(-180, 1000, {.maxSpeed = 110}, false);
    particleFilter.set_starting_pose(cavalry::Pose(48, -40, 0), true);
    pros::delay(500);
    robot.moveToPoint(48.5, -25, 1000, {.forwards = false, .maxSpeed = 100}, false);
    doinker.extend();
    distanceSensorBack.disable_sensor();
    particleFilter.set_starting_pose(cavalry::Pose(48, -30, 0), true);
    intakeState = LOW_GOAL;
    pros::delay(150);
    intakeState = HIGH_GOAL;
    pros::delay(2200);
    intakeState = STORE;
    robot.moveToPoint(47.5, -65, 2700, {.maxSpeed=40});
    shimmy_shake();
    robot.moveToPoint(49, -29, 2000, {.forwards = false, .maxSpeed=100}, false);
    intakeState = LOW_GOAL;
    pros::delay(150);
    particleFilter.set_starting_pose(cavalry::Pose(48, -30, 0), true);
    doinker.retract();
    intakeState = HIGH_GOAL;
    pros::delay(2200);
    intakeState = STORE;
    robot.moveToPoint(18, -59.5, 2000, {}, false);
    robot.swingToHeading(-90, DriveSide::RIGHT, 1000, {AngularDirection::CW_CLOCKWISE}, false);
    robot.moveToPoint(-8, -63, 1400, {}, false);
}

// mid goal inversion
void awp_left_blue() {
    intakeState = STORE;
    robot.setPose(1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(90, 500);
    robot.moveToPoint(20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(-25, -35.5, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(1300);
    doinker.retract();
    robot.moveToPoint(-13, -36, 1300, {}, false);
    intakeState = STORE;
    robot.turnToHeading(-27, 700);
    robot.moveToPoint(-29.5, -7.5, 1300, {});
    pros::delay(600);
    doinker.extend();
    pros::delay(700);
    robot.turnToHeading(133, 1000, {});
    doinker.retract();
    pros::delay(700);
    robot.moveToPoint(-41.5, 4, 1000, {.forwards = false, .maxSpeed = 60}, false);
    intakeState = MEDIUM_GOAL;
    pros::delay(1500);
    intakeState = OFF;
    robot.moveToPoint(-18, -20.5, 1000, {});
    robot.turnToHeading(-92, 1000, {}, false);
    robot.moveToPoint(-47, -20.5, 2000, {}, false);
   
}

// Low goal inversion
void awp_right_blue(){
    intakeState = STORE;
    robot.setPose(-1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(-90, 500);
    robot.moveToPoint(-20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(-20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(25, -35, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(1300);
    doinker.retract();
    robot.moveToPoint(13, -36, 1300, {}, false);
    intakeState = STORE;
    robot.turnToHeading(27, 700);
    robot.moveToPoint(29.5, -7.5, 1300, {});
    pros::delay(600);
    doinker.extend();
    robot.turnToHeading(48, 1000, {});
    doinker.retract();
    pros::delay(700);
    robot.moveToPoint(36, -2.5, 1000, {.maxSpeed = 60}, false);
    intakeState = LOW_GOAL;
    pros::delay(1500);
    intakeState = OFF;
    robot.moveToPoint(18, -23, 1000, {.forwards=false});
    robot.turnToHeading(-90, 1000., {}, false);
    robot.moveToPoint(42, -22.5, 1500, {.forwards=false}, false);
    pros::delay(1000000000);
}

// middle goal hold 
void awp_left_red() {
    intakeState = STORE;
    robot.setPose(1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(90, 500);
    robot.moveToPoint(20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(-25, -35.5, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(650);
    doinker.retract();
    intakeState = STORE;
    robot.moveToPoint(-13, -36, 1300, {}, false);
    robot.turnToHeading(-27, 700);
    robot.moveToPoint(-29.5, -7.5, 1300, {});
    pros::delay(600);
    doinker.extend();
    pros::delay(700);
    robot.turnToHeading(133, 1000, {});
    doinker.retract();
    pros::delay(700);
    robot.moveToPoint(-42.5, 4, 1000, {.forwards = false, .maxSpeed = 60}, false);
    intakeState = MEDIUM_GOAL;
    pros::delay(1600);
    intakeState = OFF;
}

// solo
void awp_right_red(){
    intakeState = STORE;
    robot.setPose(-1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(-90, 500);
    robot.moveToPoint(-20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(-20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(25, -35, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(1300);
    intakeState = STORE;
    doinker.retract();
    robot.moveToPoint(13, -36, 1300, {}, false);
    robot.turnToHeading(27, 700);
    robot.moveToPoint(29.5, -7.5, 1300, {});
    pros::delay(600);
    doinker.extend();
    robot.turnToHeading(0, 500, {}, false);
    robot.moveToPoint(29.25, 28, 3000, {.minSpeed=100, .earlyExitRange=3});
    doinker.retract();
    intakeState = LOW_GOAL;
    pros::delay(100);
    intakeState = STORE;
    pros::delay(765);
    doinker.extend();
    robot.moveToPoint(29, 37, 2500, {.maxSpeed=80});
    robot.turnToHeading(-45, 600);
    robot.moveToPoint(38, 28, 650, {.forwards = false, .maxSpeed=100}, false);
    intakeState = MEDIUM_GOAL;
    pros::delay(690);
    intakeState = STORE;
    robot.moveToPoint(0, 64.25, 2500, {}, false);
    robot.turnToHeading(-90, 700);
    robot.moveToPoint(-40, 64.25, 300, {.minSpeed=100, .earlyExitRange = 3});
    robot.moveToPoint(-40, 64, 600, {.maxSpeed=40}, false);
    robot.moveToPoint(28, 63.25, 2000, {.forwards = false, .maxSpeed=80});
    pros::delay(250);
    intakeState = LOW_GOAL;
    pros::delay(250);
    intakeState = HIGH_GOAL;
    pros::delay(1000000);
}

// 7 ball left
void elim_left_blue() {
    intakeState = STORE;
    wing.extend();
    robot.moveToPoint(0, 7, 1500, {.minSpeed=80, .earlyExitRange=3});
    robot.turnToHeading(-50, 700, {.earlyExitRange=3});
    robot.moveToPoint(-13.5, 22.5, 3000, {.maxSpeed=100, .earlyExitRange=3});
    pros::delay(370);
    doinker.extend();
    robot.turnToHeading(47, 700, {.earlyExitRange=3});
    robot.moveToPoint(-33.5, 3.0, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange=5});
    robot.turnToHeading(175, 800, {.earlyExitRange=3});
    robot.moveToPoint(-36.5, -40, 1500, {.maxSpeed=40});
    robot.moveToPoint(-38, 24, 1300, {.forwards = false, .maxSpeed=100});
    pros::delay(600);
    intakeState = LOW_GOAL;
    pros::delay(200);
    intakeState = HIGH_GOAL;
    pros::delay(2050);
    intakeState = STORE;
    doinker.retract();
    pros::delay(1000000000);
    robot.moveToPoint(-38, 16, 700, {.minSpeed = 80, .earlyExitRange = 3});
    robot.moveToPoint(26.5, 15, 2000);
    robot.turnToHeading(-185, 700, {}, false);
    wing.retract();
    robot.moveToPoint(robot.getPose().x, 35, 2000, {.forwards = false, .minSpeed = 120, .earlyExitRange=3});
    robot.moveToPoint(robot.getPose().x, 48, 2000, {.forwards = false, .maxSpeed = 100});
    robot.turnToHeading(185, 1000);
    pros::delay(900000000);
}

// 7 ball right
void elim_right_blue() {    
    intakeState = STORE;
    wing.extend();
    robot.moveToPoint(0, 7, 1000, {.minSpeed=80, .earlyExitRange=3});
    robot.turnToHeading(50, 700, {.earlyExitRange=3});
    robot.moveToPoint(13.5, 22.5, 2000, {.maxSpeed=100, .earlyExitRange=3});
    pros::delay(370);
    doinker.extend();
    robot.turnToHeading(-47, 700, {.earlyExitRange=3});
    robot.moveToPoint(34.25, 3.0, 3000, {.forwards = false, .minSpeed = 80, .earlyExitRange=5});
    robot.turnToHeading(-175, 800, {.earlyExitRange=3});
    robot.moveToPoint(36.5, -40, 1500, {.maxSpeed=40});
    robot.moveToPoint(38, 24, 1300, {.forwards = false, .maxSpeed=100});
    pros::delay(600);
    intakeState = LOW_GOAL;
    pros::delay(200);
    intakeState = HIGH_GOAL;
    pros::delay(2050);
    intakeState = STORE;
    doinker.retract();
    robot.moveToPoint(38, 16, 700, {.minSpeed = 80, .earlyExitRange = 3});
    robot.moveToPoint(27.2, 15, 1200);
    robot.turnToHeading(185, 500, {}, false);
    wing.retract();
    robot.moveToPoint(robot.getPose().x, 35, 2000, {.forwards = false, .minSpeed = 120, .earlyExitRange=3});
    robot.moveToPoint(robot.getPose().x, 48, 2000, {.forwards = false, .maxSpeed = 100});
    robot.turnToHeading(185, 1000);
    pros::delay(900000000);
}

// middle goal score
void elim_left_red() {
    intakeState = STORE;
    wing.extend();
    robot.moveToPoint(0, 10, 1500, {}, false);
    robot.turnToHeading(-52, 1500, {}, false);
    robot.moveToPoint(-14, 23, 3000, {});
    pros::delay(300);
    doinker.extend();
    robot.turnToHeading(-135, 1000);
    pros::delay(200);
    robot.moveToPoint(-1.5, 35, 2000, {.forwards = false, .maxSpeed=80}, false);
    intakeState = MEDIUM_GOAL;
    pros::delay(800);
    intakeState = STORE;
    robot.moveToPoint(-37, 2.5, 2000, {.maxSpeed=100}, false);
    robot.turnToHeading(-180, 800, {}, false);
    robot.moveToPoint(-36.5, -30, 1400, {.maxSpeed=55});
    robot.moveToPoint(-36.5, 24, 1200, {.forwards = false, .maxSpeed=60});
    pros::delay(700);
    intakeState = LOW_GOAL;
    pros::delay(50);
    intakeState = STORE;
    pros::delay(250);
    intakeState = HIGH_GOAL;
    doinker.retract();
    pros::delay(1700);
    intakeState = STORE;
    robot.moveToPoint(-36.5, 8, 1000);
    robot.turnToHeading(-270, 700);
    robot.moveToPoint(-46, 8, 1000, {.forwards = false}, false);
    robot.turnToHeading(-180, 700, {}, false);
    wing.retract();
    robot.moveToPoint(-46, 38, 2000, {.forwards = false, .maxSpeed=100}, false);
    robot.turnToHeading(-185, 1000);
}

// low goal score
void elim_right_red() {
    intakeState = STORE;
    wing.extend();
    robot.moveToPoint(0, 10, 1500, {}, false);
    robot.turnToHeading(48, 800, {}, false);
    robot.moveToPoint(13.5, 22.5, 3000, {.maxSpeed=60}, false);
    robot.turnToHeading(-44, 700);
    doinker.retract();
    robot.moveToPoint(5.6,  30.8, 2000, {.maxSpeed=80}, false);
    intakeState = LOW_GOAL;
    pros::delay(1500);
    intakeState = STORE;
    robot.turnToHeading(-45, 700);
    robot.moveToPoint(36, 0, 3000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(-175, 800, {}, false);
    robot.moveToPoint(38.5, -20, 1100, {.maxSpeed=70});
    robot.moveToPoint(38.5, 24, 1300, {.forwards = false, .maxSpeed=100});
    pros::delay(800);
    intakeState = LOW_GOAL;
    pros::delay(200);
    intakeState = HIGH_GOAL;
    pros::delay(1950);
    intakeState = STORE;
    doinker.retract();
    robot.moveToPoint(38.5, 9, 1000);
    robot.turnToHeading(90, 700);
    robot.moveToPoint(29.5, 9, 1000, {.forwards = false}, false);
    robot.turnToHeading(180, 700, {}, false);
    wing.retract();
    robot.moveToPoint(29.5, 40, 2000, {.forwards = false}, false);
}

void fastpush_left() {
    intakeState = STORE;
    robot.setPose(1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(90, 500);
    robot.moveToPoint(20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(-25, -35.5, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(1300);
    doinker.retract();
    robot.moveToPoint(-16, -35.5, 1000, {.minSpeed=90, .earlyExitRange=3});
    robot.moveToPoint(-24, -50, 1000);
    robot.turnToHeading(90, 1000, {}, false);
    robot.moveToPoint(-44, -45, 2000, {.forwards = false, .maxSpeed=90});
}

void fastpush_right(){
    intakeState = STORE;
    robot.setPose(-1.5, -2.3, 0);
    robot.moveToPoint(0, -32, 2000, {.forwards = false}, false);
    doinker.extend();
    robot.turnToHeading(-90, 500);
    robot.moveToPoint(-20, -30, 300, {.minSpeed=100, .earlyExitRange=3});
    robot.moveToPoint(-20, -30, 800, {.maxSpeed=40}, false);
    robot.moveToPoint(25, -35, 1200, {.forwards = false, .maxSpeed=80});
    pros::delay(800);
    intakeState = HIGH_GOAL;
    pros::delay(1300);
    doinker.retract();
    intakeState = STORE;
    robot.moveToPoint(17, -34, 1000, {.minSpeed=90, .earlyExitRange=3});
    robot.moveToPoint(17, -24, 1000);
    robot.turnToHeading(-90, 1000, {}, false);
    robot.moveToPoint(44, -24.5, 2000, {.forwards = false, .maxSpeed=90});
}

void select_auto() {
    switch (currentAutoState) {
        case AWP_LEFT_BLUE:
            awp_left_blue();
            break;
        case AWP_RIGHT_BLUE:
            awp_right_blue();
            break;
        case AWP_LEFT_RED:
            awp_left_red();
            break;
        case AWP_RIGHT_RED:
            awp_right_red();
            break;
        case ELIM_LEFT_BLUE:
            elim_left_blue();
            break;
        case ELIM_RIGHT_BLUE:
            elim_right_blue();
            break;
        case ELIM_LEFT_RED:
            elim_left_red();
            break;
        case ELIM_RIGHT_RED:
            elim_right_red();
            break;
        case FAST_PUSH_LEFT:
            fastpush_left();
            break;
        case FAST_PUSH_RIGHT:
            fastpush_right();
            break;
        case SKILLS:
            skills();
            break;
        default:
            break;
    }
}

void shimmy_shake() {
    robot.swingToHeading(robot.getPose().theta + 30, DriveSide::RIGHT, 200, {.direction=AngularDirection::CW_CLOCKWISE});
    robot.swingToHeading(robot.getPose().theta - 30, DriveSide::LEFT, 200, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE});
    robot.swingToHeading(robot.getPose().theta + 30, DriveSide::RIGHT, 200, {.direction=AngularDirection::CW_CLOCKWISE});
    robot.swingToHeading(robot.getPose().theta - 30, DriveSide::LEFT, 200, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE}, false);
}