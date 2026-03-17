#include "opcontrol.hpp"
#include "cavalry/distancesensor.hpp"
#include "globals.hpp"
#include "main.h"

void opcontrol() {
    intakeState = OFF;
    driveState = DRIVER;
    
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        robot.tank(leftY, rightY);

        handle_intake();
        handle_wings();
        handle_doinker();   
        handle_medium_goal();
        handle_middle_toggle();
        handle_mcl_toggle();
        handle_intake_dejam();

		pros::delay(TASK_DELAY);
	}
}

void handle_intake(){
    if (controller.get_digital_new_press(INTAKE_FORWARD_TOGGLE)) {
        intakeState = (intakeState == OFF || intakeState == LOW_GOAL) ? intakeStateHidden : OFF;
    }
    else if (controller.get_digital_new_press(INTAKE_STATE_FORWARD_TOGGLE)) {
        if (intakeStateHidden == STORE) intakeStateHidden = HIGH;
        else if (intakeStateHidden == HIGH) intakeStateHidden = STORE;
        else intakeStateHidden = STORE;

        if (intakeState != OFF) intakeState = intakeStateHidden;
        stateJustSwitched = true;
    }
    else if (controller.get_digital_new_press(INTAKE_STATE_BACKWARD_TOGGLE)){
        if (intakeStateHidden == STORE) intakeStateHidden = HIGH;
        else if (intakeStateHidden == HIGH) intakeStateHidden = STORE;
        else intakeStateHidden = STORE;
        
        if (intakeState != OFF) intakeState = intakeStateHidden;
        stateJustSwitched = true;
    }
    else if (controller.get_digital_new_press(INTAKE_BACKWARD_TOGGLE)){
        intakeState = (intakeState == LOW_GOAL) ? OFF : LOW_GOAL;
    }
}

void handle_wings() {
    if (controller.get_digital_new_press(WINGS_TOGGLE)) {
        wing.toggle();
    }
}

void handle_doinker() {
    if (controller.get_digital_new_press(DOINK_TOGGLE)) {
        doinker.toggle();
    }
}

void handle_medium_goal() {
    if (controller.get_digital_new_press(MEDIUM_GOAL_TOGGLE)) {
        autonomous();
    }
}

void handle_middle_toggle() {
    if (controller.get_digital_new_press(INTAKE_MIDDLE_TOGGLE)) {
        if (intakeState == MEDIUM_GOAL) {
            intakeState = OFF;
            intakeStateHidden = STORE;
        } else {
            intakeStateHidden = MEDIUM_GOAL;
            intakeState = MEDIUM_GOAL;
        }
        stateJustSwitched = true;
    }
}

void handle_mcl_toggle() {
    if (controller.get_digital_new_press(MCL_TOGGLE)) {
        //particleFilter.set_enabled(particleFilter.is_enabled() ? false : true);
        if (distanceSensorFront.is_disabled()) distanceSensorFront.enable_sensor();
        else distanceSensorFront.disable_sensor();
    }
}

void handle_intake_dejam() {
    if (controller.get_digital_new_press(INTAKE_DEJAM_TOGGLE)){
        intakeStateHidden = intakeState;
        intakeState = DEJAM;
        stateJustSwitched = true;
    }
}