#include "opcontrol.hpp"

void opcontrol() {
    driveState = DRIVER;

    while (true) {
        handle_drive_controls();
        handle_intake_controls();
        handle_pneumatic_controls();
        handle_auto_selector_controls();

        pros::delay(TASK_DELAY);
    }
}

void handle_drive_controls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // Keep the robot still when sticks are near center.
    if (std::abs(leftY) < DRIVE_DEADBAND) {
        leftY = 0;
    }
    if (std::abs(rightY) < DRIVE_DEADBAND) {
        rightY = 0;
    }

    robot.tank(leftY, rightY);
}

void handle_intake_controls() {
    if (controller.get_digital_new_press(INTAKE_TOGGLE_BTN)) {
        intakeEnabled = !intakeEnabled;
    }

    if (controller.get_digital_new_press(INTAKE_REVERSE_BTN)) {
        intakeReversed = !intakeReversed;
    }

    if (!intakeEnabled) {
        intakeLow.brake();
        intakeHigh.brake();
        return;
    }

    const int intakeCommand = intakeReversed ? -INTAKE_SPEED : INTAKE_SPEED;
    intakeLow.move(intakeCommand);
    intakeHigh.move(intakeCommand);
}

void handle_pneumatic_controls() {
    if (controller.get_digital_new_press(WING_TOGGLE_BTN)) {
        wingExtended = !wingExtended;
        if (wingExtended) {
            wing.extend();
        } else {
            wing.retract();
        }
    }

    if (controller.get_digital_new_press(DOINKER_TOGGLE_BTN)) {
        doinkerExtended = !doinkerExtended;
        if (doinkerExtended) {
            doinker.extend();
        } else {
            doinker.retract();
        }
    }
}

void handle_auto_selector_controls() {
    // Press X while disabled in the pit to cycle autonomous modes.
    if (!controller.get_digital_new_press(AUTO_CYCLE_BTN)) {
        return;
    }

    if (currentAutoState == AUTO_DO_NOTHING) {
        currentAutoState = AUTO_TEST_DRIVE;
    } else if (currentAutoState == AUTO_TEST_DRIVE) {
        currentAutoState = AUTO_SKILLS_TEMPLATE;
    } else {
        currentAutoState = AUTO_DO_NOTHING;
    }
}