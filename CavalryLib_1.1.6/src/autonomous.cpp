#include "autonomous.hpp"

void autonomous() {
    driveState = AUTONOMOUS;
    run_autonomous();
}

void run_autonomous() {
    switch (currentAutoState) {
        case AUTO_DO_NOTHING:
            auto_do_nothing();
            break;
        case AUTO_TEST_DRIVE:
            auto_test_drive();
            break;
        case AUTO_SKILLS_TEMPLATE:
            auto_skills_template();
            break;
        default:
            auto_do_nothing();
            break;
    }
}

void auto_do_nothing() {
    // Safe fallback autonomous.
    pros::delay(15000);
}

void auto_test_drive() {
    // Replace this routine with your match autonomous path.
    // Tip: Start with short, reliable movements before adding complexity.
    robot.setPose(0.0, 0.0, 0.0);
    robot.moveToPoint(0.0, 24.0, 1500);
    robot.turnToHeading(90.0, 1000);
    robot.moveToPoint(24.0, 24.0, 1500);
}

void auto_skills_template() {
    // Put your full skills routine here.
    // Suggested structure:
    // 1) Score preload
    // 2) Collect and score cycles
    // 3) Endgame actions
    robot.setPose(0.0, 0.0, 0.0);

    // Example opening movement.
    robot.moveToPoint(0.0, 18.0, 1200);

    // TODO: Add manipulator commands here (intake, pneumatics, etc.).
    pros::delay(500);
}