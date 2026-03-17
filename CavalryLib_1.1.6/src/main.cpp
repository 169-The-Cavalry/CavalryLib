#include "main.h"

#include "globals.hpp"
#include "tasks.hpp"

void initialize() {
    // 1) Calibrate all configured sensors.
    // Keep robot still during this step.
    robot.calibrate();

    // 2) Set your preferred autonomous default here.
    currentAutoState = AUTO_TEST_DRIVE;

    // 3) Set your odometry start pose.
    // TODO: Change this per your starting tile and heading.
    robot.setPose(0.0, 0.0, 0.0);

    // 4) Start optional background tasks.
    static pros::Task telemetryTask(screen_print_task);
    (void)telemetryTask;

    // Example: enable if you implement subsystem_template_task logic.
    // static pros::Task subsystemTask(subsystem_template_task);
}

void disabled() {
    // Runs repeatedly while disabled.
    // Good place for autonomous selection or sensor checks.
}

void competition_initialize() {
    // Runs once after initialize() before autonomous/opcontrol in comp mode.
    // Put LCD or controller-based autonomous selector setup here.
}