#include "tasks.hpp"

void screen_print_task(void* args) {
    (void)args;

    while (true) {
        const cavalry::Pose pose = robot.getPose();

        pros::screen::set_pen(pros::Color::black);
        pros::screen::fill_rect(0, 0, 480, 240);

        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Cavalry Template Telemetry");
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "X: %.2f", pose.x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Y: %.2f", pose.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Heading: %.2f", pose.theta);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Auto Mode: %d", static_cast<int>(currentAutoState));

        // Mirror key data on controller for quick checks on the field.
        controller.print(0, 0, "X:%4.1f Y:%4.1f", pose.x, pose.y);
        controller.print(1, 0, "A:%d I:%d", static_cast<int>(currentAutoState), intakeEnabled ? 1 : 0);

        pros::delay(50);
    }
}

void subsystem_template_task(void* args) {
    (void)args;

    while (true) {
        // Put optional non-drive background logic here.
        // Examples:
        // - Flywheel velocity regulation
        // - Lift hold control
        // - Sensor filtering
        pros::delay(TASK_DELAY);
    }
}