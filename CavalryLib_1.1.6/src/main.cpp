#include "main.h"
#include "cavalry/distancesensor.hpp"
#include "cavalry/geom.hpp"
#include "globals.hpp"
#include "tasks.hpp"
#include "pros/rtos.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    create_field_map();
    if (currentAutoState == SKILLS){
        distanceSensorFront.disable_sensor();
        particleFilter.set_starting_pose(AUTO_STARTING_POSITIONS[currentAutoState], true);
        particleFilter.set_enabled(true);
    }

    robot.calibrate();

    intakeHigh.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::Task handleIntakeTask(handle_intake_task);
    pros::Task printScreenTask(screen_print_task);
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

void create_field_map() {
    
    const float matchLoaderWidth = 4.5;
    const float matchLoaderLength = 4.5;

    const float longGoalSupportWidth = 6.0;
    const float longGoalSupportLength = 9.0;

    const float shortGoalSupportWidth = 5.0;
    const float shortGoalSupportLength = 5.0;


    cavalry::Rectangle matchLoaderRedRight = {
        {48, static_cast<float>(-cavalry::DIST_WALL_FROM_ZERO + matchLoaderWidth / 2.0)},
        matchLoaderWidth, matchLoaderLength, 0
    };
    particleFilter.add_rectangle(matchLoaderRedRight);

    cavalry::Rectangle matchLoaderRedLeft = {
        {-48, static_cast<float>(-cavalry::DIST_WALL_FROM_ZERO + matchLoaderWidth / 2.0)},
        matchLoaderWidth, matchLoaderLength, 0
    };
    particleFilter.add_rectangle(matchLoaderRedLeft);

    cavalry::Rectangle matchLoaderBlueRight = {
        {48, static_cast<float>(cavalry::DIST_WALL_FROM_ZERO - matchLoaderWidth / 2.0)},
        matchLoaderWidth, matchLoaderLength, 0
    };
    particleFilter.add_rectangle(matchLoaderBlueRight);

    cavalry::Rectangle matchLoaderBlueLeft = {
        {-48, static_cast<float>(cavalry::DIST_WALL_FROM_ZERO - matchLoaderWidth / 2.0)},
        matchLoaderWidth, matchLoaderLength, 0
    };
    particleFilter.add_rectangle(matchLoaderBlueLeft);

    cavalry::Rectangle longGoalSupportRedRight = {
        {48, -24}, 
        longGoalSupportLength, longGoalSupportWidth, 0
    };
    particleFilter.add_rectangle(longGoalSupportRedRight);

    cavalry::Rectangle longGoalSupportRedLeft = {
        {-48, -24}, 
        longGoalSupportLength, longGoalSupportWidth, 0
    };
    particleFilter.add_rectangle(longGoalSupportRedLeft);

    cavalry::Rectangle longGoalSupportBlueRight = {
        {48, 24}, 
        longGoalSupportLength, longGoalSupportWidth, 0
    };
    particleFilter.add_rectangle(longGoalSupportBlueRight);

    cavalry::Rectangle longGoalSupportBlueLeft = {
        {-48, 24}, 
        longGoalSupportLength, longGoalSupportWidth, 0
    };
    particleFilter.add_rectangle(longGoalSupportBlueLeft);

    cavalry::Rectangle centerGoalZone = {
        {0, 0}, 
        shortGoalSupportLength, shortGoalSupportWidth, 45
    };
    particleFilter.add_rectangle(centerGoalZone);
}