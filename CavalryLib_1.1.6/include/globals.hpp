#pragma once

#include "cavalry/api.hpp"
#include "main.h"

#include <cstdint>
#include <vector>

// ------------------------------
// Robot tuning constants
// ------------------------------
// TODO: Tune these PID values for your drivetrain.
constexpr double DRIVE_P = 7.5;
constexpr double DRIVE_I = 0.0;
constexpr double DRIVE_D = 35.0;
constexpr double DRIVE_ANTI_WINDUP = 0.0;
constexpr double DRIVE_SMALL_ERROR = 1.0;
constexpr double DRIVE_SMALL_TIMEOUT = 30.0;
constexpr double DRIVE_LARGE_ERROR = 3.0;
constexpr double DRIVE_LARGE_TIMEOUT = 100.0;
constexpr double DRIVE_ACCELERATION = 0.0;

// TODO: Tune these turn PID values for your drivetrain.
constexpr double TURN_P = 2.5;
constexpr double TURN_I = 0.0;
constexpr double TURN_D = 15.0;
constexpr double TURN_ANTI_WINDUP = 0.0;
constexpr double TURN_SMALL_ERROR = 1.0;
constexpr double TURN_SMALL_TIMEOUT = 25.0;
constexpr double TURN_LARGE_ERROR = 3.0;
constexpr double TURN_LARGE_TIMEOUT = 50.0;
constexpr double TURN_ACCELERATION = 0.0;

// ------------------------------
// Controller bindings
// ------------------------------
// TODO: Change these buttons to match your driver preference.
constexpr auto INTAKE_TOGGLE_BTN = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto INTAKE_REVERSE_BTN = pros::E_CONTROLLER_DIGITAL_R2;
constexpr auto WING_TOGGLE_BTN = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto DOINKER_TOGGLE_BTN = pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto AUTO_CYCLE_BTN = pros::E_CONTROLLER_DIGITAL_X;

// ------------------------------
// Ports (example layout)
// ------------------------------
// TODO: Replace all ports below with your robot's actual ports.
constexpr std::int8_t DRIVE_RIGHT_FRONT = -19;
constexpr std::int8_t DRIVE_RIGHT_MIDDLE = 9;
constexpr std::int8_t DRIVE_RIGHT_BACK = -15;
constexpr std::int8_t DRIVE_LEFT_FRONT = 3;
constexpr std::int8_t DRIVE_LEFT_MIDDLE = -2;
constexpr std::int8_t DRIVE_LEFT_BACK = 1;

constexpr std::int8_t INTAKE_LOW_PORT = -10;
constexpr std::int8_t INTAKE_HIGH_PORT = 6;

constexpr std::int8_t IMU_1_PORT = 4;
constexpr std::int8_t IMU_2_PORT = 21;
constexpr std::int8_t VERTICAL_ENCODER_PORT = -17;

constexpr char DOINKER_PORT = 'D';
constexpr char WING_PORT = 'C';

// ------------------------------
// Drivetrain model constants
// ------------------------------
// TODO: Update wheel type, track width, and drift to your drivetrain.
constexpr auto TRACK_WIDTH = 11.0;
constexpr auto DRIVE_WHEELS = cavalry::Omniwheel::NEW_325;
constexpr auto RPM = 450;
constexpr auto HORIZONTAL_DRIFT = 2.0;

// TODO: Use the matching wheel and offset for your tracking encoder.
constexpr auto TRACKING_WHEELS = cavalry::Omniwheel::NEW_2;
constexpr auto VERTICAL_OFFSET = 1.5;

// Motor gearsets
constexpr auto DRIVE_GEARSET = pros::MotorGearset::blue;
constexpr auto INTAKE_GEARSET = pros::MotorGearset::blue;

// Utility constants
constexpr int TASK_DELAY = 10;
constexpr int DRIVE_DEADBAND = 5;
constexpr int INTAKE_SPEED = 127;

// ------------------------------
// Hardware objects (defined in src/globals.cpp)
// ------------------------------
extern pros::Controller controller;

extern std::vector<std::int8_t> rightDrivePorts;
extern std::vector<std::int8_t> leftDrivePorts;
extern pros::MotorGroup rightDrive;
extern pros::MotorGroup leftDrive;

extern pros::Motor intakeLow;
extern pros::Motor intakeHigh;

extern pros::IMU imu1;
extern pros::IMU imu2;
extern pros::Rotation verticalEncoder;

extern pros::adi::Pneumatics doinker;
extern pros::adi::Pneumatics wing;

// ------------------------------
// Cavalry objects (defined in src/globals.cpp)
// ------------------------------
extern cavalry::TrackingWheel verticalTracker;
extern cavalry::OdomSensors odomSensors;
extern cavalry::Drivetrain drivetrain;
extern cavalry::ControllerSettings drivePID;
extern cavalry::ControllerSettings turnPID;
extern cavalry::ExpoDriveCurve throttleCurve;
extern cavalry::ExpoDriveCurve steerCurve;
extern cavalry::Robot robot;

// ------------------------------
// Runtime states
// ------------------------------
enum DriveState {
    DRIVER,
    AUTONOMOUS
};

enum AutoState {
    AUTO_DO_NOTHING,
    AUTO_TEST_DRIVE,
    AUTO_SKILLS_TEMPLATE
};

extern DriveState driveState;
extern AutoState currentAutoState;

extern bool intakeEnabled;
extern bool intakeReversed;
extern bool wingExtended;
extern bool doinkerExtended;