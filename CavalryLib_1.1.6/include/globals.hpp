#pragma once
#include "main.h"
#include "cavalry/api.hpp"
#include "pros/apix.h"
#include "pros/misc.h"

#include <iostream>
#include <format>

// PID constants
constexpr double DRIVE_P = 7.5;
constexpr double DRIVE_I = 0.0;
constexpr double DRIVE_D = 35.0;
constexpr double DRIVE_ANTI_WINDUP = 0.0;
constexpr double DRIVE_SMALL_ERROR = 1.0;
constexpr double DRIVE_SMALL_TIMEOUT = 30.0;
constexpr double DRIVE_LARGE_ERROR = 3.0;
constexpr double DRIVE_LARGE_TIMEOUT = 100.0;
constexpr double DRIVE_ACCELERATION = 0.0;

constexpr double TURN_P = 2.5;
constexpr double TURN_I = 0.0;
constexpr double TURN_D = 15.0;
constexpr double TURN_ANTI_WINDUP = 0.0;
constexpr double TURN_SMALL_ERROR = 1.0;
constexpr double TURN_SMALL_TIMEOUT = 25.0; 
constexpr double TURN_LARGE_ERROR = 3.0;
constexpr double TURN_LARGE_TIMEOUT = 50.0;
constexpr double TURN_ACCELERATION = 0.0;

// Controller bindings
constexpr auto INTAKE_FORWARD_TOGGLE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto INTAKE_MIDDLE_TOGGLE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto INTAKE_STATE_FORWARD_TOGGLE = pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto INTAKE_STATE_BACKWARD_TOGGLE = pros::E_CONTROLLER_DIGITAL_R2;
constexpr auto INTAKE_BACKWARD_TOGGLE = pros::E_CONTROLLER_DIGITAL_X;
constexpr auto DOINK_TOGGLE = pros::E_CONTROLLER_DIGITAL_UP;
constexpr auto WINGS_TOGGLE = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr auto MEDIUM_GOAL_TOGGLE = pros::E_CONTROLLER_DIGITAL_LEFT;
constexpr auto MCL_TOGGLE = pros::E_CONTROLLER_DIGITAL_A;
constexpr auto INTAKE_DEJAM_TOGGLE = pros::E_CONTROLLER_DIGITAL_Y;

// VEX ports
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

constexpr std::int8_t DISTANCE_RIGHT_PORT = 16;
constexpr std::int8_t DISTANCE_LEFT_PORT = 7;
constexpr std::int8_t DISTANCE_FRONT_PORT = 18;
constexpr std::int8_t DISTANCE_BACK_PORT = 5;

constexpr char DOINKER_PORT = 'D';
constexpr char WING_PORT = 'C';
constexpr char MEDIUM_GOAL_PORT = 'B';

// Drivetrain constants
constexpr auto TRACK_WIDTH = 11.0;
constexpr auto DRIVE_WHEELS = cavalry::Omniwheel::NEW_325;
constexpr auto RPM = 450;
constexpr auto HORIZONTAL_DRIFT = 2;

// Motor gearsets
constexpr auto DRIVE_GEARSET = pros::MotorGearset::blue;
constexpr auto INTAKE_GEARSET = pros::MotorGearset::blue;

// Odometry constants
constexpr auto TRACKING_WHEELS = cavalry::Omniwheel::NEW_2;
constexpr auto VERTICAL_OFFSET = 1.5;

// MCL constants
constexpr auto NUM_PARTICLES = 1000;
constexpr auto VERTICAL_NOISE = 0.25;
constexpr auto HORIZONTAL_NOISE = 0.00;
constexpr auto ANGLE_NOISE = 3.0;
constexpr auto MAX_DISTANCE = 2.0;
constexpr auto MAX_TIME = 2000.0;
constexpr auto START_RADIUS = 5.0;

constexpr auto DISTANCE_RIGHT_CONSTANT = 1.0f;
constexpr auto DISTANCE_LEFT_CONSTANT = 1.0f;
constexpr auto DISTANCE_FRONT_CONSTANT = 1.0f;
constexpr auto DISTANCE_BACK_CONSTANT = 1.0f;

constexpr auto DISTANCE_RIGHT_OFFSET_X = 6.50f;
constexpr auto DISTANCE_RIGHT_OFFSET_Y = 1.25f;
constexpr auto DISTANCE_RIGHT_OFFSET_T = 90.0f;

constexpr auto DISTANCE_LEFT_OFFSET_X = -5.50f;
constexpr auto DISTANCE_LEFT_OFFSET_Y = 4.0f;
constexpr auto DISTANCE_LEFT_OFFSET_T = -90.0f;

constexpr auto DISTANCE_FRONT_OFFSET_X = 6.3f;
constexpr auto DISTANCE_FRONT_OFFSET_Y = 3.5f;
constexpr auto DISTANCE_FRONT_OFFSET_T = 0.0f;

constexpr auto DISTANCE_BACK_OFFSET_X = -3.5f;
constexpr auto DISTANCE_BACK_OFFSET_Y = -5.25f;
constexpr auto DISTANCE_BACK_OFFSET_T = 180.0f;

// Utility variables
constexpr auto TASK_DELAY = 10;
constexpr auto INTAKE_SPEED = 127;
constexpr auto INTAKE_SLOW_SPEED = 30;
constexpr auto INTAKE_DEJAM_TIME = 60;
extern const std::vector<std::string> STATE_MAP;
extern const std::vector<std::string> AUTO_MAP;
extern const std::vector<cavalry::Pose> AUTO_STARTING_POSITIONS;

// VEX declarations (defined in globals.cpp)
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
extern pros::adi::Pneumatics mediumGoal;

// Cavalry declarations (defined in globals.cpp)
extern cavalry::TrackingWheel verticalTracker;
extern cavalry::OdomSensors odomSensors;
extern cavalry::Drivetrain drivetrain;
extern cavalry::ControllerSettings lateralPID;
extern cavalry::ControllerSettings angularPID;
extern cavalry::ExpoDriveCurve throttleCurve;
extern cavalry::ExpoDriveCurve steerCurve;
extern cavalry::Robot robot;
// MCL declarations (defined in globals.cpp)
extern cavalry::DistanceSensor distanceSensorRight;
extern cavalry::DistanceSensor distanceSensorLeft;
extern cavalry::DistanceSensor distanceSensorFront;
extern cavalry::DistanceSensor distanceSensorBack;
extern cavalry::ParticleFilter particleFilter;

/**
 * Auto Drive State definitions
 * Auto Drive State is used to prevent race condition during autonomous
*/
enum DriveState {
    DRIVER,
    AUTONOMOUS,
    MACRO
};

enum IntakeState {
    STORE,
    HIGH_GOAL, 
    MEDIUM_GOAL,
    LOW_GOAL,
    OFF,
    DEJAM
};

enum AutoState {
    AWP_LEFT_BLUE,
    AWP_RIGHT_BLUE,
    AWP_LEFT_RED,
    AWP_RIGHT_RED, 
    ELIM_LEFT_BLUE,
    ELIM_RIGHT_BLUE,
    ELIM_LEFT_RED,
    ELIM_RIGHT_RED,
    FAST_PUSH_RIGHT,
    FAST_PUSH_LEFT,
    SKILLS
};

// Runtime variable declarations
extern DriveState driveState, previousDriveState;
extern AutoState currentAutoState;
extern int intakeState;
extern int intakeStateHidden;
extern bool stateJustSwitched;