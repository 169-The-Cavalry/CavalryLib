#include "globals.hpp"
#include "cavalry/robot/particlefilter.hpp"

const std::vector<std::string> STATE_MAP = {
    "STORE",
    "HIGH",
    "MEDIUM",
    "LOW",
    "OFF"
};

const std::vector<std::string> AUTO_MAP = {
    "MGI",
    "LGI", 
    "MGH",
    "SOLO",
    "7BL",
    "7BR",
    "MGS",
    "LGS",
    "FPL",
    "FPR",
    "SKILLS"
};

const std::vector<cavalry::Pose> AUTO_STARTING_POSITIONS = {
    cavalry::Pose(-15, -45.5, 0),    // AWP_LEFT_BLUE
    cavalry::Pose(15, -45.5, 0),     // AWP_RIGHT_BLUE
    cavalry::Pose(-15, -45.5, 0),    // AWP_LEFT_RED
    cavalry::Pose(21, -48, 90),      // AWP_RIGHT_RED
    cavalry::Pose(-16.5, -46, 0),    // ELIM_LEFT_BLUE
    cavalry::Pose(16.5, -47.5, -90), // ELIM_RIGHT_BLUE
    cavalry::Pose(-15, -45.5, 0),    // ELIM_LEFT_RED
    cavalry::Pose(15, -45.5, 0),     // ELIM_RIGHT_RED
    cavalry::Pose(-15, -45.5, 0),    // ELIM_LEFT_RED
    cavalry::Pose(15, -45.5, 0),     // ELIM_RIGHT_RED
    cavalry::Pose(-10.5, -46, 0)    // SKILLS
};

// VEX definitons (declared in globals.hpp)
pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::vector<std::int8_t> rightDrivePorts = {DRIVE_RIGHT_FRONT, DRIVE_RIGHT_MIDDLE, DRIVE_RIGHT_BACK};
pros::MotorGroup rightDrive(rightDrivePorts, DRIVE_GEARSET);

std::vector<std::int8_t> leftDrivePorts = {DRIVE_LEFT_FRONT, DRIVE_LEFT_MIDDLE, DRIVE_LEFT_BACK};
pros::MotorGroup leftDrive (leftDrivePorts, DRIVE_GEARSET);

pros::Motor intakeLow(INTAKE_LOW_PORT, INTAKE_GEARSET);
pros::Motor intakeHigh(INTAKE_HIGH_PORT, INTAKE_GEARSET);

pros::IMU imu1(IMU_1_PORT);
pros::IMU imu2(IMU_2_PORT);
pros::Rotation verticalEncoder(VERTICAL_ENCODER_PORT);

pros::adi::Pneumatics doinker(DOINKER_PORT, false);
pros::adi::Pneumatics wing(WING_PORT, false);
pros::adi::Pneumatics mediumGoal(MEDIUM_GOAL_PORT, false);

// Cavalry definitions (declared in globals.cpp)
cavalry::TrackingWheel verticalTracker(&verticalEncoder, TRACKING_WHEELS, VERTICAL_OFFSET);
cavalry::OdomSensors odomSensors(&verticalTracker, nullptr, nullptr, nullptr, &imu1, &imu2);

cavalry::Drivetrain drivetrain(
    &leftDrive, 
    &rightDrive, 
    TRACK_WIDTH, 
    DRIVE_WHEELS, 
    RPM, 
    HORIZONTAL_DRIFT
);
cavalry::ControllerSettings drivePID(
    DRIVE_P, 
    DRIVE_I, 
    DRIVE_D, 
    DRIVE_ANTI_WINDUP, 
    DRIVE_SMALL_ERROR, 
    DRIVE_SMALL_TIMEOUT, 
    DRIVE_LARGE_ERROR, 
    DRIVE_LARGE_TIMEOUT, 
    DRIVE_ACCELERATION
);
cavalry::ControllerSettings turnPID(
    TURN_P, 
    TURN_I, 
    TURN_D, 
    TURN_ANTI_WINDUP, 
    TURN_SMALL_ERROR, 
    TURN_SMALL_TIMEOUT, 
    TURN_LARGE_ERROR, 
    TURN_LARGE_TIMEOUT, 
    TURN_ACCELERATION
);
cavalry::ExpoDriveCurve throttleCurve(3, 10, 1.019);
cavalry::ExpoDriveCurve steerCurve(3, 10, 1.019);
cavalry::Robot robot(
    drivetrain, 
    drivePID, 
    turnPID, 
    odomSensors, 
    &throttleCurve, 
    &steerCurve,
    &particleFilter
);

// MCL definitions (declared in globals.hpp)
cavalry::Pose distanceRightOffset(DISTANCE_RIGHT_OFFSET_X, DISTANCE_RIGHT_OFFSET_Y, DISTANCE_RIGHT_OFFSET_T);
cavalry::Pose distanceLeftOffset(DISTANCE_LEFT_OFFSET_X, DISTANCE_LEFT_OFFSET_Y, DISTANCE_LEFT_OFFSET_T);
cavalry::Pose distanceFrontOffset(DISTANCE_FRONT_OFFSET_X, DISTANCE_FRONT_OFFSET_Y, DISTANCE_FRONT_OFFSET_T);
cavalry::Pose distanceBackOffset(DISTANCE_BACK_OFFSET_X, DISTANCE_BACK_OFFSET_Y, DISTANCE_BACK_OFFSET_T);

cavalry::DistanceSensor distanceSensorRight(DISTANCE_RIGHT_PORT, distanceRightOffset, DISTANCE_RIGHT_CONSTANT);
cavalry::DistanceSensor distanceSensorLeft(DISTANCE_LEFT_PORT, distanceLeftOffset, DISTANCE_LEFT_CONSTANT);
cavalry::DistanceSensor distanceSensorFront(DISTANCE_FRONT_PORT, distanceFrontOffset, DISTANCE_FRONT_CONSTANT);
cavalry::DistanceSensor distanceSensorBack(DISTANCE_BACK_PORT, distanceBackOffset, DISTANCE_BACK_CONSTANT);

cavalry::ParticleFilterSensors particleFilterSensors(
    {&distanceSensorRight, &distanceSensorLeft, &distanceSensorFront, &distanceSensorBack},
    &imu1, &imu2,
    &verticalTracker, nullptr
);
cavalry::ParticleFilterSettings particleFilterSettings(
    NUM_PARTICLES,
    VERTICAL_NOISE,
    HORIZONTAL_NOISE,
    ANGLE_NOISE,
    MAX_DISTANCE,
    MAX_TIME,
    START_RADIUS
);

cavalry::ParticleFilter particleFilter(particleFilterSensors, particleFilterSettings, false);

// Runtime variable definitons
DriveState driveState = AUTONOMOUS;
DriveState previousDriveState = driveState;
AutoState currentAutoState = ELIM_RIGHT_BLUE;
int intakeState = OFF;
int intakeStateHidden = STORE;
bool stateJustSwitched = false;
