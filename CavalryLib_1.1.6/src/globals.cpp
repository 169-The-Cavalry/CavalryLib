#include "globals.hpp"

// ------------------------------
// Hardware object definitions
// ------------------------------
pros::Controller controller(pros::E_CONTROLLER_MASTER);

std::vector<std::int8_t> rightDrivePorts = {
    DRIVE_RIGHT_FRONT,
    DRIVE_RIGHT_MIDDLE,
    DRIVE_RIGHT_BACK
};
pros::MotorGroup rightDrive(rightDrivePorts, DRIVE_GEARSET);

std::vector<std::int8_t> leftDrivePorts = {
    DRIVE_LEFT_FRONT,
    DRIVE_LEFT_MIDDLE,
    DRIVE_LEFT_BACK
};
pros::MotorGroup leftDrive(leftDrivePorts, DRIVE_GEARSET);

pros::Motor intakeLow(INTAKE_LOW_PORT, INTAKE_GEARSET);
pros::Motor intakeHigh(INTAKE_HIGH_PORT, INTAKE_GEARSET);

pros::IMU imu1(IMU_1_PORT);
pros::IMU imu2(IMU_2_PORT);
pros::Rotation verticalEncoder(VERTICAL_ENCODER_PORT);

pros::adi::Pneumatics doinker(DOINKER_PORT, false);
pros::adi::Pneumatics wing(WING_PORT, false);

// ------------------------------
// Cavalry object definitions
// ------------------------------
// Only a vertical tracking wheel is configured by default.
// Add horizontal trackers or external heading sensors if your robot uses them.
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

// Particle filter is optional. Start with nullptr for a simpler template.
cavalry::Robot robot(
    drivetrain,
    drivePID,
    turnPID,
    odomSensors,
    &throttleCurve,
    &steerCurve,
    nullptr
);

// ------------------------------
// Runtime state definitions
// ------------------------------
DriveState driveState = DRIVER;
AutoState currentAutoState = AUTO_TEST_DRIVE;

bool intakeEnabled = false;
bool intakeReversed = false;
bool wingExtended = false;
bool doinkerExtended = false;