#pragma once

#include "pros/imu.hpp"
#include "cavalry/distancesensor.hpp"
#include "cavalry/pose.hpp"
#include "cavalry/geom.hpp"
#include "cavalry/robot/trackingWheel.hpp"

#include <vector>
#include <random>

namespace cavalry {

class ParticleFilterSettings {
public:
    ParticleFilterSettings(int numParticles,
                           float verticalNoise,
                           float horizontalNoise,
                           float angularNoise,
                           float maxDistance,
                           float maxTime,
                           float startRadius)
        : numParticles(numParticles),
          verticalNoise(verticalNoise),
          angularNoise(angularNoise),
          horizontalNoise(horizontalNoise),
          maxDistance(maxDistance),
          maxTime(maxTime),
          startRadius(startRadius) {}
    int numParticles;
    float verticalNoise;
    float horizontalNoise;
    float angularNoise;
    float maxDistance;
    float maxTime;
    float startRadius;
};

class ParticleFilterSensors {
public:
    ParticleFilterSensors(std::vector<DistanceSensor*> distanceSensors,
                        pros::IMU* imu1, 
                        pros::IMU* imu2 = nullptr,
                        cavalry::TrackingWheel* verticalWheel = nullptr,
                        cavalry::TrackingWheel* horizontalWheel = nullptr)
        : distanceSensors(std::move(distanceSensors)), imu1(imu1), imu2(imu2), verticalWheel(verticalWheel), horizontalWheel(horizontalWheel) {}

    pros::IMU* imu1;
    pros::IMU* imu2;
    std::vector<DistanceSensor*> distanceSensors;
    cavalry::TrackingWheel* verticalWheel;
    cavalry::TrackingWheel* horizontalWheel;
};


class ParticleFilter {
private:
    int NUM_PARTICLES = 0;

    std::vector<std::pair<float, float>> particles;
    std::vector<std::pair<float, float>> oldParticles;
    std::vector<float> weights;

    pros::IMU* imu1 = nullptr;
    pros::IMU* imu2 = nullptr;
    std::vector<DistanceSensor*> sensors;
    cavalry::TrackingWheel* verticalWheel = nullptr;
    cavalry::TrackingWheel* horizontalWheel = nullptr;
    
    std::vector<Circle> circles;
    std::vector<Rectangle> rectangles;
    std::vector<Polygon> polygons;

    cavalry::Pose startPose = cavalry::Pose(0, 0, 0);
    cavalry::Pose prediction = cavalry::Pose(0, 0, 0);

    std::uniform_real_distribution<> fieldDistribution {-DIST_WALL_FROM_ZERO, DIST_WALL_FROM_ZERO};
    
    std::normal_distribution<> verticalDistribution;
    std::normal_distribution<> horizontalDistribution;
    std::normal_distribution<> angularDistribution;

    std::mt19937 randomGenerator;
    ParticleFilterSettings settings;

    float lastOdomVertical = 0.0f;
    float lastOdomHorizontal = 0.0f;
    float distanceSinceLastUpdate = 0.0f;
    float timeSinceLastUpdate = 0.0f;
    float timeLastUpdate = 0.0f;
    bool startPoseInitialized = false;
    bool enabled = false;

public:
    ParticleFilter(ParticleFilterSensors sensors, ParticleFilterSettings settings, bool enabled);

    void init_uniform(float xMin, float xMax, float yMin, float yMax);

    void init_normal(const cavalry::Pose& mean, float stdDevX, float stdDevY);

    void update();

    void add_sensor(DistanceSensor* sensor);

    void add_circle(Circle circle);

    void add_rectangle(Rectangle rectangle);

    void add_polygon(Polygon polygon);

    void set_starting_pose(const cavalry::Pose& pose, bool normal);

    void set_enabled(bool enabled);

    cavalry::Pose get_prediction() const;

    std::vector<std::pair<float, float>> get_particles() const;

    std::vector<float> get_weights() const;

    const std::vector<Circle>& get_circles() const;

    const std::vector<Rectangle>& get_rectangles() const;

    const std::vector<Polygon>& get_polygons() const;

    const std::vector<DistanceSensor*>& get_sensors() const;

    bool is_initialized();

    bool is_enabled();

private:
    void create_distributions(float deltaOdomVertical, float deltaOdomHorizontal, float currentTheta);

    void update_sensors();

    float weight_particles(float currentTheta);

    std::pair<double, double> resample_particles(float weightSum);

    float weight_particle(const cavalry::Pose& particle);

    cavalry::Pose get_noisy_motion();

    float calculate_norm(const cavalry::Pose& pose);

    bool out_of_field(const std::pair<float, float>& particle);
};
} // namespace cavalry