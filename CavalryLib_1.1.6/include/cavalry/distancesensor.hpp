#pragma once

#include "pros/distance.hpp"
#include "cavalry/pose.hpp"
#include "cavalry/geom.hpp"

namespace cavalry {
    
constexpr auto DIST_WALL_FROM_ZERO = 71.0;
constexpr auto PARTICLE_UNCERTAINTY = 0.05f;
constexpr auto METERS_TO_INCHES = 39.3701;
constexpr auto INCHES_TO_METERS = 0.0254;

class DistanceSensor {
private:
    pros::Distance distance;

    cavalry::Pose offset;

    float measured = 0.0f;
    float standardDeviation = 0.0f;
    float tuningConstant = 0.0f;

    int objectSize = 0;

    bool validReading = true;
    bool disabled = false;

    Rectangle fieldBoundary = {
        {0.0f, 0.0f},
        2.0f * DIST_WALL_FROM_ZERO,
        2.0f * DIST_WALL_FROM_ZERO,
        0.0f
    };

public:
    DistanceSensor(uint8_t port, const cavalry::Pose& offset, float tuningConstant);

    void update();

    std::optional<double> p(const cavalry::Pose& particle, const std::vector<Circle>& circles, const std::vector<Rectangle>& rectangles, const std::vector<Polygon>& polygons);

    cavalry::Ray get_ray(const cavalry::Pose& particle);

    float get_simulated_distance(const cavalry::Ray& ray, const std::vector<Circle>& circles, const std::vector<Rectangle>& rectangles, const std::vector<Polygon>& polygons);

    float get_distance() const;

    int get_object_size() const;

    bool is_valid_reading() const;

    float get_standard_deviation() const;

    float return_confidence();

    void disable_sensor();

    void enable_sensor();

    bool is_disabled() const;

    ~DistanceSensor();

private:
    float calculate_std_dev(float measured);

    float normal_pdf(const float x, const float mean, const float stddev);

    float fast_normal_pdf(const float x);
};
} // namespace cavalry