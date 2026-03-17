#pragma once
#include <vector>
#include <cmath>
#include <limits>

namespace cavalry{

struct Point {
    float x, y;
};

struct Ray {
    Point origin;
    Point direction; // Normalized
};

struct Circle {
    Point center;
    float radius;
};

struct Rectangle {
    Point center;
    float width;
    float length;
    float angle; // radians
};

struct Polygon {
    std::vector<Point> vertices;
};

namespace Geom {

    inline float distSq(Point p1, Point p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    }

    // Ray-Circle Intersection
    // Returns distance to intersection, or infinity if no intersection
    inline float intersect(const Ray& ray, const Circle& circle) {
        float ox = ray.origin.x - circle.center.x;
        float oy = ray.origin.y - circle.center.y;

        float a = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y; // Should be 1.0 if normalized
        float b = 2.0f * (ox * ray.direction.x + oy * ray.direction.y);
        float c = ox * ox + oy * oy - circle.radius * circle.radius;

        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0) {
            return std::numeric_limits<float>::infinity();
        } else {
            float t1 = (-b - std::sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + std::sqrt(discriminant)) / (2.0f * a);

            if (t1 > 0) return t1;
            if (t2 > 0) return t2;
            return std::numeric_limits<float>::infinity();
        }
    }

    // Ray-LineSegment Intersection
    // Returns distance to intersection, or infinity
    inline float intersectSegment(const Ray& ray, Point p1, Point p2) {
        float v1x = ray.origin.x - p1.x;
        float v1y = ray.origin.y - p1.y;
        float v2x = p2.x - p1.x;
        float v2y = p2.y - p1.y;
        float v3x = -ray.direction.x;
        float v3y = -ray.direction.y;

        float dot = v2x * v3y - v2y * v3x;
        if (std::abs(dot) < 0.000001f) return std::numeric_limits<float>::infinity();

        float t1 = (v2x * v1y - v2y * v1x) / dot;
        float t2 = (v3x * v1y - v3y * v1x) / dot;

        if (t1 >= 0.0f && (t2 >= 0.0f && t2 <= 1.0f)) {
            return t1;
        }
        return std::numeric_limits<float>::infinity();
    }

    // Ray-Polygon Intersection
    inline float intersect(const Ray& ray, const Polygon& poly) {
        float minDist = std::numeric_limits<float>::infinity();
        size_t n = poly.vertices.size();
        for (size_t i = 0; i < n; ++i) {
            float d = intersectSegment(ray, poly.vertices[i], poly.vertices[(i + 1) % n]);
            if (d < minDist) {
                minDist = d;
            }
        }
        return minDist;
    }

    // Ray-RotatedRectangle Intersection (local-space slab method)
    // Returns distance to intersection, or infinity
    inline float intersect(const Ray& ray, const Rectangle& rectangle) {
        constexpr float EPS = 0.000001f;

        const float halfWidth = rectangle.width * 0.5f;
        const float halfLength = rectangle.length * 0.5f;

        const float c = std::cos(rectangle.angle);
        const float s = std::sin(rectangle.angle);

        const float relOx = ray.origin.x - rectangle.center.x;
        const float relOy = ray.origin.y - rectangle.center.y;

        Ray localRay{
            { c * relOx + s * relOy, -s * relOx + c * relOy },
            { c * ray.direction.x + s * ray.direction.y, -s * ray.direction.x + c * ray.direction.y }
        };

        const float minX = -halfWidth;
        const float maxX = halfWidth;
        const float minY = -halfLength;
        const float maxY = halfLength;

        float tMin = -std::numeric_limits<float>::infinity();
        float tMax = std::numeric_limits<float>::infinity();

        if (std::abs(localRay.direction.x) < EPS) {
            if (localRay.origin.x < minX || localRay.origin.x > maxX) return std::numeric_limits<float>::infinity();
        } else {
            float tx1 = (minX - localRay.origin.x) / localRay.direction.x;
            float tx2 = (maxX - localRay.origin.x) / localRay.direction.x;
            if (tx1 > tx2) {
                float temp = tx1;
                tx1 = tx2;
                tx2 = temp;
            }
            tMin = std::fmax(tMin, tx1);
            tMax = std::fmin(tMax, tx2);
        }

        if (std::abs(localRay.direction.y) < EPS) {
            if (localRay.origin.y < minY || localRay.origin.y > maxY) return std::numeric_limits<float>::infinity();
        } else {
            float ty1 = (minY - localRay.origin.y) / localRay.direction.y;
            float ty2 = (maxY - localRay.origin.y) / localRay.direction.y;
            if (ty1 > ty2) {
                float temp = ty1;
                ty1 = ty2;
                ty2 = temp;
            }
            tMin = std::fmax(tMin, ty1);
            tMax = std::fmin(tMax, ty2);
        }

        if (tMax < tMin || tMax < 0.0f) return std::numeric_limits<float>::infinity();
        return tMin >= 0.0f ? tMin : tMax;
    }
}

} // namespace cavalry