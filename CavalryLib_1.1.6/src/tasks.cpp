#include "tasks.hpp"
#include "globals.hpp"

void handle_intake_task(void* args) {

    while (true) {
        if (intakeState == OFF){
            intake_off();
        }
        else if (intakeState == STORE) {
            intake_store();
        }
        else if (intakeState == HIGH_GOAL) {
            intake_dejam();
            intake_high_goal();
        }
        else if (intakeState == MEDIUM_GOAL) {
            intake_medium_goal();
        }
        else if (intakeState == LOW_GOAL) {
            intake_low_goal();
        }
        else if (intakeState == DEJAM){
            intake_dejam();
            intakeState = intakeStateHidden;
        }
        else {
            throw std::invalid_argument("Undefined intake state");
        }

        pros::delay(TASK_DELAY);
    }
}

void intake_high_goal() {
    mediumGoal.retract();
    intakeLow.move(INTAKE_SPEED);
    intakeHigh.move(INTAKE_SPEED);
}

void intake_medium_goal() {
    mediumGoal.extend();
    if (currentAutoState == SKILLS) {
        intakeLow.move(INTAKE_SPEED);
        intakeHigh.move(INTAKE_SPEED / 2);
        
    }
    else if ((currentAutoState == AWP_RIGHT_RED || currentAutoState == AWP_LEFT_RED) && driveState == AUTONOMOUS){
        intakeLow.move(INTAKE_SPEED * 0.8);
        intakeHigh.move(INTAKE_SPEED / 2);
    }
    else {
        intakeLow.move(INTAKE_SPEED);
        intakeHigh.move(INTAKE_SPEED / 2);
    }
}

void intake_low_goal() {
    mediumGoal.retract();
    intakeHigh.move(-INTAKE_SPEED);
    intakeLow.move(-INTAKE_SPEED);
}

void intake_store() {
    mediumGoal.retract();
    intakeLow.move(INTAKE_SPEED);
    intakeHigh.move(-INTAKE_SLOW_SPEED);
}

void intake_off() {
    mediumGoal.retract();
    intakeLow.brake();
    intakeHigh.brake();
}

void intake_dejam() {
    if (!stateJustSwitched) return;
        
    stateJustSwitched = false;
    intakeLow.move(-INTAKE_SPEED);
    intakeHigh.move(-INTAKE_SPEED);
    pros::delay(INTAKE_DEJAM_TIME);
}

void screen_print_task(void* args) {
    while (true) {
        // Constants for drawing
        const int padding = 10;
        const int fieldSize = 240 - 2 * padding; // 220
        const int fieldX = (480 - fieldSize) / 2 - 100; // 30
        const int fieldY = padding; // 10
        
        // Clear screen
        pros::screen::set_pen(pros::Color::black);
        pros::screen::fill_rect(0, 0, 480, 240); 

        // Draw Field Border
        pros::screen::set_pen(pros::Color::white);
        pros::screen::draw_rect(fieldX, fieldY, fieldX + fieldSize, fieldY + fieldSize);

        // Scale factor
        // Field is -DIST_WALL_FROM_ZERO to DIST_WALL_FROM_ZERO
        float fieldRange = 2 * cavalry::DIST_WALL_FROM_ZERO;
        float scale = fieldSize / fieldRange;

        // Draw Objects
        pros::screen::set_pen(pros::Color::orange);
        /*
        // Circles
        std::vector<cavalry::Circle> circles = particleFilter.get_circles();
        for (const auto& circle : circles) {
            int cx = fieldX + (circle.center.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
            int cy = (fieldY + fieldSize) - (circle.center.y + cavalry::DIST_WALL_FROM_ZERO) * scale;
            int r = circle.radius * scale;
            pros::screen::draw_circle(cx, cy, r);
        }

        // Polygons
        std::vector<cavalry::Polygon> polygons = particleFilter.get_polygons();
        for (const auto& poly : polygons) {
            if (poly.vertices.empty()) continue;
            for (size_t i = 0; i < poly.vertices.size(); ++i) {
                cavalry::Point p1 = poly.vertices[i];
                cavalry::Point p2 = poly.vertices[(i + 1) % poly.vertices.size()];
                
                int x1 = fieldX + (p1.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
                int y1 = (fieldY + fieldSize) - (p1.y + cavalry::DIST_WALL_FROM_ZERO) * scale;
                int x2 = fieldX + (p2.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
                int y2 = (fieldY + fieldSize) - (p2.y + cavalry::DIST_WALL_FROM_ZERO) * scale;
                
                pros::screen::draw_line(x1, y1, x2, y2);
            }
        }*/
        /*
        // Rectangles
        const std::vector<cavalry::Rectangle>& rectangles = particleFilter.get_rectangles();
        for (const auto& rectangle : rectangles) {
            const float halfWidth = rectangle.width * 0.5f;
            const float halfLength = rectangle.length * 0.5f;
            const float cosine = std::cos(rectangle.angle);
            const float sine = std::sin(rectangle.angle);

            const cavalry::Point localCorners[4] = {
                {-halfWidth, -halfLength},
                { halfWidth, -halfLength},
                { halfWidth,  halfLength},
                {-halfWidth,  halfLength}
            };

            cavalry::Point worldCorners[4];
            for (int i = 0; i < 4; i++) {
                worldCorners[i].x = rectangle.center.x + (localCorners[i].x * cosine - localCorners[i].y * sine);
                worldCorners[i].y = rectangle.center.y + (localCorners[i].x * sine + localCorners[i].y * cosine);
            }

            for (int i = 0; i < 4; i++) {
                const cavalry::Point p1 = worldCorners[i];
                const cavalry::Point p2 = worldCorners[(i + 1) % 4];

                const int x1 = fieldX + (p1.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
                const int y1 = (fieldY + fieldSize) - (p1.y + cavalry::DIST_WALL_FROM_ZERO) * scale;
                const int x2 = fieldX + (p2.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
                const int y2 = (fieldY + fieldSize) - (p2.y + cavalry::DIST_WALL_FROM_ZERO) * scale;

                pros::screen::draw_line(x1, y1, x2, y2);
            }
        }*/
        /*
        // Draw particles
        auto particles = particleFilter.get_particles();
        pros::screen::set_pen(pros::Color::red);
        for (const auto& p : particles) {
            // Map x, y to screen
            int sx = fieldX + (p.first + cavalry::DIST_WALL_FROM_ZERO) * scale;
            // VEX field: y increases upwards. Screen: y increases downwards.
            int sy = (fieldY + fieldSize) - (p.second + cavalry::DIST_WALL_FROM_ZERO) * scale;
            
            if (sx >= fieldX && sx <= fieldX + fieldSize && sy >= fieldY && sy <= fieldY + fieldSize) {
                 pros::screen::fill_circle(sx, sy, 2);
            }
        }*/

        // Draw Robot Prediction
        cavalry::Pose mclPrediction = particleFilter.get_prediction();
        cavalry::Pose prediction = robot.getPose();
        pros::screen::set_pen(pros::Color::white);
        int rx = fieldX + (mclPrediction.x + cavalry::DIST_WALL_FROM_ZERO) * scale;
        int ry = (fieldY + fieldSize) - (mclPrediction.y + cavalry::DIST_WALL_FROM_ZERO) * scale;
        
        if (rx >= fieldX && rx <= fieldX + fieldSize && ry >= fieldY && ry <= fieldY + fieldSize) {
             pros::screen::fill_circle(rx, ry, 4);
        }


        // Print coordinates
        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 20, "X: %.2f", prediction.x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 40, "Y: %.2f", prediction.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 60, "Theta: %.2f", prediction.theta);
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 80, "Odom: %.2f", verticalTracker.getDistanceTraveled());

        if (distanceSensorRight.is_valid_reading()) {
            pros::screen::set_pen(pros::Color::white);
        } else {
            pros::screen::set_pen(pros::Color::red);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 100, "Right: %f", distanceSensorRight.get_distance());
        if (distanceSensorLeft.is_valid_reading()) {
            pros::screen::set_pen(pros::Color::white);
        } else {
            pros::screen::set_pen(pros::Color::red);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 120, "Left: %f", distanceSensorLeft.get_distance());

        if (distanceSensorBack.is_valid_reading()) {
            pros::screen::set_pen(pros::Color::white);
        } else {
            pros::screen::set_pen(pros::Color::red);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 140, "Back: %f", distanceSensorBack.get_distance());

        if (distanceSensorFront.is_valid_reading()) {
            pros::screen::set_pen(pros::Color::white);
        } else {
            pros::screen::set_pen(pros::Color::red);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 160, "Front: %f", distanceSensorFront.get_distance());

        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_MEDIUM, 260, 180, "Auto: %s", AUTO_MAP[currentAutoState]);
        controller.print(0, 0, "X: %.2f Y: %.2f", prediction.x, prediction.y);

        pros::delay(TASK_DELAY);
    }
}