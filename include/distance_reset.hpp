#pragma once

// =========================
// DISTANCE SENSOR POSITION RESETS
//
// Reads a V5 Distance Sensor pointed at a field perimeter wall and fixes
// ONE axis (X or Y) of the LemLib odometry pose to remove drift.
//
// Coordinate convention (field-center origin, inches):
//   (0, 0) = center of the field
//   +Y = "forward" field direction, +X = right
//   LemLib heading: 0 deg = facing +Y, clockwise is positive (90 = facing +X)
//
// RULES:
//   - Resets NEVER run on their own. You call them in autonomous.
//   - Only call when the robot is STOPPED near a wall, right after
//     chassis.waitUntilDone(). Never during a motion.
//   - The auton must start with chassis.setPose(x, y, heading) in FIELD
//     coordinates, otherwise the wall positions don't line up.
//
// All robot-specific numbers live in the CONFIG section of
// src/distance_reset.cpp.
// =========================

#include "pros/distance.hpp"

// One distance sensor and where it sits on the robot.
struct DistSensor {
    const char* name;      // shown in debug output, e.g. "back"
    pros::Distance sensor; // the V5 Distance Sensor
    double x_off;          // inches RIGHT of center of rotation (negative = left)
    double y_off;          // inches FORWARD of center of rotation (negative = back)
    double mount;          // direction the sensor faces vs robot front:
                           // 0 = front, 90 = right, 180 = back, 270 = left
};

// Sensors defined in the CONFIG section of src/distance_reset.cpp
extern DistSensor backSensor;
extern DistSensor leftSensor;
extern DistSensor rightSensor;

// Field walls and limits, also from the CONFIG section (MCL uses these too)
extern const double WALL_POS_X, WALL_NEG_X, WALL_POS_Y, WALL_NEG_Y;
extern const double MAX_READING_IN;
extern const double MAX_ANGLE_DEG;
extern const int MIN_CONFIDENCE;

// Read one sensor and correct the axis of the wall it is looking at.
// Returns true if a correction was applied, false if the reading was rejected.
bool distanceReset(DistSensor& s);

// Wait a short moment for the robot to settle, then reset with both sensors
// (usually one sees an X wall and the other a Y wall).
// Returns true only if BOTH corrections were applied.
bool distanceResetXY(DistSensor& a, DistSensor& b);
