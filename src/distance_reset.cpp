#include "distance_reset.hpp"
#include "mcl.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp" // lemlib::getSpeed()
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>

// The chassis lives in main.cpp
extern lemlib::Chassis chassis;


// =====================================================================
// =====================================================================
//   CONFIG  -- the ONLY section you should need to edit for a new robot
// =====================================================================
// =====================================================================

// ----- Sensors -----
// { name, port, x_off, y_off, mount }
//   x_off: inches right of the center of rotation (negative = left)
//   y_off: inches forward of the center of rotation (negative = back)
//   Measure from the center of rotation to the FACE of the sensor.
//   mount: 0 = faces front, 90 = right, 180 = back, 270 = left
//
// TODO: ports and offsets below are PLACEHOLDERS. Measure the real robot!
DistSensor backSensor  = {"back",  pros::Distance(7),  0.0, -6.0, 180};
DistSensor leftSensor  = {"left",  pros::Distance(8), -5.5,  0.0, 270};
DistSensor rightSensor = {"right", pros::Distance(9),  5.5,  0.0,  90};
// Optional front sensor (also add "extern DistSensor frontSensor;" to the .hpp):
// DistSensor frontSensor = {"front", pros::Distance(16), 0.0, 6.0, 0};

// ----- Field walls (field-center origin, inches) -----
// Interior wall-to-wall is about 140.4 in, so each wall is ~70.2 in from center.
// Measure the field you compete on and adjust if needed.
const double WALL_POS_X = 70.2;   // +X wall (the -X wall is at -WALL_POS_X)
const double WALL_NEG_X = -70.2;
const double WALL_POS_Y = 70.2;   // +Y wall
const double WALL_NEG_Y = -70.2;

// ----- Safety limits -----
const double MAX_READING_IN    = 60.0; // ignore readings farther than this
const double MAX_CORRECTION_IN = 6.0;  // ignore if it would move the pose more than this
const double MAX_ANGLE_DEG     = 20.0; // ignore if beam is this far from perpendicular to the wall
const int    MIN_CONFIDENCE    = 40;   // 0-63, only checked when reading > 200 mm
// Ignore beams that hit a wall this close to a corner (inches, 0 = off).
// In Override the corner LOADERS sit in front of the wall there, so the sensor
// measures the loader instead of the wall (found in the simulator: a 3.8" wrong reset).
const double CORNER_MARGIN     = 15.0;
// The robot must be really stopped: a LemLib motion can end while the robot is still
// rolling (found in the simulator: ~10 in/s after moveToPoint ended -> 1.5" wrong reset)
const double STILL_SPEED_IPS  = 1.0;  // "stopped" = slower than this (in/s)
const int    MAX_SETTLE_MS    = 500;  // distanceResetXY waits up to this long to stop

// ----- Debug -----
// true = print pose before/after and the raw reading to the terminal and
// the brain screen (lines 4-6). Use this to check corrections are small
// (usually < 2 in) and in the right direction. Set false for competition.
const bool DISTANCE_RESET_DEBUG = true;

// =====================================================================
//   END CONFIG
// =====================================================================


// Print to the terminal and (if debugging) the brain screen.
static void debugPrint(int line, const char* text) {
    if (!DISTANCE_RESET_DEBUG) return;
    printf("[distReset] %s\n", text);
    pros::lcd::print(line, "%s", text);
}

static double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

// How fast the robot is moving (in/s), from LemLib odometry
static double robotSpeed() {
    lemlib::Pose speed = lemlib::getSpeed();
    return std::hypot(speed.x, speed.y);
}

bool distanceReset(DistSensor& s) {
    char msg[80];

    // Safety: never reset while LemLib is driving or the robot is still rolling
    if (chassis.isInMotion() || robotSpeed() > STILL_SPEED_IPS) {
        snprintf(msg, sizeof(msg), "%s: skipped, robot moving", s.name);
        debugPrint(4, msg);
        return false;
    }

    // ---- 1. Read the sensor ----
    int32_t mm = s.sensor.get_distance();
    if (mm == PROS_ERR || mm <= 0 || mm >= 9999) {
        // PROS_ERR = unplugged/error, 9999 = no object seen
        snprintf(msg, sizeof(msg), "%s: bad reading (%ld)", s.name, (long)mm);
        debugPrint(4, msg);
        return false;
    }
    double reading = mm / 25.4; // mm -> inches

    if (reading > MAX_READING_IN) {
        snprintf(msg, sizeof(msg), "%s: too far (%.1f in)", s.name, reading);
        debugPrint(4, msg);
        return false;
    }
    if (mm > 200) {
        int conf = s.sensor.get_confidence();
        if (conf < MIN_CONFIDENCE) {
            snprintf(msg, sizeof(msg), "%s: low confidence (%d)", s.name, conf);
            debugPrint(4, msg);
            return false;
        }
    }

    // ---- 2. Where is the sensor on the field? ----
    lemlib::Pose pose = chassis.getPose(); // x, y in inches, theta in degrees
    double th = degToRad(pose.theta);

    // Rotate the sensor offset by the robot heading.
    // (heading 0 = +Y, clockwise positive, so "right" at heading 0 is +X)
    double offX = s.x_off * cos(th) + s.y_off * sin(th);
    double offY = -s.x_off * sin(th) + s.y_off * cos(th);
    // Sensor field position = (pose.x + offX, pose.y + offY)

    // ---- 3. Which wall is the beam pointing at? ----
    double beam = fmod(pose.theta + s.mount, 360.0);
    if (beam < 0) beam += 360.0;

    // Snap to the nearest of 0 (+Y), 90 (+X), 180 (-Y), 270 (-X)
    int side = (int)std::round(beam / 90.0) % 4;
    double angleOff = std::fabs(beam - side * 90.0);
    if (angleOff > 180.0) angleOff = 360.0 - angleOff; // handles beam near 360
    if (angleOff > MAX_ANGLE_DEG) {
        snprintf(msg, sizeof(msg), "%s: %.0f deg off square", s.name, angleOff);
        debugPrint(4, msg);
        return false;
    }

    // ---- 4. Solve the one axis of that wall ----
    // Beam direction as a vector. Using the real angle (not the snapped one)
    // corrects for the robot not being perfectly square to the wall.
    double a = degToRad(beam);
    double dirX = sin(a);
    double dirY = cos(a);

    double newX = pose.x;
    double newY = pose.y;
    double correction = 0;

    // Where along the wall the beam lands; skip it if that's near a corner
    double along = (side == 0 || side == 2) ? pose.x + offX + reading * dirX
                                            : pose.y + offY + reading * dirY;
    double halfWall = (side == 0 || side == 2) ? (WALL_POS_X - WALL_NEG_X) / 2 : (WALL_POS_Y - WALL_NEG_Y) / 2;
    if (CORNER_MARGIN > 0 && std::fabs(along) > halfWall - CORNER_MARGIN) {
        snprintf(msg, sizeof(msg), "%s: hits near a corner (loader?)", s.name);
        debugPrint(4, msg);
        return false;
    }

    if (side == 0 || side == 2) {
        // Hitting a Y wall: (pose.y + offY) + reading * dirY = wall
        double wall = (side == 0) ? WALL_POS_Y : WALL_NEG_Y;
        double newSensorY = wall - reading * dirY;
        newY = newSensorY - offY; // back to robot center
        correction = newY - pose.y;
    } else {
        // Hitting an X wall: (pose.x + offX) + reading * dirX = wall
        double wall = (side == 1) ? WALL_POS_X : WALL_NEG_X;
        double newSensorX = wall - reading * dirX;
        newX = newSensorX - offX; // back to robot center
        correction = newX - pose.x;
    }

    if (std::fabs(correction) > MAX_CORRECTION_IN) {
        snprintf(msg, sizeof(msg), "%s: correction too big (%.1f in)", s.name, correction);
        debugPrint(4, msg);
        return false;
    }

    // ---- 5. Apply: change only that axis, keep heading ----
    chassis.setPose(newX, newY, pose.theta);
    mclRecenter(); // if MCL is running, restart its particles at the new pose

    snprintf(msg, sizeof(msg), "%s: read %.1f in, fix %c by %+.2f",
             s.name, reading, (side == 0 || side == 2) ? 'Y' : 'X', correction);
    debugPrint(4, msg);
    snprintf(msg, sizeof(msg), "before %.1f, %.1f, %.1f", pose.x, pose.y, pose.theta);
    debugPrint(5, msg);
    snprintf(msg, sizeof(msg), "after  %.1f, %.1f, %.1f", newX, newY, pose.theta);
    debugPrint(6, msg);

    return true;
}

bool distanceResetXY(DistSensor& a, DistSensor& b) {
    // Wait until the robot has really stopped (up to MAX_SETTLE_MS), plus a moment
    // for the distance sensors to take a fresh reading
    uint32_t start = pros::millis();
    while (robotSpeed() > STILL_SPEED_IPS && pros::millis() - start < (uint32_t)MAX_SETTLE_MS) {
        pros::delay(10);
    }
    pros::delay(50);

    // Run both, even if the first one fails
    bool okA = distanceReset(a);
    bool okB = distanceReset(b);
    return okA && okB;
}
