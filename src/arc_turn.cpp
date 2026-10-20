#include "arc_turn.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// The chassis lives in main.cpp
extern lemlib::Chassis chassis;

// =========================
// CONFIG
// =========================
const double TRACK_WIDTH   = 9.75; // inches, must match lemlib::Drivetrain in main.cpp
const double DONE_ERROR    = 1.0;  // degrees, stop when this close to target
const double SLOWDOWN_DEG  = 20.0; // start slowing down this many degrees from target
const double MIN_SPEED_PCT = 0.3;  // never slow below 30% of maxSpeed while slowing


// Smallest signed difference target - current, in (-180, 180]
static double headingError(double target, double current) {
    double e = fmod(target - current, 360.0);
    if (e > 180.0) e -= 360.0;
    if (e <= -180.0) e += 360.0;
    return e;
}

void arcTurn(double radius, double targetHeading, int maxSpeed, int timeout, bool forwards) {
    // Don't fight a LemLib motion that's still running
    chassis.waitUntilDone();

    if (radius < 0) radius = 0;

    // Which way to turn: +1 = clockwise (right), -1 = counter-clockwise (left)
    double startError = headingError(targetHeading, chassis.getPose().theta);
    int dir = (startError >= 0) ? 1 : -1;

    // Inner side speed ratio. If radius < w/2 this goes negative and the inner
    // side spins backwards (tighter than a pivot on one wheel).
    double ratio = (radius - TRACK_WIDTH / 2) / (radius + TRACK_WIDTH / 2);

    // Heading goes clockwise when left moves "more forward" than right.
    // Forwards + clockwise: left is outer. Backwards + clockwise: right is outer.
    bool leftIsOuter = (dir == 1) == forwards;

    uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < (uint32_t)timeout) {
        double error = headingError(targetHeading, chassis.getPose().theta);

        // Done once we reach (or pass) the target in our turn direction
        if (error * dir <= DONE_ERROR) break;

        // Slow down near the end to reduce overshoot
        double scale = 1.0;
        if (std::fabs(error) < SLOWDOWN_DEG) {
            scale = std::fabs(error) / SLOWDOWN_DEG;
            if (scale < MIN_SPEED_PCT) scale = MIN_SPEED_PCT;
        }

        double outer = maxSpeed * scale;
        double inner = outer * ratio;
        if (!forwards) {
            outer = -outer;
            inner = -inner;
        }

        if (leftIsOuter) chassis.tank((int)outer, (int)inner, true);
        else             chassis.tank((int)inner, (int)outer, true);

        pros::delay(10);
    }

    chassis.tank(0, 0, true);
}
