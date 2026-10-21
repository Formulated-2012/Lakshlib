// RAMSETE robot side for PROS + LemLib 0.5.
// Based on followPathRamsete(), driveWheelSpeeds() and measureDriveFeedforward() from
// RW-Template (src/path-following.cpp),
// Copyright (c) 2025 Richard Wang, MIT License - see licenses/RW-Template-LICENSE.txt
// Rewritten to use PROS motor groups and the LemLib pose.
#include "ramsete.hpp"
#include "robot_constants.hpp"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// These live in main.cpp
extern lemlib::Chassis chassis;
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;


// =====================================================================
//   CONFIG
// =====================================================================

// Feedforward: volts = kS * direction + kV * speed + kA * accel
// Measure with measureDriveFeedforward(), then paste the numbers here.
// kV = 0 / kA = 0 means "estimate from DRIVE_RPM and WHEEL_DIAMETER" (rough!).
const double DRIVE_KS = 0.5;     // volts to get the robot moving
const double DRIVE_KV = 0;       // volts per in/s
const double DRIVE_KA = 0;       // volts per in/s^2
const double VELOCITY_KP = 0.03; // extra volts per in/s a side is off from its target speed

const double MAX_SPEED = 0;            // in/s top speed for paths (0 = 80% of the drive's top speed)
const double MAX_ACCEL = 100;          // in/s^2 how fast to speed up / slow down
const double MAX_LATERAL_ACCEL = 120;  // in/s^2 slows down in curves so the robot doesn't slide

// Track width, wheel size and DRIVE_RPM come from include/robot_constants.hpp

// =====================================================================
//   END CONFIG
// =====================================================================


// Drive top speed with no load (in/s)
static double driveFreeSpeed() { return DRIVE_RPM / 60.0 * M_PI * WHEEL_DIAMETER; }
static double driveKV() { return DRIVE_KV > 0 ? DRIVE_KV : 12.0 / (driveFreeSpeed() * 0.92); }
static double driveKA() { return DRIVE_KA > 0 ? DRIVE_KA : driveKV() * 0.13; }

static double profileSpeed(double requested) {
    if (requested > 0) return requested;
    if (MAX_SPEED > 0) return MAX_SPEED;
    return (12 - DRIVE_KS) / driveKV() * 0.8;
}

// Motor cartridge RPM (100 red, 200 green, 600 blue), read from the motors
static double cartridgeRpm(pros::MotorGroup& side) {
    switch (side.get_gearing()) {
        case pros::MotorGears::red:   return 100;
        case pros::MotorGears::green: return 200;
        default:                      return 600;
    }
}

// Average wheel speed of one side (in/s)
static double wheelSpeed(pros::MotorGroup& side) {
    std::vector<double> rpms = side.get_actual_velocity_all();
    if (rpms.empty()) return 0;
    double sum = 0;
    for (double r : rpms) sum += r;
    double motorRpm = sum / rpms.size();
    double wheelRpm = motorRpm * DRIVE_RPM / cartridgeRpm(side);
    return wheelRpm / 60.0 * M_PI * WHEEL_DIAMETER;
}

static void driveVolts(double left, double right) {
    leftDrive.move_voltage((int)(left * 1000));
    rightDrive.move_voltage((int)(right * 1000));
}

/*
 * driveWheelSpeeds
 * Drives each side at a target speed using feedforward plus a little feedback on
 * the measured speed. If a side needs more than 12 V, both scale down together
 * so the robot still curves the right way.
 */
static void driveWheelSpeeds(double leftSpeed, double rightSpeed, double leftAccel, double rightAccel) {
    double kV = driveKV(), kA = driveKA();
    double leftVolts = rw::feedforwardVolts(leftSpeed, leftAccel, DRIVE_KS, kV, kA)
                     + VELOCITY_KP * (leftSpeed - wheelSpeed(leftDrive));
    double rightVolts = rw::feedforwardVolts(rightSpeed, rightAccel, DRIVE_KS, kV, kA)
                      + VELOCITY_KP * (rightSpeed - wheelSpeed(rightDrive));
    double biggest = fmax(fabs(leftVolts), fabs(rightVolts));
    if (biggest > 12) {
        leftVolts = leftVolts / biggest * 12;
        rightVolts = rightVolts / biggest * 12;
    }
    driveVolts(leftVolts, rightVolts);
}

Path pathFromAsset(const asset& file) {
    // Asset files aren't null-terminated, so copy into a string first
    std::string text((const char*)file.buf, file.size);
    return rw::pathFromJerryIO(text.c_str());
}

void followPathRamsete(const Path& path, int timeout, bool reverse, bool exit, double maxSpeed) {
    // Never fight a LemLib motion
    // (Not chassis.waitUntilDone(): that waits forever if no LemLib motion has run yet)
    while (chassis.isInMotion()) pros::delay(10);

    int n = (int)path.samples.size();
    if (n < 2) {
        printf("followPathRamsete: the path is empty, skipping it\n");
        return;
    }

    double direction = reverse ? -1 : 1;
    double speed = profileSpeed(maxSpeed);
    double currentSpeed = fmax(0, direction * (wheelSpeed(leftDrive) + wheelSpeed(rightDrive)) / 2);
    std::vector<double> speeds = rw::planPathSpeeds(path, speed, MAX_ACCEL, MAX_LATERAL_ACCEL,
                                                    currentSpeed, exit ? 0 : speed * 0.5, true);
    std::vector<double> times = rw::planPathTimes(path, speeds);
    double halfTrack = TRACK_WIDTH / 2.0;
    int index = 0;
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < (uint32_t)timeout) {
        double t = (pros::millis() - startTime) / 1000.0;

        // Where the timetable says to be now (between two samples)
        while (index + 1 < n - 1 && times[index + 1] <= t) index++;
        const rw::PathSample& a = path.samples[index];
        const rw::PathSample& b = path.samples[index + 1];
        double span = times[index + 1] - times[index];
        double f = span > 1e-9 ? fmin(fmax((t - times[index]) / span, 0.0), 1.0) : 1.0;
        double refX = a.x + (b.x - a.x) * f;
        double refY = a.y + (b.y - a.y) * f;
        double refHeading = a.heading_deg + remainder(b.heading_deg - a.heading_deg, 360.0) * f;
        double refSpeed = speeds[index] + (speeds[index + 1] - speeds[index]) * f;
        double refCurvature = a.curvature + (b.curvature - a.curvature) * f;
        double refAccel = span > 1e-9 ? (speeds[index + 1] - speeds[index]) / span : 0;
        if (t >= times[n - 1]) {
            // Past the end of the timetable: hold the end point
            refX = path.samples[n - 1].x;
            refY = path.samples[n - 1].y;
            refHeading = path.samples[n - 1].heading_deg;
            refSpeed = speeds[n - 1];
            refCurvature = 0;
            refAccel = 0;
        }

        lemlib::Pose pose = chassis.getPose();

        // Finish at the end of the timetable (settling on the end point if exit is true)
        double endError = hypot(path.samples[n - 1].x - pose.x, path.samples[n - 1].y - pose.y);
        double endHeadingError = fabs(remainder(path.samples[n - 1].heading_deg + (reverse ? 180 : 0) - pose.theta, 360.0));
        if (t >= times[n - 1] && (!exit || (endError < 1.0 && endHeadingError < 2.0) || t > times[n - 1] + 0.3)) {
            break;
        }

        // Backward: the robot faces away from the direction of travel
        double speedCommand = 0, turnCommand = 0;
        rw::ramsete(pose.x, pose.y, pose.theta, refX, refY, refHeading + (reverse ? 180 : 0),
                    direction * refSpeed, refSpeed * refCurvature, speedCommand, turnCommand);
        double turnSpeed = turnCommand * halfTrack;
        driveWheelSpeeds(speedCommand + turnSpeed, speedCommand - turnSpeed,
                         direction * refAccel, direction * refAccel);
        pros::delay(10);
    }

    if (exit) {
        // Hold still (the next LemLib motion or driver control takes over)
        leftDrive.move_velocity(0);
        rightDrive.move_velocity(0);
    }
}

void measureDriveFeedforward() {
    // (Not chassis.waitUntilDone(): that waits forever if no LemLib motion has run yet)
    while (chassis.isInMotion()) pros::delay(10);

    const double voltsSteps[6] = {3, -3, 6, -6, 9, -9};
    double steadySpeed[6];
    double accelSum = 0, accelSpeedSum = 0;
    int accelCount = 0;
    pros::lcd::print(4, "Measuring the drivetrain...");

    for (int i = 0; i < 6; i++) {
        uint32_t start = pros::millis();
        double speedSum = 0, earlySpeed = 0, earlyTime = -1;
        int speedCount = 0;
        while (pros::millis() - start < 800) {
            driveVolts(voltsSteps[i], voltsSteps[i]);
            double t = pros::millis() - start;
            double speed = fabs(wheelSpeed(leftDrive) + wheelSpeed(rightDrive)) / 2;
            // Steady speed near the end of each step gives kS and kV
            if (t > 550) {
                speedSum += speed;
                speedCount++;
            }
            // How fast the 9 V steps speed up gives kA
            if (fabs(voltsSteps[i]) == 9) {
                if (t >= 30 && earlyTime < 0) {
                    earlyTime = t;
                    earlySpeed = speed;
                } else if (earlyTime >= 0 && t >= earlyTime + 100 && t < earlyTime + 110) {
                    accelSum += (speed - earlySpeed) / ((t - earlyTime) / 1000.0);
                    accelSpeedSum += (speed + earlySpeed) / 2;
                    accelCount++;
                }
            }
            pros::delay(10);
        }
        driveVolts(0, 0);
        leftDrive.brake();
        rightDrive.brake();
        pros::delay(500);
        steadySpeed[i] = speedCount > 0 ? speedSum / speedCount : 0;
    }

    // Straight line through (speed, volts): volts = kS + kV * speed
    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    for (int i = 0; i < 6; i++) {
        double x = steadySpeed[i], y = fabs(voltsSteps[i]);
        sumX += x; sumY += y; sumXX += x * x; sumXY += x * y;
    }
    double kV = (6 * sumXY - sumX * sumY) / fmax(6 * sumXX - sumX * sumX, 1e-9);
    double kS = (sumY - kV * sumX) / 6;
    double kA = 0;
    if (accelCount > 0 && accelSum > 0) {
        double accel = accelSum / accelCount, midSpeed = accelSpeedSum / accelCount;
        kA = (9 - kS - kV * midSpeed) / accel;
    }

    printf("DRIVE_KS = %.3f; DRIVE_KV = %.4f; DRIVE_KA = %.4f\n", kS, kV, kA);
    while (true) {
        // Lines 4-7, since main.cpp's screen task keeps writing lines 0-2
        pros::lcd::print(4, "Put these in src/ramsete.cpp:");
        pros::lcd::print(5, "DRIVE_KS = %.3f", kS);
        pros::lcd::print(6, "DRIVE_KV = %.4f", kV);
        pros::lcd::print(7, "DRIVE_KA = %.4f", kA);
        pros::delay(200);
    }
}
