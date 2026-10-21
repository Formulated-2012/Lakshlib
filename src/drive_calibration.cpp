// Drive calibration for the simulator, for PROS.
// Based on calibrateDrive() from RW-Template (src/drive-log.cpp),
// Copyright (c) 2025 Richard Wang, MIT License - see licenses/RW-Template-LICENSE.txt
#include "drive_calibration.hpp"
#include "robot_constants.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

// These live in main.cpp
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;
extern pros::Imu imu;

// One step of the pattern: what to command and for how long
struct CalibrationStep {
    double seconds;
    int mode;  // 0 = volts, 1 = brake, 2 = coast
    double left_start, right_start, left_end, right_end;  // volts at the start and end (ramps when different)
};

static const CalibrationStep calibration_steps[] = {
    {2.0, 0, 0, 0, 6, 6},          // slow ramp forward: friction and speed per volt
    {1.0, 2, 0, 0, 0, 0},          // coast to a stop
    {2.0, 0, 0, 0, -6, -6},        // slow ramp backward
    {1.0, 2, 0, 0, 0, 0},
    {0.45, 0, 12, 12, 12, 12},     // full-power launch: acceleration and wheel slip
    {0.6, 1, 0, 0, 0, 0},          // hard brake
    {0.45, 0, -12, -12, -12, -12},
    {0.6, 1, 0, 0, 0, 0},
    {0.9, 0, 8, -8, 8, -8},        // turn in place: turning scrub
    {0.5, 1, 0, 0, 0, 0},
    {0.9, 0, -8, 8, -8, 8},
    {0.5, 1, 0, 0, 0, 0},
    {1.0, 0, 9, 4, 9, 4},          // arc: sideways skid
    {0.8, 2, 0, 0, 0, 0},
    {1.0, 0, -9, -4, -9, -4},      // arc back
    {0.8, 2, 0, 0, 0, 0},
};

static double average(const std::vector<double>& values) {
    double sum = 0;
    for (double v : values) sum += v;
    return values.empty() ? 0 : sum / values.size();
}

static void setBrake(pros::MotorBrake mode) {
    leftDrive.set_brake_mode_all(mode);
    rightDrive.set_brake_mode_all(mode);
}

void calibrateDrive() {
    // Motor cartridge RPM, so the fitter knows how far the robot moves per motor turn
    double cartridge = 600;
    if (leftDrive.get_gearing() == pros::MotorGears::red) cartridge = 100;
    if (leftDrive.get_gearing() == pros::MotorGears::green) cartridge = 200;
    double inchesPerMotorRev = DRIVE_RPM / cartridge * M_PI * WHEEL_DIAMETER;

    const int max_rows = 2000;
    static char log_text[max_rows * 72 + 512];
    int length = 0;
    length += snprintf(log_text + length, sizeof(log_text) - length,
                       "# Drive log: wheel_distance_in=%.4f track=%.4f horizontal=0 vertical=0 "
                       "horizontal_offset=0 vertical_offset=0 horizontal_diameter=2 vertical_diameter=2\n"
                       "t,mode,left_volts,right_volts,left_deg,right_deg,imu_deg,horizontal_deg,vertical_deg\n",
                       inchesPerMotorRev, TRACK_WIDTH);

    // Log motor positions in degrees (LemLib uses rotations; put that back at the end)
    leftDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    rightDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    std::vector<pros::MotorBrake> oldBrake = leftDrive.get_brake_mode_all();

    pros::lcd::print(4, "Calibrating the drive for the sim...");
    uint32_t start = pros::millis();
    int rows = 0;
    for (const CalibrationStep& step : calibration_steps) {
        uint32_t stepStart = pros::millis();
        while (pros::millis() - stepStart < step.seconds * 1000 && rows < max_rows) {
            double f = (pros::millis() - stepStart) / (step.seconds * 1000);
            double left = step.left_start + (step.left_end - step.left_start) * f;
            double right = step.right_start + (step.right_end - step.right_start) * f;
            if (step.mode == 0) {
                leftDrive.move_voltage((int)(left * 1000));
                rightDrive.move_voltage((int)(right * 1000));
            } else {
                setBrake(step.mode == 1 ? pros::MotorBrake::brake : pros::MotorBrake::coast);
                leftDrive.brake();
                rightDrive.brake();
            }
            length += snprintf(log_text + length, sizeof(log_text) - length,
                               "%.3f,%d,%.3f,%.3f,%.2f,%.2f,%.3f,0,0\n",
                               (pros::millis() - start) / 1000.0, step.mode,
                               step.mode == 0 ? left : 0, step.mode == 0 ? right : 0,
                               average(leftDrive.get_position_all()), average(rightDrive.get_position_all()),
                               imu.get_rotation());
            rows++;
            pros::delay(10);
        }
    }
    setBrake(pros::MotorBrake::brake);
    leftDrive.brake();
    rightDrive.brake();
    if (!oldBrake.empty()) setBrake(oldBrake[0]);
    leftDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    rightDrive.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

    // "/usd/" is the SD card on the V5 brain
    FILE* file = fopen("/usd/drive_log.csv", "w");
    if (file) {
        fwrite(log_text, 1, length, file);
        fclose(file);
        pros::lcd::print(5, "Saved drive_log.csv to SD (%d rows)", rows);
    } else {
        pros::lcd::print(5, "No SD card: log printed to terminal");
        printf("%s", log_text);
    }
}
