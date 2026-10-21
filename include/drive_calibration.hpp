#pragma once

// =========================
// DRIVE CALIBRATION FOR THE SIMULATOR (optional)
//
// Runs a fixed ~15 s driving pattern while recording the drive's voltages,
// encoders and IMU. On a computer,
//     sh sim/fit-profile.sh drive_log.csv
// turns the recording into sim/robot-profile.txt, so the simulator speeds up,
// turns, slips and stops like OUR robot. Ported from RW-Template.
//
// Put the robot in open space with about 4 ft clear on EVERY side.
// Saves drive_log.csv to the SD card (or prints it to the terminal if there's
// no card). Call it after initialize() (the IMU must be calibrated).
// =========================

void calibrateDrive();
