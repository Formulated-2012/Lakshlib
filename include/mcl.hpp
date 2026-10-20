#pragma once

// =========================
// MONTE CARLO LOCALIZATION (MCL)
//
// Background task that uses the distance sensors (the same ones as
// distance_reset) to keep LemLib's X/Y from drifting WHILE the robot drives.
// Heading still comes from the IMU. Math ported from RW-Template.
//
// How it works: a cloud of "particles" (guesses of where the robot is) moves
// with odometry. Each distance reading makes the guesses that match the walls
// more likely. LemLib's pose is nudged toward the best guess, at most 0.15 in
// per axis every 30 ms, so motions never see a jump.
//
// RULES:
//   - Field-center coordinates only (same as distance resets).
//   - Call startMcl() in autonomous AFTER chassis.setPose(start position).
//   - Unlike distance resets, MCL changes the pose DURING motions (in tiny steps).
//   - If you call chassis.setPose() while MCL runs, call mclRecenter() right after.
//     (distanceReset() already does this for you.)
//
// Settings are in the CONFIG section of src/mcl.cpp.
// NOT TESTED ON A REAL ROBOT YET (RW-Template has only tested it in simulation).
// =========================

// Start (or resume) MCL. Particles start around the current pose.
void startMcl();

// Pause MCL. LemLib keeps running on plain odometry.
void stopMcl();

// Tell MCL the pose was just set on purpose (setPose / distance reset).
void mclRecenter();

// MCL's current best guess and how unsure it is (inches; small spread = confident).
// Returns false if MCL isn't running.
bool mclGetEstimate(double& x, double& y, double& spread);
