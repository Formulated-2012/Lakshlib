#pragma once

// =========================
// DRIVETRAIN CONSTANTS
//
// Shared by the LemLib drivetrain (main.cpp), arcTurn and RAMSETE,
// so each number only lives in one place. Edit these for a new robot.
// =========================

const double TRACK_WIDTH    = 9.75; // inches, left wheel center to right wheel center
const double WHEEL_DIAMETER = 2.75; // inches (same as lemlib::Omniwheel::NEW_275)
const double DRIVE_RPM      = 360;  // wheel RPM after gearing (blue 600 RPM * 36/60 = 360)
