#pragma once

// =========================
// RAMSETE PATH FOLLOWING
//
// Follows a smooth path on a TIMETABLE: it plans when the robot should be
// where (slowing for curves), then steers and speeds up/slows down to stay
// on schedule. Timing is very repeatable. Math ported from RW-Template.
//
// It drives the motors with feedforward voltages (kS, kV, kA), so for good
// results MEASURE them once with measureDriveFeedforward() and put the
// numbers in the CONFIG section of src/ramsete.cpp.
//
// It BLOCKS until done and is not a LemLib motion (no motion chaining with
// LemLib motions). It waits for any running LemLib motion to finish first.
// Uses field-center coordinates and the LemLib pose.
//
// NOT TESTED ON A REAL ROBOT YET.
// =========================

#include "rw/path.hpp"
#include "lemlib/asset.hpp"

// Use the path types without typing "rw::"
using rw::Path;
using rw::Waypoint;

// A smooth path through each point in order, e.g.
//   Path p = pathFromPoints({{-36, -60}, {-36, -24}, {0, -12, 0.5}});
// The optional third number limits the speed there (0-1 of max speed).
using rw::pathFromPoints;

// A path drawn in path.jerryio (saved in the "LemLib v0.5" format), as text.
using rw::pathFromJerryIO;

// Same, but from a LemLib asset file in static/, e.g.
//   ASSET(skills1_txt);  ...  Path p = pathFromAsset(skills1_txt);
// The same jerryio file also works with LemLib's chassis.follow().
Path pathFromAsset(const asset& file);

// path:      from pathFromPoints / pathFromJerryIO / pathFromAsset
// timeout:   ms, gives up after this long
// reverse:   true = drive the path backwards (back of the robot first)
// exit:      true = stop at the end; false = end still moving
// maxSpeed:  in/s (0 = MAX_SPEED from the CONFIG section)
void followPathRamsete(const Path& path, int timeout, bool reverse = false, bool exit = true,
                       double maxSpeed = 0);

// Measures kS, kV, kA. Needs about 4 ft of open space in front and behind.
// Shows the numbers on the brain screen and terminal. NEVER RETURNS.
void measureDriveFeedforward();
