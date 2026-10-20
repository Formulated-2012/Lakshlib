#pragma once
#include <vector>

// Paths and RAMSETE math, ported from RW-Template (include/path.h).
// Copyright (c) 2025 Richard Wang, MIT License - see licenses/RW-Template-LICENSE.txt
// Headings are compass style (degrees, 0 = +Y, clockwise positive), distances in inches.
// You normally use these through ramsete.hpp.

namespace rw {

// ============================================================================
// PATHS
// ============================================================================

// A point the path goes through, with an optional speed limit there (0-1 of max speed)
struct Waypoint {
  double x, y, speed;
  Waypoint(double x_in, double y_in, double speed_fraction = 1.0) : x(x_in), y(y_in), speed(speed_fraction) {}
};

// The path, sampled about every inch
struct PathSample {
  double x, y, heading_deg, curvature, distance, speed_limit;
};
struct Path {
  std::vector<PathSample> samples;
  double length;
  Path() : length(0) {}
};

Path pathFromPoints(const std::vector<Waypoint>& waypoints);
Path pathFromJerryIO(const char* text);

// Speed plan along a path (in/s at each sample): slow for curves, speed up and slow down at max_accel
std::vector<double> planPathSpeeds(const Path& path, double max_speed, double max_accel, double max_lateral_accel,
                                   double start_speed, double end_speed, bool limit_start);
// Time stamps for a speed plan (seconds at each sample)
std::vector<double> planPathTimes(const Path& path, const std::vector<double>& speeds);

// ============================================================================
// RAMSETE
// ============================================================================

void ramsete(double x, double y, double heading_deg, double ref_x, double ref_y, double ref_heading_deg,
             double ref_speed, double ref_turn_rate, double& speed, double& turn_rate);

double feedforwardVolts(double speed, double accel, double kS, double kV, double kA);

} // namespace rw
