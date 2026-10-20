#pragma once

// Pure math helpers ported from RW-Template (src/utils.cpp).
// Copyright (c) 2025 Richard Wang, MIT License - see licenses/RW-Template-LICENSE.txt
//
// Headings are compass style (degrees, 0 = +Y, clockwise positive), same as LemLib.
// Everything is inside "namespace rw" so names don't clash with LemLib.

namespace rw {

double degToRad(double deg);
double radToDeg(double rad);

// Rotates a robot-relative offset (x = right, y = forward) into field coordinates.
void localToGlobal(double local_x, double local_y, double heading_rad, double& global_x, double& global_y);

// Distance along a sensor beam to the field wall it hits.
double beamToWall(double sensor_x, double sensor_y, double beam_x, double beam_y, double field_half_size,
                  bool& hits_x_wall, double& other_wall_gap);

// Works out the robot's X or Y from one distance reading against a wall.
// Returns 1 = new X, 2 = new Y, 0 = skip this reading (corner, too steep, ...).
int distanceResetPosition(double robot_x, double robot_y, double heading_deg, double s_x, double s_y,
                          double sensor_angle_offset, double reading_in, double field_half_size,
                          double max_angle_deg, double corner_margin, double& new_pos);

} // namespace rw
