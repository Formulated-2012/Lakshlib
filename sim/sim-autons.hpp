#pragma once

// One auton the simulator can run (list is in sim/sim-autons.cpp)
struct SimAuton {
    const char* name;
    void (*run)();
    double start_x, start_y, start_heading;
};

extern const SimAuton SIM_AUTONS[];
extern const int SIM_AUTON_COUNT;
extern const double SIM_DRIVE_CARTRIDGE_RPM;
extern const double SIM_ROBOT_SIZE;
