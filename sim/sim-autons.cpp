// =========================
// SIMULATOR: AUTONS AND ROBOT SETUP
//
// The ONLY simulator file you should need to edit.
// =========================
#include "sim-autons.hpp"

// ----- Autons the simulator can run -----
// To add one: declare it here (it's defined in src/main.cpp), then add a row below.
void far2plus1();
void close2plus1();
void testauto();
void PIDangular();
void PIDlateral();
void distanceResetExample();
void mclRamseteExample();
void stopAndGoExample();
void smoothExample();
void testAuto();
extern "C" void autonomous();

// { name, function, start x, start y, start heading }
// Start = where the robot is placed on the field (field-center inches, LemLib heading).
// For autons that use field coordinates, this must match their first chassis.setPose().
// Autons that start with setPose(0, 0, 0) work from anywhere; pick their real starting tile
// so the walls and field elements are in the right place.
// TODO: replace the -35, -60 starts with where each routine really starts.
const SimAuton SIM_AUTONS[] = {
    {"autonomous",           autonomous,           -35, -60,   0},
    {"far2plus1",            far2plus1,            -35, -60,   0},
    {"close2plus1",          close2plus1,          -35, -60,   0},
    {"testauto",             testauto,             -35, -60,   0},
    {"PIDangular",           PIDangular,             0,   0,   0},
    {"PIDlateral",           PIDlateral,             0, -24,   0},
    {"distanceResetExample", distanceResetExample, -60, -36,   0},
    {"mclRamseteExample",    mclRamseteExample,    -60, -36,   0},
    {"stopAndGoExample",     stopAndGoExample,     -60, -36,   0},
    {"smoothExample",        smoothExample,        -60, -36,   0},
    {"testAuto",             testAuto,             -35, -60,   0},
};
const int SIM_AUTON_COUNT = sizeof(SIM_AUTONS) / sizeof(SIM_AUTONS[0]);

// ----- Simulated robot -----
const double SIM_DRIVE_CARTRIDGE_RPM = 600; // drive motor cartridge (blue = 600). TODO: check
const double SIM_ROBOT_SIZE = 15;           // inches, for bumping into walls
