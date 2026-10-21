// Simulator runner: runs one of our autons on a simulated robot and Override field with the
// real robot code and LemLib, and writes an HTML replay. Run: sh sim/run-sim.sh far2plus1
// Based on RW-Template's sim/sim-main.cpp, Copyright (c) 2025 Richard Wang, MIT License.
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/odom.hpp"
#include "distance_reset.hpp"
#include "mcl.hpp"
#include "robot_constants.hpp"
#include "sim-autons.hpp"
#include "engine/sim-engine.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// These live in src/main.cpp and src/distance_reset.cpp
extern lemlib::Chassis chassis;
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;

namespace {

struct Options {
    std::string auton = "autonomous";
    std::string profile = "sim/robot-profile.txt";
    bool use_profile = true;
    std::string viewer = "sim/engine/viewer.html";
    std::string output;
    rwsim::WorldSetup world;
    bool start_given = false;
    bool list = false;
};

void usage() {
    std::printf(
        "Usage: sh sim/run-sim.sh [auton name] [options]\n"
        "  --list                 List the autons in sim/sim-autons.cpp\n"
        "  --start X,Y,HEADING    Where the robot starts (field-center inches, LemLib heading)\n"
        "                         (default: the start listed in sim/sim-autons.cpp)\n"
        "  --time SECONDS         Stop after this long (default 15, use 60 for skills)\n"
        "  --ideal                No wheel slip, IMU error, or sensor noise\n"
        "  --walls                Empty field (no Override goals, loaders, or stacks)\n"
        "  --seed N               Random seed for slip and noise\n"
        "  --profile FILE         Robot profile from fit-profile (default sim/robot-profile.txt if it exists)\n"
        "  --no-profile           Ignore the robot profile and use the default robot\n"
        "  --out FILE             Where to write the replay\n");
}

Options parse(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", name); std::exit(1); }
            return argv[++i];
        };
        if (arg == "--list") opt.list = true;
        else if (arg == "--start") {
            if (std::sscanf(next("--start"), "%lf,%lf,%lf", &opt.world.start_x, &opt.world.start_y,
                            &opt.world.start_heading_deg) != 3) {
                std::fprintf(stderr, "--start needs X,Y,HEADING\n");
                std::exit(1);
            }
            opt.start_given = true;
        }
        else if (arg == "--time") opt.world.time_limit_s = std::atof(next("--time"));
        else if (arg == "--ideal") opt.world.realistic = false;
        else if (arg == "--walls") opt.world.override_field = false;
        else if (arg == "--seed") opt.world.seed = (unsigned)std::atoi(next("--seed"));
        else if (arg == "--out") opt.output = next("--out");
        else if (arg == "--profile") opt.profile = next("--profile");
        else if (arg == "--no-profile") opt.use_profile = false;
        else if (arg == "--viewer") opt.viewer = next("--viewer");
        else if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
        else if (!arg.empty() && arg[0] == '-') { std::fprintf(stderr, "Unknown option %s\n", arg.c_str()); usage(); std::exit(1); }
        else opt.auton = arg;
    }
    return opt;
}

std::string fileName(const std::string& name) {
    std::string out;
    for (char c : name) out += (std::isalnum((unsigned char)c) ? c : '-');
    return out.empty() ? "auton" : out;
}

// What the robot code thinks (read by the engine every 10 ms for the replay)
void readBelief(double& x, double& y, double& heading, bool& moving, bool& mcl, double& mcl_x, double& mcl_y) {
    lemlib::Pose pose = lemlib::getPose(false);
    x = pose.x;
    y = pose.y;
    heading = pose.theta;
    moving = chassis.isInMotion();
    double spread = 0;
    mcl = mclGetEstimate(mcl_x, mcl_y, spread);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parse(argc, argv);

    if (opt.list) {
        for (int i = 0; i < SIM_AUTON_COUNT; i++) {
            std::printf("  %-22s starts at %.0f, %.0f, %.0f\n", SIM_AUTONS[i].name, SIM_AUTONS[i].start_x,
                        SIM_AUTONS[i].start_y, SIM_AUTONS[i].start_heading);
        }
        return 0;
    }
    const SimAuton* auton = nullptr;
    for (int i = 0; i < SIM_AUTON_COUNT; i++) {
        if (opt.auton == SIM_AUTONS[i].name) auton = &SIM_AUTONS[i];
    }
    if (!auton) {
        std::fprintf(stderr, "No auton named \"%s\" in sim/sim-autons.cpp. Listed autons:\n", opt.auton.c_str());
        for (int i = 0; i < SIM_AUTON_COUNT; i++) std::fprintf(stderr, "  %s\n", SIM_AUTONS[i].name);
        return 1;
    }
    if (!opt.start_given) {
        opt.world.start_x = auton->start_x;
        opt.world.start_y = auton->start_y;
        opt.world.start_heading_deg = auton->start_heading;
    }

    rwsim::adoptMainThread();

    // The simulated robot comes from include/robot_constants.hpp, the drive motor groups in
    // src/main.cpp and the distance sensors in src/distance_reset.cpp
    rwsim::RobotSetup robot;
    for (int p : leftDrive.sim_ports()) { robot.left_ports.push_back(p); rwsim::motorRegister(p, SIM_DRIVE_CARTRIDGE_RPM); }
    for (int p : rightDrive.sim_ports()) { robot.right_ports.push_back(p); rwsim::motorRegister(p, SIM_DRIVE_CARTRIDGE_RPM); }
    robot.track_width_in = TRACK_WIDTH;
    robot.inches_per_motor_rev = DRIVE_RPM / SIM_DRIVE_CARTRIDGE_RPM * M_PI * WHEEL_DIAMETER;
    robot.robot_size_in = SIM_ROBOT_SIZE;
    for (DistSensor* s : {&backSensor, &leftSensor, &rightSensor}) {
        robot.distance_sensors.push_back({s->sensor.get_port(), s->x_off, s->y_off, s->mount});
    }

    // A fitted robot profile, if there is one (otherwise a typical drive for these motors and wheels)
    bool from_profile = opt.use_profile && rwsim::loadDriveModel(opt.profile.c_str(), robot.drive);
    robot.drive_from_profile = from_profile;

    rwsim::BeliefSource belief;
    belief.read = readBelief;
    rwsim::setup(robot, opt.world, belief);

    // Like turning the robot on: initialize() calibrates the IMU and starts odometry and tasks
    rwsim::setCompetition(false, false);
    initialize();

    std::string output = opt.output.empty() ? "sim/out/" + fileName(opt.auton) + ".html" : opt.output;
    std::printf("Simulating \"%s\" from x %.1f, y %.1f, heading %.1f (%s, %s field, %.0f s limit)\n",
                opt.auton.c_str(), opt.world.start_x, opt.world.start_y, opt.world.start_heading_deg,
                opt.world.realistic ? "realistic" : "ideal", opt.world.override_field ? "Override" : "empty",
                opt.world.time_limit_s);
    std::printf("Robot: %s\n", from_profile ? ("fitted profile " + opt.profile).c_str() : "default drive (no robot profile)");
    std::fflush(stdout);

    rwsim::startRecording(opt.auton, opt.viewer, output);
    rwsim::setCompetition(true, true);
    auton->run();
    // On the robot, a LemLib motion started without waitUntilDone() keeps going after the
    // auton function returns, so wait for it (the time limit still applies)
    while (chassis.isInMotion()) pros::delay(10);
    // Keep watching a moment so the replay shows the robot settle
    pros::delay(1000);
    rwsim::finishRun("auton finished");
}
