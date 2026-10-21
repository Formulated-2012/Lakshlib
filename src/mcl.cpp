// MCL robot side for PROS + LemLib 0.5.
// Based on runMcl() from RW-Template (src/motor-control.cpp),
// Copyright (c) 2025 Richard Wang, MIT License - see licenses/RW-Template-LICENSE.txt
// Rewritten to read and nudge the LemLib pose instead of RW's own odometry.
#include "mcl.hpp"
#include "distance_reset.hpp"
#include "rw/mcl_filter.hpp"
#include "rw/rw_math.hpp"
#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>

// The chassis lives in main.cpp
extern lemlib::Chassis chassis;


// =====================================================================
//   CONFIG
// =====================================================================

// Sensors MCL uses (offsets/ports are set in src/distance_reset.cpp)
DistSensor* const MCL_SENSORS[] = {&backSensor, &leftSensor, &rightSensor};

const int    MCL_PARTICLES   = 200;  // more = smoother but slower (max 1000). Lower if "ms" on screen > ~5
const double SENSOR_DELAY_MS = 40;   // how far behind distance readings are
const double JUMP_IN         = 4.0;  // pose jumped more than this in 10 ms = someone called setPose
const bool   MCL_DEBUG       = true; // print estimate to terminal + brain screen line 3

// Walls (WALL_POS_X, ...), MAX_READING_IN, MAX_ANGLE_DEG, MIN_CONFIDENCE and
// CORNER_MARGIN come from the CONFIG section of src/distance_reset.cpp.

// =====================================================================
//   END CONFIG
// =====================================================================


static pros::Task* mclTask = nullptr;
static volatile bool mclRunning = false;
static volatile bool recenterRequested = false;
static double estX = 0, estY = 0, estSpread = 1e9;

void mclRecenter() {
    recenterRequested = true;
}

void stopMcl() {
    mclRunning = false;
}

bool mclGetEstimate(double& x, double& y, double& spread) {
    if (!mclRunning) return false;
    x = estX;
    y = estY;
    spread = estSpread;
    return true;
}

// MCL assumes a square field centered on (0, 0)
static double fieldHalfSize() {
    return (WALL_POS_X - WALL_NEG_X + WALL_POS_Y - WALL_NEG_Y) / 4.0;
}

static void mclLoop() {
    const int SENSOR_COUNT = sizeof(MCL_SENSORS) / sizeof(MCL_SENSORS[0]);
    const int HISTORY = 16; // last 160 ms of odometry, to line up delayed readings
    double histX[HISTORY], histY[HISTORY], histHeading[HISTORY];

    // Odometry travel, NOT counting MCL's own nudges or setPose jumps
    double motionX = 0, motionY = 0;
    double lastFilterX = 0, lastFilterY = 0, lastFilterHeading = 0;
    double movedSinceCorrect = 0, speedIps = 0, turnDps = 0;
    int32_t lastMm[SENSOR_COUNT];
    double cycleMs = 0;
    int tick = 0;

    lemlib::Pose last = chassis.getPose();

    // Start the particles at the current pose and forget old history
    auto restart = [&](const lemlib::Pose& p) {
        rw::mclInit(MCL_PARTICLES, p.x, p.y, 1.0);
        estX = p.x; // until the filter has run, the best guess is the pose itself
        estY = p.y;
        estSpread = 1.0;
        for (int i = 0; i < HISTORY; i++) {
            histX[i] = motionX;
            histY[i] = motionY;
            histHeading[i] = p.theta;
        }
        lastFilterX = motionX;
        lastFilterY = motionY;
        lastFilterHeading = p.theta;
        for (int i = 0; i < SENSOR_COUNT; i++) lastMm[i] = -1;
    };
    restart(last);

    uint32_t wakeTime = pros::millis();
    while (true) {
        pros::Task::delay_until(&wakeTime, 10);

        if (!mclRunning) {
            last = chassis.getPose();
            continue;
        }

        // ---- Track odometry movement every 10 ms ----
        lemlib::Pose pose = chassis.getPose();
        double dx = pose.x - last.x;
        double dy = pose.y - last.y;
        double dHeading = remainder(pose.theta - last.theta, 360.0);
        last = pose;

        if (recenterRequested || hypot(dx, dy) > JUMP_IN) {
            // The pose was set on purpose: start over there
            recenterRequested = false;
            restart(pose);
            continue;
        }

        motionX += dx;
        motionY += dy;
        speedIps = 0.7 * speedIps + 0.3 * (hypot(dx, dy) / 0.01);
        turnDps  = 0.7 * turnDps  + 0.3 * (fabs(dHeading) / 0.01);
        histX[tick % HISTORY] = motionX;
        histY[tick % HISTORY] = motionY;
        histHeading[tick % HISTORY] = pose.theta;

        // ---- Run the filter every 30 ms (about how often distance sensors update) ----
        if (tick % 3 == 0) {
            uint64_t cycleStart = pros::micros();

            double fdx = motionX - lastFilterX, fdy = motionY - lastFilterY;
            double fdHeading = remainder(pose.theta - lastFilterHeading, 360.0);
            lastFilterX = motionX;
            lastFilterY = motionY;
            lastFilterHeading = pose.theta;
            rw::mclPredict(fdx, fdy, fdHeading);
            movedSinceCorrect += hypot(fdx, fdy) + fabs(fdHeading) * 0.05;

            // Readings lag behind, so compare them with where the robot was back then
            int delayTicks = (int)fmin(fmax(round(SENSOR_DELAY_MS / 10.0), 0), HISTORY - 1);
            int then = ((tick - delayTicks) % HISTORY + HISTORY) % HISTORY;
            double shiftX = motionX - histX[then];
            double shiftY = motionY - histY[then];
            double headingThen = histHeading[then];
            double half = fieldHalfSize();

            // Keep readings that hit a wall cleanly
            rw::MclReading readings[SENSOR_COUNT];
            int count = 0;
            for (int i = 0; i < SENSOR_COUNT; i++) {
                DistSensor& s = *MCL_SENSORS[i];
                int32_t mm = s.sensor.get_distance();
                bool repeat = (mm == lastMm[i]); // sensor hasn't updated yet
                lastMm[i] = mm;
                if (repeat || mm == PROS_ERR || mm <= 0 || mm >= 9999) continue;

                double readingIn = mm / 25.4;
                if (readingIn < 1.5 || readingIn > MAX_READING_IN) continue;
                if (mm > 200 && s.sensor.get_confidence() < MIN_CONFIDENCE) continue;

                double unused = 0;
                if (rw::distanceResetPosition(pose.x - shiftX, pose.y - shiftY, headingThen,
                                              s.x_off, s.y_off, s.mount, readingIn, half,
                                              MAX_ANGLE_DEG, CORNER_MARGIN, unused) == 0) {
                    continue; // corner, too steep, ...
                }
                readings[count].s_x = s.x_off;
                readings[count].s_y = s.y_off;
                readings[count].angle_deg = s.mount;
                readings[count].reading_in = readingIn;
                count++;
            }

            // Only weigh after moving a little (a parked robot would get overconfident),
            // and not while spinning fast (delayed readings are least trustworthy then)
            if (count > 0 && movedSinceCorrect > 0.25 && turnDps < 200) {
                rw::mclCorrect(headingThen, readings, count, half, speedIps * 0.01, shiftX, shiftY);
                movedSinceCorrect = 0;
            }

            double spreadX = 0, spreadY = 0;
            rw::mclEstimate(estX, estY, spreadX, spreadY);
            estSpread = fmax(spreadX, spreadY);

            // ---- Nudge LemLib toward the estimate ----
            // Read the newest pose and set it right away (nothing in between), so no
            // odometry movement is lost. This task runs at a higher priority than
            // LemLib's odometry task so it isn't interrupted here.
            lemlib::Pose now = chassis.getPose();
            double nx = now.x, ny = now.y;
            rw::mclNudge(nx, ny, estX, estY, spreadX, spreadY);
            if (nx != now.x || ny != now.y) {
                chassis.setPose(nx, ny, now.theta);
                // The nudge isn't robot movement; keep it out of the next delta
                last.x += nx - now.x;
                last.y += ny - now.y;
            }

            cycleMs = 0.8 * cycleMs + 0.2 * (pros::micros() - cycleStart) / 1000.0;

            if (MCL_DEBUG && tick % 51 == 0) { // about every 0.5 s
                printf("[MCL] est %.1f, %.1f  spread %.2f  readings %d  %.1f ms\n",
                       estX, estY, estSpread, count, cycleMs);
                pros::lcd::print(3, "MCL %.1f %.1f sp %.1f %.1fms", estX, estY, estSpread, cycleMs);
            }
        }

        tick++;
    }
}

void startMcl() {
    recenterRequested = true; // start the particles at the current pose
    lemlib::Pose pose = chassis.getPose();
    estX = pose.x; // so mclGetEstimate() never returns (0, 0) before the first update
    estY = pose.y;
    estSpread = 1.0;
    mclRunning = true;
    if (mclTask == nullptr) {
        // One priority above default, so it runs ahead of LemLib's odometry task
        mclTask = new pros::Task(mclLoop, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "MCL");
    }
}
