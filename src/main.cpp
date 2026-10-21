#include "main.h"
#include "lemlib/api.hpp"
#include "distance_reset.hpp"
#include "arc_turn.hpp"
#include "mcl.hpp"
#include "ramsete.hpp"
#include "robot_constants.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);


// =========================
// MOTORS
// =========================

// Drive
pros::MotorGroup leftDrive({-1, -10});
pros::MotorGroup rightDrive({11, 15});

// Lift
pros::MotorGroup lift({-2, 12});

// Claw
pros::Motor claw(-3);

// Intake
pros::MotorGroup intake({-4, 13});

// 5.5W motor
pros::Motor toggleMotor(-5);


// =========================
// SENSORS
// =========================

// IMU
pros::Imu imu(6);

// Lift rotation sensor
pros::Rotation liftencoder(14);


// =========================
// LEMLIB CONFIG
// =========================

// Track width, wheel size and RPM are in include/robot_constants.hpp
lemlib::Drivetrain drivetrain(
    &leftDrive,
    &rightDrive,
    TRACK_WIDTH,
    WHEEL_DIAMETER,
    DRIVE_RPM,
    2
);


// Lateral PID
lemlib::ControllerSettings lateralController(
    100,
    0,
    5,
    3,
    1,
    100,
    3,
    500,
    20
);


// Angular PID
lemlib::ControllerSettings angularController(
    4,
    0,
    43,
    3,
    1,
    100,
    3,
    500,
    0
);


// No tracking wheels
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);


// Chassis
lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    sensors
);


// =========================
// CLAW FUNCTIONS
// =========================

void openClaw() {

    claw.move_voltage(5000);
    intake.move(0);
}


void closeClaw() {

    claw.move_voltage(-1500);
    intake.move(127);
}


// =========================
// LIFT PID
// =========================

double liftkP = 0.03;
double liftkI = 0.0;
double liftkD = 0.08;

int liftTarget = 0;

bool liftPIDRunning = false;


// =========================
// LIFT PID TASK
// =========================

void liftPIDTask(void* param) {

    double previousError = 0;
    double integral = 0;

    while (true) {

        if (liftPIDRunning) {

            int currentPosition =
                liftencoder.get_position();

            double error =
                liftTarget - currentPosition;


            integral += error;

            if (error > 5000 || error < -5000) {
                integral = 0;
            }


            double derivative =
                error - previousError;


            double output =
                (error * liftkP) +
                (integral * liftkI) +
                (derivative * liftkD);


            if (output > 127) {
                output = 127;
            }

            if (output < -127) {
                output = -127;
            }


            lift.move(output);

            previousError = error;

        } else {

            // Reset PID calculations
            // DO NOT control lift motor here
            // Driver control needs full control
            previousError = 0;
            integral = 0;
        }


        pros::delay(10);
    }
}

// TEMPORARY: lift PID tuning mode
// UP / DOWN  = move lift to high / low target
// A / B      = kP up / down
// X / Y      = kD up / down

// =========================
// START LIFT PID
// =========================

void liftstart() {

    // Hold current position when PID starts
    liftTarget =
        liftencoder.get_position();

    liftPIDRunning = true;
}


// =========================
// STOP LIFT PID
// =========================

void liftstop() {

    liftPIDRunning = false;

    // Stop lift once
    lift.move(0);
}


// =========================
// UPDATE LIFT TARGET
//
// DOES NOT WAIT
// =========================

void LiftMove(int position) {

    // Only changes target.
    // PID task moves lift in background.
    liftTarget = position;
}


// =========================
// INITIALIZE
// =========================

void initialize() {

    pros::lcd::initialize();


    // =========================
    // BRAKE MODES
    // =========================

    leftDrive.set_brake_mode(
        pros::E_MOTOR_BRAKE_COAST
    );

    rightDrive.set_brake_mode(
        pros::E_MOTOR_BRAKE_COAST
    );

    lift.set_brake_mode(
        pros::E_MOTOR_BRAKE_HOLD
    );

    claw.set_brake_mode(
        pros::E_MOTOR_BRAKE_HOLD
    );

    intake.set_brake_mode(
        pros::E_MOTOR_BRAKE_COAST
    );

    toggleMotor.set_brake_mode(
        pros::E_MOTOR_BRAKE_HOLD
    );


    // =========================
    // RESET LIFT
    // =========================

    liftencoder.reset_position();


    // =========================
    // CALIBRATE LEMLIB
    // =========================

    pros::lcd::set_text(
        0,
        "Calibrating..."
    );

    chassis.calibrate();

    chassis.setPose(
        0,
        0,
        0
    );


    // =========================
    // LIFT PID TASK
    // =========================

    static pros::Task liftTask(
        liftPIDTask,
        nullptr
    );


    // =========================
    // SCREEN TASK
    // =========================

    static pros::Task screenTask([]() {

        while (true) {

            lemlib::Pose pose =
                chassis.getPose();


            pros::lcd::print(
                0,
                "X: %.2f",
                pose.x
            );

            pros::lcd::print(
                1,
                "Y: %.2f",
                pose.y
            );

            pros::lcd::print(
                2,
                "Theta: %.2f",
                pose.theta
            );


            lemlib::telemetrySink()->info(
                "Chassis pose: {}",
                pose
            );


            pros::delay(50);
        }
    });
}


void disabled() {

}


void competition_initialize() {

}
void PIDangular () {

	chassis.setPose(0, 0, 0);
	chassis.turnToHeading(90, 15000);
}

void PIDlateral() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, 24, 150000);

}

void testauto() {
	chassis.setPose(0, 0, 0);
	closeClaw();
	pros::delay(200);
	liftstart();
	LiftMove(500);
	pros::delay(500);

	//chassis.turnToHeading(-45, 500);
	//LiftMove(2000);
	//pros::delay(200);
	//chassis.moveToPoint(-16, 0, 1500, {.maxSpeed = 100});
	//chassis.waitUntilDone();
	//LiftMove(-1500);
	

	

}
void far2plus1(){
    chassis.setPose(0, 0, 0);
chassis.arcade(-60, 0);
closeClaw();
pros::delay(200);
toggleMotor.move(127);
pros::delay(600);
toggleMotor.move(0);
chassis.arcade(0, 0);
chassis.setPose(0,0,0);
    liftstart();

    chassis.moveToPoint( 0, 8, 1500,{.minSpeed = 90});
    chassis.waitUntilDone();
    // Update target and immediately move on
    LiftMove(2000);
    chassis.turnToHeading(-90,1000);
chassis.moveToPoint(-18,9,2000,{.minSpeed = 90} );
chassis.waitUntilDone();
chassis.arcade(40,0);
pros::delay(300);
chassis.arcade(0,0);
pros::delay(200);
chassis.arcade(-30,0);
pros::delay(250);
chassis.arcade(0,0);
pros::delay(200);
LiftMove(500);
pros::delay(300);
openClaw();
pros::delay(200);
LiftMove(3000);
pros::delay(300);

liftstop();
}

void close2plus1(){
chassis.setPose(0, 0, 0);
chassis.arcade(-60, 0);
closeClaw();
pros::delay(200);
toggleMotor.move(127);
pros::delay(600);
toggleMotor.move(0);
chassis.arcade(0, 0);
chassis.setPose(0,0,0);
liftstart();
chassis.moveToPoint( 0, 8, 1500,{.minSpeed = 90});
chassis.waitUntilDone();
// Update target and immediately move on
LiftMove(2000);
chassis.turnToHeading(90,1000);
chassis.moveToPoint(20, 15.5 ,2000,{.minSpeed = 90} );
chassis.waitUntilDone();
chassis.arcade(44,0);
pros::delay(300);
chassis.arcade(0,0);
pros::delay(200);
chassis.arcade(-34,0);
pros::delay(250);
chassis.arcade(0,0);
pros::delay(200);
LiftMove(500);
pros::delay(300);
openClaw();
pros::delay(200);
LiftMove(3000);

liftstop();
liftstart();
chassis.moveToPoint(13, 12, 600, {.forwards = false, .minSpeed = 90});
chassis.waitUntilDone();
chassis.turnToHeading(0, 1000);
chassis.moveToPoint(62, 23, 1500,{.minSpeed = 90});
chassis.waitUntilDone();
chassis.turnToHeading(180, 1000);
chassis.waitUntilDone();
LiftMove(500);
chassis.moveToPoint(65.5, 7.75, 1500,{.minSpeed = 80});
chassis.waitUntilDone();
pros::delay(500);
closeClaw();
pros::delay(500);
chassis.moveToPoint(67, 9, 1500,{.forwards = false, .minSpeed = 80});
chassis.waitUntilDone();


}


// =========================
// EXAMPLE: DISTANCE SENSOR RESET
//
// Uses FIELD-CENTER coordinates (0,0 = middle of the field, walls at +/-70.2).
// The coordinates here are EXAMPLES - change them to your real start spot.
//
// Pattern: move -> waitUntilDone -> distanceResetXY -> keep driving
// =========================

void distanceResetExample() {

    // Start in field coordinates, NOT setPose(0, 0, 0)
    // (example: near the -X wall, facing +Y)
    chassis.setPose(-60, -36, 0);

    // Back up near the -X and -Y walls.
    // Heading stays 0, so the back sensor faces the -Y wall
    // and the left sensor faces the -X wall.
    // Not right in the corner: the corner loader blocks the -X wall there.
    chassis.moveToPoint(-58, -48, 2000, {.forwards = false});

    // ALWAYS wait until the robot has stopped before a reset
    chassis.waitUntilDone();

    // Fix X with the left sensor and Y with the back sensor
    distanceResetXY(leftSensor, backSensor);

    // Normal LemLib motions keep working with the corrected pose
    chassis.moveToPose(-24, -24, 45, 3000);
    chassis.waitUntilDone();
}


// =========================
// TEST AUTO: uses every motion type once
//
// Start: robot on the -Y wall side, 35 in left of center, back 10 in
// from the wall, facing +Y (away from the wall): x -35, y -60, heading 0.
// The route stays clear of the Override goals and wall stacks.
// =========================

void testAuto() {
    // 1. Field coordinates + MCL for the whole run
    chassis.setPose(-35, -60, 0);
    startMcl();

    // 2. Plain LemLib: drive 20 in forward
    chassis.moveToPoint(-35, -40, 2000);
    chassis.waitUntilDone();

    // 3. Smooth curve through 3 points (RAMSETE)
    driveThrough({{-30, -30}, {-24, -20}, {-12, -14}}, 4000);

    // 4. Face -Y, then a circular arc: 12 in radius, clockwise to face -X
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    arcTurn(12, 270);

    // 5. Smooth curve BACKWARDS toward the start
    //    (turns first, since the back of the robot is facing the wrong way)
    driveThrough({{-30, -40}, {-35, -52}}, 4000, {.forwards = false});

    // 6. Square up and fix X and Y with the distance sensors
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    distanceResetXY(leftSensor, backSensor);

    // 7. Plain LemLib again, with the corrected position
    chassis.moveToPoint(-35, -36, 2000);
    chassis.waitUntilDone();

    stopMcl();
}


// =========================
// EXAMPLE: SAME ROUTE, STOP-AND-GO vs SMOOTH
//
// Field-center coordinates. EXAMPLE coordinates - change them.
// =========================

// Stops (or nearly) at every point
void stopAndGoExample() {
    chassis.setPose(-60, -36, 0);
    chassis.moveToPoint(-60, -12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-42, 6, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 14, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-12, 24, 2000);
    chassis.waitUntilDone();
}

// One smooth curve through the same points (RAMSETE + MCL)
void smoothExample() {
    chassis.setPose(-60, -36, 0);
    driveThrough({{-60, -12}, {-42, 6}, {-20, 14}, {-12, 24}}, 6000);

    // Back the way we came, driving backwards
    driveThrough({{-30, 4}, {-60, -24}}, 5000, {.forwards = false});

    stopMcl();
}


// =========================
// EXAMPLE: MCL + RAMSETE
//
// Field-center coordinates. EXAMPLE coordinates - change them.
// Measure DRIVE_KS/KV/KA first (see measureDriveFeedforward) or RAMSETE
// will only be roughly right.
// =========================

void mclRamseteExample() {

    // Start in field coordinates, then start MCL (always AFTER setPose)
    chassis.setPose(-60, -36, 0);
    startMcl();

    // Smooth path through these points. The first point is where the robot is,
    // and the first segment points the way the robot faces (+Y).
    // Third number = speed limit there (0-1), e.g. slow down near the end.
    Path out = pathFromPoints({{-60, -36}, {-60, -24}, {-48, -6}, {-24, 0, 0.5}});
    followPathRamsete(out, 4000);

    // Drive the same points back, backwards
    Path back = pathFromPoints({{-24, 0}, {-48, -6}, {-60, -24}, {-60, -36}});
    followPathRamsete(back, 4000, true);

    // LemLib motions still work while MCL runs in the background
    chassis.moveToPose(-48, -48, 90, 3000);
    chassis.waitUntilDone();

    // Paths drawn in path.jerryio also work (put the file in static/):
    //   ASSET(skills1_txt);   <- at the top of the file, outside any function
    //   followPathRamsete(pathFromAsset(skills1_txt), 5000);

    stopMcl();
}


// =========================
// AUTONOMOUS
// =========================

void autonomous() {
    testauto();
}


// =========================
// DRIVER CONTROL
// =========================

void opcontrol() {

    // Disable autonomous lift PID
    // Driver now has full control
    liftstop();


    while (true) {


        // =========================
        // ARCADE DRIVE
        // =========================

        int forward =
            master.get_analog(
                pros::E_CONTROLLER_ANALOG_LEFT_Y
            );

        int turn =
            master.get_analog(
                pros::E_CONTROLLER_ANALOG_RIGHT_X
            );


        leftDrive.move(
            forward + turn
        );

        rightDrive.move(
            forward - turn
        );


        // =========================
        // CLAW + INTAKE
        // =========================

        if (
            master.get_digital(
                pros::E_CONTROLLER_DIGITAL_L1
            )
        ) {

            claw.move_voltage(5000);

            intake.move(0);

        } else {

            claw.move_voltage(-2000);

            intake.move(127);
        }


        // =========================
        // PORT 5 MOTOR
        // =========================

        if (
            master.get_digital(
                pros::E_CONTROLLER_DIGITAL_L2
            )
        ) {

            toggleMotor.move(127);

        } else {

            toggleMotor.move(0);
        }


        // =========================
        // LIFT
        // =========================

        bool r1 =
            master.get_digital(
                pros::E_CONTROLLER_DIGITAL_R1
            );

        bool r2 =
            master.get_digital(
                pros::E_CONTROLLER_DIGITAL_R2
            );


        if (r1 && r2) {

            lift.move(-30);

        } else if (r1) {

            lift.move(127);

        } else if (r2) {

            lift.move(-127);

        } else {

            lift.move(0);
        }


        pros::delay(10);
    }
}