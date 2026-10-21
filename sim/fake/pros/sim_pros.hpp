// Simulator stand-in for the PROS 4 API (only used by the computer simulator in sim/).
// Covers what LemLib 0.5.6 and our robot code use. Devices talk to the simulated robot in
// sim/engine/sim-engine.cpp. If the sim build says something is missing, add it here the
// same way. Modeled on RW-Template's fake VEX API (sim/vex/v5_vcs.h, MIT License).
#pragma once
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include "../../engine/sim-hooks.h"

#define PROS_ERR (INT32_MAX)
#define PROS_ERR_F (INFINITY)
#define PROS_SUCCESS (1)

// LemLib uses isnanf and infinity() from the V5's C library (newlib), which desktop
// compilers don't have. (Real PROS headers also pull in <optional> etc., included above.)
#ifndef isnanf
#define isnanf(x) std::isnan(x)
#endif
inline double infinity() { return INFINITY; }

#define TASK_PRIORITY_MAX 16
#define TASK_PRIORITY_DEFAULT 8
#define TASK_PRIORITY_MIN 1
#define TASK_STACK_DEPTH_DEFAULT 0x2000
#define TASK_STACK_DEPTH_MIN 0x200
#define TIMEOUT_MAX ((uint32_t)0xffffffffUL)

#define COMPETITION_DISABLED (1 << 0)
#define COMPETITION_AUTONOMOUS (1 << 1)
#define COMPETITION_CONNECTED (1 << 2)
#define COMPETITION_SYSTEM (1 << 3)

namespace pros {

// ============================================================================
// TIME AND TASKS
// ============================================================================
inline void delay(uint32_t ms) { rwsim::sleepMs(ms); }
inline uint32_t millis() { return (uint32_t)rwsim::nowMs(); }
inline uint64_t micros() { return (uint64_t)(rwsim::nowMs() * 1000.0); }

typedef void (*task_fn_t)(void*);
enum task_state_e_t { E_TASK_STATE_RUNNING = 0, E_TASK_STATE_READY, E_TASK_STATE_BLOCKED,
                      E_TASK_STATE_SUSPENDED, E_TASK_STATE_DELETED, E_TASK_STATE_INVALID };

class Task {
    int32_t id_ = -1;

  public:
    Task(task_fn_t function, void* parameters = nullptr, std::uint32_t prio = TASK_PRIORITY_DEFAULT,
         std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT, const char* name = "") {
        (void)prio; (void)stack_depth; (void)name;
        id_ = rwsim::spawnThread([function, parameters] { function(parameters); });
    }
    Task(task_fn_t function, void* parameters, const char* name) : Task(function, parameters) { (void)name; }

    // Lambdas and other callables with no arguments
    template <class F, typename = std::enable_if_t<std::is_invocable_v<F>>>
    explicit Task(F&& function, std::uint32_t prio = TASK_PRIORITY_DEFAULT,
                  std::uint16_t stack_depth = TASK_STACK_DEPTH_DEFAULT, const char* name = "") {
        (void)prio; (void)stack_depth; (void)name;
        id_ = rwsim::spawnThread(std::function<void()>(std::forward<F>(function)));
    }
    template <class F, typename = std::enable_if_t<std::is_invocable_v<F>>>
    Task(F&& function, const char* name) : Task(std::forward<F>(function)) { (void)name; }

    static void delay(uint32_t ms) { pros::delay(ms); }
    static void delay_until(uint32_t* const prev_time, const uint32_t delta) {
        *prev_time += delta;
        double wait = (double)*prev_time - rwsim::nowMs();
        rwsim::sleepMs(wait > 0 ? wait : 0);
    }
    void remove() { if (id_ >= 0) rwsim::interruptThread(id_); }
    void suspend() {}
    void resume() {}
    uint32_t notify() { return 1; }
    uint32_t notify_take(bool clear_on_exit, uint32_t timeout) { (void)clear_on_exit; pros::delay(timeout == TIMEOUT_MAX ? 10 : timeout); return 0; }
    uint32_t get_priority() { return TASK_PRIORITY_DEFAULT; }
    void set_priority(uint32_t prio) { (void)prio; }
    task_state_e_t get_state() { return rwsim::threadRunning(id_) ? E_TASK_STATE_READY : E_TASK_STATE_DELETED; }
    const char* get_name() { return ""; }
};

// Cooperative scheduler: only one task runs at a time, so taking a free mutex can't race.
// A held mutex waits (letting other tasks run) like the real one.
class Mutex {
    bool locked_ = false;

  public:
    bool take(std::uint32_t timeout = TIMEOUT_MAX) {
        double start = rwsim::nowMs();
        while (locked_) {
            if (timeout != TIMEOUT_MAX && rwsim::nowMs() - start >= timeout) return false;
            rwsim::sleepMs(1);
        }
        locked_ = true;
        return true;
    }
    bool give() { locked_ = false; return true; }
    void lock() { take(); }
    void unlock() { give(); }
    bool try_lock() { return take(0); }
};

namespace c {
inline uint32_t millis() { return pros::millis(); }
inline uint64_t micros() { return pros::micros(); }
inline void delay(uint32_t ms) { pros::delay(ms); }
}

// ============================================================================
// COMPETITION
// ============================================================================
namespace competition {
inline std::uint8_t get_status() {
    std::uint8_t status = COMPETITION_CONNECTED;
    if (!rwsim::competitionEnabled()) status |= COMPETITION_DISABLED;
    if (rwsim::competitionAutonomous()) status |= COMPETITION_AUTONOMOUS;
    return status;
}
inline bool is_autonomous() { return rwsim::competitionAutonomous(); }
inline bool is_disabled() { return !rwsim::competitionEnabled(); }
inline bool is_connected() { return true; }
inline bool is_competition_switch() { return false; }
inline bool is_field_control() { return false; }
}

// ============================================================================
// CONTROLLER (nobody is driving in the simulator: sticks centered, no buttons)
// ============================================================================
typedef enum { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER } controller_id_e_t;
typedef enum {
    E_CONTROLLER_ANALOG_LEFT_X = 0, E_CONTROLLER_ANALOG_LEFT_Y,
    E_CONTROLLER_ANALOG_RIGHT_X, E_CONTROLLER_ANALOG_RIGHT_Y
} controller_analog_e_t;
typedef enum {
    E_CONTROLLER_DIGITAL_L1 = 6, E_CONTROLLER_DIGITAL_L2, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2,
    E_CONTROLLER_DIGITAL_UP, E_CONTROLLER_DIGITAL_DOWN, E_CONTROLLER_DIGITAL_LEFT, E_CONTROLLER_DIGITAL_RIGHT,
    E_CONTROLLER_DIGITAL_X, E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_Y, E_CONTROLLER_DIGITAL_A
} controller_digital_e_t;

class Controller {
  public:
    explicit Controller(controller_id_e_t id) { (void)id; }
    std::int32_t get_analog(controller_analog_e_t channel) { (void)channel; return 0; }
    std::int32_t get_digital(controller_digital_e_t button) { (void)button; return 0; }
    std::int32_t get_digital_new_press(controller_digital_e_t button) { (void)button; return 0; }
    std::int32_t is_connected() { return 1; }
    std::int32_t rumble(const char* pattern) { (void)pattern; return 1; }
    std::int32_t clear() { return 1; }
    std::int32_t clear_line(std::uint8_t line) { (void)line; return 1; }
    std::int32_t set_text(std::uint8_t line, std::uint8_t col, const std::string& text) { (void)line; (void)col; (void)text; return 1; }
    template <typename... Params>
    std::int32_t print(std::uint8_t line, std::uint8_t col, const char* fmt, Params... args) { (void)line; (void)col; (void)fmt; ((void)args, ...); return 1; }
};

namespace c {
inline std::int32_t controller_rumble(controller_id_e_t id, const char* pattern) { (void)id; (void)pattern; return 1; }
}

// ============================================================================
// MOTORS
// ============================================================================
typedef enum motor_brake_mode_e { E_MOTOR_BRAKE_COAST = 0, E_MOTOR_BRAKE_BRAKE = 1, E_MOTOR_BRAKE_HOLD = 2,
                                  E_MOTOR_BRAKE_INVALID = INT32_MAX } motor_brake_mode_e_t;
typedef enum motor_encoder_units_e { E_MOTOR_ENCODER_DEGREES = 0, E_MOTOR_ENCODER_ROTATIONS = 1,
                                     E_MOTOR_ENCODER_COUNTS = 2, E_MOTOR_ENCODER_INVALID = INT32_MAX } motor_encoder_units_e_t;
typedef enum motor_gearset_e { E_MOTOR_GEARSET_36 = 0, E_MOTOR_GEAR_RED = 0, E_MOTOR_GEAR_100 = 0,
                               E_MOTOR_GEARSET_18 = 1, E_MOTOR_GEAR_GREEN = 1, E_MOTOR_GEAR_200 = 1,
                               E_MOTOR_GEARSET_06 = 2, E_MOTOR_GEAR_BLUE = 2, E_MOTOR_GEAR_600 = 2,
                               E_MOTOR_GEARSET_INVALID = INT32_MAX } motor_gearset_e_t;

enum class MotorBrake { coast = 0, brake = 1, hold = 2, invalid = INT32_MAX };
enum class MotorUnits { degrees = 0, deg = 0, rotations = 1, counts = 2, invalid = INT32_MAX };
using MotorEncoderUnits = MotorUnits;
enum class MotorGears { ratio_36_to_1 = 0, red = 0, rpm_100 = 0,
                        ratio_18_to_1 = 1, green = 1, rpm_200 = 1,
                        ratio_6_to_1 = 2, blue = 2, rpm_600 = 2, invalid = INT32_MAX };
using MotorGearset = MotorGears;
using MotorCart = MotorGears;

namespace sim {
// Encoder units are per port (two objects on one port share a real motor)
inline std::map<int, int>& encoderUnits() { static std::map<int, int> units; return units; }
inline int portOf(std::int8_t port) { return port < 0 ? -port : port; }
inline double freeRpm(MotorGears gears) { return gears == MotorGears::red ? 100 : (gears == MotorGears::blue ? 600 : 200); }
inline MotorGears gearsOf(int port) {
    double rpm = rwsim::motorFreeRpm(port);
    return rpm < 150 ? MotorGears::red : (rpm < 400 ? MotorGears::green : MotorGears::blue);
}
// In the simulator, a positive command always moves that side of the robot forward
// (the real port reversal just undoes how the motor is mounted)
inline void volts(int port, double v) {
    if (std::fabs(v) < 1e-6) rwsim::motorStop(port, -1); // 0 V = stop using the brake mode
    else rwsim::motorVoltage(port, v);
}
inline double position(int port) {
    double deg = rwsim::motorPositionDeg(port);
    switch (encoderUnits()[port]) {
        case 1: return deg / 360.0;
        case 2: return deg / 360.0 * (gearsOf(port) == MotorGears::red ? 1800 : (gearsOf(port) == MotorGears::green ? 900 : 300));
        default: return deg;
    }
}
}

class Motor {
    int port_;
    std::int8_t signed_port_;

  public:
    Motor(const std::int8_t port, const MotorGears gearset = MotorGears::invalid,
          const MotorUnits encoder_units = MotorUnits::invalid)
        : port_(sim::portOf(port)), signed_port_(port) {
        if (gearset != MotorGears::invalid) rwsim::motorRegister(port_, sim::freeRpm(gearset));
        if (encoder_units != MotorUnits::invalid) sim::encoderUnits()[port_] = (int)encoder_units;
    }
    std::int32_t move(std::int32_t voltage) { sim::volts(port_, voltage / 127.0 * 12.0); return 1; }
    std::int32_t move_voltage(std::int32_t millivolts) { sim::volts(port_, millivolts / 1000.0); return 1; }
    std::int32_t move_velocity(std::int32_t velocity) {
        if (velocity == 0) rwsim::motorStop(port_, -1);
        else rwsim::motorPercent(port_, velocity / rwsim::motorFreeRpm(port_) * 100.0);
        return 1;
    }
    std::int32_t brake() { rwsim::motorStop(port_, -1); return 1; }
    std::int32_t set_brake_mode(MotorBrake mode, std::uint8_t index = 0) { (void)index; rwsim::motorSetStopping(port_, (int)mode); return 1; }
    std::int32_t set_brake_mode(motor_brake_mode_e_t mode, std::uint8_t index = 0) { (void)index; rwsim::motorSetStopping(port_, (int)mode); return 1; }
    std::int32_t set_encoder_units(MotorUnits units, std::uint8_t index = 0) { (void)index; sim::encoderUnits()[port_] = (int)units; return 1; }
    std::int32_t set_encoder_units(motor_encoder_units_e_t units, std::uint8_t index = 0) { (void)index; sim::encoderUnits()[port_] = (int)units; return 1; }
    std::int32_t set_gearing(MotorGears gearset, std::uint8_t index = 0) { (void)index; rwsim::motorRegister(port_, sim::freeRpm(gearset)); return 1; }
    double get_position(std::uint8_t index = 0) { (void)index; return sim::position(port_); }
    std::int32_t tare_position(std::uint8_t index = 0) { (void)index; rwsim::motorSetPositionDeg(port_, 0); return 1; }
    std::int32_t set_zero_position(double position, std::uint8_t index = 0) { (void)index; rwsim::motorSetPositionDeg(port_, -position); return 1; }
    double get_actual_velocity(std::uint8_t index = 0) { (void)index; return rwsim::motorRpm(port_); }
    MotorGears get_gearing(std::uint8_t index = 0) { (void)index; return sim::gearsOf(port_); }
    MotorBrake get_brake_mode(std::uint8_t index = 0) { (void)index; return MotorBrake::coast; }
    double get_temperature(std::uint8_t index = 0) { (void)index; return 30; }
    double get_power(std::uint8_t index = 0) { (void)index; return 0; }
    std::int32_t get_current_draw(std::uint8_t index = 0) { (void)index; return 0; }
    std::int32_t get_voltage(std::uint8_t index = 0) { (void)index; return 0; }
    std::int8_t get_port(std::uint8_t index = 0) { (void)index; return signed_port_; }
    std::int32_t is_reversed(std::uint8_t index = 0) { (void)index; return signed_port_ < 0; }
    std::int8_t size() { return 1; }
};

class MotorGroup {
    std::vector<std::int8_t> ports_;
    std::vector<MotorBrake> brake_modes_;

  public:
    MotorGroup(std::initializer_list<std::int8_t> ports, const MotorGears gearset = MotorGears::invalid,
               const MotorUnits encoder_units = MotorUnits::invalid)
        : ports_(ports), brake_modes_(ports.size(), MotorBrake::coast) {
        for (std::int8_t p : ports_) Motor(p, gearset, encoder_units);
    }
    MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears gearset = MotorGears::invalid,
               const MotorUnits encoder_units = MotorUnits::invalid)
        : ports_(ports), brake_modes_(ports.size(), MotorBrake::coast) {
        for (std::int8_t p : ports_) Motor(p, gearset, encoder_units);
    }

    // The simulator reads these to know which ports are the drivetrain
    std::vector<int> sim_ports() const { std::vector<int> out; for (auto p : ports_) out.push_back(sim::portOf(p)); return out; }

    std::int32_t move(std::int32_t voltage) { for (auto p : ports_) Motor(p).move(voltage); return 1; }
    std::int32_t move_voltage(std::int32_t millivolts) { for (auto p : ports_) Motor(p).move_voltage(millivolts); return 1; }
    std::int32_t move_velocity(std::int32_t velocity) { for (auto p : ports_) Motor(p).move_velocity(velocity); return 1; }
    std::int32_t brake() { for (auto p : ports_) Motor(p).brake(); return 1; }

    std::int32_t set_brake_mode(MotorBrake mode, std::uint8_t index = 0) { brake_modes_.at(index) = mode; return Motor(ports_.at(index)).set_brake_mode(mode); }
    std::int32_t set_brake_mode(motor_brake_mode_e_t mode, std::uint8_t index = 0) { return set_brake_mode((MotorBrake)mode, index); }
    std::int32_t set_brake_mode_all(MotorBrake mode) { for (size_t i = 0; i < ports_.size(); i++) set_brake_mode(mode, i); return 1; }
    std::int32_t set_brake_mode_all(motor_brake_mode_e_t mode) { return set_brake_mode_all((MotorBrake)mode); }
    MotorBrake get_brake_mode(std::uint8_t index = 0) { return brake_modes_.at(index); }
    std::vector<MotorBrake> get_brake_mode_all() { return brake_modes_; }

    std::int32_t set_encoder_units(MotorUnits units, std::uint8_t index = 0) { return Motor(ports_.at(index)).set_encoder_units(units); }
    std::int32_t set_encoder_units_all(MotorUnits units) { for (auto p : ports_) Motor(p).set_encoder_units(units); return 1; }
    std::int32_t set_encoder_units_all(motor_encoder_units_e_t units) { for (auto p : ports_) Motor(p).set_encoder_units(units); return 1; }
    std::int32_t set_gearing_all(MotorGears gearset) { for (auto p : ports_) Motor(p).set_gearing(gearset); return 1; }

    double get_position(std::uint8_t index = 0) { return Motor(ports_.at(index)).get_position(); }
    std::vector<double> get_position_all() { std::vector<double> v; for (auto p : ports_) v.push_back(Motor(p).get_position()); return v; }
    std::int32_t tare_position(std::uint8_t index = 0) { return Motor(ports_.at(index)).tare_position(); }
    std::int32_t tare_position_all() { for (auto p : ports_) Motor(p).tare_position(); return 1; }
    double get_actual_velocity(std::uint8_t index = 0) { return Motor(ports_.at(index)).get_actual_velocity(); }
    std::vector<double> get_actual_velocity_all() { std::vector<double> v; for (auto p : ports_) v.push_back(Motor(p).get_actual_velocity()); return v; }
    MotorGears get_gearing(std::uint8_t index = 0) { return Motor(ports_.at(index)).get_gearing(); }
    std::vector<MotorGears> get_gearing_all() { std::vector<MotorGears> v; for (auto p : ports_) v.push_back(Motor(p).get_gearing()); return v; }
    double get_temperature(std::uint8_t index = 0) { (void)index; return 30; }
    std::vector<double> get_temperature_all() { return std::vector<double>(ports_.size(), 30); }
    std::int8_t get_port(std::uint8_t index = 0) { return ports_.at(index); }
    std::vector<std::int8_t> get_port_all() { return ports_; }
    std::int8_t size() { return (std::int8_t)ports_.size(); }
};

// ============================================================================
// SENSORS
// ============================================================================
enum class ImuStatus { ready = 0, calibrating = 1, error = 0xFF };

class Imu {
    int port_;

  public:
    explicit Imu(const std::uint8_t port) : port_(port) {}
    std::int32_t reset(bool blocking = false) {
        rwsim::imuCalibrate(port_);
        while (blocking && rwsim::imuCalibrating(port_)) pros::delay(10);
        return 1;
    }
    bool is_calibrating() { return rwsim::imuCalibrating(port_); }
    ImuStatus get_status() { return is_calibrating() ? ImuStatus::calibrating : ImuStatus::ready; }
    double get_rotation() { return rwsim::imuRotationDeg(port_); }
    double get_heading() { double h = std::fmod(get_rotation(), 360.0); return h < 0 ? h + 360.0 : h; }
    std::int32_t set_rotation(double target) { rwsim::imuSetRotationDeg(port_, target); return 1; }
    std::int32_t set_heading(double target) { rwsim::imuSetRotationDeg(port_, get_rotation() - get_heading() + target); return 1; }
    std::int32_t tare_rotation() { return set_rotation(0); }
    std::int32_t tare_heading() { return set_heading(0); }
    std::int32_t tare() { return set_rotation(0); }
    std::uint8_t get_port() { return (std::uint8_t)port_; }
};

class Rotation {
    int port_;

  public:
    explicit Rotation(const std::int8_t port) : port_(port < 0 ? -port : port) {}
    std::int32_t get_position() { return (std::int32_t)std::lround(rwsim::rotationSensorDeg(port_) * 100); } // centidegrees
    std::int32_t get_angle() { double a = std::fmod(rwsim::rotationSensorDeg(port_), 360.0); return (std::int32_t)std::lround((a < 0 ? a + 360 : a) * 100); }
    std::int32_t get_velocity() { return (std::int32_t)std::lround(rwsim::rotationSensorRpm(port_) * 600); } // centidegrees/s
    std::int32_t reset_position() { rwsim::rotationSensorSetDeg(port_, 0); return 1; }
    std::int32_t reset() { return reset_position(); }
    std::int32_t set_position(std::uint32_t position) { rwsim::rotationSensorSetDeg(port_, position / 100.0); return 1; }
    std::uint8_t get_port() { return (std::uint8_t)port_; }
};

class Distance {
    int port_;

  public:
    explicit Distance(const std::uint8_t port) : port_(port) {}
    std::int32_t get_distance() { return (std::int32_t)std::lround(rwsim::distanceMm(port_)); }
    std::int32_t get() { return get_distance(); }
    std::int32_t get_confidence() { return rwsim::distanceMm(port_) >= 9999 ? 0 : 63; }
    std::int32_t get_object_size() { return rwsim::distanceMm(port_) >= 9999 ? -1 : 200; }
    double get_object_velocity() { return 0; }
    std::uint8_t get_port() { return (std::uint8_t)port_; }
};

namespace adi {
class Encoder {
  public:
    Encoder(std::uint8_t top, std::uint8_t bottom, bool reversed = false) { (void)top; (void)bottom; (void)reversed; }
    std::int32_t get_value() { return 0; }
    std::int32_t reset() { return 1; }
};
}

// ============================================================================
// BRAIN SCREEN (llemu) - the simulator doesn't draw it
// ============================================================================
namespace lcd {
inline bool initialize() { return true; }
inline bool is_initialized() { return true; }
inline bool set_text(std::int16_t line, std::string text) { (void)line; (void)text; return true; }
inline bool clear() { return true; }
inline bool clear_line(std::int16_t line) { (void)line; return true; }
template <typename... Params>
bool print(std::int16_t line, const char* fmt, Params... args) { (void)line; (void)fmt; ((void)args, ...); return true; }
inline void register_btn0_cb(void (*cb)()) { (void)cb; }
inline void register_btn1_cb(void (*cb)()) { (void)cb; }
inline void register_btn2_cb(void (*cb)()) { (void)cb; }
}

}  // namespace pros
