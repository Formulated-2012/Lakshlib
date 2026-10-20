# Overriderobotv1

Robot code for our VEX V5 robot for the 2026-27 game **Override**.

- **PROS** kernel 4.2.2
- **LemLib** 0.5.6 (stable). Do **not** upgrade to LemLib 0.6; it is a full rewrite with a different API.

## What's in here

| File | What it does |
| --- | --- |
| `src/main.cpp` | Motors, LemLib chassis, lift PID, autons, driver control |
| `include/distance_reset.hpp`, `src/distance_reset.cpp` | Distance sensor position resets |
| `include/arc_turn.hpp`, `src/arc_turn.cpp` | `arcTurn()`: drive along a circular arc to a heading |
| `include/mcl.hpp`, `src/mcl.cpp` | Monte Carlo Localization: distance sensors correct X/Y while driving |
| `include/ramsete.hpp`, `src/ramsete.cpp` | RAMSETE path following on a timetable |
| `include/robot_constants.hpp` | Track width, wheel size, drive RPM (shared by LemLib, arcTurn, RAMSETE) |
| `include/rw/`, `src/rw/` | Math ported from [RW-Template](https://github.com/richardbwang/RW-Template) (MIT). Don't edit unless porting a fix |

LemLib's own files (`include/lemlib/`, `firmware/LemLib.a`) are **not** modified. Everything here is added beside it.

---

## Distance sensor position resets

Odometry drifts over a run. A V5 Distance Sensor pointed at a field perimeter wall tells us exactly how far away that wall is, so we can fix **one axis** (X or Y) of the pose:

1. Read the sensor (mm), throw out bad readings (error, nothing seen, too far, low confidence), convert to inches.
2. Work out where the sensor is on the field from the robot pose and the sensor's offsets.
3. Work out which wall the beam is pointing at (+Y, +X, -Y, or -X). Skip it if the robot is more than 20 degrees off square to that wall.
4. Solve for the robot's position on that one axis, using the real beam angle so it still works when the robot isn't perfectly square. Skip it if the correction is more than 6 in, since that probably means a bad reading.
5. Call `chassis.setPose()` with only that axis changed. Heading is kept.

All normal LemLib motions (`moveToPoint`, `moveToPose`, `turnToHeading`, `turnToPoint`, `swingToHeading`, `follow`, motion chaining) work as before. The reset only changes the pose **between** motions.

### Functions

```cpp
bool distanceReset(DistSensor& s);                  // one sensor, one axis
bool distanceResetXY(DistSensor& a, DistSensor& b); // short settle delay, then both
```

`distanceReset` returns `true` if it applied a correction. `distanceResetXY` returns `true` only if **both** sensors applied one.

### Coordinate convention

The resets need **field-center coordinates**:

- (0, 0) is the center of the field. Units are inches.
- Walls are at about +/-70.2 in (interior is about 140.4 in wall to wall).
- LemLib heading: 0 = facing +Y, 90 = facing +X, clockwise is positive.

Any auton that uses resets must **start with its real field position**, e.g. `chassis.setPose(-60, -36, 0)`, **not** `setPose(0, 0, 0)`. Our older autons (`far2plus1`, `close2plus1`, ...) still use start-relative `(0, 0, 0)`. Don't call resets in them until they're converted to field coordinates.

### Configuring for a new robot

Edit **only** the `CONFIG` section at the top of `src/distance_reset.cpp`:

```cpp
// { name, port, x_off, y_off, mount }
DistSensor backSensor  = {"back",  pros::Distance(7),  0.0, -6.0, 180};
DistSensor leftSensor  = {"left",  pros::Distance(8), -5.5,  0.0, 270};
DistSensor rightSensor = {"right", pros::Distance(9),  5.5,  0.0,  90};
```

- **port**: smart port of the sensor.
- **x_off**: inches to the **right** of the center of rotation (negative = left).
- **y_off**: inches **forward** of the center of rotation (negative = back).
- **mount**: where the sensor faces compared to the robot front: 0 front, 90 right, 180 back, 270 left.
- Measure offsets from the robot's **center of rotation** to the **face** of the sensor.

The same section also has the wall positions, safety limits (60 in max reading, 6 in max correction, 20 degrees max angle, confidence 40), the settle delay, and the debug flag.

> The ports and offsets in the file right now are **placeholders** (ports 7, 8, 9). Measure the real robot before using them.

If the chassis object isn't named `chassis`, change the `extern lemlib::Chassis chassis;` line at the top of `distance_reset.cpp` and `arc_turn.cpp`.

### When to call resets

Resets **never run automatically**. Rules:

- Only call them in autonomous, when the robot is **stopped near a wall**.
- Always call them **right after `chassis.waitUntilDone()`**. Never during a motion. If LemLib is still moving, `distanceReset` refuses and returns `false`.
- Make sure the beam hits the **wall**, not a game element or another robot.

Good spots in Override:

- After flipping a wall toggle
- After leaving a loader
- Before precise stacking
- Often during Programming Skills (every time you pass near a wall)

Example (`distanceResetExample()` in `src/main.cpp`):

```cpp
chassis.setPose(-60, -36, 0);                        // real field start
chassis.moveToPoint(-58, -58, 2000, {.forwards = false});
chassis.waitUntilDone();                             // stopped near the walls
distanceResetXY(leftSensor, backSensor);             // fix X and Y
chassis.moveToPose(-24, -24, 45, 3000);
```

### Testing

The code builds, but it **has not been tested on a real robot yet**. Before trusting it in a match:

1. Set `DISTANCE_RESET_DEBUG = true` (the default). Every reset prints the raw reading, the axis it changed, and the pose before and after to the terminal and to brain screen lines 4-6.
2. Put the robot at a known spot (measure with a tape measure), `setPose` to that spot, then call `distanceReset` on each sensor. The correction should be near 0. If it's large or the **sign is wrong**, check `x_off`, `y_off`, and `mount`.
3. Nudge the robot about 1 in toward a wall without changing the pose and reset again. The correction should be about 1 in in the right direction.
4. Try it at small angles (5-15 degrees off square) and make sure the result stays close.
5. Run a real auton with resets and check that the corrections are usually **under 2 in**. Bigger ones mean drift is worse than expected, or a sensor is seeing the wrong thing.
6. Check readings against a tape measure on the **field type we compete on** (wall distances vary a little between fields).
7. Set `DISTANCE_RESET_DEBUG = false` for competition.

### Mounting guidelines

- Mount **below the top of the perimeter wall**, but high enough not to see pins or cups on the floor.
- Mount **exactly square** to the frame, rigidly, on structure that won't flex or get hit. Inset it slightly from the robot edge.
- **Back + left + right** is a strong layout. Left and right let mirrored red/blue routines both use a side sensor.
- Keep sensors away from the **intake path**, and leave clearance for the beam cone so it doesn't hit the robot itself.
- Measure offsets from the robot's **center of rotation** to the **sensor face**.
- Most accurate within **about 2-3 ft** of the wall. Verify readings against a tape measure on the field type we compete on.

---

## arcTurn

```cpp
arcTurn(radius, targetHeading, maxSpeed = 80, timeout = 2000, forwards = true);
```

Drives along a circle of `radius` inches (circle center to robot center) until the robot reaches `targetHeading`, turning whichever way is shorter. The outer side runs at `maxSpeed` and the inner side at `maxSpeed * (R - w/2) / (R + w/2)`, where `w` is the track width (9.75 in, set in `include/robot_constants.hpp`). It slows down over the last 20 degrees.

It **blocks** until done and isn't a LemLib motion, so it can't be motion-chained. It waits for any running LemLib motion to finish first. **Not tested on a real robot yet.** Expect to tune the slowdown for overshoot.

---

## Monte Carlo Localization (MCL)

Distance resets fix the pose only when we stop and ask. **MCL** works all the time: a background task uses the same distance sensors to keep X/Y from drifting **while the robot drives**. Heading still comes from the IMU.

How it works, in short:

1. A cloud of 200 "particles" (guesses of where the robot is) moves along with odometry, plus a little random spread.
2. Every 30 ms, each clean distance reading makes the guesses that match the walls more likely and the others less likely. The unlikely guesses are replaced by copies of the likely ones.
3. LemLib's pose is **nudged** toward the best guess by at most 0.15 in per axis per update. It only nudges on an axis where MCL is confident, and ignores guesses more than 12 in away (MCL is probably lost then). Motions never see a jump.

It skips readings that are blocked (shorter than the wall should be), hit near a corner, hit the wall too steeply, or came from a robot spinning fast. It accounts for the ~40 ms sensor delay and recovers if the robot gets shoved.

```cpp
chassis.setPose(-60, -36, 0); // real field start (field-center coordinates)
startMcl();                   // always AFTER setPose
// ... normal LemLib motions, RAMSETE, etc ...
stopMcl();                    // optional: back to plain odometry
```

- **Sensors and walls:** MCL uses the sensors, walls and limits from the CONFIG section of `src/distance_reset.cpp`, so set those up first.
- **MCL settings:** particle count, sensor delay, corner margin and debug are in the CONFIG section of `src/mcl.cpp`.
- **setPose while running:** if you call `chassis.setPose()` while MCL is running, call `mclRecenter()` right after. `distanceReset()` already does it, and jumps over 4 in are caught automatically.
- **Changes the pose during motions:** unlike distance resets, MCL moves the pose while motions are running, in tiny steps. If a motion ever acts strangely, run it with MCL stopped to compare.
- **Debug:** with `MCL_DEBUG = true`, brain screen line 3 shows the estimate, the spread (how unsure it is; small = confident) and how many ms each update takes. Keep that under ~5 ms, or lower `MCL_PARTICLES`.

> RW-Template has only tested MCL in simulation, and **our port hasn't run on a robot yet**. Test on the practice field first: drive around with MCL on and check that the pose stays right, compared with a tape measure.

---

## RAMSETE path following

`followPathRamsete()` follows a smooth path **on a timetable**. It plans when to be where (slowing down for curves), then steers and speeds up or slows down to stay on schedule. This makes timing very repeatable, which is useful in skills. It ends near the last point, usually within 1-2 in.

```cpp
Path out = pathFromPoints({{-60, -36}, {-60, -24}, {-48, -6}, {-24, 0, 0.5}});
followPathRamsete(out, 4000);                // timeout 4000 ms
followPathRamsete(back, 4000, true);         // reverse = drive backwards
followPathRamsete(out, 4000, false, false);  // exit = false: end still moving
followPathRamsete(out, 4000, false, true, 40); // max 40 in/s
```

- **Path points:** start the path where the robot is. The first segment should point the way the robot faces. The optional third number in a point is a speed limit there (0-1 of max speed).
- **path.jerryio:** paths drawn there work too. Save in the "LemLib v0.5" format, put the file in `static/`, and use `ASSET(name_txt);` plus `pathFromAsset(name_txt)`. The same file also works with LemLib's `chassis.follow()`.
- **Blocking:** it blocks until done and waits for any running LemLib motion to finish first. It can't be motion-chained with LemLib motions.

### Tuning (do this first!)

RAMSETE drives the motors with **feedforward** (volts = kS + kV * speed + kA * accel). Without measured numbers it guesses from the drive RPM and is only roughly right.

1. Give the robot about 4 ft of open space in front and behind.
2. Temporarily call `measureDriveFeedforward();` (e.g. at the top of `autonomous()`) and run it. It drives back and forth and **never returns**.
3. Copy `DRIVE_KS`, `DRIVE_KV`, `DRIVE_KA` from the brain screen (lines 4-7) or terminal into the CONFIG section of `src/ramsete.cpp`.
4. Remove the call. Then adjust `MAX_SPEED`, `MAX_ACCEL` and `MAX_LATERAL_ACCEL` (lower = gentler, less wheel slip).

> Also check `DRIVE_RPM` and `WHEEL_DIAMETER` in `include/robot_constants.hpp`. The motor cartridge (blue/green/red) is read from the motors automatically.

---

## Credits

MCL, RAMSETE, path building and the jerryio parser are ported from [RW-Template](https://github.com/richardbwang/RW-Template) by Richard Wang, under the MIT License (see `licenses/RW-Template-LICENSE.txt`). The math in `src/rw/` is kept almost unchanged. The robot-side code (`src/mcl.cpp`, `src/ramsete.cpp`) was rewritten for PROS and LemLib.

---

## Building

```bash
pros make
```

Upload with `pros upload` (or the PROS VS Code extension) only when you mean to.
