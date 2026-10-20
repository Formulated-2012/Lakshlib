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

Drives along a circle of `radius` inches (circle center to robot center) until the robot reaches `targetHeading`, turning whichever way is shorter. The outer side runs at `maxSpeed` and the inner side at `maxSpeed * (R - w/2) / (R + w/2)`, where `w` is the track width (9.75 in, set in `src/arc_turn.cpp`). It slows down over the last 20 degrees.

It **blocks** until done and isn't a LemLib motion, so it can't be motion-chained. It waits for any running LemLib motion to finish first. **Not tested on a real robot yet.** Expect to tune the slowdown for overshoot.

---

## Building

```bash
pros make
```

Upload with `pros upload` (or the PROS VS Code extension) only when you mean to.
