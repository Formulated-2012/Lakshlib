#pragma once

// =========================
// ARC TURN
//
// Drives the robot along a circle (like RW-Template's curveCircle) until it
// reaches a target heading. The outer side runs at maxSpeed and the inner
// side runs at:
//
//     maxSpeed * (R - w/2) / (R + w/2)
//
// where R = radius of the circle (to robot center) and w = track width.
//
// It turns whichever way is shorter to targetHeading.
// It BLOCKS until done (or timeout). It is NOT a LemLib motion, so it can't
// be motion-chained; LemLib motions before/after still work normally.
// =========================

// radius:        inches, from the circle center to the robot center (>= 0)
// targetHeading: degrees, LemLib convention (0 = +Y, clockwise positive)
// maxSpeed:      outer wheel speed, 0-127
// timeout:       ms, gives up after this long
// forwards:      false = drive the arc backwards
void arcTurn(double radius, double targetHeading, int maxSpeed = 80,
             int timeout = 2000, bool forwards = true);
