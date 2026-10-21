#!/bin/sh
# Turns a drive log recorded on the robot (calibrateDrive()) into a robot profile, so the
# simulator drives like our robot. Optional: the simulator works without it.
# Usage: sh sim/fit-profile.sh drive_log.csv [output]     (or --self-test)
# From RW-Template (sim/fit-profile.sh), Copyright (c) 2025 Richard Wang, MIT License.
set -e
# Keep the log's path working after moving to the project folder
if [ -f "$1" ]; then
  LOG="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
  shift
  set -- "$LOG" "$@"
fi
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
mkdir -p sim/build

"$CXX" -std=gnu++17 -O2 -w $CXXFLAGS -I sim/engine sim/engine/fit-profile.cpp -o sim/build/fit-profile
sim/build/fit-profile "$@"
