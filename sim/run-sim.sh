#!/bin/sh
# Builds the simulator from our robot code + LemLib source and runs one auton on a simulated
# robot and Override field. Writes an HTML replay to sim/out/.
# Usage (from Git Bash in the project folder):  sh sim/run-sim.sh far2plus1 [options]
#        sh sim/run-sim.sh --help    for options,   sh sim/run-sim.sh --list   for autons
set -e
cd "$(dirname "$0")/.."
. sim/find-compiler.sh
mkdir -p sim/build sim/out

BIN=sim/build/robot-sim
# Rebuild only when code changed
if [ ! -f "$BIN" ] && [ ! -f "$BIN.exe" ] || [ -n "$(find src include sim -newer "$(ls $BIN $BIN.exe 2>/dev/null | head -1)" \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) 2>/dev/null | head -1)" ]; then
  echo "Building the simulator (first build takes a minute)..."
  # A header in include/ that says #include "pros/..." would find the REAL include/pros/ next to
  # it before any -I folder. So copy our top-level headers somewhere without a pros/ folder.
  rm -rf sim/build/include && mkdir -p sim/build/include
  for h in include/*.hpp include/*.h; do
    case "$(basename "$h")" in main.h|api.h) ;; *) cp "$h" sim/build/include/ ;; esac
  done
  # -I order matters: sim/fake (fake PROS + main.h) must come before include/ (real PROS)
  "$CXX" -std=gnu++17 -O1 -w -DFMT_HEADER_ONLY $CXXFLAGS \
    -I sim/fake -I sim -I sim/build/include -I include -I include/lemlib \
    $(find src -name '*.cpp') $(find sim/lemlib-src -name '*.cpp') \
    sim/engine/sim-engine.cpp sim/sim-autons.cpp sim/sim-main.cpp \
    -o "$BIN" -pthread
fi
"$BIN" "$@"
