# Finds a desktop C++ compiler (not the ARM one PROS uses). Sets CXX and CXXFLAGS.
# Windows: install WinLibs with   winget install BrechtSanders.WinLibs.POSIX.UCRT
CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
  # winget puts WinLibs here; a terminal opened before installing won't have it on PATH yet
  for dir in "$LOCALAPPDATA"/Microsoft/WinGet/Packages/BrechtSanders.WinLibs*/mingw64/bin \
             "$LOCALAPPDATA"/Microsoft/WinGet/Links /c/mingw64/bin; do
    if [ -x "$dir/g++.exe" ]; then
      PATH="$dir:$PATH"
      export PATH
      break
    fi
  done
fi
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "No C++ compiler found. Install one, then open a new terminal and run this again:"
  echo "  Windows: winget install BrechtSanders.WinLibs.POSIX.UCRT"
  echo "  macOS:   xcode-select --install"
  echo "  Linux:   sudo apt install g++"
  exit 1
fi
