# EchoSim UserControllerTest CMake Project

This repository contains a CMake-based C++ user controller for EchoSim. It
loads terrain data, plans a global path, tracks the path, and publishes
vehicle control commands.

The original Visual Studio `Cplusplus` project is not modified.
`PlanningSelfTest.cpp` is intentionally excluded from this target.

## Requirements

- Visual Studio 2022 with the C++ x64 toolchain
- CMake and Ninja available in the VS x64 development environment
- EchoSim SDK and OpenCV 4.12.0 installed or provided separately
- Moon terrain data for the selected task

The SDK, runtime DLLs, terrain data, and generated output are intentionally not
committed to this repository. See [docs/DEPENDENCIES.md](docs/DEPENDENCIES.md)
for the required local layout and fresh-clone configuration commands.

## Build

From an x64 Native Tools Command Prompt for VS 2022, configure the dependency
roots explicitly for a clean checkout:

```bat
cmake -S . -B out\build\x64-Debug -G Ninja ^
  -DCMAKE_BUILD_TYPE=Debug ^
  -DECHOSIM_DEPENDENCY_ROOT=C:\path\to\echosim ^
  -DOPENCV_ROOT=C:\path\to\opencv
cmake --build out\build\x64-Debug --parallel
```

Use `x64-Release` and `-DCMAKE_BUILD_TYPE=Release` for a Release build.
Visual Studio can use the checked-in `CMakeSettings.json` configurations after
the local dependency paths are available.

The executable and copied runtime DLLs are generated in
`out\build\x64-<configuration>\bin`.

## Configuration

Edit `makeDefaultTaskConfig()` in `src/TaskConfig.cpp` for task coordinates,
planner settings, tracking parameters, and debug output. Terrain data can be
selected without changing source code by setting
`USER_CONTROLLER_TERRAIN_ROOT` or `ECHOSIM_TERRAIN_ROOT`.

Generated runtime data is written to the local `output/` directory and is
ignored by Git.
