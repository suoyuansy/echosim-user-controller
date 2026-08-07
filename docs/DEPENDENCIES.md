# External Dependencies

This repository intentionally does not distribute the EchoSim SDK, OpenCV
runtime libraries, or terrain data. These files are vendor/runtime assets and
may be subject to separate redistribution terms.

## Required dependencies

- Visual Studio 2022 C++ x64 toolchain
- CMake and Ninja
- EchoSim SDK with its headers and MSVC libraries/DLLs
- OpenCV 4.12.0 with its headers, MSVC libraries, and runtime DLL
- Moon terrain data used by the task configuration

## Local dependency layout

The existing CMake project can use a local dependency bundle with this layout:

```text
<echosim-root>/
  include/
  libs/
  libsdebug/
  bin/
  ...
<opencv-root>/
  include/
  lib/
  bin/
```

The source tree currently uses `third_party/echosim` and
`third_party/opencv` when those directories exist locally. Because
`third_party/` is ignored, a fresh clone should provide the paths explicitly.

## Configure a fresh clone

Run these commands from an x64 Native Tools Command Prompt for VS 2022:

```bat
cd /d C:\path\to\echosim-user-controller

cmake -S . -B out\build\x64-Debug -G Ninja ^
  -DCMAKE_BUILD_TYPE=Debug ^
  -DECHOSIM_DEPENDENCY_ROOT=C:\path\to\echosim ^
  -DOPENCV_ROOT=C:\path\to\opencv
cmake --build out\build\x64-Debug --parallel

cmake -S . -B out\build\x64-Release -G Ninja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DECHOSIM_DEPENDENCY_ROOT=C:\path\to\echosim ^
  -DOPENCV_ROOT=C:\path\to\opencv
cmake --build out\build\x64-Release --parallel
```

The executable is generated under
`out\build\x64-<configuration>\bin\UserControllerTest.exe`.

## Terrain data

Set one of these environment variables when terrain data is outside the
repository:

```bat
set USER_CONTROLLER_TERRAIN_ROOT=C:\path\to\Moon2
```

The program also supports `ECHOSIM_TERRAIN_ROOT`. The first configured path
takes precedence over project-relative fallback paths.
