cmake_minimum_required(VERSION 3.16)

project(UserController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Use the dependency bundle copied with this CMake project. The cache
# variables can still be overridden explicitly when dependencies live elsewhere.
set(_default_dependency_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/echosim")
set(ECHOSIM_DEPENDENCY_ROOT "${_default_dependency_root}" CACHE PATH
    "EchoSim dependency root")

set(_default_opencv_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opencv")
set(OPENCV_ROOT "${_default_opencv_root}" CACHE PATH "OpenCV root")

set(ECHOSIM_INCLUDE_DIR "${ECHOSIM_DEPENDENCY_ROOT}/include")
set(ECHOSIM_RELEASE_LIB_DIR "${ECHOSIM_DEPENDENCY_ROOT}/libs")
set(ECHOSIM_DEBUG_LIB_DIR "${ECHOSIM_DEPENDENCY_ROOT}/libsdebug")
set(OPENCV_INCLUDE_DIR "${OPENCV_ROOT}/include")
set(OPENCV_LIB_DIR "${OPENCV_ROOT}/lib")
set(OPENCV_BIN_DIR "${OPENCV_ROOT}/bin")

set(USER_CONTROLLER_SOURCES
    src/app/UserController.cpp
    src/planning/TerrainCostmap.cpp
    src/planning/GlobalPlanner.cpp
    src/optimization/PathOptimizer.cpp
    src/tracking/PathTracker.cpp
    src/app/TrajectoryVisualizer.cpp
    src/app/TaskConfig.cpp
    src/app/PlanningPipeline.cpp
    src/tracking/EchoSimRuntime.cpp
)

add_executable(UserController ${USER_CONTROLLER_SOURCES})

include(CTest)
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/PathTrackerSelfTest.cpp")
    add_executable(PathTrackerSelfTest
        tests/PathTrackerSelfTest.cpp
        src/tracking/PathTracker.cpp
    )
    target_include_directories(PathTrackerSelfTest PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
    if(MSVC)
        target_compile_options(PathTrackerSelfTest PRIVATE /utf-8)
    endif()
    add_test(NAME PathTrackerSelfTest COMMAND PathTrackerSelfTest)
elseif(BUILD_TESTING)
    message(WARNING
        "BUILD_TESTING is enabled, but tests/PathTrackerSelfTest.cpp was not found; "
        "skipping PathTrackerSelfTest.")
endif()

target_compile_definitions(UserController PRIVATE
    ECHOSIM_USE_ECHOSIM_SDK
    ECHOSIM_USE_OPENCV
    NOMINMAX
)

target_include_directories(UserController PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ECHOSIM_INCLUDE_DIR}"
    "${ECHOSIM_INCLUDE_DIR}/libmsg"
    "${ECHOSIM_INCLUDE_DIR}/terrain"
    "${OPENCV_INCLUDE_DIR}"
)

if(MSVC)
    target_compile_options(UserController PRIVATE /utf-8)
else()
    message(WARNING
        "The bundled EchoSim/OpenCV libraries are MSVC libraries. "
        "Configure from the VS 2022 x64 Native Tools environment for a runnable build.")
endif()

target_link_directories(UserController PRIVATE
    "$<$<CONFIG:Debug>:${ECHOSIM_DEBUG_LIB_DIR}>"
    "$<$<NOT:$<CONFIG:Debug>>:${ECHOSIM_RELEASE_LIB_DIR}>"
    "${OPENCV_LIB_DIR}"
)

target_link_libraries(UserController PRIVATE
    TerrainServiceAPI
    libSimMsg
    "$<$<CONFIG:Debug>:opencv_world4120d>"
    "$<$<NOT:$<CONFIG:Debug>>:opencv_world4120>"
)

if(EXISTS "${ECHOSIM_DEPENDENCY_ROOT}/bin")
    set(ECHOSIM_RUNTIME_DIR "${ECHOSIM_DEPENDENCY_ROOT}/bin")
else()
    set(ECHOSIM_RUNTIME_DIR "${ECHOSIM_DEPENDENCY_ROOT}/x64/Release")
endif()

set(ECHOSIM_DLLS
    esync.dll
    libSimMsg.dll
    osg162-osg.dll
    osg162-osgDB.dll
    osg162-osgUtil.dll
    ot21-OpenThreads.dll
    python38.dll
    TerrainServiceAPI.dll
)

foreach(runtime_dll IN LISTS ECHOSIM_DLLS)
    add_custom_command(TARGET UserController POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ECHOSIM_RUNTIME_DIR}/${runtime_dll}"
            "$<TARGET_FILE_DIR:UserController>"
        COMMENT "Copy EchoSim runtime ${runtime_dll}")
endforeach()

add_custom_command(TARGET UserController POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<IF:$<CONFIG:Debug>,${OPENCV_BIN_DIR}/opencv_world4120d.dll,${OPENCV_BIN_DIR}/opencv_world4120.dll>"
        "$<TARGET_FILE_DIR:UserController>"
    COMMENT "Copy OpenCV runtime")
