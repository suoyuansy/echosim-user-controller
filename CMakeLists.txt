cmake_minimum_required(VERSION 3.16)

project(UserControllerTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Prefer the dependency bundle copied with this CMake project. The cache
# variables can still be overridden when the SDK or OpenCV live elsewhere.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/echosim/include")
    set(_default_dependency_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/echosim")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../Cplusplus/include")
    set(_default_dependency_root "${CMAKE_CURRENT_SOURCE_DIR}/../Cplusplus")
else()
    set(_default_dependency_root "${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()
set(ECHOSIM_DEPENDENCY_ROOT "${_default_dependency_root}" CACHE PATH
    "EchoSim dependency root")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opencv/include")
    set(_default_opencv_root "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opencv")
else()
    set(_default_opencv_root "${ECHOSIM_DEPENDENCY_ROOT}/third_party/opencv")
endif()
set(OPENCV_ROOT "${_default_opencv_root}" CACHE PATH "OpenCV root")

set(ECHOSIM_INCLUDE_DIR "${ECHOSIM_DEPENDENCY_ROOT}/include")
set(ECHOSIM_RELEASE_LIB_DIR "${ECHOSIM_DEPENDENCY_ROOT}/libs")
set(ECHOSIM_DEBUG_LIB_DIR "${ECHOSIM_DEPENDENCY_ROOT}/libsdebug")
set(OPENCV_INCLUDE_DIR "${OPENCV_ROOT}/include")
set(OPENCV_LIB_DIR "${OPENCV_ROOT}/lib")
set(OPENCV_BIN_DIR "${OPENCV_ROOT}/bin")

set(USER_CONTROLLER_SOURCES
    src/UserControllerTest.cpp
    src/TerrainCostmap.cpp
    src/GlobalPlanner.cpp
    src/PathTracker.cpp
    src/TrajectoryVisualizer.cpp
    src/TaskConfig.cpp
    src/PlanningPipeline.cpp
    src/EchoSimRuntime.cpp
)

add_executable(UserControllerTest ${USER_CONTROLLER_SOURCES})

target_compile_definitions(UserControllerTest PRIVATE
    ECHOSIM_USE_ECHOSIM_SDK
    ECHOSIM_USE_OPENCV
    NOMINMAX
)

target_include_directories(UserControllerTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ECHOSIM_INCLUDE_DIR}"
    "${ECHOSIM_INCLUDE_DIR}/libmsg"
    "${ECHOSIM_INCLUDE_DIR}/terrain"
    "${OPENCV_INCLUDE_DIR}"
)

if(MSVC)
    target_compile_options(UserControllerTest PRIVATE /utf-8)
else()
    message(WARNING
        "The bundled EchoSim/OpenCV libraries are MSVC libraries. "
        "Configure from the VS 2022 x64 Native Tools environment for a runnable build.")
endif()

target_link_directories(UserControllerTest PRIVATE
    "$<$<CONFIG:Debug>:${ECHOSIM_DEBUG_LIB_DIR}>"
    "$<$<NOT:$<CONFIG:Debug>>:${ECHOSIM_RELEASE_LIB_DIR}>"
    "${OPENCV_LIB_DIR}"
)

target_link_libraries(UserControllerTest PRIVATE
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
    add_custom_command(TARGET UserControllerTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ECHOSIM_RUNTIME_DIR}/${runtime_dll}"
            "$<TARGET_FILE_DIR:UserControllerTest>"
        COMMENT "Copy EchoSim runtime ${runtime_dll}")
endforeach()

add_custom_command(TARGET UserControllerTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<IF:$<CONFIG:Debug>,${OPENCV_BIN_DIR}/opencv_world4120d.dll,${OPENCV_BIN_DIR}/opencv_world4120.dll>"
        "$<TARGET_FILE_DIR:UserControllerTest>"
    COMMENT "Copy OpenCV runtime")
