#pragma once

#include <cmath>

// EchoSim vehDynamics 的速度字段单位是 km/h；跟踪器内部统一使用 m/s。
constexpr double kEchoSimKphToMps = 1.0 / 3.6;
constexpr double kEchoSimDegToRad = 3.14159265358979323846 / 180.0;

inline double echoSimKphToMps(double speed_kph)
{
    return speed_kph * kEchoSimKphToMps;
}

inline double echoSimDegToRad(double angle_deg)
{
    return angle_deg * kEchoSimDegToRad;
}

// request_front_wheel_angle 的 SDK 边界单位是弧度；内部同样统一使用弧度，
// 因此这里明确禁止再次做度/弧度转换。
inline double toEchoSimFrontWheelAngle(double angle_rad)
{
    return angle_rad;
}
