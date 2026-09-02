#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

// 路径距离是有限值时，车辆仍有明确的最近路径可用于低速恢复；只有
// 距离无效时才无法判断恢复方向，必须进入持续安全停车。
inline bool requiresPersistentPathSafetyStop(double path_distance_m)
{
    return !std::isfinite(path_distance_m);
}

// 终点捕获采用进入/退出两个半径，避免车辆在边界附近因动力学滑移
// 在 ParkingGoal 与 RecoveringGoal 之间来回切换。
inline bool goalCaptureShouldRecover(double distance_m,
                                     double capture_radius_m,
                                     double exit_radius_m)
{
    const double exit_radius = std::max(
        std::max(0.0, capture_radius_m), std::max(0.0, exit_radius_m));
    return !std::isfinite(distance_m) || distance_m > exit_radius;
}

// 终点计时只由位置捕获条件触发，不再要求速度先降到某个阈值。
// 在进入半径与退出半径之间短暂滑出时暂停而不清零，重新进入后继续累计；
// 真正超过退出半径时由状态机离开停车阶段并清零。
inline double accumulateGoalHoldSeconds(double current_hold_sec,
                                        double dt_sec,
                                        bool in_capture_range)
{
    if (!in_capture_range)
        return std::max(0.0, current_hold_sec);
    return std::max(0.0, current_hold_sec)
        + std::max(0.0, std::isfinite(dt_sec) ? dt_sec : 0.0);
}

// 终点回收速度随“超出进入半径的距离”增长，刚越过退出边界时仍然
// 低速回收，避免再次以巡航速度冲入目标圆域。
inline double goalRecoverySpeed(double distance_m,
                                double capture_radius_m,
                                double max_speed_mps)
{
    if (!std::isfinite(distance_m) || !std::isfinite(max_speed_mps))
        return 0.0;
    return std::clamp(distance_m - std::max(0.0, capture_radius_m),
                      0.0, std::abs(max_speed_mps));
}

// 终点位置控制的低速目标：刚刚位于捕获范围外时仍给一个很小的
// 目标速度，避免车辆因目标速度过小而停在范围外；距离增大后再逐步
// 提高回收速度。进入捕获范围后返回 0，由加速度控制器制动并保持。
inline double goalPositionControlSpeed(double distance_m,
                                       double capture_radius_m,
                                       double crawl_speed_mps,
                                       double max_speed_mps)
{
    if (!std::isfinite(distance_m) || !std::isfinite(crawl_speed_mps)
        || !std::isfinite(max_speed_mps))
        return 0.0;
    if (distance_m <= std::max(0.0, capture_radius_m))
        return 0.0;
    return std::clamp(
        std::max(std::abs(crawl_speed_mps),
                 goalRecoverySpeed(distance_m, capture_radius_m,
                                   max_speed_mps)),
        0.0, std::abs(max_speed_mps));
}

// 车辆已经越过终点、目标落在车后方时，允许以小速度倒车回收。
// 距离很近时直接保持零目标速度，避免在目标点两侧来回抖动。
inline double goalReverseCrawlSpeed(double distance_m,
                                    double crawl_speed_mps,
                                    double max_speed_mps)
{
    if (!std::isfinite(distance_m) || !std::isfinite(crawl_speed_mps)
        || !std::isfinite(max_speed_mps)
        || distance_m <= 0.5)
        return 0.0;
    return std::clamp(std::abs(crawl_speed_mps),
                      0.0, std::abs(max_speed_mps));
}

// 终点阶段始终保持可控的低速蠕动。速度与到终点的真实距离成比例，
// 但保留一个很小的下限，避免目标速度为 0 后车辆只能依赖 PARK 滑停。
inline double goalCrawlSpeed(double distance_m,
                             double capture_radius_m,
                             double max_speed_mps)
{
    if (!std::isfinite(distance_m) || !std::isfinite(capture_radius_m)
        || !std::isfinite(max_speed_mps))
        return 0.0;
    const double radius = std::max(std::abs(capture_radius_m), 1e-3);
    const double maximum = std::abs(max_speed_mps);
    if (maximum <= 0.0)
        return 0.0;
    return std::clamp(std::max(0.05, maximum * distance_m / radius),
                      0.05, maximum);
}

// 途经点进入减速区后，必须锁定当前途经点作为控制目标；不能继续让
// 普通全局路径前视点把车辆带入下一个路径阶段。
inline bool shouldApproachWaypoint(double waypoint_distance_m,
                                   double slowdown_distance_m,
                                   bool already_approaching)
{
    if (already_approaching)
        return true;
    return std::isfinite(waypoint_distance_m)
        && std::isfinite(slowdown_distance_m)
        && waypoint_distance_m <= std::max(0.0, slowdown_distance_m);
}

// TARGET_ACC_CONTROL 的统一内部约定：
//   request_acc > 0：沿当前挡位驱动；倒挡下即为倒车驱动。
//   request_acc < 0：减速度/制动，不用于产生倒车驱动力。
struct AccelerationCommand
{
    double request_acc_mps2 = 0.0;
    bool braking = false;
};

// 将车体系纵向速度转换为当前挡位下的“任务方向速度”。
// DRIVE 时车辆向前为正、回溜为负；REVERSE 时倒车方向为正。
// 这样纵向 PID 能识别 DRIVE 下的回溜，而不是只看到速度大小。
inline double longitudinalSpeedForGear(double vehicle_vx_mps,
                                       bool reverse)
{
    return reverse ? -vehicle_vx_mps : vehicle_vx_mps;
}

// 换挡完成只看当前挡位下的纵向速度。车辆转向时可能有明显横向滑移，
// 但横向速度不代表仍在纵向驱动；使用合速度会使换挡状态永久卡在 PARK。
inline bool gearShiftStopped(double signed_longitudinal_speed_mps,
                             double stopped_speed_mps)
{
    return std::isfinite(signed_longitudinal_speed_mps)
        && std::abs(signed_longitudinal_speed_mps)
            <= std::max(0.0, stopped_speed_mps);
}

// 将带符号的车辆纵向速度转换成停车控制加速度。
// request_acc 的符号语义是：负值表示制动，正值表示沿当前挡位驱动；
// 因此无论车辆当前是向前还是倒车，只要仍在运动，停车阶段都必须发负值。
// 速度进入停止死区后不再输出加速度，避免停车控制把车辆从静止状态重新推走。
inline double makeStopAcceleration(double signed_speed_mps,
                                   double stopped_speed_mps,
                                   double max_acceleration_mps2,
                                   double max_deceleration_mps2,
                                   double dt_sec = 0.1)
{
    const double deadband = std::max(0.0, stopped_speed_mps);
    if (!std::isfinite(signed_speed_mps)
        || std::abs(signed_speed_mps) <= deadband)
        return 0.0;
    const double dt = std::max(std::isfinite(dt_sec) ? dt_sec : 0.1, 1e-3);
    (void)max_acceleration_mps2;
    return -std::min(std::abs(max_deceleration_mps2),
                     std::abs(signed_speed_mps) / dt);
}

inline AccelerationCommand makeAccelerationCommand(double acceleration_mps2)
{
    return {acceleration_mps2, acceleration_mps2 < 0.0};
}

// 当车体纵向速度因横向运动而被低估时，合速度保护禁止继续驱动。
inline double applyGroundSpeedSafety(double acceleration_mps2,
                                     double target_speed_mps,
                                     double ground_speed_mps,
                                     double margin_mps,
                                     double max_deceleration_mps2)
{
    if (std::isfinite(target_speed_mps) && std::isfinite(ground_speed_mps)
        && std::isfinite(margin_mps) && std::isfinite(max_deceleration_mps2)
        && ground_speed_mps > std::abs(target_speed_mps)
                                      + std::max(0.0, margin_mps))
        return -std::abs(max_deceleration_mps2);
    return acceleration_mps2;
}

// 途经点锁定阶段用于日志、窗口和控制命令的统一路径索引。
// 即使 PP 的普通计算结果已经前视到后续路径点，锁定的必达点仍应显示为
// 当前目标；超出路径长度时只做边界保护。
inline std::size_t clampTaskTargetPathIndex(std::size_t task_index,
                                            std::size_t path_size)
{
    if (path_size == 0)
        return 0;
    return std::min(task_index, path_size - 1);
}
