// 文件功能：实现 Pure Pursuit、Stanley、简化 LQR 和纵向速度 P 控制。
#include "PathTracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
constexpr double kPi = 3.14159265358979323846;

// 将航向角归一化到 [-pi, pi]。
double normalize_angle_(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

// 计算车辆平面速度大小。
double planar_speed_(const VehicleState2D& state)
{
    return std::hypot(state.vx, state.vy);
}

// 计算带符号横向误差。
double signed_cross_track_error_(const PathPoint& target,
                             const VehicleState2D& state)
{
    return std::sin(target.yaw) * (state.x - target.x)
        - std::cos(target.yaw) * (state.y - target.y);
}
} // namespace

// 保存跟踪器配置。
PathTracker::PathTracker(TrackingConfig config)
    : config_(std::move(config))
{
}

// 依据配置统一选择跟踪算法。
TrackingCommand PathTracker::calculate(const Path& path,
                                       const VehicleState2D& state) const
{
    switch (config_.method)
    {
    case TrackerMethod::PurePursuit:
        return calculatePurePursuit(path, state);
    case TrackerMethod::Lqr:
        return calculateLqr(path, state);
    case TrackerMethod::Stanley:
    default:
        return calculateStanley(path, state);
    }
}

// 生成所有横向控制器共用的路径状态。
TrackingCommand PathTracker::make_base_command_(const Path& path,
                                              const VehicleState2D& state) const
{
    TrackingCommand command;
    if (path.empty())
    {
        command.reached_goal = true;
        return command;
    }
    const std::size_t nearest = find_nearest_index_(path, state.x, state.y);
    command.remaining_path_distance_m = remaining_distance_(path, nearest);
    const double distance_to_goal = std::hypot(
        path.back().x - state.x, path.back().y - state.y);
    const double yaw_error = normalize_angle_(path.back().yaw - state.yaw);
    command.reached_goal = distance_to_goal <= config_.goal_position_tolerance_m
        && std::abs(yaw_error) <= config_.goal_yaw_tolerance_rad;
    if (!command.reached_goal)
    {
        const double speed_limit = LongitudinalController::distanceSpeedLimit(
            command.remaining_path_distance_m, config_.base_speed_mps,
            config_.max_deceleration_mps2);
        command.target_speed_mps = std::clamp(
            std::min(config_.base_speed_mps, speed_limit), 0.0,
            std::max(0.0, config_.max_speed_mps));
    }
    return command;
}

// 搜索距离车辆最近的路径点。
std::size_t PathTracker::find_nearest_index_(const Path& path, double x,
                                           double y) const
{
    if (path.empty())
        return 0;
    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const double distance = std::hypot(path[index].x - x,
                                           path[index].y - y);
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest = index;
        }
    }
    return nearest;
}

// 返回公开的最近参考点序号。
std::size_t PathTracker::findNearestPathPoint(const Path& path,
                                              const VehicleState2D& state) const
{
    return find_nearest_index_(path, state.x, state.y);
}

// 计算最近路径点到终点的折线长度。
double PathTracker::remaining_distance_(const Path& path,
                                      std::size_t index) const
{
    if (path.empty() || index >= path.size())
        return 0.0;
    double distance = 0.0;
    for (std::size_t i = index; i + 1 < path.size(); ++i)
    {
        distance += std::hypot(path[i + 1].x - path[i].x,
                               path[i + 1].y - path[i].y);
    }
    return distance;
}

// 限制前轮角到车辆允许范围。
double PathTracker::clamp_steer_(double angle_rad) const
{
    return std::clamp(angle_rad,
                      -config_.geometry.max_front_wheel_angle_rad,
                      config_.geometry.max_front_wheel_angle_rad);
}

// 使用前视点和轴距计算纯跟踪转角。
TrackingCommand PathTracker::calculatePurePursuit(
    const Path& path, const VehicleState2D& state) const
{
    TrackingCommand command = make_base_command_(path, state);
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = find_nearest_index_(path, state.x, state.y);
    const double lookahead = std::max(
        0.5, config_.pure_pursuit_lookahead_m
            + config_.pure_pursuit_lookahead_gain * planar_speed_(state));
    std::size_t target_index = nearest;
    double accumulated = 0.0;
    while (target_index + 1 < path.size() && accumulated < lookahead)
    {
        accumulated += std::hypot(path[target_index + 1].x - path[target_index].x,
                                  path[target_index + 1].y - path[target_index].y);
        ++target_index;
    }
    const double dx = path[target_index].x - state.x;
    const double dy = path[target_index].y - state.y;
    const double target_distance = std::max(std::hypot(dx, dy), 0.5);
    const double alpha = normalize_angle_(std::atan2(dy, dx) - state.yaw);
    command.front_wheel_angle_rad = clamp_steer_(std::atan2(
        2.0 * config_.geometry.wheelbase_m * std::sin(alpha), target_distance));
    return command;
}

// 使用横向误差和航向误差计算 Stanley 转角。
TrackingCommand PathTracker::calculateStanley(
    const Path& path, const VehicleState2D& state) const
{
    TrackingCommand command = make_base_command_(path, state);
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = find_nearest_index_(path, state.x, state.y);
    const PathPoint& target = path[nearest];
    const double heading_error = normalize_angle_(target.yaw - state.yaw);
    const double cross_track_error = signed_cross_track_error_(target, state);
    const double speed = std::max(config_.stanley_min_speed_mps, planar_speed_(state));
    command.front_wheel_angle_rad = clamp_steer_(heading_error + std::atan2(
        config_.stanley_gain * cross_track_error, speed));
    return command;
}

// 使用 [横向误差、航向误差、速度误差] 计算简化 LQR 输出。
TrackingCommand PathTracker::calculateLqr(
    const Path& path, const VehicleState2D& state) const
{
    TrackingCommand command = make_base_command_(path, state);
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = find_nearest_index_(path, state.x, state.y);
    const PathPoint& target = path[nearest];
    const double lateral_error = signed_cross_track_error_(target, state);
    const double heading_error = normalize_angle_(target.yaw - state.yaw);
    const double actual_speed = planar_speed_(state);
    const double speed_error = actual_speed - command.target_speed_mps;
    const double steer_weight = std::max(config_.lqr_steer_weight, 1e-6);
    const double lateral_gain = config_.lqr_lateral_weight / steer_weight
        / std::max(actual_speed, 1.0);
    const double heading_gain = config_.lqr_heading_weight / steer_weight;
    command.front_wheel_angle_rad = clamp_steer_(
        lateral_gain * lateral_error + heading_gain * heading_error);
    const double input_weight = std::max(config_.lqr_speed_input_weight, 1e-6);
    command.target_speed_mps = std::clamp(
        command.target_speed_mps - config_.lqr_speed_weight / input_weight * speed_error,
        0.0, std::max(0.0, config_.max_speed_mps));
    return command;
}

// 计算带上下限的纵向速度 P 控制加速度。
double LongitudinalController::speedP(double target_speed_mps,
                                      double actual_speed_mps,
                                      double kp,
                                      double min_acceleration_mps2,
                                      double max_acceleration_mps2)
{
    const double lower = std::min(min_acceleration_mps2, max_acceleration_mps2);
    const double upper = std::max(min_acceleration_mps2, max_acceleration_mps2);
    return std::clamp(kp * (target_speed_mps - actual_speed_mps), lower, upper);
}

// 根据剩余距离和最大减速度计算安全速度上限。
double LongitudinalController::distanceSpeedLimit(
    double remaining_distance_m, double base_speed_mps,
    double max_deceleration_mps2)
{
    if (remaining_distance_m <= 0.0 || max_deceleration_mps2 <= 0.0)
        return 0.0;
    return std::min(std::max(0.0, base_speed_mps), std::sqrt(
        2.0 * max_deceleration_mps2 * remaining_distance_m));
}
