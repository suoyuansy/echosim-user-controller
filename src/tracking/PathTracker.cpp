// 文件功能：实现 Pure Pursuit、Stanley、简化 LQR 和纵向速度 P 控制。
#include "tracking/PathTracker.h"

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

// 查找车辆前方预览距离内的第一个明显折点，并将目标点推到折点之后。
// 这样 PP 会在进入拐点前就看到弯后的路径，而不是只看到拐点本身。
std::size_t corner_preview_target_(const Path& path,
                                   std::size_t nearest,
                                   double preview_distance_m,
                                   double heading_threshold_rad,
                                   double after_distance_m)
{
    if (path.size() < 3 || nearest >= path.size() - 2)
        return nearest;

    const double preview_distance = std::max(0.0, preview_distance_m);
    const double threshold = std::clamp(
        std::abs(heading_threshold_rad), 0.0, kPi);
    double distance_to_vertex = 0.0;
    for (std::size_t segment = nearest; segment + 2 < path.size(); ++segment)
    {
        const double first_dx = path[segment + 1].x - path[segment].x;
        const double first_dy = path[segment + 1].y - path[segment].y;
        const double second_dx = path[segment + 2].x - path[segment + 1].x;
        const double second_dy = path[segment + 2].y - path[segment + 1].y;
        const double first_length = std::hypot(first_dx, first_dy);
        const double second_length = std::hypot(second_dx, second_dy);
        if (first_length > 1e-6 && second_length > 1e-6)
        {
            const double first_heading = std::atan2(first_dy, first_dx);
            const double second_heading = std::atan2(second_dy, second_dx);
            const double heading_change = std::abs(normalize_angle_(
                second_heading - first_heading));
            if (distance_to_vertex <= preview_distance
                && heading_change >= threshold)
            {
                std::size_t target = segment + 1;
                double distance_after_corner = 0.0;
                while (target + 1 < path.size()
                       && distance_after_corner
                              < std::max(0.0, after_distance_m))
                {
                    distance_after_corner += std::hypot(
                        path[target + 1].x - path[target].x,
                        path[target + 1].y - path[target].y);
                    ++target;
                }
                return target;
            }
        }
        distance_to_vertex += first_length;
        if (distance_to_vertex > preview_distance)
            break;
    }
    return nearest;
}
} // namespace

PathSafetyState classifyPathSafety(double path_distance_m,
                                   const TrackingConfig& config)
{
    if (!std::isfinite(path_distance_m))
        return PathSafetyState::Stop;
    const double warning = std::max(0.0, config.off_path_warning_distance_m);
    const double recovery = std::max(warning,
                                     config.off_path_recovery_distance_m);
    const double stop = std::max(recovery, config.off_path_stop_distance_m);
    if (path_distance_m <= warning)
        return PathSafetyState::Normal;
    // 3~6 m 为低速恢复区；不再使用一个实际不会被稳定区分的
    // Warning 中间态，避免调用方把同一距离误当作普通跟踪。
    if (path_distance_m <= recovery)
        return PathSafetyState::Recovery;
    if (path_distance_m <= stop)
        return PathSafetyState::SevereRecovery;
    return PathSafetyState::Stop;
}

// 保存跟踪器配置。
PathTracker::PathTracker(TrackingConfig config)
    : config_(std::move(config))
{
}

// 依据配置统一选择跟踪算法。
TrackingCommand PathTracker::calculate(const Path& path,
                                       const VehicleState2D& state,
                                       std::size_t progress_index,
                                       bool reverse) const
{
    switch (config_.method)
    {
    case TrackerMethod::PurePursuit:
        return calculatePurePursuit(path, state, progress_index, reverse);
    case TrackerMethod::Lqr:
        return calculateLqr(path, state, progress_index, reverse);
    case TrackerMethod::Stanley:
    default:
        return calculateStanley(path, state, progress_index, reverse);
    }
}

// 生成所有横向控制器共用的路径状态。
TrackingCommand PathTracker::make_base_command_(const Path& path,
                                               const VehicleState2D& state,
                                               std::size_t progress_index,
                                               bool reverse) const
{
    TrackingCommand command;
    if (path.empty())
    {
        command.reached_goal = true;
        return command;
    }
    const std::size_t nearest = find_nearest_index_(
        path, state.x, state.y, progress_index);
    command.nearest_path_index = nearest;
    command.reverse = reverse;
    command.target_path_index = nearest;
    command.remaining_path_distance_m = remaining_distance_(path, nearest);
    // 路径末点可能只是任务终点的栅格吸附点。运行时必须使用任务配置中的
    // 精确终点坐标判定到达，不能在这里因 path.back() 提前停止横向控制。
    command.reached_goal = false;
    // 纵向不在 PP 内部做距离-速度控制；这里只提供巡航参考速度，
    // 后续由运行时的参考速度器和加速度 PID 独立调节。
    // 即使最终路径点已进入容差，也不能在这里把速度清零：栅格吸附后的
    // 路径终点可能与任务终点存在偏差，是否停车必须由运行时的任务终点
    // 坐标判定。
    command.target_speed_mps = std::clamp(
        // DRIVE/REVERSE 只表示运动方向；两种挡位使用同一巡航目标速度，
        // 这样倒车不会因为单独的旧参数而被限制在较低速度。弯道、脱轨
        // 恢复和终点接近阶段仍由调用方沿用同一套后续限速/减速逻辑。
        std::abs(config_.base_speed_mps),
        0.0, std::max(0.0, config_.max_speed_mps));
    return command;
}

// 搜索距离车辆最近的路径点。
std::size_t PathTracker::find_nearest_index_(const Path& path, double x,
                                            double y,
                                            std::size_t progress_index) const
{
    if (path.empty())
        return 0;
    const std::size_t first_index = std::min(progress_index, path.size() - 1);
    std::size_t nearest = first_index;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = first_index; index < path.size(); ++index)
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
                                              const VehicleState2D& state,
                                              std::size_t progress_index) const
{
    return find_nearest_index_(path, state.x, state.y, progress_index);
}

double PathTracker::distanceToPath(const Path& path,
                                   const VehicleState2D& state,
                                   std::size_t begin_index) const
{
    if (path.empty())
        return std::numeric_limits<double>::infinity();
    begin_index = std::min(begin_index, path.size() - 1);
    double minimum_distance = std::hypot(path[begin_index].x - state.x,
                                        path[begin_index].y - state.y);
    for (std::size_t index = begin_index; index + 1 < path.size(); ++index)
    {
        const double ax = path[index].x;
        const double ay = path[index].y;
        const double bx = path[index + 1].x;
        const double by = path[index + 1].y;
        const double dx = bx - ax;
        const double dy = by - ay;
        const double length_squared = dx * dx + dy * dy;
        double projection = 0.0;
        if (length_squared > 1e-12)
            projection = std::clamp(((state.x - ax) * dx
                                     + (state.y - ay) * dy)
                                    / length_squared, 0.0, 1.0);
        const double nearest_x = ax + projection * dx;
        const double nearest_y = ay + projection * dy;
        minimum_distance = std::min(minimum_distance,
                                    std::hypot(state.x - nearest_x,
                                               state.y - nearest_y));
    }
    return minimum_distance;
}

TrackingCommand PathTracker::calculateToTarget(const VehicleState2D& state,
                                               const Pose2D& target,
                                               bool reverse) const
{
    TrackingCommand command;
    command.reverse = reverse;
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double target_distance = std::max(std::hypot(dx, dy), 0.5);
    const double effective_yaw = normalize_angle_(
        state.yaw + (reverse ? kPi : 0.0));
    const double alpha = normalize_angle_(std::atan2(dy, dx) - effective_yaw);
    const double forward_angle = std::atan2(
        2.0 * config_.geometry.wheelbase_m * std::sin(alpha),
        target_distance);
    command.front_wheel_angle_rad = clamp_steer_(reverse
        ? -forward_angle : forward_angle);
    return command;
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
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index, bool reverse) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index, reverse);
    if (path.empty())
        return command;
    const std::size_t nearest = command.nearest_path_index;
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
    const std::size_t corner_target = corner_preview_target_(
        path, nearest, config_.pure_pursuit_corner_preview_distance_m,
        config_.pure_pursuit_corner_heading_threshold_rad,
        config_.pure_pursuit_corner_target_after_distance_m);
    const bool corner_ahead = corner_target > nearest
        && corner_target > target_index;
    if (corner_ahead)
        target_index = corner_target;
    command.target_path_index = target_index;
    if (corner_ahead)
        command.target_speed_mps = std::min(
            std::abs(command.target_speed_mps),
            std::abs(config_.curve_speed_mps));
    const double dx = path[target_index].x - state.x;
    const double dy = path[target_index].y - state.y;
    const double target_distance = std::max(std::hypot(dx, dy), 0.5);
    const double effective_yaw = normalize_angle_(
        state.yaw + (reverse ? kPi : 0.0));
    const double alpha = normalize_angle_(std::atan2(dy, dx) - effective_yaw);
    const double forward_angle = std::atan2(
        2.0 * config_.geometry.wheelbase_m * std::sin(alpha), target_distance);
    // 倒车时纵向速度为负，车辆运动方向与车头方向相反；同一条轨迹的
    // 前轮转角因此需要取反，才能产生与前进时相同的路径曲率。
    command.front_wheel_angle_rad = clamp_steer_(reverse
        ? -forward_angle : forward_angle);
    return command;
}

// 使用横向误差和航向误差计算 Stanley 转角。
TrackingCommand PathTracker::calculateStanley(
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index, bool reverse) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index, reverse);
    if (path.empty())
        return command;
    const std::size_t nearest = command.nearest_path_index;
    const PathPoint& target = path[nearest];
    const double effective_yaw = normalize_angle_(state.yaw + (reverse ? kPi : 0.0));
    const double heading_error = normalize_angle_(target.yaw - effective_yaw);
    const double cross_track_error = signed_cross_track_error_(target, state);
    const double speed = std::max(config_.stanley_min_speed_mps, planar_speed_(state));
    command.front_wheel_angle_rad = clamp_steer_(heading_error + std::atan2(
        config_.stanley_gain * cross_track_error, speed));
    return command;
}

// 使用 [横向误差、航向误差、速度误差] 计算简化 LQR 输出。
TrackingCommand PathTracker::calculateLqr(
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index, bool reverse) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index, reverse);
    if (path.empty())
        return command;
    const std::size_t nearest = command.nearest_path_index;
    const PathPoint& target = path[nearest];
    const double lateral_error = signed_cross_track_error_(target, state);
    const double effective_yaw = normalize_angle_(state.yaw + (reverse ? kPi : 0.0));
    const double heading_error = normalize_angle_(target.yaw - effective_yaw);
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
void LongitudinalController::reset()
{
    integral_ = 0.0;
    previous_error_ = 0.0;
    initialized_ = false;
}

double LongitudinalController::update(double target_speed_mps,
                                      double actual_speed_mps,
                                      double dt_sec,
                                      const TrackingConfig& config)
{
    const double error = target_speed_mps - actual_speed_mps;
    const double lower = -std::abs(config.max_deceleration_mps2);
    const double upper = std::abs(config.max_acceleration_mps2);

    // 重复的仿真时间戳只允许重新计算当前控制量，不能推进积分，也不能
    // 用 1 ms 人造步长放大微分项。下一个有效时间戳到来后再正常更新 PID。
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0)
    {
        const double output = config.longitudinal_accel_kp * error
            + config.longitudinal_accel_ki * integral_;
        return std::clamp(output, lower, upper);
    }

    const double dt = std::clamp(dt_sec, 1e-3, 1.0);
    const double candidate_integral = std::clamp(
        integral_ + error * dt,
        -std::abs(config.pid_integral_limit),
        std::abs(config.pid_integral_limit));
    const double derivative = initialized_ ? (error - previous_error_) / dt : 0.0;
    const double unsaturated = config.longitudinal_accel_kp * error
        + config.longitudinal_accel_ki * candidate_integral
        + config.longitudinal_accel_kd * derivative;
    const double output = std::clamp(unsaturated, lower, upper);
    if (output == unsaturated || (output >= upper && error < 0.0)
        || (output <= lower && error > 0.0))
        integral_ = candidate_integral;
    previous_error_ = error;
    initialized_ = true;
    return output;
}
