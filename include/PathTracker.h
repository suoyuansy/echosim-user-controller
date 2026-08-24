#pragma once

#include "PathTypes.h"

#include <cstddef>

// 文件功能：定义车辆状态、跟踪配置、三种横向跟踪算法和纵向 P 控制接口。
struct VehicleState2D
{
    double x = 0.0; // 车辆世界坐标 X，单位为米。
    double y = 0.0; // 车辆世界坐标 Y，单位为米。
    double yaw = 0.0; // 车辆航向角，单位为弧度。
    double vx = 0.0; // 车辆 X 方向速度，单位为米每秒。
    double vy = 0.0; // 车辆 Y 方向速度，单位为米每秒。
};

struct VehicleGeometry
{
    double wheelbase_m = 2.76; // 车辆轴距，单位为米。
    double front_axle_offset_m = 1.41; // 质心到前轴距离，当前版本保留未参与计算。
    double rear_axle_offset_m = 1.35; // 质心到后轴距离，当前版本保留未参与计算。
    double max_front_wheel_angle_rad = 23.0 * 3.14159265358979323846 / 180.0; // 最大前轮角。
};

enum class TrackerMethod
{
    PurePursuit = 0, // 纯跟踪法。
    Stanley = 1, // Stanley 法。
    Lqr = 2 // 简化 LQR 法。
};

struct TrackingConfig
{
    TrackerMethod method = TrackerMethod::PurePursuit; // 当前使用的跟踪算法。
    VehicleGeometry geometry; // 车辆几何参数。
    double pure_pursuit_lookahead_m = 4.0; // 纯跟踪基础前视距离。
    double pure_pursuit_lookahead_gain = 0.5; // 纯跟踪速度前视增益。
    double stanley_gain = 1.0; // Stanley 横向误差增益。
    double stanley_min_speed_mps = 0.5; // Stanley 最小计算速度。
    double base_speed_mps = 4*3.6; // Pure Pursuit/Stanley 的基础 SDK 速度上限。
    double max_speed_mps = 4*3.6; // Pure Pursuit/Stanley 的最大速度上限。
    double lqr_speed_weight = 1.0; // 简化 LQR 速度误差权重。
    double lqr_lateral_weight = 3.0; // 简化 LQR 横向误差权重。
    double lqr_heading_weight = 2.0; // 简化 LQR 航向误差权重。
    double lqr_steer_weight = 1.0; // 简化 LQR 转向输入权重。
    double lqr_speed_input_weight = 1.0; // 简化 LQR 速度输入权重。
    double lqr_longitudinal_kp = 0.8; // LQR 纵向速度 P 控制增益。
    double max_acceleration_mps2 = 1.0; // 最大加速度。
    double max_deceleration_mps2 = 1.5; // 最大减速度绝对值。
    double goal_position_tolerance_m = 1.0; // 终点位置容差。
    double goal_yaw_tolerance_rad = 10.0 * 3.14159265358979323846 / 180.0; // 终点航向容差。
};

struct TrackingCommand
{
    double front_wheel_angle_rad = 0.0; // 目标前轮角，单位为弧度。
    double target_speed_mps = 0.0; // LQR 使用的目标速度，其他算法不修改 SDK 速度目标。
    double remaining_path_distance_m = 0.0; // 当前参考点到终点剩余折线距离。
    std::size_t nearest_path_index = 0; // 用于计算进度的最近路径点。
    std::size_t target_path_index = 0; // 用于控制的路径目标点。
    bool reached_goal = false; // 是否满足终点位置和航向角要求。
};

class PathTracker
{
public:
    explicit PathTracker(TrackingConfig config = TrackingConfig());

    // 按配置选择 Pure Pursuit、Stanley 或简化 LQR。
    TrackingCommand calculate(const Path& path,
                              const VehicleState2D& state,
                              std::size_t progress_index = 0) const;

    // 计算纯跟踪前轮角。
    TrackingCommand calculatePurePursuit(const Path& path,
                                         const VehicleState2D& state,
                                         std::size_t progress_index = 0) const;

    // 计算 Stanley 前轮角。
    TrackingCommand calculateStanley(const Path& path,
                                     const VehicleState2D& state,
                                     std::size_t progress_index = 0) const;

    // 计算简化 LQR 前轮角和目标速度修正。
    TrackingCommand calculateLqr(const Path& path,
                                 const VehicleState2D& state,
                                 std::size_t progress_index = 0) const;

    // 返回车辆对应的最近参考路径点序号。
    std::size_t findNearestPathPoint(const Path& path,
                                     const VehicleState2D& state,
                                     std::size_t progress_index = 0) const;

private:
    TrackingCommand make_base_command_(const Path& path,
                                       const VehicleState2D& state,
                                       std::size_t progress_index) const;
    std::size_t find_nearest_index_(const Path& path, double x, double y,
                                    std::size_t progress_index) const;
    double remaining_distance_(const Path& path, std::size_t index) const;
    double clamp_steer_(double angle_rad) const;

    TrackingConfig config_;
};

class LongitudinalController
{
public:
    // 计算带上下限的速度误差 P 控制加速度。
    static double speedP(double target_speed_mps,
                         double actual_speed_mps,
                         double kp,
                         double min_acceleration_mps2,
                         double max_acceleration_mps2);

    // 根据剩余路径距离计算满足减速约束的速度上限。
    static double distanceSpeedLimit(double remaining_distance_m,
                                     double base_speed_mps,
                                     double max_deceleration_mps2);
};
