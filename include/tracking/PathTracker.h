#pragma once

#include "common/PathTypes.h"
#include "tracking/ReversePlanner.h"

#include <cstddef>

// 文件功能：定义 PP 横向跟踪和独立纵向加速度 PID 接口。
struct VehicleState2D
{
    double x = 0.0; // 车辆世界坐标 X，单位为米。
    double y = 0.0; // 车辆世界坐标 Y，单位为米。
    double yaw = 0.0; // 车辆航向角，单位为弧度。
    double vx = 0.0; // 车辆 X 方向速度，单位为米每秒。
    double vy = 0.0; // 车辆 Y 方向速度，单位为米每秒。
    double ax = 0.0; // 车辆 X 方向实际加速度，单位为米每二次方秒。
    double ay = 0.0; // 车辆 Y 方向实际加速度，单位为米每二次方秒。
    double az = 0.0; // 车辆 Z 方向实际加速度，单位为米每二次方秒。
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
    double pure_pursuit_lookahead_m = 6.0; // 纯跟踪基础前视距离。
    double pure_pursuit_lookahead_gain = 0.75; // 纯跟踪速度前视增益。
    // 前方急弯预览：在拐点进入该距离前就把 PP 目标放到弯后，
    // 让转向和降速提前发生，避免车辆高速冲到折点才开始转向。
    double pure_pursuit_corner_preview_distance_m = 8.0;
    double pure_pursuit_corner_heading_threshold_rad =
        60.0 * 3.14159265358979323846 / 180.0;
    double pure_pursuit_corner_target_after_distance_m = 3.0;
    double stanley_gain = 1.0; // Stanley 横向误差增益。
    double stanley_min_speed_mps = 0.5; // Stanley 最小计算速度。
    double base_speed_mps = 3.5; // 直线参考速度，单位为米每秒。
    double reverse_speed_mps = 3.5; // 兼容旧配置字段；倒车巡航实际与 base_speed_mps 一致。
    double max_speed_mps = 3.5; // 参考速度上限，单位为米每秒。
    ReverseDetectionConfig reverse_detection;
    double gear_change_stop_duration_sec = 0.3;
    double curve_speed_mps = 1.2; // 大转角时的参考速度绝对值，单位为米每秒。
    double lqr_speed_weight = 1.0; // 简化 LQR 速度误差权重。
    double lqr_lateral_weight = 3.0; // 简化 LQR 横向误差权重。
    double lqr_heading_weight = 2.0; // 简化 LQR 航向误差权重。
    double lqr_steer_weight = 1.0; // 简化 LQR 转向输入权重。
    double lqr_speed_input_weight = 1.0; // 简化 LQR 速度输入权重。
    double longitudinal_accel_kp = 1.5; // 速度误差 P 增益。
    double longitudinal_accel_ki = 0.06; // 速度误差 I 增益。
    double longitudinal_accel_kd = 0.04; // 速度误差 D 增益。
    double pid_integral_limit = 2.0; // 积分项限幅。
    double max_acceleration_mps2 = 4; // 最大驱动加速度。
    double max_deceleration_mps2 = 3; // 最大减速度绝对值。
    double goal_position_tolerance_m = 1.0; // 终点位置容差。
    double goal_capture_exit_radius_m = 1.25; // 终点回收退出半径，必须大于进入半径。
    double waypoint_capture_radius_m = 2.0;
    double waypoint_slowdown_distance_m = 18.0;
    double goal_slowdown_distance_m = 25.0;
    double goal_brake_safety_distance_m = 2.0;
    double goal_recovery_speed_mps = 1.0; // 终点接近和回收速度。
    double stopped_speed_mps = 0.10;
    double waypoint_hold_duration_sec = 3.0;
    double goal_hold_duration_sec = 3.0;
    double goal_yaw_tolerance_rad = 10.0 * 3.14159265358979323846 / 180.0; // 保留兼容配置；终点完成仅按位置判定。
    double waypoint_recovery_speed_mps = 0.5; // 途经点回收参考速度。
    double off_path_warning_distance_m = 3.0; // 开始脱轨告警和限速的路径距离。
    double off_path_recovery_distance_m = 6.0; // 强化低速回收的路径距离。
    double off_path_stop_distance_m = 10.0; // 超过此距离进入严重脱轨恢复等级。
    double off_path_recovery_speed_mps = 1.0; // 脱轨恢复参考速度，单位为米每秒。
    double parking_steering_limit_rad = 10.0 * 3.14159265358979323846 / 180.0;
    double speed_guard_margin_mps = 0.2; // 合速度超过目标速度后的安全余量。
};

enum class PathSafetyState
{
    Normal,
    Warning,
    Recovery,
    SevereRecovery,
    Stop
};

PathSafetyState classifyPathSafety(double path_distance_m,
                                   const TrackingConfig& config);

struct TrackingCommand
{
    double front_wheel_angle_rad = 0.0; // 目标前轮角，单位为弧度。
    double target_speed_mps = 0.0; // LQR 使用的目标速度，其他算法不修改 SDK 速度目标。
    double remaining_path_distance_m = 0.0; // 当前参考点到终点剩余折线距离。
    std::size_t nearest_path_index = 0; // 用于计算进度的最近路径点。
    std::size_t target_path_index = 0; // 用于控制的路径目标点。
    bool reached_goal = false; // 是否进入终点位置容差范围。
    bool reverse = false; // 当前横向/纵向控制是否沿倒车方向。
};

class PathTracker
{
public:
    explicit PathTracker(TrackingConfig config = TrackingConfig());

    // 按配置选择 Pure Pursuit、Stanley 或简化 LQR。
    TrackingCommand calculate(const Path& path,
                              const VehicleState2D& state,
                              std::size_t progress_index = 0,
                              bool reverse = false) const;

    // 计算纯跟踪前轮角。
    TrackingCommand calculatePurePursuit(const Path& path,
                                         const VehicleState2D& state,
                                         std::size_t progress_index = 0,
                                         bool reverse = false) const;

    // 计算 Stanley 前轮角。
    TrackingCommand calculateStanley(const Path& path,
                                     const VehicleState2D& state,
                                     std::size_t progress_index = 0,
                                     bool reverse = false) const;

    // 计算简化 LQR 前轮角和目标速度修正。
    TrackingCommand calculateLqr(const Path& path,
                                 const VehicleState2D& state,
                                 std::size_t progress_index = 0,
                                 bool reverse = false) const;

    // 返回车辆对应的最近参考路径点序号。
    std::size_t findNearestPathPoint(const Path& path,
                                      const VehicleState2D& state,
                                      std::size_t progress_index = 0) const;

    // 返回车辆到全局路径折线的最短平面距离，单位为米。
    double distanceToPath(const Path& path, const VehicleState2D& state,
                          std::size_t begin_index = 0) const;

    // 使用指定目标点计算 PP 前轮角，供途经点/终点回收阶段使用。
    TrackingCommand calculateToTarget(const VehicleState2D& state,
                                      const Pose2D& target,
                                      bool reverse = false) const;


private:
    TrackingCommand make_base_command_(const Path& path,
                                       const VehicleState2D& state,
                                       std::size_t progress_index,
                                       bool reverse) const;
    std::size_t find_nearest_index_(const Path& path, double x, double y,
                                    std::size_t progress_index) const;
    double remaining_distance_(const Path& path, std::size_t index) const;
    double clamp_steer_(double angle_rad) const;

    TrackingConfig config_;
};

class LongitudinalController
{
public:
    void reset();
    double update(double target_speed_mps, double actual_speed_mps,
                  double dt_sec, const TrackingConfig& config);

private:
    double integral_ = 0.0;
    double previous_error_ = 0.0;
    bool initialized_ = false;
};
