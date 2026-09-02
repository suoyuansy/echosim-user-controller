// 实现 EchoSim 状态订阅、三种跟踪控制发布、日志和到点停车。
#include "tracking/EchoSimRuntime.h"

#include "tracking/PathTracker.h"
#include "app/TrajectoryVisualizer.h"
#include "app/SpeedControlLogger.h"
#include "planning/TerrainCostmap.h"
#include "tracking/SimulationClock.h"
#include "tracking/VehicleUnits.h"
#include "tracking/TerrainSurfaceQuery.h"
#include "tracking/ControlSemantics.h"
#include "tracking/ReversePlanner.h"

#include "message_publisher_subscriber.h"
#include "msg_control.pb.h"
#include "msg_simple_car_states.pb.h"
#include "vehicle_message_init.h"

#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double kLoopDtSec = 0.1; // 控制循环仿真时间步长，单位为秒。
constexpr int kLoopPeriodMs = 100; // 状态订阅超时和循环休眠时间，单位为毫秒。
constexpr double kPi = 3.14159265358979323846; // 圆周率常量。
constexpr double kDegToRad = kPi / 180.0; // 角度转弧度的比例系数。
constexpr double kRadToDeg = 180.0 / kPi; // 弧度转角度的比例系数。
constexpr char kModuleName[] = "UserController_PathPlanning"; // EchoSim 消息模块名称。
constexpr char kControlTopic[] = "echo.controller.userdefined"; // 控制消息发布主题。
constexpr char kStateTopic[] = "echo.vehicle.states.Ego"; // 车辆状态订阅主题。

const char* path_safety_name_(PathSafetyState state)
{
    switch (state)
    {
    case PathSafetyState::Warning: return "WARNING";
    case PathSafetyState::Recovery: return "RECOVERY";
    case PathSafetyState::SevereRecovery: return "SEVERE_RECOVERY";
    case PathSafetyState::Stop: return "SAFETY_STOP";
    case PathSafetyState::Normal:
    default: return "NORMAL";
    }
}

bool target_is_behind_(const VehicleState2D& state, const Pose2D& target)
{
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 0.5)
        return false;
    const double forward_projection = std::cos(state.yaw) * dx
        + std::sin(state.yaw) * dy;
    // 只有目标确实位于车后方才换倒挡，普通大角度转弯仍然前进。
    return forward_projection < -0.17 * distance;
}

bool goal_target_is_behind_(const VehicleState2D& state, const Pose2D& target)
{
    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double distance = std::hypot(dx, dy);
    if (distance < 0.05)
        return false;
    return std::cos(state.yaw) * dx + std::sin(state.yaw) * dy < 0.0;
}

// 将 SDK 车辆状态转换为算法使用的二维状态。
VehicleState2D readVehicleState_(const sim_msg::VehicleState_SimpleCar& message)
{
    const auto& dynamics = message.car_state().vehdynamics();
    // EchoSim 的 vehDynamics.vx/vy 是车体系 km/h；算法边界统一转换为 m/s。
    // vehDynamics.ax/ay/az 是实际加速度，单位为 m/s^2，直接透传。
    return {dynamics.xo(), dynamics.yo(), dynamics.yaw() * kDegToRad,
            echoSimKphToMps(dynamics.vx()),
            echoSimKphToMps(dynamics.vy()), dynamics.ax(), dynamics.ay(),
            dynamics.az()};
}

// 检查状态消息是否包含车辆动力学数据。
bool hasVehicleDynamics_(const sim_msg::VehicleState_SimpleCar& message)
{
    return message.has_car_state() && message.car_state().has_vehdynamics();
}

bool read_surface_(const TerrainGrid& grid, double x, double y,
                   double& slope_deg, double& roughness)
{
    int col = 0;
    int row = 0;
    if (!grid.worldToGrid(x, y, col, row))
        return false;
    const std::size_t index = grid.index(row, col);
    if (index >= grid.slope_deg.size() || index >= grid.roughness.size())
        return false;
    slope_deg = grid.slope_deg[index];
    roughness = grid.roughness[index];
    return std::isfinite(slope_deg) && std::isfinite(roughness);
}

bool read_surface_(const TerrainSurfaceQuery& query, double x, double y,
                   double& slope_deg, double& roughness)
{
    return query.query(x, y, slope_deg, roughness);
}

double normalize_angle_(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

bool reverse_required_(const Path& path, std::size_t progress_index,
                       const VehicleState2D& state,
                       const ReverseDetectionConfig& config)
{
    return reverseRequired(path, progress_index, state.yaw, config);
}

// 终点前采用随距离下降的参考速度，而不是等进入 1 m 捕获范围后才
// 突然把目标速度从 1 m/s 改成 0。速度曲线同时受理论刹车上限约束，
// goal_brake_safety_distance_m 用来吸收通信和车辆动力学延迟。
double goalApproachSpeed_(double distance_to_goal,
                          const TrackingConfig& config)
{
    const double tolerance = std::max(0.0, config.goal_position_tolerance_m);
    const double slowdown_distance = std::max(
        config.goal_slowdown_distance_m, tolerance + 1e-3);
    const double distance_ratio = std::clamp(
        (distance_to_goal - tolerance)
            / (slowdown_distance - tolerance),
        0.0, 1.0);
    const double linear_speed = std::abs(config.base_speed_mps)
        * distance_ratio;
    const double braking_distance = std::max(
        0.0, distance_to_goal - tolerance
            - std::max(0.0, config.goal_brake_safety_distance_m));
    const double braking_speed = std::sqrt(
        2.0 * std::abs(config.max_deceleration_mps2)
        * braking_distance);
    const double profile_speed = std::min(linear_speed, braking_speed);
    // 只要还在终点容差之外，就不能因为制动距离保护而把目标速度
    // 变成 0；否则车辆可能停在终点外，又没有进入 ParkingGoal 状态。
    // 这里保留低速爬行，进入任务终点容差后再由停车状态置 0。
    const double crawl_speed = std::min(
        std::abs(config.goal_recovery_speed_mps),
        std::abs(config.base_speed_mps));
    return std::clamp(std::max(profile_speed, crawl_speed),
                      0.0, std::max(0.0, config.max_speed_mps));
}

double signed_longitudinal_speed_(const VehicleState2D& state)
{
    // readVehicleState_ 已将 EchoSim 的车体系 vx 转成 m/s；vx 的符号就是
    // 当前车辆相对车头方向的前进/倒退方向。
    return state.vx;
}

// 将挡位配置限制到 SDK 合法枚举范围。
const char* gear_name_(int gear_mode)
{
    switch (gear_mode)
    {
    case 1: return "PARK";
    case 2: return "REVERSE";
    case 3: return "NEUTRAL";
    case 4: return "DRIVE";
    default: return "NO_CONTROL";
    }
}

int gear_modeOrDrive_(int gear_mode)
{
    if (gear_mode >= static_cast<int>(sim_msg::Control_GEAR_MODE_NO_CONTROL)
        && gear_mode <= static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE))
        return gear_mode;
    return static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE);
}

// 构造距离/速度模式控制消息，横向控制使用目标前轮角。
// 所有纵向跟踪统一使用显式目标加速度，PP 只提供前轮角。
sim_msg::Control buildAcceleration_(const TrackingCommand& command,
                                    double acceleration_mps2,
                                    double timestamp_sec,
                                    int gear_mode)
{
    sim_msg::Control control;
    sim_msg::InitControl(control);
    control.mutable_header()->set_time_stamp(timestamp_sec);
    control.mutable_control_type()->set_acc_control_type(
        sim_msg::Control::CONTROL_TYPE::TARGET_ACC_CONTROL);
    const AccelerationCommand acceleration_command =
        makeAccelerationCommand(acceleration_mps2);
    control.mutable_control_type()->set_brake_control_type(
        acceleration_command.braking
            ? sim_msg::Control::CONTROL_TYPE::BRAKE_TARGET_ACC_CONTROL
            : sim_msg::Control::CONTROL_TYPE::BRAKE_NO_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_acc(
        acceleration_command.request_acc_mps2);
    if (!acceleration_command.braking)
        control.mutable_control_cmd()->set_request_brake_pressure(0.0);
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        toEchoSimFrontWheelAngle(command.front_wheel_angle_rad));
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造停车控制消息。
sim_msg::Control buildStop_(double timestamp_sec, int gear_mode)
{
    sim_msg::Control control;
    sim_msg::InitControl(control);
    control.mutable_header()->set_time_stamp(timestamp_sec);
    control.mutable_control_type()->set_acc_control_type(
        sim_msg::Control::CONTROL_TYPE::TARGET_ACC_CONTROL);
    control.mutable_control_type()->set_brake_control_type(
        sim_msg::Control::CONTROL_TYPE::BRAKE_TARGET_ACC_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    // 收尾阶段不要再发送负加速度。负 request_acc 是制动请求，车辆
    // 尚未完全停稳时可以使用；程序退出/驻车保持时必须是零加速度。
    control.mutable_control_cmd()->set_request_acc(0.0);
    control.mutable_control_cmd()->set_request_front_wheel_angle(0.0);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 发布控制消息并统一处理失败日志。
bool publish_(sim_msg::MessagePublisher<sim_msg::Control>& publisher,
              const sim_msg::Control& control,
              const char* label)
{
    if (publisher.Publish(control))
        return true;
    // 发布失败时保留 SDK 原始错误，便于区分主题、共享内存和消息格式问题。
    // SDK 示例未将单帧发布失败视为致命错误，保留运行循环以等待下一帧通信恢复。
    std::cerr << "[control] publish failed: " << label << std::endl;
    return false;
}
} // namespace

// 初始化消息系统并运行状态接收、控制计算、控制发布和到点停车流程。
void EchoSimRuntime::initialize() const
{
    std::cout << "[control] initializing message system" << std::endl;
    if (!sim_msg::Initialize(kModuleName))
        throw std::runtime_error("message system initialization failed: "
                                 + sim_msg::GetLastError());
    std::cout << "[control] message system initialized" << std::endl;
}

// 运行状态接收、控制计算、控制发布和到点停车流程。
int EchoSimRuntime::run(const TaskConfig& config, const RoutePlan& route) const
{
    const Path& path = route.path;
    if (path.empty())
        throw std::runtime_error("cannot run control with an empty path");
    if (!sim_msg::IsInitialized())
        initialize();
    std::cout << "[control] message system initialized; waiting for vehicle state"
              << std::endl;

    PathTracker tracker(config.tracking);
    TrajectoryVisualizer visualizer;
    if (!visualizer.initialize(path, config.start, config.goal, config.waypoints,
                               config.output_directory.string(),
                               config.enable_debug_output,
                               config.visualization_sample_interval_sec))
    {
        sim_msg::Shutdown();
        throw std::runtime_error("trajectory visualizer initialization failed");
    }
    std::cout << "[control] trajectory visualizer initialized; window="
              << (config.enable_debug_output ? "enabled" : "disabled") << std::endl;

    sim_msg::MessagePublisher<sim_msg::Control> control_publisher(kControlTopic);
    sim_msg::MessageSubscriber<sim_msg::VehicleState_SimpleCar> state_subscriber(kStateTopic);
    std::cout << "[control] publisher/subscriber created" << std::endl;
    SimulationClock simulation_clock;
    double control_time_sec = 0.0; // 当前仿真时间戳，单位为秒。
    bool fallback_time_warned = false;
    std::size_t state_count = 0; // 有效车辆状态接收计数。
    bool start_checked = false; // 是否已完成起点状态检查。
    bool stop_published = false;
    bool have_state = false; // 是否至少接收过一帧有效状态。
    std::size_t progress_index = 0; // 当前路径进度，只允许向前搜索。
    // 首帧状态到来前持续发送安全停车控制，避免仿真端等待控制帧而不推进状态反馈。
    sim_msg::Control last_control; // 最近一次成功发布的控制消息。
    bool have_last_control = true; // 初始停车控制可以在状态到来前复用。

    // 首帧尚未收到车辆状态，不发布控制消息，行为与原 SDK 示例保持一致。
    have_last_control = false;

    std::size_t subscribe_miss_count = 0;
    LongitudinalController longitudinal;
    SpeedControlLogger speed_logger;
    if (!speed_logger.initialize(config.output_directory.string(),
                                 config.enable_speed_visualization))
        throw std::runtime_error("speed control log initialization failed");
    std::size_t next_stop = 0;
    enum class Stage { Driving, ApproachingWaypoint, ParkingWaypoint,
                       RecoveringWaypoint, ApproachingGoal, ParkingGoal,
                       RecoveringGoal, SafetyStop };
    Stage stage = Stage::Driving;
    double hold_sec = 0.0;
    bool reverse_mode = false;
    bool goal_recovery_mode = false;
    bool shifting_gear = false;
    bool shift_target_reverse = false;
    double shift_hold_sec = 0.0;

    std::cout << "[control] entering state-control loop" << std::endl;
    while (!sim_msg::HasError() && !stop_published)
    {
        sim_msg::VehicleState_SimpleCar state_message;
        long long message_time = 0; // SDK 返回的消息时间戳。
        if (!state_subscriber.Subscribe(state_message, &message_time, kLoopPeriodMs))
        {
            ++subscribe_miss_count;
            if (subscribe_miss_count % 200 == 0)
            {
                std::cout << "[state] waiting for vehicle state, timeout_count="
                          << subscribe_miss_count << std::endl;
            }
            if (have_last_control)
            {
                last_control.mutable_header()->set_time_stamp(control_time_sec);
                publish_(control_publisher, last_control, "reused control");
            }
            sim_msg::SleepMS(kLoopPeriodMs);
            // 没有收到状态时不推进仿真时钟，避免把墙钟等待时间误当成仿真时间。
            continue;
        }
        if (!hasVehicleDynamics_(state_message))
        {
            std::cerr << "[state] vehicle dynamics unavailable" << std::endl;
            continue;
        }

        const VehicleState2D state = readVehicleState_(state_message);
        have_state = true;
        ++state_count;

        // VehicleState_SimpleCar::time_stamp() 是仿真时间（秒），控制器的
        // PID、驻留计时、速度日志和轨迹采样均以此为时间基准。
        const double reported_sim_time_sec = state_message.time_stamp();
        const SimulationTimeStep time_step = simulation_clock.update(
            reported_sim_time_sec, kLoopDtSec);
        control_time_sec = time_step.time_sec;
        const double dt = time_step.dt_sec;
        if (!time_step.reported_time_valid && !simulation_clock.initialized())
        {
            if (!fallback_time_warned)
            {
                std::cerr << "[state] simulation timestamp unavailable or not "
                             "monotonic; using 0.1 s fallback" << std::endl;
                fallback_time_warned = true;
            }
        }
        if (!start_checked)
            std::cout << "[state] first valid vehicle state received" << std::endl;
        subscribe_miss_count = 0;
        if (!start_checked)
        {
            const double position_error = std::hypot(
                state.x - config.start.x, state.y - config.start.y);
            const double yaw_error = std::abs(std::atan2(
                std::sin(state.yaw - config.start.yaw),
                std::cos(state.yaw - config.start.yaw)));
            if (position_error > config.start_position_tolerance_m
                || yaw_error > config.start_yaw_tolerance_rad)
            {
                std::cerr << std::fixed << std::setprecision(3)
                          << "[state] initial pose differs: position_error="
                          << position_error << " yaw_error_rad=" << yaw_error << std::endl;
            }
            start_checked = true;
        }

        // 正常行驶时仅由前方连续反向几何触发换挡；普通急转弯仍保持前进。
        const bool requested_reverse = reverse_required_(
            path, progress_index, state, config.tracking.reverse_detection);
        if (!shifting_gear && requested_reverse != reverse_mode
            && stage == Stage::Driving)
        {
            shifting_gear = true;
            shift_target_reverse = requested_reverse;
            shift_hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[gear] preparing to shift from "
                      << (reverse_mode ? "REVERSE" : "DRIVE") << " to "
                      << (shift_target_reverse ? "REVERSE" : "DRIVE")
                      << "; braking to zero" << std::endl;
        }

        TrackingCommand command = tracker.calculate(
            path, state, progress_index, reverse_mode);
        progress_index = command.nearest_path_index;
        // PID 使用当前挡位下的有符号任务方向速度：Drive 下回溜为负，
        // Reverse 下车体负 vx 表示沿倒车方向前进。日志仍单独记录速度大小。
        const double signed_speed_for_pid = longitudinalSpeedForGear(
            state.vx, reverse_mode);
        const double actual_speed = std::abs(state.vx);
        // 途经点是否到达只看当前锁存任务点的坐标。路径栅格点可能因为
        // 吸附、折线绕行或车辆回收而没有“越过”stop_index；再叠加索引
        // 条件会导致车辆进入途经点圆域却永远不能锁存停车状态。
        const bool at_waypoint = next_stop < config.waypoints.size()
            && std::hypot(state.x - config.waypoints[next_stop].x,
                          state.y - config.waypoints[next_stop].y)
                <= config.tracking.waypoint_capture_radius_m;
        const double waypoint_distance_m = next_stop < config.waypoints.size()
            ? std::hypot(state.x - config.waypoints[next_stop].x,
                         state.y - config.waypoints[next_stop].y)
            : std::numeric_limits<double>::infinity();
        const bool at_goal = std::hypot(state.x - config.goal.x,
                                        state.y - config.goal.y)
            <= config.tracking.goal_position_tolerance_m;

        const double path_distance_m = tracker.distanceToPath(path, state, 0);
        const PathSafetyState path_safety = classifyPathSafety(
            path_distance_m, config.tracking);
        if (path_safety == PathSafetyState::Stop
            && requiresPersistentPathSafetyStop(path_distance_m)
            && stage != Stage::ParkingWaypoint
            && stage != Stage::ParkingGoal
            && stage != Stage::SafetyStop)
        {
            stage = Stage::SafetyStop;
            longitudinal.reset();
            std::cerr << "[safety] vehicle is " << path_distance_m
                      << " m from global path; entering persistent PARK" << std::endl;
        }
        // 脱轨时，不能只沿着旧的单调进度继续追踪；从全局最近路径点
        // 重新计算 PP，车辆才有机会回到当前路径。只要路径距离仍是
            // 有效数值，即使超过原来的 stop 阈值也以 1 m/s 低速恢复，避免
        // 车辆已经回到可识别路径附近后仍被永久 PARK 锁死。
        if ((path_safety == PathSafetyState::Recovery
             || path_safety == PathSafetyState::SevereRecovery
             || (path_safety == PathSafetyState::Stop
                 && std::isfinite(path_distance_m)))
            && stage != Stage::ParkingWaypoint
            && stage != Stage::ParkingGoal
            && stage != Stage::SafetyStop)
        {
            command = tracker.calculate(path, state, 0, reverse_mode);
            progress_index = command.nearest_path_index;
            const double recovery_speed = std::abs(
                config.tracking.off_path_recovery_speed_mps);
            command.target_speed_mps = std::min(
                std::abs(command.target_speed_mps), recovery_speed);
        }
        if (stage == Stage::Driving || stage == Stage::ApproachingWaypoint
            || stage == Stage::ApproachingGoal)
        {
            if (at_waypoint)
            {
                stage = Stage::ParkingWaypoint; hold_sec = 0.0;
                std::cout << "[waypoint] reached waypoint " << next_stop + 1
                          << ", entering parking, gear=PARK" << std::endl;
            }
            else if (next_stop < config.waypoints.size()
                     && shouldApproachWaypoint(
                         waypoint_distance_m,
                         config.tracking.waypoint_slowdown_distance_m,
                         stage == Stage::ApproachingWaypoint))
                stage = Stage::ApproachingWaypoint;
            else if (at_goal)
            {
                stage = Stage::ParkingGoal;
                goal_recovery_mode = false;
                hold_sec = 0.0;
                longitudinal.reset();
                std::cout << std::fixed << std::setprecision(3)
                          << "[goal] entered capture range, parking timer started"
                          << " distance_m="
                          << std::hypot(state.x - config.goal.x,
                                        state.y - config.goal.y)
                          << " hold_sec=0.000" << std::endl;
            }
            else if (next_stop >= route.stop_indices.size()
                     && std::hypot(state.x - config.goal.x,
                                   state.y - config.goal.y)
                         <= config.tracking.goal_slowdown_distance_m)
            {
                if (stage != Stage::ApproachingGoal)
                {
                    std::cout << std::fixed << std::setprecision(3)
                              << "[goal] approaching task goal, distance_m="
                              << std::hypot(state.x - config.goal.x,
                                            state.y - config.goal.y)
                              << ", target=task_goal" << std::endl;
                }
                stage = Stage::ApproachingGoal;
            }
        }
        // 停车后如果车辆明显越过终点捕获迟滞边界，回到低速终点修正
        // 阶段；仅在 1~1.25 m 迟滞带内时保持终点位置控制，不立即换挡。
        const double goal_distance_m = std::hypot(
            state.x - config.goal.x, state.y - config.goal.y);
        if (stage == Stage::ParkingGoal
            && goalCaptureShouldRecover(
                goal_distance_m,
                config.tracking.goal_position_tolerance_m,
                config.tracking.goal_capture_exit_radius_m))
        {
            stage = Stage::RecoveringGoal;
            goal_recovery_mode = true;
            hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[goal] left capture range, restarting low-speed approach"
                      << std::endl;
        }
        if (stage == Stage::ParkingWaypoint && !at_waypoint)
        {
            stage = Stage::RecoveringWaypoint;
            hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[waypoint] left capture range, recovering current waypoint"
                      << std::endl;
        }
        if (stage == Stage::RecoveringWaypoint && at_waypoint)
        {
            stage = Stage::ParkingWaypoint;
            hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[waypoint] recovered capture range, parking again"
                      << std::endl;
        }
        if (stage == Stage::RecoveringGoal && at_goal)
        {
            stage = Stage::ParkingGoal;
            goal_recovery_mode = false;
            hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[goal] recovered capture range, parking again"
                      << std::endl;
        }

        // 终点不使用持续 PARK：根据终点相对车头的前后关系选择驱动挡。
        // 目标在车头后方时挂 REVERSE，车辆越过终点后即可倒车回收；
        // 目标在车头前方时使用 DRIVE。换挡过程仍先以 PARK 安全停稳。
        const bool goal_position_stage = stage == Stage::ParkingGoal
            || stage == Stage::RecoveringGoal;
        const bool goal_reverse = goal_position_stage
            && goal_target_is_behind_(state, config.goal);
        if (goal_position_stage && !shifting_gear
            && goal_reverse != reverse_mode)
        {
            shifting_gear = true;
            shift_target_reverse = goal_reverse;
            shift_hold_sec = 0.0;
            longitudinal.reset();
            std::cout << "[goal] preparing gear change to "
                      << (goal_reverse ? "REVERSE" : "DRIVE")
                      << " for target position" << std::endl;
        }

        const bool recovery_stage = stage == Stage::RecoveringWaypoint
            || stage == Stage::RecoveringGoal;
        // 途经点仍使用 PARK 停车；终点改为 DRIVE/REVERSE 位置控制，
        // 因为仿真器在 PARK + 零加速度时仍可能有小幅溜车。
        const bool parking_stage = stage == Stage::ParkingWaypoint;
        const double ground_speed = std::hypot(state.vx, state.vy);
        const bool stopped_for_parking = ground_speed
            <= config.tracking.stopped_speed_mps;

        if (path_safety == PathSafetyState::Warning
            || path_safety == PathSafetyState::Recovery
            || path_safety == PathSafetyState::SevereRecovery
            || (path_safety == PathSafetyState::Stop
                && std::isfinite(path_distance_m)))
        {
            if (stage == Stage::Driving || stage == Stage::ApproachingWaypoint
                || stage == Stage::ApproachingGoal)
            {
                const double recovery_speed = std::abs(
                    config.tracking.off_path_recovery_speed_mps);
                command.target_speed_mps = std::min(
                    std::abs(command.target_speed_mps), recovery_speed);
                if (path_safety != PathSafetyState::Warning)
                    std::cerr << "[safety] path_distance_m=" << path_distance_m
                              << " state=" << path_safety_name_(path_safety)
                              << "; limiting speed" << std::endl;
            }
        }

        if (recovery_stage)
        {
            const Pose2D& recovery_target = stage == Stage::RecoveringWaypoint
                ? config.waypoints[next_stop] : config.goal;
            const bool recovery_reverse = stage == Stage::RecoveringGoal
                ? goal_target_is_behind_(state, recovery_target)
                : target_is_behind_(state, recovery_target);
            command = tracker.calculateToTarget(state, recovery_target,
                                                recovery_reverse);
            // 恢复时允许回退到全局最近路径点，避免车辆偏离后仍从旧的
            // 单调 progress_index 向前搜索，直接追逐很远的未来路径。
            const std::size_t recovery_nearest =
                tracker.findNearestPathPoint(path, state, 0);
            command.nearest_path_index = recovery_nearest;
            if (stage == Stage::RecoveringWaypoint
                && next_stop < route.stop_indices.size())
                command.target_path_index = clampTaskTargetPathIndex(
                    route.stop_indices[next_stop], path.size());
            else
                command.target_path_index = recovery_nearest;
            progress_index = recovery_nearest;
            command.target_speed_mps = stage == Stage::RecoveringWaypoint
                ? std::abs(config.tracking.waypoint_recovery_speed_mps)
                : goalCrawlSpeed(
                    goal_distance_m,
                    config.tracking.goal_position_tolerance_m,
                    std::min(std::abs(config.tracking.goal_recovery_speed_mps),
                             std::abs(config.tracking.off_path_recovery_speed_mps)));
            command.target_speed_mps = std::min(
                command.target_speed_mps,
                std::abs(config.tracking.off_path_recovery_speed_mps));
            command.front_wheel_angle_rad = std::clamp(
                command.front_wheel_angle_rad,
                -std::abs(config.tracking.parking_steering_limit_rad),
                std::abs(config.tracking.parking_steering_limit_rad));
            if (!shifting_gear && recovery_reverse != reverse_mode)
            {
                shifting_gear = true;
                shift_target_reverse = recovery_reverse;
                shift_hold_sec = 0.0;
                longitudinal.reset();
                std::cout << "[gear] preparing recovery shift from "
                          << (reverse_mode ? "REVERSE" : "DRIVE") << " to "
                          << (shift_target_reverse ? "REVERSE" : "DRIVE")
                          << "; braking to zero" << std::endl;
            }
        }

        if (parking_stage)
        {
            const Pose2D& parking_target = stage == Stage::ParkingWaypoint
                ? config.waypoints[next_stop] : config.goal;
            const TrackingCommand parking_command = tracker.calculateToTarget(
                state, parking_target, reverse_mode);
            command.target_speed_mps = 0.0;
            if (next_stop < route.stop_indices.size())
                command.target_path_index = clampTaskTargetPathIndex(
                    route.stop_indices[next_stop], path.size());
            command.front_wheel_angle_rad = stopped_for_parking
                ? 0.0
                : std::clamp(parking_command.front_wheel_angle_rad,
                             -std::abs(config.tracking.parking_steering_limit_rad),
                             std::abs(config.tracking.parking_steering_limit_rad));
            if (stage == Stage::ParkingWaypoint && at_waypoint)
            {
                hold_sec = accumulateGoalHoldSeconds(hold_sec, dt, true);
            }
            else
                hold_sec = 0.0;
            if (stage == Stage::ParkingWaypoint
                && hold_sec >= config.tracking.waypoint_hold_duration_sec)
            {
                std::cout << "[waypoint] waypoint " << next_stop + 1
                          << " parking complete, hold=" << hold_sec
                          << " s, gear=DRIVE" << std::endl;
                ++next_stop; stage = Stage::Driving; hold_sec = 0.0;
                longitudinal.reset();
            }
        }

        // 进入途经点减速区后，禁止继续使用普通 PP 的未来前视点。
        // 这里重新计算指向当前任务途经点的 PP 转角，并将显示/控制目标
        // 索引固定到当前途经点对应的全局路径序号。
        if (stage == Stage::ApproachingWaypoint)
        {
            const Pose2D& waypoint_target = config.waypoints[next_stop];
            const TrackingCommand waypoint_command = tracker.calculateToTarget(
                state, waypoint_target, reverse_mode);
            command.front_wheel_angle_rad = waypoint_command.front_wheel_angle_rad;
            command.target_speed_mps = std::min(
                std::abs(command.target_speed_mps), 1.0);
            if (next_stop < route.stop_indices.size())
                command.target_path_index = clampTaskTargetPathIndex(
                    route.stop_indices[next_stop], path.size());
        }
        if (stage == Stage::ParkingGoal)
        {
            // 终点进入 1 m 捕获范围后立即累计保持时间，不要求先达到
            // stopped_speed_mps。轻微滑出到 1.25 m 迟滞带时保留已累计时间；
            // 真正超过退出半径时，前面的 RecoveringGoal 状态会清零计时。
            if (at_goal)
                hold_sec = accumulateGoalHoldSeconds(hold_sec, dt, true);
        }
        if (stage == Stage::ParkingGoal)
        {
            // 终点附近不挂 PARK，而是使用任务终点作为实际位置目标。
            // 在范围外给小的驱动目标速度，目标越过车头后由上面的
            // goal_reverse 触发 REVERSE；进入范围后目标速度为 0，
            // 由加速度控制器负责减速，但仍保持 DRIVE/REVERSE 挡位。
            command = tracker.calculateToTarget(state, config.goal,
                                                 goal_reverse);
            const double goal_crawl_speed = std::min(
                std::abs(config.tracking.goal_recovery_speed_mps),
                std::abs(config.tracking.off_path_recovery_speed_mps));
            command.target_speed_mps = goalCrawlSpeed(
                goal_distance_m,
                config.tracking.goal_position_tolerance_m,
                std::abs(config.tracking.goal_recovery_speed_mps));
            if (hold_sec >= config.tracking.goal_hold_duration_sec)
            {
                stop_published = true;
                std::cout << "[goal] position hold complete, hold="
                          << hold_sec << " s, final_gear="
                          << (goal_reverse ? "REVERSE" : "DRIVE")
                          << std::endl;
            }
        }
        if (stage == Stage::ApproachingWaypoint)
            command.target_speed_mps = std::min(command.target_speed_mps, 1.0);
        if (stage == Stage::ApproachingGoal)
        {
            const double distance_to_goal = std::hypot(
                state.x - config.goal.x, state.y - config.goal.y);
            // 终点接近阶段直接把任务终点作为 PP 目标，避免最后几个
            // 栅格路径点与精确任务坐标存在偏差时继续追逐错误参考点。
            TrackingCommand goal_command = tracker.calculateToTarget(
                state, config.goal, reverse_mode);
            goal_command.nearest_path_index = command.nearest_path_index;
            goal_command.target_path_index = command.target_path_index;
            goal_command.remaining_path_distance_m =
                command.remaining_path_distance_m;
            command = goal_command;
            command.target_speed_mps = goalApproachSpeed_(
                distance_to_goal, config.tracking);
            if (goal_recovery_mode)
                command.target_speed_mps = std::min(
                    command.target_speed_mps,
                    std::abs(config.tracking.goal_recovery_speed_mps));
        }

        if (std::abs(command.front_wheel_angle_rad) > 10.0 * kDegToRad)
        {
            const double curve_speed = std::abs(config.tracking.curve_speed_mps);
            command.target_speed_mps = std::min(
                std::abs(command.target_speed_mps), curve_speed);
        }

        if (shifting_gear)
        {
            command.target_speed_mps = 0.0;
            shift_hold_sec += dt;
            if (gearShiftStopped(signed_speed_for_pid,
                                 config.tracking.stopped_speed_mps)
                && shift_hold_sec >= config.tracking.gear_change_stop_duration_sec)
            {
                reverse_mode = shift_target_reverse;
                shifting_gear = false;
                longitudinal.reset();
                std::cout << "[gear] shift complete, gear="
                          << (reverse_mode ? "REVERSE" : "DRIVE") << std::endl;
            }
        }
        if (stage == Stage::SafetyStop)
        {
            command.target_speed_mps = 0.0;
            command.front_wheel_angle_rad = 0.0;
        }
        const std::size_t nearest_index = command.nearest_path_index;
        const std::size_t target_index = command.target_path_index;
        const bool waypoint_target_stage = stage == Stage::ApproachingWaypoint
            || stage == Stage::ParkingWaypoint
            || stage == Stage::RecoveringWaypoint;
        const bool goal_target_stage = stage == Stage::ApproachingGoal
            || stage == Stage::ParkingGoal || stage == Stage::RecoveringGoal;
        const PathPoint goal_reference{config.goal.x, config.goal.y,
                                       config.goal.yaw};
        PathPoint waypoint_reference;
        if (waypoint_target_stage)
        {
            const Pose2D& waypoint = config.waypoints[next_stop];
            waypoint_reference = {waypoint.x, waypoint.y, waypoint.yaw};
        }
        const PathPoint& reference = waypoint_target_stage
            ? waypoint_reference
            : (goal_target_stage
                ? goal_reference
                : path[std::min(target_index, path.size() - 1)]);
        const double signed_speed = signed_longitudinal_speed_(state);
        // 途经点停车和终点位置控制使用不同挡位策略：途经点保持 PARK，
        // 终点使用前方 DRIVE/越过目标后的 REVERSE。
        const int active_gear = shifting_gear
            ? (goal_position_stage
                ? (reverse_mode ? 2 : config.gear_mode) : 1)
            : (stage == Stage::SafetyStop || parking_stage
                ? 1 : (reverse_mode ? 2 : config.gear_mode));
        const double stop_limit = std::abs(
            config.tracking.max_deceleration_mps2);
        const double hold_brake = std::min(stop_limit, 0.25);
        // PARK 不是主动制动。坡面或横向滑移可能让车辆在 PARK 下继续
        // 移动，因此停车和安全停止都要持续发送显式制动/保持制动。
        const double parking_acceleration = ground_speed
            > config.tracking.stopped_speed_mps
            ? -stop_limit : -hold_brake;
        const double acceleration = stage == Stage::SafetyStop
            ? (ground_speed > 0.01 ? -stop_limit : -hold_brake)
            : parking_stage
            ? parking_acceleration
            : longitudinal.update(command.target_speed_mps,
                                  signed_speed_for_pid,
                                  std::max(dt, 1e-3), config.tracking);
        const double guarded_acceleration = applyGroundSpeedSafety(
            acceleration, command.target_speed_mps, ground_speed,
            config.tracking.speed_guard_margin_mps,
            config.tracking.max_deceleration_mps2);
        speed_logger.update(control_time_sec, command.target_speed_mps,
                            actual_speed, guarded_acceleration);
        // PARK 只表示挡位，不等于主动制动；始终发布显式加速度控制，
        // 防止 buildStop_ 的零 request_acc 让车辆重新溜车。
        sim_msg::Control control = buildAcceleration_(
            command, guarded_acceleration, control_time_sec, active_gear);
        const bool control_published = publish_(control_publisher, control,
                                                "tracking control");
        if (control_published)
        {
            last_control = control;
            have_last_control = true;
        }
        visualizer.update(state, target_index, control_time_sec);
        visualizer.pumpWindow();

        if (config.control_log_interval > 0
            && state_count % config.control_log_interval == 0)
        {
            const double logged_speed_error = parking_stage
                ? -signed_speed
                : command.target_speed_mps - actual_speed;
            std::cout << std::fixed << std::setprecision(3)
                      << "[track] state_count=" << state_count
                      << " sim_time_sec=" << control_time_sec
                      << " nearest_index=" << nearest_index
                      << " target_index=" << target_index
                      << " reference=(x=" << reference.x
                      << ",y=" << reference.y
                      << ",yaw_rad=" << reference.yaw << ')'
                      << " actual=(x=" << state.x
                      << ",y=" << state.y
                      << ",yaw_rad=" << state.yaw << ')'
                      << " control=(target_speed_mps=" << command.target_speed_mps
                      << ",speed_error_mps="
                      << logged_speed_error
                      << ",vx_mps=" << state.vx << ",vy_mps=" << state.vy
                      << ",accel_mps2=" << guarded_acceleration
                      << ",gear=" << gear_name_(active_gear)
                      << ",front_wheel_deg="
                      << command.front_wheel_angle_rad * kRadToDeg << ')'
                      << " actual_speed_mps=" << ground_speed
                      << " path_distance_m=" << path_distance_m
                       << " tracking_safety_state="
                       << path_safety_name_(path_safety)
                       << " actual_accel=(ax_mps2=" << state.ax
                      << ",ay_mps2=" << state.ay
                      << ",az_mps2=" << state.az << ')'
                      ;
            double slope_deg = 0.0;
            double roughness = 0.0;
            const bool surface_available =
                (route.terrain_grid
                 && read_surface_(*route.terrain_grid, state.x, state.y,
                                  slope_deg, roughness))
                || (route.surface_query
                    && read_surface_(*route.surface_query, state.x, state.y,
                                     slope_deg, roughness));
            if (surface_available)
            {
                std::cout << " surface=(slope_deg=" << slope_deg
                          << ",roughness=" << roughness << ')';
            }
            else
            {
                std::cout << " surface=(unavailable)";
            }
            // 日志目标必须跟随状态切换后的当前尚未完成任务点。特别是
            // 途经点刚完成并执行 ++next_stop 的这一帧，不能把旧途经点
            // 在状态切换前计算的距离错标成新途经点距离。
            if (next_stop < config.waypoints.size())
            {
                const double logged_waypoint_distance_m = std::hypot(
                    state.x - config.waypoints[next_stop].x,
                    state.y - config.waypoints[next_stop].y);
                const bool logged_at_waypoint = logged_waypoint_distance_m
                    <= config.tracking.waypoint_capture_radius_m;
                const double waypoint_hold_sec =
                    (stage == Stage::ApproachingWaypoint
                     || stage == Stage::ParkingWaypoint
                     || stage == Stage::RecoveringWaypoint)
                    ? hold_sec : 0.0;
                std::cout << " waypoint_index=" << next_stop + 1
                          << " waypoint_distance_m="
                          << logged_waypoint_distance_m
                          << " waypoint_in_capture_range="
                          << (logged_at_waypoint ? "true" : "false")
                          << " waypoint_hold_sec=" << waypoint_hold_sec
                          << "/" << config.tracking.waypoint_hold_duration_sec;
            }
            else
            {
                std::cout << " goal_distance_m=" << goal_distance_m
                          << " goal_in_capture_range="
                          << (at_goal ? "true" : "false")
                          << " goal_hold_sec="
                          << ((stage == Stage::ParkingGoal) ? hold_sec : 0.0);
            }
            std::cout << std::endl;
            if (stage == Stage::ParkingGoal)
            {
                std::cout << std::fixed << std::setprecision(3)
                          << "[goal] in capture range, distance_m="
                          << goal_distance_m << ", hold_sec=" << hold_sec
                          << "/" << config.tracking.goal_hold_duration_sec
                          << std::endl;
            }
        }

        if (stage == Stage::ParkingWaypoint && hold_sec <= dt)
            std::cout << "[parking] waypoint " << next_stop + 1
                      << " stopped, target=0 m/s, gear=PARK" << std::endl;
        sim_msg::SleepMS(kLoopPeriodMs);
    }

    if (!stop_published)
        publish_(control_publisher, buildStop_(control_time_sec, config.gear_mode),
                 "shutdown stop");
    visualizer.saveFinal();
    speed_logger.close();
    state_subscriber.Unsubscribe();
    sim_msg::Shutdown();
    std::cout << "Controller stopped; received state=" << (have_state ? "yes" : "no")
              << std::endl;
    return 0;
}
