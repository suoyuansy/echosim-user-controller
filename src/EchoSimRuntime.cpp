// 文件功能：实现 EchoSim 状态订阅、三种跟踪控制发布、日志和到点停车。
#include "EchoSimRuntime.h"

#include "PathTracker.h"
#include "TrajectoryVisualizer.h"

#include "message_publisher_subscriber.h"
#include "msg_control.pb.h"
#include "msg_simple_car_states.pb.h"
#include "vehicle_message_init.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace
{
constexpr double kLoopDtSec = 0.01; // 控制循环仿真时间步长，单位为秒。
constexpr int kLoopPeriodMs = 10; // 状态订阅超时和循环休眠时间，单位为毫秒。
constexpr double kPi = 3.14159265358979323846; // 圆周率常量。
constexpr double kDegToRad = kPi / 180.0; // 角度转弧度的比例系数。
constexpr double kRadToDeg = 180.0 / kPi; // 弧度转角度的比例系数。
constexpr char kModuleName[] = "UserController_PathPlanning"; // EchoSim 消息模块名称。
constexpr char kControlTopic[] = "echo.controller.userdefined"; // 控制消息发布主题。
constexpr char kStateTopic[] = "echo.vehicle.states.Ego"; // 车辆状态订阅主题。

// 将 SDK 车辆状态转换为算法使用的二维状态。
VehicleState2D readVehicleState_(const sim_msg::VehicleState_SimpleCar& message)
{
    const auto& dynamics = message.car_state().vehdynamics();
    return {dynamics.xo(), dynamics.yo(), dynamics.yaw() * kDegToRad,
            dynamics.vx(), dynamics.vy()};
}

// 检查状态消息是否包含车辆动力学数据。
bool hasVehicleDynamics_(const sim_msg::VehicleState_SimpleCar& message)
{
    return message.has_car_state() && message.car_state().has_vehdynamics();
}

// 将挡位配置限制到 SDK 合法枚举范围。
int gear_modeOrDrive_(int gear_mode)
{
    if (gear_mode >= static_cast<int>(sim_msg::Control_GEAR_MODE_NO_CONTROL)
        && gear_mode <= static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE))
        return gear_mode;
    return static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE);
}

// 构造距离/速度模式控制消息，横向控制使用目标前轮角。
sim_msg::Control buildDistanceVelocity_(const TrackingCommand& command,
                                        double timestamp_sec,
                                        int gear_mode)
{
    sim_msg::Control control;
    sim_msg::InitControl(control);
    control.mutable_header()->set_time_stamp(timestamp_sec);
    control.mutable_control_type()->set_acc_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_DISTANCE_AND_VELOCITY);
    control.mutable_control_type()->set_brake_control_type(
        sim_msg::Control::CONTROL_TYPE::BRAKE_TARGET_ACC_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_distance_2_stop(
        std::max(0.0, command.remaining_path_distance_m));
    // Pure Pursuit/Stanley 只将距离减速后的速度上限交给 SDK 执行。
    control.mutable_control_cmd()->set_max_velocity(
        std::max(0.0, command.target_speed_mps));
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad * kRadToDeg);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造 LQR 加速度模式控制消息。
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
    control.mutable_control_type()->set_brake_control_type(
        sim_msg::Control::CONTROL_TYPE::BRAKE_TARGET_ACC_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_acc(acceleration_mps2);
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad * kRadToDeg);
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
    control.mutable_control_cmd()->set_request_acc(-2.0);
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
int EchoSimRuntime::run(const TaskConfig& config, const Path& path) const
{
    if (path.empty())
        throw std::runtime_error("cannot run control with an empty path");
    if (!sim_msg::IsInitialized())
        initialize();
    std::cout << "[control] message system initialized; waiting for vehicle state"
              << std::endl;

    PathTracker tracker(config.tracking);
    TrajectoryVisualizer visualizer;
    if (!visualizer.initialize(path, config.start, config.goal,
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
    double control_time_sec = 0.0; // 当前控制循环时间戳。
    std::size_t publish_count = 0; // 成功发布的跟踪控制帧计数。
    bool start_checked = false; // 是否已完成起点状态检查。
    bool stop_published = false; // 是否已发布终点停车指令。
    bool have_state = false; // 是否至少接收过一帧有效状态。
    // 首帧状态到来前持续发送安全停车控制，避免仿真端等待控制帧而不推进状态反馈。
    sim_msg::Control last_control; // 最近一次成功发布的控制消息。
    bool have_last_control = true; // 初始停车控制可以在状态到来前复用。

    // 首帧尚未收到车辆状态，不发布控制消息，行为与原 SDK 示例保持一致。
    have_last_control = false;

    std::size_t subscribe_miss_count = 0; // 未收到状态消息的轮询次数。

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
            control_time_sec += kLoopDtSec;
            continue;
        }
        if (!hasVehicleDynamics_(state_message))
        {
            std::cerr << "[state] vehicle dynamics unavailable" << std::endl;
            continue;
        }

        const VehicleState2D state = readVehicleState_(state_message);
        have_state = true;
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

        const std::size_t reference_index = tracker.findNearestPathPoint(path, state);
        const PathPoint& reference = path[reference_index];
        const TrackingCommand command = tracker.calculate(path, state);
        const double actual_speed = std::hypot(state.vx, state.vy);
        sim_msg::Control control;
        if (config.tracking.method == TrackerMethod::Lqr)
        {
            const double acceleration = LongitudinalController::speedP(
                command.target_speed_mps, actual_speed,
                config.tracking.lqr_longitudinal_kp,
                -config.tracking.max_deceleration_mps2,
                config.tracking.max_acceleration_mps2);
            control = buildAcceleration_(command, acceleration,
                                         control_time_sec, config.gear_mode);
        }
        else
        {
            control = buildDistanceVelocity_(command, control_time_sec,
                                              config.gear_mode);
        }
        const bool control_published = publish_(control_publisher, control,
                                                "tracking control");
        if (control_published)
        {
            last_control = control;
            have_last_control = true;
            ++publish_count;
        }
        visualizer.update(state, reference_index, control_time_sec);
        visualizer.pumpWindow();

        if (control_published && config.control_log_interval > 0
            && publish_count % config.control_log_interval == 0)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[track] publish_count=" << publish_count
                      << " ref_index=" << reference_index
                      << " reference=(x=" << reference.x
                      << ",y=" << reference.y
                      << ",yaw_rad=" << reference.yaw << ')'
                      << " actual=(x=" << state.x
                      << ",y=" << state.y
                      << ",yaw_rad=" << state.yaw << ')'
                      << " reference_control=(speed_mps=" << command.target_speed_mps
                      << ",front_wheel_deg="
                      << command.front_wheel_angle_rad * kRadToDeg << ')'
                      << " actual_speed_mps=" << actual_speed
                      << std::endl;
        }

        if (command.reached_goal)
        {
            const sim_msg::Control stop = buildStop_(
                control_time_sec + kLoopDtSec, config.gear_mode);
            if (!publish_(control_publisher, stop, "goal stop"))
                throw std::runtime_error("goal stop publish failed");
            stop_published = true;
            std::cout << "[goal] position and yaw reached; vehicle stopped" << std::endl;
        }
        control_time_sec += kLoopDtSec;
        sim_msg::SleepMS(kLoopPeriodMs);
    }

    if (!stop_published)
        publish_(control_publisher, buildStop_(control_time_sec, config.gear_mode),
                 "shutdown stop");
    visualizer.saveFinal();
    state_subscriber.Unsubscribe();
    sim_msg::Shutdown();
    std::cout << "Controller stopped; received state=" << (have_state ? "yes" : "no")
              << std::endl;
    return 0;
}
