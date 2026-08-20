// 实现 EchoSim 状态订阅、三种跟踪控制发布、日志和到点停车。
#include "tracking/EchoSimRuntime.h"

#include "tracking/PathTracker.h"
#include "app/TrajectoryVisualizer.h"

#include "message_publisher_subscriber.h"
#include "msg_control.pb.h"
#include "msg_simple_car_states.pb.h"
#include "vehicle_message_init.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace
{
constexpr double kLoopDtSec = 0.1; // 控制循环仿真时间步长，单位为秒。
constexpr int kLoopPeriodMs = 100; // 状态订阅超时和循环休眠时间，单位为毫秒。
constexpr double kPi = 3.14159265358979323846; // 圆周率常量。
constexpr double kDegToRad = kPi / 180.0; // 角度转弧度的比例系数。
constexpr double kRadToDeg = 180.0 / kPi; // 弧度转角度的比例系数。
constexpr double kKphToMps = 1.0 / 3.6; // EchoSim vx/vy 使用千米每小时（车体系分量：x 前向、y 左正）。
constexpr double kMpsToKph = 3.6; // EchoSim 速度控制字段使用千米每小时。
constexpr double kSpeedCommandRisePerCycleMps = 0.05; // 速度指令每周期（0.1 s）最大上升量，约 0.5 m/s^2，与车辆实际加速能力同量级。
constexpr char kModuleName[] = "UserController_PathPlanning"; // EchoSim 消息模块名称。
constexpr char kControlTopic[] = "echo.controller.userdefined"; // 控制消息发布主题。
constexpr char kStateTopic[] = "echo.vehicle.states.Ego"; // 车辆状态订阅主题。

// 将 SDK 车辆状态转换为算法使用的二维状态和米每秒速度。
// 单位换算（收发边界处唯一换算点）：
//   xo/yo  世界系位置，米，直接透传；
//   yaw    航向角，度 -> 弧度（kDegToRad）；
//   vx/vy  车体系速度分量（x 前向、y 左正），km/h -> m/s（kKphToMps），
//          注意 EchoSim 回报的是 km/h 而非 m/s；
//   avz    横摆角速度，度每秒 -> 弧度每秒。
VehicleState2D readVehicleState_(const sim_msg::VehicleState_SimpleCar& message)
{
    const auto& dynamics = message.car_state().vehdynamics();
    return {dynamics.xo(), dynamics.yo(), dynamics.yaw() * kDegToRad,
            dynamics.vx() * kKphToMps, dynamics.vy() * kKphToMps,
            dynamics.avz() * kDegToRad};
}

// 检查状态消息是否包含车辆动力学数据。
// 仿真起步或异常帧可能缺失 car_state/vehdynamics，缺字段帧直接丢弃。
bool hasVehicleDynamics_(const sim_msg::VehicleState_SimpleCar& message)
{
    return message.has_car_state() && message.car_state().has_vehdynamics();
}

// 将挡位配置限制到 SDK 合法枚举范围。
// 越界值一律回退到 DRIVE：默认场景为前进任务，非法挡位会导致车辆不动。
int gear_modeOrDrive_(int gear_mode)
{
    if (gear_mode >= static_cast<int>(sim_msg::Control_GEAR_MODE_NO_CONTROL)
        && gear_mode <= static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE))
        return gear_mode;
    return static_cast<int>(sim_msg::Control_GEAR_MODE_DRIVE);
}

// 构造距离/速度模式控制消息，横向控制使用目标前轮角。
// Pure Pursuit/Stanley 模式使用：SDK 按以下三个控制类型字段解释消息--
//   REQUEST_DISTANCE_AND_VELOCITY 纵向：给"距停车点距离 + 速度上限"，
//     SDK 自行决定加减速；
//   REQUEST_FRONT_WHEEL_ANGLE 横向：目标前轮角，此处直接传弧度
//     （SDK 该字段接受弧度，与方向盘角接口不同，无需度换算）；
//   BRAKE_TARGET_ACC_CONTROL 制动：距离耗尽后按目标减速度刹停。
// 输入速度为米每秒，下发前乘 kMpsToKph 转 km/h。
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
    // request_distance_2_stop 单位为米：剩余路径距离耗尽后 SDK 自动刹停，
    // 这层"距离刹停"与算法内的 distanceSpeedLimit 限速互为兜底。
    control.mutable_control_cmd()->set_max_velocity(
        std::max(0.0, command.target_speed_mps * kMpsToKph));
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造 LQR 加速度模式控制消息。
// LQR 模式纵向使用 TARGET_ACC_CONTROL：直接下发目标加速度
// （米每二次方秒，由纵向 P 控制算出，负值即减速/制动），
// 横向仍为 REQUEST_FRONT_WHEEL_ANGLE 目标前轮角（弧度）。
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
        command.front_wheel_angle_rad);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造停车控制消息。
// 纵向 TARGET_ACC_CONTROL 下发 -2.0 m/s^2 制动加速度（较急但不超过
// 最大减速度约束的收尾刹车），前轮角回正为 0，挡位仍由配置决定。
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
// 模块名 kModuleName 用于在 EchoSim 总线上标识本控制器进程。
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
    // 若外部未先调用 initialize()，这里兜底初始化消息总线。
    if (!sim_msg::IsInitialized())
        initialize();
    std::cout << "[control] message system initialized; waiting for vehicle state"
              << std::endl;

    // ---- 准备阶段：构建跟踪器、可视化器与消息收发端 ----
    // 跟踪器按 TaskConfig.tracking 生成（默认 Pure Pursuit），无逐帧状态。
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

    // 控制发布端与状态订阅端：主题见 kControlTopic / kStateTopic 常量。
    sim_msg::MessagePublisher<sim_msg::Control> control_publisher(kControlTopic);
    sim_msg::MessageSubscriber<sim_msg::VehicleState_SimpleCar> state_subscriber(kStateTopic);
    std::cout << "[control] publisher/subscriber created" << std::endl;
    double control_time_sec = 0.0; // 当前控制循环时间戳。
    std::size_t publish_count = 0; // 成功发布的跟踪控制帧计数。
    bool start_checked = false; // 是否已完成起点状态检查。
    bool stop_published = false; // 是否已发布终点停车指令。
    bool have_state = false; // 是否至少接收过一帧有效状态。
    std::size_t progress_index = 0; // 当前路径进度，只允许向前搜索。
    double last_target_speed_mps = 0.0; // 上一帧速度指令，用于上升斜率限制。
    // 首帧状态到来前持续发送安全停车控制，避免仿真端等待控制帧而不推进状态反馈。
    sim_msg::Control last_control; // 最近一次成功发布的控制消息。
    bool have_last_control = true; // 初始停车控制可以在状态到来前复用。

    // 首帧尚未收到车辆状态，不发布控制消息，行为与原 SDK 示例保持一致。
    have_last_control = false;

    std::size_t subscribe_miss_count = 0; // 未收到状态消息的轮询次数。

    // ---- 主控制循环：等状态 -> 读状态 -> 跟踪计算 -> 限速门控 ->
    // 构造控制消息 -> 发布 -> 可视化 -> 日志 -> 到点判定/停车 ----
    // 退出条件：总线出错（HasError）或已发布到点停车指令（stop_published）。
    std::cout << "[control] entering state-control loop" << std::endl;
    while (!sim_msg::HasError() && !stop_published)
    {
        // 阶段 1：等待状态。订阅超时（kLoopPeriodMs=100 ms）未收到消息时
        // 重发上一帧控制保持执行连续性，休眠一个周期后重试。
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
                // 复用上一帧成功发布的控制消息（仅刷新时间戳）：
                // 仿真端若长时间收不到控制帧可能停止推进状态反馈。
                last_control.mutable_header()->set_time_stamp(control_time_sec);
                publish_(control_publisher, last_control, "reused control");
            }
            sim_msg::SleepMS(kLoopPeriodMs);
            control_time_sec += kLoopDtSec;
            continue;
        }
        // 阶段 2：读取状态。缺动力学字段的消息帧直接丢弃等待下一帧。
        if (!hasVehicleDynamics_(state_message))
        {
            std::cerr << "[state] vehicle dynamics unavailable" << std::endl;
            continue;
        }

        // 单位换算后的算法状态（米/弧度/米每秒），世界系位姿 + 车体系速度。
        const VehicleState2D state = readVehicleState_(state_message);
        have_state = true;
        if (!start_checked)
            std::cout << "[state] first valid vehicle state received" << std::endl;
        subscribe_miss_count = 0;
        // 首帧起点一致性检查：实际初始位姿与配置起点偏差超容差时仅告警
        // 不中断（位置误差米、航向误差弧度，atan2 形式做角度归一），
        // 以仿真器摆好的实际位姿为准开始跟踪。
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

        // 阶段 3+4：跟踪计算与限速门控。calculate 内部完成横向转角求解，
        // 并按 turn/bend/curve/slide 与距离刹车逐级取小得到目标速度上限。
        // progress_index 用上一帧最近点做锚点，保证进度单调推进。
        TrackingCommand command = tracker.calculate(
            path, state, progress_index);
        progress_index = command.nearest_path_index;
        // 速度指令上升斜率限制：阶梯弯内路径短暂摆直时各速度门控会瞬时
        // 全开，指令从 1.0 一帧跳到 4.0 重新激励侧滑（实测弯中二次滑移
        // 侧偏角 18°）。减速不受限，保护性压速需立即生效。
        command.target_speed_mps = std::min(
            command.target_speed_mps,
            last_target_speed_mps + kSpeedCommandRisePerCycleMps);
        last_target_speed_mps = command.target_speed_mps;
        // 参考点信息供日志与可视化使用。
        const std::size_t nearest_index = command.nearest_path_index;
        const std::size_t target_index = command.target_path_index;
        const PathPoint& reference = path[target_index];
        // 实际车速取平面合成速度 hypot(vx, vy)（米每秒），含侧向分量，
        // 纵向 P 控制的速度误差用它衡量（侧滑时模长偏大，减速更坚决）。
        const double actual_speed = std::hypot(state.vx, state.vy);
        // 阶段 5：构造控制消息。LQR 走加速度模式（先算纵向 P 控制加速度，
        // 上下限为 ±max_acc/max_dec），其余算法走距离/速度模式。
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
        // 阶段 6：发布控制帧。成功则缓存为 last_control 供超时重发，
        // 失败仅记日志不终止（等待下一帧通信恢复）。
        const bool control_published = publish_(control_publisher, control,
                                                "tracking control");
        if (control_published)
        {
            last_control = control;
            have_last_control = true;
            ++publish_count;
        }
        // 阶段 7：可视化。更新轨迹窗口并处理窗口消息泵。
        visualizer.update(state, target_index, control_time_sec);
        visualizer.pumpWindow();

        // 阶段 8：周期日志。按 control_log_interval 间隔输出一帧关键状态：
        // 最近点/目标点索引、参考点与实际位姿（米/弧度）、
        // 参考控制（速度 m/s、前轮角换算为度便于人工核对）与实际车速。
        if (control_published && config.control_log_interval > 0
            && publish_count % config.control_log_interval == 0)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[track] publish_count=" << publish_count
                      << " nearest_index=" << nearest_index
                      << " target_index=" << target_index
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

        // 阶段 9：到点判定/停车。TrackingCommand.reached_goal 为真
        // （位置与航向均在容差内）时发布 -2.0 m/s^2 停车指令并置
        // stop_published 结束循环；停车帧发布失败视为致命错误直接抛出。
        if (command.reached_goal)
        {
            const sim_msg::Control stop = buildStop_(
                control_time_sec + kLoopDtSec, config.gear_mode);
            if (!publish_(control_publisher, stop, "goal stop"))
                throw std::runtime_error("goal stop publish failed");
            stop_published = true;
            std::cout << "[goal] position and yaw reached; vehicle stopped" << std::endl;
        }
        // 本帧结束：推进控制时间戳并休眠一个仿真步长（0.1 s）。
        control_time_sec += kLoopDtSec;
        sim_msg::SleepMS(kLoopPeriodMs);
    }

    // ---- 收尾：异常退出（未到点）也补发停车指令防止车辆失控滑行，
    // 保存最终轨迹图、退订并关闭消息总线。 ----
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
