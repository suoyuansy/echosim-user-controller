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
constexpr double kLoopDtSec = 0.1; // 兜底控制步长：状态消息缺少仿真时间源时使用，单位为秒。
constexpr int kLoopPeriodMs = 100; // 状态订阅超时和循环休眠时间（墙钟），仅作节奏控制，单位为毫秒。
constexpr double kPi = 3.14159265358979323846; // 圆周率常量。
constexpr double kDegToRad = kPi / 180.0; // 角度转弧度的比例系数。
constexpr double kRadToDeg = 180.0 / kPi; // 弧度转角度的比例系数。
constexpr double kMpsToKph = 3.6; // SDK 距离/速度模式的 max_velocity 使用千米每小时。
constexpr double kKphToMps = 1.0 / 3.6; // EchoSim vx/vy 使用千米每小时（车体系分量：x 前向、y 左正）。
// 速度指令上升斜率限制，单位米每二次方秒。按真实仿真步长 dt 换算成每帧
// 允许的上升量：渲染慢于实时时（墙钟 100 ms 仅对应仿真约 14 ms），固定
// “每周期 0.05 m/s” 的旧口径会把实际加速度放大数倍，这里直接以加速度计。
constexpr double kSpeedCommandRiseAccelMps2 = 0.5; // 与车辆实际加速能力同量级。
// 速度指令下降斜率限制，单位米每二次方秒（2026-08-23 转角修复）。此前下降
// 不受限：弯道入口指令从 4.0 一帧跳到 1.0，SDK 急刹产生 10.5 m/s² 加速度
// 尖峰（评分 max_lat_acc 红线 3.5，Run 20260823_125403 t=45.4）。仅当实际
// 车速高于 2.0 m/s 时限斜率：低速段（侧滑/弯道保护性压速）仍瞬时生效，
// 不牺牲保护响应。
constexpr double kSpeedCommandFallAccelMps2 = 2.0; // 指令下降斜率上限。
constexpr double kFallRateLimitMinActualSpeedMps = 2.0; // 下降斜率限制生效的最低实际车速。
// 倒溜保护：前进挡下车辆沿坡道后溜（vx < -0.1）累计超过 0.3 s 后，
// 非终点捕获状态直接施加正向驱动力，并显式清除制动压力。禁止先锁轮制动，
// 避免车辆在低附着坡面变成“雪橇”并继续加速后滑；恢复前进后交回跟踪器。
constexpr double kRollbackSpeedThresholdMps = -0.1; // 判定后溜的前向速度阈值。
constexpr double kRollbackTriggerDurationSec = 0.3; // 触发无制动反推的累计后溜时长（仿真秒）。
constexpr double kRollbackReleaseSpeedMps = -0.05; // 终点捕获滑出后判定仍在后溜的阈值。
constexpr double kRollbackThrustExitSpeedMps = 0.2; // 反推恢复前进后退出反推的车速阈值（米/秒）。
constexpr double kRollbackThrustAccelMps2 = 0.8; // 反推目标加速度（米每二次方秒）。
// 终点滞留蠕动：常规加速度 P 控制若仍在低速坡面克服不了静摩擦，连续滞留
// 2 s 后改发更强的固定正加速度，恢复前进或到点后退出。
constexpr double kGoalCreepStallSpeedMps = 0.05; // 判定滞留的车速上限（米/秒）。
constexpr double kGoalCreepTriggerSec = 2.0; // 触发蠕动的持续滞留时长（仿真秒）。
constexpr double kGoalCreepAccelMps2 = 0.6; // 蠕动目标正加速度（米每二次方秒）。
constexpr double kGoalCreepExitSpeedMps = 0.8; // 退出蠕动恢复正常控制的车速（米/秒）。
// 终点坡道驻车辅助：Moon2 终点存在约 3.3° 坡度，持续目标减速度会锁轮并以
// 约 -0.16 m/s 向后滑。捕获圈内用带迟滞的正加速度反推把前向速度约束在
// ±0.05 m/s 附近。仅速度迟滞会使每次反推积累微小前移（Run
// 20260824_231419 在圈内停留 107 s 后漂出），因此再用目标在车体
// 前向轴上的投影做位置迟滞：目标足够靠前才允许反推，已驶过目标
// 则保持制动、利用坡道自然回滑。
constexpr double kGoalHoldThrustEnterSpeedMps = -0.05;
constexpr double kGoalHoldThrustExitSpeedMps = 0.05;
constexpr double kGoalHoldThrustEnterPositionM = 0.20; // 目标在车前超过此距离才允许反推。
constexpr double kGoalHoldThrustExitPositionM = 0.05; // 接近目标中心后提前停止反推。
// 仿真时间源单帧差分的防御性上限（秒）：状态时间戳跳变/重启时钳制单帧 dt，
// 避免斜率限制被一次性放大。
constexpr double kMaxSimDtSec = 1.0;
constexpr char kModuleName[] = "UserController_PathPlanning"; // EchoSim 消息模块名称。
constexpr char kControlTopic[] = "echo.controller.userdefined"; // 控制消息发布主题。
constexpr char kStateTopic[] = "echo.vehicle.states.Ego"; // 车辆状态订阅主题。

// 将 SDK 车辆状态转换为算法使用的状态和米每秒速度。
// 单位换算（收发边界处唯一换算点）：
//   xo/yo/zo 世界系位置，米，直接透传；
//   yaw    航向角，度 -> 弧度（kDegToRad）；
//   vx/vy  车体系速度分量（x 前向、y 左正），km/h -> m/s（kKphToMps），
//          注意 EchoSim 回报的是 km/h 而非 m/s；
//   avz    横摆角速度，度每秒 -> 弧度每秒。
VehicleState2D readVehicleState_(const sim_msg::VehicleState_SimpleCar& message)
{
    const auto& dynamics = message.car_state().vehdynamics();
    return {dynamics.xo(), dynamics.yo(), dynamics.zo(),
            dynamics.yaw() * kDegToRad,
            dynamics.vx() * kKphToMps, dynamics.vy() * kKphToMps,
            dynamics.avz() * kDegToRad};
}

// 检查状态消息是否包含车辆动力学数据。
// 仿真起步或异常帧可能缺失 car_state/vehdynamics，缺字段帧直接丢弃。
bool hasVehicleDynamics_(const sim_msg::VehicleState_SimpleCar& message)
{
    return message.has_car_state() && message.car_state().has_vehdynamics();
}

// 解析状态消息对应的仿真时间，单位为秒。
// 优先取消息自带的顶层 time_stamp 字段（秒）；缺失时回退 SDK Subscribe
// 返回的原始时间戳 message_time——其单位未在文档中约定，用相邻帧差分
// 自动定标（仿真步长 10 ms 量级，按差分大小归入纳秒/微秒/毫秒三档）。
// 返回 0 表示本帧拿不到有效仿真时间（首帧未定标或字段未填充）。
// last_raw_time/raw_time_scale 为跨帧定标状态，由调用方持有。
double readSimTimeSec_(const sim_msg::VehicleState_SimpleCar& message,
                       long long message_time,
                       long long& last_raw_time,
                       double& raw_time_scale)
{
    // 首选：消息自带时间戳（仿真侧权威时间，单位秒）。
    const double stamped = message.time_stamp();
    if (stamped > 0.0)
        return stamped;

    // 兜底：SDK 原始时间戳差分定标。
    if (message_time <= 0)
        return 0.0;
    if (raw_time_scale <= 0.0)
    {
        if (last_raw_time > 0 && message_time > last_raw_time)
        {
            const long long delta = message_time - last_raw_time;
            if (delta >= 1000000)
                raw_time_scale = 1e-9; // 纳秒
            else if (delta >= 10000)
                raw_time_scale = 1e-6; // 微秒
            else
                raw_time_scale = 1e-3; // 毫秒
        }
        last_raw_time = message_time;
        if (raw_time_scale <= 0.0)
            return 0.0; // 首帧或时间戳未推进，暂无法定标。
    }
    return message_time * raw_time_scale;
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

// Pure Pursuit/Stanley 使用 SDK 距离/速度模式：控制器提供剩余停车距离与
// 速度上限，SDK 自行调节驱动和制动。算法内部速度单位为 m/s，max_velocity
// 按 SDK 接口要求转换为 km/h；前轮角保持弧度。
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
    control.mutable_control_cmd()->set_max_velocity(
        std::max(0.0, command.target_speed_mps * kMpsToKph));
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造侧滑滑行控制：显式请求零油门和零制动压力，避免 SDK 距离/速度模式
// 为追踪侧滑速度上限主动制动。不能使用 ACCEL_NO_CONTROL/BRAKE_NO_CONTROL，
// 因为 NO_CONTROL 不会清除上一帧可能锁存的油门或制动请求。
sim_msg::Control buildCoast_(const TrackingCommand& command,
                             double timestamp_sec,
                             int gear_mode)
{
    sim_msg::Control control;
    sim_msg::InitControl(control);
    control.mutable_header()->set_time_stamp(timestamp_sec);
    control.mutable_control_type()->set_acc_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_THROTTLE);
    control.mutable_control_type()->set_brake_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_BRAKE_PRESSURE_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_throttle(0.0);
    control.mutable_control_cmd()->set_request_brake_pressure(0.0);
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// LQR、倒溜反推与终点蠕动使用显式加速度控制。正/零加速度时显式清除
// 制动压力，避免旧制动帧锁存后吞掉驱动力请求。
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
    // protobuf 枚举默认值 0 正好是 BRAKE_TARGET_ACC_CONTROL，因此正/零
    // 加速度不能省略制动字段。BRAKE_NO_CONTROL 只是不更新制动器，无法
    // 清除上一帧锁存的制动压力；必须显式请求 0 制动压力。
    if (acceleration_mps2 < 0.0)
    {
        control.mutable_control_type()->set_brake_control_type(
            sim_msg::Control::CONTROL_TYPE::BRAKE_TARGET_ACC_CONTROL);
    }
    else
    {
        control.mutable_control_type()->set_brake_control_type(
            sim_msg::Control::CONTROL_TYPE::REQUEST_BRAKE_PRESSURE_CONTROL);
        control.mutable_control_cmd()->set_request_brake_pressure(0.0);
    }
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_acc(acceleration_mps2);
    control.mutable_control_cmd()->set_request_front_wheel_angle(
        command.front_wheel_angle_rad);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造保持制动控制消息：倒溜保护期间使用。
// 与收尾停车的差异仅在于减速度可调（倒溜保护用 -3.0 m/s^2）：实测坡道上
// -2.0 的制动从 -3.65 m/s 停车耗时约 20 s（Run 20260823_180725 t=480-500），
// 加急制动缩短溜车距离；前轮同样回正（倒溜中大舵角刮擦阻力大且方向反常）。
sim_msg::Control buildBrakeHold_(double timestamp_sec, int gear_mode,
                                 double acceleration_mps2)
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
    control.mutable_control_cmd()->set_request_front_wheel_angle(0.0);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造倒溜反推控制消息：保持制动无效（锁轮仍加速后滑）时使用。
// 纵向 TARGET_ACC 正加速度直接向电机要扭矩对抗坡道重力，前轮回正把轮胎
// 刮擦阻力降到最低（实测同坡段 2.8° 舵角 + 0.8 油门即可从静止爬起，
// Run 20260823_194908 t=572-588）。显式请求 0 制动压力，以清除之前制动帧
// 锁存的制动压力；BRAKE_NO_CONTROL 仅停止更新，不能保证松开制动器。
sim_msg::Control buildRollbackThrust_(double timestamp_sec, int gear_mode,
                                      double acceleration_mps2)
{
    sim_msg::Control control;
    sim_msg::InitControl(control);
    control.mutable_header()->set_time_stamp(timestamp_sec);
    control.mutable_control_type()->set_acc_control_type(
        sim_msg::Control::CONTROL_TYPE::TARGET_ACC_CONTROL);
    control.mutable_control_type()->set_brake_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_BRAKE_PRESSURE_CONTROL);
    control.mutable_control_type()->set_steer_control_type(
        sim_msg::Control::CONTROL_TYPE::REQUEST_FRONT_WHEEL_ANGLE);
    control.mutable_control_cmd()->set_request_acc(acceleration_mps2);
    control.mutable_control_cmd()->set_request_brake_pressure(0.0);
    control.mutable_control_cmd()->set_request_front_wheel_angle(0.0);
    control.set_gear_cmd(static_cast<sim_msg::Control_GEAR_MODE>(
        gear_modeOrDrive_(gear_mode)));
    return control;
}

// 构造停车控制消息。
// 纵向 TARGET_ACC_CONTROL 下发 -2.0 m/s^2 制动加速度（较急但不超过
// 最大减速度约束的收尾刹车），前轮角回正为 0，挡位仍由配置决定。
sim_msg::Control buildStop_(double timestamp_sec, int gear_mode)
{
    return buildBrakeHold_(timestamp_sec, gear_mode, -2.0);
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
    double control_time_sec = 0.0; // 当前控制循环时间戳（跟随仿真时间，秒）。
    double last_sim_time_sec = 0.0; // 上一帧状态消息的仿真时间，秒。
    bool have_sim_time = false; // 是否已获得有效仿真时间源。
    double last_dt_sec = kLoopDtSec; // 上一帧实际仿真步长，时间源缺失时回退固定步长。
    bool sim_time_warned = false; // 时间源缺失告警是否已输出（只告警一次）。
    long long last_raw_time = 0; // 上一帧 SDK 原始时间戳（单位未知，用于差分定标）。
    double raw_time_scale = 0.0; // SDK 原始时间戳换算到秒的比例，0 表示未定标。
    std::size_t publish_count = 0; // 成功发布的跟踪控制帧计数。
    bool start_checked = false; // 是否已完成起点状态检查。
    // 完成状态只用于锁存“已满足驻留时长”并避免重复发布完成日志。
    // 不能用它退出控制循环：任务运行器不会因用户控制器返回而终止仿真，
    // Moon2 坡道上若失去持续驻车辅助，车辆会在数秒内滑出目标圈。
    bool goal_hold_complete = false;
    bool have_state = false; // 是否至少接收过一帧有效状态。
    std::size_t progress_index = 0; // 当前路径进度，只允许向前搜索。
    double last_target_speed_mps = 0.0; // 上一帧速度指令，用于上升斜率限制。
    // 倒溜保护状态：rollback_duration_sec 按仿真时间累计后溜时长；触发后
    // 直接进入无制动反推，避免普通路段锁轮后继续侧滑/后滑。
    double rollback_duration_sec = 0.0; // 连续后溜累计时长（仿真秒）。
    bool rollback_thrust = false; // 是否处于无制动倒溜反推状态。
    // 终点滞留蠕动状态：goal_stall_duration_sec 累计低速滞留时长，
    // goal_creep 置位期间改发 TARGET_ACC 小加速度帧推动车辆重新起步。
    double goal_stall_duration_sec = 0.0; // 连续滞留累计时长（仿真秒）。
    bool goal_creep = false; // 是否处于终点滞留蠕动状态。
    // 终点捕获状态：第一次满足位置（及可选航向）容差后锁存制动。车辆若因
    // 入圈速度过高最终停在圈外，则解除锁存并让跟踪器重新靠近；只有圈内低速
    // 连续保持 goal_hold_duration_sec 后才判定控制任务完成。
    bool goal_brake_active = false; // 是否正在持续发布终点制动帧。
    bool goal_hold_thrust = false; // 终点捕获期间是否用正加速度抵消坡道后溜。
    double goal_stationary_hold_sec = 0.0; // 圈内停稳后的连续保持时间（仿真秒）。
    // 首帧状态到来前持续发送安全停车控制，避免仿真端等待控制帧而不推进状态反馈。
    sim_msg::Control last_control; // 最近一次成功发布的控制消息。
    bool have_last_control = true; // 初始停车控制可以在状态到来前复用。

    // 首帧尚未收到车辆状态，不发布控制消息，行为与原 SDK 示例保持一致。
    have_last_control = false;

    std::size_t subscribe_miss_count = 0; // 未收到状态消息的轮询次数。

    // ---- 主控制循环：等状态 -> 读状态 -> 跟踪计算 -> 限速门控 ->
    // 构造控制消息 -> 发布 -> 可视化 -> 日志 -> 到点判定/停车 ----
    // 退出条件只是总线结束/出错（HasError）。终点圈内完成驻留后仍
    // 继续发布坡道驻车控制，直到仿真运行器真正结束本次任务。
    std::cout << "[control] entering state-control loop" << std::endl;
    while (!sim_msg::HasError())
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
            // 超时期间仿真未推进，控制时间戳保持不变。
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

        // 仿真时间基准：以状态消息自带的仿真时间为准计算本帧真实步长 dt。
        // 渲染慢于实时时（实测墙钟可为仿真时间的 4~10 倍），墙钟休眠不再
        // 扭曲控制周期--时间戳与斜率限制全部对齐物理时间。
        const double sim_time_sec = readSimTimeSec_(state_message, message_time,
                                                    last_raw_time, raw_time_scale);
        double dt_sec = 0.0; // 本帧仿真步长，默认 0（时间未推进不放开斜率）。
        if (sim_time_sec > 0.0)
        {
            if (!have_sim_time)
            {
                have_sim_time = true;
                last_sim_time_sec = sim_time_sec;
            }
            else if (sim_time_sec > last_sim_time_sec)
            {
                dt_sec = std::min(sim_time_sec - last_sim_time_sec, kMaxSimDtSec);
                last_sim_time_sec = sim_time_sec;
            }
            // 时间回退/重复帧：dt 保持 0，时间戳不推进。
            control_time_sec = sim_time_sec;
        }
        else if (!have_sim_time)
        {
            // 两种时间源均不可用：退回固定步长口径并一次性告警。
            if (!sim_time_warned)
            {
                sim_time_warned = true;
                std::cerr << "[state] sim time unavailable; falling back to fixed "
                          << kLoopDtSec << "s control step" << std::endl;
            }
            dt_sec = kLoopDtSec;
            control_time_sec += kLoopDtSec;
        }
        last_dt_sec = dt_sec;

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

        // 终点捕获优先于倒溜恢复与常规跟踪。竞赛任务的完成条件要求车辆在
        // 1 m 目标圈内保持 3 s；旧逻辑仅在“位置+占位 yaw”同时满足时发送
        // 一帧制动后立即退出，Test6 因此以 3.48 m/s 穿过目标圈。这里直接按
        // 配置目标坐标计算三维位置误差，并在可选航向条件满足后锁存持续
        // 制动。评测器 distance_to_goal 使用 XYZ 欧氏距离；只用 XY 会因约
        // 0.35 m 的目标高度差提前约 1.7 s 完成驻留计时（Run 003445）。
        const double actual_speed = std::hypot(state.vx, state.vy);
        const double horizontal_distance_to_goal = std::hypot(
            config.goal.x - state.x, config.goal.y - state.y);
        const double distance_to_goal = std::hypot(
            horizontal_distance_to_goal, config.goal_z - state.z);
        const double goal_yaw_error = std::abs(std::atan2(
            std::sin(config.goal.yaw - state.yaw),
            std::cos(config.goal.yaw - state.yaw)));
        // 目标位置在车体前向轴上的有符号投影：正值表示目标在前，
        // 负值表示车辆已越过目标。驻车辅助用它消除速度迟滞的单向漂移。
        const double goal_longitudinal_error_m =
            (config.goal.x - state.x) * std::cos(state.yaw)
            + (config.goal.y - state.y) * std::sin(state.yaw);
        const bool inside_goal =
            distance_to_goal <= config.tracking.goal_position_tolerance_m
            && (!config.tracking.require_goal_yaw
                || goal_yaw_error <= config.tracking.goal_yaw_tolerance_rad);
        // 目标前后位置迟滞仅在终点捕获区内启用。途中（包括接近终点但
        // 尚未进入容差圈的阶段）只执行常规跟踪/减速，不使用目标前后投影
        // 约束，避免它干扰正常路径跟踪。
        const bool goal_position_hysteresis_active = inside_goal;
        if (inside_goal && !goal_brake_active && !rollback_thrust)
        {
            goal_brake_active = true;
            goal_hold_thrust = false;
            goal_stationary_hold_sec = 0.0;
            goal_creep = false;
            goal_stall_duration_sec = 0.0;
            rollback_thrust = false;
            rollback_duration_sec = 0.0;
            std::cout << std::fixed << std::setprecision(3)
                      << "[goal] capture entered: distance=" << distance_to_goal
                      << " m speed=" << actual_speed
                      << " m/s; holding brake" << std::endl;
        }
        if (goal_brake_active)
        {
            // 若已滑出捕获圈，立即解除捕获锁存。后溜时直接切入正加速度
            // 反推，恢复前进后再由跟踪器重新靠近；禁止在反推尚未结束时
            // 因短暂穿过目标圈而重新锁上制动。
            if (!inside_goal)
            {
                goal_brake_active = false;
                goal_hold_thrust = false;
                goal_stationary_hold_sec = 0.0;
                // 完成后若仍滑出容差圈，当前驻车条件已不成立。清除
                // 锁存并重新接近，下次入圈后必须再完整驻留一次。
                goal_hold_complete = false;
                if (state.vx < kRollbackReleaseSpeedMps)
                {
                    rollback_thrust = true;
                    rollback_duration_sec = 0.0;
                    std::cout << "[goal] capture slid outside at distance "
                              << distance_to_goal
                              << " m; thrusting forward to retry" << std::endl;
                }
                else
                {
                    std::cout << "[goal] capture left tolerance at distance "
                              << distance_to_goal
                              << " m; retrying approach" << std::endl;
                }
            }
            else
            {
                // 圈内驻车辅助采用位置+速度双迟滞：只有目标仍在车前
                // 且已开始后溜时才释放制动、施加正加速度。恢复微小前进
                // 速度，或者已接近/越过目标中心时重新制动。速度阈值低于
                // 任务 stopped_speed，辅助期间可连续累计驻留时间。
                if (!goal_hold_thrust
                    && state.vx <= kGoalHoldThrustEnterSpeedMps
                    && goal_position_hysteresis_active
                    && goal_longitudinal_error_m
                        >= kGoalHoldThrustEnterPositionM)
                {
                    goal_hold_thrust = true;
                    std::cout << "[goal] brake hold sliding backward at "
                              << state.vx
                              << " m/s with longitudinal goal error "
                              << goal_longitudinal_error_m
                              << " m; applying hill-hold thrust" << std::endl;
                }
                else if (goal_hold_thrust
                         && (state.vx >= kGoalHoldThrustExitSpeedMps
                             || !goal_position_hysteresis_active
                             || goal_longitudinal_error_m
                                <= kGoalHoldThrustExitPositionM))
                {
                    goal_hold_thrust = false;
                    std::cout << "[goal] hill-hold thrust restored forward speed; "
                                 "longitudinal goal error="
                              << goal_longitudinal_error_m
                              << " m; braking again" << std::endl;
                }

                const sim_msg::Control hold_control = goal_hold_thrust
                    ? buildRollbackThrust_(
                        control_time_sec + last_dt_sec, config.gear_mode,
                        kGoalCreepAccelMps2)
                    : buildStop_(control_time_sec + last_dt_sec,
                                 config.gear_mode);
                const char* hold_label = goal_hold_thrust
                    ? "goal hill-hold thrust"
                    : "goal brake hold";
                if (!publish_(control_publisher, hold_control, hold_label))
                    throw std::runtime_error("goal hold publish failed");
                last_control = hold_control;
                have_last_control = true;

                if (actual_speed <= config.tracking.goal_stopped_speed_mps)
                    goal_stationary_hold_sec += dt_sec;
                else
                    goal_stationary_hold_sec = 0.0;

                if (!goal_hold_complete
                    && goal_stationary_hold_sec
                    >= config.tracking.goal_hold_duration_sec)
                {
                    // 完成可能恰好发生在驻车辅助反推帧；锁存完成时
                    // 无条件覆盖为制动帧，避免这一帧遗留正加速度请求。
                    const sim_msg::Control final_stop = buildStop_(
                        control_time_sec + 2.0 * last_dt_sec,
                        config.gear_mode);
                    if (!publish_(control_publisher, final_stop,
                                  "goal completion stop"))
                    {
                        throw std::runtime_error(
                            "goal completion stop publish failed");
                    }
                    last_control = final_stop;
                    have_last_control = true;
                    goal_hold_thrust = false;
                    goal_hold_complete = true;
                    std::cout << "[goal] stopped inside tolerance for "
                              << goal_stationary_hold_sec
                              << " s; goal hold complete; maintaining parking control"
                              << std::endl;
                }
            }

            if (goal_brake_active)
            {
                visualizer.update(state, progress_index, control_time_sec);
                visualizer.pumpWindow();
                sim_msg::SleepMS(kLoopPeriodMs);
                continue;
            }
        }

        // 非终点倒溜恢复：持续后溜达到阈值后直接发 TARGET_ACC 正加速度，
        // 同时显式请求 0 制动压力。整个恢复过程不下发制动，前轮回正以减小
        // 轮胎刮擦阻力；vx 恢复到 +0.2 m/s 后再交回正常路径跟踪。
        if (rollback_thrust)
        {
            if (state.vx >= kRollbackThrustExitSpeedMps)
            {
                rollback_thrust = false;
                rollback_duration_sec = 0.0;
                std::cout << "[recover] rollback thrust regained forward motion; "
                             "resuming tracking" << std::endl;
            }
        }
        else
        {
            if (state.vx < kRollbackSpeedThresholdMps)
                rollback_duration_sec += dt_sec;
            else
                rollback_duration_sec = 0.0;
            if (rollback_duration_sec > kRollbackTriggerDurationSec)
            {
                rollback_thrust = true;
                std::cout << "[recover] rollback detected: applying brake-free forward thrust"
                          << std::endl;
            }
        }

        if (rollback_thrust)
        {
            // 无制动反推帧（TARGET_ACC +0.8 m/s^2 + 制动压力 0 + 前轮回正）。
            const sim_msg::Control thrust_control = buildRollbackThrust_(
                control_time_sec + last_dt_sec, config.gear_mode,
                kRollbackThrustAccelMps2);
            if (publish_(control_publisher, thrust_control, "rollback thrust"))
            {
                last_control = thrust_control;
                have_last_control = true;
            }
            visualizer.update(state, progress_index, control_time_sec);
            visualizer.pumpWindow();
            sim_msg::SleepMS(kLoopPeriodMs);
            continue;
        }

        // 阶段 3+4：跟踪计算与限速门控。calculate 内部完成横向转角求解，
        // 并按 turn/bend/curve/可选 slide 与距离刹车逐级取小得到目标速度上限。
        // progress_index 用上一帧最近点做锚点，保证进度单调推进。
        TrackingCommand command = tracker.calculate(
            path, state, progress_index);
        progress_index = command.nearest_path_index;
        // PathTracker 的几何路径仍是二维，因此其 reached_goal/distance 字段
        // 初值按 XY 计算。运行时终点语义必须覆盖为与评测器一致的 XYZ 距离。
        // 若二维已到而三维尚未到，PathTracker 会把目标速度置零；这里恢复
        // d/t 的低速靠近指令，继续缩小可控的水平误差直到进入三维目标球。
        const bool planar_goal_reached = command.reached_goal;
        command.reached_goal = inside_goal;
        command.distance_to_goal_m = distance_to_goal;
        if (planar_goal_reached && !inside_goal
            && config.tracking.goal_approach_time_sec > 0.0)
        {
            command.target_speed_mps = std::max(
                command.target_speed_mps,
                distance_to_goal / config.tracking.goal_approach_time_sec);
        }
        // actual_speed 已在终点捕获检查前计算；纵向 P 控制使用平面合成速度
        // hypot(vx, vy)，侧滑时模长偏大，减速会更坚决。
        // 终点接近速度上限 v=d/t：默认从 d=4 m/s*5 s=20 m 开始连续降速，
        // 到 1 m 圈边界时目标速度仅 0.2 m/s。它在运行时最后叠加，可覆盖
        // 最近点已跳到路径末端时的 2 m/s 活性地板，防止再次高速穿圈。
        if (config.tracking.goal_approach_time_sec > 0.0)
        {
            command.target_speed_mps = std::min(
                command.target_speed_mps,
                command.distance_to_goal_m
                    / config.tracking.goal_approach_time_sec);
        }
        // 侧滑速度约束：只有显式启用时才在全部路径/终点速度修正之后执行。
        // 关闭时仍保留 command.sliding 诊断状态，但不改变目标速度。
        if (config.tracking.enable_sideslip_speed_limit && command.sliding)
        {
            command.target_speed_mps = std::min(
                command.target_speed_mps,
                config.tracking.pure_pursuit_slide_speed_mps);
        }
        // 速度指令上升斜率限制：阶梯弯内路径短暂摆直时各速度门控会瞬时
        // 全开，指令从 1.0 一帧跳到 4.0 重新激励侧滑（实测弯中二次滑移
        // 侧偏角 18°）。斜率以加速度计，按本帧真实仿真步长 dt 换算；
        // 减速不受限，保护性压速需立即生效。
        command.target_speed_mps = std::min(
            command.target_speed_mps,
            last_target_speed_mps + kSpeedCommandRiseAccelMps2 * dt_sec);
        // 下降斜率限制（仅实际车速高于 2.0 m/s 时生效）：弯道入口指令
        // 一帧跳降会触发 SDK 急刹，产生超评分红线的加速度尖峰；低速段
        // 保护性压速仍瞬时下降，不牺牲侧滑/弯道保护响应。
        if (!(config.tracking.enable_sideslip_speed_limit && command.sliding)
            && actual_speed > kFallRateLimitMinActualSpeedMps)
        {
            command.target_speed_mps = std::max(
                command.target_speed_mps,
                last_target_speed_mps - kSpeedCommandFallAccelMps2 * dt_sec);
        }
        last_target_speed_mps = command.target_speed_mps;
        // 可选低速舵角上限；当前默认关闭。启用时，|vx| 低于 cap_speed
        // 钳到 cap，到 full_speed 线性放开到全舵；使用 |vx| 使倒溜时也生效。
        if (config.tracking.enable_low_speed_steer_limit)
        {
            const double forward_abs = std::abs(state.vx);
            const double& cap_speed =
                config.tracking.low_speed_steer_cap_speed_mps;
            const double& full_speed =
                config.tracking.low_speed_steer_full_speed_mps;
            double steer_limit = config.tracking.geometry
                .max_front_wheel_angle_rad;
            if (forward_abs < full_speed)
            {
                // cap_speed 以下取 cap，cap_speed 到 full_speed 之间线性过渡。
                const double blend = forward_abs <= cap_speed
                    ? 0.0
                    : (forward_abs - cap_speed) / std::max(full_speed - cap_speed,
                                                           1e-9);
                steer_limit = config.tracking.low_speed_steer_cap_rad
                    + blend * (steer_limit
                               - config.tracking.low_speed_steer_cap_rad);
            }
            command.front_wheel_angle_rad = std::clamp(
                command.front_wheel_angle_rad, -steer_limit, steer_limit);
        }
        // 参考点信息供日志与可视化使用。
        const std::size_t nearest_index = command.nearest_path_index;
        const std::size_t target_index = command.target_path_index;
        const PathPoint& reference = path[target_index];
        // 终点滞留蠕动检测：未到点、车速近零且距终点仍超容差并持续 2 s，
        // 判定常规加速度 P 输出仍未克服坡面静摩擦，置位 goal_creep 改发
        // +0.6 m/s² 固定正加速度。判距用车体-终点直线距离而非剩余折线距离：
        // 越过终点后剩余折线距离已归零，按它判定会漏检。
        // 到点或恢复前进（车速超退出阈值）即退出蠕动，恢复正常控制；
        // 若蠕动又触发倒溜则由倒溜保护接管。
        if (!goal_creep)
        {
            if (actual_speed < kGoalCreepStallSpeedMps)
                goal_stall_duration_sec += dt_sec;
            else
                goal_stall_duration_sec = 0.0;
            if (goal_stall_duration_sec > kGoalCreepTriggerSec
                && !command.reached_goal
                && command.distance_to_goal_m
                       > config.tracking.goal_position_tolerance_m)
            {
                goal_creep = true;
                std::cout << "[goal] stalled short of goal (remaining "
                          << command.remaining_path_distance_m
                          << " m); creeping" << std::endl;
            }
        }
        else if (command.reached_goal)
        {
            goal_creep = false;
            goal_stall_duration_sec = 0.0;
            std::cout << "[goal] creep reached goal tolerance" << std::endl;
        }
        else if (actual_speed > kGoalCreepExitSpeedMps)
        {
            goal_creep = false;
            goal_stall_duration_sec = 0.0;
            std::cout << "[goal] creep restored motion; resuming normal control"
                      << std::endl;
        }

        // 阶段 5：Pure Pursuit/Stanley 通常使用 SDK 距离/速度模式；启用侧滑
        // 限速且实际平面速度仍高于侧滑上限时，改发零油门、零制动滑行帧，
        // 避免 SDK 为快速追踪限速而制动并在坡面穿过零速。LQR保留显式加速度。
        double requested_acceleration_mps2 = 0.0;
        bool sdk_speed_mode = false;
        bool coast_mode = false;
        sim_msg::Control control;
        if (goal_creep)
        {
            // 蠕动帧：临时切换 TARGET_ACC 固定正加速度推动车辆重新起步。
            requested_acceleration_mps2 = kGoalCreepAccelMps2;
            control = buildAcceleration_(command, requested_acceleration_mps2,
                                         control_time_sec, config.gear_mode);
        }
        else if (config.tracking.enable_sideslip_speed_limit
                 && command.sliding
                 && actual_speed
                    > config.tracking.pure_pursuit_slide_speed_mps)
        {
            coast_mode = true;
            control = buildCoast_(command, control_time_sec,
                                  config.gear_mode);
        }
        else if (config.tracking.method == TrackerMethod::Lqr)
        {
            requested_acceleration_mps2 = LongitudinalController::speedP(
                command.target_speed_mps, state.vx,
                config.tracking.longitudinal_accel_kp,
                -config.tracking.max_deceleration_mps2,
                config.tracking.max_acceleration_mps2);
            control = buildAcceleration_(command, requested_acceleration_mps2,
                                         control_time_sec, config.gear_mode);
        }
        else
        {
            sdk_speed_mode = true;
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
        // 参考控制（速度 m/s、加速度 m/s²、前轮角换算为度）与实际车速。
        if (control_published && config.control_log_interval > 0
            && publish_count % config.control_log_interval == 0)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "[track] publish_count=" << publish_count
                      << " sim_t=" << control_time_sec
                      << " dt=" << dt_sec
                      << " nearest_index=" << nearest_index
                      << " target_index=" << target_index
                      << " reference=(x=" << reference.x
                      << ",y=" << reference.y
                      << ",yaw_rad=" << reference.yaw << ')'
                      << " actual=(x=" << state.x
                      << ",y=" << state.y
                      << ",yaw_rad=" << state.yaw << ')'
                      << " reference_control=(speed_mps=" << command.target_speed_mps
                      << ",mode=" << (coast_mode
                                           ? "coast"
                                           : (sdk_speed_mode
                                                  ? "sdk_speed"
                                                  : "target_acc"))
                      << ",sliding=" << (command.sliding ? "yes" : "no")
                      << ",accel_mps2=" << requested_acceleration_mps2
                      << ",front_wheel_deg="
                      << command.front_wheel_angle_rad * kRadToDeg << ')'
                      << " actual_speed_mps=" << actual_speed
                      << std::endl;
        }

        // 本帧结束：仅按墙钟休眠做节奏控制，控制时间戳由下一帧状态消息的
        // 仿真时间驱动（慢于实时渲染时墙钟与仿真时间不再绑定）。
        sim_msg::SleepMS(kLoopPeriodMs);
    }

    // ---- 收尾：异常退出（未到点）也补发停车指令防止车辆失控滑行，
    // 保存最终轨迹图、退订并关闭消息总线。 ----
    if (!goal_hold_complete)
        publish_(control_publisher, buildStop_(control_time_sec, config.gear_mode),
                 "shutdown stop");
    visualizer.saveFinal();
    state_subscriber.Unsubscribe();
    sim_msg::Shutdown();
    std::cout << "Controller stopped; received state=" << (have_state ? "yes" : "no")
              << std::endl;
    return 0;
}
