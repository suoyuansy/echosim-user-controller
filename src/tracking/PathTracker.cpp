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
// 输入/输出均为弧度；用于航向差、方位角等可能绕圈的角量比较。
double normalize_angle_(double angle)
{
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

// 计算车辆平面速度大小。
// 实际车速 = hypot(vx, vy)（米每秒）：车体系平面合成，包含侧向滑动分量，
// 用于纵向误差计算等需要"真实车速"的场景。
double planar_speed_(const VehicleState2D& state)
{
    return std::hypot(state.vx, state.vy);
}

// 计算车体前向速度。vehdynamics 的 vx/vy 为车体系分量（x 前向、y 左正），
// 不是世界系投影，前向速度直接取 vx。
double forward_speed_(const VehicleState2D& state)
{
    return state.vx;
}

// 车体系横向速度（左正）：后轴侧滑检测直接使用，无需坐标旋转。
double lateral_speed_(const VehicleState2D& state)
{
    return state.vy;
}

// 计算带符号横向误差。
// 以参考点切向为基准，把车辆位置投影到路径法向：右偏为正、左偏为负。
// 公式 e = sin(yaw_ref)·(x - x_ref) - cos(yaw_ref)·(y - y_ref)，
// 输入为世界系坐标（米），输出带符号横向误差（米）。
double signed_cross_track_error_(const PathPoint& target,
                             const VehicleState2D& state)
{
    return std::sin(target.yaw) * (state.x - target.x)
        - std::cos(target.yaw) * (state.y - target.y);
}
} // namespace

// 保存跟踪器配置。跟踪器无逐帧内部状态，配置在整个任务期间只读。
PathTracker::PathTracker(TrackingConfig config)
    : config_(std::move(config))
{
}

// 依据配置统一选择跟踪算法。
// 三种算法共用 make_base_command_ 的路径状态基座，差异仅在横向转角与
// 速度修正策略；默认配置为 Pure Pursuit（月壤低抓地下已整体调谐）。
TrackingCommand PathTracker::calculate(const Path& path,
                                       const VehicleState2D& state,
                                       std::size_t progress_index) const
{
    switch (config_.method)
    {
    case TrackerMethod::PurePursuit:
        return calculatePurePursuit(path, state, progress_index);
    case TrackerMethod::Lqr:
        return calculateLqr(path, state, progress_index);
    case TrackerMethod::Stanley:
    default:
        return calculateStanley(path, state, progress_index);
    }
}

// 生成所有横向控制器共用的路径状态。
// 步骤：空路径直接判达 -> 最近点搜索 -> 剩余折线距离 -> 到点判定
// （位置容差 + 可选航向容差）-> 距离刹车限速得到基础目标速度。
// 输出中的 target_speed_mps 仅含距离刹车与 max_speed 钳位，
// 各算法的速度门控在后续叠加。
TrackingCommand PathTracker::make_base_command_(const Path& path,
                                               const VehicleState2D& state,
                                               std::size_t progress_index) const
{
    TrackingCommand command;
    // 空路径视为已完成，调用方收到零值指令直接停车。
    if (path.empty())
    {
        command.reached_goal = true;
        return command;
    }
    // 最近点：进度锚点限窗搜索，防止车身回退时进度卡死或跳变。
    const std::size_t nearest = find_nearest_index_(
        path, state.x, state.y, progress_index);
    command.nearest_path_index = nearest;
    command.target_path_index = nearest;
    command.remaining_path_distance_m = remaining_distance_(path, nearest);
    // 到点判定：用终点直线距离（而非折线长度）对照位置容差。竞赛任务的
    // heading_tolerance=0 表示不约束终点航向，不能把 goal.yaw=0 误解为必须
    // 朝向世界 X 正轴；仅 require_goal_yaw=true 时才追加航向容差。
    const double distance_to_goal = std::hypot(
        path.back().x - state.x, path.back().y - state.y);
    const double yaw_error = normalize_angle_(path.back().yaw - state.yaw);
    command.distance_to_goal_m = distance_to_goal;
    command.reached_goal = distance_to_goal <= config_.goal_position_tolerance_m
        && (!config_.require_goal_yaw
            || std::abs(yaw_error) <= config_.goal_yaw_tolerance_rad);
    // 未到点时计算基础目标速度：先按 v = sqrt(2·a·d) 距离刹车限速，
    // 再与巡航上限取小并钳到 [0, max_speed]。终点区用有效减速度
    // （terminal_brake_decel_mps2，含 SDK 制动滞后），见头文件注释。
    if (!command.reached_goal)
    {
        const double speed_limit = LongitudinalController::distanceSpeedLimit(
            command.remaining_path_distance_m, config_.base_speed_mps,
            config_.terminal_brake_decel_mps2);
        command.target_speed_mps = std::clamp(
            std::min(config_.base_speed_mps, speed_limit), 0.0,
            std::max(0.0, config_.max_speed_mps));
        // 越过终点后的活性保障：车身越过路径末端后最近点即末端、剩余折线
        // 距离归零，距离刹车会把目标速度压死为 0，SDK 距离刹停后 v=0 不再
        // 重启，控制器死寂（Run Test6 20260823_213755 t=48-58 停在终点后
        // 3.3 m 处 10 s 无任何指令响应）。此时维持途中速度地板，让纯跟踪
        // 对身后终点的饱和转角形成回绕圆弧；回到终点前方后距离刹车自然接管。
        if (command.remaining_path_distance_m <= 0.0
            && distance_to_goal > config_.goal_position_tolerance_m)
        {
            command.target_speed_mps = std::max(
                command.target_speed_mps,
                config_.pure_pursuit_min_moving_speed_mps);
        }
    }
    return command;
}

// 搜索距离车辆最近的路径点。
// 从进度点向前全量搜索、向后只开一个限窗：
// 前向独占会在车身大幅回退（起点掉头、绕圈恢复）时把进度卡死。
std::size_t PathTracker::find_nearest_index_(const Path& path, double x,
                                            double y,
                                            std::size_t progress_index) const
{
    if (path.empty())
        return 0;
    constexpr std::size_t kBackwardSearchPoints = 30; // 允许回退的路径点数，2 m 栅格约合 60 m。
    const std::size_t anchor = std::min(progress_index, path.size() - 1);
    const std::size_t first_index = anchor > kBackwardSearchPoints
        ? anchor - kBackwardSearchPoints
        : 0;
    std::size_t nearest = first_index;
    double nearest_distance = std::numeric_limits<double>::infinity();
    // 从窗口起点向路径终点线性扫描，取欧氏距离最小的路径点。
    // 复杂度 O(n)，路径点规模（数百量级）下每帧可接受。
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

// 计算最近路径点到终点的折线长度。
// 逐段累加相邻路径点欧氏距离（米）；用于进度统计与距离刹车限速。
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
// 输入/输出均为弧度；满舵（±23°）时轮胎刮擦阻力极大，正常跟踪不应触界，
// 钳位仅作为几何可达性的最后保护。
double PathTracker::clamp_steer_(double angle_rad) const
{
    return std::clamp(angle_rad,
                      -config_.geometry.max_front_wheel_angle_rad,
                      config_.geometry.max_front_wheel_angle_rad);
}

// 使用前视点和轴距计算纯跟踪转角。
TrackingCommand PathTracker::calculatePurePursuit(
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index);
    // 空路径或已到终点：直接返回基座指令（零转角/零速），不做跟踪计算。
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = command.nearest_path_index;
    // 前视增益使用前向速度，侧偏/打滑时不用速度模长虚增前视距离。
    // 前视距离 Ld = max(0.5, Ld_base + k_gain·vx)：低速时保持最小前视
    // 抑制抖动，速度越高看得越远以提前响应弯道。
    const double lookahead = std::max(
        0.5, config_.pure_pursuit_lookahead_m
            + config_.pure_pursuit_lookahead_gain
                * std::max(0.0, forward_speed_(state)));
    // 沿路径累计弧长，在精确前视距离处对线段插值目标点，
    // 避免目标点按整段栅格跳变导致转角阶跃。
    std::size_t target_index = nearest;
    // 目标点初值取最近点：路径很短（前视超出终点）时直接收敛到终点附近。
    double target_x = path[nearest].x;
    double target_y = path[nearest].y;
    double accumulated = 0.0;
    // 从最近点起逐段累加弧长，直到覆盖前视距离或到达路径末尾。
    while (target_index + 1 < path.size() && accumulated < lookahead)
    {
        const PathPoint& from = path[target_index];
        const PathPoint& to = path[target_index + 1];
        const double segment_length = std::hypot(to.x - from.x, to.y - from.y);
        // 恰好越过前视距离的线段上按比例插值，得到连续移动的前视目标点；
        // 否则整段推进，目标点取下一路径点。
        if (accumulated + segment_length > lookahead && segment_length > 1e-9)
        {
            const double ratio = (lookahead - accumulated) / segment_length;
            target_x = from.x + ratio * (to.x - from.x);
            target_y = from.y + ratio * (to.y - from.y);
        }
        else
        {
            target_x = to.x;
            target_y = to.y;
        }
        accumulated += segment_length;
        ++target_index;
    }
    command.target_path_index = target_index;
    // 纯跟踪几何以后轴为基准点，而非质心位置：后轴是阿克曼几何的
    // 瞬时转向中心约束点，用质心会使曲率计算混入轴距偏置误差。
    // 后轴位置 = 质心沿航向反推 rear_axle_offset_m（世界系，米）。
    const double rear_x = state.x
        - config_.geometry.rear_axle_offset_m * std::cos(state.yaw);
    const double rear_y = state.y
        - config_.geometry.rear_axle_offset_m * std::sin(state.yaw);
    const double dx = target_x - rear_x;
    const double dy = target_y - rear_y;
    // 后轴到前视目标点的距离 Ld，下限 0.5 m 防止目标贴身时公式发散。
    const double target_distance = std::max(std::hypot(dx, dy), 0.5);
    // 可达性动态限幅：目标方位角超过 asin(Ld/(2·R_cap)) 时，目标落在转弯圆
    // 内部，打满转角也只能绕圈。将 α 钳到弧线必达目标的边界；R_cap 取安全
    // 系数×最小转弯半径，限幅生效时转角约 20° 而非满舵——满舵下轮胎刮擦
    // 阻力会吃掉全部驱动力矩，车速跌到 0.2 m/s 以下。
    const double min_turn_radius_m = config_.geometry.wheelbase_m
        / std::tan(config_.geometry.max_front_wheel_angle_rad);
    const double turn_radius_cap_m = std::max(
        config_.pure_pursuit_turn_radius_margin * min_turn_radius_m,
        min_turn_radius_m);
    const double alpha_cap = std::asin(std::min(
        1.0, target_distance / (2.0 * turn_radius_cap_m)));
    // 原始方位角 α：目标点方向与车体航向的夹角（车体系，弧度，逆时针为正）。
    // 限幅后的 α 送入曲率公式；限幅前的 raw_alpha 保留用于速度门控判定。
    const double raw_alpha = normalize_angle_(std::atan2(dy, dx) - state.yaw);
    const double alpha = std::clamp(raw_alpha, -alpha_cap, alpha_cap);
    // 速度门控（逐级取小）：月壤侧向抓地低，实测带误差/带侧滑提速会触发
    // 后轴侧滑甩尾（横摆 50 deg/s、横向分速度 2.9 m/s），甩尾又把车甩离
    // 路径形成循环。横向误差收敛且无侧滑才允许全速；航向对齐由 α 门控
    // （栅格路径逐点 yaw 抖动 ±27°，不能直接用车体 yaw 对比路径点 yaw）。
    const double cross_track_error = signed_cross_track_error_(path[nearest], state);
    double speed_cap = std::max(0.0, config_.max_speed_mps);
    if (std::abs(cross_track_error) > config_.pure_pursuit_converged_cross_track_m
        || std::abs(raw_alpha) > alpha_cap)
    {
        speed_cap = std::min(speed_cap, config_.pure_pursuit_turn_speed_mps);
    }
    // 弯道预限速：α 增大说明前视目标已进入弯道，提前压低车速。
    // 实测阶梯弯入口带速 2.8 m/s 入弯，侧滑门控被动触发时侧偏角已 16°，
    // 冲到 24°（评分上限 8°）才衰减。
    if (std::abs(raw_alpha) > config_.pure_pursuit_bend_alpha_rad)
    {
        speed_cap = std::min(speed_cap, config_.pure_pursuit_bend_speed_mps);
    }
    // 曲率预判减速：高速巡航（物理极速 14.4 m/s）下，单靠前视 α 门控只提前
    // ~19 m 报警，减速距离不够会带速入弯侧滑。扫描前方一段路径，找到第一个
    // 累计航向变化超过阈值的弯，按最大减速度把车速压到弯道限速以内。
    {
        double bend_distance = std::numeric_limits<double>::infinity();
        double arc = 0.0;
        // 以当前前视目标点航向为基准，向后累计弧长扫描 curve_lookahead_m。
        const double base_heading = path[target_index].yaw;
        for (std::size_t j = target_index + 1; j < path.size(); ++j)
        {
            arc += std::hypot(path[j].x - path[j - 1].x,
                              path[j].y - path[j - 1].y);
            // 超出前瞻距离仍未发现弯道：维持当前车速。
            if (arc > config_.curve_lookahead_m)
                break;
            // 累计航向变化超阈值即认定进入弯道，记录距离后停止扫描。
            if (std::abs(normalize_angle_(path[j].yaw - base_heading))
                > config_.curve_heading_threshold_rad)
            {
                bend_distance = arc;
                break;
            }
        }
        // 从当前车速按最大减速度刹车，到达弯道入口时恰好降到弯道限速：
        // v = sqrt(v_bend^2 + 2·a_max·d_bend)。发现弯道（距离小于前瞻窗）才生效。
        if (bend_distance < config_.curve_lookahead_m)
        {
            speed_cap = std::min(speed_cap, std::sqrt(
                config_.pure_pursuit_bend_speed_mps
                    * config_.pure_pursuit_bend_speed_mps
                + 2.0 * config_.max_deceleration_mps2 * bend_distance));
        }
    }
    // 侧滑检测：按后轴侧偏角判定。vehdynamics 的 vy 是质心量测，转弯时含
    // 运动学项 ω×后轴距（实测正常转弯 0.17 m/s 的"横向速度"全是这项，并非
    // 轮胎侧滑），须先扣掉再算真实侧偏角，否则正常转弯会被误判为侧滑压速。
    const double lateral_slip = lateral_speed_(state)
        - state.yaw_rate * config_.geometry.rear_axle_offset_m;
    const double sideslip = std::atan2(std::abs(lateral_slip),
                                       std::max(forward_speed_(state), 0.8));
    // 侧偏角 = |后轴真实侧向滑动速度| / 前向速度（弧度）；前向速度设 0.8 m/s
    // 下限防止低速/停车时比值发散误报。
    // 低速豁免（2026-08-23 坡道失速修复）：前向速度低于途中速度地板时不压速。
    // 实测 Run 20260823_194908 t=455-477：车速从 4 m/s 衰减到 2 m/s 时 vy/vx
    // 越过阈值触发门控，压到地板后 1.5 m/s + 8° 舵角在 2~3.7° 坡道上驱动力
    // 不足，失速倒溜并锁轮倒滑 100 m。爬行速度下 vy/vx 比值本身失真，此时
    // 压速只会杀死爬坡动量；侧偏角评分的真正杀手是失速倒滑（倒退时侧偏角
    // 直接等于 π），保住前进动量才是最优保护。带速（≥ 地板值）侧滑照常压速。
    if (forward_speed_(state) >= config_.pure_pursuit_min_moving_speed_mps
        && sideslip > config_.pure_pursuit_slide_sideslip_rad)
    {
        // 超阈值（默认 5°，评分上限 8° 留余量）判定侧滑，压速到 slide_speed，
        // 等残余滑动衰减后再逐步提速。
        speed_cap = std::min(speed_cap, config_.pure_pursuit_slide_speed_mps);
    }
    // 四级速度门控逐级取小后，叠加到基座目标速度（距离刹车限速）之上。
    command.target_speed_mps = std::min(command.target_speed_mps, speed_cap);
    // 途中最低行驶速度地板：远离终点时目标速度不得低于地板值。门控压速是
    // 保护性的，但低速 + 大舵角会让车辆在坡道上失速倒溜（见头文件注释），
    // 2.0 m/s 动量可带过坡段；终点逼近阶段（距离刹车限速已低于地板值）豁免，
    // 保证收尾刹停语义不变。
    const double distance_brake_limit = LongitudinalController::distanceSpeedLimit(
        command.remaining_path_distance_m, config_.base_speed_mps,
        config_.terminal_brake_decel_mps2);
    if (distance_brake_limit > config_.pure_pursuit_min_moving_speed_mps)
    {
        command.target_speed_mps = std::max(
            command.target_speed_mps, config_.pure_pursuit_min_moving_speed_mps);
    }
    // 纯跟踪核心公式：δ = atan2(2·L·sin(α), Ld)（弧度）。由圆弧几何推出：
    // 以后轴为原点、转过 α 到达距离 Ld 的目标点所需曲率 κ = 2·sin(α)/Ld，
    // 阿克曼几何 δ = atan(κ·L)。α 为限幅后值，输出再经 ±23° 钳位。
    command.front_wheel_angle_rad = clamp_steer_(std::atan2(
        2.0 * config_.geometry.wheelbase_m * std::sin(alpha), target_distance));
    // 终点区航向保持（2026-08-23 终点刹车测试修复）：剩余距离小于前视距离
    // 时前视目标点就是路径终点，而终点是"即将被压线越过"的点，α 指向它
    // 会在车辆通过前瞬间饱和（Run Test6 20260823_213755 t=46：全程居中
    // ±0.1 m，舵角仍跳到 -21°，叠加强制动引发侧滑）。终点仍在车辆前方时
    // 改用航向保持：舵角正比于终点航向差，居中时 ≈ 0，刹车过程车身稳定；
    // 终点落到车身后方（越过终点后的回绕恢复）不覆盖，保留纯跟踪几何--
    // 对身后目标的饱和转角自然形成回绕圆弧。
    const double goal_ahead = (path.back().x - state.x) * std::cos(state.yaw)
        + (path.back().y - state.y) * std::sin(state.yaw) > 0.0;
    if (goal_ahead && target_index + 1 >= path.size()
        && command.remaining_path_distance_m < lookahead)
    {
        // 无终点航向约束时沿最后一段路径切线直行，避免把占位的 goal.yaw=0
        // 当成真实目标航向。Test6 最后一段约 59.4°，旧逻辑在距终点 4 m 内
        // 强行转向 0°，既破坏刹车直线性，也使位置到达后仍无法判定完成。
        const double terminal_yaw = config_.require_goal_yaw || path.size() < 2
            ? path.back().yaw
            : std::atan2(path.back().y - path[path.size() - 2].y,
                         path.back().x - path[path.size() - 2].x);
        command.front_wheel_angle_rad = clamp_steer_(
            config_.terminal_heading_gain * normalize_angle_(
                terminal_yaw - state.yaw));
    }
    return command;
}

// 使用横向误差和航向误差计算 Stanley 转角。
TrackingCommand PathTracker::calculateStanley(
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index);
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = command.nearest_path_index;
    // Stanley 以最近参考点为跟踪基准（不做前视，天然抗路径栅格噪声）。
    const PathPoint& target = path[nearest];
    // 航向误差项（弧度）：车体航向与参考点切向的差，直接消除朝向偏差。
    const double heading_error = normalize_angle_(target.yaw - state.yaw);
    // 横向误差项（米，右偏为正）：Stanley 的位置收敛项。
    const double cross_track_error = signed_cross_track_error_(target, state);
    // 横向项按速度归一：v 越高同样的转角修正力越小，恰好与车辆动力学一致；
    // 速度取平面合成值并设下限，防止静止时 atan2 增益发散。
    const double speed = std::max(config_.stanley_min_speed_mps, planar_speed_(state));
    // Stanley 公式：δ = heading_error + atan2(k·e, v)，
    // 航向项负责快速对准、误差项负责渐进回到路径，最后钳位到 ±23°。
    command.front_wheel_angle_rad = clamp_steer_(heading_error + std::atan2(
        config_.stanley_gain * cross_track_error, speed));
    return command;
}

// 使用 [横向误差、航向误差、速度误差] 计算简化 LQR 输出。
TrackingCommand PathTracker::calculateLqr(
    const Path& path, const VehicleState2D& state,
    std::size_t progress_index) const
{
    TrackingCommand command = make_base_command_(path, state, progress_index);
    if (path.empty() || command.reached_goal)
        return command;
    const std::size_t nearest = command.nearest_path_index;
    // LQR 以最近参考点为基准，误差向量为 [横向误差、航向误差、速度误差]。
    const PathPoint& target = path[nearest];
    // 横向误差（米）与航向误差（弧度），符号约定同 Stanley。
    const double lateral_error = signed_cross_track_error_(target, state);
    const double heading_error = normalize_angle_(target.yaw - state.yaw);
    // 实际速度取平面合成速度（含侧向分量），与基座目标速度求差得到
    // 纵向速度误差（米每秒，偏快为正）。
    const double actual_speed = planar_speed_(state);
    const double speed_error = actual_speed - command.target_speed_mps;
    // 转向输入权重归一（下限 1e-6 防除零），其余权重按与它的比值近似为反馈增益。
    const double steer_weight = std::max(config_.lqr_steer_weight, 1e-6);
    // 横向增益随速度增大而减小（v 下限 1.0 m/s 防低速发散），
    // 模拟 LQR 高速弱横向修正、低速强修正的速度调度特性。
    const double lateral_gain = config_.lqr_lateral_weight / steer_weight
        / std::max(actual_speed, 1.0);
    const double heading_gain = config_.lqr_heading_weight / steer_weight;
    // 简化横向控制律：δ = k_lat·e_lat + k_head·e_yaw（弧度），钳位到 ±23°。
    command.front_wheel_angle_rad = clamp_steer_(
        lateral_gain * lateral_error + heading_gain * heading_error);
    // 纵向简化为速度误差 P 修正：v_cmd -= (w_speed/w_input)·(v - v_cmd)，
    // 输出钳位到 [0, max_speed]，避免修正后超出巡航上限。
    const double input_weight = std::max(config_.lqr_speed_input_weight, 1e-6);
    command.target_speed_mps = std::clamp(
        command.target_speed_mps - config_.lqr_speed_weight / input_weight * speed_error,
        0.0, std::max(0.0, config_.max_speed_mps));
    return command;
}

// 计算带上下限的纵向速度 P 控制加速度。
// 输入：目标/实际速度（米每秒）、P 增益 kp、加速度上下限（米每二次方秒）。
// 先归一上下界顺序（允许任意顺序传入），再输出 a = clamp(kp·(v_t - v_a))。
// 由 EchoSimRuntime 的全部跟踪模式调用，经 TARGET_ACC_CONTROL 下发。
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
// 刹车距离模型：v = sqrt(2·a_max·d)，即从该速度按最大减速度刹停恰好
// 走完剩余距离 d（米）。剩余距离或减速度非正时返回 0（要求立即停），
// 正常输出与 base_speed 取小，不超出巡航上限。
double LongitudinalController::distanceSpeedLimit(
    double remaining_distance_m, double base_speed_mps,
    double max_deceleration_mps2)
{
    if (remaining_distance_m <= 0.0 || max_deceleration_mps2 <= 0.0)
        return 0.0;
    return std::min(std::max(0.0, base_speed_mps), std::sqrt(
        2.0 * max_deceleration_mps2 * remaining_distance_m));
}
